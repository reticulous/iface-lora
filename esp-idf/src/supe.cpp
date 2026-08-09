/**
 * supe — implementation of SUPE's pure core (header carries the design).
 *
 * Host-compilable: <stdint.h>, <string.h> and <math.h>, nothing else. The host
 * test in test/ compiles this file directly.
 */
#include "supe.h"

#include <string.h>
#include <math.h>

/* ─────────────── regime tables ─────────────── */

/* Regime 1 — ETSI EN 300 220 (863–870 MHz). Nine uniform channels: 500 kHz,
 * 25 mW e.r.p., all on adaptive spectrum access, none crossing a band boundary,
 * and at least 200 kHz of clear spectrum between any two edges — which is what
 * keeps each channel's airtime budget independent of its neighbours'.
 *
 * Channel 9 fills band N edge to edge between two alarm allocations. If
 * out-of-band emission performance does not support 500 kHz there, the fallback
 * is 250 kHz at the same centre, which costs peak rate on that channel and no
 * airtime at all (plans/SUPE.md §16). */
static const SupeChan kEu863Chans[] = {
    { 863350000, 500000 },
    { 864050000, 500000 },
    { 864750000, 500000 },
    { 865450000, 500000 },
    { 866150000, 500000 },
    { 866850000, 500000 },
    { 867550000, 500000 },
    { 868250000, 500000 },
    { 868950000, 500000 },
};

static const SupeRegime kRegimes[] = {
    /* Regime 0 — Single Channel. One frequency, one bandwidth, the spreading
     * factor the only thing that moves. It needs no channel plan and therefore
     * no regulatory band plan, which is what makes it the regime a network can
     * run anywhere — and why it states no ceilings of its own: it runs on the
     * hailing channel, whose limits belong to whatever regime that network is
     * operating under, and which SUPE does not own. What still bounds a detour
     * is the field widths, not a figure invented here. */
    { SUPE_REGIME_SINGLE, SUPE_VERSION, "Single Channel",
      nullptr, 0, /*hailBwOnly=*/true,
      /*trainCeilMs=*/0, /*txnCeilMs=*/0,
      /*airtimeMaxMs=*/0, /*airtimeWinMs=*/0, /*reuseGapMs=*/0,
      /*ccaDbm125=*/0, /*ccaDbm500=*/0,
      /*ccaListenUs=*/0, /*ccaDeferUs=*/0, /*ccaDeadMs=*/0,
      /*maxTxpDbm=*/SUPE_TXP_IFACE },

    /* Regime 1 — ETSI EN 300 220-2 V3.2.1 annex B table B.1 for the bands and
     * their duty cycles, EN 300 220-1 V3.1.1 clause 5.21 and tables 45/48 for
     * adaptive spectrum access, which every 863–870 MHz entry carrying a duty
     * cycle permits in place of that duty cycle. The regulation fixes these
     * constants directly; they are not free parameters. The power figure is
     * 25 mW *effective radiated power* — not e.i.r.p. and not conducted power
     * at the connector; the three differ by antenna gain and by 2.15 dB, and a
     * number recorded without saying which is being measured is a compliance
     * failure no functional test will catch. */
    { SUPE_REGIME_EU863, SUPE_VERSION, "EU 863-870",
      kEu863Chans, (uint8_t)(sizeof kEu863Chans / sizeof kEu863Chans[0]),
      /*hailBwOnly=*/false,
      /*trainCeilMs=*/1000,      /* Ton_max, single transmission */
      /*txnCeilMs=*/4000,        /* Ton_max, dialogue or polling sequence */
      /*airtimeMaxMs=*/100000,   /* max Tcum_on, per 200 kHz of spectrum … */
      /*airtimeWinMs=*/3600000,  /* … in any window of this length */
      /*reuseGapMs=*/100,        /* Toff_min, same operating frequency */
      /*ccaDbm125=*/-81, /*ccaDbm500=*/-75,   /* table 45, referenced to 0 dBd */
      /*ccaListenUs=*/160, /*ccaDeferUs=*/160, /*ccaDeadMs=*/5,
      /*maxTxpDbm=*/14 },
};
static const int kNumRegimes = (int)(sizeof kRegimes / sizeof kRegimes[0]);

const SupeRegime* supeRegime(uint8_t regime) {
    for (int i = 0; i < kNumRegimes; i++)
        if (kRegimes[i].regime == regime) return &kRegimes[i];
    return nullptr;
}

const SupeChan* supeRegimeChans(uint8_t regime, int* count) {
    const SupeRegime* g = supeRegime(regime);
    if (!g || !g->chans) { if (count) *count = 0; return nullptr; }
    if (count) *count = g->nChans;
    return g->chans;
}

/* ─────────────── expiry ─────────────── */

/* __DATE__ is "Mmm dd yyyy" with a space-padded day. The app descriptor carries
 * the same timestamp; taking it from the compiler keeps this file free of
 * ESP-IDF and so keeps it testable on a host. */
static int monthFromDate(const char* d) {
    static const char* kM = "JanFebMarAprMayJunJulAugSepOctNovDec";
    for (int i = 0; i < 12; i++)
        if (strncmp(d, kM + 3 * i, 3) == 0) return i + 1;
    return 1;
}

/* Days from 1970-01-01 to y-m-d, Howard Hinnant's civil-days algorithm. */
static int32_t daysFromCivil(int y, int m, int d) {
    y -= m <= 2;
    int32_t era = (y >= 0 ? y : y - 399) / 400;
    uint32_t yoe = (uint32_t)(y - era * 400);
    uint32_t doy = (uint32_t)((153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1);
    uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int32_t)doe - 719468;
}

uint32_t supeBuildUnix(void) {
    const char* d = __DATE__;
    const char* t = __TIME__;
    int mon  = monthFromDate(d);
    int day  = (d[4] == ' ' ? 0 : (d[4] - '0') * 10) + (d[5] - '0');
    int year = (d[7] - '0') * 1000 + (d[8] - '0') * 100 + (d[9] - '0') * 10 + (d[10] - '0');
    int hh = (t[0] - '0') * 10 + (t[1] - '0');
    int mm = (t[3] - '0') * 10 + (t[4] - '0');
    int ss = (t[6] - '0') * 10 + (t[7] - '0');
    int32_t days = daysFromCivil(year, mon, day);
    return (uint32_t)(days * 86400 + hh * 3600 + mm * 60 + ss);
}

uint32_t supeExpiryUnix(void) {
    return supeBuildUnix() + (uint32_t)SUPE_EXPIRY_DAYS * 86400u;
}

bool supeExpired(uint32_t nowUnix) {
    /* An unresolved or plainly wrong clock must not silently disable the
     * protocol — a node that cannot tell the time has a larger problem than a
     * stale dialect, and reading "before the build" as expired would take SUPE
     * off the air on every boot before the clock lands. */
    if (nowUnix < supeBuildUnix()) return false;
    return nowUnix >= supeExpiryUnix();
}

/* ─────────────── the ladder ─────────────── */

/* The demodulator's required signal-to-noise, deci-dB, 2.5 dB per spreading
 * factor from Semtech's figures: SF5 −2.5 through SF12 −20.0. */
static inline int16_t reqSnrDeci(int sf) { return (int16_t)(-25 * (sf - 4)); }

/* Thermal noise relative to 125 kHz, deci-dB: 10·log₁₀(BW / 125 kHz). The
 * ladder's margin column is written against SF7/BW125, so that is the
 * reference every entry is measured from. */
static int16_t bwNoiseDeci(uint32_t bwHz) {
    if (bwHz == 125000) return 0;
    return (int16_t)lround(100.0 * log10((double)bwHz / 125000.0));
}

/* The bandwidths a regime admits beyond the hailing one. Regime 1's channels
 * are 500 kHz wide, so every LoRa bandwidth up to that fits inside one. */
static const uint32_t kBandwidths[] = { 125000, 250000, 500000 };
static const int kNumBandwidths = (int)(sizeof kBandwidths / sizeof kBandwidths[0]);

/* ─────────────── step choice ─────────────── */

/* Receiver noise figure. A middling figure for the parts this drives, and it
 * cancels out of every *difference* the ladder works in — it only sets where
 * the absolute floor sits. */
#define SUPE_NOISE_FIGURE_DB  6

int16_t supeReqSnrDeci(uint8_t sf) { return reqSnrDeci((int)sf); }

int16_t supeSensitivityDeci(const SupeCfg* c) {
    double thermal = -174.0 + 10.0 * log10((double)c->bwHz);
    return (int16_t)lround(10.0 * (thermal + SUPE_NOISE_FIGURE_DB) + (double)reqSnrDeci(c->sf));
}

/* ─────────────── airtime ─────────────── */

double supeAirtimeSeconds(int sf, int bw_hz, int cr_denom, int preamble,
                          int payload, bool implicitHeader, bool crc) {
    if (sf <= 0 || bw_hz <= 0 || cr_denom < 5 || cr_denom > 8) return 0.0;
    double tSym = (double)((uint32_t)1 << sf) / (double)bw_hz;
    int    de   = (tSym > 0.016) ? 1 : 0;
    int    cr   = cr_denom - 4;
    double num  = 8.0 * payload - 4.0 * sf + 28.0
                  + (crc ? 16.0 : 0.0)
                  - (implicitHeader ? 20.0 : 0.0);
    double den  = 4.0 * (sf - 2 * de);
    double payloadSym = 8.0 + fmax(ceil(num / den) * (cr + 4), 0.0);
    return (preamble + 4.25) * tSym + payloadSym * tSym;
}

/* ─────────────── deadlines ─────────────── */

static uint32_t toaMs(const SupeCfg* c, int crDenom, int preamble, int payload) {
    return (uint32_t)lround(1000.0 * supeAirtimeSeconds(
               c->sf, (int)c->bwHz, crDenom, preamble, payload,
               /*implicitHeader=*/false, /*crc=*/false));
}

/* ─────────────── codec ─────────────── */

static inline uint8_t packNibbles(uint8_t hi, uint8_t lo) {
    return (uint8_t)(((hi & 0x0F) << 4) | (lo & 0x0F));
}

static void encCaps(uint8_t* p, const SupeCaps* c) {
    p[0] = packNibbles(c->fam, c->topStep);
    p[1] = (uint8_t)(supeEncLevel(c->maxPwrDbm) | (c->adaptive ? 0x80 : 0x00));
}

static void decCaps(const uint8_t* p, SupeCaps* c) {
    c->fam      = (uint8_t)(p[0] >> 4);
    c->topStep  = (uint8_t)(p[0] & 0x0F);
    c->adaptive = (p[1] & 0x80) != 0;
    c->maxPwrDbm = (int8_t)supeDecLevel((uint8_t)(p[1] & 0x7F));
}

size_t supeEncAnn2(uint8_t* out, size_t cap, const SupeAnn2* a) {
    if (a->count == 0 || a->count > SUPE_ANN2_MAX) return 0;
    size_t n = SUPE_ANN2_BASE + (size_t)a->count * SUPE_ID_LEN;
    if (cap < n) return 0;
    out[0] = SUPE_T_ANNOUNCE2;
    out[1] = packNibbles(a->regime, a->version);
    encCaps(out + 2, &a->caps);
    out[4] = supeEncLevel(a->pwrDbm);
    /* Hashes last, so the count needs no byte of its own. */
    for (int i = 0; i < a->count; i++)
        memcpy(out + SUPE_ANN2_BASE + i * SUPE_ID_LEN, a->ids[i], SUPE_ID_LEN);
    return n;
}

bool supeDecAnn2(const uint8_t* f, size_t len, SupeAnn2* out) {
    if (len < 2 || f[0] != SUPE_T_ANNOUNCE2) return false;
    uint8_t regime  = (uint8_t)(f[1] >> 4);
    uint8_t version = (uint8_t)(f[1] & 0x0F);
    if (!supeLenOk2(SUPE_T_ANNOUNCE2, regime, version, len)) return false;
    out->regime  = regime;
    out->version = version;
    decCaps(f + 2, &out->caps);
    out->pwrDbm  = (int8_t)supeDecLevel(f[4]);
    out->count   = (uint8_t)((len - SUPE_ANN2_BASE) / SUPE_ID_LEN);
    for (int i = 0; i < out->count; i++)
        memcpy(out->ids[i], f + SUPE_ANN2_BASE + i * SUPE_ID_LEN, SUPE_ID_LEN);
    return true;
}

/* ═══════════════ the revised protocol (SUPE.md as specified) ═══════════════ */

/* ─────────────── the ladder, revised (§14.3) ───────────────
 *
 * §14.3 admits no floating-point arithmetic anywhere: two implementations that
 * resolve a budget differently do not fail loudly — they set different
 * spreading factors and the link simply dies. Everything below is integers,
 * and test/supe-ladder-vectors.txt is the conformance authority. */

/* §14.6: which spreading factors a family reaches. */
static bool famReachesSf(uint8_t fam, int sf) {
    if (sf < 5 || sf > 12) return false;
    if (fam == SUPE_FAM_SX127X && sf < 6) return false;   /* no SF5 at all */
    return true;
}

/* §14.6: SF6 demands an implicit header on SX127x, and no framing in this
 * protocol supplies the fixed length that would need. */
static bool famSf6Explicit(uint8_t fam) { return fam != SUPE_FAM_SX127X; }

/* The ordering key, exactly as §14.3.2 states it: (bw × sf) >> sf, unsigned,
 * ascending. The products fit 32 bits for every bandwidth this protocol
 * permits. */
static inline uint32_t ladKey(int sf, uint32_t bwHz) {
    return (bwHz * (uint32_t)sf) >> sf;
}

/* Low-data-rate optimisation, integer form of "symbol longer than 16 ms":
 * 2^SF / BW > 16 ms  ⇔  1000·2^SF > 16·BW. */
static inline bool ldroForInt(int sf, uint32_t bwHz) {
    return (1000u << sf) > 16u * bwHz;
}

/* Margin cost in deci-dB against the hailing configuration: the demodulator's
 * required SNR per spreading factor (2.5 dB a step) plus the thermal-noise
 * cost of the wider bandwidth. Integer lookup for the permitted widths; the
 * hailing entry itself is the reference and costs zero by construction. */
static int16_t bwNoiseDeciInt(uint32_t bwHz) {
    switch (bwHz) {
        case 125000: return 0;
        case 250000: return 30;    /* 10·log10(2) ≈ 3.0 dB */
        case 500000: return 60;
        default:     return bwNoiseDeci(bwHz);   /* an odd hailing width */
    }
}

int supeLadder(uint8_t regime, uint8_t version,
               uint8_t hailSf, uint32_t hailBwHz, uint32_t chanMaxBwHz,
               uint8_t famA, uint8_t famB,
               SupeLadderEntry* out, int cap) {
    const SupeRegime* g = supeRegime(regime);
    if (!g || g->version != version) return 0;
    if (hailSf < 5 || hailSf > 12 || hailBwHz == 0 || cap <= 0) return 0;

    struct E { uint8_t sf; uint32_t bw; uint32_t key; };
    E es[1 + 8 * 3];
    int n = 0;

    /* The hailing entry is always in the ladder and is always index 0 —
     * §14.3.1 states it as membership by construction, and the ordering below
     * puts it first because every other entry has sf ≤ hail_sf and
     * bw ≥ hail_bw, both of which raise the key. */
    es[n++] = { hailSf, hailBwHz, ladKey(hailSf, hailBwHz) };

    for (int bi = 0; bi < kNumBandwidths; bi++) {
        uint32_t bw = kBandwidths[bi];
        if (bw < hailBwHz || bw > chanMaxBwHz) continue;
        if (g->hailBwOnly && bw != hailBwHz) continue;
        for (int sf = 5; sf <= (int)hailSf; sf++) {
            if (sf == (int)hailSf && bw == hailBwHz) continue;   /* already in */
            if (sf < 7 && (!famReachesSf(famA, sf) || !famReachesSf(famB, sf)))
                continue;
            if (sf == 6 && (!famSf6Explicit(famA) || !famSf6Explicit(famB)))
                continue;
            es[n++] = { (uint8_t)sf, bw, ladKey(sf, bw) };
        }
    }

    /* Insertion sort by (key asc, bw asc, sf desc) — §14.3.2's tie-break:
     * toward the narrower bandwidth first, then the higher spreading factor.
     * At equal rate, take the entry with more margin. */
    for (int i = 1; i < n; i++) {
        E e = es[i];
        int j = i - 1;
        while (j >= 0 && (es[j].key > e.key ||
                          (es[j].key == e.key &&
                           (es[j].bw > e.bw ||
                            (es[j].bw == e.bw && es[j].sf < e.sf))))) {
            es[j + 1] = es[j];
            j--;
        }
        es[j + 1] = e;
    }

    if (n > SUPE_LADDER_MAX_ENTRIES) n = SUPE_LADDER_MAX_ENTRIES;
    if (n > cap) n = cap;
    int16_t hailMargin = (int16_t)((reqSnrDeci(hailSf) - reqSnrDeci(7))
                                   + bwNoiseDeciInt(hailBwHz));
    for (int i = 0; i < n; i++) {
        out[i].sf         = es[i].sf;
        out[i].bwHz       = es[i].bw;
        out[i].ldro       = ldroForInt(es[i].sf, es[i].bw);
        out[i].marginDeci = (int16_t)((reqSnrDeci(es[i].sf) - reqSnrDeci(7))
                                      + bwNoiseDeciInt(es[i].bw) - hailMargin);
    }
    return n;
}

bool supeResolveBudget(uint8_t regime, uint8_t version,
                       uint8_t hailSf, uint32_t hailBwHz, uint32_t chanMaxBwHz,
                       uint8_t famA, uint8_t famB, uint8_t budget, SupeCfg* out) {
    if (budget >= SUPE_BUDGET_REFUSED) return false;
    SupeLadderEntry lad[SUPE_LADDER_MAX_ENTRIES];
    int n = supeLadder(regime, version, hailSf, hailBwHz, chanMaxBwHz,
                       famA, famB, lad, SUPE_LADDER_MAX_ENTRIES);
    if ((int)budget >= n) return false;
    out->sf         = lad[budget].sf;
    out->bwHz       = lad[budget].bwHz;
    out->ldro       = lad[budget].ldro;
    out->marginDeci = lad[budget].marginDeci;
    return true;
}

uint8_t supeSyncWordFor(uint8_t regime, const SupeCfg* c, uint8_t budget,
                        uint8_t ifaceSync) {
    /* Regime 0's budget 0 is the hailing configuration on the hailing channel
     * — not off the channel in any sense the sync word cares about. Every
     * other grant is a detour, even at budget 0 under regime 1, where the
     * frequency moved though the modulation did not. */
    if (regime == SUPE_REGIME_SINGLE && budget == 0) return ifaceSync;
    return c->sf == 5 ? SUPE_SYNC_SF5 : SUPE_SYNC_UNICAST;
}

/* ─────────────── the load's airtime (§6) ─────────────── */

uint32_t supeLoadAirtimeMs(uint8_t loadUnits, const SupeCfg* c, int crDenom) {
    if (loadUnits == 0 || crDenom < 5 || crDenom > 8) return 0;
    uint32_t bytes = supeDecLoadBytes(loadUnits);
    int de = c->ldro ? 1 : 0;
    /* Data bits per symbol are (SF − 2·DE)·4/cr_denom, so
     * ms = bytes·8 / that · tSym = bytes·2·cr·2^SF·1000 / ((SF−2·DE)·BW),
     * rounded up. 64-bit: 8160·2·8·4096·1000 ≈ 2^39. */
    uint64_t num = (uint64_t)bytes * 2u * (uint32_t)crDenom
                   * ((uint64_t)1 << c->sf) * 1000u;
    uint64_t den = (uint64_t)(c->sf - 2 * de) * c->bwHz;
    return (uint32_t)((num + den - 1) / den);
}

/* ─────────────── deadlines, revised (§14.7) ─────────────── */

uint32_t supeGrantDeadlineMs(uint8_t hailSf, uint32_t hailBwHz,
                             int crDenom, int preamble) {
    SupeCfg hail = { hailSf, hailBwHz, ldroForInt(hailSf, hailBwHz), 0 };
    return SUPE_TURNAROUND_MS + toaMs(&hail, crDenom, preamble, SUPE_GRANT_LEN)
           + SUPE_GUARD_MS;
}

uint32_t supeManifestFirstDeadlineMs(const SupeCfg* c, int crDenom, int preamble) {
    return SUPE_RETUNE_GAP_MS + SUPE_TURNAROUND_MS
           + toaMs(c, crDenom, preamble, SUPE_MANIFEST2_LEN) + SUPE_GUARD_MS;
}

uint32_t supeManifestReverseDeadlineMs(const SupeCfg* c, int crDenom, int preamble) {
    return SUPE_TURNAROUND_MS
           + toaMs(c, crDenom, preamble, SUPE_MANIFEST2_LEN) + SUPE_GUARD_MS;
}

/* ─────────────── revised codec (§0.1) ─────────────── */

bool supeLenOk2(uint8_t type, uint8_t regime, uint8_t version, size_t len) {
    const SupeRegime* g = supeRegime(regime);
    if (!g || g->version != version) return false;
    switch (type) {
        case SUPE_T_START:
            return len == SUPE_START2_LEN || len == SUPE_START2_ID_LEN;
        case SUPE_T_GRANT:
            return len == SUPE_GRANT_LEN;
        case SUPE_T_ANNOUNCE2: {
            if (len < SUPE_ANN2_BASE + SUPE_ID_LEN) return false;
            size_t idBytes = len - SUPE_ANN2_BASE;
            if (idBytes % SUPE_ID_LEN) return false;
            return idBytes / SUPE_ID_LEN <= SUPE_ANN2_MAX;
        }
        case SUPE_T_MANIFEST:
            return len == SUPE_MANIFEST2_LEN;
        default:
            return false;   /* reserved — 0xC3 and 0xC8 included — is discarded
                             * exactly as a wrong length is */
    }
}

size_t supeEncStart2(uint8_t* out, size_t cap, const SupeStart2* s) {
    size_t n = s->haveIdent ? SUPE_START2_ID_LEN : SUPE_START2_LEN;
    if (cap < n) return 0;
    out[0] = SUPE_T_START;
    out[1] = packNibbles(s->regime, s->version);
    memcpy(out + 2, s->tag, SUPE_TAG_LEN);
    out[5] = packNibbles(s->fam, s->ceiling);
    out[6] = s->load;
    if (s->haveIdent) memcpy(out + 7, s->ident, SUPE_TAG_LEN);
    return n;
}

bool supeDecStart2(const uint8_t* f, size_t len, SupeStart2* out) {
    if (len < 2 || f[0] != SUPE_T_START) return false;
    uint8_t regime  = (uint8_t)(f[1] >> 4);
    uint8_t version = (uint8_t)(f[1] & 0x0F);
    if (!supeLenOk2(SUPE_T_START, regime, version, len)) return false;
    out->regime  = regime;
    out->version = version;
    memcpy(out->tag, f + 2, SUPE_TAG_LEN);
    out->fam     = (uint8_t)(f[5] >> 4);
    out->ceiling = (uint8_t)(f[5] & 0x0F);
    out->load    = f[6];
    /* Presence of the sender's identity is implicit in the frame length. The
     * ten-byte form is parsed and honoured while never being sent —
     * `sender_ident` ships with the transmitting form and not before (§4). */
    out->haveIdent = (len == SUPE_START2_ID_LEN);
    if (out->haveIdent) memcpy(out->ident, f + 7, SUPE_TAG_LEN);
    else                memset(out->ident, 0, SUPE_TAG_LEN);
    return true;
}

size_t supeEncGrant(uint8_t* out, size_t cap, const SupeGrant* g) {
    if (cap < SUPE_GRANT_LEN) return 0;
    out[0] = SUPE_T_GRANT;
    out[1] = packNibbles(g->regime, g->version);
    out[2] = packNibbles(g->chan, g->budget);
    out[3] = g->durByte;
    out[4] = (uint8_t)(supeEncLevel(g->pwrDbm) | (g->reverse ? 0x80 : 0x00));
    out[5] = supeEncLevel(g->rssiDbm);
    out[6] = (uint8_t)g->snrQ;
    memcpy(out + 7, g->hash, SUPE_HASH_LEN);
    return SUPE_GRANT_LEN;
}

bool supeDecGrant(const uint8_t* f, size_t len, SupeGrant* out) {
    if (len < 2 || f[0] != SUPE_T_GRANT) return false;
    uint8_t regime  = (uint8_t)(f[1] >> 4);
    uint8_t version = (uint8_t)(f[1] & 0x0F);
    if (!supeLenOk2(SUPE_T_GRANT, regime, version, len)) return false;
    out->regime  = regime;
    out->version = version;
    out->chan    = (uint8_t)(f[2] >> 4);
    out->budget  = (uint8_t)(f[2] & 0x0F);
    out->durByte = f[3];
    out->reverse = (f[4] & 0x80) != 0;
    out->pwrDbm  = (int8_t)supeDecLevel((uint8_t)(f[4] & 0x7F));
    out->rssiDbm = supeDecLevel(f[5]);
    out->snrQ    = (int8_t)f[6];
    memcpy(out->hash, f + 7, SUPE_HASH_LEN);
    return true;
}

size_t supeEncManifest2(uint8_t* out, size_t cap, const SupeManifest2* m) {
    if (cap < SUPE_MANIFEST2_LEN) return 0;
    out[0] = SUPE_T_MANIFEST;
    out[1] = supeEncLevel(m->pwrDbm);
    out[2] = supeEncLevel(m->rssiDbm);
    out[3] = (uint8_t)m->snrQ;
    encCaps(out + 4, &m->caps);
    out[6] = m->count;
    out[7] = m->lenByte;
    memcpy(out + 8, m->hash, SUPE_HASH_LEN);
    return SUPE_MANIFEST2_LEN;
}

bool supeDecManifest2(const uint8_t* f, size_t len, SupeManifest2* out) {
    /* MANIFEST carries no regime nibble: the GRANT that opened the detour
     * fixed the dialect for both sides, including the detour's own regime. */
    if (len != SUPE_MANIFEST2_LEN || f[0] != SUPE_T_MANIFEST) return false;
    out->pwrDbm  = (int8_t)supeDecLevel(f[1]);
    out->rssiDbm = supeDecLevel(f[2]);
    out->snrQ    = (int8_t)f[3];
    decCaps(f + 4, &out->caps);
    out->count   = f[6];
    out->lenByte = f[7];
    memcpy(out->hash, f + 8, SUPE_HASH_LEN);
    return true;
}
