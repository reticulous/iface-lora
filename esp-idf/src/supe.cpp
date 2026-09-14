/**
 * supe — implementation of SUPE's pure core (header carries the design).
 *
 * Host-compilable: <stdint.h>, <string.h> and <math.h>, nothing else. The host
 * test in test/ compiles this file directly.
 */
#include "supe.h"
#include "lora_toa.h"   /* the shared time-on-air formula */

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
    return (uint32_t)daysFromCivil(SUPE_EXPIRY_Y, SUPE_EXPIRY_M, SUPE_EXPIRY_D) * 86400u;
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
/* Receiver noise figure, dB. Deliberately a bare radio's, not this board's:
 * every caller asks this about the FAR end — what power must reach a peer for
 * it to decode — and a peer's front end is not ours to assume. Crediting one
 * with an amplifier it may not have would under-power the link, while assuming
 * it has none only ever spends a dB or two too many, so the conservative
 * reading is the one that stands.
 *
 * A node reasoning about its OWN reception has no use for this: nothing in the
 * tree compares a level of ours against an absolute floor. Margin is an SNR
 * question (peersHeadroom10) and carrier sense tracks its own floor from the
 * channel, so both are already indifferent to what sits in front of us. */
#define SUPE_NOISE_FIGURE_DB  6

int16_t supeReqSnrDeci(uint8_t sf) { return reqSnrDeci((int)sf); }

/* A bandwidth of zero is not a wide channel, it is an unset one — a caller that
 * left the configuration blank. Taken at face value the logarithm runs to
 * negative infinity and the cast lands on 0, which reads as a receiver that
 * needs 0 dBm to hear anything: every real signal then looks hopelessly weak,
 * and a margin test built on it can only ever answer "too little power". Say
 * "unknown" with a floor no measurement will beat instead. */
#define SUPE_SENS_UNKNOWN_DECI  (-1200)   /* -120 dBm: assume generous margin */

int16_t supeSensitivityDeci(const SupeCfg* c) {
    if (!c || c->bwHz == 0) return SUPE_SENS_UNKNOWN_DECI;
    double thermal = -174.0 + 10.0 * log10((double)c->bwHz);
    return (int16_t)lround(10.0 * (thermal + SUPE_NOISE_FIGURE_DB) + (double)reqSnrDeci(c->sf));
}

/* ─────────────── airtime ─────────────── */

double supeAirtimeSeconds(int sf, int bw_hz, int cr_denom, int preamble,
                          int payload, bool implicitHeader, bool crc) {
    return loraToaSeconds(sf, bw_hz, cr_denom, preamble, payload, implicitHeader, crc);
}

/* ─────────────── codec ─────────────── */

static inline uint8_t packNibbles(uint8_t hi, uint8_t lo) {
    return (uint8_t)(((hi & 0x0F) << 4) | (lo & 0x0F));
}

static void encCaps(uint8_t* p, const SupeCaps* c) {
    p[0] = packNibbles(c->fam, c->topStep);
    p[1] = supeEncLevel(c->maxPwrDbm);
}

static void decCaps(const uint8_t* p, SupeCaps* c) {
    c->fam      = (uint8_t)(p[0] >> 4);
    c->topStep  = (uint8_t)(p[0] & 0x0F);
    c->maxPwrDbm = (int8_t)supeDecLevel(p[1]);
}

size_t supeEncAnn(uint8_t* out, size_t cap, const SupeAnn* a) {
    if (a->count == 0 || a->count > SUPE_ANN_MAX) return 0;
    size_t n = SUPE_ANN_BASE + (size_t)a->count * SUPE_ID_LEN;
    if (cap < n) return 0;
    out[0] = SUPE_T_ANNOUNCE;
    out[1] = packNibbles(a->regime, a->version);
    encCaps(out + 2, &a->caps);
    out[4] = supeEncLevel(a->pwrDbm);
    /* Hashes last, so the count needs no byte of its own. */
    for (int i = 0; i < a->count; i++)
        memcpy(out + SUPE_ANN_BASE + i * SUPE_ID_LEN, a->ids[i], SUPE_ID_LEN);
    return n;
}

static bool lenOkAnn(size_t len);

bool supeDecAnn(const uint8_t* f, size_t len, SupeAnn* out) {
    if (len < 2 || f[0] != SUPE_T_ANNOUNCE) return false;
    uint8_t regime  = (uint8_t)(f[1] >> 4);
    uint8_t version = (uint8_t)(f[1] & 0x0F);
    if (!lenOkAnn(len)) return false;
    /* "I do not speak SUPE" is not a dialect, so it is not checked against one:
     * a node renouncing the protocol has no version to agree about, and the
     * identities the frame carries are worth reading either way. */
    if (regime != SUPE_REGIME_NONE) {
        const SupeRegime* g = supeRegime(regime);
        if (!g || g->version != version) return false;
    }
    out->regime  = regime;
    out->version = version;
    decCaps(f + 2, &out->caps);
    out->pwrDbm  = (int8_t)supeDecLevel(f[4]);
    out->count   = (uint8_t)((len - SUPE_ANN_BASE) / SUPE_ID_LEN);
    for (int i = 0; i < out->count; i++)
        memcpy(out->ids[i], f + SUPE_ANN_BASE + i * SUPE_ID_LEN, SUPE_ID_LEN);
    return true;
}

/* ═══════════════ the revised protocol (SUPE.md as specified) ═══════════════ */

/* ─────────────── the ladder, revised (§14.3) ───────────────
 *
 * §14.3 admits no floating-point arithmetic anywhere: two implementations that
 * resolve a budget differently do not fail loudly — they set different
 * spreading factors and the link simply dies. Everything below is integers,
 * and test/supe-rate-table-vectors.txt is the conformance authority. */

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
    if (budget >= SUPE_LADDER_MAX_ENTRIES) return false;
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

/* ─────────────── the frame checksum (§8) ─────────────── */

uint8_t supeCrc8(const uint8_t* d, size_t n) {
    uint8_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (uint8_t)((crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1));
    }
    return crc;
}

/* ─────────────── the sync-word list (§14.5) ───────────────
 *
 * A sync nibble is transmitted as a symbol at bin `nibble × 8` in a space of
 * 2^SF bins. Bin 0 is another preamble upchirp, so zero nibbles detect weakly;
 * frequency error of ~20 ppm at 868 MHz spreads energy about two nibble-steps,
 * so foreign words get a two-nibble berth in both symbols. SF5's 32 bins admit
 * only nibbles 1–3 — every one of its nine words sits inside 0x12's berth, so
 * the berth rule would empty the list and SF5 takes exact exclusions only:
 * the weak separation is spent where no other network listens. */

static bool wordExcluded(uint8_t w, uint8_t foreign, bool exactOnly) {
    if (exactOnly) return w == foreign;
    int w1 = w >> 4, w2 = w & 0x0F;
    int f1 = foreign >> 4, f2 = foreign & 0x0F;
    int d1 = w1 > f1 ? w1 - f1 : f1 - w1;
    int d2 = w2 > f2 ? w2 - f2 : f2 - w2;
    return d1 <= 2 && d2 <= 2;      /* one distant symbol is separation enough */
}

int supeSyncWords(uint8_t sf, uint8_t ifaceSync, uint8_t* out, int cap) {
    int nibMax = sf >= 7 ? 15 : (sf == 6 ? 7 : 3);
    bool exactOnly = sf <= 5;
    const uint8_t foreign[4] = { 0x12, 0x24, 0x34, ifaceSync };
    int n = 0;
    for (int hi = 1; hi <= nibMax; hi++) {
        for (int lo = 1; lo <= nibMax; lo++) {
            uint8_t w = (uint8_t)((hi << 4) | lo);
            bool excl = false;
            for (int i = 0; i < 4 && !excl; i++)
                excl = wordExcluded(w, foreign[i], exactOnly);
            if (excl) continue;
            if (n < cap) out[n] = w;
            n++;
        }
    }
    return n < cap ? n : cap;
}

uint8_t supeSyncWordAt(uint8_t sf, uint8_t ifaceSync, uint8_t sByte) {
    uint8_t words[SUPE_SYNC_WORDS_CAP];
    int n = supeSyncWords(sf, ifaceSync, words, (int)(sizeof words));
    if (n <= 0) return 0x22;            /* unreachable: every SF admits words */
    return words[sByte % (uint8_t)n];
}

/* ─────────────── the schedule (§7) ─────────────── */

void supeDeriveSchedule(const uint8_t d0[32], const uint8_t d1[32],
                        bool wide, uint8_t nChans, SupeSchedD* out) {
    memset(out, 0, sizeof *out);
    memcpy(out->hash3, d0, SUPE_HASH_LEN);
    /* stream = D0[3..31] ‖ D1[0..31]: 61 bytes, three per slot — more than
     * SUPE_SLOTS_MAX can consume. */
    uint8_t stream[61];
    memcpy(stream, d0 + 3, 29);
    memcpy(stream + 29, d1, 32);

    uint32_t horizon = wide ? SUPE_WIDE_HORIZON_MS : SUPE_NARROW_HORIZON_MS;
    uint32_t t = 0;
    uint8_t  n = 0;
    for (uint8_t k = 0; k < SUPE_SLOTS_MAX; k++) {
        uint8_t j = stream[3 * k + 0];
        uint8_t c = stream[3 * k + 1];
        uint8_t s = stream[3 * k + 2];
        if (k == 0) {
            t = wide ? 150u + (j % 40u) : (uint32_t)SUPE_NARROW_T0_MS;
        } else {
            uint32_t base = wide ? (60u + 30u * k) : 100u;
            if (wide && base > 350u) base = 350u;
            t += base + (j % (wide ? 40u : 24u));
        }
        if (t > horizon) break;
        uint8_t chan = nChans ? (uint8_t)(1 + (c % nChans)) : SUPE_CH_HAIL;
        /* A narrow schedule's second slot is never on the first's channel —
         * that is what it is for: a busy channel is answered by a different
         * one (§7). Redrawn one step along, so both ends land identically. */
        if (!wide && n == 1 && nChans > 1 && chan == out->slot[0].chan)
            chan = (uint8_t)(1 + ((c + 1) % nChans));
        out->slot[n].tMs   = (uint16_t)t;
        out->slot[n].chan  = chan;
        out->slot[n].sByte = s;
        n++;
    }
    out->nSlots = n;
}

/* ─────────────── codec, the meeting frames (§0.1) ─────────────── */

static bool lenOkAnn(size_t len) {
    if (len < SUPE_ANN_BASE + SUPE_ID_LEN) return false;
    size_t idBytes = len - SUPE_ANN_BASE;
    if (idBytes % SUPE_ID_LEN) return false;
    return idBytes / SUPE_ID_LEN <= SUPE_ANN_MAX;
}

size_t supeEncHail(uint8_t* out, size_t cap, const SupeHail* h) {
    size_t n = h->haveIdent ? SUPE_HAIL_ID_LEN : SUPE_HAIL_LEN;
    if (cap < n || h->count > SUPE_TRAIN_MAX) return 0;
    out[0] = SUPE_T_HAIL;
    out[1] = packNibbles(h->regime, h->version);
    memcpy(out + 2, h->tag, SUPE_TAG_LEN);
    out[5] = supeEncLevel(h->pwrDbm);
    out[6] = h->salt;
    out[7] = h->budgetCeil;
    out[8] = h->count;
    out[9] = h->lenByte;
    /* The identity last: presence is implicit in the frame length, and a
     * hail-back tags the identity the hail it answers carried here. */
    if (h->haveIdent) memcpy(out + SUPE_HAIL_LEN, h->ident, SUPE_TAG_LEN);
    return n;
}

bool supeDecHail(const uint8_t* f, size_t len, SupeHail* out) {
    if (len < 2 || f[0] != SUPE_T_HAIL) return false;
    if (len != SUPE_HAIL_LEN && len != SUPE_HAIL_ID_LEN) return false;
    uint8_t regime  = (uint8_t)(f[1] >> 4);
    uint8_t version = (uint8_t)(f[1] & 0x0F);
    const SupeRegime* g = supeRegime(regime);
    if (!g || g->version != version) return false;
    if (f[8] > SUPE_TRAIN_MAX) return false;
    out->regime  = regime;
    out->version = version;
    memcpy(out->tag, f + 2, SUPE_TAG_LEN);
    out->pwrDbm     = (int8_t)supeDecLevel(f[5]);
    out->salt       = f[6];
    out->budgetCeil = f[7];
    out->count      = f[8];
    out->lenByte    = f[9];
    /* Presence of the sender's identity is implicit in the frame length; a
     * node with `sender_ident` off still parses and honours the long form. */
    out->haveIdent = (len == SUPE_HAIL_ID_LEN);
    if (out->haveIdent) memcpy(out->ident, f + SUPE_HAIL_LEN, SUPE_TAG_LEN);
    else                memset(out->ident, 0, SUPE_TAG_LEN);
    return true;
}

/* READY's nine bytes are GOT's first nine: one layout, written once. */
static void encReadyBody(uint8_t* out, uint8_t type, const SupeReady* g) {
    out[0] = type;
    memcpy(out + 1, g->hash, SUPE_HASH_LEN);
    out[4] = supeEncLevel(g->pwrDbm);
    out[5] = g->budget;
    out[6] = g->countCeil;
    out[7] = supeEncLevel(g->heardRssi);
    out[8] = (uint8_t)g->heardSnrQ;
}

static void decReadyBody(const uint8_t* f, SupeReady* g) {
    memcpy(g->hash, f + 1, SUPE_HASH_LEN);
    g->pwrDbm    = (int8_t)supeDecLevel(f[4]);
    g->budget    = f[5];
    g->countCeil = f[6];
    g->heardRssi = supeDecLevel(f[7]);
    g->heardSnrQ = (int8_t)f[8];
}

size_t supeEncReady(uint8_t* out, size_t cap, const SupeReady* g) {
    if (cap < SUPE_READY_LEN) return 0;
    encReadyBody(out, SUPE_T_READY, g);
    return SUPE_READY_LEN;
}

bool supeDecReady(const uint8_t* f, size_t len, SupeReady* out) {
    if (len != SUPE_READY_LEN || f[0] != SUPE_T_READY) return false;
    decReadyBody(f, out);
    return true;
}

size_t supeEncGot(uint8_t* out, size_t cap, const SupeGot* h) {
    size_t n = h->answering ? (size_t)SUPE_GOT_ANS_BASE + h->maskLen
                            : (size_t)SUPE_GOT_LEN;
    if (cap < n || h->count > SUPE_TRAIN_MAX) return 0;
    if (h->answering && (h->maskLen == 0 || h->maskLen > SUPE_MASK_MAX)) return 0;
    encReadyBody(out, SUPE_T_GOT, &h->g);
    out[9]  = h->count;
    out[10] = h->lenByte;
    if (h->answering) memcpy(out + SUPE_GOT_ANS_BASE, h->mask, h->maskLen);
    return n;
}

bool supeDecGot(const uint8_t* f, size_t len, uint8_t peerCount, SupeGot* out) {
    if (len < 1 || f[0] != SUPE_T_GOT) return false;
    size_t ansLen = (size_t)SUPE_GOT_ANS_BASE + supeMaskLen(peerCount);
    bool answering;
    if (len == SUPE_GOT_LEN) answering = false;
    else if (peerCount > 0 && len == ansLen) answering = true;
    else return false;
    if (f[9] > SUPE_TRAIN_MAX) return false;
    decReadyBody(f, &out->g);
    out->count     = f[9];
    out->lenByte   = f[10];
    out->answering = answering;
    memset(out->mask, 0, sizeof out->mask);
    out->maskLen = 0;
    if (answering) {
        out->maskLen = supeMaskLen(peerCount);
        memcpy(out->mask, f + SUPE_GOT_ANS_BASE, out->maskLen);
    }
    return true;
}

size_t supeEncEnd(uint8_t* out, size_t cap, const SupeEnd* t) {
    if (t->count == 0 || t->count > SUPE_TRAIN_MAX) return 0;
    size_t n = (size_t)SUPE_END_BASE + t->count;
    if (cap < n) return 0;
    out[0] = SUPE_T_END;
    out[1] = supeEncLevel(t->pwrDbm);
    out[2] = t->salt;
    out[3] = supeEncLevel(t->haveHeard ? t->heardRssi : SUPE_LEVEL_NONE);
    out[4] = (uint8_t)(t->haveHeard ? t->heardSnrQ : 0);
    memcpy(out + SUPE_END_BASE, t->csum, t->count);
    return n;
}

bool supeDecEnd(const uint8_t* f, size_t len, SupeEnd* out) {
    if (len < SUPE_END_BASE + 1 || f[0] != SUPE_T_END) return false;
    size_t count = len - SUPE_END_BASE;
    if (count > SUPE_TRAIN_MAX) return false;
    out->pwrDbm    = (int8_t)supeDecLevel(f[1]);
    out->salt      = f[2];
    out->heardRssi = supeDecLevel(f[3]);
    out->heardSnrQ = (int8_t)f[4];
    out->haveHeard = out->heardRssi != SUPE_LEVEL_NONE;
    out->count     = (uint8_t)count;
    memcpy(out->csum, f + SUPE_END_BASE, count);
    return true;
}

size_t supeEncResend(uint8_t* out, size_t cap, const SupeResendF* m) {
    if (m->maskLen == 0 || m->maskLen > SUPE_MASK_MAX) return 0;
    size_t n = (size_t)SUPE_RESEND_BASE + m->maskLen;
    if (cap < n) return 0;
    out[0] = SUPE_T_RESEND;
    memcpy(out + 1, m->mask, m->maskLen);
    return n;
}

bool supeDecResend(const uint8_t* f, size_t len, uint8_t peerCount,
                   SupeResendF* out) {
    if (len < 1 || f[0] != SUPE_T_RESEND || peerCount == 0) return false;
    uint8_t ml = supeMaskLen(peerCount);
    if (len != (size_t)SUPE_RESEND_BASE + ml) return false;
    out->maskLen = ml;
    memset(out->mask, 0, sizeof out->mask);
    memcpy(out->mask, f + 1, ml);
    return true;
}
