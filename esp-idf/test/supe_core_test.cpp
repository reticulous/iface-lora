/**
 * supe_core_test — host-side tests for SUPE's pure core, and the producer of
 * the golden frame vectors.
 *
 *   make -C iface-lora/esp-idf/test           # build, run, regenerate golden.txt
 *
 * Two jobs, and the second is the one that matters on device. Every later phase
 * that injects a SUPE frame — `lora <n> supe rx <hex>` — replays a line from
 * golden.txt rather than hand-hexed bytes, so the codec and the injections
 * cannot disagree. Regenerating the file after a codec change is how a
 * deliberate wire change is reviewed: the diff *is* the change.
 *
 * Nothing here links ESP-IDF. If this file stops compiling with a plain g++,
 * something ESP-specific has leaked into supe.cpp.
 */
#include "../src/supe.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>

static int g_fail = 0;
static int g_run  = 0;

static void ok(bool cond, const char* what) {
    g_run++;
    if (!cond) { g_fail++; printf("FAIL  %s\n", what); }
}

static void eqi(long got, long want, const char* what) {
    g_run++;
    if (got != want) { g_fail++; printf("FAIL  %s: got %ld want %ld\n", what, got, want); }
}

/* `0x`-prefixed, because that is what the device's own byte parser wants: a
 * bare hex run is taken as literal ASCII there, so an unprefixed vector would
 * inject the characters rather than the frame. Emitting it in the form the CLI
 * accepts is what makes a vector copy-pasteable, which is the whole point. */
static std::string hex(const uint8_t* p, size_t n) {
    std::string s = "0x";
    char b[4];
    for (size_t i = 0; i < n; i++) { snprintf(b, sizeof b, "%02x", p[i]); s += b; }
    return s;
}

/* ─────────────── golden vectors ─────────────── */

struct Golden { std::string name, bytes; };
static std::vector<Golden> g_golden;

static void golden(const char* name, const uint8_t* p, size_t n) {
    g_golden.push_back({ name, hex(p, n) });
}

/* The ladder itself is covered by testLadder2 below and, authoritatively, by
 * supe-ladder-vectors.txt over the full §14.3.4 cross-product. */

static void testQuantisation(void) {
    eqi(supeEncDur(0), 1, "a zero duration still claims one step");
    eqi(supeEncDur(20), 1, "20 ms is one duration step");
    eqi(supeEncDur(21), 2, "the duration encoding rounds up");
    eqi((long)supeDecDur(255), SUPE_DUR_MAX_MS, "the duration byte reaches 5.1 s");
    eqi(supeEncDur(999999), 255, "a duration past the ceiling clamps");
    eqi(supeEncLen(5), 1, "5 ms is one length step");
    eqi(supeEncLen(6), 2, "the length encoding rounds up");
    eqi((long)supeDecLen(255), SUPE_LEN_MAX_MS, "the length byte reaches 1.275 s");
    /* The train ceiling regime 1 states has to fit the field it is announced in. */
    ok(supeDecLen(supeEncLen(supeRegime(SUPE_REGIME_EU863)->trainCeilMs))
           >= supeRegime(SUPE_REGIME_EU863)->trainCeilMs,
       "the length byte reaches regime 1's train ceiling");
    ok(supeDecDur(supeEncDur(supeRegime(SUPE_REGIME_EU863)->txnCeilMs))
           >= supeRegime(SUPE_REGIME_EU863)->txnCeilMs,
       "the duration byte reaches regime 1's transaction ceiling");
}

static void testLevels(void) {
    eqi(supeDecLevel(supeEncLevel(14)), 14, "a transmit power round-trips");
    eqi(supeDecLevel(supeEncLevel(-130)), -130, "a level below −128 dBm round-trips");
    eqi(supeDecLevel(supeEncLevel(-192)), -192, "the bottom of the range round-trips");
    eqi(supeDecLevel(supeEncLevel(63)), 63, "the top of the range round-trips");
    eqi(supeDecLevel(supeEncLevel(-200)), -192, "below the range clamps");
    /* The adaptive flag rides in the top bit of the maximum-power byte, which is
     * free there because a transmit power never stores a negative value. */
    ok((supeEncLevel(22) & 0x80) == 0, "a transmit power leaves the top bit free");
    ok((supeEncLevel(-130) & 0x80) != 0, "a received level does not — hence the rule");
    eqi(supeDecSnr10(supeEncSnrQ(75)), 75, "7.5 dB of SNR round-trips");
    eqi(supeDecSnr10(supeEncSnrQ(-200)), -200, "−20 dB of SNR round-trips");
}

/* ─────────────── frames ─────────────── */

static void testTypeBytes(void) {
    /* The rule the receive path's dispatch rests on: the split framing reaches
     * every byte whose low nibble is 0 or 1, and no assigned type value is one
     * of those. */
    ok(supeIsFramingByte(0xC0) && supeIsFramingByte(0xC1), "0xC0/0xC1 are framing bytes");
    ok(supeIsFramingByte(0xD0) && supeIsFramingByte(0xD1), "0xD0/0xD1 are framing bytes");
    ok(!supeIsTypeByte(0xC0) && !supeIsTypeByte(0xD1), "framing bytes are not type bytes");
    ok(supeIsTypeByte(SUPE_T_START) && supeIsTypeByte(SUPE_T_ANNOUNCE2) &&
       supeIsTypeByte(SUPE_T_GRANT) && supeIsTypeByte(SUPE_T_MANIFEST),
       "every assigned type is a type byte");
    ok(!supeIsTypeByte(0xBF) && !supeIsTypeByte(0xE0), "the range is 0xC0–0xDF");
    for (int b = 0; b <= 0xFF; b++)
        if (supeIsTypeByte((uint8_t)b))
            ok((b & 0x0F) > 1, "no type byte is reachable by the framing");
}

static void testAnn2Codec(void) {
    SupeAnn2 a = {};
    a.regime = SUPE_REGIME_SINGLE;
    a.version = 0;
    a.caps.fam = SUPE_FAM_SX126X;
    a.caps.topStep = 2;
    a.caps.maxPwrDbm = 22;
    a.caps.adaptive = true;
    a.pwrDbm = 14;
    a.count = 2;
    uint8_t ids[2][4] = { { 0x6b, 0x87, 0xeb, 0x8b }, { 0x4e, 0x05, 0x21, 0x01 } };
    memcpy(a.ids, ids, sizeof ids);

    uint8_t f[SUPE_MAX_FRAME];
    size_t n = supeEncAnn2(f, sizeof f, &a);
    eqi((long)n, SUPE_ANN2_BASE + 2 * SUPE_ID_LEN, "ANNOUNCE2 is 5 + 4*count");
    golden("announce2.regime0.2ids", f, n);

    SupeAnn2 d = {};
    ok(supeDecAnn2(f, n, &d), "ANNOUNCE2 decodes");
    eqi(d.count, 2, "the count is implicit in the length");
    eqi(d.caps.fam, SUPE_FAM_SX126X, "family round-trips");
    eqi(d.caps.topStep, 2, "the ceiling round-trips");
    eqi(d.caps.maxPwrDbm, 22, "maximum power round-trips");
    ok(d.caps.adaptive, "the adaptive flag round-trips");
    eqi(d.pwrDbm, 14, "the frame's own power round-trips");
    ok(memcmp(d.ids[1], ids[1], 4) == 0, "the second identity round-trips");

    /* A single identity, which is what most nodes send. */
    a.count = 1;
    a.caps.adaptive = false;
    n = supeEncAnn2(f, sizeof f, &a);
    golden("announce2.regime0.1id", f, n);
    ok(supeDecAnn2(f, n, &d) && d.count == 1 && !d.caps.adaptive,
       "a one-identity ANNOUNCE2 round-trips");

    /* The bundling cap follows from the single-frame maximum and nothing else. */
    a.count = SUPE_ANN2_MAX;
    n = supeEncAnn2(f, sizeof f, &a);
    ok(n > 0 && n <= SUPE_MAX_FRAME, "a full bundle still fits one frame");
    a.count = SUPE_ANN2_MAX + 1;
    eqi((long)supeEncAnn2(f, sizeof f, &a), 0, "past the cap nothing is built");
    a.count = 0;
    eqi((long)supeEncAnn2(f, sizeof f, &a), 0, "an empty announcement is not a frame");

    /* Lengths that are not 5 + 4*count are discarded. */
    ok(!supeDecAnn2(f, SUPE_ANN2_BASE, &d), "a countless ANNOUNCE2 is discarded");
    uint8_t stub[8] = { SUPE_T_ANNOUNCE2, 0x00, 0, 0, 0, 0, 0, 0 };
    ok(!supeDecAnn2(stub, 8, &d), "a part-identity ANNOUNCE2 is discarded");
}

/* ─────────────── expiry ─────────────── */

static void testExpiry(void) {
    uint32_t built = supeBuildUnix();
    ok(built > 1750000000u, "the build timestamp is plausible");
    eqi((long)(supeExpiryUnix() - built), (long)SUPE_EXPIRY_DAYS * 86400,
        "the expiry is fourteen days past the build");
    ok(!supeExpired(built), "a fresh build is not expired");
    ok(!supeExpired(built + 13 * 86400), "thirteen days on it is still current");
    ok(supeExpired(built + 15 * 86400), "fifteen days on it is not");
    ok(!supeExpired(0), "an unresolved clock does not read as expired");
}

/* ─────────────── airtime ─────────────── */

static void testAirtime(void) {
    /* SUPE.md §3's quantisation table, at SF7/BW125 preamble 8 with the check
     * off: 1–3 bytes cost 26 ms, 4–7 cost 31, 8–10 cost 36. Each SUPE frame is
     * sized to sit on one of those group boundaries. */
    auto ms = [](int payload) {
        return (int)lround(1000.0 * supeAirtimeSeconds(7, 125000, 5, 8, payload, false, false));
    };
    eqi(ms(3), 26, "three bytes fill the first symbol group");
    eqi(ms(7), 31, "seven bytes fill the second — START sits exactly on it");
    eqi(ms(10), 36, "ten bytes fill the third — the identity form sits on it");
    ok(ms(4) == ms(7), "every length inside a group costs the same");
    ok(ms(8) > ms(7), "the byte that crosses costs the whole group");

    /* The check is what a SUPE frame does not pay for: sixteen bits push a frame
     * into the next group four times in seven. */
    int withCrc = (int)lround(1000.0 * supeAirtimeSeconds(7, 125000, 5, 8, 7, false, true));
    ok(withCrc > ms(7), "the CRC costs a symbol group at seven bytes");

    /* A network hailing slower moves every figure and none of the layouts. */
    ok((int)lround(1000.0 * supeAirtimeSeconds(12, 125000, 5, 8, 7, false, false)) > 400,
       "the same START costs the best part of half a second at SF12");
}

/* ─────────────── regime tables ─────────────── */

static void testRegimeTables(void) {
    const SupeRegime* g0 = supeRegime(SUPE_REGIME_SINGLE);
    const SupeRegime* g1 = supeRegime(SUPE_REGIME_EU863);
    ok(g0 && g1, "both regimes resolve");
    ok(supeRegime(9) == nullptr, "an unrecognised regime resolves to nothing");

    int n = 0;
    ok(supeRegimeChans(SUPE_REGIME_SINGLE, &n) == nullptr && n == 0,
       "regime 0 has no channel plan");
    const SupeChan* c = supeRegimeChans(SUPE_REGIME_EU863, &n);
    eqi(n, 9, "regime 1 names nine channels");
    ok(c && c[0].freqHz == 863350000u && c[8].freqHz == 868950000u,
       "the channel raster is the one in §14.2");
    /* At least 200 kHz of clear spectrum between any two edges, which is what
     * keeps each channel's airtime budget independent of its neighbours'. */
    for (int i = 1; i < n; i++) {
        uint32_t prevEdge = c[i - 1].freqHz + c[i - 1].bwHz / 2;
        uint32_t thisEdge = c[i].freqHz - c[i].bwHz / 2;
        ok(thisEdge >= prevEdge + 200000, "channels keep 200 kHz between edges");
    }
    /* Regime 0 must not invent ceilings it has no regulatory basis for. */
    ok(g0->trainCeilMs == 0 && g0->txnCeilMs == 0 && g0->airtimeMaxMs == 0,
       "regime 0 states no ceilings of its own");
    eqi((long)g1->airtimeMaxMs, 100000, "regime 1 allows 100 s …");
    eqi((long)g1->airtimeWinMs, 3600000, "… in any 3600 s");
    eqi(g1->maxTxpDbm, 14, "regime 1 caps radiated power at 25 mW e.r.p.");
    eqi(g0->maxTxpDbm, SUPE_TXP_IFACE, "regime 0 takes the interface's own power");
}

/* ═══════════════ the revised protocol ═══════════════ */

static void testLadder2(void) {
    /* §14.3.3's own table: SF7/BW125 hailing, 500 kHz channel, SX126x pair. */
    SupeLadderEntry lad[SUPE_LADDER_MAX_ENTRIES];
    int n = supeLadder(SUPE_REGIME_EU863, 0, 7, 125000, 500000,
                       SUPE_FAM_SX126X, SUPE_FAM_SX126X,
                       lad, SUPE_LADDER_MAX_ENTRIES);
    eqi(n, 9, "the §14.3.3 ladder has nine entries");
    struct { uint8_t sf; uint32_t bw; int margin; } want[] = {
        { 7, 125000,   0 }, { 6, 125000,  25 }, { 7, 250000,  30 },
        { 5, 125000,  50 }, { 6, 250000,  55 }, { 7, 500000,  60 },
        { 5, 250000,  80 }, { 6, 500000,  85 }, { 5, 500000, 110 },
    };
    for (int i = 0; i < n && i < 9; i++) {
        char what[48];
        snprintf(what, sizeof what, "ladder2 index %d", i);
        eqi(lad[i].sf, want[i].sf, what);
        eqi((long)lad[i].bwHz, (long)want[i].bw, what);
        eqi(lad[i].marginDeci, want[i].margin, what);
    }

    /* The same inputs with a 250 kHz channel maximum: the §14.3.1 rules admit
     * six entries — the first five of the table plus SF5/BW250 on top. (The
     * §14.3.3 prose says "five"; the rules and this file are the authority
     * when they and a reading of the prose disagree, §14.3.4.) Index 6 is
     * then invalid rather than meaning something else. */
    n = supeLadder(SUPE_REGIME_EU863, 0, 7, 125000, 250000,
                   SUPE_FAM_SX126X, SUPE_FAM_SX126X, lad, SUPE_LADDER_MAX_ENTRIES);
    eqi(n, 6, "a 250 kHz channel maximum gives a six-entry ladder");
    ok(lad[5].sf == 5 && lad[5].bwHz == 250000, "…topped by SF5/BW250");
    SupeCfg c;
    ok(!supeResolveBudget(SUPE_REGIME_EU863, 0, 7, 125000, 250000,
                          SUPE_FAM_SX126X, SUPE_FAM_SX126X, 6, &c),
       "budget 6 is invalid there, not something else");

    /* A pair including an SX127x has bandwidth entries alone: SF7/BW250 and
     * SF7/BW500 from SF7/BW125 on a 500 kHz channel. */
    n = supeLadder(SUPE_REGIME_EU863, 0, 7, 125000, 500000,
                   SUPE_FAM_SX127X, SUPE_FAM_SX126X, lad, SUPE_LADDER_MAX_ENTRIES);
    eqi(n, 3, "an SX127x pair keeps only the bandwidth entries");
    ok(lad[1].sf == 7 && lad[1].bwHz == 250000, "…SF7/BW250 first");
    ok(lad[2].sf == 7 && lad[2].bwHz == 500000, "…then SF7/BW500");

    /* Under regime 0 the same pair hailing at SF7 has index 0 and nothing
     * else — its first step would be the barred SF6. */
    n = supeLadder(SUPE_REGIME_SINGLE, 0, 7, 125000, 125000,
                   SUPE_FAM_SX127X, SUPE_FAM_SX127X, lad, SUPE_LADDER_MAX_ENTRIES);
    eqi(n, 1, "an SX127x pair hailing SF7 has no regime-0 entries above 0");
    /* From SF8 it has one (SF7), from SF9 two. */
    eqi(supeLadder(SUPE_REGIME_SINGLE, 0, 8, 125000, 125000,
                   SUPE_FAM_SX127X, SUPE_FAM_SX127X, lad, SUPE_LADDER_MAX_ENTRIES),
        2, "from SF8 the same pair has one entry above 0");
    eqi(supeLadder(SUPE_REGIME_SINGLE, 0, 9, 125000, 125000,
                   SUPE_FAM_SX127X, SUPE_FAM_SX127X, lad, SUPE_LADDER_MAX_ENTRIES),
        3, "from SF9 two");

    /* Index 0 is the hailing configuration even at a width no detour uses. */
    ok(supeResolveBudget(SUPE_REGIME_SINGLE, 0, 9, 62500, 62500,
                         SUPE_FAM_SX126X, SUPE_FAM_SX126X, 0, &c)
           && c.sf == 9 && c.bwHz == 62500,
       "budget 0 is the hailing configuration at an unusual bandwidth");

    /* Monotonic and undominated, over the big SX126x cross-section. */
    for (int hail = 7; hail <= 12; hail++) {
        n = supeLadder(SUPE_REGIME_EU863, 0, (uint8_t)hail, 125000, 500000,
                       SUPE_FAM_SX126X, SUPE_FAM_SX126X, lad, SUPE_LADDER_MAX_ENTRIES);
        for (int i = 1; i < n; i++) {
            ok(lad[i].marginDeci >= lad[i - 1].marginDeci,
               "margin cost never falls as the ladder climbs");
            ok(((lad[i].bwHz * (uint32_t)lad[i].sf) >> lad[i].sf) >=
               ((lad[i - 1].bwHz * (uint32_t)lad[i - 1].sf) >> lad[i - 1].sf),
               "the ordering key never falls");
        }
    }
}

static void testSyncWordFor(void) {
    SupeCfg hail = { 7, 125000, false, 0 };
    SupeCfg sf6  = { 6, 125000, false, 25 };
    SupeCfg sf5  = { 5, 500000, false, 110 };
    eqi(supeSyncWordFor(SUPE_REGIME_SINGLE, &hail, 0, 0x42), 0x42,
        "regime 0 budget 0 keeps the interface's word — it never left");
    eqi(supeSyncWordFor(SUPE_REGIME_SINGLE, &sf6, 1, 0x42), SUPE_SYNC_UNICAST,
        "a regime-0 budget above 0 takes 0x67 though the frequency stays");
    eqi(supeSyncWordFor(SUPE_REGIME_EU863, &hail, 0, 0x42), SUPE_SYNC_UNICAST,
        "a regime-1 budget 0 moved frequency, so it takes 0x67");
    eqi(supeSyncWordFor(SUPE_REGIME_EU863, &sf5, 8, 0x42), SUPE_SYNC_SF5,
        "a budget landing on SF5 takes 0x21");
}

static void testLoad(void) {
    /* ceil(Σ(bytes + 16) / 32), saturating at 255 → 8160 bytes. */
    eqi(supeEncLoad(0), 0, "an empty queue is load 0");
    eqi(supeEncLoad(1), 1, "one byte claims one unit");
    eqi(supeEncLoad(32), 1, "32 adjusted bytes is one unit");
    eqi(supeEncLoad(33), 2, "the encoding rounds up");
    eqi(supeEncLoad(500 + 16), 17, "one full packet is 17 units");
    eqi(supeEncLoad(1u << 30), 255, "past the range it saturates");
    eqi((long)supeDecLoadBytes(255), 8160, "a saturated load reads 8160 bytes");

    /* The airtime a load converts to, at the modulation the peer chose. A
     * saturated load at SF5/BW500 is about a second of air (§6: ~7 KB/s). */
    SupeCfg fast = { 5, 500000, false, 110 };
    uint32_t ms = supeLoadAirtimeMs(255, &fast, 5);
    ok(ms > 800 && ms < 1400, "a saturated load at SF5/BW500 is about a second");
    SupeCfg hail = { 7, 125000, false, 0 };
    ok(supeLoadAirtimeMs(255, &hail, 5) > 8 * ms,
       "the same load at hailing rate costs an order more");
    eqi((long)supeLoadAirtimeMs(0, &fast, 5), 0, "load 0 takes no air");
}

static void testDeadlines2(void) {
    /* §14.7's table: every deadline from two constants and a time on air. */
    SupeCfg fast = { 5, 500000, false, 110 };
    uint32_t g = supeGrantDeadlineMs(7, 125000, 5, 12);
    ok(g > SUPE_TURNAROUND_MS + SUPE_GUARD_MS, "the GRANT deadline covers its airtime");
    ok(g < 200, "…and is well under a fifth of a second at SF7");
    uint32_t m1 = supeManifestFirstDeadlineMs(&fast, 5, 12);
    uint32_t m2 = supeManifestReverseDeadlineMs(&fast, 5, 12);
    eqi((long)(m1 - m2), SUPE_RETUNE_GAP_MS,
        "the first MANIFEST waits one retune gap longer than the reverse one");
    eqi((long)supeLenDeadlineMs(supeEncLen(200)), 200 + SUPE_GUARD_MS,
        "a grace deadline is the stated length plus the guard");
}

static void testGrantCodec(void) {
    SupeGrant g = {};
    g.regime = SUPE_REGIME_EU863;
    g.version = 0;
    g.chan = 4;
    g.budget = 6;
    g.durByte = supeEncDur(1200);
    g.pwrDbm = 14;
    g.rssiDbm = -97;
    g.snrQ = supeEncSnrQ(-55);
    g.hash[0] = 0xab; g.hash[1] = 0xcd; g.hash[2] = 0xef;
    uint8_t f[16];
    size_t n = supeEncGrant(f, sizeof f, &g);
    eqi((long)n, SUPE_GRANT_LEN, "a GRANT is ten bytes");
    golden("grant.regime1.ch4.budget6", f, n);

    SupeGrant d = {};
    ok(supeDecGrant(f, n, &d), "the GRANT decodes");
    eqi(d.chan, 4, "channel round-trips");
    eqi(d.budget, 6, "budget round-trips");
    eqi((long)supeDecDur(d.durByte), 1200, "duration round-trips");
    eqi(d.pwrDbm, 14, "the frame's own power round-trips");
    eqi(d.rssiDbm, -97, "the START's level round-trips");
    eqi(supeDecSnr10(d.snrQ), -55, "the START's SNR round-trips");
    ok(memcmp(d.hash, g.hash, 3) == 0, "the START hash round-trips");
    ok(!supeGrantRefused(&d), "budget 6 is not a refusal");
    ok(!d.reverse, "no reverse traffic declared");

    /* The reverse-pending bit rides the power byte's free top bit — a
     * transmit power never stores a negative value. Set, it promises a
     * reverse MANIFEST after the requester's train; clear, both sides go
     * home on the train's end with no further frame. */
    g.reverse = true;
    n = supeEncGrant(f, sizeof f, &g);
    golden("grant.regime1.reverse", f, n);
    ok(supeDecGrant(f, n, &d) && d.reverse, "the reverse bit round-trips");
    eqi(d.pwrDbm, 14, "…without disturbing the stated power");
    g.reverse = false;

    /* Refusal is a first-class answer: budget 15, the channel nibble carries
     * the reason, and the measurement pair still rides it. */
    g.budget = SUPE_BUDGET_REFUSED;
    g.chan   = SUPE_REFUSE_AIRTIME;
    n = supeEncGrant(f, sizeof f, &g);
    golden("grant.refused.airtime", f, n);
    ok(supeDecGrant(f, n, &d) && supeGrantRefused(&d) && d.chan == SUPE_REFUSE_AIRTIME,
       "a refusal decodes with its reason");
    eqi(d.rssiDbm, -97, "a refusal still carries the free path-loss reading");

    ok(!supeDecGrant(f, 9, &d) && !supeDecGrant(f, 11, &d), "only ten bytes is a GRANT");
    uint8_t bogus[SUPE_GRANT_LEN];
    memcpy(bogus, f, SUPE_GRANT_LEN);
    bogus[1] = 0x90;
    ok(!supeDecGrant(bogus, SUPE_GRANT_LEN, &d), "an unknown regime is discarded");
}

static void testStart2Codec(void) {
    SupeStart2 s = {};
    s.regime = SUPE_REGIME_EU863;
    s.version = 0;
    s.tag[0] = 0xd1; s.tag[1] = 0x0d; s.tag[2] = 0x51;
    s.fam = SUPE_FAM_SX126X;
    s.ceiling = 8;
    s.load = supeEncLoad(2 * (500 + 16));   /* two full packets queued */
    uint8_t f[16];
    size_t n = supeEncStart2(f, sizeof f, &s);
    eqi((long)n, SUPE_START2_LEN, "a revised START is seven bytes");
    golden("start2.regime1.load2pkt", f, n);

    SupeStart2 d = {};
    ok(supeDecStart2(f, n, &d), "the revised START decodes");
    eqi(d.fam, SUPE_FAM_SX126X, "family round-trips");
    eqi(d.ceiling, 8, "ceiling round-trips");
    eqi((long)supeDecLoadBytes(d.load), 33 * 32, "the load round-trips in units");
    ok(memcmp(d.tag, s.tag, 3) == 0, "tag round-trips");
    ok(!d.haveIdent, "no identity in the short form");

    /* The ten-byte form is parsed and honoured while never being sent (§4). */
    s.haveIdent = true;
    s.ident[0] = 0x6b; s.ident[1] = 0x87; s.ident[2] = 0xeb;
    n = supeEncStart2(f, sizeof f, &s);
    eqi((long)n, SUPE_START2_ID_LEN, "a START naming its sender is ten bytes");
    golden("start2.regime1.ident", f, n);
    ok(supeDecStart2(f, n, &d) && d.haveIdent && memcmp(d.ident, s.ident, 3) == 0,
       "the ten-byte form decodes and carries the identity");
    ok(!supeDecStart2(f, 8, &d) && !supeDecStart2(f, 9, &d),
       "lengths between the two forms are discarded");
}

static void testManifest2Codec(void) {
    SupeManifest2 m = {};
    m.pwrDbm = 14;
    m.rssiDbm = -88;
    m.snrQ = supeEncSnrQ(35);
    m.caps.fam = SUPE_FAM_SX126X;
    m.caps.topStep = 8;
    m.caps.maxPwrDbm = 22;
    m.caps.adaptive = true;
    m.count = 7;
    m.lenByte = supeEncLen(640);
    m.hash[0] = 0xab; m.hash[1] = 0xcd; m.hash[2] = 0xef;
    uint8_t f[16];
    size_t n = supeEncManifest2(f, sizeof f, &m);
    eqi((long)n, SUPE_MANIFEST2_LEN, "a revised MANIFEST is eleven bytes");
    golden("manifest2.count7", f, n);

    SupeManifest2 d = {};
    ok(supeDecManifest2(f, n, &d), "the revised MANIFEST decodes");
    eqi(d.pwrDbm, 14, "the train's power round-trips");
    eqi(d.rssiDbm, -88, "the peer's level round-trips");
    eqi(supeDecSnr10(d.snrQ), 35, "the SNR round-trips");
    eqi(d.caps.topStep, 8, "capabilities round-trip");
    ok(d.caps.adaptive, "the adaptive flag round-trips");
    eqi(d.count, 7, "the packet count round-trips");
    eqi((long)supeDecLen(d.lenByte), 640, "the train length round-trips");

    /* Count zero is meaningful: 0/0 closes, 0 with a length is the grace. */
    m.count = 0;
    m.lenByte = 0;
    n = supeEncManifest2(f, sizeof f, &m);
    golden("manifest2.close", f, n);
    ok(supeDecManifest2(f, n, &d) && d.count == 0 && d.lenByte == 0,
       "the count-0 close round-trips");
    m.lenByte = supeEncLen(100);
    n = supeEncManifest2(f, sizeof f, &m);
    golden("manifest2.grace100ms", f, n);
    ok(supeDecManifest2(f, n, &d) && d.count == 0 && supeDecLen(d.lenByte) == 100,
       "the grace round-trips");

    ok(!supeDecManifest2(f, 10, &d), "the legacy ten-byte layout is not a revised MANIFEST");
}

static void testLenOk2(void) {
    ok(supeLenOk2(SUPE_T_GRANT, SUPE_REGIME_SINGLE, 0, 10), "GRANT's length is enumerable");
    ok(!supeLenOk2(SUPE_T_GRANT, SUPE_REGIME_SINGLE, 0, 7), "…and unique");
    ok(!supeLenOk2(0xC8, SUPE_REGIME_SINGLE, 0, 7),
       "0xC8 is burned — discarded exactly as a wrong length is");
    ok(!supeLenOk2(0xC3, SUPE_REGIME_SINGLE, 0, 7), "0xC3 stays reserved");
    ok(supeLenOk2(SUPE_T_MANIFEST, SUPE_REGIME_SINGLE, 0, SUPE_MANIFEST2_LEN),
       "the revised MANIFEST length is enumerable");
    ok(!supeLenOk2(SUPE_T_MANIFEST, SUPE_REGIME_SINGLE, 0, 10),
       "the legacy ten-byte MANIFEST length is not");
}

/* ─────────────── the conformance vectors (§14.3.4) ───────────────
 *
 * The full cross-product: requesting family × answering family × hailing SF
 * 5–12 × hailing bandwidth × channel maximum bandwidth × both regimes. Each
 * line gives the inputs, the ladder length, and every entry as
 * index:sf/bw/ldro/sync. `iface` marks the one entry that keeps the
 * interface's own sync word. The file is the authority when it and a reading
 * of §14.3 disagree. */
static void writeLadderVectors(const char* path) {
    static const uint32_t kBw[] = { 125000, 250000, 500000 };
    FILE* fp = fopen(path, "w");
    if (!fp) { ok(false, "supe-ladder-vectors.txt is writable"); return; }
    fprintf(fp,
        "# SUPE ladder conformance vectors (SUPE.md \u00a714.3.4), regenerated by\n"
        "# supe_core_test. An implementation is conformant iff it reproduces this\n"
        "# file exactly. Entries are index:sf/bw/ldro/sync; `iface` is the\n"
        "# interface's own sync word. The ladder is truncated at 15 entries:\n"
        "# the budget nibble reaches 14, 15 being the refusal.\n");
    int lines = 0;
    for (int regime = 0; regime <= 1; regime++)
    for (int famA = 0; famA <= 4; famA++)
    for (int famB = 0; famB <= 4; famB++)
    for (int sf = 5; sf <= 12; sf++)
    for (size_t hb = 0; hb < sizeof kBw / sizeof kBw[0]; hb++)
    for (size_t cb = 0; cb < sizeof kBw / sizeof kBw[0]; cb++) {
        SupeLadderEntry lad[SUPE_LADDER_MAX_ENTRIES];
        int n = supeLadder((uint8_t)regime, 0, (uint8_t)sf, kBw[hb], kBw[cb],
                           (uint8_t)famA, (uint8_t)famB,
                           lad, SUPE_LADDER_MAX_ENTRIES);
        fprintf(fp, "r=%d famA=%d famB=%d hail=%d/%u chmax=%u n=%d",
                regime, famA, famB, sf, (unsigned)kBw[hb], (unsigned)kBw[cb], n);
        for (int i = 0; i < n; i++) {
            SupeCfg c = { lad[i].sf, lad[i].bwHz, lad[i].ldro, lad[i].marginDeci };
            bool iface = (regime == 0 && i == 0);
            fprintf(fp, " %d:%u/%u/%d/", i, lad[i].sf, (unsigned)lad[i].bwHz,
                    lad[i].ldro ? 1 : 0);
            if (iface) fprintf(fp, "iface");
            else       fprintf(fp, "0x%02x", supeSyncWordFor((uint8_t)regime, &c,
                                                             (uint8_t)i, 0x42));
        }
        fprintf(fp, "\n");
        lines++;
    }
    fclose(fp);
    printf("wrote %d ladder vectors to %s\n", lines, path);
}

/* ─────────────── main ─────────────── */

int main(int argc, char** argv) {
    testRegimeTables();
    testQuantisation();
    testLevels();
    testTypeBytes();
    testAnn2Codec();
    testExpiry();
    testAirtime();
    testLadder2();
    testSyncWordFor();
    testLoad();
    testDeadlines2();
    testGrantCodec();
    testStart2Codec();
    testManifest2Codec();
    testLenOk2();

    writeLadderVectors((argc > 2) ? argv[2] : "supe-ladder-vectors.txt");

    const char* out = (argc > 1) ? argv[1] : "golden.txt";
    FILE* fp = fopen(out, "w");
    if (fp) {
        fprintf(fp, "# SUPE golden frame vectors — regenerated by supe_core_test.\n"
                    "# Replay one on device: `lora <n> supe rx 0x…` — paste the second\n"
                    "# column verbatim. Never hand-hex a frame, or the codec and the\n"
                    "# injection can disagree, which is the one thing this file prevents.\n");
        for (auto& g : g_golden) fprintf(fp, "%-28s %s\n", g.name.c_str(), g.bytes.c_str());
        fclose(fp);
        printf("wrote %zu golden vectors to %s\n", g_golden.size(), out);
    }

    printf("%d checks, %d failed\n", g_run, g_fail);
    return g_fail ? 1 : 0;
}
