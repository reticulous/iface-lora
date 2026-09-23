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

/* ─────────────── quantised fields, levels ─────────────── */

static void testQuantisation(void) {
    eqi(supeEncLen(5), 1, "5 ms is one length step");
    eqi(supeEncLen(6), 2, "the length encoding rounds up");
    eqi((long)supeDecLen(255), SUPE_LEN_MAX_MS, "the length byte reaches 1.275 s");
    /* The train ceiling regime 1 states has to fit the field it is announced in. */
    ok(supeDecLen(supeEncLen(supeRegime(SUPE_REGIME_EU863)->trainCeilMs))
           >= supeRegime(SUPE_REGIME_EU863)->trainCeilMs,
       "the length byte reaches regime 1's train ceiling");
}

static void testLevels(void) {
    eqi(supeDecLevel(supeEncLevel(14)), 14, "a transmit power round-trips");
    eqi(supeDecLevel(supeEncLevel(-130)), -130, "a level below −128 dBm round-trips");
    eqi(supeDecLevel(supeEncLevel(-192)), -192, "the bottom of the range round-trips");
    eqi(supeDecLevel(supeEncLevel(63)), 63, "the top of the range round-trips");
    eqi(supeDecLevel(supeEncLevel(-200)), -192, "below the range clamps");
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
    ok(supeIsTypeByte(SUPE_T_HAIL) && supeIsTypeByte(SUPE_T_ANNOUNCE) &&
       supeIsTypeByte(SUPE_T_GOT) && supeIsTypeByte(SUPE_T_READY) &&
       supeIsTypeByte(SUPE_T_END) && supeIsTypeByte(SUPE_T_BYE) &&
       supeIsTypeByte(SUPE_T_RESEND),
       "every assigned type is a type byte");
    ok(!supeIsTypeByte(0xBF) && !supeIsTypeByte(0xE0), "the range is 0xC0–0xDF");
    for (int b = 0; b <= 0xFF; b++)
        if (supeIsTypeByte((uint8_t)b))
            ok((b & 0x0F) > 1, "no type byte is reachable by the framing");
    /* Values assigned densely from 0xC2; nothing is held out beyond what the
     * framing rule excludes. */
    eqi(SUPE_T_HAIL, 0xC2, "HAIL is 0xC2");
    eqi(SUPE_T_RESEND, 0xC8, "RESEND is 0xC8, the last assigned value");
}

static void testAnnCodec(void) {
    SupeAnn a = {};
    a.regime = SUPE_REGIME_SINGLE;
    a.version = 0;
    a.caps.fam = SUPE_FAM_SX126X;
    a.caps.topStep = 2;
    a.caps.maxPwrDbm = 22;
    a.pwrDbm = 14;
    a.count = 2;
    uint8_t ids[2][4] = { { 0x6b, 0x87, 0xeb, 0x8b }, { 0x4e, 0x05, 0x21, 0x01 } };
    memcpy(a.ids, ids, sizeof ids);

    uint8_t f[SUPE_MAX_FRAME];
    size_t n = supeEncAnn(f, sizeof f, &a);
    eqi((long)n, SUPE_ANN_BASE + 2 * SUPE_ID_LEN, "two identities encode to 13 bytes");
    eqi(f[0], SUPE_T_ANNOUNCE, "the type byte is 0xC3");
    golden("announce.regime0.2ids", f, n);

    SupeAnn d = {};
    ok(supeDecAnn(f, n, &d), "the announcement decodes");
    eqi(d.count, 2, "both identities survive");
    ok(memcmp(d.ids[0], ids[0], 4) == 0 && memcmp(d.ids[1], ids[1], 4) == 0,
       "byte for byte");
    eqi(d.caps.maxPwrDbm, 22, "maximum power round-trips");
    eqi(d.pwrDbm, 14, "the frame's own power round-trips");
    ok(!supeDecAnn(f, n - 1, &d), "a truncated announcement is discarded");
    ok(!supeDecAnn(f, n + 1, &d), "a padded one too");

    a.count = 1;
    n = supeEncAnn(f, sizeof f, &a);
    golden("announce.regime0.1id", f, n);

    /* "I do not speak SUPE" — the one announcement that is not about a dialect.
     * It must decode without being checked against a regime table it is not
     * claiming membership of, and it must still carry the identities. */
    a.regime = SUPE_REGIME_NONE;
    a.version = 0;
    n = supeEncAnn(f, sizeof f, &a);
    ok(n > 0, "the renunciation encodes");
    eqi((long)(f[1] >> 4), SUPE_REGIME_NONE, "the regime nibble carries it");
    golden("announce.notspeaking.1id", f, n);
    ok(supeDecAnn(f, n, &d), "…and decodes without a regime table entry");
    eqi(d.regime, SUPE_REGIME_NONE, "…as the renunciation");
    eqi(d.count, 1, "…still naming the node");
    ok(memcmp(d.ids[0], ids[0], 4) == 0, "…by its identity");
    f[1] = (uint8_t)((SUPE_REGIME_NONE << 4) | 0x0D);
    ok(supeDecAnn(f, n, &d), "any version decodes — there is no dialect to match");
    f[1] = 0x9F;
    ok(!supeDecAnn(f, n, &d), "an unknown regime is still discarded");
}

static void testHailCodec(void) {
    SupeHail h = {};
    h.regime = SUPE_REGIME_EU863;
    h.version = 0;
    uint8_t tag[3] = { 0xd1, 0x0d, 0x51 };
    memcpy(h.tag, tag, 3);
    h.pwrDbm = -9;
    h.salt = 0x5a;
    h.budgetCeil = 8;
    h.count = 3;
    h.lenByte = supeEncLen(450);
    uint8_t f[SUPE_HAIL_ID_LEN];
    size_t n = supeEncHail(f, sizeof f, &h);
    eqi((long)n, SUPE_HAIL_LEN, "the anonymous form is ten bytes");
    eqi(f[0], SUPE_T_HAIL, "the type byte is 0xC2");
    golden("hail.regime1.anon", f, n);
    SupeHail d = {};
    ok(supeDecHail(f, n, &d), "it decodes");
    ok(!d.haveIdent, "…as anonymous");
    eqi(d.pwrDbm, -9, "the stated power round-trips — the hail is a measurement");
    ok(memcmp(d.tag, tag, 3) == 0, "the tag round-trips");
    eqi(d.salt, 0x5a, "the salt round-trips — every seed is unique");
    eqi(d.budgetCeil, 8, "the proposed ceiling rides");
    eqi(d.count, 3, "the train's count rides");
    eqi(d.lenByte, supeEncLen(450), "…and its length at the ceiling");

    h.haveIdent = true;
    uint8_t id[3] = { 0xa1, 0xa2, 0xa3 };
    memcpy(h.ident, id, 3);
    n = supeEncHail(f, sizeof f, &h);
    eqi((long)n, SUPE_HAIL_ID_LEN, "the named form is thirteen bytes");
    golden("hail.regime1.ident", f, n);
    ok(supeDecHail(f, n, &d) && d.haveIdent && memcmp(d.ident, id, 3) == 0,
       "the sender identity rides the frame length");
    ok(!supeDecHail(f, 11, &d), "a length outside the enumerated set is discarded");
    f[1] = 0x9F;                       /* regime 9: not one this build holds */
    ok(!supeDecHail(f, n, &d), "an unknown regime is discarded");

    /* A hail-back: a count of zero, and nothing else about it is special. */
    h.regime = SUPE_REGIME_SINGLE;
    h.count = 0;
    h.lenByte = 0;
    n = supeEncHail(f, sizeof f, &h);
    golden("hail.regime0.hailback", f, n);
    ok(supeDecHail(f, n, &d) && d.count == 0, "a hail-back decodes with a count of zero");
    h.count = SUPE_TRAIN_MAX + 1;
    ok(supeEncHail(f, sizeof f, &h) == 0, "a count past the train cap refuses");
}

static void testReadyCodec(void) {
    SupeReady g = {};
    uint8_t hash[3] = { 0xde, 0xad, 0x01 };
    memcpy(g.hash, hash, 3);
    g.pwrDbm = 0;
    g.budget = 6;
    g.countCeil = 12;
    g.heardRssi = -95;
    g.heardSnrQ = supeEncSnrQ(60);
    uint8_t f[SUPE_READY_LEN];
    size_t n = supeEncReady(f, sizeof f, &g);
    eqi((long)n, SUPE_READY_LEN, "READY is nine bytes, always");
    golden("gimme.budget6", f, n);
    SupeReady d = {};
    ok(supeDecReady(f, n, &d), "it decodes");
    eqi(d.heardRssi, -95, "the reading of the frame it answers rides");
    eqi(d.budget, 6, "the confirmed budget rides");
    eqi(d.countCeil, 12, "the count ceiling rides — a RAM promise");
    ok(!supeDecReady(f, 8, &d), "a length outside the enumerated set is discarded");
    ok(!supeDecReady(f, 10, &d), "…either way");
}

static void testGotCodec(void) {
    SupeGot h = {};
    uint8_t hash[3] = { 0xde, 0xad, 0x01 };
    memcpy(h.g.hash, hash, 3);
    h.g.pwrDbm = 2;
    h.g.budget = 8;
    h.g.countCeil = 12;
    h.g.heardRssi = -70;
    h.g.heardSnrQ = supeEncSnrQ(90);
    h.count = 5;
    h.lenByte = supeEncLen(200);
    uint8_t f[SUPE_GOT_ANS_BASE + SUPE_MASK_MAX];
    size_t n = supeEncGot(f, sizeof f, &h);
    eqi((long)n, SUPE_GOT_LEN, "the opening form is eleven bytes — READY with a train behind it");
    golden("have.open.5frames", f, n);
    SupeGot d = {};
    ok(supeDecGot(f, n, 0, &d), "it decodes with no peer train in hand");
    ok(!d.answering, "…as the opening form");
    eqi(d.g.budget, 8, "the proposed ceiling rides");
    eqi(d.count, 5, "the count rides");
    eqi(d.g.pwrDbm, 2, "the meeting power rides");
    eqi(d.g.heardRssi, -70, "the reading rides — READY's fields are GOT's first nine");

    /* The answering form: its length is enumerable only from the peer train's
     * count, which both sides hold. */
    h.answering = true;
    h.g.heardRssi = -88;
    h.g.heardSnrQ = supeEncSnrQ(45);
    h.maskLen = supeMaskLen(7);
    h.mask[0] = 0x22;                  /* frames 1 and 5 of theirs are missing */
    n = supeEncGot(f, sizeof f, &h);
    eqi((long)n, SUPE_GOT_ANS_BASE + 1, "answering: 11 bytes + one mask byte");
    golden("have.answer.mask22", f, n);
    ok(supeDecGot(f, n, 7, &d) && d.answering, "it decodes against count 7");
    eqi(d.g.heardRssi, -88, "the train's worst reading rides");
    eqi(d.mask[0], 0x22, "the repair request rides");
    ok(!supeDecGot(f, n, 0, &d),
       "the answering form is not decodable without the peer count");
    ok(!supeDecGot(f, n, 12, &d), "…or against the wrong one");
}

static void testEndCodec(void) {
    SupeEnd t = {};
    t.pwrDbm = 5;
    t.count = 4;
    t.csum[0] = 0x11; t.csum[1] = 0x22; t.csum[2] = 0x22; t.csum[3] = 0x44;
    uint8_t f[SUPE_END_BASE + SUPE_TRAIN_MAX];
    size_t n = supeEncEnd(f, sizeof f, &t);
    eqi((long)n, SUPE_END_BASE + 4, "no count byte: the length says n");
    golden("thatsit.4frames", f, n);
    SupeEnd d = {};
    ok(supeDecEnd(f, n, &d), "it decodes");
    eqi(d.count, 4, "the count comes from the frame's own length");
    eqi(d.pwrDbm, 5,
        "the train's power rides — stated after the fact, chosen on the report");
    ok(memcmp(d.csum, t.csum, 4) == 0, "the checksum list IS the sequence");
    ok(!d.haveHeard,
       "no reading rides as the sentinel, not as a zero — 0 dBm is a level");

    /* The reading: how the peer's last frame reached the sender. It is the
     * answering side's only measurement of the direction it transmits in. */
    t.haveHeard = true;
    t.heardRssi = -103;
    t.heardSnrQ = supeEncSnrQ(-45);
    n = supeEncEnd(f, sizeof f, &t);
    ok(supeDecEnd(f, n, &d), "it decodes with a reading");
    ok(d.haveHeard, "the reading is there");
    eqi(d.heardRssi, -103, "the level rides");
    eqi(supeDecSnr10(d.heardSnrQ), -45, "and its signal-to-noise, quarter-dB");
    eqi(d.pwrDbm, 5, "beside the train's own power, which is a different fact");

    t.count = SUPE_TRAIN_MAX + 1;
    ok(supeEncEnd(f, sizeof f, &t) == 0, "a count past the train cap refuses");
}

static void testByeResendCodec(void) {
    uint8_t bye[1] = { SUPE_T_BYE };
    golden("bye", bye, 1);

    SupeResendF r = {};
    r.maskLen = supeMaskLen(9);
    r.mask[0] = 0x05;
    r.mask[1] = 0x01;                  /* frames 0, 2 and 8 are missing */
    uint8_t f[SUPE_RESEND_BASE + SUPE_MASK_MAX];
    size_t n = supeEncResend(f, sizeof f, &r);
    eqi((long)n, SUPE_RESEND_BASE + 2, "nine frames want two mask bytes");
    golden("resend.mask0501", f, n);
    SupeResendF d = {};
    ok(supeDecResend(f, n, 9, &d), "it decodes against count 9");
    ok(d.mask[0] == 0x05 && d.mask[1] == 0x01, "the bitmask rides");
    ok(!supeDecResend(f, n, 5, &d), "…and not against a count wanting one byte");
}

/* ─────────────── the checksum ─────────────── */

static void testCrc8(void) {
    /* CRC-8 poly 0x07, init 0, MSB first: the check value for "123456789". */
    eqi(supeCrc8((const uint8_t*)"123456789", 9), 0xF4, "the CRC-8 check value");
    eqi(supeCrc8(nullptr, 0), 0x00, "an empty frame checks to zero");
    uint8_t a[3] = { 1, 2, 3 }, b[3] = { 1, 2, 4 };
    ok(supeCrc8(a, 3) != supeCrc8(b, 3), "one flipped bit separates");
}

/* ─────────────── the sync-word list (§14.5) ─────────────── */

static void testSyncWords(void) {
    uint8_t w[SUPE_SYNC_WORDS_CAP];
    int n7 = supeSyncWords(7, 0x42, w, (int)sizeof w);
    ok(n7 > 100, "SF7 admits a three-digit word list");
    for (int i = 0; i < n7; i++) {
        ok((w[i] >> 4) != 0 && (w[i] & 0x0F) != 0, "no zero nibbles");
        if (i) ok(w[i] > w[i - 1], "the list is ascending");
    }
    /* The berth: two nibble-steps in BOTH symbols around every foreign word. */
    for (int i = 0; i < n7; i++) {
        ok(w[i] != 0x12 && w[i] != 0x24 && w[i] != 0x34 && w[i] != 0x42,
           "no foreign word appears");
        int d1 = (w[i] >> 4) - 1, d2 = (w[i] & 0x0F) - 2;   /* vs 0x12 */
        if (d1 < 0) d1 = -d1;
        if (d2 < 0) d2 = -d2;
        ok(d1 > 2 || d2 > 2, "every word escapes 0x12's berth in one symbol");
    }
    /* SF6: nibbles cap at 7; still a usable list. */
    int n6 = supeSyncWords(6, 0x42, w, (int)sizeof w);
    ok(n6 >= 8, "SF6 keeps a usable list");
    for (int i = 0; i < n6; i++)
        ok((w[i] >> 4) <= 7 && (w[i] & 0x0F) <= 7, "SF6 nibbles fit 64 bins");
    /* SF5: exact exclusions only — the berth rule would empty its nine words. */
    int n5 = supeSyncWords(5, 0x42, w, (int)sizeof w);
    eqi(n5, 8, "SF5 keeps eight of its nine words (0x12 excluded exactly)");
    for (int i = 0; i < n5; i++)
        ok((w[i] >> 4) <= 3 && (w[i] & 0x0F) <= 3, "SF5 nibbles fit 32 bins");

    /* Indexing is total: every stream byte lands on a word. */
    for (int s = 0; s <= 255; s++) {
        uint8_t word = supeSyncWordAt(7, 0x42, (uint8_t)s);
        ok(word != 0x00, "every stream byte resolves to a word");
    }
    /* The configured hailing word moves the list — both ends share it. */
    int nAlt = supeSyncWords(7, 0x77, w, (int)sizeof w);
    ok(nAlt != 0 && nAlt != n7, "a different hailing word reshapes the list");
}

/* ─────────────── the schedule (§7) ─────────────── */

static void fillDigest(uint8_t d[32], uint8_t seed) {
    for (int i = 0; i < 32; i++) d[i] = (uint8_t)(seed + i * 7);
}

static void testSchedule(void) {
    uint8_t d0[32], d1[32];
    fillDigest(d0, 0x10);
    fillDigest(d1, 0x81);

    SupeSchedD s;
    supeDeriveSchedule(d0, d1, /*wide=*/false, /*nChans=*/9, &s);
    ok(memcmp(s.hash3, d0, 3) == 0, "the wire id is D0's first three bytes");
    eqi(s.nSlots, 2, "a narrow schedule holds exactly two slots");
    eqi(s.slot[0].tMs, SUPE_NARROW_T0_MS,
        "the first narrow slot is one seed gap after the epoch");
    for (int k = 0; k < s.nSlots; k++) {
        ok(s.slot[k].tMs <= SUPE_NARROW_HORIZON_MS, "slots stay inside the horizon");
        ok(s.slot[k].chan >= 1 && s.slot[k].chan <= 9, "channels come from the raster");
        if (k) {
            int gap = s.slot[k].tMs - s.slot[k - 1].tMs;
            ok(gap >= 100 && gap <= 123, "narrow spacing is 100 + (j mod 24)");
            ok(s.slot[k].chan != s.slot[0].chan, "the second slot is never on the first's channel");
        }
    }
    /* Every seed, not just this one: the second channel differs. */
    for (int seed = 0; seed < 64; seed++) {
        uint8_t x0[32], x1[32];
        fillDigest(x0, (uint8_t)(seed * 5 + 1));
        fillDigest(x1, (uint8_t)(seed * 11 + 3));
        SupeSchedD t;
        supeDeriveSchedule(x0, x1, false, 9, &t);
        ok(t.nSlots == 2 && t.slot[1].chan != t.slot[0].chan,
           "two slots on two channels, for every seed");
    }

    SupeSchedD w;
    supeDeriveSchedule(d0, d1, /*wide=*/true, /*nChans=*/9, &w);
    ok(w.nSlots >= 8, "a wide schedule holds a dozen-odd slots");
    ok(w.slot[0].tMs >= 150 && w.slot[0].tMs <= 189,
       "the first wide slot is 150 ms + jitter after the epoch");
    for (int k = 1; k < w.nSlots; k++) {
        int gap = w.slot[k].tMs - w.slot[k - 1].tMs;
        ok(gap >= 60 && gap <= 350 + 39, "wide spacing widens and caps at 350 + jitter");
        ok(w.slot[k].tMs <= SUPE_WIDE_HORIZON_MS, "…inside the 3 s horizon");
    }

    /* No channel raster: the derivation still runs, every slot on channel 0 —
     * regime 0 itself derives no schedule (§7), so the vectors leave it out. */
    SupeSchedD r0;
    supeDeriveSchedule(d0, d1, false, 0, &r0);
    for (int k = 0; k < r0.nSlots; k++)
        eqi(r0.slot[k].chan, SUPE_CH_HAIL, "with no raster, slots stay on channel 0");
    ok(!supeRegimeHasPlan(SUPE_REGIME_SINGLE) && supeRegimeHasPlan(SUPE_REGIME_EU863),
       "regime 0 has no plan, regime 1 has one");
    eqi(supeListenSfLow(SUPE_FAM_LR2021, 7), 5, "family 4 listens down to SF5 from SF7");
    eqi(supeListenSfLow(SUPE_FAM_LR2021, 9), 6, "…and to SF6 from SF9: four detectors");
    eqi(supeListenSfLow(SUPE_FAM_SX126X, 7), 7, "every other family listens at the hailing SF");

    /* Determinism: the same digests derive the same schedule, byte for byte. */
    SupeSchedD again;
    supeDeriveSchedule(d0, d1, true, 9, &again);
    ok(memcmp(&again, &w, sizeof w) == 0, "the derivation is a pure function");
    /* And a different seed moves everything. */
    fillDigest(d0, 0x11);
    supeDeriveSchedule(d0, d1, true, 9, &again);
    ok(memcmp(&again, &w, sizeof w) != 0, "a different seed derives elsewhere");
}

/* ─────────────── expiry ─────────────── */

static void testExpiry(void) {
    uint32_t built  = supeBuildUnix();
    uint32_t expires = supeExpiryUnix();
    ok(built > 1750000000u, "the build timestamp is plausible");
    /* 2026-09-10T00:00:00Z. Stated as the number so the test fails when the
     * date moves without the test being looked at. */
    eqi((long)expires, 1791590400L, "the expiry is the stated calendar date");
    ok(expires > built, "this build was made before its own expiry");
    ok(!supeExpired(built), "a fresh build is not expired");
    ok(!supeExpired(expires - 86400), "the day before, it is still current");
    ok(supeExpired(expires), "on the day, it is not");
    ok(!supeExpired(0), "an unresolved clock does not read as expired");
}

/* ─────────────── airtime ─────────────── */

static void testAirtime(void) {
    /* SUPE.md §3's quantisation table, at SF7/BW125 preamble 8 with the check
     * off: 1–3 bytes cost 26 ms, 4–7 cost 31, 8–10 cost 36. */
    auto ms = [](int payload) {
        return (int)lround(1000.0 * supeAirtimeSeconds(7, 125000, 5, 8, payload, false, false));
    };
    eqi(ms(3), 26, "three bytes fill the first symbol group");
    eqi(ms(7), 31, "seven bytes fill the second");
    eqi(ms(10), 36, "ten bytes fill the third — READY's narrow form sits on it");
    ok(ms(4) == ms(7), "every length inside a group costs the same");
    ok(ms(7) == ms(4), "PRIVSYNC's seven bytes fill the second group exactly");
    ok(ms(8) > ms(7), "the byte that crosses costs the whole group");

    int withCrc = (int)lround(1000.0 * supeAirtimeSeconds(7, 125000, 5, 8, 7, false, true));
    ok(withCrc > ms(7), "the CRC costs a symbol group at seven bytes");
}

/* Sensitivity is the yardstick every margin test measures against, so the one
 * answer it must never give is a confident wrong one. A blank configuration —
 * a caller that forgot to fill it — puts zero into a logarithm, and taken at
 * face value that lands on 0 dBm: a receiver that needs a signal stronger than
 * any transmitter can produce, against which every real link reads as too
 * weak. */
static void testSensitivity(void) {
    SupeCfg hail = {}; hail.sf = 7; hail.bwHz = 125000;
    SupeCfg fast = {}; fast.sf = 5; fast.bwHz = 500000;
    int16_t sHail = supeSensitivityDeci(&hail);
    int16_t sFast = supeSensitivityDeci(&fast);
    ok(sHail < -1150 && sHail > -1350, "SF7/125k lands near -123 dBm");
    ok(sFast > sHail, "a wider, faster regime needs a stronger signal");

    SupeCfg blank = {};
    int16_t sBlank = supeSensitivityDeci(&blank);
    ok(sBlank < -900, "a configuration with no bandwidth reads as unknown, not 0 dBm");
    ok(supeSensitivityDeci(nullptr) == sBlank, "…and so does no configuration at all");
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
    for (int i = 1; i < n; i++) {
        uint32_t prevEdge = c[i - 1].freqHz + c[i - 1].bwHz / 2;
        uint32_t thisEdge = c[i].freqHz - c[i].bwHz / 2;
        ok(thisEdge >= prevEdge + 200000, "channels keep 200 kHz between edges");
    }
    ok(g0->trainCeilMs == 0 && g0->txnCeilMs == 0 && g0->airtimeMaxMs == 0,
       "regime 0 states no ceilings of its own");
    eqi((long)g1->airtimeMaxMs, 100000, "regime 1 allows 100 s …");
    eqi((long)g1->airtimeWinMs, 3600000, "… in any 3600 s");
    eqi(g1->maxTxpDbm, 14, "regime 1 caps radiated power at 25 mW e.r.p.");
    eqi(g0->maxTxpDbm, SUPE_TXP_IFACE, "regime 0 takes the interface's own power");
}

/* ─────────────── the ladder ─────────────── */

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

    n = supeLadder(SUPE_REGIME_EU863, 0, 7, 125000, 250000,
                   SUPE_FAM_SX126X, SUPE_FAM_SX126X, lad, SUPE_LADDER_MAX_ENTRIES);
    eqi(n, 6, "a 250 kHz channel maximum gives a six-entry ladder");
    ok(lad[5].sf == 5 && lad[5].bwHz == 250000, "…topped by SF5/BW250");
    SupeCfg c;
    ok(!supeResolveBudget(SUPE_REGIME_EU863, 0, 7, 125000, 250000,
                          SUPE_FAM_SX126X, SUPE_FAM_SX126X, 6, &c),
       "budget 6 is invalid there, not something else");

    n = supeLadder(SUPE_REGIME_EU863, 0, 7, 125000, 500000,
                   SUPE_FAM_SX127X, SUPE_FAM_SX126X, lad, SUPE_LADDER_MAX_ENTRIES);
    eqi(n, 3, "an SX127x pair keeps only the bandwidth entries");
    ok(lad[1].sf == 7 && lad[1].bwHz == 250000, "…SF7/BW250 first");
    ok(lad[2].sf == 7 && lad[2].bwHz == 500000, "…then SF7/BW500");

    n = supeLadder(SUPE_REGIME_SINGLE, 0, 7, 125000, 125000,
                   SUPE_FAM_SX127X, SUPE_FAM_SX127X, lad, SUPE_LADDER_MAX_ENTRIES);
    eqi(n, 1, "an SX127x pair hailing SF7 has no regime-0 entries above 0");
    eqi(supeLadder(SUPE_REGIME_SINGLE, 0, 8, 125000, 125000,
                   SUPE_FAM_SX127X, SUPE_FAM_SX127X, lad, SUPE_LADDER_MAX_ENTRIES),
        2, "from SF8 the same pair has one entry above 0");
    eqi(supeLadder(SUPE_REGIME_SINGLE, 0, 9, 125000, 125000,
                   SUPE_FAM_SX127X, SUPE_FAM_SX127X, lad, SUPE_LADDER_MAX_ENTRIES),
        3, "from SF9 two");

    ok(supeResolveBudget(SUPE_REGIME_SINGLE, 0, 9, 62500, 62500,
                         SUPE_FAM_SX126X, SUPE_FAM_SX126X, 0, &c)
           && c.sf == 9 && c.bwHz == 62500,
       "budget 0 is the hailing configuration at an unusual bandwidth");

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

/* ─────────────── the conformance vectors (§14.3.4) ─────────────── */

static void writeLadderVectors(const char* path) {
    static const uint32_t kBw[] = { 125000, 250000, 500000 };
    FILE* fp = fopen(path, "w");
    if (!fp) { ok(false, "supe-rate-table-vectors.txt is writable"); return; }
    fprintf(fp,
        "# SUPE ladder conformance vectors (SUPE.md §14.3.4), regenerated by\n"
        "# supe_core_test. An implementation is conformant iff it reproduces this\n"
        "# file exactly. Entries are index:sf/bw/ldro. The ladder is truncated at\n"
        "# 15 entries, the reach of the budget byte's index space.\n");
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
        for (int i = 0; i < n; i++)
            fprintf(fp, " %d:%u/%u/%d", i, lad[i].sf, (unsigned)lad[i].bwHz,
                    lad[i].ldro ? 1 : 0);
        fprintf(fp, "\n");
        lines++;
    }
    fclose(fp);
    printf("wrote %d ladder vectors to %s\n", lines, path);
}

/* ─────────────── the schedule vectors (§7, §14.3.4's discipline) ───────────
 *
 * The derivation is a pure function of the two digests, so the vectors run
 * from fixed digest patterns rather than from seeds — hashing is the host's,
 * not the core's. Word lists per spreading factor ride along, since the slot's
 * sync word is the one derived quantity the slots themselves do not carry. */
static void writeScheduleVectors(const char* path) {
    FILE* fp = fopen(path, "w");
    if (!fp) { ok(false, "supe-schedule-vectors.txt is writable"); return; }
    fprintf(fp,
        "# SUPE schedule conformance vectors (SUPE.md §7), regenerated by\n"
        "# supe_core_test over fixed digest patterns d[i] = seed + 7i. An\n"
        "# implementation is conformant iff it reproduces this file exactly.\n"
        "# Channel plan 1 only: channel plan 0 derives no schedule and its\n"
        "# exchange keeps the interface's sync word (SUPE.md §7, §14.5).\n"
        "# Slots are t:chan:sync(SF-of-slot).\n");
    int lines = 0;
    for (int seed = 0; seed < 8; seed++)
    for (int wide = 0; wide <= 1; wide++) {
        const int nch = 9;
        uint8_t d0[32], d1[32];
        fillDigest(d0, (uint8_t)(0x10 + seed * 13));
        fillDigest(d1, (uint8_t)(0x81 + seed * 13));
        SupeSchedD s;
        supeDeriveSchedule(d0, d1, wide != 0, (uint8_t)nch, &s);
        fprintf(fp, "seed=%d wide=%d nch=%d hash=%02x%02x%02x n=%u",
                seed, wide, nch, s.hash3[0], s.hash3[1], s.hash3[2],
                (unsigned)s.nSlots);
        for (int k = 0; k < s.nSlots; k++)
            fprintf(fp, " %u:%u:0x%02x", (unsigned)s.slot[k].tMs,
                    (unsigned)s.slot[k].chan,
                    supeSyncWordAt(7, 0x42, s.slot[k].sByte));
        fprintf(fp, "\n");
        lines++;
    }
    for (int sf = 5; sf <= 7; sf++) {
        uint8_t w[SUPE_SYNC_WORDS_CAP];
        int n = supeSyncWords((uint8_t)sf, 0x42, w, (int)sizeof w);
        fprintf(fp, "words sf=%d iface=0x42 n=%d", sf, n);
        for (int i = 0; i < n; i++) fprintf(fp, " 0x%02x", w[i]);
        fprintf(fp, "\n");
        lines++;
    }
    fclose(fp);
    printf("wrote %d schedule vectors to %s\n", lines, path);
}

/* ─────────────── main ─────────────── */

int main(int argc, char** argv) {
    testRegimeTables();
    testQuantisation();
    testLevels();
    testTypeBytes();
    testAnnCodec();
    testHailCodec();
    testReadyCodec();
    testGotCodec();
    testEndCodec();
    testByeResendCodec();
    testCrc8();
    testSyncWords();
    testSchedule();
    testExpiry();
    testAirtime();
    testSensitivity();
    testLadder2();

    writeLadderVectors((argc > 2) ? argv[2] : "supe-rate-table-vectors.txt");
    writeScheduleVectors("supe-schedule-vectors.txt");

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
