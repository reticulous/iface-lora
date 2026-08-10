/**
 * supe — SUPE's pure core: regime tables, the modulation ladder, the frame
 * codec and the deadline arithmetic.
 *
 * SUPE (Spectrum Utilization and Performance Enhancements) moves unicast
 * traffic off the shared LoRa channel onto short private high-rate detours,
 * inside the modem, with the Reticulum daemon unmodified and unaware. The
 * protocol is specified in plans/SUPE.md and that document is authoritative for
 * anything on the air; plans/SUPE-in-reticulous.md maps it onto this straddle.
 *
 * **Nothing in this file touches a radio, ESP-IDF or FreeRTOS.** It is plain
 * arithmetic over plain bytes so it can be compiled and tested on a host — see
 * test/supe_core_test.cpp, which also emits the golden frame vectors every
 * on-device injection test replays. Keep it that way: a dependency here is a
 * dependency the tests cannot satisfy.
 *
 * The four things it answers:
 *
 *   - **What a budget resolves to.** The family-filtered, channel-bound
 *     ladder of SUPE.md §14.3, integer-only, with supe-ladder-vectors.txt in
 *     test/ as the conformance authority — the same index means a different
 *     modulation on a different network, and nothing about the baseline is
 *     ever transmitted.
 *   - **What a frame looks like.** Encode and decode for every frame, with the
 *     permitted lengths driven by the same tables, so a length outside the
 *     enumerable set is rejected outright (SUPE.md §3).
 *   - **When to give up.** Every §14.7 deadline — the GRANT after a START,
 *     each MANIFEST, a stated train length — from regime constants and a time
 *     on air, so both ends derive the identical number with nothing exchanged.
 *   - **Whether this dialect is still current.** Each regime version carries an
 *     expiry stamped at build time; past it a node neither sends nor accepts
 *     frames naming that regime.
 */
#ifndef IFACE_LORA_SUPE_H
#define IFACE_LORA_SUPE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ─────────────── frame types (SUPE.md §0.1, normative) ───────────────
 *
 * The type byte is 3 bits of protocol (`110`) and 5 bits of type, so every SUPE
 * frame begins 0xC0–0xDF. **A type value never ends in 0 or 1**: an interface
 * may put framing of its own ahead of the Reticulum packet, and this one does —
 * a random 4-bit sequence in the high nibble and a split flag in bit 0, which
 * reaches every byte whose low nibble is 0 or 1 and nothing else. Conceding
 * those four values is what lets a receiver tell a SUPE frame from the framing
 * byte of a node that has never heard of SUPE. */
#define SUPE_TYPE_MIN      0xC0
#define SUPE_TYPE_MAX      0xDF
#define SUPE_T_START       0xC2
#define SUPE_T_ANNOUNCE2   0xC4
#define SUPE_T_MANIFEST    0xC9
/* 0xC3 was SUPE_ANNOUNCE1 and 0xC8 was HERE. Both frames are gone; both
 * values stay reserved and are never reassigned. */

/** True for a byte the split framing cannot produce — the receive path's whole
 *  basis for telling a SUPE frame from a packet's first byte. */
static inline bool supeIsFramingByte(uint8_t b) { return (b & 0x0F) <= 0x01; }

/** True iff `b` is a SUPE type byte: in range, and not one the framing reaches. */
static inline bool supeIsTypeByte(uint8_t b) {
    return b >= SUPE_TYPE_MIN && b <= SUPE_TYPE_MAX && !supeIsFramingByte(b);
}

/* ─────────────── level and power encoding ───────────────
 *
 * Every field carrying a transmit power or a received level is `dBm + 64` read
 * as a signed byte, so the range is −192 … +63 dBm at 1 dB. The offset buys the
 * bottom: a plain signed byte stops at −128 dBm while receivers already report
 * below −130. Nothing needs the top — +63 dBm is two kilowatts.
 *
 * Signal-to-noise is a separate signed byte in quarter-decibels, covering
 * −32 … +31.75 dB. It is a ratio rather than a level, needs no offset, and that
 * is what the silicon reports natively. */
#define SUPE_LEVEL_OFFSET   64
#define SUPE_LEVEL_MIN_DBM  (-192)
#define SUPE_LEVEL_MAX_DBM  (63)

static inline uint8_t supeEncLevel(int dbm) {
    if (dbm < SUPE_LEVEL_MIN_DBM) dbm = SUPE_LEVEL_MIN_DBM;
    if (dbm > SUPE_LEVEL_MAX_DBM) dbm = SUPE_LEVEL_MAX_DBM;
    return (uint8_t)(int8_t)(dbm + SUPE_LEVEL_OFFSET);
}
static inline int16_t supeDecLevel(uint8_t b) {
    return (int16_t)((int)(int8_t)b - SUPE_LEVEL_OFFSET);
}
static inline int8_t supeEncSnrQ(int snr10) {          /* deci-dB in, quarter-dB out */
    int q = (snr10 * 2) / 5;                            /* ×(1/10 dB)×4 = ×2/5 */
    if (q < -128) q = -128;
    if (q > 127)  q = 127;
    return (int8_t)q;
}
static inline int16_t supeDecSnr10(int8_t q) { return (int16_t)((int)q * 5 / 2); }

/* ─────────────── radio families (SUPE.md §14.6) ───────────────
 * The nibble names what a peer's silicon can do in the few respects that change
 * what goes on the air. Values match iface-lora's own LoraFamily enum. */
enum SupeFamily : uint8_t {
    SUPE_FAM_SX126X = 0,
    SUPE_FAM_SX127X = 1,   /* no SF5 at all, and SF6 wants an implicit header */
    SUPE_FAM_SX128X = 2,
    SUPE_FAM_LR11X0 = 3,
    SUPE_FAM_LR2021 = 4,
};

/* ─────────────── regimes ───────────────
 *
 * A regime is the complete set of constants two nodes must hold identically in
 * order to meet at all. Everything an index on the wire selects lives here,
 * keyed by regime and version, and none of it may be a setting: two neighbours
 * who configured differently would meet at different frequencies, steps or sync
 * words and never hear each other (SUPE.md §3).
 *
 * The hailing channel is the exception and stays interface configuration — that
 * channel belongs to the Reticulum network being joined, not to SUPE, which
 * reads those keys and never writes them. */

#define SUPE_REGIME_SINGLE  0    /* Single Channel — the SF ladder, nowhere else to go */
#define SUPE_REGIME_EU863   1    /* ETSI EN 300 220, 863–870 MHz, nine channels */
#define SUPE_VERSION        0

/** Channel 0 is the hailing channel in every regime and is flagged never-leave:
 *  no regime may direct a detour onto it. A probe names it when it changes
 *  nothing. */
#define SUPE_CH_HAIL        0
#define SUPE_CH_MAX         10   /* hailing channel + the largest regime's agile set */

struct SupeChan {
    uint32_t freqHz;
    uint32_t bwHz;
};

struct SupeRegime {
    uint8_t  regime, version;
    const char* name;
    const SupeChan* chans;       /* agile channels, index 1.. ; null for regime 0 */
    uint8_t  nChans;
    bool     hailBwOnly;         /* the ladder may not change bandwidth */
    /* Ceilings. 0 means the regime states none of its own — regime 0 has no band
     * plan and therefore no regulatory basis to draw one from, and a fabricated
     * figure would be a limit nobody imposed. What still bounds a transaction
     * there is the field widths: 5.1 s of duration byte, 1.275 s of length byte. */
    uint32_t trainCeilMs;
    uint32_t txnCeilMs;
    /* Airtime as a budget and a window rather than a percentage — that pair is
     * what travels. European polite spectrum access is 100/3600, a European duty
     * cycle 360/3600, North American frequency-hopping dwell 0.4/20: three
     * regulatory shapes, one field pair, no special case downstream. 0 = none. */
    uint32_t airtimeMaxMs;
    uint32_t airtimeWinMs;
    uint32_t reuseGapMs;         /* minimum gap before returning to a frequency */
    int16_t  ccaDbm125, ccaDbm500;  /* clear-channel threshold per bandwidth */
    uint16_t ccaListenUs;        /* minimum listen before transmitting */
    uint16_t ccaDeferUs;         /* obligatory deferral after a busy reading */
    uint16_t ccaDeadMs;          /* maximum gap from a clear reading to transmitting */
    int8_t   maxTxpDbm;          /* SUPE_TXP_IFACE = whatever the interface is set to */
};

#define SUPE_TXP_IFACE  ((int8_t)0x7F)

/** The regime table for a number, or null when this firmware does not recognise
 *  it — which resolves to no agile channels, the safe reading of a value it
 *  cannot understand. */
const SupeRegime* supeRegime(uint8_t regime);

/** The agile channels a regime draws detours from. Index 0 of the returned
 *  array is channel 1: channel 0 is the hailing channel and is never in a
 *  regime's table, since it takes the operator's frequency and bandwidth. */
const SupeChan* supeRegimeChans(uint8_t regime, int* count);

/* ─────────────── expiry ───────────────
 *
 * Each version of each regime carries an expiry fixed when the software is
 * built. Past that date a node neither sends nor accepts frames naming it and
 * falls back to plain main-channel operation, so an obsolete dialect leaves the
 * air by itself instead of having to be spoken forever.
 *
 * At this stage of development no build may set an expiry more than fourteen
 * days ahead of its own build date (SUPE.md §3). The offset is a table constant
 * and the date is the build's own timestamp, so neither is ever hand-maintained
 * and a node that has gone quiet by expiry says so rather than looking like a
 * radio fault. */
#define SUPE_EXPIRY_DAYS  14

/** Unix seconds of this build, from the compiler's own __DATE__/__TIME__. */
uint32_t supeBuildUnix(void);

/** Unix seconds at which every regime in this build stops being spoken. */
uint32_t supeExpiryUnix(void);

/** Has this build's dialect expired at wall-clock `nowUnix`? A zero or plainly
 *  pre-build clock is treated as *not* expired: an unresolved clock must not
 *  silently disable the protocol, and the node has bigger problems anyway. */
bool supeExpired(uint32_t nowUnix);

/* ─────────────── what a budget resolves to ───────────────
 *
 * What a budget resolves to in absolute terms is what governs the radio, so
 * the sync word, the header mode and the family limits all follow the
 * *resulting* configuration rather than the index. The ladder itself —
 * membership, ordering, resolution — is further down, with the rest of the
 * revised protocol. */

struct SupeCfg {
    uint8_t  sf;
    uint32_t bwHz;
    bool     ldro;        /* low-data-rate optimisation — part of what a budget
                           * resolves to, not a local choice: both ends must set
                           * it identically or neither decodes */
    int16_t  marginDeci;  /* margin cost in deci-dB against the hailing
                           * configuration */
};

/* The sync words a detour takes (SUPE.md §14.5): `0x67` sits 40 bins from
 * `0x12` and 24 from LoRaWAN's `0x34`; a budget landing on SF5 takes `0x21`,
 * the best of the nine words SF5's 32 bins admit — `0x42` is out of range
 * there entirely, its leading nibble wanting bin 32 of 32. Which one a grant
 * takes is supeSyncWordFor, below. */
#define SUPE_SYNC_UNICAST  0x67
#define SUPE_SYNC_SF5      0x21

/** The demodulator's required signal-to-noise for a spreading factor,
 *  deci-dB — Semtech's figures, 2.5 dB per factor: SF5 −2.5 through SF12
 *  −20.0. What a measured SNR is judged against when choosing a budget. */
int16_t supeReqSnrDeci(uint8_t sf);

/** Receiver sensitivity in deci-dBm for a configuration: thermal noise in the
 *  occupied bandwidth, the receiver's noise figure, and the demodulator's
 *  required signal-to-noise for the spreading factor. */
int16_t supeSensitivityDeci(const SupeCfg* c);

/* The margin a link keeps after paying for a budget. Ten decibels is two step
 * widths of the fastest part of the ladder plus change: enough that ordinary
 * fading does not take a detour out, and cheap because the alternative — the
 * budget below — costs 2.5 dB of rate rather than the link. */
#define SUPE_TARGET_MARGIN_DB  10

/* ─────────────── airtime ───────────────
 *
 * Semtech AN1200.13. The CRC term is a parameter because SUPE's own frames
 * carry none: all a check would buy is the radio rejecting a corrupt frame
 * instead of our own parse rejecting it a moment later, and nothing downstream
 * ever sees these frames. What it costs is a symbol group. The Reticulum
 * packets a train carries keep theirs — same reasoning, different consumer. */
double supeAirtimeSeconds(int sf, int bw_hz, int cr_denom, int preamble,
                          int payload, bool implicitHeader, bool crc);

/* ─────────────── deadlines ───────────────
 *
 * One failure path, the same for both sides: go back to the main channel.
 * Every deadline follows from regime constants and a time on air, so both
 * sides know them without exchanging anything; the derived forms live with
 * the rest of the revised protocol below.
 *
 * The turnaround is the one term that is not arithmetic, and it is a regime
 * constant rather than a per-device figure precisely because both ends must
 * derive the same number. It is generous because it is free — it appears only
 * inside deadlines, and nothing waits out a turnaround that has already been
 * satisfied. Provisional, pending the measurement SUPE.md §16 asks
 * simulation for. */
#define SUPE_TURNAROUND_MS   25
/* Slop on every derived deadline: tick quantisation at each end plus the
 * preamble the receiver must still detect. */
#define SUPE_GUARD_MS        10

/* ─────────────── quantised fields ───────────────
 * The duration byte is 20 ms steps because its range has to reach the
 * transaction ceiling; the length byte is 5 ms steps because its range has to
 * reach the train ceiling. Both encodings round *up*, so a stated duration is
 * never shorter than the thing it describes. */
#define SUPE_DUR_STEP_MS   20
#define SUPE_LEN_STEP_MS    5
#define SUPE_DUR_MAX_MS   (255 * SUPE_DUR_STEP_MS)   /* 5.1 s */
#define SUPE_LEN_MAX_MS   (255 * SUPE_LEN_STEP_MS)   /* 1.275 s */

static inline uint8_t supeEncDur(uint32_t ms) {
    uint32_t q = (ms + SUPE_DUR_STEP_MS - 1) / SUPE_DUR_STEP_MS;
    if (q < 1)   q = 1;
    if (q > 255) q = 255;
    return (uint8_t)q;
}
static inline uint32_t supeDecDur(uint8_t b) { return (uint32_t)b * SUPE_DUR_STEP_MS; }
static inline uint8_t supeEncLen(uint32_t ms) {
    uint32_t q = (ms + SUPE_LEN_STEP_MS - 1) / SUPE_LEN_STEP_MS;
    if (q < 1)   q = 1;
    if (q > 255) q = 255;
    return (uint8_t)q;
}
static inline uint32_t supeDecLen(uint8_t b) { return (uint32_t)b * SUPE_LEN_STEP_MS; }

/* ─────────────── frames ───────────────
 *
 * Every frame has a length the receiver can enumerate from its regime, version
 * and type alone: one value for most, two for START depending on whether the
 * sender names itself, and a count-derived length for ANNOUNCE2. Nothing is
 * signalled by a flags byte and nothing is negotiated, so anything outside the
 * enumerated set is discarded — the test rejects outright rather than merely
 * suspecting, and it is the last cheap filter before we act on anything. */

#define SUPE_TAG_LEN       3     /* first three bytes of a packet's first address */
#define SUPE_HASH_LEN      3     /* first three bytes of a START's own SHA-256 */
#define SUPE_ID_LEN        4     /* first four bytes of an identity hash */
#define SUPE_ANN2_BASE     5
/* One LoRa frame caps at 255 bytes, and a SUPE frame carries no split header,
 * so that is the whole budget. The bundling count follows from it and from
 * nothing else — it is far beyond any real identity count, and if it ever binds
 * the surplus simply waits for the next beat, which is a sender-local choice
 * needing no agreement. */
#define SUPE_MAX_FRAME   255
#define SUPE_ANN2_MAX    ((SUPE_MAX_FRAME - SUPE_ANN2_BASE) / SUPE_ID_LEN)   /* 62 */

/** Capabilities — two bytes, carried by ANNOUNCE2 and MANIFEST and never by a
 *  START. The adaptive-power flag rides in the top bit of the *maximum power*
 *  byte only: free there because a transmit power never stores a negative
 *  value, where a received level routinely does. Do not generalise the trick. */
struct SupeCaps {
    uint8_t fam;
    uint8_t topStep;
    int8_t  maxPwrDbm;
    bool    adaptive;
};

struct SupeAnn2 {
    uint8_t  regime, version;
    SupeCaps caps;
    int8_t   pwrDbm;                    /* what this frame went out at */
    uint8_t  count;
    uint8_t  ids[SUPE_ANN2_MAX][SUPE_ID_LEN];
};

/* Encoders return the byte count written, or 0 if the frame cannot be built
 * (a count past the bundling cap, a buffer too small). Decoders return false on
 * any length, range or type mismatch and leave `out` untouched. */
size_t supeEncAnn2(uint8_t* out, size_t cap, const SupeAnn2* a);
bool   supeDecAnn2(const uint8_t* f, size_t len, SupeAnn2* out);

/* ═══════════════ the wire protocol ═══════════════
 *
 * The peer chooses the channel and the budget (SUPE_GRANT, 0xC5) and declares
 * with the GRANT's reverse flag whether a reverse MANIFEST will exist at all;
 * the START carries a byte load instead of a duration; a MANIFEST precedes
 * each train that exists, and nothing is waited for anywhere when things go
 * right. The `2` suffixes on the START and MANIFEST types name the current
 * layouts against earlier ones that shared their type bytes. */

#define SUPE_T_GRANT       0xC5
/* 0xC8 was HERE. The revised protocol's answer (SUPE_GRANT) lives on the main
 * channel; the value is burned and never reassigned. */

#define SUPE_START2_LEN     7
#define SUPE_START2_ID_LEN 10
#define SUPE_GRANT_LEN     10
#define SUPE_MANIFEST2_LEN 11

/* ─────────────── the load (SUPE.md §6) ───────────────
 *
 * Stated in bytes, not milliseconds: the requester does not know the
 * modulation — the peer is about to choose it. What it counts is
 * `ceil( Σ(bytes + 16) / 32 )` over the packets queued for the peer,
 * saturating at 255 and therefore reaching 8160 bytes. The sixteen bytes
 * charged per packet stand in for its preamble and header — a figure both ends
 * must charge identically or they will size the same queue differently. */
#define SUPE_LOAD_PKT_OVERHEAD 16
#define SUPE_LOAD_UNIT         32

/** Encode a load. `adjustedBytes` is Σ(bytes + 16) over the queued packets. */
static inline uint8_t supeEncLoad(uint32_t adjustedBytes) {
    uint32_t u = (adjustedBytes + SUPE_LOAD_UNIT - 1) / SUPE_LOAD_UNIT;
    return (uint8_t)(u > 255 ? 255 : u);
}
static inline uint32_t supeDecLoadBytes(uint8_t units) {
    return (uint32_t)units * SUPE_LOAD_UNIT;
}

/** Approximate time on air of a load at a resolved configuration, for the
 *  duration the GRANT announces. The per-packet overhead is already folded
 *  into the load's bytes, so the whole thing is billed at the configuration's
 *  byte rate: data bits per symbol are `(SF − 2·DE) · 4 / cr_denom`. Integer
 *  arithmetic; rounds up. */
uint32_t supeLoadAirtimeMs(uint8_t loadUnits, const SupeCfg* c, int crDenom);

/* ─────────────── refusal (SUPE.md §6, §16) ───────────────
 *
 * Budget 15 in a GRANT means refused, and the channel nibble then carries the
 * reason. The spec leaves the reason set open; these five are this
 * implementation's, chosen so each maps onto a distinct backoff. */
#define SUPE_BUDGET_REFUSED 15
enum SupeRefusal : uint8_t {
    SUPE_REFUSE_BUSY     = 0,   /* busy this second — retry soon */
    SUPE_REFUSE_NO_QUIET = 1,   /* no channel currently quiet — retry after a beat */
    SUPE_REFUSE_AIRTIME  = 2,   /* out of airtime on every channel — back off long */
    SUPE_REFUSE_REGIME   = 3,   /* the regime named is not one this node runs */
    SUPE_REFUSE_CEILING  = 4,   /* the ceiling asked for is beyond this node */
};

/* ─────────────── the ladder, revised (SUPE.md §14.3) ───────────────
 *
 * Membership is family-filtered and channel-bound, so the *index space itself*
 * depends on both families and on the named channel's maximum bandwidth — all
 * of which both ends hold (family from the START and from the peer's
 * announcement, channel from the GRANT's own byte). Ordering is the integer
 * key `(bw × sf) >> sf` ascending, ties toward the narrower bandwidth then the
 * higher spreading factor. Index 0 is always the hailing configuration.
 *
 * Conformance is `supe-ladder-vectors.txt` (test/), generated over the full
 * §14.3.4 cross-product; the file is the authority where it and a reading of
 * the prose disagree. */

/* The budget nibble reaches 14 (15 is the refusal), so entries past index 14
 * are unaddressable and the ladder is truncated there. */
#define SUPE_LADDER_MAX_ENTRIES 15

struct SupeLadderEntry {
    uint8_t  sf;
    uint32_t bwHz;
    bool     ldro;
    int16_t  marginDeci;   /* margin cost against the hailing configuration */
};

/** Build the ladder. Returns the entry count (0 on an unknown regime/version
 *  or an impossible hailing configuration); out[0] is the hailing entry. */
int supeLadder(uint8_t regime, uint8_t version,
               uint8_t hailSf, uint32_t hailBwHz, uint32_t chanMaxBwHz,
               uint8_t famA, uint8_t famB,
               SupeLadderEntry* out, int cap);

/** Resolve one budget index against that ladder. False for an index the
 *  ladder does not reach (which discards the frame carrying it). */
bool supeResolveBudget(uint8_t regime, uint8_t version,
                       uint8_t hailSf, uint32_t hailBwHz, uint32_t chanMaxBwHz,
                       uint8_t famA, uint8_t famB, uint8_t budget, SupeCfg* out);

/** The sync word a granted (channel, budget) takes (SUPE.md §14.5): regime 0's
 *  budget 0 is the hailing configuration on the hailing channel and keeps the
 *  interface's word; every other grant is a detour — 0x67, or 0x21 at SF5 —
 *  even at budget 0 on a regime-1 channel, where the frequency moved. */
uint8_t supeSyncWordFor(uint8_t regime, const SupeCfg* c, uint8_t budget,
                        uint8_t ifaceSync);

/* ─────────────── deadlines, revised (SUPE.md §14.7) ───────────────
 *
 * Two constants and a time on air; all derived, none transmitted.
 *
 *   waiting for GRANT      armed at end of START     turnaround + guard, then watch
 *                          — two stages. The first expires when the GRANT must
 *                          have BEGUN, and asks the receiver whether a frame is
 *                          arriving. Something on the air is the answer being
 *                          delivered, so the second stage waits it out
 *                          (toa(GRANT, hailing) + guard). Nothing on the air at
 *                          the moment the peer should have been transmitting is
 *                          silence established half a frame earlier than waiting
 *                          the airtime out would establish it, and against
 *                          evidence rather than against an estimate.
 *   first MANIFEST         armed at end of GRANT     retune + turnaround + toa(MANIFEST, budget) + guard
 *   a train                armed at its MANIFEST     the stated length + guard
 *   reverse MANIFEST       armed at end of own train turnaround + toa(MANIFEST, budget) + guard
 *                          — armed only when the GRANT's reverse flag declared
 *                          one; otherwise the requester's last frame ends the
 *                          transaction and nothing is armed at all
 */
#define SUPE_RETUNE_GAP_MS  1    /* the synthesizer, not the software (§14.7) */

/* When the GRANT must have started: the peer's turnaround and the slop. No time
 * on air in it — that is the point, since what it gates is a look at the
 * receiver rather than a decision about a frame. */
static inline uint32_t supeGrantStartDeadlineMs(void) {
    return SUPE_TURNAROUND_MS + SUPE_GUARD_MS;
}
/* And when a GRANT already on the air must have finished. */
uint32_t supeGrantDeadlineMs(uint8_t hailSf, uint32_t hailBwHz,
                             int crDenom, int preamble);
uint32_t supeManifestFirstDeadlineMs(const SupeCfg* c, int crDenom, int preamble);
uint32_t supeManifestReverseDeadlineMs(const SupeCfg* c, int crDenom, int preamble);
static inline uint32_t supeLenDeadlineMs(uint8_t lenByte) {
    return supeDecLen(lenByte) + SUPE_GUARD_MS;
}

/* ─────────────── revised frames (SUPE.md §0.1) ─────────────── */

struct SupeStart2 {
    uint8_t regime, version;
    uint8_t tag[SUPE_TAG_LEN];
    uint8_t fam;                        /* this node's radio family (§14.6) */
    uint8_t ceiling;                    /* highest budget it will accept */
    uint8_t load;                       /* 32-byte units, supeEncLoad */
    bool    haveIdent;                  /* implicit in the frame length */
    uint8_t ident[SUPE_TAG_LEN];
};

struct SupeGrant {
    uint8_t regime, version;            /* the DETOUR's — may be lower than the
                                         * START's (§6), never higher */
    uint8_t chan;                       /* refusal reason when budget is 15 */
    uint8_t budget;
    uint8_t durByte;                    /* whole transaction, 20 ms steps */
    bool    reverse;                    /* the answerer has traffic queued for
                                         * the requester: a reverse MANIFEST
                                         * will follow the requester's train.
                                         * Clear means both sides go home the
                                         * moment the train is done — no close
                                         * frame, nothing waited on. Rides the
                                         * power byte's top bit, free there
                                         * because a transmit power never
                                         * stores a negative value (the same
                                         * bit the capabilities byte uses). */
    int8_t  pwrDbm;                     /* what this frame went out at */
    int16_t rssiDbm;                    /* what the START was heard at */
    int8_t  snrQ;                       /* … in quarter-dB */
    uint8_t hash[SUPE_HASH_LEN];        /* SHA-256 prefix of the START verbatim */
};

static inline bool supeGrantRefused(const SupeGrant* g) {
    return g->budget == SUPE_BUDGET_REFUSED;
}

struct SupeManifest2 {
    int8_t   pwrDbm;                    /* this train's power */
    int16_t  rssiDbm;                   /* the peer's last frame as heard */
    int8_t   snrQ;
    SupeCaps caps;                      /* the sender's, unconditionally */
    uint8_t  count;                     /* 0 is meaningful (§8) */
    uint8_t  lenByte;                   /* the train's airtime plus its flip
                                         * gaps, 5 ms steps; 0 with count 0 */
    uint8_t  hash[SUPE_HASH_LEN];       /* the GRANT's, returned unchanged */
};

/** The revised length rule: one length for GRANT and MANIFEST, two for START,
 *  count-derived for ANNOUNCE2. */
bool supeLenOk2(uint8_t type, uint8_t regime, uint8_t version, size_t len);

size_t supeEncStart2(uint8_t* out, size_t cap, const SupeStart2* s);
bool   supeDecStart2(const uint8_t* f, size_t len, SupeStart2* out);
size_t supeEncGrant(uint8_t* out, size_t cap, const SupeGrant* g);
bool   supeDecGrant(const uint8_t* f, size_t len, SupeGrant* out);
size_t supeEncManifest2(uint8_t* out, size_t cap, const SupeManifest2* m);
bool   supeDecManifest2(const uint8_t* f, size_t len, SupeManifest2* out);

#endif /* IFACE_LORA_SUPE_H */
