/**
 * supe — SUPE's pure core: regime tables, the modulation ladder, the frame
 * codec, the schedule derivation and the deadline arithmetic.
 *
 * SUPE (Spectrum Utilization and Performance Enhancements) turns unicast
 * traffic on a shared LoRa channel into meetings, inside the modem, with the
 * Reticulum daemon unmodified and unaware. A HAIL on the shared channel asks;
 * the party it names answers — a turnaround later on the same channel in
 * regime 0, at derived slots on a private channel under a channel plan — and
 * the traffic follows at the best rate the link supports. The protocol is
 * specified in plans/SUPE.md and that document is authoritative for anything
 * on the air.
 *
 * **Nothing in this file touches a radio, ESP-IDF or FreeRTOS.** It is plain
 * arithmetic over plain bytes so it can be compiled and tested on a host — see
 * test/supe_core_test.cpp, which also emits the golden frame vectors every
 * on-device injection test replays. Keep it that way: a dependency here is a
 * dependency the tests cannot satisfy.
 *
 * The five things it answers:
 *
 *   - **What a budget resolves to.** The family-filtered, channel-bound
 *     ladder of SUPE.md §14.3, integer-only, with supe-rate-table-vectors.txt in
 *     test/ as the conformance authority.
 *   - **What a frame looks like.** Encode and decode for every frame, with the
 *     permitted lengths driven by the same tables, so a length outside the
 *     enumerable set is rejected outright (SUPE.md §3).
 *   - **Where the slots are.** The §7 schedule derivation — offsets, channels
 *     and sync words from a seed's digests — as pure integer functions both
 *     ends compute identically. Regime 0 derives none: a hail there is
 *     answered in place.
 *   - **When to give up.** Every §14.7 deadline from regime constants and a
 *     time on air, so both ends derive the identical number with nothing
 *     exchanged.
 *   - **Whether this dialect is still current.** Each regime version carries an
 *     expiry stamped at build time; past it a node neither sends nor accepts
 *     frames naming that regime.
 */
#ifndef IFACE_LORA_SUPE_H
#define IFACE_LORA_SUPE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ─────────────── frame types (SUPE.md §0.3, normative) ───────────────
 *
 * The type byte is 3 bits of protocol (`110`) and 5 bits of type, so every SUPE
 * frame begins 0xC0–0xDF. **A type value never ends in 0 or 1**: an interface
 * may put framing of its own ahead of the Reticulum packet, and this one does —
 * a random 4-bit sequence in the high nibble and a split flag in bit 0, which
 * reaches every byte whose low nibble is 0 or 1 and nothing else. Conceding
 * those four values is what lets a receiver tell a SUPE frame from the framing
 * byte of a node that has never heard of SUPE.
 *
 * Every frame's full name carries the prefix SUPE_; the protocol document,
 * being about nothing else, omits it. */
#define SUPE_TYPE_MIN      0xC0
#define SUPE_TYPE_MAX      0xDF
#define SUPE_T_HAIL        0xC2   /* main channel: traffic waiting, this train, answer me */
#define SUPE_T_ANNOUNCE    0xC3   /* main channel: identities + capabilities */
#define SUPE_T_GOT        0xC4   /* READY with a train behind it */
#define SUPE_T_READY       0xC5   /* the receiver's terms: budget, ceiling, reading */
#define SUPE_T_END     0xC6   /* the train's power + checksums */
#define SUPE_T_BYE         0xC7   /* everything accounted for */
#define SUPE_T_RESEND      0xC8   /* one repair round's bitmask */

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

/* The top of the range doubles as "no reading". +63 dBm is two kilowatts at the
 * antenna and no receiver on earth reports it, so a field that must be present
 * on the wire whether or not there is a measurement behind it says so with this
 * rather than with a zero — 0 dBm is a level a real frame can arrive at. */
#define SUPE_LEVEL_NONE     SUPE_LEVEL_MAX_DBM

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
    SUPE_FAM_LR2021 = 4,   /* listens at several spreading factors at once */
};

/** The lowest spreading factor a family listens at on the hailing channel,
 *  given the hailing one (SUPE.md §14.6). Family 4 runs one main and up to
 *  three side detectors, all at one bandwidth, no more than four apart, so it
 *  listens at the hailing SF and the three below it, never below 5. Every
 *  other family listens at the hailing SF alone. A rule and not a capability
 *  field, because the configuration is this protocol's to make and both ends
 *  compute it from the hailing SF. */
static inline uint8_t supeListenSfLow(uint8_t fam, uint8_t hailSf) {
    if (fam != SUPE_FAM_LR2021) return hailSf;
    return hailSf > 8 ? (uint8_t)(hailSf - 3) : (uint8_t)5;
}

/* ─────────────── regimes ───────────────
 *
 * A regime is the complete set of constants two nodes must hold identically in
 * order to meet at all. Everything an index or a derivation selects lives here,
 * keyed by regime and version, and none of it may be a setting: two neighbours
 * who configured differently would meet at different frequencies, times or sync
 * words and never hear each other (SUPE.md §3).
 *
 * The hailing channel is the exception and stays interface configuration — that
 * channel belongs to the Reticulum network being joined, not to SUPE, which
 * reads those keys and never writes them. */

#define SUPE_REGIME_SINGLE  0    /* Single Channel — a dialogue on the hailing channel */
#define SUPE_REGIME_EU863   1    /* ETSI EN 300 220, 863–870 MHz, nine channels */
/* Not a regime: an announcement stating that this node does NOT speak SUPE, so
 * a neighbour holding the opposite belief drops it and goes back to plain
 * framing. Silence cannot say this — a node that has spoken SUPE and then stops
 * is indistinguishable from one that has gone away, and the neighbour would
 * keep trying to meet it. The regime field is a nibble, so this is its top
 * value rather than a byte-wide sentinel. */
#define SUPE_REGIME_NONE  0x0F
#define SUPE_VERSION        0

/** Channel 0 is the hailing channel in every regime: regime 0 never leaves
 *  it, regime 1's schedules never draw it. */
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
     * figure would be a limit nobody imposed. What still bounds a train there
     * is the length byte's own reach: 1.275 s. */
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

/** The agile channels a regime draws meetings from. Index 0 of the returned
 *  array is channel 1: channel 0 is the hailing channel and is never in a
 *  regime's table, since it takes the operator's frequency and bandwidth. */
const SupeChan* supeRegimeChans(uint8_t regime, int* count);

/** A regime with a channel plan derives schedules; one without answers a hail
 *  in place (SUPE.md §7, §8). */
static inline bool supeRegimeHasPlan(uint8_t regime) {
    int n = 0;
    return supeRegimeChans(regime, &n) != nullptr && n > 0;
}

/* ─────────────── expiry ───────────────
 *
 * Each version of each regime carries an expiry fixed when the software is
 * built. Past that date a node neither sends nor accepts frames naming it and
 * falls back to plain main-channel operation, so an obsolete dialect leaves the
 * air by itself instead of having to be spoken forever.
 *
 * At this stage of development the expiry is a CALENDAR DATE, stated here and
 * moved by hand (SUPE.md §3). A date rather than an offset from the build
 * because what matters is that every node on a channel stops speaking the same
 * dialect at the same moment: with an offset, two nodes flashed a week apart
 * expire a week apart, and the older one spends that week talking to nobody
 * while looking like a radio fault. A date they were both built with retires
 * the dialect on both at once.
 *
 * The cost is that it has to be advanced deliberately, and that a build made
 * after it is born expired — which is the right failure: it is loud, it is
 * immediate, and it says the dialect needs a decision rather than a reflash. */
#define SUPE_EXPIRY_Y  2026
#define SUPE_EXPIRY_M  10
#define SUPE_EXPIRY_D  10

/** Unix seconds of this build, from the compiler's own __DATE__/__TIME__. */
uint32_t supeBuildUnix(void);

/** Unix seconds at which every regime in this build stops being spoken. */
uint32_t supeExpiryUnix(void);

/** Has this build's dialect expired at wall-clock `nowUnix`? A zero or plainly
 *  pre-build clock is treated as *not* expired: an unresolved clock must not
 *  silently disable the protocol, and the node has bigger problems anyway. */
bool supeExpired(uint32_t nowUnix);

/* ─────────────── what a budget resolves to ─────────────── */

struct SupeCfg {
    uint8_t  sf;
    uint32_t bwHz;
    bool     ldro;        /* low-data-rate optimisation — part of what a budget
                           * resolves to, not a local choice: both ends must set
                           * it identically or neither decodes */
    int16_t  marginDeci;  /* margin cost in deci-dB against the hailing
                           * configuration */
};

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
 * fading does not take a meeting out, and cheap because the alternative — the
 * budget below — costs 2.5 dB of rate rather than the link. */
#define SUPE_TARGET_MARGIN_DB  10

/* ─────────────── airtime ───────────────
 *
 * Semtech AN1200.13. The CRC term is a parameter because SUPE's own frames
 * carry none: all a check would buy is the radio rejecting a corrupt frame
 * instead of our own parse rejecting it a moment later, and nothing downstream
 * ever sees these frames. What it costs is a symbol group. The LoRa frames a
 * train carries keep the interface's — same reasoning, different consumer. */
double supeAirtimeSeconds(int sf, int bw_hz, int cr_denom, int preamble,
                          int payload, bool implicitHeader, bool crc);

/* ─────────────── timing constants (SUPE.md §14.7) ─────────────── */
/* The longest a node may take to answer (SUPE.md §14.7). An answer waits the
 * flip below and then rides the platform's path to the air — a one-shot timer,
 * the radio task's wake, a pass of its loop — which at the hailing
 * configuration has been measured at up to 40 ms after the frame answered.
 * The deadlines built on this must not expire on an answer that is merely
 * slow; `lora` stats show the answer latency as made. */
#define SUPE_TURNAROUND_MS   50
/* The least a node leaves between the end of a frame it answers and the start
 * of its answer (SUPE.md §14.7). The node that just transmitted is turning
 * from transmit to receive — tx-done serviced, the chip out of standby, the
 * receiver started, and after a READY retuned to the budget first — and a
 * preamble that starts before it has finished is never heard. At the hailing
 * configuration a preamble is long enough to hide the turn; at the fastest
 * budgets it is two milliseconds, and hides nothing. Every answer waits this:
 * READY after GOT, the train after READY, RESEND or BYE after END, the
 * answer to a hail in regime 0. */
#define SUPE_FLIP_MS         10
#define SUPE_RETUNE_GAP_MS    1  /* the synthesizer, not the software */
/* The interval between the frames of a train (SUPE.md §14.7): the receiving
 * task must service rx-done and read the frame out of the chip before the
 * next one lands in the same buffer. The receiver itself stays open throughout
 * — it is never restarted between frames, since a fresh startReceive begins
 * with standby and would abort the preamble already being demodulated. It is
 * what a send that follows our OWN frame is parked for (a END after the
 * train) and the per-frame pad in a train's budgeted length. Between two
 * frames of one train it is not parked but PAID — the frame is fired from
 * tx-done, and servicing that costs the peer's rx side the same interval to
 * do the same work (see supeEngOnTxDone). */
#define SUPE_TRAIN_GAP_MS     2
#define SUPE_GUARD_MS        10  /* slop on every deadline inside a meeting */

/* A slot's own slop, and it is NOT the deadline guard. Inside a meeting the
 * task is hot — the previous frame's completion is what drives the next step —
 * so 10 ms covers it. A slot is reached from an idle task: a timer fires, the
 * task wakes, retunes, senses, builds. Measured on hardware, that path puts a
 * slot's opening frame 16–20 ms past its nominal moment.
 *
 * The listener's window has to cover that lateness, and it cannot borrow the
 * cover from the preamble, because the preamble shrinks with the budget: 16.6 ms
 * at a hailing SF7/125k, 2.1 ms at SF5/500k. A window sized to the preamble is
 * generous where the schedule is slow and shut before the speaker transmits
 * where it is fast — which is exactly the wide schedule, and exactly why one
 * never carried anything.
 *
 * So the window is sized to the software's slop instead: the listener aims to
 * be listening from SUPE_SLOT_GUARD_MS early (late by its own slop, it is still
 * listening before the speaker transmits) and stays until the speaker's own
 * tolerance has run out. The speaker gives up on a slot at SUPE_SLOT_LATE_MS,
 * which is well inside that tail. */
#define SUPE_SLOT_GUARD_MS   40  /* the listener's window, each edge */
#define SUPE_SLOT_LATE_MS    20  /* the speaker's tolerance for its own slot */

/* The run (SUPE.md §12, §14.7). A hail unanswered — no answer by its deadline
 * in regime 0, both slots unmet under a plan — is followed by an interval in
 * which the hailed party may hail back, then a louder hail; three unanswered
 * and the peer is unreachable. Patience is per packet, from the moment it was
 * queued: a run's three hails, schedules and intervals fit inside one, with
 * the channel-access waits in front of each hail (a second and more behind a
 * busy hailing channel), and it sits well inside the daemon's shortest per-hop
 * timer. */
#define SUPE_HAIL_INTERVAL_MS      300
#define SUPE_HAIL_INTERVAL_JIT_MS  200
#define SUPE_PATIENCE_MS          3000
#define SUPE_RUN_HAILS               3

/* ─────────────── quantised fields ───────────────
 * The length byte is 5 ms steps because its range has to reach the train
 * ceiling. It rounds *up*, so a stated length is never shorter than the thing
 * it describes. A length of 0 is a train of no frames. */
#define SUPE_LEN_STEP_MS    5
#define SUPE_LEN_MAX_MS   (255 * SUPE_LEN_STEP_MS)   /* 1.275 s */

static inline uint8_t supeEncLen(uint32_t ms) {
    if (ms == 0) return 0;
    uint32_t q = (ms + SUPE_LEN_STEP_MS - 1) / SUPE_LEN_STEP_MS;
    if (q < 1)   q = 1;
    if (q > 255) q = 255;
    return (uint8_t)q;
}
static inline uint32_t supeDecLen(uint8_t b) { return (uint32_t)b * SUPE_LEN_STEP_MS; }

/* ─────────────── the frame checksum (SUPE.md §8) ───────────────
 *
 * One byte per train frame, listed by END after the fact: the checksum
 * list IS the sequence, so the frames themselves carry no numbering. CRC-8,
 * polynomial 0x07, init 0, MSB first, over the frame's on-air bytes.
 * Provisional per SUPE.md §16 — it cannot change inside a version. */
uint8_t supeCrc8(const uint8_t* d, size_t n);

/* ─────────────── the sync-word list (SUPE.md §14.5) ───────────────
 *
 * Under a channel plan every meeting flies under a derived word; the list is
 * built per spreading factor from four rules — no zero nibbles (bin 0 is
 * another preamble upchirp), nibbles inside the SF's bin space
 * (nibble × 8 < 2^SF), a two-nibble berth in BOTH symbols around every
 * foreign word (0x12, 0x24, LoRaWAN's 0x34, and the interface's own hailing
 * word), and at SF5 exact exclusions only, since the berth rule would empty
 * its nine-word space. Ordered ascending; indexed by the slot's stream byte.
 * Regime 0 keeps the interface's word throughout. */
#define SUPE_SYNC_WORDS_CAP 226

/** Build the word list for a spreading factor. Returns the count (never 0). */
int supeSyncWords(uint8_t sf, uint8_t ifaceSync, uint8_t* out, int cap);

/** The word a slot's stream byte selects at a spreading factor: W_sf[s mod N].
 *  The meeting's frames re-index the same byte against whatever SF is flying,
 *  so both ends derive the train's word from the slot's byte identically even
 *  where the budget's SF admits a different list than the slot's. */
uint8_t supeSyncWordAt(uint8_t sf, uint8_t ifaceSync, uint8_t sByte);

/* ─────────────── the schedule (SUPE.md §7) ───────────────
 *
 * A pure function of one frame both ends hold, derived only under a channel
 * plan. The caller supplies the two digests — D0 = SHA-256(seed),
 * D1 = SHA-256(seed ‖ 0x01) — because hashing is a host capability;
 * everything after that is integer arithmetic:
 *
 *   hash   = D0[0..2]                     the 3 bytes READY and GOT quote
 *   stream = D0[3..31] ‖ D1[0..31]        slot k consumes stream[3k..3k+2]
 *                                         as j_k, c_k, s_k
 *   t_0    = seed_gap                                      (narrow)
 *          = 150 + (j_0 mod 40)                            (wide)
 *   t_k    = t_(k-1) + 100 + (j_k mod 24)                  (narrow)
 *          = t_(k-1) + min(60 + 30·k, 350) + (j_k mod 40)  (wide)
 *   chan_k = 1 + (c_k mod nChans); narrow k = 1 redrawn off chan_0
 *   slots exist while t_k ≤ horizon (230 narrow, 3000 wide)
 *
 * The narrow constants yield exactly two slots, the second never on the
 * first's channel. Who speaks is fixed by ROLE and is deliberately not
 * derived: on a narrow schedule the hailed party speaks at both and the hailer
 * listens; on a wide one the slots alternate from the side that received the
 * last train. A disagreed seed under fixed roles means empty slots, which the
 * horizon already handles; roles from the stream would mean both parties
 * transmitting at each other with nothing to detect it. */
#define SUPE_SLOTS_MAX          16
#define SUPE_NARROW_HORIZON_MS  230
#define SUPE_WIDE_HORIZON_MS  3000

/* The seed's end to the first slot: the SPEAKER's allowance. The hailed party
 * has just demodulated a frame and must derive a schedule from it, retune to a
 * channel the frame chose, sense, and be speaking; the hailer knew the
 * schedule before its seed had finished flying and is on the channel with its
 * receiver open at no cost to anyone. A slow speaker is heard by a listener
 * that is already there; the reverse arrangement is the one that fails, since
 * a receiver opened part-way through a frame has missed the preamble and hears
 * nothing however strong the signal. */
#define SUPE_NARROW_T0_MS      100

/* Own slots a wide schedule may spend unanswered before it is abandoned for the
 * shared channel. A schedule that meets is freed on the spot, so a live one
 * that has spoken this often has been ignored every time — the far end derived
 * a different schedule, or is gone. Sized to be cheaper than the horizon it
 * replaces and no cheaper than the hail it buys. */
#define SUPE_SCHED_GIVEUP_SPOKE  2

struct SupeSlotD {
    uint16_t tMs;      /* offset from the epoch */
    uint8_t  chan;     /* 1..nChans */
    uint8_t  sByte;    /* the stream byte the sync word derives from */
};

struct SupeSchedD {
    uint8_t   hash3[3];             /* D0's first three bytes — the wire id */
    uint8_t   nSlots;
    SupeSlotD slot[SUPE_SLOTS_MAX];
};

void supeDeriveSchedule(const uint8_t d0[32], const uint8_t d1[32],
                        bool wide, uint8_t nChans, SupeSchedD* out);

/* ─────────────── frames (SUPE.md §0.3) ───────────────
 *
 * Every frame has a length the receiver can enumerate from its type and from
 * state both sides already hold: one value for most, two for HAIL and GOT,
 * count-derived for ANNOUNCE, END, RESEND and the answering GOT. Nothing
 * is signalled by a flags byte and nothing is negotiated, so anything outside
 * the enumerated set is discarded. */

#define SUPE_TAG_LEN       3     /* first three bytes of an address the peer holds */
#define SUPE_HASH_LEN      3     /* first three bytes of a hail's SHA-256 */
#define SUPE_ID_LEN        4     /* first four bytes of an identity hash */
#define SUPE_ANN_BASE      5
/* One LoRa frame caps at 255 bytes, and a SUPE frame carries no split header,
 * so that is the whole budget. */
#define SUPE_MAX_FRAME   255
#define SUPE_ANN_MAX     ((SUPE_MAX_FRAME - SUPE_ANN_BASE) / SUPE_ID_LEN)   /* 62 */

/* One train's frame cap. A train is a RAM commitment on the receiving side
 * (SUPE.md §8): delivery is whole and in sequence at the close, so this bounds
 * the buffer as well as the END and the bitmask. It is also the count
 * ceiling this node states in every READY and GOT. */
#define SUPE_TRAIN_MAX   12
#define SUPE_MASK_MAX    ((SUPE_TRAIN_MAX + 7) / 8)

static inline uint8_t supeMaskLen(uint8_t count) { return (uint8_t)((count + 7) / 8); }

/** Capabilities — two bytes, carried by ANNOUNCE only. The top bit of the
 *  maximum-power byte is free — a transmit power never stores a negative
 *  value — and currently unassigned; it must never be spent on a level. */
struct SupeCaps {
    uint8_t fam;
    uint8_t topStep;
    int8_t  maxPwrDbm;
};

struct SupeAnn {
    uint8_t  regime, version;
    SupeCaps caps;
    int8_t   pwrDbm;                    /* what this frame went out at */
    uint8_t  count;
    uint8_t  ids[SUPE_ANN_MAX][SUPE_ID_LEN];
};

/* HAIL — "I have this many frames for whoever holds this address, and this
 * is the fastest I propose to send them. Answer me; if you cannot, hail me
 * when you can." Ten bytes, thirteen naming its sender. */
struct SupeHail {
    uint8_t regime, version;
    uint8_t tag[SUPE_TAG_LEN];
    int8_t  pwrDbm;                     /* what this frame went out at */
    uint8_t salt;                       /* random — the seed's freshness where a
                                         * schedule is derived from it. Without it
                                         * two identical requests hash identically
                                         * and derive the SAME schedule. Regime 0
                                         * derives nothing and carries it so the
                                         * frame has one layout */
    uint8_t budgetCeil;                 /* the highest budget the hailer proposes */
    uint8_t count;                      /* LoRa frames it holds for this tag; 0
                                         * for a hail-back */
    uint8_t lenByte;                    /* that train's airtime at the ceiling */
    bool    haveIdent;                  /* implicit in the frame length */
    uint8_t ident[SUPE_TAG_LEN];        /* one of the sender's announced identities */
};

/* READY — the receiver's terms: "heard you; fly at this budget, no more than
 * this many frames; here is how the frame I am answering reached me."
 * Always nine bytes. GOT is READY with a train behind it — the same fields,
 * then a count and a length of its sender's own; answering a END it adds
 * the repair bitmask over the peer's train. */
struct SupeReady {
    uint8_t hash[SUPE_HASH_LEN];        /* which hail this answers */
    int8_t  pwrDbm;                     /* this side's power until it states another */
    uint8_t budget;                     /* confirmed (READY); proposed (opening GOT) */
    uint8_t countCeil;                  /* the most frames this side will hold */
    int16_t heardRssi;                  /* the frame this answers: the hail, an
                                         * opening GOT, the train's worst frame */
    int8_t  heardSnrQ;
};

struct SupeGot {
    SupeReady g;
    uint8_t count;                      /* LoRa frames in this side's own train */
    uint8_t lenByte;                    /* its airtime + flip gaps at g.budget */
    bool    answering;                  /* implicit in the frame length */
    uint8_t maskLen;                    /* answering: over the peer's train */
    uint8_t mask[SUPE_MASK_MAX];        /* bit i set: frame i is missing */
};

/* END — "that was my train, here is its checksum list, and here is how your
 * last frame reached me." The reading is what makes the exchange teach BOTH
 * ends: READY and GOT quote a level back to whoever opened the leg, so without
 * this the answering party would learn only the direction it receives in, for
 * ever (§15.2). Absent readings ride as SUPE_LEVEL_NONE. */
struct SupeEnd {
    int8_t  pwrDbm;                     /* what the train it closes went out at */
    uint8_t salt;                       /* random — this goodbye's freshness (§7) */
    bool    haveHeard;                  /* the reading below is one */
    int16_t heardRssi;                  /* the peer's last frame, as it reached us */
    int8_t  heardSnrQ;
    uint8_t count;                      /* implicit in the frame length */
    uint8_t csum[SUPE_TRAIN_MAX];
};

struct SupeResendF {
    uint8_t maskLen;                    /* caller-known: ceil(peer count / 8) */
    uint8_t mask[SUPE_MASK_MAX];
};

#define SUPE_HAIL_LEN         10
#define SUPE_HAIL_ID_LEN      13
#define SUPE_READY_LEN         9
#define SUPE_GOT_LEN         11
#define SUPE_GOT_ANS_BASE    11        /* + maskLen */
#define SUPE_END_BASE      5        /* + count */
#define SUPE_BYE_LEN           1
#define SUPE_RESEND_BASE       1        /* + maskLen */

/* Encoders return the byte count written, or 0 if the frame cannot be built
 * (a count past a cap, a buffer too small). Decoders return false on any
 * length, range or type mismatch and leave `out` untouched. Where a frame's
 * length depends on state both sides hold — the peer train's count for the
 * answering GOT and for RESEND — the decoder takes it as an argument. */
size_t supeEncAnn(uint8_t* out, size_t cap, const SupeAnn* a);
bool   supeDecAnn(const uint8_t* f, size_t len, SupeAnn* out);
size_t supeEncHail(uint8_t* out, size_t cap, const SupeHail* h);
bool   supeDecHail(const uint8_t* f, size_t len, SupeHail* out);
size_t supeEncReady(uint8_t* out, size_t cap, const SupeReady* g);
bool   supeDecReady(const uint8_t* f, size_t len, SupeReady* out);
size_t supeEncGot(uint8_t* out, size_t cap, const SupeGot* h);
bool   supeDecGot(const uint8_t* f, size_t len, uint8_t peerCount, SupeGot* out);
size_t supeEncEnd(uint8_t* out, size_t cap, const SupeEnd* t);
bool   supeDecEnd(const uint8_t* f, size_t len, SupeEnd* out);
size_t supeEncResend(uint8_t* out, size_t cap, const SupeResendF* m);
bool   supeDecResend(const uint8_t* f, size_t len, uint8_t peerCount,
                     SupeResendF* out);

/* ─────────────── the ladder (SUPE.md §14.3) ───────────────
 *
 * Membership is family-filtered and channel-bound, so the *index space itself*
 * depends on both families and on the slot channel's maximum bandwidth — all
 * of which both ends hold. Ordering is the integer key `(bw × sf) >> sf`
 * ascending, ties toward the narrower bandwidth then the higher spreading
 * factor. Index 0 is always the hailing configuration.
 *
 * Conformance is `supe-rate-table-vectors.txt` (test/), generated over the full
 * §14.3.4 cross-product; the file is the authority where it and a reading of
 * the prose disagree. */

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

#endif /* IFACE_LORA_SUPE_H */
