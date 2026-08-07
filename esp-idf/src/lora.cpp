/**
 * lora — LoRa interface task (one or more radios).
 *
 * Drives up to CONFIG_LORA_COUNT LoRa modems off one shared SPI bus. Pins
 * and per-radio type come from Kconfig (LORA_* / LORAn_*), set by the
 * board's sdkconfig.defaults, `spangap menuconfig`, or `spangap build
 * --loraN-*` switches — this component no longer reaches into the board's
 * header for them. Each radio registers with rnsd as its own interface
 * named lora/<slot> (lora/0, lora/1, ...).
 *
 * Drives any RadioLib LoRa chip via RadioLib + EspIdfHal: the SX126x family
 * (SX1261/2/8, LLCC68), SX127x (SX1272/6/7/8, a.k.a. RFM9x), SX128x (2.4 GHz),
 * LR11x0 (LR1110/20/21) and LR2021. Per-radio chip + pins come from Kconfig; the
 * task loop is chip-agnostic — it holds RadioLib's PhysicalLayer base and only
 * construction + begin() dispatch per chip (see the chip-dispatch section). A
 * single task services every radio: each radio's IRQ line notifies the task; on
 * wake the task polls each radio's IRQ flags (read-only, so polling one never
 * disturbs another's in-flight RX), drains the one that completed, reassembles
 * split-framed packets, and forwards to rnsd.
 *
 * On-air split framing (a self-contained 1-byte-header format local to this
 * codebase — not RNode/HDLC/KISS, no byte-stuffing):
 *   [1B header][≤254B payload]
 *   header upper nibble = random sequence id
 *   header bit 0       = SPLIT (this is part of a 2-frame split packet)
 * A 500-byte RNS packet rides at most two frames.
 *
 * TX: non-blocking. startTransmit() fires the chip and returns; the TxDone IRQ
 * (the same DIO1 line as RX) wakes the task, which finishes and either sends a
 * split-second frame or re-arms RX. The task is free for the whole airtime, so
 * nothing on its core is starved even at SF12. serviceRadio() reads the chip's IRQ
 * flags to decide what completed rather than guessing TX-vs-RX from state. The
 * radio is half-duplex, so we never start a transmit while a split RX is being
 * reassembled (splitPending) or another transmit is on-air (txActive).
 */
#include "rnsd.h"
#include "lora.h"
#include "esp_idf_hal.h"
#include "spangap.h"
#include "mem.h"       /* gp_alloc (PSRAM) for the LoRaMon ring/history buffers */
#include "rolling.h"   /* one-hour running totals in ten-minute buckets */
#include "ports.h"
#if CONFIG_SPANGAP_NET
/* Only the RNode endpoint's TCP door needs net; the radio itself is bare, and
 * this gate is also what adds spangap-net to REQUIRES when it is staged. On a
 * net-less build the door compiles away and the serial one still works. */
#include "net.h"        /* net_port_msg_t / NET_PORT_REG_PORT */
#endif

#include <RadioLib.h>

#include "driver/gpio.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"   /* radio task → interface task record queue */

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <cctype>

#define LORA_VERSION         4
#define RNS_MTU              500
#define RNODE_MAX_PAYLOAD    254
#define RNODE_FLAG_SPLIT     0x01
#define SPLIT_RX_TIMEOUT_MS  5000

/* ─────────────── LoRaMon: per-on-air-frame record ring ───────────────
 * One record per LoRa frame (so two per split RNS packet). Always recording
 * while a radio is up; the ring holds ~a busy hour. A LoRaMon viewer (web/LCD)
 * pulls the whole ring once over ITS (port LORAMON_PORT) and then follows live
 * frames via the ephemeral `lora.<n>.mon.*` storage keys. `dur_ms` is the
 * frame's computed time-on-air; `t_ms` its start on the monotonic ms clock. */
#define LORA_MON_CAP  4096          /* max published packet nodes per radio (FIFO backstop) */

/* Per-frame protocol class, published as the record's last field and coloured
 * by the LoRaMon apps. Reticulum traffic is everything rnsd sends and receives;
 * "ours" is this straddle's own air protocol: the radio check's BATCH and sweep
 * frames, and the 0x04 power request. */
#define LORA_PKT_RNS    0
#define LORA_PKT_OURS   1
#define LORA_PKT_RNODE  2

/* Channel index carried by every frame record and every RSSI sample. 0 is the
 * reticulum hailing channel — the one a node camps on, at whatever frequency
 * and bandwidth the radio is configured for, and the only channel that exists
 * until frequency agility is switched on. Higher indices are the agile channels
 * of the regime in force. */
#define LORA_CH_HAIL    0
#define LORA_CH_MAX     10          /* hailing channel + the largest regime's agile set */

/* ─────────────── regimes ───────────────
 *
 * A regime is a numbered, versioned statement of what is permissible on air:
 * the channels, and per channel the airtime allowance, the transmission-length
 * ceilings and the power limit. It is the thing two nodes name when they agree
 * how to behave, so the number is the negotiation currency and the table is
 * resolved locally at each end.
 *
 * `s.lora.<n>.afa` IS the regime number. 0 is not a regime — it means no
 * agility, the hailing channel alone, which is what a node does before any of
 * this is switched on. 1 is the EU 863-870 MHz plan below.
 *
 * The airtime allowance is a seconds-per-window PAIR rather than a percentage,
 * because that is what makes the table portable across regulators: EU polite
 * spectrum access is 100 s per 3600 s, an EU duty cycle is 360 s per 3600 s,
 * and US frequency-hopping dwell is 0.4 s per 20 s. One field pair, three
 * regulatory shapes, and nothing downstream has to special-case any of them.
 *
 * Nothing enforces these yet — no transmission ever leaves channel 0. The table
 * exists now so that records, measurements and the UI are all written against
 * channel indices from the start rather than retrofitted later. */

struct RegimeChan {
    uint32_t freqHz;
    uint32_t bwHz;
    float    airtimeMaxS;    /* transmit seconds allowed … */
    uint32_t airtimeWinS;    /* … in any window of this many seconds */
    float    maxTxS;         /* ceiling on one transmission */
    float    maxTxTxnS;      /* ceiling on a dialogue or polling sequence */
    int8_t   maxTxpDbm;
};

/* Regime 1 — EU 863-870 MHz, nine 500 kHz channels on polite spectrum access
 * (100 s/h each), 25 mW e.r.p. Channel 0 is absent from this table: it is the
 * hailing channel and takes the radio's configured frequency and bandwidth,
 * because that is a user choice and not ours to fix. */
static const RegimeChan kRegime1[] = {
    { 863350000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 864050000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 864750000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 865450000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 866150000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 866850000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 867550000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 868250000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 868950000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
};

/* The agile channels of a regime, or none at all for regime 0. */
static const RegimeChan* regimeChans(uint8_t regime, int* count) {
    switch (regime) {
        case 1: *count = (int)(sizeof kRegime1 / sizeof kRegime1[0]); return kRegime1;
        default: *count = 0; return nullptr;
    }
}

/* Channel-RSSI sampling cadence. One getRSSI(false) per beat: a single SPI
 * transaction against a radio that is already in RX, so it costs no measurable
 * power. Carrier sense outranks it — see rssiSamplePoll. */
#define LORA_RSSI_SAMPLE_MS 1000

/* Settling allowed after entering RX on a retuned channel, before the reading
 * is believed. `GetRssiInst` is only meaningful once the receiver is actually
 * running: asked earlier it answers 0xFF, which decodes to −127.5 dBm and looks
 * exactly like a very quiet channel. That is below the thermal noise of any
 * bandwidth this part receives, which is what makes it detectable as garbage
 * rather than data — see LORA_RSSI_INVALID_DBM.
 *
 * STDBY_RC → FS → RX is tens of µs on an SX126x; this is several times that,
 * and the whole nine-channel sweep still fits the excursion budget in
 * plans/psa.md §3.5. */
#define LORA_RSSI_SETTLE_US   200

/* Thermal noise in the narrowest bandwidth in use is about −123 dBm
 * (−174 + 10·log₁₀(125 kHz) + 6 dB noise figure ≈ −117, with margin). Anything
 * at or below this is the receiver not being ready, not a quiet channel. */
#define LORA_RSSI_INVALID_DBM (-125.0f)

/* "No reading this beat" — positive, so it cannot collide with a dBm value.
 * Published as an empty field, which the viewers draw as a gap. */
#define LORA_RSSI_NONE        1

/* Where a locally-originated packet entered the radio segment. The three
 * endpoints — the radio, rnsd, and an attached RNode client — are one segment,
 * so a packet is presented to the two it did not come from; the origin is what
 * says which those are, and it follows the packet down to its LoRaMon colour
 * and its neighbour row. */
#define LORA_ORIG_RNSD   0
#define LORA_ORIG_RNODE  1

/* Manual CLI transmit (`lora <n> tx | tx_psa | tx_prot`). One request at a time
 * per radio: the CLI fills the request fields and notifies the task, which does
 * the actual radio work in its own loop and bumps a result generation the CLI
 * polls on. MTX_RAW blind-transmits a payload, MTX_PSA carrier-senses first,
 * MTX_PROT emits a header that commits receivers for a chosen time. */
enum : uint8_t { MTX_RAW = 0, MTX_PSA = 1, MTX_PROT = 2 };
enum : uint8_t { MTXP_OFF = 0, MTXP_LBT = 1, MTXP_TX = 2 };

/* ─────────────── RNode endpoint ───────────────
 * A stock Reticulum RNodeInterface client attaches to this device as if it were
 * RNode hardware, over USB serial and/or RNode-over-TCP, and becomes the third
 * endpoint of the radio segment: a packet entering from any one of the radio,
 * rnsd and the client is presented to the other two. Radio commands from the
 * client are executed by writing the ordinary s.lora.<n>.* keys, so they take
 * the same path an operator's edit does — and persist, which is the point and
 * also the hazard the README warns about.
 *
 * Wire format is raw KISS on both transports. The client dials TCP port 7633
 * and nothing else: its config URI's host:port suffix is handed whole to
 * getaddrinfo as a hostname, so a port there does not override the constant.
 *
 * We report an AVR platform on purpose. Platform ESP32 arms the client's
 * "CMD_RESET seen while online → IOError" teardown and, with NRF52, unlocks its
 * framebuffer methods — which makes its own detach() emit a framebuffer-disable
 * frame. AVR sidesteps both. The firmware version has to clear the client's
 * 1.52 floor or it calls RNS.panic(), which is os._exit(255).
 *
 * Frames we must never send: CMD_STAT_RX / CMD_STAT_TX (the client's handler
 * calls ord() on an int and takes the interface offline), CMD_ERROR 0x03 / 0x04
 * (unhandled → "Unknown hardware failure"), and any spontaneous CMD_RESET. */

#define RNODE_ITS_PORT   0x524E   /* 'RN'; both net and the serial machinery dial it */

/* KISS framing. */
#define KISS_FEND   0xC0
#define KISS_FESC   0xDB
#define KISS_TFEND  0xDC
#define KISS_TFESC  0xDD

/* Commands — RNS/Interfaces/RNodeInterface.py, class KISS. */
#define RN_CMD_DATA         0x00
#define RN_CMD_FREQUENCY    0x01
#define RN_CMD_BANDWIDTH    0x02
#define RN_CMD_TXPOWER      0x03
#define RN_CMD_SF           0x04
#define RN_CMD_CR           0x05
#define RN_CMD_RADIO_STATE  0x06
#define RN_CMD_DETECT       0x08
#define RN_CMD_LEAVE        0x0A
#define RN_CMD_ST_ALOCK     0x0B
#define RN_CMD_LT_ALOCK     0x0C
#define RN_CMD_READY        0x0F
#define RN_CMD_STAT_RSSI    0x23
#define RN_CMD_STAT_SNR     0x24
#define RN_CMD_PLATFORM     0x48
#define RN_CMD_MCU          0x49
#define RN_CMD_FW_VERSION   0x50
#define RN_CMD_ERROR        0x90

#define RN_DETECT_REQ       0x73
#define RN_DETECT_RESP      0x46
#define RN_RADIO_OFF        0x00
#define RN_RADIO_ON         0x01
#define RN_RADIO_ASK        0xFF
#define RN_ERROR_INITRADIO  0x01
#define RN_PLATFORM_AVR     0x90
#define RN_MCU_1284P        0x91   /* the AVR RNode's MCU; the client only records it */
#define RN_FW_MAJ           1
#define RN_FW_MIN           78
#define RN_RSSI_OFFSET      157    /* the client subtracts this from CMD_STAT_RSSI */
#define RNODE_TXP_MAX       22     /* chip ceiling; a higher request is clamped */

/* Signal stamped onto a packet injected into rnsd from the client. It never
 * crossed the air, so any real-looking reading would be a fiction: -10 dBm at
 * 10.0 dB SNR is top-of-scale "perfect local", impossible over the air, and so
 * unmistakable as an injection in every signal view. */
#define RNODE_INJ_RSSI    (-10)
#define RNODE_INJ_SNR10   100

/* One session at a time, across every transport. */
struct RnodeState {
    int      handle = -1;       /* ITS handle of the attached client; -1 = none */
    int      radio;             /* radio index this session is bound to */

    /* KISS decoder. */
    bool     inFrame, haveCmd, escape, overflow;
    uint8_t  cmd;
    uint8_t  buf[RNS_MTU + 8];
    size_t   len;

    /* Inbound carry: a frame can complete mid-chunk and park a packet, and the
     * bytes after it in the same read must not be lost. */
    uint8_t  inBuf[128];
    size_t   inLen, inPos;

    /* One decoded packet, waiting for the channel. While it is parked the pump
     * stops reading, so backpressure reaches the client rather than a second
     * packet overwriting the first. */
    uint8_t  txPkt[RNS_MTU];
    size_t   txLen;

    bool     echoPend;          /* the apply pass owes the client a config echo */
    bool     offPend;           /* RADIO_STATE OFF, deferred to the apply deadline */
    bool     wantOn;            /* the client asked for ON; a failed start is an error */
    bool     txAlternate;       /* fair share of the channel between rnsd and the client */
};
static RnodeState s_rnode;

/* Rolling one-hour airtime, 12 × 5-minute buckets per radio. The apps derive
 * every shorter window from the frame records themselves; only the hour — which
 * needs more history than a viewer may have been open for — is published. */
#define AIR_BUCKETS     12
#define AIR_BUCKET_MS   (5u * 60u * 1000u)

struct AirBucket { uint32_t absIdx; uint32_t rxMs, txMs; };

/* ─────────────── passive neighbour table ("Eve") ───────────────
 * Built entirely from observing rx + tx RNS packets on the interface — no rnsd
 * API, no peer cooperation. Only frames whose ORIGINATOR is the transmitting RF
 * neighbour are used: the wire hops byte is incremented by the receiving
 * transport, so a frame fresh from its originator carries hops == 0 on air
 * (µR's hops() reports 1 for the same frame). Announces at hops 0 name the
 * transmitter cryptographically (signature-verified identity key; dest ==
 * H(name_hash ‖ identity_hash) groups dest hashes per device); LR/LRPROOF pairs
 * name links; proofs we elicited close a per-neighbour delivery-quality loop.
 * IFAC frames are masked end-to-end and are skipped (the table stays empty on
 * an IFAC network). Surfaced by `lora [<n>] neighbors`. */
#define NEI_MAX              24      /* neighbour entries per radio */
#define NEI_DESTS_MAX        8       /* dest hashes clustered per node */
#define NEI_IDS_MAX          4       /* identities clustered per node — one
                                      * device legitimately runs several (its
                                      * transport, rnsh and lxmf identities are
                                      * all distinct), and a 0x03 is what folds
                                      * them into one row */
#define NEI_LINKS_MAX        12      /* observed links per radio */
#define NEI_PEND_MAX         8       /* outstanding proof expectations per radio */
#define NEI_PROOF_TIMEOUT_MS 30000   /* elicited proof must return within this */
#define NEI_BUCKETS          12      /* last-hour rollup: 12 × 5 min */
#define NEI_BUCKET_MS        (5u * 60u * 1000u)
#define NEI_LINK4_MAX        12      /* first-4 hashes linked to a node by 0x03 */

/* RNS wire constants (see the header decode in loraTracePacket). */
#define NEI_PT_DATA     0
#define NEI_PT_ANNOUNCE 1
#define NEI_PT_LINKREQ  2
#define NEI_PT_PROOF    3
#define NEI_DT_SINGLE   0
#define NEI_DT_LINK     3
#define NEI_CTX_LRPROOF 0xFF
#define NEI_ECPUBSIZE   64          /* LR ephemeral keys; link_id hashes only these */

struct NeiBucket {                  /* one 5-minute rollup slot */
    uint32_t absIdx;                /* millis()/NEI_BUCKET_MS this slot holds */
    uint16_t cnt;
    int32_t  rssiSum;
    int32_t  snrSum10;
};

#define NEI_NAME_MAX 20             /* announced display name, truncated */

struct NeiDest {                    /* one destination hash in a node's cluster */
    uint8_t  hash[16];
    uint8_t  nameHash[10];          /* from the announce; labels the row */
    bool     haveName;
    char     name[NEI_NAME_MAX];    /* display name from the announce app_data */
    uint32_t announces;
    uint32_t lastMs;
};

struct Neighbor {
    bool     used;
    bool     isUs;                  /* built from our own tx announces */
    bool     isRnode;               /* built from the RNode client's tx announces —
                                     * a second local row, distinct from ours */
    uint8_t  ids[NEI_IDS_MAX][16];
    uint8_t  nIds;
    NeiDest  dests[NEI_DESTS_MAX];
    uint8_t  nDests;
    /* Signal envelope over rx frames provably transmitted by this node. */
    bool     haveSig;
    int16_t  rssiMin, rssiMax;
    int16_t  snrMin10, snrMax10;
    uint32_t lastHeardMs;
    uint32_t frames;
    /* Link quality: EWMA (0–255) of elicited proofs that came back (LR→LRPROOF,
     * data to a dest that has proven before) — direct dests only. */
    bool     haveQuality;
    uint8_t  quality;
    uint16_t qSent, qProved;
    bool     provesData;            /* has proven a plain data packet (PROVE_ALL) */
    bool     transit;               /* seen rebroadcasting announces (a transport node) */
    /* Hashes learned to belong to this node, held as first-4. `node4` is the
     * node's rnstransport first-4. Both are populated by the identity join. */
    uint8_t  node4[4];
    bool     haveNode4;
    uint8_t  link4[NEI_LINK4_MAX][4];
    uint8_t  nLink4;
    bool     haveAdv;               /* peer stated a hash count / roaming bit */
    uint8_t  advHashes;
    bool     roaming;
    bool     ourProto;              /* has spoken our air protocol to us */
    bool     haveTxPwr;             /* a probe settled on a power for this node */
    int8_t   txPwr;
    /* Adaptive TX power: the single power every frame to this node goes out at,
     * and where that number came from. Settled once and kept (see AP_ below). */
    bool     haveApPwr;
    int8_t   apPwr;
    bool     apFromEst;             /* derived from reciprocity, not measured */
    NeiBucket buck[NEI_BUCKETS];    /* last-hour rollup ring */
};

struct NeiLink {
    bool     used;
    uint8_t  linkId[16];
    bool     haveDest;
    uint8_t  dest[16];              /* the LR's destination */
    bool     ours;                  /* we are an endpoint (initiated or host) */
    bool     established;           /* LRPROOF seen */
    bool     unresolved;            /* discovered from mid-link traffic, LR missed */
    bool     haveSig;
    int16_t  lastRssi, lastSnr10;   /* last rx frame on this link */
    uint32_t lastMs;
    uint32_t frames;
    /* A 0x04 power request the peer bound to this link's setup. A link is
     * identifiable, so unlike an ad-hoc exchange it may legally hold state
     * about a peer it cannot otherwise name — the request covers the session. */
    bool     haveSuggest;
    int8_t   suggestDbm;
};

struct NeiPend {                    /* an outstanding proof expectation */
    bool     used;
    bool     isLR;
    bool     counted;               /* a miss scores against quality */
    uint8_t  phash[16];             /* proofs are addressed to this (truncated
                                     * packet hash; link_id for an LR) */
    uint8_t  dest[16];              /* the elicitor's destination */
    uint32_t deadlineMs;
};

/* Unattributed relayed traffic: every rx frame at wire hops ≥ 1 was, by
 * definition, transmitted by an in-range transport node — even when nothing
 * names it (HEADER_1 relays, relayed proofs, a silent access-point bridge).
 * One aggregate row per radio; anonymous transmitters can't be told apart. */
struct NeiAnon {
    uint32_t frames;
    uint32_t inbandRelays;          /* packet re-heard one hop later: RF→RF repeat */
    bool     haveSig;
    int16_t  rssiMin, rssiMax;
    int16_t  snrMin10, snrMax10;
    uint32_t lastMs;
};

#define NEI_SEEN_MAX     16         /* recent-rx ring for relay coupling */
#define NEI_SEEN_WIN_MS  30000      /* relay must re-appear within this */

struct NeiSeen {
    uint8_t  hash[16];              /* truncated packet hash (hops-invariant) */
    uint8_t  hops;
    uint32_t ms;
};

struct NeiState {
    Neighbor nei[NEI_MAX];
    NeiLink  links[NEI_LINKS_MAX];
    NeiPend  pend[NEI_PEND_MAX];
    NeiAnon  anon;
    NeiSeen  seen[NEI_SEEN_MAX];
    uint8_t  seenNext;
    uint32_t sinceMs;               /* millis() at first allocation */
};

/* ── CSMA / listen-before-talk ──
 * Non-blocking DIFS + contention-window backoff, run from the task loop.
 * Carrier sense is instantaneous channel RSSI vs a tracked noise floor.
 *
 * Two backoff regimes share the sense and the state machine, selected per radio
 * by s.lora.<i>.appc (both require s.lora.<i>.lbt):
 *   appc=0  the constants below — a contention window that grows exponentially
 *           on each busy encounter and resets after every grant.
 *   appc=1  the APPC_* constants further down — RNode's regime, where the
 *           window is drawn from a band chosen by our own recent airtime. */
#define CSMA_CW_MIN          2       /* initial CW exponent → up to 2^2 = 4 slots */
#define CSMA_CW_MAX          6       /* CW ceiling → up to 2^6 = 64 slots */
#define CSMA_RSSI_MARGIN_DB  6.0f    /* dB above noise floor that reads as busy */
#define CSMA_NOISE_FLOOR_DBM (-105.0f)  /* initial noise-floor estimate */
#define CSMA_SLOT_MS_MIN     2       /* slot-time clamp (derived from symbol time) */
#define CSMA_SLOT_MS_MAX     25

/* ── APPC: adaptive p-persistent CSMA ──
 *
 * The acronym is ours, coined for this straddle — no upstream project uses it,
 * and it is imprecise: the mechanism is not p-persistent CSMA in the textbook
 * sense (Kleinrock & Tobagi 1975), which gates each transmit opportunity behind
 * a probability p. There is no coin flip here. What this implements — copied
 * from RNode firmware, which is where the numbers below come from — is an
 * *adaptive contention window*: the random backoff is drawn from one of four
 * bands, and the band is selected by how much of the recent past this radio
 * spent transmitting. Busier radio, higher band, longer expected backoff. The
 * effect is the load-responsive politeness p-persistence aims at, reached by
 * sizing the window instead of by rolling dice, so the name is a label for the
 * feature and not a description of the algorithm.
 *
 * The load input is our OWN transmit duty cycle, not observed channel
 * occupancy. That is RNode's choice too: its update_csma_parameters() reads
 * `airtime` (own time-on-air over the last two bins), not `total_channel_util`
 * (which adds carrier-detect samples and is only reported to the host). Every
 * radio on a congested channel is transmitting more — including its retries —
 * so own-airtime tracks aggregate load closely enough to act on, and it costs
 * no extra sensing: we already know each frame's time-on-air.
 *
 * Upstream reference (RNode_Firmware, Config.h CSMA Parameters,
 * update_csma_parameters(), tx_queue_handler(), add_airtime(), updateBitrate()).
 * Divergences from upstream are listed in INTERNALS.md §6a. */
#define APPC_SLOT_SYMBOLS        12      /* slot = 12 symbol times … */
#define APPC_SLOT_MAX_MS        100      /* … clamped to this ceiling … */
#define APPC_SLOT_MIN_MS         24      /* … and this floor … */
#define APPC_SLOT_MIN_FAST_DELTA 18      /* … which drops to 6 ms above the fast rate */
#define APPC_FAST_THRESHOLD_BPS  30000   /* nominal bitrate that counts as "fast" */
#define APPC_SIFS_MS              0      /* DIFS = SIFS + 2 slots */
#define APPC_CW_BANDS             4      /* number of contention bands */
#define APPC_CW_PER_BAND_WINDOWS 15      /* window count each band spans */
#define APPC_BAND_1_MAX_AIRTIME   7      /* ≤ this own-airtime % stays in band 1 */
#define APPC_BAND_N_MIN_AIRTIME  85      /* airtime % that maps to the top band */
#define APPC_BIN_MS            7500      /* airtime accounting bin (RNode: 3 ms × 2500) */
#define APPC_BINS               480      /* bins in the uptime hour (3600 s / 7.5 s) */
#define APPC_HOUR_MS        3600000

enum CsmaPhase : uint8_t { CSMA_IDLE, CSMA_DIFS, CSMA_BACKOFF };

/* ─────────────── announce buffer + radio check ───────────────
 * Full protocol in plans/iface-lora/announce-batch-and-radio-check.md.
 *
 * Every outgoing announce this node ORIGINATES (wire hops 0) is kept, keyed by
 * destination hash, replaced when that destination announces again, dropped
 * after an hour. Every 15 minutes — and on `lora a[nnounce]` — the whole buffer
 * is replayed, followed by a radio check:
 *
 *   [sense] ANNOUNCE… bunched to just under 1 s of air, back-to-back
 *   [sense] BATCH ×2  manifest of 1-byte hashes, + the power they went out at
 *           20 ms    everyone retunes to the sweep regime
 *           SWEEP ×7  at SF7, power walking min → max
 *           SWEEP ×14 at SF5, the same walk
 *           restore
 *
 * One-way. Nobody answers and nothing is retried: a receiver's whole job is to
 * notice the lowest-powered sweep frame it could decode, which is that node's
 * power floor toward it and — path loss being reciprocal — an estimate of what
 * we need to reach them. Every sweep frame states the power it was sent at, so
 * the estimate is of PATH LOSS and stays valid even when the far end is itself
 * adapting; that is what makes the reciprocity here safe where passive
 * reciprocity is not (plans/adaptive-power.md §3).
 *
 * The replayed announces need no new coupling path: an announce is an announce,
 * so neiObserve's hops-0 ingest and cryptographic identity join (§13) run on
 * them unchanged. The radio check adds only the power floor, which passive
 * observation cannot give. */
#define ANN_MAX_ENTRIES   16                      /* bounds RAM *and* the manifest, hence the tail's airtime */
#define ANN_MAX_LEN       256                     /* an announce is ~167 B + app_data; longer is not buffered */
#define ANN_TTL_MS        (60u * 60u * 1000u)     /* an hour, replayed or not */
#define ANN_INTERVAL_DEF  30                      /* s.lora.<i>.announce_interval, minutes */
#define ANN_JITTER_PCT    10                      /* ± this much, so a fleet does not synchronise */

/* One bunch of announces is filled until the next would push it past this.
 * PSA's single-transmission ceiling (plans/psa.md §2.4) used as the bunch size:
 * a run of separately-framed packets would more naturally be a dialogue at 4 s,
 * but nothing wants bunches longer than a second and taking the tighter reading
 * costs nothing. At SF7/BW125 it fits three. */
#define ANN_BUNCH_MAX_MS  1000

#define LORA_MAGIC_BATCH  0x05
#define RC_BATCH_REPEATS  2
#define RC_BATCH_GAP_MS   5      /* between the two BATCH copies */
#define RC_LEAD_MS        20     /* BATCH end → first sweep frame */
#define RC_SWEEP_LEN      4      /* headerless, so both ends agree it statically */
/* Two above the minimum. Nothing is being discovered — the receiver was told to
 * expect these — but at SF5 a slot is barely 12 ms and the receiver must turn
 * RX_DONE around and re-arm inside the guard; the extra symbols widen the
 * window it has to catch the next frame in. Dropping to 6 costs alternate SF5
 * frames on the receive side. */
#define RC_SWEEP_PREAMBLE 8
#define RC_SWEEP_SYNC     0x23   /* not 0x42 — a node not in a check never correlates */
#define RC_STEPS_SF7      7
#define RC_STEPS_SF5      14
#define RC_SWEEP_SF5      5
/* Padding on each sweep slot, so a slot is ToA + this. It is the receiver's
 * turnaround budget: RX_DONE serviced, radio re-armed, next preamble detected.
 * At SF5 the frame itself is under 8 ms, so too small a guard costs ALTERNATE
 * frames — the receiver is still busy when the next one flies. Generous here is
 * cheap: the whole tail is half of Ton_max even at this. */
#define RC_SLOT_GUARD_MS  12
#define RC_RETUNE_MS      10     /* SF7 → SF5 changeover */
#define RC_FLOOR_DBM      (-9)   /* the ladder's low end; chips clamp up from it */

/* `step` byte: phase in the high nibble, index in the low. The index is what
 * lets a receiver that missed the LAST frame still time the changeover — and
 * the last frame is the full-power one, so the node most likely to miss it is
 * exactly the distant node the sweep exists to measure. */
#define RC_PHASE_SF7      0x00
#define RC_PHASE_SF5      0x10
#define RC_STEP_PHASE(b)  ((uint8_t)((b) & 0xF0))
#define RC_STEP_INDEX(b)  ((uint8_t)((b) & 0x0F))

struct AnnRec {
    uint8_t  dest[16];
    uint16_t len;
    uint32_t ms;                 /* millis() when stored */
    bool     used;
    uint8_t  data[ANN_MAX_LEN];  /* the RNS packet verbatim — it is signed, so it
                                  * cannot be regenerated and must not be altered */
};
struct AnnBuf { AnnRec e[ANN_MAX_ENTRIES]; };

/* Transmit side of a run, and the receive side of somebody else's. */
enum RcPhase : uint8_t {
    RC_OFF = 0,
    RC_ANN,        /* replaying announce bunches, carrier-sensed per bunch */
    RC_TAIL_LBT,   /* winning the channel for the whole tail, once */
    RC_BATCH,      /* the two manifest copies */
    RC_SWEEP,      /* both sweep phases, driven off the slot timer */
    RC_LISTEN,     /* receiving somebody else's sweep */
};

struct RcState {
    uint8_t  phase;
    uint8_t  step;                       /* index within the current phase */
    uint8_t  sfPhase;                    /* RC_PHASE_SF7 / RC_PHASE_SF5 */
    uint8_t  n;                          /* manifest entries */
    uint8_t  hashes[ANN_MAX_ENTRIES];    /* the manifest itself */
    uint8_t  tag[2];                     /* first announce's first two dest bytes */
    uint8_t  annIdx;                     /* next buffer slot to replay */
    uint32_t bunchAirMs;                 /* air already committed to this bunch */
    uint16_t tailWaitMs;                 /* carrier sense the tail paid, for the record */
    int8_t   txpFloor, txpCeil;          /* the ladder's ends */
    int64_t  slotUs;                     /* absolute µs of the next slot */
    bool     req;                        /* CLI or the beat asked for a run */
    uint32_t nextRunMs;                  /* the 15-minute beat */
    esp_timer_handle_t timer;

    /* Modem state to put back when the sweep regime comes off. */
    uint8_t  savedSync;
    uint16_t savedPreamble;
    uint8_t  savedSf;

    /* Receive side: whose sweep we are listening to, and what we have heard.
     * rxDest is the full destination hash the manifest byte resolved to, so the
     * result lands on the right neighbour row however many rows share a byte. */
    uint8_t  rxTag[2];
    uint8_t  rxDest[16];
    bool     haveRxDest;
    int8_t   rxBestTxp;                  /* lowest stated power we decoded */
    int8_t   rxBestRssi;
    uint8_t  rxHeard;                    /* frames decoded this run */
    int64_t  rxUntilUs;                  /* give up listening at this µs */
};

/* ── adaptive TX power (s.lora.<i>.adaptive_txpwr, default 0) ──
 * The first slice of plans/adaptive-power.md: one power determination per
 * neighbour node, then every frame whose first RF hop is that node goes out at
 * it. There is no control loop yet — no walk-down, no failure recovery, no
 * re-measurement. One number per node, settled once, applied from then on.
 *
 * Getting the number. **Nothing is initiated any more** — we never ask a peer
 * to measure itself against us. Two passive sources:
 *
 *   - MEASURED. A node running our code sweeps its whole power range after
 *     every announce batch. The lowest step we can still decode is the least
 *     power it needs to reach us, and since each sweep frame states the power
 *     it was sent at, that is a PATH LOSS measurement — it stays valid even
 *     while the far end adapts its own power, which a bare RSSI reading would
 *     not. Reciprocity makes it our estimate toward them.
 *   - ESTIMATED. Everyone else, from the reciprocity estimate of §13.1 plus
 *     AP_EST_MARGIN_DB.
 *
 * The margin exists because the estimate credits an unmeasured peer with
 * s.lora.assumed_peer_txp rather than knowing its power — and, in both cases,
 * because ambient noise is NOT reciprocal even where path loss is. A node
 * sitting beside an interferer needs more from us than our own quiet receiver
 * would suggest, and no measurement we can make from here will say so.
 *
 * A node with the key off still sweeps, so its neighbours can measure
 * themselves against it; only *applying* a determination is gated on the key.
 *
 * The determination is per NODE, keyed through the neighbour table's identity
 * clustering, so it covers every hash that node owns — including hashes learned
 * later by announce.
 *
 * Never on a broadcast: an announce has no single next hop and must reach
 * everyone, so it always goes out at the configured tx_power. */
#define AP_EST_MARGIN_DB      5      /* added to the estimate when nothing measured */

/* ── 0x04: the power request ──
 * A 4-byte frame sent BACK TO BACK in front of the packet it relates to, in the
 * normal modem regime (sync 0x42, explicit header, preamble 12) so it needs no
 * reconfigure between the two. It carries a TX power we suggest the peer use,
 * and/or our measurement of the peer's last frame:
 *
 *   [0x04][suggested txp, int8 dBm][rssi, encoded][snr, encoded]
 *
 * The two payload fields map onto the two knowledge states, which is why either
 * may be absent: if we know who the peer is we hold its history and can compute
 * the answer, so we send a suggestion; if we don't, we can still report what we
 * measured, and the peer closes the loop itself because it knows what power it
 * transmitted at. Sentinels carry the "and/or", so no flags byte is needed.
 *
 * **Absence of the frame means "use your maximum."** That is the whole fallback,
 * and it needs no constant agreed between the ends — so recovery from a bad
 * suggestion is simply to stop sending one, and a peer that cannot comply with a
 * request just clamps at its own ceiling with nothing to signal.
 *
 * An explicit request outranks our own reciprocity estimate: the receiver is the
 * authority on its own reception, having folded in its noise floor, antenna and
 * sensitivity, none of which a transmitter can see.
 *
 * Honouring a request is gated on s.lora.<i>.adaptive_txpwr: obeying one puts
 * our transmit power under someone else's control, observably, so a node with
 * the key off must stay at its configured power or the opt-out isn't one.
 *
 * Nothing under 20 B on air can be an RNS packet (HEADER_MINSIZE is 19 and we
 * add a framing byte), which is how our own air frames discriminate; a 4-byte
 * frame is unambiguous on length alone.
 *
 * Design and the reasoning behind each choice: plans/adaptive-power.md §3a. */
#define LORA_MAGIC_PWRREQ     0x04
#define PWRREQ_LEN            4
#define PWRREQ_NO_TXP         ((int8_t)0x7F)   /* "no suggestion" sentinel */
#define AP_PWR_MAX_DBM        22     /* no radio exceeds this, so asking for it
                                      * is what sending nothing already means */
#define AP_MIN_SAMPLES        3      /* recent frames before we dial a peer down */

#if defined(CONFIG_LORA0_CS_PIN)

static const char* TAG = "lora";

/* Our mesh radio layer, shown as a node tag in `lora n`: Spectrum Utilization
 * and Performance Enhancements, designed in plans/iface-lora/SUPE.md. The same
 * name both LoRaMon viewers give the protocol in their legends. */
#define RF_PROTO_NAME         "SUPE"

/* ─────────────── Kconfig → descriptor table ─────────────── */

/* Every RadioLib LoRa chip this interface drives, as (enum suffix, RadioLib
 * class name, family). The X-macro generates the LoraChip enum, the name table,
 * the family map, and the constructor switch from this one list — and its order
 * fixes the numeric CONFIG_LORAn_CHIP_ID the Kconfig choice resolves to, so keep
 * it in lockstep with iface-lora/esp-idf/Kconfig (id = position, from 0). Families
 * differ only in begin() shape + a couple of init extras (see radioBegin). */
enum LoraFamily { FAM_SX126X, FAM_SX127X, FAM_SX128X, FAM_LR11X0, FAM_LR2021 };

#define LORA_CHIPS(X) \
    X(SX1261, FAM_SX126X) X(SX1262, FAM_SX126X) X(SX1268, FAM_SX126X) X(LLCC68, FAM_SX126X) \
    X(SX1272, FAM_SX127X) X(SX1276, FAM_SX127X) X(SX1277, FAM_SX127X) X(SX1278, FAM_SX127X) \
    X(SX1280, FAM_SX128X) X(SX1281, FAM_SX128X) X(SX1282, FAM_SX128X) \
    X(LR1110, FAM_LR11X0) X(LR1120, FAM_LR11X0) X(LR1121, FAM_LR11X0) \
    X(LR2021, FAM_LR2021)

enum LoraChip {
#define X(name, fam) CHIP_##name,
    LORA_CHIPS(X)
#undef X
};

struct LoraSlot {
    int      cs, dio1, busy, rst;  /* dio1 = the chip's IRQ line (DIO1/DIO0/IRQ) */
    int      tcxo_mv;              /* TCXO control voltage, mV (0 = XTAL); SX126x/LR only */
    bool     dio2_rf_switch;       /* SX126x: drive DIO2 as the RF switch */
    int      rfsw_rx, rfsw_tx;     /* external RF-switch GPIOs (-1 = none, see Module::setRfSwitchPins) */
    LoraChip chip;
};

#ifdef CONFIG_LORA0_DIO2_RF_SWITCH
#  define LORA0_DIO2 true
#else
#  define LORA0_DIO2 false
#endif
#ifdef CONFIG_LORA1_DIO2_RF_SWITCH
#  define LORA1_DIO2 true
#else
#  define LORA1_DIO2 false
#endif
#ifdef CONFIG_LORA2_DIO2_RF_SWITCH
#  define LORA2_DIO2 true
#else
#  define LORA2_DIO2 false
#endif
#ifdef CONFIG_LORA3_DIO2_RF_SWITCH
#  define LORA3_DIO2 true
#else
#  define LORA3_DIO2 false
#endif

static const LoraSlot kSlots[] = {
    { CONFIG_LORA0_CS_PIN, CONFIG_LORA0_DIO1_PIN, CONFIG_LORA0_BUSY_PIN, CONFIG_LORA0_RST_PIN,
      CONFIG_LORA0_TCXO_MV, LORA0_DIO2, CONFIG_LORA0_RFSW_RX_PIN, CONFIG_LORA0_RFSW_TX_PIN,
      (LoraChip)CONFIG_LORA0_CHIP_ID },
#if defined(CONFIG_LORA1_CS_PIN)
    { CONFIG_LORA1_CS_PIN, CONFIG_LORA1_DIO1_PIN, CONFIG_LORA1_BUSY_PIN, CONFIG_LORA1_RST_PIN,
      CONFIG_LORA1_TCXO_MV, LORA1_DIO2, CONFIG_LORA1_RFSW_RX_PIN, CONFIG_LORA1_RFSW_TX_PIN,
      (LoraChip)CONFIG_LORA1_CHIP_ID },
#endif
#if defined(CONFIG_LORA2_CS_PIN)
    { CONFIG_LORA2_CS_PIN, CONFIG_LORA2_DIO1_PIN, CONFIG_LORA2_BUSY_PIN, CONFIG_LORA2_RST_PIN,
      CONFIG_LORA2_TCXO_MV, LORA2_DIO2, CONFIG_LORA2_RFSW_RX_PIN, CONFIG_LORA2_RFSW_TX_PIN,
      (LoraChip)CONFIG_LORA2_CHIP_ID },
#endif
#if defined(CONFIG_LORA3_CS_PIN)
    { CONFIG_LORA3_CS_PIN, CONFIG_LORA3_DIO1_PIN, CONFIG_LORA3_BUSY_PIN, CONFIG_LORA3_RST_PIN,
      CONFIG_LORA3_TCXO_MV, LORA3_DIO2, CONFIG_LORA3_RFSW_RX_PIN, CONFIG_LORA3_RFSW_TX_PIN,
      (LoraChip)CONFIG_LORA3_CHIP_ID },
#endif
};
static constexpr int kNumRadios = (int)(sizeof(kSlots) / sizeof(kSlots[0]));

/* ─────────────── per-radio state ─────────────── */

struct LoraRadio {
    int             idx;
    const LoraSlot* slot;

    EspIdfHal*      hal;
    Module*         mod;
    PhysicalLayer*  radio;        /* RadioLib base; concrete class per slot chip */
    int             found;        /* -1 unprobed, 0 absent, 1 detected */

    int             rnsdHandle;
    bool            running;
    bool            enabled;
    uint8_t         curMode;
    uint32_t        curBitrate;
    /* Live modem params, kept for per-packet airtime accounting (the airtime
     * formula needs SF/BW/CR/preamble, and only radioStart reads them). */
    int             cfgSf, cfgBwHz, cfgCr, cfgPreamble;
    uint8_t         airSf;           /* spreading factor CURRENTLY installed */
    /* Framing the radio is CURRENTLY set to — tracks the radio-check sweep regime,
     * so airtime records are computed with the parameters the frame really used
     * rather than the configured ones. */
    int             airPreamble;
    bool            airImplicit;
    char            curIfacNetname[32];   /* IFAC network_name (s.) */
    char            curIfacNetkey[64];    /* IFAC passphrase (secrets.) */
    uint8_t         curIfacSize;          /* IFAC access-code length */
    uint8_t         curAnnounceCap;       /* % bandwidth cap for announces (s.) */

    /* Split-RX reassembly — one in-flight split at a time per radio. */
    uint8_t         splitBuf[RNS_MTU + 16];
    size_t          splitLen;
    uint8_t         splitSeq;
    bool            splitPending;
    TickType_t      splitDeadline;

    /* CSMA / listen-before-talk. slotTicks/difsTicks derive from the LoRa
     * symbol time at config; the phase machine is driven from the task loop. */
    bool            lbt;             /* carrier-sense enabled (s.lora.<i>.lbt) */
    bool            rxBoostedGain;   /* SX126x LNA boosted RX gain (s.lora.<i>.rx_boosted_gain) */
    TickType_t      slotTicks;       /* CSMA slot time */
    TickType_t      difsTicks;       /* inter-frame listen before backoff */
    CsmaPhase       csmaPhase;
    int             csmaCw;          /* current contention-window exponent */
    int             csmaBackoff;     /* backoff slots remaining */
    TickType_t      csmaDifsStart;   /* tick the current unbroken free window began */
    TickType_t      csmaSlotDeadline;/* next backoff slot boundary */
    TickType_t      csmaStart;       /* tick this frame's channel-access attempt began */
    bool            csmaStalled;     /* stall warning emitted for this frame (once) */
    float           noiseFloor;      /* tracked channel noise floor, dBm */
    uint32_t        lbtTimeoutMs;    /* drop a frame LBT can't clear within this (s.lora.<i>.lbt_timeout) */
    TickType_t      lbtTimeoutTicks; /* lbtTimeoutMs in ticks; 0 = never drop */

    /* APPC: adaptive contention window (s.lora.<i>.appc), inside the lbt gate.
     * Its own slot/DIFS timing, since RNode sizes those differently from the
     * exponential regime's. The window is a wall-time target accumulated only
     * while the medium reads free, not a slot countdown. */
    bool            appc;
    TickType_t      appcSlotTicks;   /* 12 symbols, clamped */
    TickType_t      appcDifsTicks;   /* SIFS + 2 slots */
    int             appcCw;          /* windows drawn for this frame; -1 = redraw */
    TickType_t      appcCwTarget;    /* appcCw × appcSlotTicks */
    TickType_t      appcCwPassed;    /* free-medium time accumulated toward it */
    TickType_t      appcCwStart;     /* last accumulation stamp; 0 = not counting */
    TickType_t      appcDifsStart;   /* tick the unbroken free window began; 0 = none */
    uint8_t         appcBand;        /* contention band 1..APPC_CW_BANDS in force */
    uint32_t        appcBinIdx;      /* airtime bin (of the uptime hour) last touched */
    uint32_t        appcBinCur;      /* our on-air ms inside that bin … */
    uint32_t        appcBinPrev;     /* … and inside the one before it */

    /* Non-blocking TX: startTransmit() fires the chip and returns; the TxDone IRQ
     * (same DIO1 line as RX) wakes the task, which finishes and either sends the
     * split-second frame or re-arms RX. txActive gates RX servicing and new
     * outbound so the half-duplex radio is never asked to do two things at once. */
    bool            txActive;        /* a frame is on-air, awaiting TxDone */
    TickType_t      txDeadline;      /* watchdog: recover if TxDone never arrives */
    TickType_t      txWatchTicks;    /* per-frame TxDone watchdog budget (airtime + margin) */
    uint8_t         txSeq;           /* 4-bit seq nibble shared by a split pair */
    uint8_t         txFrame[2][1 + RNODE_MAX_PAYLOAD];  /* prebuilt on-air frame(s) */
    size_t          txFrameLen[2];
    uint8_t         txFrameCount;    /* 1, or 2 for a split packet */
    uint8_t         txFrameSent;     /* completed frames so far */
    size_t          txPayloadBytes;  /* RNS payload bytes, credited on completion */
    /* Wall time the head frame spent queued before its first bit went on air.
     * Started the first pass we have something to send and can't, so it covers
     * every reason a frame waits — the radio held by a radio check or a
     * frame, a split reassembly still landing, our own transmit finishing, and
     * then DIFS/backoff against a channel that reads busy — rather than
     * contention alone. Stamped into the frame's LoRaMon record. */
    uint32_t        txWaitStartMs;
    bool            txWaitPend;
    uint16_t        txWaitMs;

    /* Stats — published to ephemeral storage once per task tick. */
    uint64_t        txBytes, rxBytes, txFrames, rxFrames, crcErr, splitTimeouts, txDropped;
    float           rssiLast, snrLast;

    /* LoRaMon — each on-air frame becomes a storage node lora.<n>.packets.<ms>;
     * this FIFO of start-ms drives expiry (delete nodes > 1 h old). */
    int8_t          cfgTxp;          /* configured TX power dBm */
    uint8_t         cfgSync;         /* configured sync word (restored after a sweep) */
    int8_t          txPwrNow;        /* power of the frame on-air, stamped into tx records */
    uint8_t         txType[2];       /* per frame: LORA_PKT_*. A 0x04 power request
                                      * and the RNS packet it prefixes share one
                                      * channel access but not one protocol. */
    bool            txFromRnode;     /* the packet on air came from the RNode client;
                                      * its queue is released when the last frame
                                      * of it leaves (see txRearmRx). */
    AirBucket       air[AIR_BUCKETS];/* rolling one-hour airtime */
    uint32_t        txFrameStartMs;  /* start (millis) of the on-air TX frame */
    /* LoRaMon expiry FIFO. Owned by the INTERFACE task — it is the only thing
     * that publishes or deletes packet nodes (§ "the two tasks"). The radio
     * task allocates it once at radioStart and never touches it again. */
    uint32_t*       pktMs;           /* FIFO of published packet start-ms (gp_alloc'd at radioStart) */
    uint16_t        pktCap, pktHead, pktCount;

    /* Channel-RSSI sampling (radio task). One getRSSI(false) per beat while the
     * radio is idle; carrier sense outranks it, so a frame contending for the
     * medium leaves a gap in the series rather than a stale reading. */
    /* Tick at the end of the last reception. Carrier sense cannot see every
     * frame this radio decodes, so the receiver reports the medium it held —
     * see csmaMediumHeld. 0 = nothing outstanding. */
    TickType_t      rxHeldTick;

    uint32_t        cfgFreqHz;       /* configured carrier — the hailing channel */
    uint8_t         afa;             /* s.lora.<i>.afa: the regime number, 0 = no agility */
    uint16_t        annIntervalMin;  /* s.lora.<i>.announce_interval, 0 = manual only */
    Rolling1h       txAir[LORA_CH_MAX];  /* transmit seconds per channel, rolling hour */

    TickType_t      rssiNext;        /* tick the next sample is due */
    uint32_t        rssiDropped;     /* samples lost to a full interface queue */
    uint32_t        monDropped;      /* frame records lost to a full interface queue */

    /* Passive neighbour table (gp_alloc'd at radioStart, kept across cycles). */
    NeiState*       nei;

    /* Announce buffer + radio check (gp_alloc'd alongside nei, kept likewise). */
    AnnBuf*         ann;
    RcState         rc;

    /* Adaptive TX power (overview at AP_EST_MARGIN_DB). */
    bool            adaptive;        /* s.lora.<i>.adaptive_txpwr */
    /* A 0x04 power request just received, awaiting the frame it prefixes. The
     * frame carries no binding field — it binds by adjacency alone — so this is
     * consumed or discarded by the very next rx frame, never held. */
    bool            apRxSuggestPend;
    int8_t          apRxSuggest;


    /* Manual CLI transmit (`lora <n> tx | tx_psa | tx_prot`). The CLI fills the
     * request under mtxReq and notifies; manualTxPoll (task loop) services it and
     * bumps mtxResGen with the outcome in mtxResOk/mtxResMsg. mtxPhase drives the
     * PSA carrier-sense state; while it is non-OFF the normal outbound drain and
     * the radio check stand off the radio. */
    volatile bool     mtxReq;        /* CLI → task: a request is pending */
    uint8_t           mtxKind;       /* MTX_RAW / MTX_PSA / MTX_PROT */
    uint8_t           mtxData[256];  /* payload bytes (RAW / PSA) */
    uint16_t          mtxLen;        /* payload length (RAW / PSA) */
    uint16_t          mtxProtMs;     /* PROT: requested receiver-commit time, ms */
    uint8_t           mtxPhase;      /* MTXP_* */
    TickType_t        mtxDeadline;   /* PSA: give up carrier-sense at this tick */
    volatile uint32_t mtxResGen;     /* bumped when a request completes */
    bool              mtxResOk;      /* last request's success */
    char              mtxResMsg[72]; /* human-readable outcome for the CLI echo */
};

static LoraRadio     s_radios[kNumRadios];
/* Announce buffer + radio check: taps and hooks live in paths defined above
 * their implementations (applyConfig, handleRxDone), so declare them here. */
static uint32_t annNextGap(const LoraRadio* r);
static bool     annOfferTx(LoraRadio* r, const uint8_t* pkt, size_t len);
static void     rcOnBatch(LoraRadio* r, const uint8_t* f, size_t len, int16_t rssi);
static void     rcOnSweep(LoraRadio* r, const uint8_t* f, float rssi);
static void     rcPoll(LoraRadio* r);
static uint32_t rcTailMs(const LoraRadio* r, int n);
static int      annCount(const LoraRadio* r);
static void     apSettle(LoraRadio* r, Neighbor* e, bool measured, int8_t rung);
/* RNode endpoint (implementation in its own section, below the unit bridge — it
 * needs the frequency/bandwidth range constants defined there). */
static void rnodeForwardData(LoraRadio* r, const uint8_t* data, size_t len, bool withStats);
static void rnodeSendReady(void);
static void rnodePump(void);
static void rnodeEchoFlush(void);
static void rnodeApplyTransports(void);
static void rnodeSettleOff(void);
static void rnodeDropSession(void);
static void rnsdInject(LoraRadio* r, const uint8_t* data, size_t len,
                       int16_t rssi, int16_t snr10);
static TaskHandle_t  s_task = nullptr;
static volatile bool s_stop = false;   /* rns stop → break the work loop and park */
static volatile bool s_parked = false; /* true while parked (stopped); loraStop waits on it */
/* Config apply is coalesced: a change arms a deadline and the task loop applies
 * everything at once when it falls due. A radio restart is the unit of work an
 * apply costs (radioStop + radioStart + a re-registration with rnsd), and a
 * configuration burst — a client's frequency, bandwidth, spreading factor and
 * power arriving as four separate writes, or the same typed as one CLI line —
 * would otherwise pay it once per key.
 *
 * The deadline is armed once and never pushed out by later changes: an
 * immovable deadline is what keeps a busy device from starving the apply
 * forever (storage.cpp's save timer documents the same trap). It can only be
 * pulled *in*, which cannot starve anything, and an RNode client's 0.25 s echo
 * validation window needs exactly that. */
#define LORA_CFG_COALESCE_MS 300
static volatile bool       s_cfgPend    = true;
static volatile TickType_t s_cfgDueTick = 0;
static volatile bool s_displayDirty = false;   /* an MHz/kHz display key was edited */

/* Ask for a config apply in at most `delayMs`. Keeps the earlier of the
 * requested and any already-pending deadline — see the coalescing note above. */
static void cfgArm(uint32_t delayMs) {
    TickType_t due = xTaskGetTickCount() + pdMS_TO_TICKS(delayMs);
    if (s_cfgPend && (int32_t)(due - s_cfgDueTick) >= 0) {
        if (s_task) xTaskNotifyGive(s_task);   /* pending and no sooner: just wake */
        return;
    }
    s_cfgDueTick = due;
    s_cfgPend    = true;
    if (s_task) xTaskNotifyGive(s_task);
}
static volatile bool     s_radioIrq  = false;  /* DIO1 fired; gate the chip SPI poll on it */
static volatile uint32_t s_radioIrqUs = 0;     /* µs stamp of the last DIO1 IRQ (truncated
                                                * esp_timer time; 32-bit so the store is
                                                * atomic) */

/* ─────────────── chip dispatch ───────────────
 *
 * The whole task loop is chip-agnostic: every runtime call (getIrqFlags,
 * setPacketReceivedAction, startReceive, readData, transmit, getRSSI/SNR,
 * sleep) is a PhysicalLayer virtual, so the per-radio state holds a
 * PhysicalLayer*. Only three things vary by chip and dispatch here:
 *   - construction (radioNew): which concrete class to `new`.
 *   - begin (radioBegin): each family's begin() takes a different argument set.
 *   - the chip's display name (chipName).
 * The RF switch (Module::setRfSwitchPins) and the IRQ wiring are uniform and
 * handled at the call sites, not here. */

static const char* chipName(LoraChip c) {
    switch (c) {
#define X(name, fam) case CHIP_##name: return #name;
        LORA_CHIPS(X)
#undef X
    }
    return "?";
}

static LoraFamily chipFamily(LoraChip c) {
    switch (c) {
#define X(name, fam) case CHIP_##name: return fam;
        LORA_CHIPS(X)
#undef X
    }
    return FAM_SX126X;
}

static PhysicalLayer* radioNew(LoraChip c, Module* m) {
    switch (c) {
#define X(name, fam) case CHIP_##name: return new name(m);
        LORA_CHIPS(X)
#undef X
    }
    return nullptr;
}

/* Human-readable RadioLib status code (RADIOLIB_ERR_* in TypeDef.h) for
 * the codes our begin/startReceive/transmit paths can hit. Call sites print the
 * raw code alongside so unlisted values stay searchable in RadioLib docs. */
static const char* rlErrName(int16_t st) {
    switch (st) {
        case RADIOLIB_ERR_NONE:                        return "ok";
        case RADIOLIB_ERR_UNKNOWN:                     return "unknown error";
        case RADIOLIB_ERR_CHIP_NOT_FOUND:              return "chip not found";
        case RADIOLIB_ERR_PACKET_TOO_LONG:             return "packet too long";
        case RADIOLIB_ERR_TX_TIMEOUT:                  return "tx timeout";
        case RADIOLIB_ERR_RX_TIMEOUT:                  return "rx timeout";
        case RADIOLIB_ERR_INVALID_BANDWIDTH:           return "invalid bandwidth";
        case RADIOLIB_ERR_INVALID_SPREADING_FACTOR:    return "invalid spreading factor";
        case RADIOLIB_ERR_INVALID_CODING_RATE:         return "invalid coding rate";
        case RADIOLIB_ERR_INVALID_FREQUENCY:           return "invalid frequency";
        case RADIOLIB_ERR_INVALID_OUTPUT_POWER:        return "invalid output power";
        case RADIOLIB_ERR_SPI_WRITE_FAILED:            return "SPI write failed";
        case RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH:     return "invalid preamble length";
        case RADIOLIB_ERR_WRONG_MODEM:                 return "wrong modem";
        case RADIOLIB_ERR_INVALID_FREQUENCY_DEVIATION: return "invalid frequency deviation";
        case RADIOLIB_ERR_INVALID_RX_BANDWIDTH:        return "invalid rx bandwidth";
        case RADIOLIB_ERR_INVALID_SYNC_WORD:           return "invalid sync word";
        case RADIOLIB_ERR_INVALID_TCXO_VOLTAGE:        return "invalid TCXO voltage";
        case RADIOLIB_ERR_SPI_CMD_TIMEOUT:             return "SPI cmd timeout";
        case RADIOLIB_ERR_SPI_CMD_INVALID:             return "SPI cmd invalid";
        case RADIOLIB_ERR_SPI_CMD_FAILED:              return "SPI cmd failed";
        default:                                       return "unknown";
    }
}

/* LR11x0's begin() takes neither frequency nor power — set them (and the TCXO)
 * after. `high` selects the 2.4 GHz front-end on parts that have one. */
static int16_t lr11x0Begin(LoraRadio* r, float freq, float bw, uint8_t sf, uint8_t cr,
                           uint8_t sync, int8_t power, uint16_t preamble, float tcxoV) {
    LR11x0* lr = static_cast<LR11x0*>(r->radio);
    int16_t st = lr->begin(bw, sf, cr, sync, preamble, /*high=*/freq >= 2000.0f);
    if (st == RADIOLIB_ERR_NONE && tcxoV > 0.0f) st = lr->setTCXO(tcxoV);
    if (st == RADIOLIB_ERR_NONE) st = lr->setFrequency(freq);
    if (st == RADIOLIB_ERR_NONE) st = lr->setOutputPower(power);
    return st;
}

/* begin() the radio with the common LoRa parameters. Each family's begin() has
 * a different signature: SX126x carries TCXO + regulator; SX127x a LNA-gain arm
 * (0 = AGC) and no TCXO; SX128x is 2.4 GHz and bare; LR11x0 sets freq/power
 * separately (lr11x0Begin); LR2021 takes everything including TCXO. We cast to
 * the concrete class (the pointer really is that class) so dispatch is correct
 * regardless of where each begin() sits in RadioLib's hierarchy. SX126x also
 * applies DIO2-as-RF-switch when the slot asks for it, and the LNA boosted-RX-gain
 * option (r->rxBoostedGain): ~+3 dB sensitivity for ~0.4 mA more RX current. */
static int16_t radioBegin(LoraRadio* r, float freq, float bw, uint8_t sf, uint8_t cr,
                          uint8_t sync, int8_t power, uint16_t preamble, float tcxoV) {
    PhysicalLayer* p = r->radio;
    int16_t st = RADIOLIB_ERR_UNKNOWN;
    switch (r->slot->chip) {
        case CHIP_SX1261: st = static_cast<SX1261*>(p)->begin(freq, bw, sf, cr, sync, power, preamble, tcxoV, false); break;
        case CHIP_SX1262: st = static_cast<SX1262*>(p)->begin(freq, bw, sf, cr, sync, power, preamble, tcxoV, false); break;
        case CHIP_SX1268: st = static_cast<SX1268*>(p)->begin(freq, bw, sf, cr, sync, power, preamble, tcxoV, false); break;
        case CHIP_LLCC68: st = static_cast<LLCC68*>(p)->begin(freq, bw, sf, cr, sync, power, preamble, tcxoV, false); break;
        case CHIP_SX1272: return static_cast<SX1272*>(p)->begin(freq, bw, sf, cr, sync, power, preamble, 0);
        case CHIP_SX1276: return static_cast<SX1276*>(p)->begin(freq, bw, sf, cr, sync, power, preamble, 0);
        case CHIP_SX1277: return static_cast<SX1277*>(p)->begin(freq, bw, sf, cr, sync, power, preamble, 0);
        case CHIP_SX1278: return static_cast<SX1278*>(p)->begin(freq, bw, sf, cr, sync, power, preamble, 0);
        case CHIP_SX1280: return static_cast<SX1280*>(p)->begin(freq, bw, sf, cr, sync, power, preamble);
        case CHIP_SX1281: return static_cast<SX1281*>(p)->begin(freq, bw, sf, cr, sync, power, preamble);
        case CHIP_SX1282: return static_cast<SX1282*>(p)->begin(freq, bw, sf, cr, sync, power, preamble);
        case CHIP_LR1110:
        case CHIP_LR1120:
        case CHIP_LR1121: return lr11x0Begin(r, freq, bw, sf, cr, sync, power, preamble, tcxoV);
        case CHIP_LR2021: return static_cast<LR2021*>(p)->begin(freq, bw, sf, cr, sync, power, preamble, tcxoV);
        default:          return RADIOLIB_ERR_UNKNOWN;
    }
    /* SX126x only: DIO2 drives the antenna RF switch, and the LNA RX gain mode. */
    if (st == RADIOLIB_ERR_NONE && r->slot->dio2_rf_switch)
        st = static_cast<SX126x*>(p)->setDio2AsRfSwitch(true);
    if (st == RADIOLIB_ERR_NONE && chipFamily(r->slot->chip) == FAM_SX126X)
        st = static_cast<SX126x*>(p)->setRxBoostedGainMode(r->rxBoostedGain);
    return st;
}

/* ─────────────── storage key helpers (per radio) ─────────────── */

static const char* sk(char* b, size_t n, int i, const char* leaf) {
    snprintf(b, n, "s.lora.%d.%s", i, leaf); return b;
}
static const char* rk(char* b, size_t n, int i, const char* leaf) {
    snprintf(b, n, "lora.%d.%s", i, leaf); return b;
}

/* ─────────────── per-packet debug trace (`log lora debug`) ───────────────
 *
 * Decodes the Reticulum header of a whole (reassembled) RNS frame into a
 * one-line summary at debug level; at verbose level the entire frame is also
 * dumped as hex. This traces the RNS packet, not the on-air split frame — so
 * it runs at the reassembly boundary (deliverInbound for rx, beginTx for
 * tx), free of the local 1-byte split header.
 *
 * RNS wire header (RNS/Packet.py):
 *   byte0 flags: [IFAC 0x80][hdr2 0x40][ctxflag 0x20][transport 0x10]
 *                [dest-type 0x0C][packet-type 0x03]
 *   byte1 hops
 *   [IFAC access code: ifac_size bytes, present iff IFAC flag set]
 *   [transport-id: 16 bytes, present iff hdr2 (HEADER_2)]
 *   [destination hash: 16 bytes]
 *   [context: 1 byte]
 *   [data ...]
 */

static void loraHex(char* out, const uint8_t* d, size_t n) {
    static const char H[] = "0123456789abcdef";
    for (size_t i = 0; i < n; i++) { out[2*i] = H[d[i] >> 4]; out[2*i+1] = H[d[i] & 0xF]; }
    out[2*n] = '\0';
}

static const char* loraPktType(uint8_t t) {   /* flags & 0x03 */
    switch (t) { case 0: return "data"; case 1: return "announce";
                 case 2: return "linkreq"; default: return "proof"; }
}
static const char* loraDestType(uint8_t t) {   /* (flags >> 2) & 0x03 */
    switch (t) { case 0: return "single"; case 1: return "group";
                 case 2: return "plain"; default: return "link"; }
}
static const char* loraCtx(uint8_t c) {        /* context byte; NULL = unknown */
    switch (c) {
        case 0x00: return nullptr;                 /* none — omitted from the line */
        case 0x01: return "resource";
        case 0x02: return "resource-adv";
        case 0x03: return "resource-req";
        case 0x04: return "resource-hmu";
        case 0x05: return "resource-prf";
        case 0x06: return "resource-icl";
        case 0x07: return "resource-rcl";
        case 0x08: return "cache-req";
        case 0x09: return "request";
        case 0x0a: return "response";
        case 0x0b: return "path-resp";
        case 0x0c: return "command";
        case 0x0d: return "command-status";
        case 0x0e: return "channel";
        case 0xfa: return "keepalive";
        case 0xfb: return "link-identify";
        case 0xfc: return "link-close";
        case 0xfd: return "link-proof";
        case 0xfe: return "link-rtt";
        case 0xff: return "link-req-proof";
        default:   return nullptr;                 /* unknown → flag the line */
    }
}

/* Full frame as an offset-prefixed hexdump, 16 bytes/line, no ASCII. Verbose
 * only — the level check guards the per-row formatting cost. */
static void loraHexdump(int idx, const char* dir, const uint8_t* p, size_t len) {
    if (esp_log_level_get(TAG) < ESP_LOG_VERBOSE) return;
    for (size_t off = 0; off < len; off += 16) {
        char row[16 * 3 + 8];
        int o = snprintf(row, sizeof row, "%04x", (unsigned)off);
        for (size_t i = 0; i < 16 && off + i < len; i++)
            o += snprintf(row + o, sizeof row - (size_t)o, " %02x", p[off + i]);
        verb("lora/%d %s %s", idx, dir, row);
    }
}

/* Trace one whole RNS frame, gated on `log lora debug`. haveQual: fold in the
 * radio's last rx rssi/snr (rx); false on tx. logIsDebug short-circuits the
 * decode when off, so this is free on the hot path in normal operation.
 * Retained for future RNS-level inspection but no longer wired into rx/tx —
 * LoRaMon logs per on-air frame instead (loraMonPush). */
__attribute__((unused))
static void loraTracePacket(LoraRadio* r, const char* dir,
                            const uint8_t* p, size_t len, bool haveQual,
                            double airMs, int frames) {
    if (!logIsDebug(TAG)) return;

    char qual[24] = "";
    if (haveQual)
        snprintf(qual, sizeof qual, " %ddBm snr%.1f",
                 (int)r->rssiLast, (double)r->snrLast);

    /* Computed airtime of the on-air frame(s) at the live modem params; a
     * 2-frame split shows its per-frame count so the doubled preamble is clear. */
    char air[24] = "";
    if (frames == 2) snprintf(air, sizeof air, " air=%.0fms/2frm", airMs);
    else             snprintf(air, sizeof air, " air=%.0fms", airMs);

    bool   ifac   = len >= 1 && (p[0] & 0x80);
    bool   hdr2   = len >= 1 && (p[0] & 0x40);
    size_t ifacB  = ifac ? (r->curIfacSize ? r->curIfacSize : 1u) : 0;
    size_t addrB  = hdr2 ? 32 : 16;                 /* [transport 16] + dest 16 */
    size_t hdrEnd = 2 + ifacB + addrB + 1;          /* through the context byte */

    /* Too short to hold a header — decode nothing, show the first ≤10 bytes so
     * a stray/foreign frame is still visible. */
    if (len < 2 || len < hdrEnd) {
        size_t s = len < 10 ? len : 10;
        char hx[21];
        loraHex(hx, p, s);
        dbg("lora/%d %s%s%s <unparsed %uB> %s", r->idx, dir, qual, air, (unsigned)len, hx);
        loraHexdump(r->idx, dir, p, len);
        return;
    }

    uint8_t  flags     = p[0];
    uint8_t  hops      = p[1];
    bool     transport = flags & 0x10;              /* 0 broadcast, 1 transport */
    uint8_t  dtype     = (flags >> 2) & 0x03;
    uint8_t  ptype     = flags & 0x03;
    const uint8_t* dest = p + 2 + ifacB + (hdr2 ? 16 : 0);
    uint8_t  ctx       = p[2 + ifacB + addrB];

    char destHex[33]; loraHex(destHex, dest, 16);

    char via[48] = "";
    if (hdr2) { char v[33]; loraHex(v, p + 2 + ifacB, 16); snprintf(via, sizeof via, " via %s", v); }

    bool anomaly = hops > 128;                      /* RNS caps hops at 128 */
    char ctxbuf[24] = "";
    if (ctx != 0x00) {
        const char* cn = loraCtx(ctx);
        if (cn) snprintf(ctxbuf, sizeof ctxbuf, " ctx=%s", cn);
        else { snprintf(ctxbuf, sizeof ctxbuf, " ctx=0x%02x", ctx); anomaly = true; }
    }

    dbg("lora/%d %s%s%s %s %s %s %s%s%s hops=%u%s",
        r->idx, dir, qual, air,
        loraPktType(ptype), transport ? "to" : "bcast",
        loraDestType(dtype), destHex, via, ctxbuf,
        (unsigned)hops, anomaly ? " ?" : "");

    loraHexdump(r->idx, dir, p, len);
}

/* ─────────────── passive neighbour table: implementation ───────────────
 *
 * Everything below runs on the lora task (observe/expire) except the CLI
 * printer, which — like cliPrintSlot — reads live state cross-task without a
 * lock; a torn row during heavy traffic is acceptable for a diagnostic view. */

/* Decoded RNS wire header (layout in the loraTracePacket comment above). */
struct NeiHdr {
    uint8_t        hops, ptype, dtype, ctx;
    bool           hdr2;
    const uint8_t* transportId;   /* HEADER_2 only, else null */
    const uint8_t* dest;
    const uint8_t* data;
    size_t         dataLen;
};

/* IFAC frames (flag 0x80) are masked from byte 2 on — unparseable, skipped. */
static bool neiParse(const uint8_t* p, size_t len, NeiHdr* h) {
    if (len < 2 || (p[0] & 0x80)) return false;
    h->hdr2 = (p[0] & 0x40) != 0;
    size_t hdrEnd = 2 + (h->hdr2 ? 32 : 16) + 1;
    if (len < hdrEnd) return false;
    h->hops    = p[1];
    h->ptype   = p[0] & 0x03;
    h->dtype   = (p[0] >> 2) & 0x03;
    h->transportId = h->hdr2 ? p + 2 : nullptr;
    h->dest    = p + 2 + (h->hdr2 ? 16 : 0);
    h->ctx     = p[hdrEnd - 1];
    h->data    = p + hdrEnd;
    h->dataLen = len - hdrEnd;
    return true;
}

/* Truncated packet hash — what a proof is addressed to. Hashable part is
 * [flags & 0x0F] + raw[2:] (HEADER_1) / raw[18:] (HEADER_2, transport_id
 * excluded); for an LR the link_id additionally drops any data beyond the
 * 64-byte ephemeral keys (MTU signalling). Static buffer: lora task only. */
static void neiPacketHash(const NeiHdr* h, const uint8_t* p, size_t len,
                          bool isLr, uint8_t out[16]) {
    static uint8_t buf[1 + RNS_MTU + 16];
    size_t skip = h->hdr2 ? 18 : 2;
    size_t n = len - skip;
    if (isLr && h->dataLen > NEI_ECPUBSIZE) n -= h->dataLen - NEI_ECPUBSIZE;
    buf[0] = p[0] & 0x0F;
    memcpy(buf + 1, p + skip, n);
    uint8_t sha[RNSD_HASH_LEN];
    rnsdSha256(buf, 1 + n, sha);
    memcpy(out, sha, 16);
}

/* Known app.aspect name-hashes, to label dest rows. Computed once (name_hash =
 * SHA-256(expanded name)[:10], matching µR's Destination::expand_name). */
static const char* const kNeiNames[] = {
    "lxmf.delivery", "lxmf.propagation", "nomadnetwork.node",
    "rnstransport.probe", "rnsh",
};
static constexpr int kNeiNameCount = (int)(sizeof(kNeiNames) / sizeof(kNeiNames[0]));
static uint8_t s_neiNameHash[kNeiNameCount][10];

static void neiNamesInit(void) {
    for (int i = 0; i < kNeiNameCount; i++) {
        uint8_t sha[RNSD_HASH_LEN];
        rnsdSha256((const uint8_t*)kNeiNames[i], strlen(kNeiNames[i]), sha);
        memcpy(s_neiNameHash[i], sha, 10);
    }
}

static const char* neiNameLabel(const uint8_t nameHash[10]) {
    for (int i = 0; i < kNeiNameCount; i++)
        if (memcmp(s_neiNameHash[i], nameHash, 10) == 0) return kNeiNames[i];
    return nullptr;
}

/* ── table bookkeeping ── */

static Neighbor* neiFindByIdentity(NeiState* st, const uint8_t id[16]) {
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used) continue;
        for (int n = 0; n < e->nIds; n++)
            if (memcmp(e->ids[n], id, 16) == 0) return e;
    }
    return nullptr;
}

static Neighbor* neiFindByDest(NeiState* st, const uint8_t dest[16]) {
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used) continue;
        for (int d = 0; d < e->nDests; d++)
            if (memcmp(e->dests[d].hash, dest, 16) == 0) return e;
    }
    return nullptr;
}

/* A row that is one of this device's own two local endpoints — us, or the
 * attached RNode client — rather than a node out on the air. Every RF-layer
 * guard that means "this traffic terminates at our transmitter" tests this: a
 * packet addressed to the client's identities is delivered over the wire, so
 * the radio must no more probe, power-adapt or route toward it than toward
 * ourselves. Guards that mean "our own identities" keep plain isUs. */
static inline bool neiIsLocal(const Neighbor* e) { return e->isUs || e->isRnode; }

static bool neiDestIsLocal(NeiState* st, const uint8_t dest[16]) {
    Neighbor* e = neiFindByDest(st, dest);
    return e && neiIsLocal(e);
}

/* The row a linkage frame has attributed a first-4 to without the hash itself
 * ever having been heard: its node key, or an entry in a 0x03 list. Narrower
 * than neiFindBy4() on purpose — a full dest that merely shares its first four
 * bytes with an unrelated one is a collision, not the same device, so it must
 * not pull two rows together. */
static Neighbor* neiFindClaim4(NeiState* st, const uint8_t b4[4]) {
    if (!st) return nullptr;
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used) continue;
        if (e->haveNode4 && memcmp(e->node4, b4, 4) == 0) return e;
        for (int l = 0; l < e->nLink4; l++)
            if (memcmp(e->link4[l], b4, 4) == 0) return e;
    }
    return nullptr;
}

/* Allocate an entry, evicting the longest-unheard non-us one when full. */
static Neighbor* neiAlloc(NeiState* st, uint32_t now) {
    Neighbor* victim = nullptr;
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used) { victim = e; break; }
        if (neiIsLocal(e)) continue;
        if (!victim || (int32_t)(victim->lastHeardMs - e->lastHeardMs) > 0) victim = e;
    }
    if (!victim) return nullptr;
    memset(victim, 0, sizeof(*victim));
    victim->used = true;
    victim->lastHeardMs = now;
    return victim;
}

/* Drop the placeholder a linkage frame left for a first-4. Invariant: a hash is
 * on a row either as a dest or as a `link4` stub, never both — neiKnownHashes()
 * counts the two together, and the printer lists them as separate lines. */
static void neiDropLink4(Neighbor* e, const uint8_t b4[4]) {
    for (int l = 0; l < e->nLink4; l++) {
        if (memcmp(e->link4[l], b4, 4) != 0) continue;
        memmove(e->link4[l], e->link4[l + 1], (size_t)(e->nLink4 - 1 - l) * 4);
        e->nLink4--;
        return;
    }
}

static NeiDest* neiAddDest(Neighbor* e, const uint8_t dest[16], uint32_t now) {
    for (int d = 0; d < e->nDests; d++)
        if (memcmp(e->dests[d].hash, dest, 16) == 0) return &e->dests[d];
    neiDropLink4(e, dest);   /* the hash is heard now; the stub is redundant */
    NeiDest* nd;
    if (e->nDests < NEI_DESTS_MAX) nd = &e->dests[e->nDests++];
    else {                                       /* replace the stalest dest */
        nd = &e->dests[0];
        for (int d = 1; d < NEI_DESTS_MAX; d++)
            if ((int32_t)(nd->lastMs - e->dests[d].lastMs) > 0) nd = &e->dests[d];
    }
    memset(nd, 0, sizeof(*nd));
    memcpy(nd->hash, dest, 16);
    nd->lastMs = now;
    return nd;
}

/* Entry for a dest that provably transmitted to us direct but has never
 * announced (LRPROOF at hops 0, a proof of our packet) — identity unknown. */
static Neighbor* neiEnsureDest(NeiState* st, const uint8_t dest[16], uint32_t now) {
    Neighbor* e = neiFindByDest(st, dest);
    if (e) return e;
    /* A 0x03 may already have claimed this hash for a node — the dest belongs on
     * that row, not on a fresh one. Never across the us/them boundary: a peer's
     * linkage claim is unauthenticated. */
    e = neiFindClaim4(st, dest);
    if (e && !neiIsLocal(e)) { neiAddDest(e, dest, now); return e; }
    e = neiAlloc(st, now);
    if (e) neiAddDest(e, dest, now);
    return e;
}

/* One rx frame provably transmitted by this node: signal envelope + rollup. */
static void neiSample(Neighbor* e, int16_t rssi, int16_t snr10, uint32_t now) {
    if (!e->haveSig || rssi < e->rssiMin)   e->rssiMin  = rssi;
    if (!e->haveSig || rssi > e->rssiMax)   e->rssiMax  = rssi;
    if (!e->haveSig || snr10 < e->snrMin10) e->snrMin10 = snr10;
    if (!e->haveSig || snr10 > e->snrMax10) e->snrMax10 = snr10;
    e->haveSig = true;
    e->lastHeardMs = now;
    e->frames++;
    uint32_t absIdx = now / NEI_BUCKET_MS;
    NeiBucket* b = &e->buck[absIdx % NEI_BUCKETS];
    if (b->absIdx != absIdx) { b->absIdx = absIdx; b->cnt = 0; b->rssiSum = 0; b->snrSum10 = 0; }
    b->cnt++;
    b->rssiSum  += rssi;
    b->snrSum10 += snr10;
}

/* One resolved proof expectation: ratio counters + EWMA (α = 1/4). */
static void neiQuality(Neighbor* e, bool hit) {
    e->qSent++;
    if (hit) e->qProved++;
    uint8_t s = hit ? 255 : 0;
    if (!e->haveQuality) { e->quality = s; e->haveQuality = true; }
    else e->quality = (uint8_t)((3 * (int)e->quality + s + 2) / 4);
}

static NeiLink* neiLinkFind(NeiState* st, const uint8_t linkId[16]) {
    for (int i = 0; i < NEI_LINKS_MAX; i++)
        if (st->links[i].used && memcmp(st->links[i].linkId, linkId, 16) == 0)
            return &st->links[i];
    return nullptr;
}

static NeiLink* neiLinkEnsure(NeiState* st, const uint8_t linkId[16], uint32_t now) {
    NeiLink* L = neiLinkFind(st, linkId);
    if (L) return L;
    NeiLink* victim = nullptr;
    for (int i = 0; i < NEI_LINKS_MAX; i++) {
        L = &st->links[i];
        if (!L->used) { victim = L; break; }
        if (!victim || (int32_t)(victim->lastMs - L->lastMs) > 0) victim = L;
    }
    memset(victim, 0, sizeof(*victim));
    victim->used = true;
    memcpy(victim->linkId, linkId, 16);
    victim->lastMs = now;
    return victim;
}

static void neiPendAdd(NeiState* st, const uint8_t phash[16], const uint8_t dest[16],
                       bool isLR, bool counted, uint32_t now) {
    NeiPend* victim = nullptr;
    for (int i = 0; i < NEI_PEND_MAX; i++) {
        NeiPend* pd = &st->pend[i];
        if (!pd->used) { victim = pd; break; }
        if (!victim || (int32_t)(victim->deadlineMs - pd->deadlineMs) > 0) victim = pd;
    }
    victim->used = true;
    victim->isLR = isLR;
    victim->counted = counted;
    memcpy(victim->phash, phash, 16);
    memcpy(victim->dest, dest, 16);
    victim->deadlineMs = now + NEI_PROOF_TIMEOUT_MS;
}

/* Match-and-free a pend entry by the hash a proof is addressed to. The freed
 * slot's fields stay readable until the next neiPendAdd (single task). */
static NeiPend* neiPendTake(NeiState* st, const uint8_t phash[16]) {
    for (int i = 0; i < NEI_PEND_MAX; i++) {
        NeiPend* pd = &st->pend[i];
        if (pd->used && memcmp(pd->phash, phash, 16) == 0) { pd->used = false; return pd; }
    }
    return nullptr;
}

static Neighbor* neiFindBy4(NeiState* st, const uint8_t b4[4]);

static bool neiHasId(const Neighbor* e, const uint8_t id[16]) {
    for (int n = 0; n < e->nIds; n++)
        if (memcmp(e->ids[n], id, 16) == 0) return true;
    return false;
}

static void neiAddId(Neighbor* e, const uint8_t id[16]) {
    if (neiHasId(e, id)) return;
    if (e->nIds < NEI_IDS_MAX) { memcpy(e->ids[e->nIds++], id, 16); return; }
    memmove(e->ids[0], e->ids[1], (NEI_IDS_MAX - 1) * 16);
    memcpy(e->ids[NEI_IDS_MAX - 1], id, 16);
}

static void neiAddLink4Raw(Neighbor* e, const uint8_t b4[4]) {
    for (int d = 0; d < e->nDests; d++)
        if (memcmp(e->dests[d].hash, b4, 4) == 0) return;
    for (int l = 0; l < e->nLink4; l++)
        if (memcmp(e->link4[l], b4, 4) == 0) return;
    if (e->nLink4 < NEI_LINK4_MAX) { memcpy(e->link4[e->nLink4++], b4, 4); return; }
    memmove(e->link4[0], e->link4[1], (NEI_LINK4_MAX - 1) * 4);
    memcpy(e->link4[NEI_LINK4_MAX - 1], b4, 4);
}

/* Fold `src` into `dst` and free it. Used when two rows turn out to be one
 * device: an announce naming a dest-only row's identity, or a 0x03 asserting
 * that several hashes (and so several identities) are the same node. */
static void neiMergeInto(Neighbor* dst, Neighbor* src) {
    if (dst == src || !src->used) return;
    for (int n = 0; n < src->nIds; n++) neiAddId(dst, src->ids[n]);
    for (int i = 0; i < src->nDests; i++) {
        NeiDest* nd = neiAddDest(dst, src->dests[i].hash, src->dests[i].lastMs);
        nd->announces += src->dests[i].announces;
        if (src->dests[i].haveName) {
            memcpy(nd->nameHash, src->dests[i].nameHash, 10);
            nd->haveName = true;
        }
        if (!nd->name[0] && src->dests[i].name[0])
            safeStrncpy(nd->name, src->dests[i].name, sizeof nd->name);
    }
    for (int l = 0; l < src->nLink4; l++) neiAddLink4Raw(dst, src->link4[l]);
    if (src->haveSig) {
        if (!dst->haveSig || src->rssiMin < dst->rssiMin)   dst->rssiMin  = src->rssiMin;
        if (!dst->haveSig || src->rssiMax > dst->rssiMax)   dst->rssiMax  = src->rssiMax;
        if (!dst->haveSig || src->snrMin10 < dst->snrMin10) dst->snrMin10 = src->snrMin10;
        if (!dst->haveSig || src->snrMax10 > dst->snrMax10) dst->snrMax10 = src->snrMax10;
        dst->haveSig = true;
    }
    dst->frames  += src->frames;
    dst->qSent   += src->qSent;
    dst->qProved += src->qProved;
    if (!dst->haveQuality && src->haveQuality) { dst->quality = src->quality; dst->haveQuality = true; }
    if (!dst->haveTxPwr && src->haveTxPwr) { dst->txPwr = src->txPwr; dst->haveTxPwr = true; }
    /* A measured determination outranks an estimated one; otherwise first wins.
     * Losing it across a fold would make a node forget its power the moment a
     * 0x03 linked its rows, exactly as with txPwr above. */
    if (src->haveApPwr && (!dst->haveApPwr || (dst->apFromEst && !src->apFromEst))) {
        dst->apPwr     = src->apPwr;
        dst->apFromEst = src->apFromEst;
        dst->haveApPwr = true;
    }
    dst->provesData |= src->provesData;
    dst->transit    |= src->transit;
    dst->ourProto   |= src->ourProto;
    dst->isUs       |= src->isUs;
    dst->isRnode    |= src->isRnode;
    if (!dst->haveNode4 && src->haveNode4) { memcpy(dst->node4, src->node4, 4); dst->haveNode4 = true; }
    if (src->haveAdv && src->advHashes > dst->advHashes) {
        dst->haveAdv = true;
        dst->advHashes = src->advHashes;
    }
    dst->roaming |= src->roaming;
    if ((int32_t)(src->lastHeardMs - dst->lastHeardMs) > 0) dst->lastHeardMs = src->lastHeardMs;
    for (int i = 0; i < NEI_BUCKETS; i++) {
        NeiBucket* eb = &dst->buck[i];
        NeiBucket* db = &src->buck[i];
        if (!db->cnt) continue;
        if (eb->absIdx == db->absIdx) {
            eb->cnt += db->cnt; eb->rssiSum += db->rssiSum; eb->snrSum10 += db->snrSum10;
        } else if ((int32_t)(db->absIdx - eb->absIdx) > 0) {
            *eb = *db;
        }
    }
    src->used = false;
}

/* Display name out of an announce's app_data. LXMF wraps it in msgpack
 * (optionally behind a 32-byte ratchet); NomadNet and very old clients send raw
 * UTF-8. We only need the first element, so this is a deliberately small subset
 * of the parser lxmf/ carries — iface-lora talks to rnsd alone and must not
 * depend on a consumer straddle. */
static void neiParseName(const uint8_t* p, size_t n, char* out, size_t outsz) {
    out[0] = '\0';
    if (!p || !n) return;

    auto plausible = [&](size_t off, size_t len) {
        if (off >= n || !len) return false;
        for (size_t k = 0; k < len && off + k < n; k++) {
            uint8_t b = p[off + k];
            if (b == 0x7F || (b < 0x20 && b != '\t' && b != '\n' && b != '\r')) return false;
        }
        return true;
    };
    auto copy = [&](const uint8_t* q, size_t len) {
        if (len >= outsz) len = outsz - 1;
        memcpy(out, q, len);
        out[len] = '\0';
    };
    /* msgpack array whose first element is the name (str/bin/nil). */
    auto tryArray = [&](size_t i) {
        if (i >= n) return false;
        uint8_t b = p[i++];
        if (b >= 0x90 && b <= 0x9F) { if (!(b & 0x0F)) return false; }
        else if (b == 0xDC) { if (i + 2 > n) return false; i += 2; }
        else return false;
        if (i >= n) return false;
        uint8_t t = p[i++];
        size_t len;
        if (t == 0xC0) return true;                       /* nil name — valid, empty */
        else if (t >= 0xA0 && t <= 0xBF) len = t & 0x1F;   /* fixstr */
        else if (t == 0xD9 || t == 0xC4) { if (i >= n) return false; len = p[i++]; }
        else if (t == 0xDA || t == 0xC5) { if (i + 2 > n) return false; len = ((size_t)p[i] << 8) | p[i+1]; i += 2; }
        else return false;
        if (i + len > n) return false;
        copy(p + i, len);
        return true;
    };
    if (n >= 34 && tryArray(32)) return;
    if (tryArray(0)) return;
    if (n > 32 && plausible(32, n - 32)) { copy(p + 32, n - 32); return; }
    if (plausible(0, n)) copy(p, n);
}


/* ── announce ingest: the identity join ── */

static void neiAnnounce(LoraRadio* r, const NeiHdr* h, bool isTx,
                        int16_t rssi, int16_t snr10, uint32_t now,
                        uint8_t txOrigin) {
    NeiState* st = r->nei;
    /* pubkey(64) | name_hash(10) | random_hash(10) | signature(64) | app_data */
    if (h->dataLen < 64 + 10 + 10 + 64) return;
    const uint8_t* pub    = h->data;
    const uint8_t* nameH  = h->data + 64;
    const uint8_t* randH  = h->data + 74;
    const uint8_t* sig    = h->data + 84;
    const uint8_t* appD   = h->data + 148;
    size_t         appLen = h->dataLen - 148;

    /* The join is cryptographic or it is nothing: identity = H(pubkey)[:16],
     * the dest must equal H(name_hash ‖ identity)[:16], and the announce
     * signature must verify under that key — same checks as µR's
     * validate_announce, minus the cache. */
    uint8_t sha[RNSD_HASH_LEN], idh[16];
    rnsdSha256(pub, 64, sha);
    memcpy(idh, sha, 16);
    uint8_t mat[26];
    memcpy(mat, nameH, 10);
    memcpy(mat + 10, idh, 16);
    rnsdSha256(mat, 26, sha);
    if (memcmp(sha, h->dest, 16) != 0) return;
    static uint8_t sd[16 + 64 + 10 + 10 + RNS_MTU];   /* signed_data; lora task only */
    size_t o = 0;
    memcpy(sd + o, h->dest, 16); o += 16;
    memcpy(sd + o, pub, 64);     o += 64;
    memcpy(sd + o, nameH, 10);   o += 10;
    memcpy(sd + o, randH, 10);   o += 10;
    memcpy(sd + o, appD, appLen); o += appLen;
    if (!rnsdVerify(pub, sd, o, sig)) return;

    Neighbor* e = neiFindByIdentity(st, idh);
    Neighbor* d = neiFindByDest(st, h->dest);
    if (!e && d && d->nIds == 0) {
        /* Dest-only entry (seen via LRPROOF/proof before any announce) — the
         * announce names its identity now. */
        neiAddId(d, idh);
        e = d;
    } else if (e && d && e != d && d->nIds == 0) {
        /* Same device split across an identity entry and a dest-only entry. */
        neiMergeInto(e, d);
    }
    if (!e) {
        e = neiAlloc(st, now);
        if (!e) return;
    }
    neiAddId(e, idh);
    if (isTx) {
        if (txOrigin == LORA_ORIG_RNODE) e->isRnode = true;
        else                             e->isUs    = true;
    }
    if (neiIsLocal(e)) {
        /* Each of our own identities announces separately and so builds its own
         * row, but they are all one device by construction — no 0x03 needed,
         * and none would ever arrive, since we don't hear ourselves. The RNode
         * client's identities fold the same way into its own row. Per flag, so
         * the two local rows stay two. */
        for (int i = 0; i < NEI_MAX; i++) {
            Neighbor* o = &st->nei[i];
            if (o == e || !o->used) continue;
            if ((e->isUs && o->isUs) || (e->isRnode && o->isRnode)) neiMergeInto(e, o);
        }
    }

    /* A 0x03 may have claimed this dest before it ever announced, in which case
     * the claiming row and this one are one device: the linkage frame said so,
     * and the announce has now supplied the hash it only held a stub for. Fold,
     * so the aspect joins the node instead of starting a row of its own. Same
     * us/them guard as neiLink(): a peer's claim must not reach our row. */
    if (!neiIsLocal(e)) {
        Neighbor* c = neiFindClaim4(st, h->dest);
        if (c && c != e && !neiIsLocal(c)) neiMergeInto(e, c);
    }

    NeiDest* nd = neiAddDest(e, h->dest, now);
    memcpy(nd->nameHash, nameH, 10);
    nd->haveName = true;
    nd->announces++;
    nd->lastMs = now;
    {   /* Only the naming aspects carry a human name in app_data. */
        const char* lbl = neiNameLabel(nameH);
        if (lbl && (strcmp(lbl, "lxmf.delivery") == 0 ||
                    strcmp(lbl, "nomadnetwork.node") == 0)) {
            char nm[NEI_NAME_MAX];
            neiParseName(appD, appLen, nm, sizeof nm);
            if (nm[0]) safeStrncpy(nd->name, nm, sizeof nd->name);
        }
    }
    if (isTx) e->lastHeardMs = now;   /* keep the us row fresh; no rx signal */
    else      neiSample(e, rssi, snr10, now);
}

/* ── the per-packet observation tap ── */

static void neiObserve(LoraRadio* r, const uint8_t* p, size_t len, bool isTx,
                       int16_t rssi, int16_t snr10, uint8_t txOrigin) {
    NeiState* st = r->nei;
    if (!st) return;
    NeiHdr h;
    if (!neiParse(p, len, &h)) return;
    uint32_t now = millis();

    /* Any HEADER_2 frame arriving at hops > 0 names its relayer in the
     * transport_id — the node forwarded someone else's packet to us, which is
     * transport behaviour whether or not it was an announce. */
    if (!isTx && h.hdr2 && h.hops > 0) {
        Neighbor* tr = neiFindBy4(st, p + 2);
        if (tr && !tr->isUs) tr->transit = true;
    }

    if (!isTx) {
        /* In-band relay coupling: the truncated packet hash is invariant
         * across relaying (hops and transport_id are excluded from the
         * hashable part), so the same hash re-heard one hop higher means a
         * node in range repeated it RF→RF — transport mode, confirmed from
         * the outside. Our own tx never enters the ring, so relaying we do
         * ourselves doesn't self-count. */
        uint8_t ph[16];
        neiPacketHash(&h, p, len, false, ph);
        for (int i = 0; i < NEI_SEEN_MAX; i++) {
            NeiSeen* sn = &st->seen[i];
            if (sn->ms && (uint8_t)(sn->hops + 1) == h.hops &&
                now - sn->ms < NEI_SEEN_WIN_MS && memcmp(sn->hash, ph, 16) == 0) {
                st->anon.inbandRelays++;
                break;
            }
        }
        NeiSeen* sn = &st->seen[st->seenNext];
        st->seenNext = (uint8_t)((st->seenNext + 1) % NEI_SEEN_MAX);
        memcpy(sn->hash, ph, 16);
        sn->hops = h.hops;
        sn->ms = now ? now : 1;

        /* A relayed frame's transmitter is an in-range transport node even
         * when nothing names it. Rebroadcast announces (HEADER_2) are
         * attributed to their named transit row below; everything else
         * relayed lands in the aggregate anonymous-transit row. */
        if (h.hops >= 1 && !(h.ptype == NEI_PT_ANNOUNCE && h.hdr2)) {
            NeiAnon* a = &st->anon;
            if (!a->haveSig || rssi < a->rssiMin)    a->rssiMin  = rssi;
            if (!a->haveSig || rssi > a->rssiMax)    a->rssiMax  = rssi;
            if (!a->haveSig || snr10 < a->snrMin10)  a->snrMin10 = snr10;
            if (!a->haveSig || snr10 > a->snrMax10)  a->snrMax10 = snr10;
            a->haveSig = true;
            a->frames++;
            a->lastMs = now;
        }
    }

    switch (h.ptype) {

    case NEI_PT_ANNOUNCE:
        if (h.hops == 0) {
            neiAnnounce(r, &h, isTx, rssi, snr10, now, txOrigin);
        } else if (!isTx && h.hdr2) {
            /* A rebroadcast announce is the one hops>0 frame whose transmitter
             * IS named: the rebroadcaster stamps its own identity hash as the
             * HEADER_2 transport_id (that is how path tables learn first_hop).
             * Attribute the signal to that transit neighbour, keyed by identity
             * so its own hops-0 announces (if any) land in the same row. The
             * announce signature covers the originator, not the relayer, so
             * this is unverified — the same trust the path table places in it. */
            Neighbor* e = neiFindByIdentity(st, h.transportId);
            if (!e) {
                e = neiAlloc(st, now);
                if (e) {
                    neiAddId(e, h.transportId);
                }
            }
            if (e && !e->isUs) {
                e->transit = true;
                neiSample(e, rssi, snr10, now);
            }
        } else if (isTx && h.hdr2) {
            /* Our own rebroadcast stamps OUR transport identity as the
             * transport_id — the exact frame neighbours identify us by, so
             * learn "who we are" from it symmetrically. This is the only way
             * the transport identity surfaces here unless rnsd also hosts an
             * announcing destination on it (rnstransport.probe usually does,
             * in which case this merges into that us row and tags it). */
            Neighbor* e = neiFindByIdentity(st, h.transportId);
            if (!e) {
                e = neiAlloc(st, now);
                if (e) {
                    neiAddId(e, h.transportId);
                }
            }
            if (e) {
                if (txOrigin == LORA_ORIG_RNODE) e->isRnode = true;
                else                             e->isUs    = true;
                e->transit = true;
                e->lastHeardMs = now;
            }
        }
        break;

    case NEI_PT_LINKREQ: {
        if (h.dtype != NEI_DT_SINGLE || h.hops != 0) break;   /* relayed LR: transit, out of scope */
        uint8_t lid[16];
        neiPacketHash(&h, p, len, true, lid);
        NeiLink* L = neiLinkEnsure(st, lid, now);
        memcpy(L->dest, h.dest, 16);
        L->haveDest = true;
        L->unresolved = false;
        L->lastMs = now;
        L->frames++;
        if (isTx) {
            L->ours = true;
            /* We initiated: the LRPROOF will be addressed to the link_id.
             * Counted only if the dest is already a known direct neighbour. */
            Neighbor* e = neiFindByDest(st, h.dest);
            neiPendAdd(st, lid, h.dest, true, e && !e->isUs, now);
        } else {
            bool toUs = neiDestIsLocal(st, h.dest);
            if (toUs) L->ours = true;                      /* inbound dial to us */
            L->haveSig = true;                             /* initiator's setup signal */
            L->lastRssi = rssi;
            L->lastSnr10 = snr10;
            /* A power request prefixed to this LR is the initiator telling us
             * how loud our side of the session needs to be. We cannot name the
             * initiator — an LR carries no sender — but the link_id is a handle
             * both ends share, so the request rides it for the whole session.
             * Only for a link dialled to us: a request overheard on someone
             * else's link is not addressed to anything we will transmit on. */
            if (toUs && r->apRxSuggestPend) {
                L->suggestDbm  = r->apRxSuggest;
                L->haveSuggest = true;
                info("lora/%d power request: link %02x%02x%02x%02x -> reply at %d dBm",
                     r->idx, lid[0], lid[1], lid[2], lid[3], (int)r->apRxSuggest);
            }
        }
        break;
    }

    case NEI_PT_PROOF: {
        if (h.ctx == NEI_CTX_LRPROOF) {
            /* dest field = link_id; the transmitter is the link's destination. */
            NeiLink* L = neiLinkFind(st, h.dest);
            if (L) {
                L->established = true;
                L->lastMs = now;
                L->frames++;
                if (!isTx) { L->haveSig = true; L->lastRssi = rssi; L->lastSnr10 = snr10; }
            }
            if (!isTx) {
                NeiPend* pd = neiPendTake(st, h.dest);
                if (h.hops == 0) {
                    const uint8_t* dh = (L && L->haveDest) ? L->dest
                                      : (pd ? pd->dest : nullptr);
                    if (dh && !neiDestIsLocal(st, dh)) {
                        Neighbor* e = neiEnsureDest(st, dh, now);
                        if (e) {
                            neiSample(e, rssi, snr10, now);
                            if (pd) neiQuality(e, true);
                        }
                    }
                }
                /* hops > 0: the LRPROOF came relayed — the dest is not a direct
                 * neighbour, so it neither samples nor scores quality (§5 of
                 * plans/adaptive-power.md: proof is end-to-end, power is
                 * first-hop). */
            }
        } else if (!isTx) {
            /* A delivery proof is addressed to the proved packet's truncated
             * hash — match it against what we elicited. */
            NeiPend* pd = neiPendTake(st, h.dest);
            if (pd && !pd->isLR && h.hops == 0 && !neiDestIsLocal(st, pd->dest)) {
                Neighbor* e = neiEnsureDest(st, pd->dest, now);
                if (e) {
                    e->provesData = true;   /* this dest proves plain data */
                    neiSample(e, rssi, snr10, now);
                    neiQuality(e, true);
                }
            }
        }
        break;
    }

    case NEI_PT_DATA: {
        if (h.dtype == NEI_DT_LINK) {
            NeiLink* L = neiLinkFind(st, h.dest);
            if (!L) {
                L = neiLinkEnsure(st, h.dest, now);
                L->unresolved = true;       /* mid-link traffic, setup missed */
            }
            if (isTx && h.hops == 0) L->ours = true;   /* we originate on it */
            L->lastMs = now;
            L->frames++;
            if (!isTx) {
                L->haveSig = true;
                L->lastRssi = rssi;
                L->lastSnr10 = snr10;
                /* On a link we initiated to a direct peer, every inbound frame
                 * at hops 0 is provably the peer (the dest) transmitting. */
                if (h.hops == 0 && L->ours && L->haveDest && !neiDestIsLocal(st, L->dest)) {
                    Neighbor* e = neiEnsureDest(st, L->dest, now);
                    if (e) neiSample(e, rssi, snr10, now);
                }
            }
        } else if (h.dtype == NEI_DT_SINGLE && isTx && h.hops == 0) {
            /* Our originated single-dest packet (probes included): if the dest
             * is a known direct neighbour, expect a proof back — the quality
             * elicitor. A miss counts only once the dest has proven before. */
            Neighbor* e = neiFindByDest(st, h.dest);
            if (e && !e->isUs) {
                uint8_t ph[16];
                neiPacketHash(&h, p, len, false, ph);
                neiPendAdd(st, ph, h.dest, false, e->provesData, now);
            }
        }
        break;
    }
    }
}

/* Expire overdue proof expectations (driven from the task loop; nextDeadline
 * wakes the task for the soonest outstanding deadline). */
static void neiExpire(LoraRadio* r, uint32_t now) {
    NeiState* st = r->nei;
    if (!st) return;
    for (int i = 0; i < NEI_PEND_MAX; i++) {
        NeiPend* pd = &st->pend[i];
        if (!pd->used || (int32_t)(now - pd->deadlineMs) < 0) continue;
        pd->used = false;
        if (pd->counted) {
            Neighbor* e = neiFindByDest(st, pd->dest);
            if (e) neiQuality(e, false);
        }
    }
}

/* ─────────────── ISR ─────────────── */

static IRAM_ATTR void loraRadioIsr(void) {
    /* Shared across radios — any DIO1 wakes the task, which polls each
     * radio's IRQ flags to find the one(s) that completed. The flag lets the
     * loop skip the chip poll on wakes that weren't a DIO1 (ITS / cfg / stats). */
    s_radioIrqUs = (uint32_t)esp_timer_get_time();   /* ISR-safe, µs-accurate */
    s_radioIrq = true;
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &hp);
    portYIELD_FROM_ISR(hp);
}

/* ─────────────── helpers ─────────────── */

/* Time-on-air (seconds) for a `payload`-byte LoRa frame, per Semtech
 * AN1200.13. Symbol time Tsym = 2^SF / BW; the preamble runs (n+4.25)
 * symbols and the payload rounds up into whole symbols, with low-data-rate
 * optimisation (DE) engaged once a symbol exceeds 16 ms. CRC is assumed on.
 *
 * `implicitHeader` is NOT optional bookkeeping: a headerless frame drops the
 * 20-bit header from the payload term, and the radio-check sweep also runs a
 * 6-symbol preamble instead of the configured 12. Computing a 4-byte sweep frame
 * as explicit/preamble-12 over-states its airtime by ~11 ms at SF7 — some 47% —
 * which lands in the LoRaMon bar widths, the rx start times (start = end − ToA)
 * and the airtime rollups. */
static double loraAirtimeSeconds(int sf, int bw_hz, int cr_denom,
                                 int preamble, int payload, bool implicitHeader) {
    if (sf <= 0 || bw_hz <= 0 || cr_denom < 5 || cr_denom > 8) return 0.0;
    double tSym = (double)((uint32_t)1 << sf) / (double)bw_hz;
    int    de   = (tSym > 0.016) ? 1 : 0;          /* low-data-rate optimize */
    int    cr   = cr_denom - 4;                    /* coded bits 1..4 */
    double num  = 8.0 * payload - 4.0 * sf + 28.0 + 16.0 /*CRC*/
                  - (implicitHeader ? 20.0 : 0.0);
    double den  = 4.0 * (sf - 2 * de);
    double payloadSym = 8.0 + fmax(ceil(num / den) * (cr + 4), 0.0);
    return (preamble + 4.25) * tSym + payloadSym * tSym;
}

/* Total on-air time (ms) for the frame(s) that carry one RNS packet, at the
 * radio's live SF/BW/CR/preamble. Each LoRa frame carries its own preamble,
 * header and CRC, so a split packet's airtime is the SUM of its frames' — never
 * one airtime over the combined length. frameLens[i] is each frame's full
 * on-air byte count (1-byte split header included). */
static double loraPacketAirtimeMs(const LoraRadio* r, const size_t* frameLens, int frames) {
    double s = 0.0;
    for (int i = 0; i < frames; i++)
        s += loraAirtimeSeconds(r->cfgSf, r->cfgBwHz, r->cfgCr, r->airPreamble,
                                (int)frameLens[i], r->airImplicit);
    return s * 1000.0;
}

/* Effective bps to register with rnsd. RNS derives its first-hop link
 * timeout as MTU*8/bitrate + 6 s, so registering bitrate = 4000/ceil(toa)
 * makes that term equal the (whole-second-rounded) airtime of one MTU — the
 * link establishment budget then tracks how long a 500-byte frame really
 * takes on this SF/BW/CR/preamble. */
static uint32_t computeBitrate(int sf, int bw_hz, int cr_denom, int preamble) {
    double toa = loraAirtimeSeconds(sf, bw_hz, cr_denom, preamble, RNS_MTU, false);
    if (toa <= 0.0) return 0;
    double secs = ceil(toa);
    if (secs < 1.0) secs = 1.0;
    return (uint32_t)((double)(RNS_MTU * 8) / secs);
}

static uint8_t modeFromString(const char* s) {
    if (!s || !*s)                      return RNS_IFACE_MODE_GATEWAY;
    if (strcmp(s, "full")         == 0) return RNS_IFACE_MODE_FULL;
    if (strcmp(s, "access_point") == 0) return RNS_IFACE_MODE_ACCESS_POINT;
    if (strcmp(s, "roaming")      == 0) return RNS_IFACE_MODE_ROAMING;
    if (strcmp(s, "boundary")     == 0) return RNS_IFACE_MODE_BOUNDARY;
    return RNS_IFACE_MODE_GATEWAY;
}

static const char* modeName(uint8_t m) {
    switch (m) {
        case RNS_IFACE_MODE_FULL:         return "full";
        case RNS_IFACE_MODE_GATEWAY:      return "gateway";
        case RNS_IFACE_MODE_ACCESS_POINT: return "access_point";
        case RNS_IFACE_MODE_ROAMING:      return "roaming";
        case RNS_IFACE_MODE_BOUNDARY:     return "boundary";
        default:                          return "?";
    }
}

/* Defined with the CSMA machine, below — needed by the telemetry flush and by
 * radio bring-up, both of which precede it. */
static void    csmaResetAccess(LoraRadio* r);
static void    csmaMediumHeld(LoraRadio* r, uint32_t durMs);
static float   appcAirtime(const LoraRadio* r);
static uint8_t appcLiveBand(const LoraRadio* r);

/* ─────────────── the two tasks ───────────────
 *
 * `lora` is the RADIO task: it owns the chip and nothing else may call
 * RadioLib. Sharing one task across all radios is deliberate — it serialises
 * the SPI bus by construction, so no sibling radio can slip a transaction into
 * a channel-access sequence. Everything it does is bounded, in-RAM work.
 *
 * `lora-if` is the INTERFACE task: the storage side. Publishing packet nodes,
 * expiring them, the stats flush, the RSSI series. It runs below the radio task
 * so it can never delay it.
 *
 * The line between them is **flash**. A storage write can erase a sector, and an
 * erase blocks the instruction cache for milliseconds — which is survivable for
 * telemetry and fatal for a channel-access deadline. Keeping every storage op
 * off the radio task is what makes the timing arguments in plans/psa.md hold.
 *
 * Traffic is one way, radio → interface, over a bounded queue that is never
 * allowed to block: a full queue drops the record and counts it. Telemetry
 * yielding under pressure is correct; a recorder that can stall a transmit is
 * not.
 *
 * The neighbour table — including its inline Ed25519 announce verification —
 * is still on the radio task. It wants to move for the same reason, but it is
 * read cross-task by the CLI, the probe and the adaptive-power path, so its
 * ownership has to be settled first. */

enum : uint8_t {
    IFM_MON  = 0,   /* one on-air frame → a packet node */
    IFM_RSSI = 1,   /* one channel-RSSI sample */
};

struct IfMsg {
    uint8_t  kind;
    uint8_t  radio;
    uint8_t  dir;        /* MON: 0 rx, 1 tx */
    uint8_t  type;       /* MON: LORA_PKT_* */
    uint8_t  ch;         /* channel index; 0 = the reticulum hailing channel */
    int8_t   txp;        /* MON tx: power of the frame */
    uint32_t t_ms;       /* MON: frame start; RSSI: sample time */
    uint16_t dur_ms;     /* MON: time on air */
    uint16_t bytes;      /* MON: payload bytes */
    uint16_t wait_ms;    /* MON tx: queue latency before the first bit */
    int16_t  rssi;       /* MON rx: dBm; RSSI: channel 0's reading */
    int16_t  snr10;      /* MON rx: deci-dB */
    uint8_t  nch;        /* RSSI: how many of chRssi carry a reading */
    int16_t  chRssi[LORA_CH_MAX];   /* RSSI: dBm per channel, index = channel */
};

/* Deep enough for a whole radio check (2 BATCH + 7 + 14 sweep frames) to land
 * inside ~500 ms without the storage side having to keep up frame for frame.
 * At 24 it overflowed at the end of every run, dropping the last records —
 * which reads as "the last frames were never sent". */
#define LORA_IFQ_DEPTH 48
static QueueHandle_t s_ifq = nullptr;

/* Cached by the interface task at 1 Hz so the radio task can gate recording on
 * it without a storage read of its own. */
static volatile bool s_monWatched = false;

/* Post to the interface task. Never blocks: a full queue means the storage side
 * is behind, and dropping telemetry is the correct answer. Returns false so the
 * caller can count the loss against the radio it belongs to. */
static bool ifPost(const IfMsg* m) {
    if (!s_ifq) return false;
    return xQueueSend(s_ifq, m, 0) == pdTRUE;
}

static void publishStats(LoraRadio* r) {
    /* Skip the churn on a headless, WiFi-down node — nothing pulls these keys
     * there. A UI (web over WiFi, or an LCD build) re-populates them when it
     * appears; see uiTelemetryWanted(). */
    if (!uiTelemetryWanted()) return;
    char b[48];
    /* One bracket → one storage op. Unbracketed this fired ~8 separate sync
     * round-trips to the storage task every second; under an inbound-message
     * burst those pile up on the storage op port and stall the radio task. */
    storageBegin();
    storageSet(rk(b, sizeof b, r->idx, "stats.tx_bytes"),  (int)(r->txBytes & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.rx_bytes"),  (int)(r->rxBytes & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.tx_frames"), (int)(r->txFrames & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.rx_frames"), (int)(r->rxFrames & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.crc_err"),   (int)(r->crcErr & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.split_rx_timeout"), (int)(r->splitTimeouts & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.tx_dropped"), (int)(r->txDropped & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.rssi_last"), (int)r->rssiLast);
    storageSet(rk(b, sizeof b, r->idx, "stats.snr_last"),  (int)r->snrLast);
    /* APPC: what the contention window is being drawn from right now. Both move
     * only on a transmit, so they belong in the same event-driven flush. */
    if (r->appc) {
        storageSet(rk(b, sizeof b, r->idx, "stats.airtime_pct"),
                   (int)(appcAirtime(r) * 100.0f));
        storageSet(rk(b, sizeof b, r->idx, "stats.cw_band"), (int)appcLiveBand(r));
    }
    storageEnd();
}

/* ─────────────── LoRaMon: recording, windows, publish, ITS server ─────────────── */

/* True while a LoRaMon viewer (web or LCD) is open — gates recording. */
static bool loraMonWatched(void) {
    return storageGetInt("sys.stats.web_loramon", 0) ||
           storageGetInt("sys.stats.lcd_loramon", 0);
}

/* Delete published packet nodes older than the 1-hour window, and enforce the
 * FIFO cap. The FIFO holds start-ms oldest-first; pop + delete from the front.
 * Interface task only — it owns the FIFO and every packet node in storage. */
static void loraMonExpire(LoraRadio* r, uint32_t now) {
    if (!r->pktMs) return;
    while (r->pktCount) {
        uint32_t oldest = r->pktMs[r->pktHead];
        bool aged = (now - oldest) > 3600u * 1000;      /* > 1 h (u32 diff, wrap-safe) */
        bool full = r->pktCount >= r->pktCap;           /* backstop against a flood */
        if (!aged && !full) break;
        char key[40];
        snprintf(key, sizeof key, "lora.%d.packets.%u", r->idx, (unsigned)oldest);
        storageDeleteTree(key);
        r->pktHead = (uint16_t)((r->pktHead + 1) % r->pktCap);
        r->pktCount--;
    }
}

/* Publish one packet node `lora.<n>.packets.<ms>` holding a packed string:
 * "r|rssi|snr|dur|bytes|type|ch" (rx) or "t|txp|dur|bytes|type|wait|ch" (tx).
 * The leading token is the direction; snr is deci-dB; ch is the channel the
 * frame flew on, 0 being the reticulum hailing channel. Then age old nodes out.
 *
 * INTERFACE TASK ONLY — this is the storage half of a record. */
static void loraMonRecord(LoraRadio* r, const IfMsg* m) {
    if (!r->pktMs || !r->pktCap) return;
    char key[40], val[56];
    snprintf(key, sizeof key, "lora.%d.packets.%u", r->idx, (unsigned)m->t_ms);
    if (m->dir) snprintf(val, sizeof val, "t|%d|%u|%u|%u|%u|%u",
                         (int)m->txp, (unsigned)m->dur_ms, (unsigned)m->bytes,
                         (unsigned)m->type, (unsigned)m->wait_ms, (unsigned)m->ch);
    else        snprintf(val, sizeof val, "r|%d|%d|%u|%u|%u|%u",
                         (int)m->rssi, (int)m->snr10, (unsigned)m->dur_ms,
                         (unsigned)m->bytes, (unsigned)m->type, (unsigned)m->ch);
    storageSet(key, val);

    loraMonExpire(r, m->t_ms);                           /* age out + free a slot if full */
    r->pktMs[(r->pktHead + r->pktCount) % r->pktCap] = m->t_ms;
    r->pktCount++;
}

/* Record one on-air frame. RADIO TASK: the in-RAM rollups, the debug line, and
 * a hand-off to the interface task for the storage node. Nothing here touches
 * flash.
 *
 * `wait_ms` is tx-only and belongs to the FIRST frame of a burst — the frames
 * behind it followed immediately and waited for nothing. */
static void loraMonPush(LoraRadio* r, uint8_t dir, uint32_t t_ms, uint16_t dur_ms,
                        uint16_t bytes, int16_t rssi, int16_t snr10, int8_t txp,
                        uint8_t type, uint16_t wait_ms) {
    /* Airtime rollup runs whether or not a viewer is open — the hour it covers
     * is longer than a viewer is typically up, so it can't be built on demand. */
    {
        uint32_t absIdx = t_ms / AIR_BUCKET_MS;
        AirBucket* b = &r->air[absIdx % AIR_BUCKETS];
        if (b->absIdx != absIdx) { b->absIdx = absIdx; b->rxMs = b->txMs = 0; }
        if (dir) b->txMs += dur_ms; else b->rxMs += dur_ms;
    }
    /* Transmit seconds per channel over the rolling hour — the figure the
     * airtime ledger will gate on once anything is ever transmitted on a
     * channel other than the hailing one. */
    if (dir) r->txAir[LORA_CH_HAIL].add((float)dur_ms / 1000.0f);
    if (logIsDebug(TAG)) {
        if (dir) dbg("lora/%d tx %u..%u (%ums) %uB txp=%ddBm waited=%ums",
                     r->idx, (unsigned)t_ms, (unsigned)(t_ms + dur_ms),
                     (unsigned)dur_ms, (unsigned)bytes, (int)txp,
                     (unsigned)wait_ms);
        else     dbg("lora/%d rx %u..%u (%ums) %uB rssi=%d snr=%.1f",
                     r->idx, (unsigned)t_ms, (unsigned)(t_ms + dur_ms),
                     (unsigned)dur_ms, (unsigned)bytes, (int)rssi, (double)snr10 / 10.0);
    }
    if (!s_monWatched) return;

    IfMsg m = {};
    m.kind = IFM_MON;  m.radio = (uint8_t)r->idx;
    m.dir  = dir;      m.type  = type;
    m.ch   = LORA_CH_HAIL;
    m.txp  = txp;      m.t_ms  = t_ms;
    m.dur_ms = dur_ms; m.bytes = bytes; m.wait_ms = wait_ms;
    m.rssi = rssi;     m.snr10 = snr10;
    if (!ifPost(&m)) r->monDropped++;
}

/* Publish the rolling one-hour airtime, per mille, per direction. The apps
 * compute every shorter window from the frame records; the hour needs more
 * history than a viewer holds, so it is the one figure the device publishes. */
static void loraPublishAirtime(LoraRadio* r, uint32_t now) {
    uint32_t absNow = now / AIR_BUCKET_MS;
    uint64_t rx = 0, tx = 0;
    for (int i = 0; i < AIR_BUCKETS; i++) {
        const AirBucket* b = &r->air[i];
        if ((b->rxMs || b->txMs) && absNow - b->absIdx < AIR_BUCKETS) {
            rx += b->rxMs;
            tx += b->txMs;
        }
    }
    char kb[48];
    storageBegin();
    storageSet(rk(kb, sizeof kb, r->idx, "air1h.rx"), (int)(rx * 1000 / 3600000u));
    storageSet(rk(kb, sizeof kb, r->idx, "air1h.tx"), (int)(tx * 1000 / 3600000u));
    storageEnd();
}

/* Drop all of a radio's published nodes + FIFO (last viewer closed). */
static void loraMonClear(LoraRadio* r) {
    char pfx[32];
    snprintf(pfx, sizeof pfx, "lora.%d.packets", r->idx);
    storageDeleteTree(pfx);
    r->pktHead = r->pktCount = 0;
}

/* Publish the channel list the regime puts in force:
 * `lora.<n>.chans` = "<freqHz>,<bwHz>|…", index = channel, 0 = hailing.
 *
 * One key rather than a subtree: it is a handful of numbers that only change on
 * a config apply, and the viewers want all of it at once to label their graphs.
 * With no agility in force it is just the hailing channel, so a viewer can tell
 * the two cases apart by the entry count alone and needs no separate flag. */
static void publishChannels(LoraRadio* r) {
    int n = 0;
    const RegimeChan* ch = regimeChans(r->afa, &n);
    char val[24 * LORA_CH_MAX];
    int  w = snprintf(val, sizeof val, "%u,%u",
                      (unsigned)r->cfgFreqHz, (unsigned)r->cfgBwHz);
    for (int i = 0; i < n && i + 1 < LORA_CH_MAX && w > 0 && w < (int)sizeof val; i++)
        w += snprintf(val + w, sizeof val - w, "|%u,%u",
                      (unsigned)ch[i].freqHz, (unsigned)ch[i].bwHz);
    char kb[48];
    storageSet(rk(kb, sizeof kb, r->idx, "chans"), val);
}

static void publishState(LoraRadio* r, const char* state) {
    char b[48];
    storageBegin();
    storageSet(rk(b, sizeof b, r->idx, "state"), state);
    storageSet(rk(b, sizeof b, r->idx, "up"), r->running ? 1 : 0);
    storageEnd();
}

/* ─────────────── rnsd registration ─────────────── */

static void onRnsdRecv(int handle, size_t bytesAvail);
static void onRnsdDisconnect(int ref);

static LoraRadio* radioByHandle(int h) {
    for (int i = 0; i < kNumRadios; i++)
        if (s_radios[i].rnsdHandle == h) return &s_radios[i];
    return nullptr;
}

static void deregisterFromRnsd(LoraRadio* r) {
    if (r->rnsdHandle >= 0) {
        itsDisconnect(r->rnsdHandle);
        r->rnsdHandle = -1;
    }
}

static bool registerWithRnsd(LoraRadio* r) {
    deregisterFromRnsd(r);
    rnsd_iface_t reg = {};
    snprintf(reg.name, sizeof(reg.name), "lora/%d", r->idx);
    reg.mtu     = RNS_MTU;
    reg.bitrate = r->curBitrate;
    reg.mode    = r->curMode;
    reg.in = reg.out = 1;
    reg.fwd = (r->curMode == RNS_IFACE_MODE_FULL || r->curMode == RNS_IFACE_MODE_GATEWAY) ? 1 : 0;
    reg.rpt = 0;
    reg.ifac_size = r->curIfacSize;
    reg.announce_cap = r->curAnnounceCap;
    reg.rx_signal = 1;   /* inbound data frames carry the 4-byte RSSI/SNR prefix */
    safeStrncpy(reg.ifac_netname, r->curIfacNetname, sizeof(reg.ifac_netname));
    safeStrncpy(reg.ifac_netkey,  r->curIfacNetkey,  sizeof(reg.ifac_netkey));
    /* ref = radio index — onRnsdDisconnect uses it to find the radio. */
    r->rnsdHandle = itsConnect("rnsd", RNSD_PORT_IFACE, &reg, sizeof(reg),
                               pdMS_TO_TICKS(500), r->idx,
                               onRnsdRecv, onRnsdDisconnect);
    if (r->rnsdHandle < 0) {
        warn("lora/%d rnsd register failed", r->idx);
        return false;
    }
    info("registered as iface lora/%d (mtu=%u bitrate=%u mode=%s)",
         r->idx, (unsigned)RNS_MTU, (unsigned)r->curBitrate, modeName(r->curMode));
    return true;
}

static void onRnsdDisconnect(int ref) {
    if (ref >= 0 && ref < kNumRadios) s_radios[ref].rnsdHandle = -1;
    /* The task loop will re-register if the radio is still enabled. */
}

/* ─────────────── radio control ─────────────── */

static void radioStop(LoraRadio* r) {
    pmGpioWakeDisable(r->slot->dio1);
    r->radio->clearPacketReceivedAction();
    /* Sleep the radio; we always re-apply the full config in radioStart, so the
     * (chip-specific) config-retention mode doesn't matter here. */
    r->radio->sleep();
    r->running = false;
    r->splitPending = false;
    r->splitLen = 0;
    r->txActive = false;   /* any in-flight transmit is abandoned with the radio */
    r->txFromRnode = false;
    /* A radio check in flight dies with the radio. No RF restore: the chip is
     * going down, and the modem regime is re-applied wholesale on the way up. */
    if (r->rc.phase != RC_OFF) {
        if (r->rc.timer) esp_timer_stop(r->rc.timer);
        r->rc.phase     = RC_OFF;
        r->rc.step      = 0;
        r->rc.savedSync = 0;
    }
    if (r->nei)            /* proofs can't return while RF is down — drop, uncounted */
        for (int i = 0; i < NEI_PEND_MAX; i++) r->nei->pend[i].used = false;
    deregisterFromRnsd(r);
    publishState(r, "down");
}

static bool radioStart(LoraRadio* r) {
    char kb[48];
    int freq_hz  = storageGetInt(sk(kb, sizeof kb, r->idx, "frequency"), 0);
    int bw_hz    = storageGetInt(sk(kb, sizeof kb, r->idx, "bandwidth"), 0);
    int sf       = storageGetInt(sk(kb, sizeof kb, r->idx, "spreading_factor"), 0);
    int cr       = storageGetInt(sk(kb, sizeof kb, r->idx, "coding_rate"), 0);
    int txp      = storageGetInt(sk(kb, sizeof kb, r->idx, "tx_power"), 0);
    int preamble = storageGetInt(sk(kb, sizeof kb, r->idx, "preamble"), 12);

    /* Sync word is stored as a string so the panel can accept hex like
     * "0x42" alongside plain decimal. strtol(base=0) handles both. */
    char syncBuf[16] = "";
    storageGetStr(sk(kb, sizeof kb, r->idx, "sync_word"), syncBuf, sizeof(syncBuf), "0x42");
    int syncWord = (int)strtol(syncBuf, nullptr, 0);
    if (syncWord <= 0 || syncWord > 0xFF) syncWord = 0x42;

    if (freq_hz <= 0 || bw_hz <= 0 || sf < 5 || sf > 12 ||
        cr < 5 || cr > 8 || txp < -9 || txp > 22) {
        info("lora/%d not started: configure freq/bw/sf/cr/txp first", r->idx);
        publishState(r, "unconfigured");
        return false;
    }

    /* RadioLib takes frequency in MHz and bandwidth in kHz; TCXO in volts. */
    float freq_mhz = (float)freq_hz / 1.0e6f;
    float bw_khz   = (float)bw_hz   / 1.0e3f;
    float tcxo_v   = (float)r->slot->tcxo_mv / 1000.0f;

    /* SX126x LNA boosted RX gain: ~+3 dB sensitivity for ~0.4 mA more RX current.
     * On by default; radioBegin applies it. Read before begin so it takes effect
     * in the same bring-up. */
    r->rxBoostedGain = storageGetInt(sk(kb, sizeof kb, r->idx, "rx_boosted_gain"), 1) != 0;

    int16_t st = radioBegin(r, freq_mhz, bw_khz, (uint8_t)sf, (uint8_t)cr,
                            (uint8_t)syncWord, (int8_t)txp, (uint16_t)preamble, tcxo_v);
    if (st != RADIOLIB_ERR_NONE) {
        /* SPI cmd timeout is often TCXO/PLL. radioBegin also applies the
         * DIO2-as-RF-switch option (SX126x) before returning. */
        err("lora/%d %s begin failed: %s (%d)",
            r->idx, chipName(r->slot->chip), rlErrName(st), (int)st);
        publishState(r, "error");
        return false;
    }

    /* CSMA/LBT: derive the slot time from the LoRa symbol time (2^SF / BW),
     * clamped to a sane range; DIFS is two slots. Enabled by default; a
     * per-radio toggle (s.lora.<i>.lbt=0) falls back to blind transmit. */
    double tSymMs = (double)((uint32_t)1 << sf) / (double)bw_hz * 1000.0;
    uint32_t slotMs = (uint32_t)(tSymMs + 0.5);
    if (slotMs < CSMA_SLOT_MS_MIN) slotMs = CSMA_SLOT_MS_MIN;
    if (slotMs > CSMA_SLOT_MS_MAX) slotMs = CSMA_SLOT_MS_MAX;
    r->slotTicks = pdMS_TO_TICKS(slotMs);
    if (r->slotTicks < 1) r->slotTicks = 1;
    r->difsTicks = 2 * r->slotTicks;
    r->lbt = storageGetInt(sk(kb, sizeof kb, r->idx, "lbt"), 1) != 0;
    /* Hidden safety valve: if LBT can't win the channel within lbt_timeout ms the
     * frame is dropped rather than backing off forever. 0 = never drop. */
    r->lbtTimeoutMs = (uint32_t)storageGetInt(sk(kb, sizeof kb, r->idx, "lbt_timeout"), 5000);
    r->lbtTimeoutTicks = r->lbtTimeoutMs ? pdMS_TO_TICKS(r->lbtTimeoutMs) : 0;
    r->csmaCw = CSMA_CW_MIN;
    r->csmaStalled = false;
    r->noiseFloor = CSMA_NOISE_FLOOR_DBM;

    /* APPC slot/DIFS. Upstream's "fast rate" test is against the nominal LoRa
     * bitrate, which is not curBitrate — that one is deliberately distorted to
     * shape the RNS link timeout, so the physical figure is computed here. Both
     * the truncation to int and the clamp asymmetry (compare against
     * APPC_SLOT_MIN_MS, assign the fast-rate floor) are upstream's. */
    uint32_t nominalBps = (uint32_t)((double)sf * (4.0 / (double)cr) / tSymMs * 1000.0);
    int appcSlotFloorMs = APPC_SLOT_MIN_MS;
    if (nominalBps > APPC_FAST_THRESHOLD_BPS) appcSlotFloorMs -= APPC_SLOT_MIN_FAST_DELTA;
    int appcSlotMs = (int)(tSymMs * APPC_SLOT_SYMBOLS);
    if (appcSlotMs > APPC_SLOT_MAX_MS) appcSlotMs = APPC_SLOT_MAX_MS;
    if (appcSlotMs < APPC_SLOT_MIN_MS) appcSlotMs = appcSlotFloorMs;
    r->appcSlotTicks = pdMS_TO_TICKS(appcSlotMs);
    if (r->appcSlotTicks < 1) r->appcSlotTicks = 1;
    r->appcDifsTicks = pdMS_TO_TICKS(APPC_SIFS_MS) + 2 * r->appcSlotTicks;
    r->appc = storageGetInt(sk(kb, sizeof kb, r->idx, "appc"), 1) != 0;
    r->appcBand    = 1;
    r->appcBinIdx  = (millis() % APPC_HOUR_MS) / APPC_BIN_MS;
    r->appcBinCur  = 0;
    r->appcBinPrev = 0;
    csmaResetAccess(r);

    /* Non-blocking TX watchdog: 2.5× the airtime of a full frame (+100 ms floor)
     * — long enough never to fire in normal operation, short enough that a wedged
     * transmit can't pin the outbound queue. */
    double maxToa = loraAirtimeSeconds(sf, bw_hz, cr, preamble, RNODE_MAX_PAYLOAD, false);
    r->txWatchTicks = pdMS_TO_TICKS((uint32_t)(maxToa * 1000.0 * 2.5) + 100);
    r->txActive = false;

    /* Mode for RNS iface registration. LoRa defaults to access_point (edge
     * segment); see straddle.yaml for why full/gateway are opt-in-by-hand. */
    char mode[24] = "access_point";
    storageGetStr(sk(kb, sizeof kb, r->idx, "mode"), mode, sizeof(mode), "access_point");
    r->curMode    = modeFromString(mode);
    r->curBitrate = computeBitrate(sf, bw_hz, cr, preamble);
    r->cfgSf = sf; r->cfgBwHz = bw_hz; r->cfgCr = cr; r->cfgPreamble = preamble;
    r->cfgFreqHz = (uint32_t)freq_hz;
    r->airPreamble = preamble; r->airImplicit = false; r->airSf = (uint8_t)sf;
    r->cfgTxp = (int8_t)txp;
    r->cfgSync = (uint8_t)syncWord;
    r->txPwrNow = (int8_t)txp;

    /* The regime in force. The key's value IS the regime number, so 0 means no
     * agility at all rather than "regime 0" — see the regime table. An unknown
     * number resolves to no agile channels, which is the safe reading: measure
     * and transmit on the hailing channel only. */
    r->afa = (uint8_t)storageGetInt(sk(kb, sizeof kb, r->idx, "afa"), 0);
    r->annIntervalMin = (uint16_t)storageGetInt(
        sk(kb, sizeof kb, r->idx, "announce_interval"), ANN_INTERVAL_DEF);
    publishChannels(r);

    /* Adaptive TX power. Determinations already made live in the neighbour
     * table, which survives a config cycle, so turning the key back on resumes
     * with what was measured rather than re-probing the mesh. */
    r->adaptive = storageGetInt(sk(kb, sizeof kb, r->idx, "adaptive_txpwr"), 0) != 0;

    /* LoRaMon expiry FIFO: allocated once, kept across config cycles. */
    if (!r->pktMs) {
        r->pktMs = (uint32_t*)gp_alloc((size_t)LORA_MON_CAP * sizeof(uint32_t));
        r->pktCap = r->pktMs ? LORA_MON_CAP : 0;
        r->pktHead = r->pktCount = 0;
    }

    /* Announce buffer: allocated once, kept across config cycles like the
     * neighbour table. A failed alloc leaves the replay off (every path guards
     * on ann) without disturbing anything else. */
    if (!r->ann) {
        r->ann = (AnnBuf*)gp_alloc(sizeof(AnnBuf));
        if (r->ann) std::memset(r->ann, 0, sizeof(AnnBuf));
    }
    /* First beat one period out, jittered — a fleet powered up together must
     * not converge on the same minute forever after. */
    if (r->rc.nextRunMs == 0) r->rc.nextRunMs = millis() + annNextGap(r);

    /* Passive neighbour table: allocated once, history kept across cycles.
     * A failed alloc just leaves the feature off (every path guards on nei).
     * Published only after init — the CLI reads the pointer cross-task. */
    if (!r->nei) {
        NeiState* ns = (NeiState*)gp_alloc(sizeof(NeiState));
        if (ns) {
            memset(ns, 0, sizeof(NeiState));
            ns->sinceMs = millis();
            r->nei = ns;
        }
    }

    /* IFAC: network_name is config (s.), passphrase is a secret (secrets.). */
    storageGetStr(sk(kb, sizeof kb, r->idx, "ifac_netname"), r->curIfacNetname, sizeof(r->curIfacNetname), "");
    {
        char skb[48];
        snprintf(skb, sizeof skb, "secrets.lora.%d.ifac_netkey", r->idx);
        storageGetStr(skb, r->curIfacNetkey, sizeof(r->curIfacNetkey), "");
    }
    r->curIfacSize = (uint8_t)storageGetInt(sk(kb, sizeof kb, r->idx, "ifac_size"), 0);
    r->curAnnounceCap = (uint8_t)storageGetInt(sk(kb, sizeof kb, r->idx, "announce_cap"), RNS_IFACE_ANNOUNCE_CAP_DEFAULT);

    storageBegin();
    storageSet(rk(kb, sizeof kb, r->idx, "chip"), chipName(r->slot->chip));
    storageSet(rk(kb, sizeof kb, r->idx, "bitrate_eff"), (int)r->curBitrate);
    storageEnd();

    /* Arm RX and hook the chip's IRQ line (unified API maps to the right DIO). */
    r->radio->setPacketReceivedAction(loraRadioIsr);
    st = r->radio->startReceive();
    if (st != RADIOLIB_ERR_NONE) {
        err("lora/%d startReceive failed: %s (%d)", r->idx, rlErrName(st), (int)st);
        publishState(r, "error");
        return false;
    }
    /* Make DIO1 a genuine light-sleep wake source. RadioLib's attachInterrupt
     * armed it POSEDGE, but edges are invisible in light sleep (the GPIO clock is
     * gated), so an incoming packet couldn't wake the SoC — RX only landed on the
     * next ~1 Hz poll. Re-arm HIGH_LEVEL (DIO1 asserts high on IRQ): the level
     * both wakes us and re-fires the isr trampoline, and the task drops the line
     * by clearing the IRQ in readData(). This is the level-triggered DIO1 the
     * re-arm paths already assume; paired with pmGpioWakeDisable in radioStop. */
    pmGpioWakeEnable(r->slot->dio1, GPIO_INTR_HIGH_LEVEL);

    r->running = true;
    publishState(r, "up");
    info("lora/%d up: %.3f MHz BW=%.0fkHz SF%d CR4/%d TXP=%ddBm preamble=%d sync=0x%02x",
         r->idx, (double)freq_mhz, (double)bw_khz, sf, cr, txp, preamble, syncWord);

    if (!registerWithRnsd(r)) {
        publishState(r, "rnsd_unavailable");
        /* Stay up on radio — task loop will retry register. */
    }
    return true;
}

/* Boot-time presence probe: a bare begin() (safe defaults + the slot's TCXO
 * voltage) returns RADIOLIB_ERR_NONE iff the radio answers on SPI. Records
 * r->found and logs it, then sleeps the radio — radioStart() re-begins with
 * the real config when the radio is enabled. Independent of enable so the
 * boot log / `lora` CLI shows which slots actually have hardware. */
static void probeRadio(LoraRadio* r) {
    const char* chip = chipName(r->slot->chip);
    float tcxo_v = (float)r->slot->tcxo_mv / 1000.0f;
    /* Probe in the chip's own band — a sub-GHz freq would make a 2.4 GHz part
     * (SX128x) fail begin() and read as absent. */
    bool ghz24 = chipFamily(r->slot->chip) == FAM_SX128X;
    int16_t st = radioBegin(r, ghz24 ? 2450.0f : 434.0f, ghz24 ? 812.5f : 125.0f,
                            9, 7, 0x12, 10, 8, tcxo_v);
    r->found = (st == RADIOLIB_ERR_NONE) ? 1 : 0;
    if (r->found) {
        info("lora/%d: %s found (cs=%d irq=%d busy=%d rst=%d)",
             r->idx, chip, r->slot->cs, r->slot->dio1, r->slot->busy, r->slot->rst);
        char b[48];
        storageSet(rk(b, sizeof b, r->idx, "chip"), chip);
        r->radio->sleep();
    } else {
        warn("lora/%d: %s NOT found at cs=%d (begin: %s (%d))",
             r->idx, chip, r->slot->cs, rlErrName(st), (int)st);
    }
}

/* ─────────────── inbound (radio → rnsd) ─────────────── */

/* Hand one packet to rnsd, prefixed with its signal telemetry (rnsd strips the
 * prefix and sets it on the received packet): int16 rssi(dBm) | int16 snr(dB*10),
 * big-endian. Shared by the radio's receive path, which passes the readings
 * captured in the same synchronous RX call, and by the RNode bridge, which
 * passes a synthetic pair because its packets never crossed the air. */
static void rnsdInject(LoraRadio* r, const uint8_t* data, size_t len,
                       int16_t rssi, int16_t snr10) {
    if (r->rnsdHandle < 0) return;
    uint8_t f[4 + RNS_MTU + 16];
    if (len > sizeof(f) - 4) len = sizeof(f) - 4;   /* defensive clamp */
    f[0] = (uint8_t)(rssi  >> 8); f[1] = (uint8_t)rssi;
    f[2] = (uint8_t)(snr10 >> 8); f[3] = (uint8_t)snr10;
    memcpy(f + 4, data, len);
    size_t s = itsSend(r->rnsdHandle, f, 4 + len, pdMS_TO_TICKS(100));
    if (s == 0) warn("lora/%d rnsd ITS send dropped (%u B)", r->idx, (unsigned)len);
}

static void deliverInbound(LoraRadio* r, const uint8_t* data, size_t len,
                           double airMs, int frames) {
    (void)airMs; (void)frames;   /* per-frame recording/logging now happens in handleRxDone */
    /* Passive neighbour tap — before the rnsd gate, so the table fills even
     * while the interface is unregistered. rssiLast/snrLast were captured in
     * the same synchronous RX call, so they belong to exactly this packet. */
    auto rnd = [](float x) { return (int16_t)(x < 0 ? x - 0.5f : x + 0.5f); };
    int16_t rssi = rnd(r->rssiLast);
    int16_t snr  = rnd(r->snrLast * 10.0f);
    neiObserve(r, data, len, false, rssi, snr, LORA_ORIG_RNSD);
    /* One segment, two other endpoints. Only reassembled packets that are not
     * our own air protocol reach here, so the client sees exactly the Reticulum
     * traffic — with the signal this radio measured for it. */
    rnodeForwardData(r, data, len, /*withStats=*/true);
    rnsdInject(r, data, len, rssi, snr);
}

/* Re-arm continuous RX and re-enable the level-triggered DIO1 (the trampoline
 * disables it on each fire; a completed readData()/finishTransmit() has cleared
 * the chip IRQ so the line has dropped low and the next edge fires again).
 * Shared by the RX drain and the post-TX return to listening. */
static void rearmRx(LoraRadio* r) {
    /* A transmit fired meanwhile (the radio-check slot timer runs off-task) — a
     * startReceive now would abort it. TxDone re-arms RX when it completes. */
    if (r->txActive) {
        gpio_intr_enable((gpio_num_t)r->slot->dio1);
        return;
    }
    r->radio->startReceive();
    gpio_intr_enable((gpio_num_t)r->slot->dio1);
}

/* Drain a completed reception. serviceRadio has already confirmed RX_DONE from
 * the chip's IRQ flags, so go straight to reading the packet. */
static void handleRxDone(LoraRadio* r) {
    size_t pktLen = r->radio->getPacketLength();
    if (pktLen == 0 || pktLen > 1 + RNODE_MAX_PAYLOAD) {
        rearmRx(r);
        return;
    }
    /* Time on air of what just landed, from the framing it actually flew with.
     * Computed before the CRC verdict because channel occupancy does not depend
     * on the frame decoding — a corrupt frame held the medium exactly as long. */
    uint32_t airMs = (uint32_t)lround(1000.0 * loraAirtimeSeconds(
                         r->airSf, r->cfgBwHz, r->cfgCr, r->airPreamble,
                         (int)pktLen, r->airImplicit));

    uint8_t frame[1 + RNODE_MAX_PAYLOAD];
    int16_t st = r->radio->readData(frame, pktLen);
    if (st != RADIOLIB_ERR_NONE) {
        if (st == RADIOLIB_ERR_CRC_MISMATCH) {
            r->crcErr++;
            csmaMediumHeld(r, airMs);
            if (logIsDebug("lora"))       /* CRC = the RX error-check info */
                dbg("lora/%d rx CRC-FAIL %uB rssi=%.0f snr=%.1f",
                    r->idx, (unsigned)pktLen,
                    (double)r->radio->getRSSI(), (double)r->radio->getSNR());
        }
        rearmRx(r);
        return;
    }
    csmaMediumHeld(r, airMs);
    r->rxFrames++;
    r->rssiLast = r->radio->getRSSI();
    r->snrLast  = r->radio->getSNR();

    uint8_t  header     = frame[0];
    uint8_t  seq        = header & 0xF0;
    bool     isSplit    = (header & RNODE_FLAG_SPLIT) != 0;
    size_t   payloadLen = pktLen - 1;

    /* Our own air protocol (the radio check's BATCH and sweep frames) is consumed
     * here — it never enters split framing or rnsd. Classified before the
     * LoRaMon record so the record can carry its protocol colour. */
    /* Radio check, before the probe tap: while we are listening to somebody's
     * sweep the modem is on the 0x23 implicit regime, so a 4-byte frame here is
     * a sweep frame and nothing else. A BATCH arrives on the ordinary regime. */
    bool ours = false;
    if (r->rc.phase == RC_LISTEN && pktLen == RC_SWEEP_LEN) {
        rcOnSweep(r, frame, r->rssiLast);
        ours = true;
    } else if (pktLen >= 4 && frame[0] == LORA_MAGIC_BATCH) {
        rcOnBatch(r, frame, pktLen, (int16_t)lround(r->rssiLast));
        ours = true;
    }

    /* Record this on-air frame (RX_DONE marks end-of-air, so start = end − ToA). */
    {
        uint32_t now = millis();
        uint32_t dur = airMs;
        loraMonPush(r, 0 /*rx*/, (dur <= now ? now - dur : now), (uint16_t)dur,
                    (uint16_t)payloadLen, (int16_t)lround(r->rssiLast),
                    (int16_t)lround(r->snrLast * 10.0), 0,
                    ours ? LORA_PKT_OURS : LORA_PKT_RNS, 0 /*rx never waits*/);
    }

    if (ours) {
        rearmRx(r);
        return;
    }

    if (!isSplit) {
        size_t fl = pktLen;                        /* whole on-air frame (incl. header) */
        deliverInbound(r, frame + 1, payloadLen, loraPacketAirtimeMs(r, &fl, 1), 1);
        r->rxBytes += payloadLen;
    } else if (!r->splitPending) {
        std::memcpy(r->splitBuf, frame + 1, payloadLen);
        r->splitLen      = payloadLen;
        r->splitSeq      = seq;
        r->splitPending  = true;
        r->splitDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(SPLIT_RX_TIMEOUT_MS);
    } else if (r->splitSeq == seq) {
        if (r->splitLen + payloadLen <= sizeof(r->splitBuf)) {
            /* Two on-air frames, each with its own preamble/header/CRC. */
            size_t fl[2] = { r->splitLen + 1, payloadLen + 1 };
            std::memcpy(r->splitBuf + r->splitLen, frame + 1, payloadLen);
            r->splitLen += payloadLen;
            deliverInbound(r, r->splitBuf, r->splitLen,
                           loraPacketAirtimeMs(r, fl, 2), 2);
            r->rxBytes += r->splitLen;
        }
        r->splitPending = false;
    } else {
        /* Different sender's split — restart assembly on the new seq. */
        std::memcpy(r->splitBuf, frame + 1, payloadLen);
        r->splitLen      = payloadLen;
        r->splitSeq      = seq;
        r->splitDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(SPLIT_RX_TIMEOUT_MS);
    }

    /* A pending power request binds to the one RNS frame that follows it and to
     * nothing else — whether that frame claimed it or not, it is spent here. The
     * `ours` path above returns early on purpose: that is how the request
     * survives from its own frame to the one it prefixes. */
    r->apRxSuggestPend = false;

    rearmRx(r);
}

/* ─────────────── CSMA / listen-before-talk ─────────────── */

/* Instantaneous channel RSSI (dBm), read without leaving continuous RX.
 * getRSSI(false) is the "current channel" overload (vs the base getRSSI() which
 * returns last-packet RSSI); it lives on the concrete chip class, not on
 * PhysicalLayer, so dispatch per chip like radioBegin does. */
static float channelRssi(LoraRadio* r) {
    PhysicalLayer* p = r->radio;
    switch (r->slot->chip) {
        case CHIP_SX1261: case CHIP_SX1262: case CHIP_SX1268: case CHIP_LLCC68:
            return static_cast<SX126x*>(p)->getRSSI(false);
        case CHIP_SX1272:
            return static_cast<SX1272*>(p)->getRSSI(false);
        case CHIP_SX1276: case CHIP_SX1277: case CHIP_SX1278:
            return static_cast<SX1278*>(p)->getRSSI(false);
        case CHIP_SX1280: case CHIP_SX1281: case CHIP_SX1282:
            return static_cast<SX128x*>(p)->getRSSI(false);
        case CHIP_LR1110: case CHIP_LR1120: case CHIP_LR1121:
            return static_cast<LR11x0*>(p)->getRSSI(false);
        case CHIP_LR2021:
            return static_cast<LR2021*>(p)->getRSSI(false);
    }
    return -200.0f;   /* unhandled chip → read as free (fail open to blind TX) */
}

/* Channel-RSSI sample for the LoRaMon floor: one reading a second, handed to
 * the interface task to publish. Radio task; the reading is the same
 * getRSSI(false) carrier sense uses, so it costs one SPI transaction.
 *
 * **Carrier sense outranks measurement.** While the CSMA machine holds the
 * radio — or a transmit is on air, or a probe owns the chip, or a split is
 * still reassembling — no sample is taken and none is published. The series
 * goes absent for the duration and the viewers draw the gap. A reading taken
 * mid-contention would describe the transmission we are queued behind, not the
 * channel's resting noise, and channel access is the radio's actual job.
 *
 * The beat is not advanced when a sample is skipped, so sampling resumes as
 * soon as the radio is idle again rather than waiting out the rest of a second. */
/* Measure the agile channels of the regime in force, as one excursion off the
 * hailing channel and back.
 *
 * Retune, read, retune home: standby → setFrequency → startReceive → getRSSI
 * per channel, then the same to come home. Nine channels is roughly 2–3 ms away
 * from the hailing channel — inside the ~4 ms an 8-symbol SF7/BW125 preamble
 * allows before a frame could be missed (plans/psa.md §3.5). The caller has
 * already established that nothing else wants the radio.
 *
 * **Bandwidth is deliberately not retuned.** Every channel is measured with the
 * receiver the hailing channel is configured for, so the readings share one
 * noise reference and are directly comparable — which is what a graph of nine
 * channels needs. Measuring each at its own width would make a 500 kHz channel
 * read ~6 dB hotter than a 125 kHz one from thermal noise alone, for no gain. A
 * regulatory Clear Channel Assessment is the opposite case and would have to
 * match the channel's occupied bandwidth; that is a different measurement for a
 * different purpose, and not this one. */
static void rssiSweepAgile(LoraRadio* r, IfMsg* m, const RegimeChan* ch, int n) {
    for (int i = 0; i < n && i + 1 < LORA_CH_MAX; i++) {
        r->radio->standby();
        if (r->radio->setFrequency((float)ch[i].freqHz / 1.0e6f) != RADIOLIB_ERR_NONE)
            continue;                                  /* out of the part's range */
        if (r->radio->startReceive() != RADIOLIB_ERR_NONE) continue;
        r->hal->delayMicroseconds(LORA_RSSI_SETTLE_US);
        float dbm = channelRssi(r);
        /* Below the thermal noise of any bandwidth this part can receive, so
         * not a measurement — it is the receiver answering before it is
         * listening. Leave the channel unreported rather than draw a floor
         * that isn't one. */
        if (dbm <= LORA_RSSI_INVALID_DBM) continue;
        m->chRssi[i + 1] = (int16_t)lround(dbm);
    }
    /* Home. Unconditional and unchecked: a failed retune above must not strand
     * the radio off the hailing channel, which is the one thing this must never
     * do. */
    r->radio->standby();
    r->radio->setFrequency((float)r->cfgFreqHz / 1.0e6f);
    r->radio->startReceive();
}

static void rssiSamplePoll(LoraRadio* r) {
    if (!r->running || !r->enabled) return;
    if ((int32_t)(xTaskGetTickCount() - r->rssiNext) < 0) return;
    if (r->txActive || r->splitPending) return;
    if (r->rc.phase != RC_OFF) return;
    if (r->csmaPhase != CSMA_IDLE || r->mtxPhase == MTXP_LBT) return;

    r->rssiNext = xTaskGetTickCount() + pdMS_TO_TICKS(LORA_RSSI_SAMPLE_MS);

    IfMsg m = {};
    m.kind  = IFM_RSSI;
    m.radio = (uint8_t)r->idx;
    m.ch    = LORA_CH_HAIL;
    m.t_ms  = millis();
    for (int i = 0; i < LORA_CH_MAX; i++) m.chRssi[i] = LORA_RSSI_NONE;

    int n = 0;
    const RegimeChan* ch = regimeChans(r->afa, &n);
    /* The field count is the regime's channel count whether or not every one of
     * them answered, so a viewer reads a stable set of columns and a channel
     * that failed to measure is an empty field rather than a shifted one. */
    m.nch = (uint8_t)((1 + n > LORA_CH_MAX) ? LORA_CH_MAX : 1 + n);

    /* The hailing channel first and in place — the radio is already on it and
     * settled, so this reading costs one transaction and no retune. */
    float hail = channelRssi(r);
    m.chRssi[LORA_CH_HAIL] = (int16_t)lround(hail);
    m.rssi = m.chRssi[LORA_CH_HAIL];

    /* Leaving the hailing channel mid-reception destroys the frame, and unlike
     * a preamble there is no partial-recovery argument. The reading just taken
     * is the cheapest available evidence that something is on air, so energy
     * above the tracked floor cancels this beat's excursion — the agile
     * channels simply go unreported and the viewers draw the gap.
     *
     * It is the same evidence carrier sense uses and carries the same blind
     * spot: a frame below the floor is invisible to it (§4.2 of plans/psa.md).
     * Closing that needs the preamble-detect and header-valid interrupts armed
     * during receive, which the receive path does not currently ask for. */
    bool quiet = hail <= r->noiseFloor + CSMA_RSSI_MARGIN_DB;
    if (ch && n > 0 && quiet) rssiSweepAgile(r, &m, ch, n);

    if (!ifPost(&m)) r->rssiDropped++;
}

/* Carrier sense: sample the channel and decide busy/free, tracking the noise
 * floor as the low envelope of RSSI (snap down fast, creep up slowly) so an
 * active channel can't inflate the reference it's compared against. Also busy
 * while a multi-frame reception is being reassembled (half-duplex). */
static bool channelBusy(LoraRadio* r) {
    if (r->splitPending) return true;
    float rssi = channelRssi(r);
    if (rssi < r->noiseFloor) r->noiseFloor = rssi;
    else                      r->noiseFloor += 0.02f * (rssi - r->noiseFloor);
    /* A reception completed since the last sense (csmaMediumHeld). The medium
     * was held for that frame's whole time on air whether or not the threshold
     * below ever noticed, so report it busy once — that is what puts the
     * inter-frame space after the frame instead of over the top of it.
     *
     * One-shot, and only if the sense caught it fresh: past one DIFS the
     * required idle period has already elapsed as real idle time, and the
     * sample stands on its own. */
    if (r->rxHeldTick) {
        TickType_t difs = r->appc ? r->appcDifsTicks : r->difsTicks;
        bool fresh = (TickType_t)(xTaskGetTickCount() - r->rxHeldTick) < difs;
        r->rxHeldTick = 0;
        if (fresh) return true;
    }
    return rssi > r->noiseFloor + CSMA_RSSI_MARGIN_DB;
}

/* End of a reception — the channel-state half of receiving a frame.
 *
 * Carrier sense cannot be relied on to have noticed the transmission we just
 * decoded. LoRa demodulates below the noise floor (SF7 at −7.5 dB SNR), so a
 * frame received perfectly may never have risen above `noiseFloor +
 * CSMA_RSSI_MARGIN_DB`; and the sense is a point sample once per slot, not a
 * continuous watch, so even a strong frame can fall between two of them. The
 * receiver holds the one piece of evidence the sense lacks: it decoded
 * something, therefore the medium was occupied.
 *
 * Two corrections follow, and the second is the one that bites:
 *
 *  - The next sense reads busy, so the inter-frame space restarts from the end
 *    of the frame rather than from wherever the sampling happened to land.
 *
 *  - Under APPC the contention window is a wall-clock target accumulated
 *    *while the medium reads free*. A reception the sense missed is therefore
 *    not merely ignored — its whole duration is banked as credit toward our own
 *    transmission, so hearing a neighbour out makes us more eager rather than
 *    less. That credit is given back here. `appcCwStart` discriminates: non-zero
 *    means the accumulator was still running when the frame ended, so the sense
 *    never saw it and the credit is bogus; zero means a busy sense had already
 *    stopped the clock and there is nothing to return.
 *
 * Called for every frame off the air, CRC failures and our own air protocol
 * included: a frame that failed its CRC still occupied the channel, and a frame
 * we consumed ourselves was still somebody else transmitting. */
static void csmaMediumHeld(LoraRadio* r, uint32_t durMs) {
    r->rxHeldTick = xTaskGetTickCount();
    if (r->rxHeldTick == 0) r->rxHeldTick = 1;      /* 0 is the "nothing to apply" sentinel */
    if (!r->appc || r->appcCwStart == 0) return;
    TickType_t held = pdMS_TO_TICKS(durMs);
    r->appcCwPassed = (r->appcCwPassed > held) ? (TickType_t)(r->appcCwPassed - held) : 0;
}

/* ── APPC: own-airtime accounting ──
 *
 * RNode buckets its own time-on-air into 7.5 s bins across a ring covering the
 * uptime hour, and reads the current plus previous bin as `airtime`, the figure
 * that picks the contention band. We keep only those two live bins: the rest of
 * the ring exists upstream to feed a long-term duty-cycle lock, which this
 * straddle does not implement. Bins are aligned to uptime, not to a rolling
 * window, so `airtime` covers between one and two bins of history — that is
 * upstream's behaviour and the band edges are calibrated against it. */
static void appcRollBins(LoraRadio* r, uint32_t nowMs) {
    uint32_t bin = (nowMs % APPC_HOUR_MS) / APPC_BIN_MS;
    if (bin == r->appcBinIdx) return;
    /* Adjacent bin → the old current ages into previous. Any longer gap means
     * both windows elapsed without a transmit of ours, so both are empty. */
    r->appcBinPrev = (bin == (r->appcBinIdx + 1) % APPC_BINS) ? r->appcBinCur : 0;
    r->appcBinCur  = 0;
    r->appcBinIdx  = bin;
}

/* Credit one transmitted frame's time-on-air. Called per frame at TxDone, where
 * the duration is already computed for the LoRaMon record. Sole writer of the
 * bins, and it runs on the radio task. */
static void appcAddAirtime(LoraRadio* r, uint32_t durMs) {
    if (!r->appc) return;
    appcRollBins(r, millis());
    r->appcBinCur += durMs;
}

/* Fraction of the last two bins this radio spent transmitting. Ages the stored
 * bins into the present without writing them, so the CLI printer — which reads
 * live radio state from its own task — cannot race the radio task's accounting.
 * Bins older than the previous window read as empty, which is what they would
 * be rolled to on the next transmit. */
static float appcAirtime(const LoraRadio* r) {
    uint32_t bin = (millis() % APPC_HOUR_MS) / APPC_BIN_MS;
    uint32_t cur = 0, prev = 0;
    if (bin == r->appcBinIdx)                        { cur = r->appcBinCur; prev = r->appcBinPrev; }
    else if (bin == (r->appcBinIdx + 1) % APPC_BINS) { prev = r->appcBinCur; }
    return (float)(cur + prev) / (2.0f * (float)APPC_BIN_MS);
}


/* Arduino map(), reproduced so the band edges land exactly where upstream's do:
 * integer arithmetic throughout, division truncating toward zero. */
static int appcMap(int x, int inMin, int inMax, int outMin, int outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/* Band for an own-airtime percentage. Upstream feeds map() the percentage with
 * APPC_BAND_1_MAX_AIRTIME added *and* uses the same value as the input floor,
 * which cancels out to a plain 0-based scale — reproduced verbatim rather than
 * simplified, so a future upstream change to either constant stays a one-line
 * diff. With today's numbers the edges are: ≤7 % band 1, 8–38 % band 2,
 * 39–77 % band 3, ≥78 % band 4. */
static uint8_t appcBandFor(int airtimePct) {
    int band;
    if (airtimePct <= APPC_BAND_1_MAX_AIRTIME) band = 1;
    else band = appcMap(airtimePct + APPC_BAND_1_MAX_AIRTIME,
                        APPC_BAND_1_MAX_AIRTIME, APPC_BAND_N_MIN_AIRTIME,
                        2, APPC_CW_BANDS);
    if (band > APPC_CW_BANDS) band = APPC_CW_BANDS;
    if (band < 1) band = 1;
    return (uint8_t)band;
}

/* The band the airtime right now selects — what a window drawn now would use.
 * appcBand holds the band of the window currently *in flight*, which is what
 * the contention stall warning wants; telemetry and the CLI want this. */
static uint8_t appcLiveBand(const LoraRadio* r) {
    return appcBandFor((int)(appcAirtime(r) * 100.0f));
}

/* Draw this frame's contention window from the band our airtime puts us in.
 * Bands partition a single 0..(bands×windows−1) ladder, so band 1 draws 0–13,
 * band 2 15–28, band 3 30–43, band 4 45–58 — upstream's random(min,max) excludes
 * the top of each band, and so does this. */
static void appcDrawWindow(LoraRadio* r) {
    int pct  = (int)(appcAirtime(r) * 100.0f);
    r->appcBand = appcBandFor(pct);
    int cwMin = (r->appcBand - 1) * APPC_CW_PER_BAND_WINDOWS;
    int cwMax = r->appcBand * APPC_CW_PER_BAND_WINDOWS - 1;
    r->appcCw = cwMin + (int)(esp_random() % (uint32_t)(cwMax - cwMin));
    r->appcCwTarget = (TickType_t)r->appcCw * r->appcSlotTicks;
}

/* Advance the APPC channel-access machine one sense. Same contract as
 * csmaClear(): true only on the pass the medium is granted.
 *
 * The window is a wall-time target, accumulated only while the medium reads
 * free — a busy sense restarts DIFS from scratch but *freezes* rather than
 * discards the accumulated backoff, so a frame contending on a loaded channel
 * keeps its progress and cannot be starved indefinitely by neighbours. The
 * window is redrawn only after a grant, never widened on a busy encounter: all
 * adaptation lives in the band. */
static bool csmaClearAppc(LoraRadio* r) {
    TickType_t now = xTaskGetTickCount();
    if (now == 0) now = 1;                          /* 0 is the "not started" sentinel */
    bool free_ = !channelBusy(r);

    if (r->appcCw < 0) appcDrawWindow(r);

    if (r->appcDifsStart == 0) {                    /* DIFS not begun */
        r->csmaPhase = CSMA_DIFS;
        if (free_) r->appcDifsStart = now;
        return false;
    }
    if (!free_) {                                   /* medium taken → restart DIFS */
        r->csmaPhase = CSMA_DIFS;
        r->appcDifsStart = 0;
        r->appcCwStart   = 0;                       /* stop counting; keep what passed */
        return false;
    }
    if ((TickType_t)(now - r->appcDifsStart) < r->appcDifsTicks) return false;

    r->csmaPhase = CSMA_BACKOFF;
    if (r->appcCwStart == 0) { r->appcCwStart = now; return false; }
    r->appcCwPassed += (TickType_t)(now - r->appcCwStart);
    r->appcCwStart   = now;
    if (r->appcCwPassed < r->appcCwTarget) return false;

    r->appcCwPassed  = 0;                           /* granted → next frame redraws */
    r->appcCw        = -1;
    r->appcDifsStart = 0;
    r->appcCwStart   = 0;
    r->csmaPhase     = CSMA_IDLE;
    return true;
}

/* Advance the channel-access state machine. Returns true only on the tick the
 * medium is granted (DIFS observed idle, then a random backoff drained without
 * the channel going busy). Otherwise updates state and returns false; the
 * caller leaves the frame queued and nextDeadline() re-wakes at the next slot.
 * cw grows on every busy encounter (exponential backoff) and resets after a
 * grant; with appc set, the window comes from the airtime band instead. */
static bool csmaClear(LoraRadio* r) {
    if (!r->lbt) return true;                       /* LBT off → blind transmit */

    if (r->appc) {
        /* csmaStart drives the shared stall warning and lbt_timeout valve, so
         * it is stamped here for both regimes. */
        if (r->csmaPhase == CSMA_IDLE) r->csmaStart = xTaskGetTickCount();
        return csmaClearAppc(r);
    }

    TickType_t now = xTaskGetTickCount();
    bool busy = channelBusy(r);

    switch (r->csmaPhase) {
        case CSMA_IDLE:
            /* New frame: begin an inter-frame (DIFS) listen. */
            r->csmaCw = CSMA_CW_MIN;
            r->csmaPhase = CSMA_DIFS;
            r->csmaStart = now;                     /* start the lbt_timeout clock */
            r->csmaDifsStart = busy ? 0 : now;      /* 0 = free window not begun */
            return false;

        case CSMA_DIFS:
            if (busy) { r->csmaDifsStart = 0; return false; }   /* restart on activity */
            if (r->csmaDifsStart == 0) r->csmaDifsStart = now;
            if ((TickType_t)(now - r->csmaDifsStart) < r->difsTicks) return false;
            /* DIFS observed idle → draw a backoff in [0, 2^cw) slots. */
            r->csmaBackoff = (int)(esp_random() & ((1u << r->csmaCw) - 1));
            if (r->csmaBackoff == 0) { r->csmaPhase = CSMA_IDLE; return true; }
            r->csmaPhase = CSMA_BACKOFF;
            r->csmaSlotDeadline = now + r->slotTicks;
            return false;

        case CSMA_BACKOFF:
            if (busy) {                             /* lost the medium → widen, re-listen */
                if (r->csmaCw < CSMA_CW_MAX) r->csmaCw++;
                r->csmaPhase = CSMA_DIFS;
                r->csmaDifsStart = 0;
                return false;
            }
            if ((int32_t)(now - r->csmaSlotDeadline) < 0) return false;  /* slot not up */
            r->csmaSlotDeadline = now + r->slotTicks;
            if (--r->csmaBackoff <= 0) { r->csmaPhase = CSMA_IDLE; return true; }
            return false;
    }
    return true;
}

/* Abandon any channel-access attempt in progress: the next frame contends from
 * scratch. Used when the queue drains, when a frame is shed by the lbt_timeout
 * valve, and when the probe takes or releases the radio. Under appc this also
 * discards the frozen backoff — upstream would carry it into the next frame,
 * but here the machine is shared by three producers and stale progress must not
 * leak from one to another. */
static void csmaResetAccess(LoraRadio* r) {
    r->csmaPhase     = CSMA_IDLE;
    r->appcCw        = -1;
    r->appcCwPassed  = 0;
    r->appcCwStart   = 0;
    r->appcDifsStart = 0;
}


/* ─────────────── outbound (rnsd → radio) ─────────────── */

/* Return the radio to continuous RX after a transmit finishes or aborts. */
/* Back to listening after a packet — completed, abandoned by the watchdog, or
 * never started. If the packet came from the RNode client, this is where its
 * frame is finished with the radio, so its queue is released here: CMD_READY is
 * harmless with flow control off and mandatory with it on. */
static void txRearmRx(LoraRadio* r) {
    r->txActive = false;
    if (r->txFromRnode) { r->txFromRnode = false; rnodeSendReady(); }
    rearmRx(r);
}

/* Fire frame `idx` of the current outbound packet. startTransmit() writes the
 * FIFO and issues SetTx, then returns — the chip modulates on its own and raises
 * TxDone on DIO1 when done (serviceRadio handles it). Non-blocking: the task is
 * free for the whole airtime, so nothing on its core is starved at high SF. */
static void startTxFrame(LoraRadio* r, int idx) {
    int16_t st = r->radio->startTransmit(r->txFrame[idx], r->txFrameLen[idx]);
    if (st != RADIOLIB_ERR_NONE) {
        warn("lora/%d startTransmit %u B failed: %s (%d)",
             r->idx, (unsigned)r->txFrameLen[idx], rlErrName(st), (int)st);
        txRearmRx(r);
        return;
    }
    r->txActive       = true;
    r->txFrameStartMs = millis();                  /* start-of-air, for the LoRaMon record */
    r->txDeadline     = xTaskGetTickCount() + r->txWatchTicks;
    gpio_intr_enable((gpio_num_t)r->slot->dio1);   /* arm DIO1 for this frame's TxDone */
}

/* ─────────────── announce buffer ───────────────
 * (overview at ANN_MAX_ENTRIES, near the top of the file) */

/* Next beat, jittered ±ANN_JITTER_PCT. Without the scatter every node that
 * booted together would announce on the same second forever. */
static uint32_t annNextGap(const LoraRadio* r) {
    if (r->annIntervalMin == 0) return 0;              /* 0 = only `lora a` emits */
    uint32_t period = (uint32_t)r->annIntervalMin * 60u * 1000u;
    uint32_t span   = period * ANN_JITTER_PCT / 100;
    return period - span + (esp_random() % (2 * span + 1));
}

/* Drop entries past their hour. Radio task only. */
static void annExpire(LoraRadio* r, uint32_t now) {
    if (!r->ann) return;
    for (int i = 0; i < ANN_MAX_ENTRIES; i++) {
        AnnRec* e = &r->ann->e[i];
        if (e->used && (uint32_t)(now - e->ms) > ANN_TTL_MS) e->used = false;
    }
}

static int annCount(const LoraRadio* r) {
    if (!r->ann) return 0;
    int n = 0;
    for (int i = 0; i < ANN_MAX_ENTRIES; i++) if (r->ann->e[i].used) n++;
    return n;
}

/* Offer an outgoing packet to the buffer. Stores it only if it is an announce
 * this node ORIGINATED — wire hops 0, which is what distinguishes our own
 * announcement from one we are relaying for the network. Relayed announces
 * belong to somebody else and replaying them would be speaking for them.
 *
 * Keyed by destination hash: a fresh announce for a destination we already hold
 * replaces it in place rather than adding an entry, so a busy destination
 * cannot crowd the buffer. At the cap, the oldest entry is evicted — with a
 * one-hour TTL that only bites on a node with more than sixteen live
 * destinations, where something has to give regardless.
 *
 * **Returns true when it has SWALLOWED the packet**, which is the normal case
 * for an announce we originate. This interface announces at its own pace and
 * nobody else's: rnsd and the RNode client hand announces over whenever their
 * own logic fires, and we sit on them until the beat comes round or `lora a`
 * says so. Nothing else is intercepted — a relayed announce is somebody else's
 * traffic and goes straight out.
 *
 * The cost is that rnsd believes its announce is away the moment it hands it
 * over, and it may be up to `announce_interval` before that is true. That is
 * the trade the pacing buys, not an oversight. */
static bool annOfferTx(LoraRadio* r, const uint8_t* pkt, size_t len) {
    if (!r->ann || len == 0 || len > ANN_MAX_LEN) return false;
    NeiHdr h;
    if (!neiParse(pkt, len, &h)) return false;
    if (h.ptype != NEI_PT_ANNOUNCE || h.hops != 0 || h.hdr2 || !h.dest) return false;

    uint32_t now = millis();
    annExpire(r, now);

    AnnRec* slot = nullptr;
    for (int i = 0; i < ANN_MAX_ENTRIES; i++) {          /* same destination → replace */
        AnnRec* e = &r->ann->e[i];
        if (e->used && std::memcmp(e->dest, h.dest, 16) == 0) { slot = e; break; }
    }
    if (!slot) for (int i = 0; i < ANN_MAX_ENTRIES; i++)  /* else a free slot */
        if (!r->ann->e[i].used) { slot = &r->ann->e[i]; break; }
    if (!slot) {                                          /* else evict the oldest */
        slot = &r->ann->e[0];
        for (int i = 1; i < ANN_MAX_ENTRIES; i++)
            if ((int32_t)(r->ann->e[i].ms - slot->ms) < 0) slot = &r->ann->e[i];
    }

    std::memcpy(slot->dest, h.dest, 16);
    std::memcpy(slot->data, pkt, len);
    slot->len  = (uint16_t)len;
    slot->ms   = now;
    slot->used = true;
    return true;
}

/* ── adaptive TX power: the tx-path half ──
 * (overview at AP_EST_MARGIN_DB, near the top of the file) */
static int8_t apClamp(LoraRadio* r, int want);
static bool neiEstimateCliff10(const LoraRadio* r, const Neighbor* e,
                               uint32_t now, int* cliff10, uint32_t* samples);

/* The first-4 naming the node an outbound frame's FIRST RF HOP goes to, or null
 * when the frame has no single next hop we can name. Power is a property of
 * that one hop; a proof is end-to-end, so a multi-hop destination says nothing
 * about the power the hop in front of us needs. */
static const uint8_t* apNextHop4(LoraRadio* r, const uint8_t* pkt, size_t len) {
    NeiState* st = r->nei;
    if (!st) return nullptr;
    NeiHdr h;
    if (!neiParse(pkt, len, &h)) return nullptr;
    /* Announces first, and that order matters: a rebroadcast announce is
     * HEADER_2, but its transport_id is the REBROADCASTER's own identity (that
     * is how path tables learn first_hop), not a next hop. It is a broadcast
     * either way and must reach everyone. */
    if (h.ptype == NEI_PT_ANNOUNCE) return nullptr;
    if (h.hdr2) return h.transportId;                 /* relayed: the next hop names itself */
    if (h.ptype == NEI_PT_PROOF || h.dtype == NEI_DT_LINK) {
        /* Link traffic and link proofs are addressed to the link_id, so the
         * peer is the link's destination — and only when that destination is
         * someone else. An inbound dial records no hash for its initiator, so
         * a link they opened to us resolves to nothing. A delivery proof is
         * addressed to a packet hash and simply misses the link table. */
        NeiLink* L = neiLinkFind(st, h.dest);
        if (!L || !L->haveDest || neiDestIsLocal(st, L->dest)) return nullptr;
        return L->dest;
    }
    if (h.dtype == NEI_DT_SINGLE) return h.dest;
    return nullptr;
}

/* Is this outbound frame addressed to a link, and did that link's peer request a
 * power for it? A 0x04 request outranks anything we could work out ourselves:
 * the receiver folded in its own noise floor, antenna and sensitivity, none of
 * which a transmitter can see. Covers the LRPROOF and every later frame of the
 * session alike, since all of them are addressed to the link_id. */
static bool apLinkSuggest(LoraRadio* r, const NeiHdr* h, int8_t* out) {
    if (h->hdr2 || (h->ptype != NEI_PT_PROOF && h->dtype != NEI_DT_LINK)) return false;
    NeiLink* L = neiLinkFind(r->nei, h->dest);
    if (!L || !L->haveSuggest) return false;
    *out = L->suggestDbm;
    return true;
}

/* The power this frame goes out at: a peer's explicit request first, then the
 * next-hop node's own determination, else the configured tx_power. */
static int8_t apTxPower(LoraRadio* r, const uint8_t* pkt, size_t len) {
    if (!r->adaptive || !r->nei) return r->cfgTxp;
    NeiHdr h;
    if (neiParse(pkt, len, &h) && h.ptype != NEI_PT_ANNOUNCE) {
        int8_t want;
        if (apLinkSuggest(r, &h, &want)) return apClamp(r, want);
    }
    const uint8_t* nh = apNextHop4(r, pkt, len);
    if (!nh) return r->cfgTxp;
    Neighbor* e = neiFindBy4(r->nei, nh);
    if (!e || neiIsLocal(e) || !e->haveApPwr) return r->cfgTxp;
    return e->apPwr;
}

/* Put the chip on `txp`. This is the only place the tx path moves the power
 * register, and txPwrNow is what the radio is currently set to as well as what
 * the LoRaMon record is stamped with, so the two can't drift — a frame to a
 * quiet neighbour must not leave the next frame transmitting at its power while
 * being recorded at another. */
static void apApplyPower(LoraRadio* r, int8_t txp) {
    if (txp == r->txPwrNow) return;
    int16_t st = r->radio->setOutputPower(txp);
    if (st != RADIOLIB_ERR_NONE) {
        warn("lora/%d setOutputPower(%d): %s (%d)",
             r->idx, (int)txp, rlErrName(st), (int)st);
        return;
    }
    r->txPwrNow = txp;
}

/* Should this outbound packet carry a power request, and what should it ask for?
 * Only a link opener so far — an LR is the one unicast frame where we chose the
 * destination, so we hold its history, and where one 4-byte prefix covers a
 * whole session's traffic coming back rather than a single reply.
 *
 * The number is what the peer needs to reach US, and that is a DIRECT
 * measurement rather than a reciprocal one: neiEstimateCliff10 is their assumed
 * power minus the headroom their frames actually arrived with at our receiver,
 * so it says how much of their power was surplus here. Its one assumption is the
 * power they transmitted at — which "no prefix means maximum" makes true.
 *
 * Nothing is sent when we would ask for a power no radio can exceed: absence
 * already means maximum, so the frame would be 35 ms saying nothing.
 *
 * Every gate names itself under `log lora debug`. Five conditions have to line
 * up before a request goes out, and a silently absent prefix is otherwise
 * indistinguishable from a broken one. */
static bool apPwrReqFor(LoraRadio* r, const uint8_t* pkt, size_t len, int8_t* out) {
    const char* why = nullptr;
    int         val = 0;
    NeiHdr      h;

    if (!r->nei) return false;
    if (!neiParse(pkt, len, &h)) return false;
    if (h.hdr2 || h.ptype != NEI_PT_LINKREQ || h.dtype != NEI_DT_SINGLE || h.hops != 0)
        return false;                       /* not a link opener — say nothing */

    int      cliff10 = 0;
    uint32_t samples = 0;
    Neighbor* e = neiFindBy4(r->nei, h.dest);
    if (!r->adaptive)                     why = "adaptive_txpwr off";
    else if (!e || neiIsLocal(e))         why = "dest hash is on no node row";
    /* Only a node that has spoken our air protocol to us will parse the frame;
     * to anyone else it is 35 ms of unparseable noise on a shared channel. That
     * is the RF_PROTO_NAME tag in `lora n`, set by a radio check or by either
     * linkage frame — which is what bootstraps eligibility. */
    else if (!e->ourProto)                why = "node has not spoken our protocol";
    else if (!neiEstimateCliff10(r, e, millis(), &cliff10, &samples))
                                          why = "no recent signal to estimate from";
    /* One frame's RSSI moves several dB, so a single sample has no business
     * dialling anyone down. */
    else if (samples < AP_MIN_SAMPLES)   { why = "too few samples"; val = (int)samples; }

    int want = 0;
    if (!why) {
        want = (cliff10 >= 0 ? cliff10 / 10 : (cliff10 - 9) / 10) + AP_EST_MARGIN_DB;
        if (want >= AP_PWR_MAX_DBM)      { why = "would ask for max anyway"; val = want; }
        if (want < RC_FLOOR_DBM) want = RC_FLOOR_DBM;
    }

    if (logIsDebug("lora")) {
        char what[56];
        if (!why)     snprintf(what, sizeof what, "asking for %d dBm", want);
        else if (val) snprintf(what, sizeof what, "%s (%d)", why, val);
        else          safeStrncpy(what, why, sizeof what);
        dbg("lora/%d pwr-req to %02x%02x%02x%02x: %s",
            r->idx, h.dest[0], h.dest[1], h.dest[2], h.dest[3], what);
    }
    if (why) return false;
    *out = (int8_t)want;
    return true;
}

/* Begin transmitting one RNS packet from `origin` (LORA_ORIG_*).
 * >RNODE_MAX_PAYLOAD splits into two frames sharing a seq nibble; the second is
 * fired from serviceRadio once the first completes. Returns immediately — the
 * airtime runs on the chip, not the task.
 *
 * This is also the fan-out point of the three-way bridge, and deliberately so:
 * "presented to the radio" means transmitted, so a packet the LBT valve drops
 * never aired and is bridged nowhere. A packet is bridged if and only if it
 * went on air. */
static void beginTx(LoraRadio* r, const uint8_t* data, size_t len, uint8_t origin,
                    bool fromBuffer = false) {
    if (!r->running || len == 0 || len > RNS_MTU) return;

    /* Our own announces are buffered, not transmitted, until our beat emits
     * them — except when this call IS that emission. */
    if (!fromBuffer && annOfferTx(r, data, len)) return;

    neiObserve(r, data, len, true, 0, 0, origin);   /* passive neighbour tap (tx side) */

    r->txSeq          = (uint8_t)((esp_random() & 0x0F) << 4);   /* 4-bit seq, upper nibble */
    r->txPayloadBytes = len;
    r->txFrameSent    = 0;

    /* A power request rides in front of the packet it relates to, in the same
     * channel access, and binds to it by adjacency alone. Only link openers
     * carry one so far; an LR never splits (83 B, far under RNODE_MAX_PAYLOAD),
     * so the pair fits the existing two-frame burst. */
    int      base = 0;
    int8_t   suggest;
    if (apPwrReqFor(r, data, len, &suggest)) {
        r->txFrame[0][0] = LORA_MAGIC_PWRREQ;
        r->txFrame[0][1] = (uint8_t)suggest;
        r->txFrame[0][2] = 0;               /* no rssi report: we know this peer, */
        r->txFrame[0][3] = 0;               /* so the suggestion is the useful half */
        r->txFrameLen[0] = PWRREQ_LEN;
        r->txType[0]     = LORA_PKT_OURS;
        base = 1;
    }

    uint8_t pktType = (origin == LORA_ORIG_RNODE) ? LORA_PKT_RNODE : LORA_PKT_RNS;
    if (len <= RNODE_MAX_PAYLOAD) {
        r->txFrame[base][0] = r->txSeq;
        std::memcpy(r->txFrame[base] + 1, data, len);
        r->txFrameLen[base] = 1 + len;
        r->txType[base]     = pktType;
        r->txFrameCount     = (uint8_t)(base + 1);
    } else {
        size_t first  = RNODE_MAX_PAYLOAD;
        size_t second = len - first;
        r->txFrame[0][0] = r->txSeq | RNODE_FLAG_SPLIT;
        std::memcpy(r->txFrame[0] + 1, data, first);
        r->txFrameLen[0] = 1 + first;
        r->txType[0]     = pktType;
        r->txFrame[1][0] = r->txSeq | RNODE_FLAG_SPLIT;
        std::memcpy(r->txFrame[1] + 1, data + first, second);
        r->txFrameLen[1] = 1 + second;
        r->txType[1]     = pktType;
        r->txFrameCount  = 2;
    }
    r->txFromRnode = (origin == LORA_ORIG_RNODE);

    /* Segment fan-out: whichever of the other two endpoints exists gets this
     * packet, because it is about to be on air and all three share the
     * channel. Our own transmissions carry no measured signal, so the client's
     * copy goes without stat frames. */
    if (origin == LORA_ORIG_RNODE) rnsdInject(r, data, len, RNODE_INJ_RSSI, RNODE_INJ_SNR10);
    else                           rnodeForwardData(r, data, len, /*withStats=*/false);

    /* Per-frame LoRaMon records + the `log lora debug` line are emitted at each
     * frame's TxDone (serviceRadio); the RNS-header trace (loraTracePacket) is
     * kept but no longer called. The request and the packet must go out at the
     * same power — the peer measures the pair as one path sample. */
    apApplyPower(r, apTxPower(r, data, len));
    startTxFrame(r, 0);
}

/* ─────────────── radio check ───────────────
 * (wire format + protocol overview at ANN_MAX_ENTRIES, top of file)
 *
 * Two clocks drive a run. The announce bunches are coarse — 276 ms frames, each
 * bunch carrier-sensed — so they run off the task loop like any other transmit.
 * The tail is not: its 5 ms and 20 ms gaps are a schedule every receiver is
 * counting on, and the FreeRTOS tick is 10 ms. So from the first BATCH to the
 * last SF5 frame the sequence is driven by an esp_timer one-shot against
 * absolute µs deadlines, and nothing is sensed inside it. */

static void rcRestoreModem(LoraRadio* r);
static void rcEnd(LoraRadio* r, const char* why);

/* How long the channel access that just granted the medium took, for the
 * LoRaMon wait mark. Valid only on the pass csmaClear() returned true — both
 * regimes stamp csmaStart on an attempt's first sense and leave it alone until
 * the next. With LBT off nothing was sensed. */
static uint16_t rcSenseWaitMs(const LoraRadio* r) {
    if (!r->lbt) return 0;
    uint32_t ms = (uint32_t)(xTaskGetTickCount() - r->csmaStart) * portTICK_PERIOD_MS;
    return (uint16_t)(ms > 0xFFFF ? 0xFFFF : ms);
}
/* Chip dispatch, defined with the rest of it further down. */
static int16_t radioHeaderMode(LoraRadio* r, bool implicit, size_t len);
static int16_t radioSyncWord(LoraRadio* r, uint8_t sync);
static int16_t radioSetSf(LoraRadio* r, uint8_t sf);

static int8_t rcClampTx(LoraRadio* r, int want) {
    int8_t clipped = (int8_t)want;
    r->radio->checkOutputPower((int8_t)want, &clipped);
    return clipped;
}

/* Time on air of one sweep frame in the phase currently installed. */
static uint32_t rcSweepToaMs(const LoraRadio* r, uint8_t sf) {
    return (uint32_t)lround(1000.0 * loraAirtimeSeconds(
               sf, r->cfgBwHz, r->cfgCr, RC_SWEEP_PREAMBLE, RC_SWEEP_LEN, true));
}

/* Total air the tail will occupy, computed before a single byte of it goes out.
 * The whole thing is one transmission held to Ton_max, so this is what channel
 * access has to be won for and what the airtime ledger has to be able to
 * cover — not just BATCH's share of it. */
static uint32_t rcTailMs(const LoraRadio* r, int n) {
    uint32_t batch = (uint32_t)lround(1000.0 * loraAirtimeSeconds(
                         r->cfgSf, r->cfgBwHz, r->cfgCr, r->cfgPreamble,
                         4 + n, false));
    return RC_BATCH_REPEATS * batch + (RC_BATCH_REPEATS - 1) * RC_BATCH_GAP_MS
         + RC_LEAD_MS
         + RC_STEPS_SF7 * (rcSweepToaMs(r, r->cfgSf) + RC_SLOT_GUARD_MS)
         + RC_RETUNE_MS
         + RC_STEPS_SF5 * (rcSweepToaMs(r, RC_SWEEP_SF5) + RC_SLOT_GUARD_MS);
}

/* The power ladder. Spans the chip's floor to the node's CONFIGURED power, not
 * the chip's ceiling: a radio held to 14 dBm for regulatory reasons stays
 * there and simply sweeps a shorter run. Nothing in this protocol may raise a
 * node above what it was configured to transmit at. */
static int8_t rcLadderPower(const LoraRadio* r, uint8_t step, uint8_t steps) {
    int lo = r->rc.txpFloor, hi = r->rc.txpCeil;
    if (steps <= 1 || hi <= lo) return (int8_t)hi;
    int v = lo + ((hi - lo) * (int)step + (int)(steps - 1) / 2) / (int)(steps - 1);
    return (int8_t)v;
}

/* Install / remove the sweep regime: implicit header at RC_SWEEP_LEN, preamble
 * 6, sync 0x23. The sync word is the load-bearing part — a node not in a radio
 * check never correlates, so these frames are silent to it rather than a stream
 * of length-mismatched garbage. */
static bool rcEnterSweep(LoraRadio* r, uint8_t sf) {
    if (r->rc.savedSync == 0) {                   /* first entry: remember the way back */
        r->rc.savedSync     = r->cfgSync;
        r->rc.savedPreamble = r->cfgPreamble;
        r->rc.savedSf       = r->cfgSf;
    }
    r->radio->standby();
    int16_t st = radioHeaderMode(r, true, RC_SWEEP_LEN);
    if (st == RADIOLIB_ERR_NONE) st = r->radio->setPreambleLength(RC_SWEEP_PREAMBLE);
    if (st == RADIOLIB_ERR_NONE) st = radioSyncWord(r, RC_SWEEP_SYNC);
    if (st == RADIOLIB_ERR_NONE && sf != r->rc.savedSf) st = radioSetSf(r, sf);
    if (st != RADIOLIB_ERR_NONE) {
        warn("lora/%d sweep cfg failed: %s (%d)", r->idx, rlErrName(st), (int)st);
        return false;
    }
    /* The LoRaMon airtime figures must follow the framing actually in use, not
     * the configured one, or every sweep frame is recorded at the wrong length. */
    r->airPreamble = RC_SWEEP_PREAMBLE;
    r->airImplicit = true;
    r->airSf       = sf;
    return true;
}

static void rcRestoreModem(LoraRadio* r) {
    if (r->rc.savedSync == 0) return;             /* never entered */
    r->radio->standby();
    int16_t st = radioHeaderMode(r, false, RC_SWEEP_LEN);
    if (st == RADIOLIB_ERR_NONE) st = r->radio->setPreambleLength((size_t)r->rc.savedPreamble);
    if (st == RADIOLIB_ERR_NONE) st = radioSyncWord(r, r->rc.savedSync);
    if (st == RADIOLIB_ERR_NONE) st = radioSetSf(r, r->rc.savedSf);
    if (st == RADIOLIB_ERR_NONE) st = r->radio->setOutputPower(r->cfgTxp);
    r->airPreamble  = r->rc.savedPreamble;
    r->airImplicit  = false;
    r->airSf        = r->rc.savedSf;
    r->txPwrNow     = r->cfgTxp;
    r->rc.savedSync = 0;
    if (st != RADIOLIB_ERR_NONE) {
        /* Stranded in the sweep regime — hardware recovery, not a config edit,
         * and there is nothing to coalesce it with. */
        warn("lora/%d sweep restore failed (%s (%d)) — restarting radio",
             r->idx, rlErrName(st), (int)st);
        cfgArm(0);
    }
    rearmRx(r);
}

/* Record one frame the tail put on air. The tail transmits through
 * PhysicalLayer::transmit() rather than startTxFrame(), so the TxDone path that
 * normally writes the LoRaMon record and credits the APPC band never runs for
 * it — without this the whole radio check is invisible to the viewers and free
 * as far as airtime accounting is concerned, which it very much is not.
 *
 * `sf`/`preamble`/`implicit` are passed rather than read off the radio because
 * during the sweep the configured values and the installed ones differ. */
static void rcRecordTx(LoraRadio* r, uint32_t startMs, uint8_t sf, int len,
                       bool implicit, uint16_t preamble, int8_t txp, uint16_t waitMs) {
    uint32_t dur = (uint32_t)lround(1000.0 * loraAirtimeSeconds(
                       sf, r->cfgBwHz, r->cfgCr, preamble, len, implicit));
    loraMonPush(r, 1 /*tx*/, startMs, (uint16_t)dur, (uint16_t)len,
                0, 0, txp, LORA_PKT_OURS, waitMs);
    appcAddAirtime(r, dur);
}

/* Build one sweep frame. `tag` couples an otherwise anonymous headerless frame
 * to the node that sent it — without it a receiver could attribute a power
 * floor to nobody. */
static void rcBuildSweep(LoraRadio* r, uint8_t f[RC_SWEEP_LEN], int8_t txp, uint8_t step) {
    f[0] = (uint8_t)txp;
    f[1] = r->rc.tag[0];
    f[2] = r->rc.tag[1];
    f[3] = (uint8_t)(r->rc.sfPhase | (step & 0x0F));
}

/* One slot of the sweep, from the esp_timer. Sends this step's frame at this
 * step's power, then arms the next slot — or hands over to SF5, or ends. */
static void rcListenEnd(LoraRadio* r);

static void rcSlotCb(void* arg) {
    LoraRadio* r = (LoraRadio*)arg;
    RcState*   p = &r->rc;

    /* Listening to somebody else's schedule. Three events: the lead-in puts us
     * on the sweep regime at SF7, the SF7 phase's end follows the sender to
     * SF5, and the SF5 phase's end puts everything back. Each is timed from the
     * BATCH, not from what we hear — a node too far away to decode a single
     * sweep frame still tracks the phases correctly and simply records nothing. */
    if (p->phase == RC_LISTEN) {
        uint32_t slot7 = rcSweepToaMs(r, r->cfgSf) + RC_SLOT_GUARD_MS;
        uint32_t slot5 = rcSweepToaMs(r, RC_SWEEP_SF5) + RC_SLOT_GUARD_MS;
        int64_t  d;
        switch (p->step) {
            case 0:
                if (!rcEnterSweep(r, r->cfgSf)) { p->phase = RC_OFF; return; }
                rearmRx(r);
                p->step = 1;
                /* Land in the MIDDLE of the sender's retune gap. At its end the
                 * reconfigure runs through the first SF5 frame; at its start any
                 * lateness in our own schedule eats the last SF7 one. Half a gap
                 * of margin on each side is the only symmetric choice. */
                d = (int64_t)(RC_STEPS_SF7 * slot7 + RC_RETUNE_MS / 2) * 1000;
                esp_timer_start_once(p->timer, (uint64_t)d);
                return;
            case 1:
                if (!rcEnterSweep(r, RC_SWEEP_SF5)) { rcListenEnd(r); return; }
                rearmRx(r);
                p->step = 2;
                d = (int64_t)(RC_STEPS_SF5 * slot5 + RC_SLOT_GUARD_MS * 4) * 1000;
                esp_timer_start_once(p->timer, (uint64_t)d);
                return;
            default:
                rcListenEnd(r);
                return;
        }
    }

    if (p->phase != RC_SWEEP) return;

    uint8_t steps = (p->sfPhase == RC_PHASE_SF5) ? RC_STEPS_SF5 : RC_STEPS_SF7;
    uint8_t sf    = (p->sfPhase == RC_PHASE_SF5) ? RC_SWEEP_SF5 : r->rc.savedSf;

    if (p->step >= steps) {
        if (p->sfPhase == RC_PHASE_SF7) {         /* SF7 done → the same walk at SF5 */
            p->sfPhase = RC_PHASE_SF5;
            p->step    = 0;
            if (!rcEnterSweep(r, RC_SWEEP_SF5)) { rcEnd(r, "sf5 switch failed"); return; }
            p->slotUs += (int64_t)RC_RETUNE_MS * 1000;
            esp_timer_start_once(p->timer, (uint64_t)(p->slotUs - esp_timer_get_time()));
            return;
        }
        rcEnd(r, nullptr);                        /* both phases done */
        return;
    }

    uint8_t f[RC_SWEEP_LEN];
    int8_t  txp = rcLadderPower(r, p->step, steps);
    rcBuildSweep(r, f, txp, p->step);
    apApplyPower(r, txp);
    r->txPwrNow = txp;
    uint32_t t0 = millis();
    r->radio->standby();
    r->radio->transmit(f, RC_SWEEP_LEN);          /* blocking: the slot IS its airtime */
    rcRecordTx(r, t0, sf, RC_SWEEP_LEN, true, RC_SWEEP_PREAMBLE, txp, 0);

    p->step++;
    p->slotUs += (int64_t)(rcSweepToaMs(r, sf) + RC_SLOT_GUARD_MS) * 1000;
    int64_t d = p->slotUs - esp_timer_get_time();
    esp_timer_start_once(p->timer, (uint64_t)(d > 0 ? d : 0));
}

static bool rcEnsureTimer(LoraRadio* r) {
    if (r->rc.timer) return true;
    esp_timer_create_args_t a = {};
    a.callback = rcSlotCb;
    a.arg      = r;
    a.dispatch_method = ESP_TIMER_TASK;
    a.name     = "lora-rc";
    return esp_timer_create(&a, &r->rc.timer) == ESP_OK;
}

static void rcEnd(LoraRadio* r, const char* why) {
    RcState* p = &r->rc;
    if (p->timer) esp_timer_stop(p->timer);
    rcRestoreModem(r);
    apApplyPower(r, r->cfgTxp);
    if (why) warn("lora/%d radio check aborted: %s", r->idx, why);
    else if (logIsDebug(TAG))
        dbg("lora/%d radio check done (%u announces)", r->idx, (unsigned)p->n);
    p->phase = RC_OFF;
    p->step  = 0;
    p->nextRunMs = millis() + annNextGap(r);
}

/* Assemble the manifest from the buffer and start a run. */
static void rcBegin(LoraRadio* r) {
    RcState* p = &r->rc;
    uint32_t now = millis();
    annExpire(r, now);

    p->n = 0;
    for (int i = 0; i < ANN_MAX_ENTRIES && p->n < ANN_MAX_ENTRIES; i++) {
        AnnRec* e = &r->ann->e[i];
        if (!e->used) continue;
        if (p->n == 0) { p->tag[0] = e->dest[0]; p->tag[1] = e->dest[1]; }
        p->hashes[p->n++] = e->dest[0];
    }
    if (p->n == 0) {                     /* nothing to say: no announces, no check */
        p->phase = RC_OFF;
        p->nextRunMs = now + annNextGap(r);
        return;
    }
    p->txpCeil    = r->cfgTxp;           /* the CONFIGURED power, never the chip's */
    p->txpFloor   = rcClampTx(r, RC_FLOOR_DBM);   /* protocol floor, chip-clamped up */
    p->annIdx     = 0;
    p->bunchAirMs = 0;
    p->step       = 0;
    p->sfPhase    = RC_PHASE_SF7;
    p->phase      = RC_ANN;
}

/* Fire the whole tail: BATCH twice back-to-back, then the slot timer takes
 * over. Called with the channel already won for the tail's full duration. */
static void rcStartTail(LoraRadio* r) {
    RcState* p = &r->rc;
    if (!rcEnsureTimer(r)) { rcEnd(r, "no slot timer"); return; }

    uint8_t f[4 + ANN_MAX_ENTRIES];
    f[0] = LORA_MAGIC_BATCH;
    f[2] = (uint8_t)p->txpCeil;
    f[3] = p->n;
    std::memcpy(f + 4, p->hashes, p->n);

    apApplyPower(r, p->txpCeil);
    r->txPwrNow = p->txpCeil;
    p->phase = RC_BATCH;
    for (int i = 0; i < RC_BATCH_REPEATS; i++) {
        f[1] = (uint8_t)i;                         /* the copy index: what lets a
                                                    * receiver time the sweep from
                                                    * EITHER copy, so losing the
                                                    * first costs nothing */
        uint32_t t0 = millis();
        r->radio->standby();
        r->radio->transmit(f, (size_t)(4 + p->n));
        rcRecordTx(r, t0, r->cfgSf, 4 + p->n, false, r->cfgPreamble, p->txpCeil,
                   i == 0 ? p->tailWaitMs : 0);
        /* Busy-wait, not delay(): the FreeRTOS tick is 10 ms, so a 5 ms
         * vTaskDelay rounds to zero and the gap silently vanishes — while every
         * listener's sweep_start arithmetic still assumes it happened. Every
         * interval in this tail is finer than the tick and none of them may be
         * expressed in ticks. */
        if (i + 1 < RC_BATCH_REPEATS) r->hal->delayMicroseconds(RC_BATCH_GAP_MS * 1000);
    }

    /* The sweep starts RC_LEAD_MS after the last BATCH ends — the window every
     * receiver uses to retune. Anchor the slot schedule on that instant. */
    p->sfPhase = RC_PHASE_SF7;
    p->step    = 0;
    p->phase   = RC_SWEEP;
    if (!rcEnterSweep(r, r->cfgSf)) { rcEnd(r, "sweep regime failed"); return; }
    p->slotUs = esp_timer_get_time() + (int64_t)RC_LEAD_MS * 1000;
    esp_timer_start_once(p->timer, (uint64_t)RC_LEAD_MS * 1000);
}

/* ── receive side ──
 *
 * A BATCH puts us on the sweep regime for exactly as long as the sender's
 * schedule runs, then puts us back. Everything is derived, nothing is
 * negotiated: the sender transmits into the dark and we work out when to
 * listen from constants we both hold.
 *
 * Nothing here answers. The result is one number per node — the lowest power
 * we could decode it at — parked in its neighbour row for adaptive TX power to
 * read. Path loss being reciprocal, that is also our estimate of what we need
 * to reach them; and because every sweep frame STATES the power it was sent at,
 * that estimate survives the far end adapting its own power, which a passive
 * RSSI reading would not (plans/adaptive-power.md §3). */

/* Fold the manifest into one node, and hand back the sender's destination.
 *
 * **The manifest IS the coupling.** A node announces n destinations back to
 * back and then lists them together, so every row holding one of them is the
 * same device and they belong in one row. This is what the retired 0x03
 * linkage frame said explicitly; here it falls out of the batch for free, and
 * unlike 0x03 it costs no extra transmission.
 *
 * One byte per hash is ambiguous across a whole table but not across the
 * candidates. Only rows heard inside RC_COUPLE_WINDOW_MS are eligible — the
 * announces this manifest describes flew seconds ago — so the set being matched
 * against is a handful of nodes that just spoke, not the table. That is what
 * the byte is sized for.
 *
 * Local rows are never folded in: a peer's claim must not swallow our own row
 * or the RNode client's, however its hashes happen to collide. */
#define RC_COUPLE_WINDOW_MS  30000

static void rcCoupleManifest(LoraRadio* r, const uint8_t* hashes, uint8_t n,
                             uint8_t outDest[16], bool* haveDest) {
    NeiState* st = r->nei;
    uint32_t  now = millis();

    /* The sender's row: the most recently heard candidate for the FIRST hash,
     * which is also the tag its sweep frames carry. */
    Neighbor* home = nullptr;
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used || neiIsLocal(e)) continue;
        if ((uint32_t)(now - e->lastHeardMs) > RC_COUPLE_WINDOW_MS) continue;
        for (int d = 0; d < e->nDests; d++) {
            if (e->dests[d].hash[0] != hashes[0]) continue;
            if (!home || (int32_t)(e->lastHeardMs - home->lastHeardMs) > 0) {
                home = e;
                memcpy(outDest, e->dests[d].hash, 16);
                *haveDest = true;
            }
            break;
        }
    }
    if (!home) return;

    home->ourProto = true;                 /* it spoke our air protocol */

    /* Everything else the manifest names is the same node. */
    for (uint8_t k = 1; k < n; k++) {
        for (int i = 0; i < NEI_MAX; i++) {
            Neighbor* e = &st->nei[i];
            if (!e->used || e == home || neiIsLocal(e)) continue;
            if ((uint32_t)(now - e->lastHeardMs) > RC_COUPLE_WINDOW_MS) continue;
            bool match = false;
            for (int d = 0; d < e->nDests && !match; d++)
                if (e->dests[d].hash[0] == hashes[k]) match = true;
            if (match) neiMergeInto(home, e);
        }
    }
    if (logIsDebug(TAG))
        dbg("lora/%d coupled %u destinations under one node", r->idx, (unsigned)n);
}

/* A BATCH from somebody else. Places their sweep from the copy index, so
 * hearing only the second copy is exactly as good as hearing the first. */
static void rcOnBatch(LoraRadio* r, const uint8_t* f, size_t len, int16_t rssi) {
    if (len < 4) return;
    uint8_t i = f[1], n = f[3];
    if (i >= RC_BATCH_REPEATS || len < (size_t)(4 + n)) return;
    if (r->rc.phase == RC_SWEEP || r->rc.phase == RC_ANN ||
        r->rc.phase == RC_TAIL_LBT || r->rc.phase == RC_BATCH) return;  /* ours is running */

    RcState* p = &r->rc;
    if (p->phase == RC_LISTEN && p->rxTag[0] == f[4]) return;   /* the second copy */

    uint32_t batchMs = (uint32_t)lround(1000.0 * loraAirtimeSeconds(
                           r->cfgSf, r->cfgBwHz, r->cfgCr, r->cfgPreamble,
                           (int)len, false));
    /* RX_DONE marks end-of-air, so "now" is this frame's end. */
    int64_t start = esp_timer_get_time()
                  + (int64_t)(RC_BATCH_REPEATS - 1 - i) * (batchMs + RC_BATCH_GAP_MS) * 1000
                  + (int64_t)RC_LEAD_MS * 1000;

    p->haveRxDest = false;
    if (n && r->nei) rcCoupleManifest(r, f + 4, n, p->rxDest, &p->haveRxDest);
    p->rxTag[0]  = n ? f[4] : 0;
    p->rxTag[1]  = 0;                  /* the sweep's second tag byte is unknown to us
                                        * from the manifest alone; matched loosely */
    p->rxBestTxp = 127;
    p->rxBestRssi = (int8_t)(rssi < -128 ? -128 : rssi);
    p->rxHeard   = 0;
    p->phase     = RC_LISTEN;
    /* Listen out the sender's whole schedule plus a slot of slack. */
    p->rxUntilUs = start + (int64_t)(rcTailMs(r, n) + RC_SLOT_GUARD_MS * 4) * 1000;

    if (!rcEnsureTimer(r)) { p->phase = RC_OFF; return; }
    /* Land HALF a lead-in early. Not now — the second BATCH copy is still to
     * come on the ordinary regime and switching would deafen us to it — but not
     * at `start` either, which is the instant the sender begins frame 0 and
     * would have us reconfiguring through it. */
    int64_t d = start - (int64_t)RC_LEAD_MS * 500 - esp_timer_get_time();
    if (d < 0) d = 0;
    esp_timer_start_once(p->timer, (uint64_t)d);
}

/* The listening window is over: put the modem back and bank the result.
 *
 * `rxBestTxp` is the lowest power at which we could still decode that node —
 * measured, not computed, and needing no RSSI calibration. It is what adaptive
 * TX power reads, via reciprocity, as the power WE need toward them. */
static void rcListenEnd(LoraRadio* r) {
    RcState* p = &r->rc;
    if (p->timer) esp_timer_stop(p->timer);
    rcRestoreModem(r);
    if (p->rxHeard && logIsDebug(TAG))
        dbg("lora/%d radio check from %02x: floor %d dBm at %d dBm rssi (%u frames)",
            r->idx, p->rxTag[0], (int)p->rxBestTxp, (int)p->rxBestRssi,
            (unsigned)p->rxHeard);
    if (p->rxHeard && p->haveRxDest && r->nei) {
        Neighbor* e = neiFindByDest(r->nei, p->rxDest);
        if (e) {
            e->ourProto = true;              /* it speaks our air protocol */
            /* The lowest power we could still decode is what THEY need to reach
             * US. Path loss is reciprocal, so it is also our estimate of what we
             * need toward them — and because every sweep frame stated its own
             * power, that holds even while the far end adapts its own, which a
             * passive RSSI reading could not (plans/adaptive-power.md §3).
             * Ambient noise is NOT reciprocal, which is what the margin is for. */
            apSettle(r, e, /*measured=*/true, rcClampTx(r, p->rxBestTxp + AP_EST_MARGIN_DB));
        }
    }
    p->haveRxDest = false;
    p->phase = RC_OFF;
    p->step  = 0;
}

/* One decoded sweep frame from somebody else. */
static void rcOnSweep(LoraRadio* r, const uint8_t* f, float rssi) {
    RcState* p = &r->rc;
    if (p->phase != RC_LISTEN) return;
    if (p->rxTag[0] && f[1] != p->rxTag[0]) return;    /* not the node we are timing */

    int8_t txp = (int8_t)f[0];
    p->rxHeard++;
    if (txp < p->rxBestTxp) {
        p->rxBestTxp  = txp;
        p->rxBestRssi = (int8_t)lround(rssi < -128.0f ? -128.0f : rssi);
    }
}

/* Task-loop half: the announce bunches, then winning the channel for the tail.
 * Returns having done at most one thing, like every other poll on this loop. */
static void rcPoll(LoraRadio* r) {
    RcState* p = &r->rc;
    if (!r->running || !r->ann) return;

    if (p->phase == RC_OFF) {
        /* nextRunMs is only meaningful while the beat is on; with the interval
         * at 0 nothing but `lora a` starts a run. */
        bool due = r->annIntervalMin && (int32_t)(millis() - p->nextRunMs) >= 0;
        if (!p->req && !due) return;
        if (r->txActive || r->splitPending || r->mtxPhase != MTXP_OFF) return;
        p->req = false;
        rcBegin(r);
        return;
    }
    if (r->txActive) return;                        /* a frame of ours is still on air */

    if (p->phase == RC_ANN) {
        /* Bunch: fill with announces until the next would push past
         * ANN_BUNCH_MAX_MS, then re-sense for the following bunch. */
        while (p->annIdx < ANN_MAX_ENTRIES && !r->ann->e[p->annIdx].used) p->annIdx++;
        if (p->annIdx >= ANN_MAX_ENTRIES) {         /* buffer drained → the tail */
            p->phase = RC_TAIL_LBT;
            csmaResetAccess(r);
            return;
        }
        AnnRec*  e   = &r->ann->e[p->annIdx];
        uint32_t toa = (uint32_t)lround(1000.0 * loraAirtimeSeconds(
                           r->cfgSf, r->cfgBwHz, r->cfgCr, r->cfgPreamble,
                           (int)e->len + 1, false));
        if (p->bunchAirMs != 0 && p->bunchAirMs + toa > ANN_BUNCH_MAX_MS) {
            p->bunchAirMs = 0;                      /* bunch full → sense again */
            csmaResetAccess(r);
            return;
        }
        uint16_t wait = 0;
        if (p->bunchAirMs == 0) {
            if (!csmaClear(r)) return;                    /* one sense per bunch */
            wait = rcSenseWaitMs(r);
        }
        /* Only the first frame of a bunch waited for anything; the rest followed
         * it back-to-back. Stamped explicitly because this path bypasses
         * drainOneOutbound, whose stamp would otherwise be inherited stale and
         * drawn as a wait reaching back over the previous announce. */
        r->txWaitMs   = wait;
        r->txWaitPend = false;
        p->bunchAirMs += toa;
        p->annIdx++;
        /* Replay the stored bytes verbatim — the announce is signed over its own
         * contents, so it can be neither regenerated nor edited here. */
        beginTx(r, e->data, e->len, LORA_ORIG_RNSD, /*fromBuffer=*/true);
        return;
    }

    if (p->phase == RC_TAIL_LBT) {
        if (!csmaClear(r)) return;
        p->tailWaitMs = rcSenseWaitMs(r);
        rcStartTail(r);
        return;
    }
}

/* Clamp a wanted TX power to what this chip can do. PhysicalLayer's default
 * checkOutputPower leaves clipped untouched, so seed it with the wanted value. */

/* This node's probe ceiling: the radio's configured tx_power, chip-clamped.
 * The same ceiling real traffic obeys, so a rung the ladder reaches is by
 * construction a power an RNS frame may also use. */

/* Link-budget headroom (deci-dB) of one measurement, computed from the
 * byte-encoded rssi/snr — integer/deterministic so both ends derive the
 * identical ladder from the exchanged bytes. SNR against the SF demodulation
 * floor while SNR is meaningful; RSSI against sensitivity once it saturates. */

/* ToA of one 4-byte sweep frame (implicit header, CRC on, preamble 6). */
/* ToA of a `len`-byte implicit-header sweep-regime frame (CRC on, preamble 6). */

/* Slot lengths. The schedule knows exactly which frame rides in which slot, so
 * each slot is only as long as its own frame needs: slot 0 carries the 8-byte
 * P2, every slot after it a 4-byte sweep frame. Sizing them all for the longest
 * frame would leave ~10 ms of dead air in every slot but the first. Both ends
 * derive these from SF/BW/CR, so they agree without exchanging anything. */

/* Start of slot `k`, µs after T0. */

/* Our own ladder, from reciprocity: `rssiB/snrQ` is OUR measurement of a peer
 * frame sent at `peerTxp`, which gives the path loss and so the power we need.
 * Start two steps under that, floor at −9 dBm, clamp to our own probe max.
 * The peer derives its ladder the same way from its own measurement — the two
 * need not match, since every frame states the power it went out at. */

/* Fire one raw probe frame at an explicit power — no framing byte; carrier
 * sense, and so the wait it reports, is the caller's job. Reuses the normal
 * non-blocking TX machinery. */

/* Modem header mode: implicit with a fixed `len` (sweep regime — headerless
 * frames carry no length, so both ends must be told) vs explicit (normal).
 * Lives on the concrete classes, not PhysicalLayer — dispatch like channelRssi. */
static int16_t radioHeaderMode(LoraRadio* r, bool implicit, size_t len) {
    PhysicalLayer* p = r->radio;
    switch (r->slot->chip) {
        case CHIP_SX1261: case CHIP_SX1262: case CHIP_SX1268: case CHIP_LLCC68:
            return implicit ? static_cast<SX126x*>(p)->implicitHeader(len)
                            : static_cast<SX126x*>(p)->explicitHeader();
        case CHIP_SX1272:
            return implicit ? static_cast<SX1272*>(p)->implicitHeader(len)
                            : static_cast<SX1272*>(p)->explicitHeader();
        case CHIP_SX1276: case CHIP_SX1277: case CHIP_SX1278:
            return implicit ? static_cast<SX1278*>(p)->implicitHeader(len)
                            : static_cast<SX1278*>(p)->explicitHeader();
        case CHIP_SX1280: case CHIP_SX1281: case CHIP_SX1282:
            return implicit ? static_cast<SX128x*>(p)->implicitHeader(len)
                            : static_cast<SX128x*>(p)->explicitHeader();
        case CHIP_LR1110: case CHIP_LR1120: case CHIP_LR1121:
            return implicit ? static_cast<LR11x0*>(p)->implicitHeader(len)
                            : static_cast<LR11x0*>(p)->explicitHeader();
        case CHIP_LR2021:
            return implicit ? static_cast<LR2021*>(p)->implicitHeader(len)
                            : static_cast<LR2021*>(p)->explicitHeader();
    }
    return RADIOLIB_ERR_UNKNOWN;
}

/* Coding rate (denominator 5..8). Lives on the concrete classes, not
 * PhysicalLayer, and SX127x splits it across SX1272/SX1278 — dispatch by chip
 * like radioHeaderMode. Used by tx_prot to put the payload on 4/8 so the header
 * it emits announces a 4/8 packet. */
static int16_t radioSetCodingRate(LoraRadio* r, uint8_t crDenom) {
    PhysicalLayer* p = r->radio;
    switch (r->slot->chip) {
        case CHIP_SX1261: case CHIP_SX1262: case CHIP_SX1268: case CHIP_LLCC68:
            return static_cast<SX126x*>(p)->setCodingRate(crDenom);
        case CHIP_SX1272:
            return static_cast<SX1272*>(p)->setCodingRate(crDenom);
        case CHIP_SX1276: case CHIP_SX1277: case CHIP_SX1278:
            return static_cast<SX1278*>(p)->setCodingRate(crDenom);
        case CHIP_SX1280: case CHIP_SX1281: case CHIP_SX1282:
            return static_cast<SX128x*>(p)->setCodingRate(crDenom);
        case CHIP_LR1110: case CHIP_LR1120: case CHIP_LR1121:
            return static_cast<LR11x0*>(p)->setCodingRate(crDenom);
        case CHIP_LR2021:
            return static_cast<LR2021*>(p)->setCodingRate(crDenom);
    }
    return RADIOLIB_ERR_UNKNOWN;
}

static int16_t radioSyncWord(LoraRadio* r, uint8_t sync) {
    PhysicalLayer* p = r->radio;
    switch (chipFamily(r->slot->chip)) {
        case FAM_SX126X: return static_cast<SX126x*>(p)->setSyncWord(sync);
        case FAM_SX127X: return static_cast<SX127x*>(p)->setSyncWord(sync);
        case FAM_SX128X: return static_cast<SX128x*>(p)->setSyncWord(sync);
        case FAM_LR11X0: return static_cast<LR11x0*>(p)->setSyncWord(sync);
        case FAM_LR2021: return static_cast<LR2021*>(p)->setSyncWord(sync);
    }
    return RADIOLIB_ERR_UNKNOWN;
}

/* Spreading factor at runtime — the radio check's SF5 phase is the only caller.
 * Not on PhysicalLayer (it is LoRa-specific), so dispatched per family like the
 * sync word above. */
static int16_t radioSetSf(LoraRadio* r, uint8_t sf) {
    PhysicalLayer* p = r->radio;
    switch (r->slot->chip) {
        case CHIP_SX1261: case CHIP_SX1262: case CHIP_SX1268: case CHIP_LLCC68:
            return static_cast<SX126x*>(p)->setSpreadingFactor(sf);
        case CHIP_SX1272:
            return static_cast<SX1272*>(p)->setSpreadingFactor(sf);
        case CHIP_SX1276: case CHIP_SX1277: case CHIP_SX1278:
            return static_cast<SX1278*>(p)->setSpreadingFactor(sf);
        case CHIP_SX1280: case CHIP_SX1281: case CHIP_SX1282:
            return static_cast<SX128x*>(p)->setSpreadingFactor(sf);
        case CHIP_LR1110: case CHIP_LR1120: case CHIP_LR1121:
            return static_cast<LR11x0*>(p)->setSpreadingFactor(sf);
        case CHIP_LR2021:
            return static_cast<LR2021*>(p)->setSpreadingFactor(sf);
    }
    return RADIOLIB_ERR_UNKNOWN;
}

/* Enter the sweep regime at implicit length `len`. Called at P1's end-of-air
 * (len 8, for slot 0's P2) and again once slot 0 is past (len 4). */

/* Shorten the implicit RX length to the 4-byte sweep frame, once slot 0 (the
 * only 8-byte frame in the schedule) can no longer arrive. Idempotent. */

/* Back to the configured modem params. On any failure, force a full radio
 * restart through the config path rather than run degraded. */

/* Reset per-run fields (keeps the CLI request/result handshake fields). */


/* ── adaptive TX power: settling one node's determination ──
 * (overview at AP_EST_MARGIN_DB, near the top of the file) */
static bool neiEstimateCliff10(const LoraRadio* r, const Neighbor* e,
                               uint32_t now, int* cliff10, uint32_t* samples);
static bool neiNodeFirst4(const Neighbor* e, uint8_t out[4]);

/* Clamp a power to what this radio may transmit at: the chip's range, and
 * never above the configured tx_power. A measured rung already satisfies both
 * (the ladder climbs to that same ceiling); the estimate-plus-margin path is
 * what needs the clamp. */
static int8_t apClamp(LoraRadio* r, int want) {
    if (want > r->cfgTxp)       want = r->cfgTxp;
    if (want < RC_FLOOR_DBM) want = RC_FLOOR_DBM;
    return rcClampTx(r, want);
}

/* Record the one determination for a node, at the end of a probe run. With a
 * measured rung that rung is it; without one the reciprocity estimate carries
 * the link, plus AP_EST_MARGIN_DB because it assumes a power the peer never
 * stated and because noise is not reciprocal even where path loss is. A node
 * with no estimate either — nothing heard from it inside the bucket ring's hour
 * — gets no determination and keeps the configured power. */
static void apSettle(LoraRadio* r, Neighbor* e, bool measured, int8_t rung) {
    if (!r->adaptive || !e || neiIsLocal(e) || e->haveApPwr) return;
    int pwr;
    if (measured) {
        pwr = rung;
    } else {
        int est10;
        if (!neiEstimateCliff10(r, e, millis(), &est10, nullptr)) return;
        pwr = (est10 >= 0 ? est10 / 10 : (est10 - 9) / 10) + AP_EST_MARGIN_DB;
    }
    e->apPwr     = apClamp(r, pwr);
    e->apFromEst = !measured;
    e->haveApPwr = true;
    uint8_t h4[4] = {};
    neiNodeFirst4(e, h4);
    info("lora/%d adaptive: %02x%02x%02x%02x settled at %d dBm (%s)",
         r->idx, h4[0], h4[1], h4[2], h4[3], (int)e->apPwr,
         measured ? "measured" : "estimate + margin");
}


/* ── cooperative hash linkage (0x02 / 0x03) ──
 * Everything here keys on a hash's first 4 bytes, which is all the linkage
 * frames carry; a full 16-byte dest already in the table matches on its first
 * 4. See the format block at the top of the file. */

/* Find a neighbour by any first-4 it is known under: its node key, a full
 * dest hash from an announce, or a hash a 0x03 linked to it. */
Neighbor* neiFindBy4(NeiState* st, const uint8_t b4[4]) {
    if (!st) return nullptr;
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used) continue;
        if (e->haveNode4 && memcmp(e->node4, b4, 4) == 0) return e;
        for (int d = 0; d < e->nDests; d++)
            if (memcmp(e->dests[d].hash, b4, 4) == 0) return e;
        for (int l = 0; l < e->nLink4; l++)
            if (memcmp(e->link4[l], b4, 4) == 0) return e;
    }
    return nullptr;
}


/* How many distinct hashes we hold for a node — announced dests plus the ones
 * a 0x03 linked in. This is what we compare against a peer's advertised count. */
static int neiKnownHashes(const Neighbor* e) {
    return (int)e->nDests + (int)e->nLink4;
}






/* Sent at the radio's full configured power, never a neighbour's adaptive one
 * heard widely saves everyone else from asking. */

/* Does this first-4 name one of the local rows' announced dest hashes? (us or
 * the RNode client — both terminate here, so neither is a probe target.) */

/* Build the next sweep frame; returns the (chip-clamped) power it must go out
 * at. Climb one rung per own-slot until DONE, then hold at found + 1 step so
 * the remaining frames report the other direction without shouting. */


/* ── the precise part: slot TX from an esp_timer one-shot ──
 * Runs in the esp_timer task (high priority): build the frame under the mux
 * (byte math, µs) and fire the transmit directly — the lora task (priority 2,
 * 10 ms tick quantization) is never in the TX timing path, and the alarm
 * itself wakes the chip from light sleep. */

/* Arm the one-shot for our next owned slot (absolute µs schedule off t0Us);
 * a slot we can no longer hit cleanly is skipped — never fired late. */


/* Light sleep hands timekeeping to the RTC slow clock (an RC oscillator on
 * most boards), which is nowhere near accurate enough for the slot schedule —
 * so a probe pins the XTAL-fed systimer by blocking light sleep end to end. */

/* Enter the fixed-time schedule. `firstSlot` is our parity: 0 for the
 * responder (whose slot 0 is P2), 1 for the initiator. T0 is the ISR stamp of
 * P1's end-of-air — the same physical instant on both ends. */

/* RX tap (from handleRxDone, every frame). Returns true when the frame was
 * ours and must not reach split framing / rnsd. */

/* TxDone hook (from serviceRadio) — phase transitions ride frame completion. */

/* Consume a CLI kick (fields already filled by the CLI task). */

/* Drive the probe state machine — every task-loop pass. */


/* ─────────────── manual CLI transmit (tx / tx_psa / tx_prot) ─────────────── */

/* Finish a manual-TX request: park the state machine and hand the outcome to the
 * CLI, which is polling mtxResGen. */
static void manualTxFinish(LoraRadio* r, bool ok, const char* msg) {
    r->mtxPhase = MTXP_OFF;
    r->mtxResOk = ok;
    safeStrncpy(r->mtxResMsg, msg ? msg : "", sizeof r->mtxResMsg);
    r->mtxResGen = r->mtxResGen + 1;   /* release the CLI's poll loop */
}

/* tx_prot: put a header on the air that announces a long 4/8 packet, then cut
 * the carrier before the body — every explicit-header receiver on the channel
 * commits its RX window for the announced duration while we occupy the channel
 * only for the preamble and the 8 header symbols.
 *
 * The header is built by the chip in normal explicit mode, so its length/CR/CRC
 * fields and the header CRC are spec-correct by construction; the "fake" is that
 * the body it promises is never sent. We size the announced payload length so the
 * post-header airtime a receiver computes for it is closest to the requested ms.
 *
 * Synchronous: this is a deliberate one-shot command, so a ~header-length task
 * stall (tens of ms) is acceptable here, unlike on the RX hot path. */
static void manualTxProt(LoraRadio* r) {
    int    sf  = r->cfgSf, bw = r->cfgBwHz, pre = r->cfgPreamble;
    double tSym = (double)((uint32_t)1 << sf) / (double)bw;   /* seconds */
    int    de   = (tSym > 0.016) ? 1 : 0;                     /* LDRO, as in loraAirtimeSeconds */

    /* Announced length whose post-header 4/8 airtime is closest to the target.
     * The receiver's commit after the header is blocks·(CR+4)·Tsym; for 4/8 that
     * is blocks·8·Tsym. Scan against the same model the device reports airtime
     * with so the figure matches what a receiver derives from the header. */
    double target = (double)r->mtxProtMs / 1000.0;
    int    bestL = 0;
    double bestErr = 1e30, bestRem = 0.0;
    for (int L = 0; L <= 255; L++) {
        double num    = 8.0 * L - 4.0 * sf + 28.0 + 16.0 /*CRC on*/;
        double den    = 4.0 * (sf - 2 * de);
        double blocks = fmax(ceil(num / den), 0.0);
        double rem    = blocks * 8.0 * tSym;                  /* CR+4 = 8 for 4/8 */
        double err    = fabs(rem - target);
        if (err < bestErr) { bestErr = err; bestL = L; bestRem = rem; }
    }

    /* Air only the preamble + the 8-symbol header block, plus a few symbols of
     * margin so the whole header is certainly modulated before we abort. */
    double hdrSec = (pre + 4.25 + 8.0 + 4.0) * tSym;
    uint16_t hdrMs = (uint16_t)ceil(hdrSec * 1000.0);

    /* Explicit header (announces a length) at 4/8, so both the header's own CR
     * field and the length→airtime mapping are those of a 4/8 packet. */
    int16_t st = radioHeaderMode(r, false, 0);
    if (st == RADIOLIB_ERR_NONE) st = radioSetCodingRate(r, 8);
    if (st == RADIOLIB_ERR_NONE) st = r->radio->standby();
    if (st != RADIOLIB_ERR_NONE) {
        radioSetCodingRate(r, (uint8_t)r->cfgCr);
        rearmRx(r);
        manualTxFinish(r, false, "modem cfg failed");
        return;
    }
    r->airImplicit = false;
    apApplyPower(r, r->cfgTxp);

    /* The body is never heard, so its contents do not matter. */
    static uint8_t dummy[256] = {0};
    r->txFrameStartMs = millis();
    st = r->radio->startTransmit(dummy, (size_t)bestL);
    if (st != RADIOLIB_ERR_NONE) {
        radioSetCodingRate(r, (uint8_t)r->cfgCr);
        rearmRx(r);
        manualTxFinish(r, false, "startTransmit failed");
        return;
    }
    delay(hdrMs);                     /* let the preamble + header go out … */
    /* … then drop the carrier before the body. finishTransmit both halts
     * modulation (chip → standby) and clears any TX-done IRQ the chip may have
     * latched if the short body actually completed within the delay — so the
     * next serviceRadio pass can't mistake it for a queued frame's completion. */
    r->radio->finishTransmit();

    loraMonPush(r, 1 /*tx*/, r->txFrameStartMs, hdrMs,
                (uint16_t)bestL, 0, 0, r->cfgTxp, LORA_PKT_OURS, 0);
    appcAddAirtime(r, hdrMs);
    r->txFrames++;

    radioSetCodingRate(r, (uint8_t)r->cfgCr);   /* restore the configured payload CR */
    r->txActive = false;
    rearmRx(r);

    char msg[72];
    snprintf(msg, sizeof msg, "committed ~%d ms (announced %d B / 4/8, header ~%d ms on air)",
             (int)lround(bestRem * 1000.0), bestL, (int)hdrMs);
    manualTxFinish(r, true, msg);
}

/* Service a pending manual-TX request and drive the PSA carrier-sense.
 * Runs after serviceRadio in the task loop, so a completed TxDone (which clears
 * txActive and re-arms RX) is already reflected when we check it here. */
static void manualTxPoll(LoraRadio* r) {
    if (r->mtxPhase == MTXP_OFF) {
        if (!r->mtxReq) return;
        r->mtxReq = false;
        if (!r->running) { manualTxFinish(r, false, "radio not up"); return; }
        if (r->splitPending || r->txActive || r->rc.phase != RC_OFF)
            { manualTxFinish(r, false, "radio busy"); return; }

        if (r->mtxKind == MTX_PROT) { manualTxProt(r); return; }

        /* RAW / PSA: the payload goes on air verbatim as one explicit-header
         * frame at the configured params — no RNS seq/split byte, so what the
         * user passed is exactly what airs. */
        memcpy(r->txFrame[0], r->mtxData, r->mtxLen);
        r->txFrameLen[0]  = r->mtxLen;
        r->txFrameCount   = 1;
        r->txFrameSent    = 0;
        r->txPayloadBytes = r->mtxLen;
        r->txType[0]      = LORA_PKT_OURS;
        r->txWaitMs       = 0;
        apApplyPower(r, r->cfgTxp);

        if (r->mtxKind == MTX_PSA && r->lbt) {   /* carrier-sense before firing */
            r->mtxPhase    = MTXP_LBT;
            csmaResetAccess(r);
            r->csmaStart   = xTaskGetTickCount();
            r->mtxDeadline = r->csmaStart +
                (r->lbtTimeoutTicks ? r->lbtTimeoutTicks : pdMS_TO_TICKS(10000));
            return;
        }
        r->mtxPhase = MTXP_TX;
        startTxFrame(r, 0);
        return;
    }

    if (r->mtxPhase == MTXP_LBT) {
        if (csmaClear(r)) { r->mtxPhase = MTXP_TX; startTxFrame(r, 0); return; }
        if ((int32_t)(xTaskGetTickCount() - r->mtxDeadline) >= 0) {
            csmaResetAccess(r);
            manualTxFinish(r, false, "channel busy (LBT timeout)");
        }
        return;
    }

    /* MTXP_TX: serviceRadio clears txActive at TxDone and re-arms RX. */
    if (!r->txActive) manualTxFinish(r, true, "sent");
}

/* Drain one pending outbound packet for this radio if it's free.
 * Half-duplex: while a split RX is being reassembled OR a transmit is already
 * on-air (txActive) we leave bytes sitting in the ITS stream buffer (our
 * outbound TX queue) and revisit once the radio is idle. */
static void drainOneOutbound(LoraRadio* r) {
    /* An active radio check owns the radio (and the CSMA machine) — normal outbound
     * waits in the ITS buffer until it ends (bounded to a few seconds). A queued
     * radio check owns it the same way: rcPoll runs just before us and is
     * mid-CSMA, and the "nothing queued" reset below would clear its progress
     * every pass, so it could never win the channel.
     *
     * Two sources feed one radio: rnsd, and — when the RNode client is bound to
     * this radio — the packet its decoder has parked. Availability is computed
     * across both before anything else, for exactly that reason: gated on rnsd
     * alone, the reset would wipe channel-access progress every pass while an
     * rnode packet waited, and that packet could never win the channel. The
     * rnode source needs no rnsd handle — the client is an endpoint of this
     * segment in its own right.
     *
     * Run the wait clock before any of the blocking returns below, so it counts
     * time lost to a probe or a linkage frame owning the radio and not just to
     * channel contention. */
    size_t rnsdAvail  = (r->running && r->rnsdHandle >= 0)
                            ? itsBytesAvailable(r->rnsdHandle) : 0;
    bool   rnodeAvail = r->running && s_rnode.handle >= 0 &&
                        s_rnode.radio == r->idx && s_rnode.txLen > 0;
    bool   any        = rnsdAvail > 0 || rnodeAvail;
    if (!any)                  r->txWaitPend = false;
    else if (!r->txWaitPend) { r->txWaitPend = true; r->txWaitStartMs = millis(); }

    if (r->mtxPhase != MTXP_OFF) return;
    /* A radio check owns the radio for its whole run: the announce bunches are
     * back-to-back by design and the tail is a schedule receivers are timing. */
    if (r->rc.phase != RC_OFF) return;
    if (!r->running || r->splitPending || r->txActive) return;
    if (!any) {
        csmaResetAccess(r);         /* nothing queued → reset channel-access state */
        return;
    }
    /* Both pending → alternate, so neither endpoint starves the other. */
    bool takeRnode = rnodeAvail && (rnsdAvail == 0 || s_rnode.txAlternate);
    if (!csmaClear(r)) {            /* listen-before-talk not yet satisfied */
        TickType_t waited = xTaskGetTickCount() - r->csmaStart;
        /* Radio contention is otherwise invisible until the drop valve fires —
         * name it explicitly once per frame so a "nothing went out" hunt can
         * rule the channel in or out at a glance. */
        if (!r->csmaStalled && r->csmaPhase != CSMA_IDLE &&
            waited >= pdMS_TO_TICKS(1000)) {
            r->csmaStalled = true;
            if (r->appc)
                warn("lora/%d LBT: tx stalled %u ms by channel contention "
                     "(phase=%s cw=%d/%d slots band=%u airtime=%d%% noise=%.0f dBm)",
                     r->idx, (unsigned)(waited * portTICK_PERIOD_MS),
                     r->csmaPhase == CSMA_DIFS ? "difs" : "backoff",
                     (int)(r->appcCwPassed / (r->appcSlotTicks ? r->appcSlotTicks : 1)),
                     r->appcCw, (unsigned)r->appcBand,
                     (int)(appcAirtime(r) * 100.0f), (double)r->noiseFloor);
            else
                warn("lora/%d LBT: tx stalled %u ms by channel contention "
                     "(phase=%s cw=%d noise=%.0f dBm)",
                     r->idx, (unsigned)(waited * portTICK_PERIOD_MS),
                     r->csmaPhase == CSMA_DIFS ? "difs" : "backoff",
                     r->csmaCw, (double)r->noiseFloor);
        }
        /* Channel never cleared within lbt_timeout → drop the head frame instead
         * of blocking the outbound queue behind a wedged-busy channel. */
        if (r->lbtTimeoutTicks && waited >= r->lbtTimeoutTicks) {
            size_t n;
            if (takeRnode) {
                n = s_rnode.txLen;
                s_rnode.txLen = 0;
                /* Release the client's queue even though the packet never
                 * aired: with flow control on it would otherwise wedge. */
                rnodeSendReady();
            } else {
                static uint8_t drop[RNS_MTU + 16];
                n = itsRecv(r->rnsdHandle, drop, sizeof(drop), 0);
            }
            r->txDropped++;
            /* Consumers gate send-failure attribution on this counter at
             * settle time, so it is mirrored the moment a frame is shed —
             * the coalesced stats flush alone lags up to a second. */
            {
                char b[48];
                storageSet(rk(b, sizeof b, r->idx, "stats.tx_dropped"),
                           (int)(r->txDropped & 0x7fffffff));
            }
            err("lora/%d LBT: channel busy > %u ms, dropped %u B frame",
                r->idx, (unsigned)r->lbtTimeoutMs, (unsigned)n);
            csmaResetAccess(r);         /* re-arm access state for the next frame */
            r->csmaStalled = false;
        }
        return;
    }
    r->csmaStalled = false;
    if (rnodeAvail && rnsdAvail > 0) s_rnode.txAlternate = !s_rnode.txAlternate;
    auto stampWait = [&]() {
        uint32_t waited = r->txWaitPend ? millis() - r->txWaitStartMs : 0;
        r->txWaitMs   = (uint16_t)(waited > 0xFFFF ? 0xFFFF : waited);
        r->txWaitPend = false;
    };
    if (takeRnode) {
        size_t n = s_rnode.txLen;
        /* Cleared before the transmit, so the pump can start decoding the next
         * packet the moment this one is framed. */
        s_rnode.txLen = 0;
        stampWait();
        beginTx(r, s_rnode.txPkt, n, LORA_ORIG_RNODE);
        return;
    }
    static uint8_t pkt[RNS_MTU + 16];
    size_t n = itsRecv(r->rnsdHandle, pkt, sizeof(pkt), 0);
    if (n > 0) {
        stampWait();
        beginTx(r, pkt, n, LORA_ORIG_RNSD);
    }
}

/* Ask the chip what just completed and act on it — the IRQ flags are ground
 * truth, so we never guess TX-vs-RX from software state. Half-duplex, so at most
 * one of TX_DONE / RX_DONE is set. txActive is consulted only to run the TxDone
 * watchdog when the chip reports nothing (a wedged transmit). */
static void serviceRadio(LoraRadio* r) {
    uint32_t flags = r->radio->getIrqFlags();

    if (flags & (1UL << RADIOLIB_IRQ_TX_DONE)) {
        r->radio->finishTransmit();          /* clear IRQ, chip → standby */
        r->txFrames++;
        /* Record the frame that just went out (one per split half). */
        uint8_t  doneIdx  = r->txFrameSent;
        uint8_t  doneType = r->txType[doneIdx];
        uint32_t dur = (uint32_t)lround(1000.0 * loraAirtimeSeconds(
                           r->airSf, r->cfgBwHz, r->cfgCr, r->airPreamble,
                           (int)r->txFrameLen[doneIdx], r->airImplicit));
        /* Everything but our own air protocol carries the 1-byte seq/split
         * header on air; the record reports payload bytes, so strip it.
         * RNode-origin packets go out through that same framing as rnsd's. */
        loraMonPush(r, 1 /*tx*/, r->txFrameStartMs, (uint16_t)dur,
                    (uint16_t)(r->txFrameLen[doneIdx] -
                               (doneType == LORA_PKT_OURS ? 0 : 1)),
                    0, 0, r->txPwrNow, doneType,
                    doneIdx == 0 ? r->txWaitMs : 0);
        /* Every frame we put on air counts toward the APPC band, probe and
         * linkage frames included — they occupy the channel like any other. */
        appcAddAirtime(r, dur);
        if (++r->txFrameSent < r->txFrameCount) {   /* split: send the second half */
            startTxFrame(r, r->txFrameSent);
            return;
        }
        r->txBytes += r->txPayloadBytes;
        txRearmRx(r);                        /* whole packet sent → back to listening */
        return;
    }

    if (flags & (1UL << RADIOLIB_IRQ_RX_DONE)) {
        handleRxDone(r);
        return;
    }

    /* Nothing completed. If a transmit is outstanding and overdue, the chip is
     * wedged — recover rather than block outbound forever. */
    if (r->txActive && (int32_t)(xTaskGetTickCount() - r->txDeadline) >= 0) {
        warn("lora/%d TxDone timeout — aborting frame, re-arming RX", r->idx);
        r->radio->finishTransmit();
        txRearmRx(r);
    }
}

static void onRnsdRecv(int handle, size_t /*bytesAvail*/) {
    LoraRadio* r = radioByHandle(handle);
    if (r) drainOneOutbound(r);
}

/* ─────────────── config reload ─────────────── */

static void applyConfig(LoraRadio* r) {
    char kb[48];
    r->enabled = storageGetInt(sk(kb, sizeof kb, r->idx, "enable"), 0) != 0;

    if (!r->enabled) {
        if (r->running) {
            info("lora/%d disable", r->idx);
            radioStop(r);
        }
        return;
    }
    /* If already running, stop and start to pick up new params. Cheap
     * (~30 ms) and avoids tracking which fields changed. */
    if (r->running) radioStop(r);
    radioStart(r);
}

static void onCfgChange(const char* /*key*/, const char* /*val*/) {
    cfgArm(LORA_CFG_COALESCE_MS);
}

/* ─────────── unit bridge: Hz storage ↔ human display keys ───────────
 * Frequency/bandwidth are stored in Hz, but the settings pane (and the CLI)
 * speak MHz/kHz. We mirror each Hz config key to an ephemeral display key —
 * lora.<i>.freq_mhz / lora.<i>.bw_khz, a trimmed decimal that a plain text
 * field in the generated pane binds to — and reconcile the other way when that
 * field is edited. All storage writes happen on the task (the change callback
 * only raises a flag); each direction writes only when the value really
 * differs, so the persistent↔display round-trip can't loop. An unparseable or
 * out-of-range entry is reverted to the stored value (the pane's "ignore if
 * not valid"). */
#define LORA_FREQ_MIN_HZ  100000000    /* 100 MHz */
#define LORA_FREQ_MAX_HZ  2000000000   /* 2 GHz — storage ints are int32, keep the cast safe */
#define LORA_BW_MIN_HZ    5000          /* 5 kHz */
#define LORA_BW_MAX_HZ    1700000       /* 1.7 MHz */

/* Hz → trimmed decimal in `scale` units (1e6 MHz, 1e3 kHz); empty if unset. */
static void hzToUnit(char* out, size_t n, int hz, double scale) {
    if (hz <= 0) { if (n) out[0] = '\0'; return; }
    snprintf(out, n, "%.6f", (double)hz / scale);
    char* p = out + strlen(out) - 1;
    while (p > out && *p == '0') *p-- = '\0';   /* trim trailing zeros … */
    if (p > out && *p == '.') *p = '\0';         /* … and a bare trailing dot */
}

/* Human decimal in `scale` units → Hz; -1 if non-numeric or overflowing. */
static int unitToHz(const char* s, double scale) {
    if (!s || !*s) return -1;
    char* end = nullptr;
    double v = strtod(s, &end);
    if (end == s) return -1;                     /* no digits */
    while (*end == ' ') end++;
    if (*end) return -1;                          /* trailing junk → invalid */
    double hz = v * scale;
    if (hz < 0.0 || hz > 2.1e9) return -1;        /* keep the int32 cast in range */
    return (int)(hz + 0.5);
}

/* Hz config → ephemeral display keys (one radio). */
static void loraPublishDisplay(int i) {
    char kb[48], vb[24];
    storageBegin();
    hzToUnit(vb, sizeof vb, storageGetInt(sk(kb, sizeof kb, i, "frequency"), 0), 1.0e6);
    storageSet(rk(kb, sizeof kb, i, "freq_mhz"), vb);
    hzToUnit(vb, sizeof vb, storageGetInt(sk(kb, sizeof kb, i, "bandwidth"), 0), 1.0e3);
    storageSet(rk(kb, sizeof kb, i, "bw_khz"), vb);
    storageEnd();
}

/* Edited display keys → Hz config (one radio). Each field writes only on a real
 * change; an invalid entry is reverted to the stored value. A persisted write
 * re-fires onCfgChange, which reconfigures the radio and re-publishes here. */
static void loraApplyDisplay(int i) {
    char kb[48], vb[24];

    storageBegin();
    storageGetStr(rk(kb, sizeof kb, i, "freq_mhz"), vb, sizeof vb, "");
    int want = unitToHz(vb, 1.0e6);
    if (want >= LORA_FREQ_MIN_HZ && want <= LORA_FREQ_MAX_HZ) {
        if (want != storageGetInt(sk(kb, sizeof kb, i, "frequency"), 0))
            storageSet(sk(kb, sizeof kb, i, "frequency"), want);
    } else if (vb[0]) {                           /* invalid entry → snap back */
        hzToUnit(vb, sizeof vb, storageGetInt(sk(kb, sizeof kb, i, "frequency"), 0), 1.0e6);
        storageSet(rk(kb, sizeof kb, i, "freq_mhz"), vb);
    }

    storageGetStr(rk(kb, sizeof kb, i, "bw_khz"), vb, sizeof vb, "");
    want = unitToHz(vb, 1.0e3);
    if (want >= LORA_BW_MIN_HZ && want <= LORA_BW_MAX_HZ) {
        if (want != storageGetInt(sk(kb, sizeof kb, i, "bandwidth"), 0))
            storageSet(sk(kb, sizeof kb, i, "bandwidth"), want);
    } else if (vb[0]) {
        hzToUnit(vb, sizeof vb, storageGetInt(sk(kb, sizeof kb, i, "bandwidth"), 0), 1.0e3);
        storageSet(rk(kb, sizeof kb, i, "bw_khz"), vb);
    }
    storageEnd();
}

static void onDisplayChange(const char* /*key*/, const char* /*val*/) {
    s_displayDirty = true;
    if (s_task) xTaskNotifyGive(s_task);
}

/* ─────────────── RNode endpoint: implementation ───────────────
 * (protocol overview and the client-side traps at RNODE_ITS_PORT, top of file) */

static bool rnodeEnabled(void) { return storageGetInt("s.lora.rnode.enable", 0) != 0; }

static int rnodeRadioIdx(void) {
    int i = storageGetInt("s.lora.rnode.radio", 0);
    return (i >= 0 && i < kNumRadios) ? i : 0;
}

/* KISS encode buffer — lora task only. Worst case is every payload byte
 * escaping to two, plus the delimiters and the command byte. */
static uint8_t s_rnodeTx[2 * RNS_MTU + 8];

/* Encode and send one whole command frame. Whole frame or nothing: the client
 * never flushes a command frame it has only part of, so a stream-mode short
 * write would leave it waiting for the rest forever and desynchronise
 * everything after it. */
static bool rnodeSendCmd(uint8_t cmd, const uint8_t* payload, size_t n) {
    if (s_rnode.handle < 0) return false;
    size_t o = 0;
    s_rnodeTx[o++] = KISS_FEND;
    s_rnodeTx[o++] = cmd;
    for (size_t i = 0; i < n && o + 3 <= sizeof(s_rnodeTx); i++) {
        uint8_t b = payload[i];
        if (b == KISS_FEND)      { s_rnodeTx[o++] = KISS_FESC; s_rnodeTx[o++] = KISS_TFEND; }
        else if (b == KISS_FESC) { s_rnodeTx[o++] = KISS_FESC; s_rnodeTx[o++] = KISS_TFESC; }
        else                     { s_rnodeTx[o++] = b; }
    }
    s_rnodeTx[o++] = KISS_FEND;
    if (itsSpacesAvailable(s_rnode.handle) < o) return false;
    return itsSend(s_rnode.handle, s_rnodeTx, o, 0) == o;
}

/* Release the client's transmit queue after one of its packets has finished
 * with the radio. Harmless with flow control off, mandatory with it on. The
 * payload byte is not optional: the client dispatches a command on its payload
 * bytes, so a frame with none is read and discarded. */
static void rnodeSendReady(void) {
    uint8_t z = 0;
    rnodeSendCmd(RN_CMD_READY, &z, 1);
}

/* Radio or rnsd → client. All-or-nothing: the stat frames and the data frame
 * are space-checked together and skipped together, because a partial write in
 * the middle of a KISS frame corrupts the stream from there on. */
static void rnodeForwardData(LoraRadio* r, const uint8_t* data, size_t len, bool withStats) {
    RnodeState& S = s_rnode;
    if (S.handle < 0 || r->idx != S.radio || len == 0 || len > RNS_MTU) return;
    size_t need = 2 * len + 3 + (withStats ? 10 : 0);
    if (itsSpacesAvailable(S.handle) < need) {
        warn("lora/%d rnode: client stream full, dropped %u B", r->idx, (unsigned)len);
        return;
    }
    if (withStats) {
        /* Both, in this order, ahead of the data frame: the client holds them
         * as sticky attributes applied to the NEXT data frame and cleared after
         * it, and its transport drops the SNR unless an RSSI came with it. */
        int rv = (int)lroundf(r->rssiLast) + RN_RSSI_OFFSET;
        uint8_t rb = (uint8_t)(rv < 0 ? 0 : rv > 255 ? 255 : rv);
        rnodeSendCmd(RN_CMD_STAT_RSSI, &rb, 1);
        int sv = (int)lroundf(r->snrLast * 4.0f);
        int8_t sb = (int8_t)(sv < -128 ? -128 : sv > 127 ? 127 : sv);
        rnodeSendCmd(RN_CMD_STAT_SNR, (const uint8_t*)&sb, 1);
    }
    rnodeSendCmd(RN_CMD_DATA, data, len);
}

/* A client command changed (or restated) the radio configuration. Arms the
 * coalesced apply and marks the echo owed — self-arming even when every write
 * was a no-op, so the client always gets its echo and never sits out the
 * validation window waiting for one that storage saw no reason to trigger. */
static void rnodeCfgTouched(void) {
    s_rnode.echoPend = true;
    cfgArm(LORA_CFG_COALESCE_MS);
}

/* Report the state that was just applied. bw / txpower / sf / state are the
 * echoes the client compares unconditionally — an absent one reads as a
 * mismatch; frequency is optional but must be within 100 Hz if present, which
 * is why it comes from the applied value rather than from what was asked; cr is
 * never validated. A mismatch costs a full reconnect every 5 s, forever, so
 * this runs from the apply pass and nowhere earlier. */
static void rnodeEchoFlush(void) {
    RnodeState& S = s_rnode;
    if (S.handle < 0 || !S.echoPend) return;
    S.echoPend = false;
    LoraRadio* r = &s_radios[S.radio];
    char kb[48];

    auto be32 = [](uint8_t* o, uint32_t v) {
        o[0] = (uint8_t)(v >> 24); o[1] = (uint8_t)(v >> 16);
        o[2] = (uint8_t)(v >> 8);  o[3] = (uint8_t)v;
    };
    uint8_t b4[4];
    be32(b4, (uint32_t)storageGetInt(sk(kb, sizeof kb, S.radio, "frequency"), 0));
    rnodeSendCmd(RN_CMD_FREQUENCY, b4, 4);
    be32(b4, (uint32_t)r->cfgBwHz);
    rnodeSendCmd(RN_CMD_BANDWIDTH, b4, 4);
    uint8_t v = (uint8_t)r->cfgTxp;
    rnodeSendCmd(RN_CMD_TXPOWER, &v, 1);
    v = (uint8_t)r->cfgSf;
    rnodeSendCmd(RN_CMD_SF, &v, 1);
    v = (uint8_t)r->cfgCr;
    rnodeSendCmd(RN_CMD_CR, &v, 1);
    v = r->running ? RN_RADIO_ON : RN_RADIO_OFF;
    rnodeSendCmd(RN_CMD_RADIO_STATE, &v, 1);

    if (S.wantOn && !r->running) {
        /* The one error code the client turns into a clean teardown and retry.
         * 0x03/0x04 are unhandled ("Unknown hardware failure") and 0x21/0x22
         * crash its handler outright — see the note at RNODE_ITS_PORT. */
        uint8_t e = RN_ERROR_INITRADIO;
        rnodeSendCmd(RN_CMD_ERROR, &e, 1);
    }
}

static void rnodeDropSession(void) {
    if (s_rnode.handle < 0) return;
    itsDisconnect(s_rnode.handle);      /* releases a claimed serial port too */
    s_rnode.handle   = -1;
    s_rnode.txLen    = 0;
    s_rnode.inLen    = s_rnode.inPos = 0;
    s_rnode.offPend  = false;
    s_rnode.echoPend = false;
    s_rnode.wantOn   = false;
}

/* ── command execution ── */

static void rnodeFrame(void) {
    RnodeState& S = s_rnode;
    char kb[48];

    switch (S.cmd) {

    /* Handshake. Answered every time it is asked, not once: over TCP the client
     * re-sends the whole four-command detect burst after 3.5 s of transmit idle
     * for the life of the connection, so these replies must be stateless and
     * repeatable. A CMD_DETECT frame carrying anything other than DETECT_RESP
     * actively clears the client's detected flag. */
    case RN_CMD_DETECT: {
        if (S.len < 1 || S.buf[0] != RN_DETECT_REQ) break;
        uint8_t resp = RN_DETECT_RESP;
        rnodeSendCmd(RN_CMD_DETECT, &resp, 1);
        break;
    }
    case RN_CMD_FW_VERSION: {
        uint8_t ver[2] = { RN_FW_MAJ, RN_FW_MIN };
        rnodeSendCmd(RN_CMD_FW_VERSION, ver, 2);
        break;
    }
    case RN_CMD_PLATFORM: {
        uint8_t p = RN_PLATFORM_AVR;
        rnodeSendCmd(RN_CMD_PLATFORM, &p, 1);
        break;
    }
    case RN_CMD_MCU: {
        uint8_t m = RN_MCU_1284P;
        rnodeSendCmd(RN_CMD_MCU, &m, 1);
        break;
    }

    /* Radio configuration. Executed by writing the ordinary config keys, so it
     * flows through the same path an operator's edit takes — and persists. */
    case RN_CMD_FREQUENCY: {
        if (S.len < 4) break;
        uint32_t hz = ((uint32_t)S.buf[0] << 24) | ((uint32_t)S.buf[1] << 16) |
                      ((uint32_t)S.buf[2] << 8)  |  (uint32_t)S.buf[3];
        if (hz < (uint32_t)LORA_FREQ_MIN_HZ || hz > (uint32_t)LORA_FREQ_MAX_HZ) {
            warn("lora/%d rnode: frequency %u Hz out of range, ignored", S.radio, (unsigned)hz);
            break;
        }
        storageBegin();
        storageSet(sk(kb, sizeof kb, S.radio, "frequency"), (int)hz);
        storageEnd();
        rnodeCfgTouched();
        break;
    }
    case RN_CMD_BANDWIDTH: {
        if (S.len < 4) break;
        uint32_t hz = ((uint32_t)S.buf[0] << 24) | ((uint32_t)S.buf[1] << 16) |
                      ((uint32_t)S.buf[2] << 8)  |  (uint32_t)S.buf[3];
        if (hz < (uint32_t)LORA_BW_MIN_HZ || hz > (uint32_t)LORA_BW_MAX_HZ) {
            warn("lora/%d rnode: bandwidth %u Hz out of range, ignored", S.radio, (unsigned)hz);
            break;
        }
        storageBegin();
        storageSet(sk(kb, sizeof kb, S.radio, "bandwidth"), (int)hz);
        storageEnd();
        rnodeCfgTouched();
        break;
    }
    case RN_CMD_TXPOWER: {
        if (S.len < 1) break;
        int txp = (int8_t)S.buf[0];
        if (txp > RNODE_TXP_MAX) {
            /* Clamped and echoed honestly. Such a client reads the echo as a
             * mismatch and re-dials every 5 s — a churn loop rather than a
             * single abort — which is the price of not lying to it. */
            warn("lora/%d rnode: tx power %d dBm above the %d dBm ceiling, clamped",
                 S.radio, txp, RNODE_TXP_MAX);
            txp = RNODE_TXP_MAX;
        }
        storageBegin();
        storageSet(sk(kb, sizeof kb, S.radio, "tx_power"), txp);
        storageEnd();
        rnodeCfgTouched();
        break;
    }
    case RN_CMD_SF: {
        if (S.len < 1) break;
        int sf = S.buf[0];
        if (sf < 5 || sf > 12) {
            warn("lora/%d rnode: spreading factor %d out of range, ignored", S.radio, sf);
            break;
        }
        storageBegin();
        storageSet(sk(kb, sizeof kb, S.radio, "spreading_factor"), sf);
        storageEnd();
        rnodeCfgTouched();
        break;
    }
    case RN_CMD_CR: {
        if (S.len < 1) break;
        int cr = S.buf[0];
        if (cr < 5 || cr > 8) {
            warn("lora/%d rnode: coding rate 4/%d out of range, ignored", S.radio, cr);
            break;
        }
        storageBegin();
        storageSet(sk(kb, sizeof kb, S.radio, "coding_rate"), cr);
        storageEnd();
        rnodeCfgTouched();
        break;
    }

    /* The configuration burst always ends with this, so pulling the apply
     * deadline in here puts the apply and its echo inside the client's
     * validation sleep (0.25 s on serial, 1.5 s on TCP). The burst still
     * coalesces: the earlier writes armed 300 ms and nothing has applied yet. */
    case RN_CMD_RADIO_STATE: {
        if (S.len < 1) break;
        if (S.buf[0] == RN_RADIO_ON) {
            S.offPend = false;
            S.wantOn  = true;
            storageBegin();
            storageSet(sk(kb, sizeof kb, S.radio, "enable"), 1);
            storageEnd();
            rnodeCfgTouched();
        } else if (S.buf[0] == RN_RADIO_OFF) {
            /* Deferred, not obeyed now. A clean client shutdown is OFF then
             * LEAVE then close, and taking the radio down for rnsd on every
             * such exit is not what "the host has left" means. LEAVE and
             * disconnect cancel it; a client that turns the radio off and stays
             * connected is honoured at the deadline. */
            S.offPend = true;
            S.wantOn  = false;
            rnodeCfgTouched();
        } else {
            S.echoPend = true;      /* ASK — report, change nothing */
        }
        cfgArm(0);
        break;
    }

    /* Parsed and echoed as zero, never enforced: airtime governance here is
     * LBT/APPC plus rnsd's announce cap. The client parses these echoes and
     * never validates them. */
    case RN_CMD_ST_ALOCK:
    case RN_CMD_LT_ALOCK: {
        uint8_t z[2] = { 0, 0 };
        rnodeSendCmd(S.cmd, z, 2);
        break;
    }

    case RN_CMD_LEAVE:
        S.offPend = false;
        S.txLen   = 0;
        rnodeDropSession();
        return;

    case RN_CMD_DATA:
        if (S.len > 0 && S.len <= RNS_MTU) {
            memcpy(S.txPkt, S.buf, S.len);
            S.txLen = S.len;
        }
        break;

    default:
        /* Everything else is ignored in silence. A client with a beacon
         * configured also injects unsolicited callsign data frames; those are
         * plain CMD_DATA and simply air. */
        break;
    }
}

/* ── KISS decoder ── */

static void rnodeByte(uint8_t b) {
    RnodeState& S = s_rnode;
    if (b == KISS_FEND) {
        /* A close is also an open — back-to-back frames share the delimiter —
         * and this is where the decoder resyncs. A freshly accepted socket can
         * carry a partial frame: the client's TCP layer buffers writes while
         * disconnected and flushes them on the next successful one. */
        if (S.inFrame && S.haveCmd && !S.overflow) rnodeFrame();
        S.inFrame = true;
        S.haveCmd = S.escape = S.overflow = false;
        S.len = 0;
        return;
    }
    if (!S.inFrame) return;                     /* pre-sync bytes: wait for a FEND */
    if (!S.haveCmd) { S.cmd = b; S.haveCmd = true; return; }
    if (S.escape) {
        S.escape = false;
        if (b == KISS_TFEND) b = KISS_FEND;
        else if (b == KISS_TFESC) b = KISS_FESC;
    } else if (b == KISS_FESC) {
        S.escape = true;
        return;
    }
    if (S.len < sizeof(S.buf)) S.buf[S.len++] = b;
    else                       S.overflow = true;   /* swallow the rest to the next FEND */
}

/* Client → decoder. The ITS ring is the inbound queue: while a decoded packet
 * is parked waiting for the channel this stops reading, so backpressure reaches
 * the client instead of a second packet overwriting the first. */
static void rnodePump(void) {
    RnodeState& S = s_rnode;
    if (S.handle < 0) return;
    for (;;) {
        while (S.inPos < S.inLen) {
            if (S.txLen) return;                /* parked — the rest stays buffered */
            rnodeByte(S.inBuf[S.inPos++]);
            if (S.handle < 0) return;           /* CMD_LEAVE dropped the session */
        }
        if (S.txLen) return;
        S.inPos = S.inLen = 0;
        size_t n = itsRecv(S.handle, S.inBuf, sizeof(S.inBuf), 0);
        if (n == 0) return;
        S.inLen = n;
    }
}

/* ── transports ── */

/* Open or close the endpoint's two doors to match the settings. Called from the
 * coalesced apply pass, which is also what puts the net registration on this
 * task — where net requires it to originate. */
static void rnodeApplyTransports(void) {
    bool en    = rnodeEnabled();
    int  radio = rnodeRadioIdx();

    /* A client bound to a radio the settings no longer point at — or to an
     * endpoint that has just been switched off — is holding a session that no
     * longer means anything. */
    if (s_rnode.handle >= 0 && (!en || s_rnode.radio != radio)) rnodeDropSession();

#if CONFIG_SPANGAP_NET
    /* TCP, in the two steps net's endpoint model asks for: register once, then
     * drive the listener by writing the port — net's own s.net.* subscriber
     * opens and closes the socket from there. Port 7633 is the only port a
     * stock client can dial. */
    static bool registered = false;
    if (!registered) {
        net_port_msg_t reg = {};
        reg.itsPort     = RNODE_ITS_PORT;
        reg.tcpNoDelay  = 1;
        reg.keepAlive   = 1;
        reg.backlog     = 1;
        reg.defaultPort = 0;          /* never auto-open; gated by the key below */
        safeStrncpy(reg.nvsKey, "rnode_port", sizeof(reg.nvsKey));
        if (itsSendAux("net", NET_PORT_REG_PORT, &reg, sizeof(reg), pdMS_TO_TICKS(500)))
            registered = true;
        else
            warn("lora rnode: net endpoint registration failed");
    }
    if (registered) {
        int tcp  = storageGetInt("s.lora.rnode.tcp", 0);
        int want = (en && tcp > 0) ? tcp : 0;
        if (storageGetInt("s.net.rnode_port", -1) != want) storageSet("s.net.rnode_port", want);
    }
#endif

    /* Serial. A claim is dormant until a client actually attaches, so holding
     * one costs nothing; what it does cost is esptool auto-reset on that port.
     * A refused claim (port 1 while the console presents only one port) is
     * warned once and retried when the port count changes. */
    static int have = -1, triedPort = -2, triedCount = -1;
    int ports = storageGetInt("sys.usb.serial_ports", 1);
    int want  = en ? storageGetInt("s.lora.rnode.serial", -1) : -1;
    if (have >= 0 && have != want) { serialPortRelease(have); have = -1; }
    if (want >= 0 && have < 0 && !(triedPort == want && triedCount == ports)) {
        if (serialPortClaim(want, TAG, RNODE_ITS_PORT)) { have = want; triedPort = -2; }
        else { triedPort = want; triedCount = ports; }
    }
}

/* ── ITS server callbacks (lora task, via itsPoll) ── */

static int onRnodeConnect(int handle, const void* /*data*/, size_t len) {
    /* One session at a time, across every transport. Enforced here as well as
     * by the port's single handle, because the refusal is what keeps a serial
     * takeover from disturbing the console while a TCP client is attached. */
    if (s_rnode.handle >= 0 || !rnodeEnabled() || s_stop) return -1;
    /* Serial or network is readable only from the connect payload's length —
     * the serial machinery sends its own one-byte struct, net a net_connect_t. */
    bool serial = (len == sizeof(serial_handler_connect_t));
    /* Fresh decoder for a fresh stream. Field by field rather than assigning a
     * RnodeState{} — the temporary would be over a kilobyte of task stack. */
    RnodeState& S = s_rnode;
    S.inFrame = S.haveCmd = S.escape = S.overflow = false;
    S.cmd = 0; S.len = 0;
    S.inLen = S.inPos = 0;
    S.txLen = 0;
    S.echoPend = S.offPend = S.wantOn = S.txAlternate = false;
    S.handle = handle;
    S.radio  = rnodeRadioIdx();
    info("lora/%d rnode: client attached over %s", S.radio, serial ? "serial" : "tcp");
    return 0;
}

static void onRnodeRecv(int /*handle*/, size_t /*bytesAvail*/) { rnodePump(); }

static void onRnodeDisconnect(int /*ref*/) {
    if (s_rnode.handle < 0) return;
    info("lora/%d rnode: client detached", s_rnode.radio);
    s_rnode.handle   = -1;
    s_rnode.txLen    = 0;
    s_rnode.inLen    = s_rnode.inPos = 0;
    s_rnode.offPend  = false;           /* a clean detach leaves the radio up */
    s_rnode.echoPend = false;
    s_rnode.wantOn   = false;
}

/* Honour a radio-off the client asked for and stayed connected through. Runs at
 * the apply deadline, before the apply itself, so the write lands in the same
 * pass it gates. */
static void rnodeSettleOff(void) {
    if (s_rnode.handle < 0 || !s_rnode.offPend) return;
    s_rnode.offPend = false;
    char kb[48];
    storageBegin();
    storageSet(sk(kb, sizeof kb, s_rnode.radio, "enable"), 0);
    storageEnd();
}

/* ─────────────── CLI ─────────────── */

static const char* foundStr(const LoraRadio* r) {
    return r->found == 1 ? "found" : r->found == 0 ? "NOT FOUND" : "unprobed";
}

static void cliPrintSlot(int i) {
    LoraRadio* r = &s_radios[i];
    const LoraSlot* s = r->slot;
    cliPrintf("lora/%d  radio=%-6s [%s]  state=%s\n", i, chipName(s->chip), foundStr(r),
              r->running ? "up" : (r->enabled ? "starting" : "down"));
    cliPrintf("        pins cs=%d irq=%d busy=%d rst=%d  tcxo=%dmV  dio2_rf=%d  rfsw=%d/%d\n",
              s->cs, s->dio1, s->busy, s->rst, s->tcxo_mv, s->dio2_rf_switch ? 1 : 0,
              s->rfsw_rx, s->rfsw_tx);

    char kb[48];
    int  freq_hz = storageGetInt(sk(kb, sizeof kb, i, "frequency"), 0);
    int  bw_hz   = storageGetInt(sk(kb, sizeof kb, i, "bandwidth"), 0);
    int  sf      = storageGetInt(sk(kb, sizeof kb, i, "spreading_factor"), 0);
    int  cr      = storageGetInt(sk(kb, sizeof kb, i, "coding_rate"), 0);
    int  txp     = storageGetInt(sk(kb, sizeof kb, i, "tx_power"), 0);
    int  pre     = storageGetInt(sk(kb, sizeof kb, i, "preamble"), 12);
    char mode[24]; storageGetStr(sk(kb, sizeof kb, i, "mode"), mode, sizeof mode, "access_point");
    char sync[16]; storageGetStr(sk(kb, sizeof kb, i, "sync_word"), sync, sizeof sync, "0x42");
    cliPrintf("        freq=%.3f MHz  bw=%.0f kHz  sf=%d  cr=4/%d  txp=%d dBm  preamble=%d\n",
              freq_hz / 1.0e6, bw_hz / 1.0e3, sf, cr, txp, pre);
    cliPrintf("        sync=%s  mode=%s  bitrate=%u bit/s\n", sync, mode, (unsigned)r->curBitrate);
    if (chipFamily(s->chip) == FAM_SX126X)
        cliPrintf("        rx_boosted_gain=%d\n",
                  storageGetInt(sk(kb, sizeof kb, i, "rx_boosted_gain"), 1) != 0);
    if (!r->lbt) {
        cliPrintf("        lbt=off (blind tx)\n");
    } else if (!r->appc) {
        cliPrintf("        lbt=on  appc=off  slot=%u ms  difs=%u ms  cw=2^%d slots\n",
                  (unsigned)(r->slotTicks * portTICK_PERIOD_MS),
                  (unsigned)(r->difsTicks * portTICK_PERIOD_MS), r->csmaCw);
    } else {
        uint8_t band = appcLiveBand(r);
        cliPrintf("        lbt=on  appc=on  slot=%u ms  difs=%u ms  "
                  "airtime=%d%%  band=%u/%d (cw %d-%d slots)\n",
                  (unsigned)(r->appcSlotTicks * portTICK_PERIOD_MS),
                  (unsigned)(r->appcDifsTicks * portTICK_PERIOD_MS),
                  (int)(appcAirtime(r) * 100.0f), (unsigned)band, APPC_CW_BANDS,
                  (band - 1) * APPC_CW_PER_BAND_WINDOWS,
                  band * APPC_CW_PER_BAND_WINDOWS - 2);
    }
    cliPrintf("        rx %u/%uB  tx %u/%uB  rssi %d dBm  snr %d dB  crc_err %u  split_to %u\n",
              (unsigned)r->rxFrames, (unsigned)r->rxBytes,
              (unsigned)r->txFrames, (unsigned)r->txBytes,
              (int)r->rssiLast, (int)r->snrLast,
              (unsigned)r->crcErr, (unsigned)r->splitTimeouts);
    {
        uint32_t rem = (int32_t)(r->rc.nextRunMs - millis()) > 0
                           ? (r->rc.nextRunMs - millis()) / 1000 : 0;
        cliPrintf("        announces %d buffered  next check in %us\n",
                  annCount(r), (unsigned)rem);
        /* Dropped telemetry is invisible on the graph and reads as frames that
         * were never transmitted, so say it out loud rather than let someone
         * chase a radio bug that isn't one. */
        if (r->monDropped || r->rssiDropped)
            cliPrintf("        telemetry dropped: %u frame records, %u rssi samples"
                      " (interface queue full)\n",
                      (unsigned)r->monDropped, (unsigned)r->rssiDropped);
    }
}

/* ── `lora [<n>] neighbors` — the passive radio-neighbourhood picture ── */

static void neiAgo(char* b, size_t n, uint32_t now, uint32_t then) {
    uint32_t s = (now - then) / 1000;
    if (s < 120)        snprintf(b, n, "%us", (unsigned)s);
    else if (s < 7200)  snprintf(b, n, "%um", (unsigned)(s / 60));
    else                snprintf(b, n, "%uh", (unsigned)(s / 3600));
}

/* ── reciprocity estimate: the radio check's measurement, guessed for free ──
 * The probe learns the TX power at which our signal lands on the peer's demod
 * floor by *asking* it. The same number can be inferred from a frame we heard
 * FROM them, if we assume a power they transmitted at: their path loss is ours.
 * This computes exactly the quantity a radio check measures as the us->them cliff,
 * so the two are directly comparable — which is the point. Against a peer that
 * has been probed we have ground truth to check the estimate against; against a
 * non-cooperating peer the estimate is all there will ever be.
 *
 * `s.lora.assumed_peer_txp` (default 22) is the power we credit an unprobed peer
 * with. Assuming high errs safe — a peer that is actually quieter makes us
 * over-estimate path loss and transmit higher than needed. It also has to be
 * settable, because a bench node parked at a low `tx_power` announces at that
 * power, not at 22, and the estimate would be off by the difference. */
static int neiAssumedPeerTxp(void) {
    int v = storageGetInt("s.lora.assumed_peer_txp", 22);
    if (v < -30 || v > 30) v = 22;
    return v;
}

/* Recent mean signal for a node, from the 5-minute bucket ring, in the same
 * byte encoding the probe uses. Returns the sample count (0 = nothing recent). */
static uint32_t neiRecentSignal(const Neighbor* e, uint32_t now,
                                uint8_t* rssiB, int8_t* snrQ) {
    uint32_t absNow = now / NEI_BUCKET_MS, cnt = 0;
    int64_t rs = 0, ss = 0;
    for (int b = 0; b < NEI_BUCKETS; b++) {
        const NeiBucket* bk = &e->buck[b];
        if (bk->cnt && absNow - bk->absIdx < NEI_BUCKETS) {
            cnt += bk->cnt; rs += bk->rssiSum; ss += bk->snrSum10;
        }
    }
    if (!cnt) return 0;
    int rssi = (int)(rs / (int64_t)cnt);
    int snr10 = (int)(ss / (int64_t)cnt);
    int rb = -rssi;
    *rssiB = (uint8_t)(rb < 0 ? 0 : rb > 255 ? 255 : rb);
    int sq = snr10 * 4 / 10;
    *snrQ = (int8_t)(sq < -128 ? -128 : sq > 127 ? 127 : sq);
    return cnt;
}

/* Link margin above the demodulation floor, deci-dB, from an encoded rssi/snr
 * pair. How much power the sender could have dropped and still been decoded —
 * which, subtracted from its assumed transmit power, is the reciprocity
 * estimate of the power we need toward it. */
static int neiHeadroom10(const LoraRadio* r, uint8_t rssiB, int8_t snrQ) {
    /* Semtech's required SNR per spreading factor, deci-dB: −7.5 dB at SF7 and
     * 2.5 dB worse per factor below it, 2.5 dB better per factor above. */
    int sf = r->cfgSf < 5 ? 5 : (r->cfgSf > 12 ? 12 : r->cfgSf);
    int floor10 = -75 - (sf - 7) * 25;
    int snr10   = ((int)snrQ * 10) / 4;
    int margin  = snr10 - floor10;
    (void)rssiB;                 /* the margin is an SNR question, not a level one */
    return margin > 0 ? margin : 0;
}

/* Estimated us->them cliff, deci-dBm. */
static bool neiEstimateCliff10(const LoraRadio* r, const Neighbor* e,
                               uint32_t now, int* cliff10, uint32_t* samples) {
    uint8_t rssiB; int8_t snrQ;
    uint32_t n = neiRecentSignal(e, now, &rssiB, &snrQ);
    if (samples) *samples = n;
    if (!n) return false;
    *cliff10 = neiAssumedPeerTxp() * 10 - neiHeadroom10(r, rssiB, snrQ);
    return true;
}

/* Walk the table in display order — us first, then the others — handing each
 * node its printed number. `want` < 0 visits everything; otherwise the walk
 * stops at that node number. Returns the matched node, or null. The printer and
 * the CLI's node resolver share this so the numbers always agree. */
typedef void (*NeiVisitFn)(Neighbor* e, int num, void* ud);
static Neighbor* neiWalk(NeiState* st, int want, NeiVisitFn fn, void* ud) {
    int num = 0;
    for (int pass = 0; pass < 2; pass++) {
        for (int k = 0; k < NEI_MAX; k++) {
            Neighbor* e = &st->nei[k];
            if (!e->used || (pass == 0) != neiIsLocal(e)) continue;
            /* Pass 0 is the local rows — us and the RNode client. Both number 0
             * and neither is addressable: naming `0` would be aiming the
             * radio at this device. */
            int n = neiIsLocal(e) ? 0 : ++num;
            if (want >= 0) { if (n == want && !neiIsLocal(e)) return e; continue; }
            if (fn) fn(e, n, ud);
        }
    }
    return nullptr;
}

struct NeiPrintCtx { LoraRadio* r; uint32_t now; bool verbose; };

static void neiPrintNode(Neighbor* e, int num, void* ud) {
    NeiPrintCtx* c = (NeiPrintCtx*)ud;
    char hex[33], ago[16], lbl[8];
    if (e->isRnode)   safeStrncpy(lbl, "rnode", sizeof lbl);
    else if (e->isUs) safeStrncpy(lbl, "us", sizeof lbl);
    else              snprintf(lbl, sizeof lbl, "%d", num);

    /* One line per hash: full hash, aspect, and the announced name if any.
     * The transport aspect leads — it is the hash every node has. */
    bool first = true;
    for (int pass = 0; pass < 2; pass++) {
        for (int d = 0; d < e->nDests; d++) {
            NeiDest* nd = &e->dests[d];
            const char* asp = nd->haveName ? neiNameLabel(nd->nameHash) : nullptr;
            bool isTransport = asp && strcmp(asp, "rnstransport.probe") == 0;
            if ((pass == 0) != isTransport) continue;
            loraHex(hex, nd->hash, 16);
            char nh[21] = "";
            if (nd->haveName && !asp) loraHex(nh, nd->nameHash, 10);
            cliPrintf("  %-5s%s %s", first ? lbl : "", hex,
                      asp ? asp : (nd->haveName ? nh : "-"));
            if (nd->name[0]) cliPrintf("  \"%s\"", nd->name);
            if (c->verbose) {
                neiAgo(ago, sizeof ago, c->now, nd->lastMs);
                if (nd->announces) cliPrintf("  ann %u", (unsigned)nd->announces);
                cliPrintf("  %s ago", ago);
            }
            cliPrintf("\n");
            first = false;
        }
    }
    /* Hashes a peer linked to this node that we have never heard directly. */
    for (int l = 0; l < e->nLink4; l++) {
        cliPrintf("  %-5s%02x%02x%02x%02x........................ (not seen yet)\n",
                  first ? lbl : "",
                  e->link4[l][0], e->link4[l][1], e->link4[l][2], e->link4[l][3]);
        first = false;
    }
    if (first) {   /* nothing but a bare node key */
        if (e->haveNode4)
            cliPrintf("  %-5s%02x%02x%02x%02x........................ (not seen yet)\n",
                      lbl, e->node4[0], e->node4[1], e->node4[2], e->node4[3]);
        else
            cliPrintf("  %-5s(no hash seen)\n", lbl);
    }

    if (!neiIsLocal(e)) {
        /* Capability line. TRANSPORT means it forwards for others; ROAMING is
         * its node-flags bit; the mesh tag that it speaks our air protocol; TX
         * the power a probe settled on for it; EST the reciprocity estimate;
         * USE the power we transmit to it at under adaptive_txpwr. */
        char f[96];
        int o = 0;
        auto add = [&](const char* t) {
            o += snprintf(f + o, sizeof f - (size_t)o, "%s%s", o ? ", " : "", t);
        };
        if (e->transit)  add("TRANSPORT");
        if (e->roaming)  add("ROAMING");
        if (e->ourProto) add(RF_PROTO_NAME);
        if (e->haveTxPwr) {
            char t[16];
            snprintf(t, sizeof t, "TX %d", (int)e->txPwr);
            add(t);
        }
        int est10;
        if (neiEstimateCliff10(c->r, e, c->now, &est10, nullptr)) {
            char t[24];
            snprintf(t, sizeof t, "EST %.0f", (double)est10 / 10.0);
            add(t);
        }
        /* USE is the power frames to this node actually go out at; the `~`
         * marks one derived from EST plus a margin rather than measured. */
        if (e->haveApPwr) {
            char t[16];
            snprintf(t, sizeof t, "USE %s%d", e->apFromEst ? "~" : "", (int)e->apPwr);
            add(t);
        }
        if (o) cliPrintf("       ( %s )\n", f);
    }

    if (c->verbose) {
        for (int n = 0; n < e->nIds; n++) {
            loraHex(hex, e->ids[n], 16);
            cliPrintf("       id:%s\n", hex);
        }
        if (e->haveSig) {
            neiAgo(ago, sizeof ago, c->now, e->lastHeardMs);
            cliPrintf("       rssi %d..%d dBm  snr %.1f..%.1f dB  heard %s ago\n",
                      (int)e->rssiMin, (int)e->rssiMax,
                      (double)e->snrMin10 / 10.0, (double)e->snrMax10 / 10.0, ago);
        }
        if (e->haveQuality)
            cliPrintf("       q %u/255 (%u/%u proofs)%s\n",
                      (unsigned)e->quality, (unsigned)e->qProved, (unsigned)e->qSent,
                      e->provesData ? "  proves-data" : "");
        if (e->haveAdv)
            cliPrintf("       hashes %d/%u\n", neiKnownHashes(e), (unsigned)e->advHashes);
        uint32_t absNow = c->now / NEI_BUCKET_MS;
        uint32_t cnt = 0; int64_t rs = 0, ss = 0;
        for (int b = 0; b < NEI_BUCKETS; b++) {
            const NeiBucket* bk = &e->buck[b];
            if (bk->cnt && absNow - bk->absIdx < NEI_BUCKETS) {
                cnt += bk->cnt; rs += bk->rssiSum; ss += bk->snrSum10;
            }
        }
        if (cnt)
            cliPrintf("       1h: %u pkt  avg %d dBm %.1f dB\n",
                      (unsigned)cnt, (int)(rs / (int64_t)cnt),
                      (double)ss / (double)cnt / 10.0);
    }
    cliPrintf("\n");
}

static void cliPrintNeighbors(int i, bool verbose) {
    LoraRadio* r = &s_radios[i];
    NeiState*  st = r->nei;
    if (!st) {
        cliPrintf("lora/%d neighbors: no observations (radio has never been up)\n", i);
        return;
    }
    uint32_t now = millis();
    int nUs = 0, nRnode = 0, nNodes = 0, nLinks = 0;
    for (int k = 0; k < NEI_MAX; k++) {
        Neighbor* e = &st->nei[k];
        if (!e->used) continue;
        if (e->isRnode)   nRnode++;
        else if (e->isUs) nUs++;
        else              nNodes++;
    }
    for (int k = 0; k < NEI_LINKS_MAX; k++)
        if (st->links[k].used) nLinks++;

    /* The local rows are named rather than counted: there is at most one of
     * each, and which of them exist is the interesting part. */
    const char* local = nUs && nRnode ? " and us + rnode"
                      : nUs           ? " and us"
                      : nRnode        ? " and rnode" : "";
    char ago[16];
    neiAgo(ago, sizeof ago, now, st->sinceMs);
    cliPrintf("lora/%d neighbors: %d other%s%s, %d open link%s (observing %s)\n\n",
              i, nNodes, nNodes == 1 ? "" : "s", local,
              nLinks, nLinks == 1 ? "" : "s", ago);
    if (r->curIfacSize)
        cliPrintf("  note: ifac enabled — frames are masked, passive parse sees nothing\n\n");

    NeiPrintCtx ctx = { r, now, verbose };
    neiWalk(st, -1, neiPrintNode, &ctx);

    if (!verbose) return;
    for (int k = 0; k < NEI_LINKS_MAX; k++) {
        NeiLink* L = &st->links[k];
        if (!L->used) continue;
        char lid[33], dst[36];
        loraHex(lid, L->linkId, 16);
        if (L->haveDest) loraHex(dst, L->dest, 16);
        else             snprintf(dst, sizeof dst, "?");
        neiAgo(ago, sizeof ago, now, L->lastMs);
        cliPrintf("  link %s -> %s  %s %s", lid, dst,
                  L->ours ? "ours" : "seen",
                  L->unresolved ? "unresolved" : (L->established ? "established" : "pending"));
        if (L->haveSig)
            cliPrintf("  %d dBm %.1f dB", (int)L->lastRssi, (double)L->lastSnr10 / 10.0);
        cliPrintf("  %u pkt  %s ago\n", (unsigned)L->frames, ago);
    }
}




/* The first-4 a probe should be addressed to for this node — its transport
 * hash where known, since that is the one every node has. */
static bool neiNodeFirst4(const Neighbor* e, uint8_t out[4]) {
    for (int d = 0; d < e->nDests; d++) {
        const char* asp = e->dests[d].haveName ? neiNameLabel(e->dests[d].nameHash) : nullptr;
        if (asp && strcmp(asp, "rnstransport.probe") == 0) { memcpy(out, e->dests[d].hash, 4); return true; }
    }
    if (e->nDests)   { memcpy(out, e->dests[0].hash, 4); return true; }
    if (e->haveNode4) { memcpy(out, e->node4, 4); return true; }
    if (e->nLink4)   { memcpy(out, e->link4[0], 4); return true; }
    return false;
}







/* `tok` abbreviates `full` when it is a prefix of it and at least `minLen`
 * long — so `lora n` / `lora neigh` / `lora neighbours` and `lora a` /
 * `lora announce` all reach the same place. */
static bool cliVerb(const char* tok, const char* full, size_t minLen) {
    size_t n = strlen(tok);
    return n >= minLen && n <= strlen(full) && strncmp(tok, full, n) == 0;
}
/* `lora [<n>] a[nnounce]` — replay every buffered announce, then the radio
 * check, now rather than at the next beat. The beat restarts from the run's
 * end, so this also reschedules. */
static void cliAnnounce(int idx) {
    if (idx < 0 || idx >= kNumRadios) { cliPrintf("no radio %d\n", idx); return; }
    LoraRadio* r = &s_radios[idx];
    if (!r->running) { cliPrintf("lora/%d not running\n", idx); return; }
    int n = annCount(r);
    if (n == 0) {
        cliPrintf("lora/%d nothing buffered — no announce has been originated here yet\n", idx);
        return;
    }
    r->rc.req = true;
    if (s_task) xTaskNotifyGive(s_task);
    cliPrintf("lora/%d replaying %d announce%s, then a radio check (~%u ms)\n",
              idx, n, n == 1 ? "" : "s", (unsigned)rcTailMs(r, n));
}

static bool cliIsNeighbors(const char* t) {
    return cliVerb(t, "neighbors", 1) || cliVerb(t, "neighbours", 1);
}

/* Pointer into `orig` just past the first `skip` whitespace-separated tokens,
 * with the remaining text kept verbatim (embedded spaces included). Returns null
 * if there are fewer than `skip` tokens; may point at the terminating NUL (empty
 * remainder). Used to recover a tx payload from the untruncated args, since the
 * tokeniser above copies only the first bytes for dispatch. */
static const char* cliRest(const char* orig, int skip) {
    const char* p = orig ? orig : "";
    for (int i = 0; i < skip; i++) {
        while (*p == ' ') p++;
        if (!*p) return nullptr;
        while (*p && *p != ' ') p++;
    }
    while (*p == ' ') p++;
    return p;
}

/* Parse a tx payload string into raw bytes. A literal `0x` followed by an even
 * run of hex digits inserts those bytes (`0x0a`, `0x48656c6c6f`); every other
 * character is taken as its own ASCII byte, spaces included. Returns the byte
 * count, or -1 on overflow or an odd-length 0x run. */
static int cliParseBytes(const char* s, uint8_t* out, size_t cap) {
    auto hx = [](char c) -> int {
        c = (char)tolower((unsigned char)c);
        return c <= '9' ? c - '0' : c - 'a' + 10;
    };
    size_t n = 0;
    while (*s) {
        if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X') &&
            isxdigit((unsigned char)s[2]) && isxdigit((unsigned char)s[3])) {
            const char* h = s + 2;
            size_t hd = 0;
            while (isxdigit((unsigned char)h[hd])) hd++;
            if (hd & 1) return -1;                 /* half a byte → malformed */
            for (size_t i = 0; i < hd; i += 2) {
                if (n >= cap) return -1;
                out[n++] = (uint8_t)((hx(h[i]) << 4) | hx(h[i + 1]));
            }
            s = h + hd;
        } else {
            if (n >= cap) return -1;
            out[n++] = (uint8_t)*s++;
        }
    }
    return (int)n;
}

/* tx / tx_psa / tx_prot: hand a manual-transmit request to the lora task and
 * block on its result. `cmd` is the verb, `rest` the verbatim argument tail. */
static void cliManualTx(long idx, const char* cmd, const char* rest) {
    LoraRadio* r = &s_radios[idx];
    bool isProt = strcmp(cmd, "tx_prot") == 0;
    bool isPsa  = strcmp(cmd, "tx_psa")  == 0;
    if (!r->running)                    { cliPrintf("lora/%ld is not up\n", idx); return; }
    if (r->mtxPhase != MTXP_OFF || r->mtxReq) {
        cliPrintf("lora/%ld tx already in progress\n", idx); return;
    }
    if (isProt) {
        if (!rest || !*rest) { cliPrintf("usage: lora %ld tx_prot <ms>\n", idx); return; }
        long ms = strtol(rest, nullptr, 10);
        if (ms <= 0)      { cliPrintf("tx_prot: <ms> must be > 0\n"); return; }
        if (ms > 0xFFFF)  ms = 0xFFFF;
        r->mtxProtMs = (uint16_t)ms;
        r->mtxKind   = MTX_PROT;
    } else {
        if (!rest || !*rest) {
            cliPrintf("usage: lora %ld %s <string>   (0x<hex> inserts raw bytes)\n", idx, cmd);
            return;
        }
        int n = cliParseBytes(rest, r->mtxData, 255);   /* one explicit frame, ≤255 B */
        if (n < 0)  { cliPrintf("%s: payload > 255 B or bad 0x<hex> run\n", cmd); return; }
        if (n == 0) { cliPrintf("%s: empty payload\n", cmd); return; }
        r->mtxLen  = (uint16_t)n;
        r->mtxKind = isPsa ? MTX_PSA : MTX_RAW;
    }

    uint32_t gen = r->mtxResGen;
    r->mtxReq = true;
    if (s_task) xTaskNotifyGive(s_task);
    /* RAW/PROT complete in a few ms; PSA can back off up to lbt_timeout. Cap the
     * wait well past the worst case (SF12 APPC + a busy channel). */
    for (int i = 0; i < 400 && r->mtxResGen == gen; i++) delay(50);   /* ≤ 20 s */
    if (r->mtxResGen == gen) { cliPrintf("lora/%ld %s: no result (timeout)\n", idx, cmd); return; }
    cliPrintf("lora/%ld %s: %s%s\n", idx, cmd,
              r->mtxResOk ? "" : "failed: ", r->mtxResMsg);
}

static void cliLora(const char* args) {
    char buf[80];
    safeStrncpy(buf, args ? args : "", sizeof buf);
    char* tok[4] = {};
    int   nt = 0;
    char* save = nullptr;
    for (char* t = strtok_r(buf, " ", &save); t && nt < 4; t = strtok_r(nullptr, " ", &save))
        tok[nt++] = t;

    if (nt == 0) {                                  /* `lora` → all slots */
        for (int i = 0; i < kNumRadios; i++) cliPrintSlot(i);
        return;
    }
    if (strcmp(tok[0], "help") == 0 || strcmp(tok[0], "-h") == 0) {
        cliPrintf("%-*s LoRa status for all radios\n",      CLI_HELP_COL, "lora");
        cliPrintf("%-*s status for one radio\n",            CLI_HELP_COL, "lora <n>");
        cliPrintf("%-*s enable/disable (no <n> = all)\n",   CLI_HELP_COL, "lora [<n>] up|down");
        cliPrintf("%-*s observed direct neighbours (-v for detail)\n", CLI_HELP_COL, "lora [<n>] n[eighbors]");
        cliPrintf("%-*s replay buffered announces, then a radio check\n", CLI_HELP_COL, "lora [<n>] a[nnounce]");
        cliPrintf("%-*s freq MHz / bw kHz / sf / cr /\n",   CLI_HELP_COL, "lora <n> <param> <val>");
        cliPrintf("%-*s   txp dBm / preamble / sync / mode / lbt 0|1 / appc 0|1 /\n", CLI_HELP_COL, "");
        cliPrintf("%-*s   rx_boosted_gain 0|1\n", CLI_HELP_COL, "");
        cliPrintf("%-*s blind-transmit a payload (0x<hex> = raw bytes)\n", CLI_HELP_COL, "lora <n> tx <string>");
        cliPrintf("%-*s carrier-sense (as normal tx), then transmit\n", CLI_HELP_COL, "lora <n> tx_psa <string>");
        cliPrintf("%-*s emit a header committing receivers for <ms> (4/8)\n", CLI_HELP_COL, "lora <n> tx_prot <ms>");
        return;
    }
    if (cliIsNeighbors(tok[0])) {                           /* all radios */
        bool v = nt > 1 && strcmp(tok[1], "-v") == 0;
        for (int i = 0; i < kNumRadios; i++) cliPrintNeighbors(i, v);
        return;
    }
    if (cliVerb(tok[0], "announce", 1)) { cliAnnounce(0); return; }   /* no index → radio 0 */

    char kb[48];
    /* `lora up|down` → all radios. */
    if (nt == 1 && (strcmp(tok[0], "up") == 0 || strcmp(tok[0], "down") == 0)) {
        int v = strcmp(tok[0], "up") == 0 ? 1 : 0;
        storageBegin();
        for (int i = 0; i < kNumRadios; i++) storageSet(sk(kb, sizeof kb, i, "enable"), v);
        storageEnd();
        cliPrintf("%s %d radio(s)\n", v ? "enabled" : "disabled", kNumRadios);
        return;
    }

    /* `lora <n> ...` */
    char* end = nullptr;
    long  idx = strtol(tok[0], &end, 10);
    if (end == tok[0] || *end || idx < 0 || idx >= kNumRadios) {
        cliPrintf("no such radio '%s' (have 0..%d)\n", tok[0], kNumRadios - 1);
        return;
    }
    if (nt == 1) { cliPrintSlot((int)idx); return; }

    const char* cmd = tok[1];
    if (strcmp(cmd, "up") == 0)   { storageSet(sk(kb, sizeof kb, idx, "enable"), 1); cliPrintf("lora/%ld enabled\n", idx);  return; }
    if (strcmp(cmd, "down") == 0) { storageSet(sk(kb, sizeof kb, idx, "enable"), 0); cliPrintf("lora/%ld disabled\n", idx); return; }
    if (cliIsNeighbors(cmd)) {
        cliPrintNeighbors((int)idx, nt > 2 && strcmp(tok[2], "-v") == 0);
        return;
    }
    /* `lora [<n>] a[nnounce]` — replay every buffered announce, then the radio
     * check, now rather than at the next beat. The beat restarts from here. */
    if (cliVerb(cmd, "announce", 1)) { cliAnnounce((int)idx); return; }

    if (strcmp(cmd, "tx") == 0 || strcmp(cmd, "tx_psa") == 0 || strcmp(cmd, "tx_prot") == 0) {
        cliManualTx(idx, cmd, cliRest(args, 2));
        return;
    }

    if (nt < 3) { cliPrintf("usage: lora %ld <freq|bw|sf|cr|txp|preamble|sync|mode|lbt|appc|rx_boosted_gain> <value>\n", idx); return; }
    const char* val = tok[2];

    /* Human units in: frequency MHz, bandwidth kHz. Storage stays in Hz. */
    if (strcmp(cmd, "freq") == 0) {
        double mhz = atof(val);
        storageSet(sk(kb, sizeof kb, idx, "frequency"), (int)(mhz * 1.0e6));
        cliPrintf("lora/%ld freq = %.3f MHz\n", idx, mhz);
    } else if (strcmp(cmd, "bw") == 0) {
        double khz = atof(val);
        storageSet(sk(kb, sizeof kb, idx, "bandwidth"), (int)(khz * 1.0e3));
        cliPrintf("lora/%ld bw = %.0f kHz\n", idx, khz);
    } else if (strcmp(cmd, "sf") == 0) {
        storageSet(sk(kb, sizeof kb, idx, "spreading_factor"), atoi(val));
        cliPrintf("lora/%ld sf = %d\n", idx, atoi(val));
    } else if (strcmp(cmd, "cr") == 0) {
        storageSet(sk(kb, sizeof kb, idx, "coding_rate"), atoi(val));
        cliPrintf("lora/%ld cr = 4/%d\n", idx, atoi(val));
    } else if (strcmp(cmd, "txp") == 0) {
        storageSet(sk(kb, sizeof kb, idx, "tx_power"), atoi(val));
        cliPrintf("lora/%ld txp = %d dBm\n", idx, atoi(val));
    } else if (strcmp(cmd, "preamble") == 0) {
        storageSet(sk(kb, sizeof kb, idx, "preamble"), atoi(val));
        cliPrintf("lora/%ld preamble = %d\n", idx, atoi(val));
    } else if (strcmp(cmd, "sync") == 0) {
        storageSet(sk(kb, sizeof kb, idx, "sync_word"), val);
        cliPrintf("lora/%ld sync = %s\n", idx, val);
    } else if (strcmp(cmd, "mode") == 0) {
        storageSet(sk(kb, sizeof kb, idx, "mode"), val);
        cliPrintf("lora/%ld mode = %s\n", idx, val);
    } else if (strcmp(cmd, "lbt") == 0) {
        int on = atoi(val) != 0;
        storageSet(sk(kb, sizeof kb, idx, "lbt"), on);
        cliPrintf("lora/%ld lbt = %s\n", idx, on ? "on (carrier-sense before tx)" : "off (blind tx)");
    } else if (strcmp(cmd, "appc") == 0) {
        int on = atoi(val) != 0;
        storageSet(sk(kb, sizeof kb, idx, "appc"), on);
        cliPrintf("lora/%ld appc = %s\n", idx,
                  on ? "on (contention window banded by own airtime)"
                     : "off (exponential backoff on collisions)");
        if (on && !storageGetInt(sk(kb, sizeof kb, idx, "lbt"), 1))
            cliPrintf("        note: inert while lbt = 0\n");
    } else if (strcmp(cmd, "rx_boosted_gain") == 0) {
        int on = atoi(val) != 0;
        storageSet(sk(kb, sizeof kb, idx, "rx_boosted_gain"), on);
        cliPrintf("lora/%ld rx_boosted_gain = %s (SX126x only)\n", idx,
                  on ? "on (boosted, +~0.4 mA RX)" : "off (power saving)");
    } else {
        cliPrintf("unknown: lora %ld %s (try freq|bw|sf|cr|txp|preamble|sync|mode|lbt|appc|rx_boosted_gain)\n", idx, cmd);
    }
}

/* ─────────────── task ─────────────── */

/* Stats publishing is event-driven, not timed. Every stat is either a cumulative
 * counter or a last-packet reading, so none of them move without a tx/rx event —
 * republishing on a timer just burns battery. So we publish only after a counter
 * changes, and at most once a second (a change inside the 1 s window is deferred
 * to the boundary, then coalesced). RX is IRQ-woken (loraRadioIsr → task notify;
 * DIO1 is a light-sleep wake source) and outbound wakes via ITS, so with nothing
 * pending the task blocks until a real event and the chip light-sleeps. */
#define LORA_STATS_MIN_MS 1000

static TickType_t nextDeadline(void) {
    TickType_t now = xTaskGetTickCount();
    /* Idle default: block until an ISR/ITS wake. Shrunk below only for real
     * pending work — a deferred stats flush, a registration retry, or outbound. */
    TickType_t soonest = portMAX_DELAY;
    /* A coalesced config apply is owed at its deadline. */
    if (s_cfgPend) {
        TickType_t d = (int32_t)(s_cfgDueTick - now) > 0 ? (TickType_t)(s_cfgDueTick - now) : 0;
        if (d < soonest) soonest = d;
    }
    /* Stats and LoRaMon maintenance keep their own beat on the interface task,
     * so nothing here has to hold a timer for them. */
    for (int i = 0; i < kNumRadios; i++) {
        LoraRadio* r = &s_radios[i];
        /* Registration retry while a radio is running-but-unregistered (a
         * transient window: onRnsdDisconnect nulls the handle and the loop
         * re-registers). Poll at 1 Hz until it takes. */
        if (r->running && r->enabled && r->rnsdHandle < 0) {
            TickType_t d = pdMS_TO_TICKS(LORA_STATS_MIN_MS);
            if (d < soonest) soonest = d;
        }
        /* Outbound queued and radio free. With LBT off, loop immediately.
         * With LBT on and channel access mid-procedure, wake at the next slot
         * boundary to re-sense — never spin at 0, which would peg the task.
         * Skipped while a transmit is on-air (txActive): the TxDone IRQ drives
         * the next step, and drainOneOutbound would no-op anyway. */
        /* A manual tx request just arrived, or its carrier-sense is mid-backoff:
         * service it now / re-sense at slot pace. */
        if (r->mtxReq) return 0;
        if (r->mtxPhase == MTXP_LBT && r->slotTicks < soonest) soonest = r->slotTicks;
        /* Gating and availability are separate conjunctions: an rnode packet is
         * pending without any rnsd handle, so folding the two together would
         * leave it unable to wake the loop. Undecoded client bytes count too —
         * the pump runs on this task. */
        bool outReady = r->running && !r->splitPending && !r->txActive;
        bool outAvail = (r->rnsdHandle >= 0 && itsBytesAvailable(r->rnsdHandle) > 0) ||
                        (s_rnode.handle >= 0 && s_rnode.radio == r->idx &&
                         (s_rnode.txLen > 0 || itsBytesAvailable(s_rnode.handle) > 0));
        if (outReady && outAvail) {
            if (!r->lbt) return 0;
            /* Sensing cadence is the derived slot in both regimes — APPC's own
             * slot is only the unit its backoff target is counted in, and is
             * always the longer of the two, so re-sensing at slotTicks keeps
             * carrier detection as responsive as it is without appc. */
            TickType_t d = r->slotTicks;
            if (!r->appc && r->csmaPhase == CSMA_BACKOFF) {
                int32_t rem = (int32_t)(r->csmaSlotDeadline - now);
                d = rem > 0 ? (TickType_t)rem : 0;
            }
            if (d == 0) return 0;
            if (d < soonest) soonest = d;
        }
        /* TxDone watchdog fallback — the IRQ normally wakes us first. */
        if (r->txActive) {
            int32_t rem = (int32_t)(r->txDeadline - now);
            TickType_t d = rem > 0 ? (TickType_t)rem : 0;
            if (d < soonest) soonest = d;
        }
        if (r->splitPending) {
            TickType_t d = (r->splitDeadline > now) ? (r->splitDeadline - now) : 0;
            if (d < soonest) soonest = d;
        }
        /* The announce/radio-check beat, and the passes a run in progress needs.
         * The tail runs off its own esp_timer, so only the announce bunches and
         * the tail's carrier sense want the task loop. */
        if (r->running && r->enabled) {
            if (r->rc.phase == RC_LISTEN) {
                /* Mid-sweep: frames are ~14 ms apart and each one has to be
                 * serviced and the radio re-armed before the next. The ISR wakes
                 * us anyway; this is the belt to that brace. */
                if (soonest > 1) soonest = 1;
            } else if (r->rc.phase == RC_ANN || r->rc.phase == RC_TAIL_LBT) {
                if (r->slotTicks < soonest) soonest = r->slotTicks;
            } else if (r->rc.phase == RC_OFF && r->annIntervalMin) {
                int32_t rem = (int32_t)(r->rc.nextRunMs - millis());
                TickType_t d = rem > 0 ? pdMS_TO_TICKS((uint32_t)rem) : 0;
                if (d < soonest) soonest = d;
            }
        }
        /* The channel-RSSI beat. Held even on a silent channel — an idle radio
         * is exactly when the reading means something. */
        if (r->running && r->enabled) {
            int32_t rem = (int32_t)(r->rssiNext - now);
            TickType_t d = rem > 0 ? (TickType_t)rem : 0;
            if (d < soonest) soonest = d;
        }
        /* Outstanding proof expectations: wake at the soonest deadline so a
         * missed proof scores its quality miss without waiting for traffic. */
        if (r->nei) {
            uint32_t nowMs = millis();
            for (int p = 0; p < NEI_PEND_MAX; p++) {
                if (!r->nei->pend[p].used) continue;
                int32_t rem = (int32_t)(r->nei->pend[p].deadlineMs - nowMs);
                TickType_t d = rem > 0 ? pdMS_TO_TICKS((uint32_t)rem) : 0;
                if (d < soonest) soonest = d;
            }
        }
    }
    return soonest;
}

/* ─────────────── interface task ───────────────
 *
 * The storage side of the interface: packet nodes, their expiry, the stats
 * flush, and the channel-RSSI series. Every storage write iface-lora makes on a
 * per-frame or per-second cadence happens here and not on the radio task — see
 * the note at IfMsg for why that separation is the point rather than tidiness.
 *
 * It blocks on the record queue with a short timeout, so it wakes for work and
 * otherwise ticks its own 1 Hz maintenance beat. Priority sits below the radio
 * task's, so a storage op that stalls on the storage task can never delay a
 * channel-access decision. */
/* Parked, it has nothing to block on, so it polls the stop flag. Only the
 * unpark latency depends on this. */
#define LORA_IF_PARK_POLL_MS 100

static TaskHandle_t  s_ifTask   = nullptr;
static volatile bool s_ifParked = false;

/* Publish the newest channel-RSSI sample as one key per radio:
 * `lora.<n>.rssi` = "<ms>|<ch0 dBm>". The device timestamp is in the value
 * rather than the key so a viewer can tell a fresh reading from a repeated one
 * and place it on the same clock the packet nodes use — and so a skipped beat
 * (carrier sense had the radio) simply leaves the key unchanged and reads as a
 * gap. One key rather than a node per sample: the series is live-only, so there
 * is no backlog to mirror and nothing to expire.
 *
 * The channel list is packed rather than singular because the agile channels
 * append to it unchanged once they exist. */
static void loraPublishRssi(LoraRadio* r, const IfMsg* m) {
    char kb[48], val[8 + 6 * LORA_CH_MAX];
    int  w = snprintf(val, sizeof val, "%u", (unsigned)m->t_ms);
    for (int i = 0; i < m->nch && i < LORA_CH_MAX && w > 0 && w < (int)sizeof val; i++)
        w += (m->chRssi[i] == LORA_RSSI_NONE)
                 ? snprintf(val + w, sizeof val - w, "|")
                 : snprintf(val + w, sizeof val - w, "|%d", (int)m->chRssi[i]);
    storageSet(rk(kb, sizeof kb, r->idx, "rssi"), val);
}

static void loraIfTaskMain(void*) {
    info("[%s-if] task up", TAG);
    bool       prevWatch = false;
    TickType_t lastBeat  = 0;
    uint64_t   statsSig   = 0;
    for (;;) {
        while (!s_stop) {
            /* Block until the next record OR the next maintenance beat,
             * whichever is sooner — so an idle interface wakes once a second,
             * not on a poll timer. The radio task's own 1 Hz RSSI sample lands
             * in the queue anyway, so this rarely times out in practice. */
            TickType_t now  = xTaskGetTickCount();
            TickType_t due  = lastBeat + pdMS_TO_TICKS(LORA_STATS_MIN_MS);
            int32_t    rem  = (int32_t)(due - now);
            TickType_t wait = rem > 0 ? (TickType_t)rem : 0;

            IfMsg m;
            if (s_ifq && xQueueReceive(s_ifq, &m, wait) == pdTRUE) {
                if (m.radio < kNumRadios) {
                    LoraRadio* r = &s_radios[m.radio];
                    if (m.kind == IFM_MON)       loraMonRecord(r, &m);
                    else if (m.kind == IFM_RSSI) loraPublishRssi(r, &m);
                }
            }

            /* Checked whether or not a record arrived: a steady stream of them
             * must not be able to starve expiry and the stats flush. */
            now = xTaskGetTickCount();
            if ((int32_t)(now - due) < 0) continue;
            lastBeat = now;

            /* Age every one-hour running total in the system, ours included.
             * One call covers them all; they linked themselves up at
             * construction. */
            {
                static TickType_t lastShift = 0;
                if (lastShift == 0) lastShift = now;
                if ((int32_t)(now - lastShift) >=
                        (int32_t)pdMS_TO_TICKS(Rolling1h::kBucketMinutes * 60u * 1000u)) {
                    lastShift = now;
                    Rolling1h::shiftAll();
                }
            }

            /* Cached for the radio task, which gates recording on it rather
             * than reading storage itself. */
            bool w = loraMonWatched();
            s_monWatched = w;

            /* Stats: counters only move on a tx/rx event, so publish only when
             * the sum of them has changed since the last beat. */
            uint64_t sig = 0;
            for (int i = 0; i < kNumRadios; i++) {
                LoraRadio* r = &s_radios[i];
                sig += r->txBytes + r->rxBytes + r->txFrames + r->rxFrames +
                       r->crcErr + r->splitTimeouts + r->txDropped +
                       (uint32_t)r->rssiLast + (uint32_t)r->snrLast;
            }
            if (sig != statsSig) {
                statsSig = sig;
                for (int i = 0; i < kNumRadios; i++) publishStats(&s_radios[i]);
            }

            /* LoRaMon expiry — 1 Hz while a viewer is open, so nodes age out of
             * the 1 h window even on an idle channel. When the last viewer
             * closes, drop each radio's whole packets subtree. */
            if (w) {
                uint32_t nowMs = millis();
                for (int i = 0; i < kNumRadios; i++) {
                    loraMonExpire(&s_radios[i], nowMs);
                    loraPublishAirtime(&s_radios[i], nowMs);
                }
            } else if (prevWatch) {
                for (int i = 0; i < kNumRadios; i++) loraMonClear(&s_radios[i]);
            }
            prevWatch = w;
        }

        /* rns stop: the radio task parks too, so nothing more will be queued.
         * Drop whatever is still in flight and park on the same flag. */
        if (s_ifq) xQueueReset(s_ifq);
        s_monWatched = false;
        s_ifParked = true;
        while (s_stop) vTaskDelay(pdMS_TO_TICKS(LORA_IF_PARK_POLL_MS));
        s_ifParked = false;
    }
}

static void loraTaskMain(void*) {
    info("[%s] task up (%d radio%s)", TAG, kNumRadios, kNumRadios == 1 ? "" : "s");

    /* No boot barrier here anymore: the RNS orchestrator only calls loraStart()
     * (which spawns this task) after rnsd is up and past its boot window, so the
     * universe is already settled by the time we run. */
    /* Server before client: the first init call sizes this task's shared inbox,
     * and the server's needs are the larger of the two. */
    itsServerInit();
    itsClientInit(kNumRadios);
    /* The RNode endpoint. Both net (a TCP client) and the core serial machinery
     * (a serial client) connect here; onRnodeConnect tells them apart by the
     * connect payload's length. maxHandles=1 plus the explicit reject there is
     * what makes the single-session policy hold across both. */
    itsServerPortOpen(RNODE_ITS_PORT, /*packetBased=*/false, /*maxHandles=*/1, 4096, 4096);
    itsServerOnConnect(RNODE_ITS_PORT, onRnodeConnect);
    itsServerOnRecv(RNODE_ITS_PORT, onRnodeRecv);
    itsServerOnDisconnect(RNODE_ITS_PORT, onRnodeDisconnect);
    neiNamesInit();          /* app.aspect name-hash dictionary for `lora neighbors` */
    storageSubscribeChanges("s.lora", onCfgChange);   /* covers the rnode group too */
    /* The second CDC port only exists while the console is on `usb cdc`, so a
     * serial claim for it has to be re-applied when the transport changes. */
    NOW_AND_ON_CHANGE("sys.usb.serial_ports", { (void)key; (void)val; cfgArm(0); });
    storageSubscribeChanges("secrets.lora", onCfgChange);  /* IFAC passphrase */
    for (int i = 0; i < kNumRadios; i++) {                  /* MHz/kHz pane fields */
        char kb[48];
        storageSubscribeChanges(rk(kb, sizeof kb, i, "freq_mhz"), onDisplayChange);
        storageSubscribeChanges(rk(kb, sizeof kb, i, "bw_khz"),   onDisplayChange);
    }

    /* Construct radio + HAL per slot. The shared SPI bus is brought up
     * idempotently by spi_helper (EspIdfHal::init), so every radio adds
     * its own device on the one bus. begin() is deferred to applyConfig
     * so we only touch RF hardware when a radio is enabled. The board's
     * peripheral power rail (if any) is already up — the buildable owns
     * it (e.g. hw-lilygo-tdeck's tdeckPowerInit), not this interface. */
    for (int i = 0; i < kNumRadios; i++) {
        LoraRadio* r = &s_radios[i];
        r->idx        = i;
        r->slot       = &kSlots[i];
        r->rnsdHandle = -1;

        /* CONFIG_LORA_SPI_HOST is the peripheral *name* (1=SPI1 2=SPI2/FSPI
         * 3=SPI3), matching the Kconfig prompt and the BOARD_*_SPI_HOST headers.
         * The IDF spi_host_device_t enum is offset by one (SPI1_HOST=0,
         * SPI2_HOST=1, SPI3_HOST=2), so subtract one — a straight cast put LoRa
         * on SPI3 while the board's shared bus (LCD + SD) lived on SPI2, and the
         * two controllers fought over the same pins (blank panel, SD DMA
         * failures, no LoRa TX). Mirrors fs.cpp's SD-host mapping. */
        const spi_host_device_t loraHost =
            (spi_host_device_t)(CONFIG_LORA_SPI_HOST - 1);
        r->hal = new EspIdfHal(loraHost,
                               CONFIG_LORA_SCK_PIN, CONFIG_LORA_MOSI_PIN,
                               CONFIG_LORA_MISO_PIN, r->slot->cs);
        r->hal->init();

        r->mod   = new Module(r->hal, r->slot->cs, r->slot->dio1,
                              r->slot->rst, r->slot->busy);
        /* External antenna RF switch driven by two MCU GPIOs (RX_EN/TX_EN), if
         * the board wires one. RADIOLIB_NC for a pin that isn't used. Set on the
         * Module before begin(), so it covers every chip family uniformly. */
        if (r->slot->rfsw_rx >= 0 || r->slot->rfsw_tx >= 0) {
            r->mod->setRfSwitchPins(
                r->slot->rfsw_rx < 0 ? RADIOLIB_NC : (uint32_t)r->slot->rfsw_rx,
                r->slot->rfsw_tx < 0 ? RADIOLIB_NC : (uint32_t)r->slot->rfsw_tx);
        }
        r->radio = radioNew(r->slot->chip, r->mod);
        probeRadio(r);
    }

    /* Clock was already resolved by rnsd before it declared ready (its own
     * waitForTime + boot window ran first), so we don't wait again here. */

    /* Seed the stat keys once so consumers see a radio before any traffic; from
     * here the interface task owns publishing, and does it only on a change. */
    for (int i = 0; i < kNumRadios; i++) publishStats(&s_radios[i]);

  for (;;) {   /* Park, don't delete: this task lives across rns stop/start, so its
                * ITS slot + boost lock are reused, not leaked (rns/INTERNALS §6.1). */
    cfgArm(0);   /* (re)apply config on entry + each resume → radios up + registered */
    while (!s_stop) {
        if (s_cfgPend && (int32_t)(xTaskGetTickCount() - s_cfgDueTick) >= 0) {
            s_cfgPend = false;
            rnodeSettleOff();   /* a radio-off the client stayed connected through */
            for (int i = 0; i < kNumRadios; i++) applyConfig(&s_radios[i]);
            for (int i = 0; i < kNumRadios; i++) loraPublishDisplay(i);   /* Hz → MHz/kHz */
            /* The echo reports the state that was just applied, so it belongs
             * here and nowhere earlier. Transports are (re)opened from the same
             * pass, which also puts the net registration on this task — where
             * net requires it to originate. */
            rnodeEchoFlush();
            rnodeApplyTransports();
        }
        if (s_displayDirty) {
            s_displayDirty = false;
            for (int i = 0; i < kNumRadios; i++) loraApplyDisplay(i);     /* MHz/kHz → Hz */
        }

        /* Poll the chip only when a DIO1 actually fired (or a transmit is in
         * flight, for the watchdog). A wake from ITS / cfg / stats must not cost
         * a getIrqFlags SPI round-trip — that turned every task nudge into radio
         * bus traffic. Atomic read-and-clear so a fire during servicing isn't
         * lost (it re-sets the flag and re-notifies for the next pass). */
        bool radioIrq = __atomic_exchange_n(&s_radioIrq, false, __ATOMIC_SEQ_CST);

        /* Decode whatever the RNode client has sent since the last pass. Also
         * driven from onRnodeRecv; this covers the passes a wake for something
         * else brings us through. */
        rnodePump();

        for (int i = 0; i < kNumRadios; i++) {
            LoraRadio* r = &s_radios[i];
            if (r->running && (radioIrq || r->txActive)) serviceRadio(r);

            if (r->splitPending &&
                (int32_t)(xTaskGetTickCount() - r->splitDeadline) >= 0) {
                r->splitPending = false;
                r->splitTimeouts++;
            }
            if (r->running && r->rnsdHandle < 0 && r->enabled) registerWithRnsd(r);
            neiExpire(r, millis());
            manualTxPoll(r);    /* CLI tx/tx_psa/tx_prot; holds the radio while active */
            rcPoll(r);          /* announce replay + radio check; holds the radio too */
            drainOneOutbound(r);
            /* Last, so every claim on the radio above has already been staked:
             * the sample is only taken if nothing else wanted the chip. */
            rssiSamplePoll(r);
        }

        itsPoll(nextDeadline());
    }   /* end while(!s_stop) */

        /* rns stop: sleep every radio — RF dead, and radioStop deregisters us from
         * rnsd (drops our RNSD_PORT_IFACE conns → onTransportDisconnect frees the
         * interface). Keep the RadioLib objects for the next start. Then PARK on the
         * inbox until loraStart() clears s_stop and notifies. */
        rnodeDropSession();   /* the segment it is an endpoint of is going away */
        for (int i = 0; i < kNumRadios; i++) {
            LoraRadio* r = &s_radios[i];
            if (r->running) radioStop(r);
        }
        s_parked = true;
        info("[%s] stopped", TAG);
        while (s_stop) itsPoll(portMAX_DELAY);
        s_parked = false;
    }
}

/* ── RNS lifecycle hooks (registered with the orchestrator; see rnsServiceRegister) ── */
static void loraStart(void) {
    s_stop = false;
    /* The record queue outlives a stop/start cycle along with the tasks. Created
     * before either of them so a frame recorded on the first pass has somewhere
     * to go. */
    if (!s_ifq) s_ifq = xQueueCreate(LORA_IFQ_DEPTH, sizeof(IfMsg));
    if (!s_task)
        /* 10 KB PSRAM stack: LoRa frame buffers + RadioLib state, plus the
         * inline Ed25519 announce verification of the neighbour table. */
        s_task = spawnTask(loraTaskMain, TAG, 10240, nullptr, 2, CORE_SECONDARY_NO_LCD, STACK_PSRAM);
    else
        xTaskNotifyGive(s_task);   /* un-park the resident task */
    /* The interface task sits one priority below the radio task: a storage op
     * that stalls must never be able to delay a channel-access decision. Its
     * stack is the smaller of the two — it holds no frame buffers, only the
     * record it popped and the key/value it formats. */
    if (!s_ifTask)
        s_ifTask = spawnTask(loraIfTaskMain, "lora-if", 4096, nullptr, 1,
                             CORE_SECONDARY_NO_LCD, STACK_PSRAM);
}

static void loraStop(void) {
    if (!s_task || s_stop) return;
    s_stop = true;
    xTaskNotifyGive(s_task);   /* break the work loop; the task parks, not deleted */
    for (int i = 0; i < 300 && !s_parked; i++) delay(10);   /* await park */
    if (!s_parked) warn("[%s] stop timed out", TAG);
    /* The interface task blocks on its queue for up to one beat, so give it
     * comfortably more than a beat to notice the flag and park. */
    for (int i = 0; i < 300 && !s_ifParked; i++) delay(10);
    if (!s_ifParked) warn("[%s-if] stop timed out", TAG);
}

void LoraService::onInit() {
    char kb[48];
    if (storageGetInt("s.lora.version", 0) < LORA_VERSION) {
        /* Per-radio defaults. Frequency + TX power are user-must-pick
         * (region / antenna); everything else defaults so an enable-toggle
         * alone gets a radio up. */
        /* Radio 0 is seeded from the settings: block in straddle.yaml, except
         * bandwidth — its pane row now binds the kHz display key, not
         * s.lora.0.bandwidth, so seed that one default here. This loop covers
         * radios 1.. on multi-radio boards. */
        storageBegin();
        storageDefault(sk(kb, sizeof kb, 0, "bandwidth"), 125000);         /* 125 kHz */
        /* adaptive_txpwr has no pane row, so seed every radio's copy — a key
         * that only exists once someone guesses its name is not discoverable. */
        for (int i = 0; i < kNumRadios; i++)
            storageDefault(sk(kb, sizeof kb, i, "adaptive_txpwr"), 0);
        /* Frequency agility: the regime number, 0 = none. Radio 0's copy comes
         * from the pane row; radios 1.. are seeded in the loop below. */
        for (int i = 1; i < kNumRadios; i++) {
            storageDefault(sk(kb, sizeof kb, i, "afa"), 0);
            storageDefault(sk(kb, sizeof kb, i, "announce_interval"), ANN_INTERVAL_DEF);
        }
        /* RNode endpoint. One endpoint for the device, so the group is global
         * rather than per radio; `.enable` is seeded by the pane row in
         * straddle.yaml. Both doors default shut — enabling the endpoint must
         * not put a listener on the network nobody asked for: -1 serial claims
         * no port, 0 tcp opens no socket. 7633 is the only TCP port a stock
         * client can dial, so it is the only useful value for that key. */
        storageDefault("s.lora.rnode.radio",  0);
        storageDefault("s.lora.rnode.serial", -1);
        storageDefault("s.lora.rnode.tcp",    0);
        for (int i = 1; i < kNumRadios; i++) {
            storageDefault(sk(kb, sizeof kb, i, "enable"), 0);
            storageDefault(sk(kb, sizeof kb, i, "mode"), "access_point");
            storageDefault(sk(kb, sizeof kb, i, "bandwidth"), 125000);     /* 125 kHz */
            storageDefault(sk(kb, sizeof kb, i, "spreading_factor"), 7);   /* SF7 */
            storageDefault(sk(kb, sizeof kb, i, "coding_rate"), 5);        /* 4/5 */
            storageDefault(sk(kb, sizeof kb, i, "preamble"), 12);
            storageDefault(sk(kb, sizeof kb, i, "sync_word"), "0x42");
        }
        storageSet("s.lora.version", LORA_VERSION);
        storageEnd();
    }

    /* Seed the ephemeral MHz/kHz display keys up front, so the settings pane
     * shows current values before the radio task's rns.ready barrier lifts. */
    for (int i = 0; i < kNumRadios; i++) loraPublishDisplay(i);

    cliRegisterCmd("lora", cliLora);

    /* Register with the RNS orchestrator instead of self-spawning: rnsStart()
     * calls loraStart() (which spawns loraTaskMain) once rnsd is up and past its
     * boot window, and rnsStop() calls loraStop(). The larger 10 KB PSRAM stack
     * is for the LoRa frame buffers + RadioLib state machine + the neighbour
     * table's inline announce verification; core placement puts the driver
     * opposite rnsd on a no-LCD build so their RX/TX bursts overlap and both
     * cores idle together for light sleep. */
    rnsServiceRegister(TAG, loraStart, loraStop, RNS_PHASE_IFACE);
}

#else  /* ── no radios configured (CONFIG_LORA_COUNT = 0) ── */

void LoraService::onInit() {
    /* iface-lora staged but inert: no LoRa pins configured for this board.
     * RadioLib links out; set CONFIG_LORA_COUNT and the pins to enable. */
}

#endif
