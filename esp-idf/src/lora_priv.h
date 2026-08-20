/**
 * lora_priv.h — iface-lora's private shared state (Phase 0 of
 * plans/structuring-lora-code.md).
 *
 * The module files hold code moved verbatim out of lora.cpp; this header holds
 * the types, constants and globals they still share, plus the declaration of
 * every function that crosses a module boundary. It shrinks as later phases
 * give each store its own owner — nothing should be added here that a module
 * interface could carry instead.
 */
#pragma once

#include "sdkconfig.h"
#include "rnsd.h"
#include "lora.h"
#include "rnode_door.h"   /* the endpoint's public door: ITS port + BLE payload */
#include "esp_idf_hal.h"
#include "spangap.h"
#include "mem.h"       /* gp_alloc (PSRAM) for the LoRaMon ring/history buffers */
#include "rolling.h"   /* one-hour running totals in ten-minute buckets */
#if !defined(CONFIG_LORA_NO_SUPE)
#include "supe.h"      /* SUPE's pure core: regimes, ladder, codec, deadlines */
#endif
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
#include <ctime>

#if defined(CONFIG_LORA0_CS_PIN)

#define LORA_VERSION         6
#define RNS_MTU              500
#define RNODE_MAX_PAYLOAD    254
#define RNODE_FLAG_SPLIT     0x01
#define SPLIT_RX_TIMEOUT_MS  5000


/* Per-frame protocol class, published as the record's last field and coloured
 * by the LoRaMon apps. Reticulum traffic is everything rnsd sends and receives;
 * "ours" is this straddle's own air protocol: SUPE's frames and the 0x04
 * power request. */
#define LORA_PKT_RNS    0
#define LORA_PKT_OURS   1
#define LORA_PKT_RNODE  2
#define LORA_PKT_BAD    3   /* rx frame that failed CRC: it flew and held the
                             * medium, but decoded to nothing. Recorded so a
                             * train missing a packet shows WHERE it died —
                             * an unrecorded loss reads as a clean train. */

/* Channel index carried by every frame record and every RSSI sample. 0 is the
 * reticulum hailing channel — the one a node camps on, at whatever frequency
 * and bandwidth the radio is configured for, and the only channel that exists
 * until frequency agility is switched on. Higher indices are the agile channels
 * of the regime in force. */
#define LORA_CH_HAIL    0
#define LORA_CH_MAX     10          /* hailing channel + the largest regime's agile set */
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


#define LORA_CFG_COALESCE_MS 300
#define LORA_STATS_MIN_MS 1000

#define LORA_FREQ_MIN_HZ  100000000    /* 100 MHz */
#define LORA_FREQ_MAX_HZ  2000000000   /* 2 GHz — storage ints are int32, keep the cast safe */
#define LORA_BW_MIN_HZ    5000          /* 5 kHz */
#define LORA_BW_MAX_HZ    1700000       /* 1.7 MHz */

static const char* TAG __attribute__((unused)) = "lora";

#if !defined(CONFIG_LORA_NO_SUPE)
/* Our mesh radio layer, shown as a node tag in `lora n`: Spectrum Utilization
 * and Performance Enhancements, designed in plans/iface-lora/SUPE.md. The same
 * name both LoRaMon viewers give the protocol in their legends. */
#define RF_PROTO_NAME         "SUPE"
#endif

/* Radio count from the per-slot Kconfig CS pins — the kSlots table (lora.cpp)
 * static_asserts against it. */
#if defined(CONFIG_LORA3_CS_PIN)
#define LORA_NUM_RADIOS 4
#elif defined(CONFIG_LORA2_CS_PIN)
#define LORA_NUM_RADIOS 3
#elif defined(CONFIG_LORA1_CS_PIN)
#define LORA_NUM_RADIOS 2
#else
#define LORA_NUM_RADIOS 1
#endif
static constexpr int kNumRadios = LORA_NUM_RADIOS;

/* Owned elsewhere; LoraRadio holds pointers. */
struct NeiState;
struct AnnBuf;
#if !defined(CONFIG_LORA_NO_SUPE)
struct SupeState;
struct ChanLedger;
#endif
struct Neighbor;

/* ─────────────── storage key helpers (per radio) ─────────────── */

static inline const char* sk(char* b, size_t n, int i, const char* leaf) {
    snprintf(b, n, "s.lora.%d.%s", i, leaf); return b;
}
static inline const char* rk(char* b, size_t n, int i, const char* leaf) {
    snprintf(b, n, "lora.%d.%s", i, leaf); return b;
}

#include "lora_queue.h"
#include "lora_radio.h"
#include "lora_csma.h"
#include "lora_mon.h"

/* ─────────────── per-radio state ─────────────── */

struct LoraRadio {
    int             idx;
    const LoraSlot* slot;

    EspIdfHal*      hal;
    Module*         mod;
    PhysicalLayer*  radio;        /* RadioLib base; concrete class per slot chip */
    int             found;        /* -1 unprobed, 0 absent, 1 detected */

    /* This chip's own IRQ-register bits for the flags the loop tests, translated
     * once from RadioLib's radio-agnostic numbering (radioIrqCache). getIrqFlags
     * returns the raw register, so the two only agree by coincidence on some
     * families. 0 = the part has no such IRQ. */
    uint32_t        irqTxDone, irqRxDone, irqPreamble, irqHdrValid, irqHdrErr;

    /* Modem-reported reception in progress: when the preamble/header evidence
     * first appeared, and how long each stage may stand before it is stale (a
     * latched bit that no packet followed). See radioRxInProgress. */
    TickType_t      rxActiveStart;    /* 0 = nothing seen */
    bool            rxHeaderSeen;
    TickType_t      rxPreambleTicks;  /* preamble → header-valid */
    TickType_t      rxPacketTicks;    /* header-valid → rx-done */

    /* Analog front-end recalibration (s.lora.<i>.agc_reset, seconds; 0 = off).
     * The only standing wake this task holds by default — see radioAgcReset. */
    uint32_t        agcResetMs;
    TickType_t      agcNext;

    int             rnsdHandle;
    bool            running;
    bool            enabled;
    uint8_t         curMode;
    uint32_t        curBitrate;
    /* Live modem params, kept for per-packet airtime accounting (the airtime
     * formula needs SF/BW/CR/preamble, and only radioStart reads them). */
    int             cfgSf, cfgBwHz, cfgCr, cfgPreamble;
    uint8_t         airSf;           /* spreading factor CURRENTLY installed */
    /* Framing the radio is CURRENTLY set to — tracks a SUPE detour's step, so airtime records are computed with the
     * parameters the frame really used rather than the configured ones. */
    int             airPreamble;
    int             airBwHz;         /* bandwidth CURRENTLY installed */
    bool            airImplicit;
    char            curIfacNetname[32];   /* IFAC network_name (s.) */
    char            curIfacNetkey[64];    /* IFAC passphrase (secrets.) */
    uint8_t         curIfacSize;          /* IFAC access-code length */
    uint8_t         curAnnounceCap;       /* % bandwidth cap for announces (s.) */
    uint8_t         curRetainAnnounces;   /* keep announces heard here, not just forward */
    uint8_t         curPolicyManual;      /* 0 = auto: policy inferred, curRouteFor unread */
    uint8_t         curRouteFor;          /* manual only: do we do transit work for this radio */

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
    float           chFloor[LORA_CH_MAX];  /* each channel's floor as this radio last
                                            * left it, indexed by channel; the hailing
                                            * channel is 0 (csmaFloorSwitch) */
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
    bool            oscHeld;         /* the oscillator is being kept alive across the
                                      * gaps inside a chain of frames (radioHoldOsc) */
    TickType_t      txDeadline;      /* watchdog: recover if TxDone never arrives */
    TickType_t      txWatchTicks;    /* per-frame TxDone watchdog budget (airtime + margin) */
    uint8_t         txSeq;           /* 4-bit seq nibble shared by a split pair */
    uint8_t         txFrame[2][1 + RNODE_MAX_PAYLOAD];  /* prebuilt on-air frame(s) */
    size_t          txFrameLen[2];
    /* The NEXT train packet, fully built — frames, split, observer tap,
     * fan-out — while the current one is still on air, so firing it is a
     * buffer copy and a startTransmit. Owned by the train path; empty
     * otherwise. */
    uint8_t         txStage[2][1 + RNODE_MAX_PAYLOAD];
    size_t          txStageLen[2];
    uint8_t         txStageCount;    /* 0 = nothing staged */
    uint8_t         txStageType[2];
    size_t          txStageBytes;    /* RNS payload bytes of the staged packet */
    int8_t          txStagePwr;
    bool            txStageFromRnode;
    uint8_t         txFrameCount;    /* 1, or 2 for a split packet */
    uint8_t         txFrameSent;     /* completed frames so far */
    size_t          txPayloadBytes;  /* RNS payload bytes, credited on completion */
    /* Wall time the head frame spent queued before its first bit went on air.
     * Started the first pass we have something to send and can't, so it covers
     * every reason a frame waits — the radio held by a SUPE transaction or a
     * frame, a split reassembly still landing, our own transmit finishing, and
     * then DIFS/backoff against a channel that reads busy — rather than
     * contention alone. Stamped into the frame's LoRaMon record. */
    uint32_t        txWaitStartMs;
    bool            txWaitPend;
    uint16_t        txWaitMs;        /* of that wait, what the channel cost … */
    uint16_t        txOwnMs;         /* … and what we cost ourselves */

    /* Stats — published to ephemeral storage once per task tick. */
    uint64_t        txBytes, rxBytes, txFrames, rxFrames, crcErr, splitTimeouts, txDropped;
    float           rssiLast, snrLast;

    /* LoRaMon — each on-air frame becomes a storage node lora.<n>.packets.<ms>;
     * this FIFO of start-ms drives expiry (delete nodes > 1 h old). */
    int8_t          cfgTxp;          /* configured TX power dBm (ANTENNA dBm — the
                                      * FEM gain conversion happens only at the
                                      * chip, in femChipDbm) */
    uint8_t         cfgSync;         /* configured sync word (restored after a sweep) */
    int8_t          txPwrNow;        /* power of the frame on-air, stamped into tx records */
    uint8_t         femType;         /* LoraFemType — external front-end module, set by femInit */
    int8_t          maxTxDbm;        /* antenna-dBm ceiling: 22 bare chip, the board's
                                      * declared figure through a FEM (27 Heltec V4,
                                      * 35 Station G2) */
    uint8_t         txType[2];       /* per frame: LORA_PKT_*. A 0x04 power request
                                      * and the RNS packet it prefixes share one
                                      * channel access but not one protocol. */
    bool            txAborted;       /* the frame on air was abandoned by the
                                      * watchdog — it never left, so nothing
                                      * waiting on it may proceed */
    bool            txFromRnode;     /* the packet on air came from the RNode client;
                                      * its queue is released when the last frame
                                      * of it leaves (see txRearmRx). */
    uint32_t        txFrameStartMs;  /* start (millis) of the on-air TX frame */

    /* Channel-RSSI sampling (radio task). One getRSSI(false) per beat while the
     * radio is idle; carrier sense outranks it, so a frame contending for the
     * medium leaves a gap in the series rather than a stale reading. */
    /* Tick at the end of the last reception. Carrier sense cannot see every
     * frame this radio decodes, so the receiver reports the medium it held —
     * see csmaMediumHeld. 0 = nothing outstanding. */
    TickType_t      rxHeldTick;

    uint32_t        cfgFreqHz;       /* configured carrier — the hailing channel */
#if !defined(CONFIG_LORA_NO_SUPE)
    uint8_t         afa;             /* s.lora.<i>.afa: the regime number, 0 = no agility */
    uint16_t        annIntervalMin;  /* s.lora.<i>.SUPE.announce_interval, minutes;
                                      * 0 = manual only. Paces the whole announce
                                      * beat: SUPE's own ANNOUNCE2 */
#endif
    /* The channel the radio is tuned to right now, so a record and its airtime
     * credit both land where the frame actually flew. Held here rather than
     * passed to loraMonPush because every caller would otherwise have to know,
     * and one that got it wrong would put detour airtime on the hailing
     * channel's duty figure — which SUPE.md §8 makes an implementation
     * requirement, and getting it wrong is silent. */
    uint8_t         chNow;

    /* Telemetry state — the event ring, the LoRaMon FIFO and the RSSI
     * beat. Owned by lora_mon; nothing else reads or writes inside it. */
    LoraMonState    mon;

#if !defined(CONFIG_LORA_NO_SUPE)
    /* Per-channel airtime ledger (ring, verdict, reuse gap). Owned by
     * lora_airtime; allocated with SUPE's state and null until then. */
    ChanLedger*     chans;
#endif


    /* Passive neighbour table (gp_alloc'd at radioStart, kept across cycles). */
    NeiState*       nei;

    /* The one seam between the bridge and the radio: everything inbound is
     * enqueued here; the engine (or the plain drain) dequeues. lora_queue owns
     * the entries; the bridge fills it, single-task by the drain's gates. */
    LoraQueue       q;

    /* Announce buffer (gp_alloc'd alongside nei, kept likewise). Every announce
     * this node originates is copied here as it goes out, so `lora a` can
     * repeat them; nothing is held back on its account. */
    AnnBuf*         ann;
    bool            annReplay;       /* a replay run is in progress */
    uint8_t         annIdx;          /* next buffer slot it will emit */

#if !defined(CONFIG_LORA_NO_SUPE)
    /* SUPE (overview at SUPE_TAGS_MAX). State is gp_alloc'd beside the
     * neighbour table and kept across config cycles, so toggling a key does not
     * throw away a tag set that took real traffic to learn. */
    SupeState*      supe;
    bool            supeOn;          /* s.lora.<i>.SUPE.enable, AND no access code */
    bool            supeAdaptive;    /* s.lora.<i>.SUPE.adaptive_txpower */
    bool            supeNameSender;  /* s.lora.<i>.SUPE.sender_ident */
#endif

    /* Adaptive TX power (overview at AP_EST_MARGIN_DB). */
    bool            adaptive;        /* the reciprocity determination and the
                                      * 0x04 request read the same key */
    /* A 0x04 power request just received, awaiting the frame it prefixes. The
     * frame carries no binding field — it binds by adjacency alone — so this is
     * consumed or discarded by the very next rx frame, never held. */
    bool            apRxSuggestPend;
    int8_t          apRxSuggest;


    /* Manual CLI transmit (`lora <n> tx | tx_psa | tx_prot`). The CLI fills the
     * request under mtxReq and notifies; manualTxPoll (task loop) services it and
     * bumps mtxResGen with the outcome in mtxResOk/mtxResMsg. mtxPhase drives the
     * PSA carrier-sense state; while it is non-OFF the normal outbound drain and
     * an announce replay stand off the radio. */
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

/* ─────────────── service-owned globals (lora.cpp) ─────────────── */

extern LoraRadio     s_radios[LORA_NUM_RADIOS];
extern TaskHandle_t  s_task;
extern volatile bool s_stop;

/* Config apply coalescing — ask for an apply in at most `delayMs`. */
void cfgArm(uint32_t delayMs);

/* Wake the interface task: its wake deadlines just changed. */
void loraNudge(void);

/* The module interfaces, in dependency order. */
#if !defined(CONFIG_LORA_NO_SUPE)
#include "lora_chanplan.h"
#endif
#include "lora_peers.h"
#include "lora_observe.h"
#include "lora_power.h"
#if !defined(CONFIG_LORA_NO_SUPE)
#include "lora_airtime.h"
#endif
#include "lora_bridge.h"
#include "lora_rnode.h"
#if !defined(CONFIG_LORA_NO_SUPE)
#include "lora_supe.h"
#endif
#include "lora_cli.h"

#endif  /* CONFIG_LORA0_CS_PIN */
