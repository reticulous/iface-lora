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
 * nothing on core 0 is starved even at SF12. serviceRadio() reads the chip's IRQ
 * flags to decide what completed rather than guessing TX-vs-RX from state. The
 * radio is half-duplex, so we never start a transmit while a split RX is being
 * reassembled (splitPending) or another transmit is on-air (txActive).
 */
#include "lora.h"
#include "esp_idf_hal.h"
#include "spangap.h"
#include "ports.h"

#include <RadioLib.h>

#include "driver/gpio.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>

#define LORA_VERSION         2
#define RNS_MTU              500
#define RNODE_MAX_PAYLOAD    254
#define RNODE_FLAG_SPLIT     0x01
#define SPLIT_RX_TIMEOUT_MS  5000

/* ── RNode-style CSMA / listen-before-talk ──
 * Non-blocking DIFS + exponential contention-window backoff, run from the task
 * loop. Carrier sense is instantaneous channel RSSI vs a tracked noise floor,
 * matching RNode's channel-free test. No airtime accounting yet. */
#define CSMA_CW_MIN          2       /* initial CW exponent → up to 2^2 = 4 slots */
#define CSMA_CW_MAX          6       /* CW ceiling → up to 2^6 = 64 slots */
#define CSMA_RSSI_MARGIN_DB  6.0f    /* dB above noise floor that reads as busy */
#define CSMA_NOISE_FLOOR_DBM (-105.0f)  /* initial noise-floor estimate */
#define CSMA_SLOT_MS_MIN     2       /* slot-time clamp (derived from symbol time) */
#define CSMA_SLOT_MS_MAX     25

enum CsmaPhase : uint8_t { CSMA_IDLE, CSMA_DIFS, CSMA_BACKOFF };

#if defined(CONFIG_LORA0_CS_PIN)   /* ── at least one radio configured ── */

static const char* TAG = "lora";

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
    TickType_t      slotTicks;       /* CSMA slot time */
    TickType_t      difsTicks;       /* inter-frame listen before backoff */
    CsmaPhase       csmaPhase;
    int             csmaCw;          /* current contention-window exponent */
    int             csmaBackoff;     /* backoff slots remaining */
    TickType_t      csmaDifsStart;   /* tick the current unbroken free window began */
    TickType_t      csmaSlotDeadline;/* next backoff slot boundary */
    TickType_t      csmaStart;       /* tick this frame's channel-access attempt began */
    float           noiseFloor;      /* tracked channel noise floor, dBm */
    uint32_t        lbtTimeoutMs;    /* drop a frame LBT can't clear within this (s.lora.<i>.lbt_timeout) */
    TickType_t      lbtTimeoutTicks; /* lbtTimeoutMs in ticks; 0 = never drop */

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

    /* Stats — published to ephemeral storage once per task tick. */
    uint64_t        txBytes, rxBytes, txFrames, rxFrames, crcErr, splitTimeouts, txDropped;
    float           rssiLast, snrLast;
};

static LoraRadio     s_radios[kNumRadios];
static TaskHandle_t  s_task = nullptr;
static volatile bool s_configDirty = true;
static volatile bool s_displayDirty = false;   /* an MHz/kHz display key was edited */

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
 * applies DIO2-as-RF-switch when the slot asks for it. */
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
    /* SX126x only: DIO2 drives the antenna RF switch. */
    if (st == RADIOLIB_ERR_NONE && r->slot->dio2_rf_switch)
        st = static_cast<SX126x*>(p)->setDio2AsRfSwitch(true);
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
 * decode when off, so this is free on the hot path in normal operation. */
static void loraTracePacket(LoraRadio* r, const char* dir,
                            const uint8_t* p, size_t len, bool haveQual) {
    if (!logIsDebug(TAG)) return;

    char qual[24] = "";
    if (haveQual)
        snprintf(qual, sizeof qual, " %ddBm snr%.1f",
                 (int)r->rssiLast, (double)r->snrLast);

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
        dbg("lora/%d %s%s <unparsed %uB> %s", r->idx, dir, qual, (unsigned)len, hx);
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

    dbg("lora/%d %s%s %s %s %s %s%s%s hops=%u%s",
        r->idx, dir, qual,
        loraPktType(ptype), transport ? "to" : "bcast",
        loraDestType(dtype), destHex, via, ctxbuf,
        (unsigned)hops, anomaly ? " ?" : "");

    loraHexdump(r->idx, dir, p, len);
}

/* ─────────────── ISR ─────────────── */

static IRAM_ATTR void loraRadioIsr(void) {
    /* Shared across radios — any DIO1 wakes the task, which polls each
     * radio's IRQ flags to find the one(s) that completed. */
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &hp);
    portYIELD_FROM_ISR(hp);
}

/* ─────────────── helpers ─────────────── */

/* Time-on-air (seconds) for a `payload`-byte LoRa frame, per Semtech
 * AN1200.13. Symbol time Tsym = 2^SF / BW; the preamble runs (n+4.25)
 * symbols and the payload rounds up into whole symbols, with low-data-rate
 * optimisation (DE) engaged once a symbol exceeds 16 ms. Assumes explicit
 * header and CRC on — the config radioStart brings the radio up with. */
static double loraAirtimeSeconds(int sf, int bw_hz, int cr_denom,
                                 int preamble, int payload) {
    if (sf <= 0 || bw_hz <= 0 || cr_denom < 5 || cr_denom > 8) return 0.0;
    double tSym = (double)((uint32_t)1 << sf) / (double)bw_hz;
    int    de   = (tSym > 0.016) ? 1 : 0;          /* low-data-rate optimize */
    int    cr   = cr_denom - 4;                    /* coded bits 1..4 */
    double num  = 8.0 * payload - 4.0 * sf + 28.0 + 16.0 /*CRC*/ - 0.0 /*explicit header*/;
    double den  = 4.0 * (sf - 2 * de);
    double payloadSym = 8.0 + fmax(ceil(num / den) * (cr + 4), 0.0);
    return (preamble + 4.25) * tSym + payloadSym * tSym;
}

/* Effective bps to register with rnsd. RNS derives its first-hop link
 * timeout as MTU*8/bitrate + 6 s, so registering bitrate = 4000/ceil(toa)
 * makes that term equal the (whole-second-rounded) airtime of one MTU — the
 * link establishment budget then tracks how long a 500-byte frame really
 * takes on this SF/BW/CR/preamble. */
static uint32_t computeBitrate(int sf, int bw_hz, int cr_denom, int preamble) {
    double toa = loraAirtimeSeconds(sf, bw_hz, cr_denom, preamble, RNS_MTU);
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

static void publishStats(LoraRadio* r) {
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
    storageEnd();
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
    r->csmaPhase = CSMA_IDLE;
    r->csmaCw = CSMA_CW_MIN;
    r->noiseFloor = CSMA_NOISE_FLOOR_DBM;

    /* Non-blocking TX watchdog: 2.5× the airtime of a full frame (+100 ms floor)
     * — long enough never to fire in normal operation, short enough that a wedged
     * transmit can't pin the outbound queue. */
    double maxToa = loraAirtimeSeconds(sf, bw_hz, cr, preamble, RNODE_MAX_PAYLOAD);
    r->txWatchTicks = pdMS_TO_TICKS((uint32_t)(maxToa * 1000.0 * 2.5) + 100);
    r->txActive = false;

    /* Mode for RNS iface registration. LoRa defaults to access_point (edge
     * segment); see straddle.yaml for why full/gateway are opt-in-by-hand. */
    char mode[24] = "access_point";
    storageGetStr(sk(kb, sizeof kb, r->idx, "mode"), mode, sizeof(mode), "access_point");
    r->curMode    = modeFromString(mode);
    r->curBitrate = computeBitrate(sf, bw_hz, cr, preamble);

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

static void deliverInbound(LoraRadio* r, const uint8_t* data, size_t len) {
    loraTracePacket(r, "rx", data, len, true);
    if (r->rnsdHandle < 0) return;
    /* Prefix this packet's signal telemetry (rnsd strips it, sets it on the
     * received packet): int16 rssi(dBm) | int16 snr(dB*10), big-endian. Captured
     * in the same synchronous RX call as `data`, so it correlates exactly. */
    auto rnd = [](float x) { return (int16_t)(x < 0 ? x - 0.5f : x + 0.5f); };
    int16_t rssi = rnd(r->rssiLast);
    int16_t snr  = rnd(r->snrLast * 10.0f);
    uint8_t f[4 + RNS_MTU + 16];
    if (len > sizeof(f) - 4) len = sizeof(f) - 4;   /* defensive clamp */
    f[0] = (uint8_t)(rssi >> 8); f[1] = (uint8_t)rssi;
    f[2] = (uint8_t)(snr  >> 8); f[3] = (uint8_t)snr;
    memcpy(f + 4, data, len);
    size_t s = itsSend(r->rnsdHandle, f, 4 + len, pdMS_TO_TICKS(100));
    if (s == 0) warn("lora/%d rnsd ITS send dropped (%u B)", r->idx, (unsigned)len);
}

/* Re-arm continuous RX and re-enable the level-triggered DIO1 (the trampoline
 * disables it on each fire; a completed readData()/finishTransmit() has cleared
 * the chip IRQ so the line has dropped low and the next edge fires again).
 * Shared by the RX drain and the post-TX return to listening. */
static void rearmRx(LoraRadio* r) {
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
    uint8_t frame[1 + RNODE_MAX_PAYLOAD];
    int16_t st = r->radio->readData(frame, pktLen);
    if (st != RADIOLIB_ERR_NONE) {
        if (st == RADIOLIB_ERR_CRC_MISMATCH) {
            r->crcErr++;
            if (logIsDebug("lora"))       /* CRC = the RX error-check info */
                dbg("lora/%d rx CRC-FAIL %uB rssi=%.0f snr=%.1f",
                    r->idx, (unsigned)pktLen,
                    (double)r->radio->getRSSI(), (double)r->radio->getSNR());
        }
        rearmRx(r);
        return;
    }
    r->rxFrames++;
    r->rssiLast = r->radio->getRSSI();
    r->snrLast  = r->radio->getSNR();

    uint8_t  header     = frame[0];
    uint8_t  seq        = header & 0xF0;
    bool     isSplit    = (header & RNODE_FLAG_SPLIT) != 0;
    size_t   payloadLen = pktLen - 1;

    if (!isSplit) {
        deliverInbound(r, frame + 1, payloadLen);
        r->rxBytes += payloadLen;
    } else if (!r->splitPending) {
        std::memcpy(r->splitBuf, frame + 1, payloadLen);
        r->splitLen      = payloadLen;
        r->splitSeq      = seq;
        r->splitPending  = true;
        r->splitDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(SPLIT_RX_TIMEOUT_MS);
    } else if (r->splitSeq == seq) {
        if (r->splitLen + payloadLen <= sizeof(r->splitBuf)) {
            std::memcpy(r->splitBuf + r->splitLen, frame + 1, payloadLen);
            r->splitLen += payloadLen;
            deliverInbound(r, r->splitBuf, r->splitLen);
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

/* Carrier sense: sample the channel and decide busy/free, tracking the noise
 * floor as the low envelope of RSSI (snap down fast, creep up slowly) so an
 * active channel can't inflate the reference it's compared against. Also busy
 * while a multi-frame reception is being reassembled (half-duplex). */
static bool channelBusy(LoraRadio* r) {
    if (r->splitPending) return true;
    float rssi = channelRssi(r);
    if (rssi < r->noiseFloor) r->noiseFloor = rssi;
    else                      r->noiseFloor += 0.02f * (rssi - r->noiseFloor);
    return rssi > r->noiseFloor + CSMA_RSSI_MARGIN_DB;
}

/* Advance the channel-access state machine. Returns true only on the tick the
 * medium is granted (DIFS observed idle, then a random backoff drained without
 * the channel going busy). Otherwise updates state and returns false; the
 * caller leaves the frame queued and nextDeadline() re-wakes at the next slot.
 * cw grows on every busy encounter (exponential backoff) and resets after a
 * grant. Mirrors RNode's CSMA/CA, minus airtime-based persistence. */
static bool csmaClear(LoraRadio* r) {
    if (!r->lbt) return true;                       /* LBT off → blind transmit */

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

/* ─────────────── outbound (rnsd → radio) ─────────────── */

/* Return the radio to continuous RX after a transmit finishes or aborts. */
static void txRearmRx(LoraRadio* r) {
    r->txActive = false;
    rearmRx(r);
}

/* Fire frame `idx` of the current outbound packet. startTransmit() writes the
 * FIFO and issues SetTx, then returns — the chip modulates on its own and raises
 * TxDone on DIO1 when done (serviceRadio handles it). Non-blocking: the task is
 * free for the whole airtime, so nothing on core 0 is starved at high SF. */
static void startTxFrame(LoraRadio* r, int idx) {
    int16_t st = r->radio->startTransmit(r->txFrame[idx], r->txFrameLen[idx]);
    if (st != RADIOLIB_ERR_NONE) {
        warn("lora/%d startTransmit %u B failed: %s (%d)",
             r->idx, (unsigned)r->txFrameLen[idx], rlErrName(st), (int)st);
        txRearmRx(r);
        return;
    }
    r->txActive   = true;
    r->txDeadline = xTaskGetTickCount() + r->txWatchTicks;
    gpio_intr_enable((gpio_num_t)r->slot->dio1);   /* arm DIO1 for this frame's TxDone */
}

/* Begin transmitting one RNS packet. >RNODE_MAX_PAYLOAD splits into two frames
 * sharing a seq nibble; the second is fired from serviceRadio once the first
 * completes. Returns immediately — the airtime runs on the chip, not the task. */
static void beginTx(LoraRadio* r, const uint8_t* data, size_t len) {
    if (!r->running || len == 0 || len > RNS_MTU) return;

    loraTracePacket(r, "tx", data, len, false);

    r->txSeq          = (uint8_t)((esp_random() & 0x0F) << 4);   /* 4-bit seq, upper nibble */
    r->txPayloadBytes = len;
    r->txFrameSent    = 0;

    if (len <= RNODE_MAX_PAYLOAD) {
        r->txFrame[0][0] = r->txSeq;
        std::memcpy(r->txFrame[0] + 1, data, len);
        r->txFrameLen[0] = 1 + len;
        r->txFrameCount  = 1;
    } else {
        size_t first  = RNODE_MAX_PAYLOAD;
        size_t second = len - first;
        r->txFrame[0][0] = r->txSeq | RNODE_FLAG_SPLIT;
        std::memcpy(r->txFrame[0] + 1, data, first);
        r->txFrameLen[0] = 1 + first;
        r->txFrame[1][0] = r->txSeq | RNODE_FLAG_SPLIT;
        std::memcpy(r->txFrame[1] + 1, data + first, second);
        r->txFrameLen[1] = 1 + second;
        r->txFrameCount  = 2;
    }
    startTxFrame(r, 0);
}

/* Drain one pending outbound packet for this radio if it's free.
 * Half-duplex: while a split RX is being reassembled OR a transmit is already
 * on-air (txActive) we leave bytes sitting in the ITS stream buffer (our
 * outbound TX queue) and revisit once the radio is idle. */
static void drainOneOutbound(LoraRadio* r) {
    if (!r->running || r->splitPending || r->txActive || r->rnsdHandle < 0) return;
    if (itsBytesAvailable(r->rnsdHandle) == 0) {
        r->csmaPhase = CSMA_IDLE;   /* nothing queued → reset channel-access state */
        return;
    }
    if (!csmaClear(r)) {            /* listen-before-talk not yet satisfied */
        /* Channel never cleared within lbt_timeout → drop the head frame instead
         * of blocking the outbound queue behind a wedged-busy channel. */
        if (r->lbtTimeoutTicks &&
            (TickType_t)(xTaskGetTickCount() - r->csmaStart) >= r->lbtTimeoutTicks) {
            static uint8_t drop[RNS_MTU + 16];
            size_t n = itsRecv(r->rnsdHandle, drop, sizeof(drop), 0);
            r->txDropped++;
            err("lora/%d LBT: channel busy > %u ms, dropped %u B frame",
                r->idx, (unsigned)r->lbtTimeoutMs, (unsigned)n);
            r->csmaPhase = CSMA_IDLE;   /* re-arm access state for the next frame */
        }
        return;
    }
    static uint8_t pkt[RNS_MTU + 16];
    size_t n = itsRecv(r->rnsdHandle, pkt, sizeof(pkt), 0);
    if (n > 0) beginTx(r, pkt, n);
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
    s_configDirty = true;
    if (s_task) xTaskNotifyGive(s_task);
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
    cliPrintf("        rx %u/%uB  tx %u/%uB  rssi %d dBm  snr %d dB  crc_err %u  split_to %u\n",
              (unsigned)r->rxFrames, (unsigned)r->rxBytes,
              (unsigned)r->txFrames, (unsigned)r->txBytes,
              (int)r->rssiLast, (int)r->snrLast,
              (unsigned)r->crcErr, (unsigned)r->splitTimeouts);
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
        cliPrintf("%-*s freq MHz / bw kHz / sf / cr /\n",   CLI_HELP_COL, "lora <n> <param> <val>");
        cliPrintf("%-*s   txp dBm / preamble / sync / mode\n", CLI_HELP_COL, "");
        return;
    }

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

    if (nt < 3) { cliPrintf("usage: lora %ld <freq|bw|sf|cr|txp|preamble|sync|mode> <value>\n", idx); return; }
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
    } else {
        cliPrintf("unknown: lora %ld %s (try freq|bw|sf|cr|txp|preamble|sync|mode)\n", idx, cmd);
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
static TickType_t s_statsLastPub = 0;      /* tick of the last publish */
static bool       s_statsPend    = false;  /* counter moved; publish owed at the 1 Hz boundary */
static uint64_t   s_statsSig     = 0;      /* last-seen sum of all counters */

static TickType_t nextDeadline(void) {
    TickType_t now = xTaskGetTickCount();
    /* Idle default: block until an ISR/ITS wake. Shrunk below only for real
     * pending work — a deferred stats flush, a registration retry, or outbound. */
    TickType_t soonest = portMAX_DELAY;
    /* A counter change is waiting to be flushed at the next 1 Hz boundary. */
    if (s_statsPend) {
        TickType_t due = s_statsLastPub + pdMS_TO_TICKS(LORA_STATS_MIN_MS);
        TickType_t d = (int32_t)(due - now) > 0 ? (TickType_t)(due - now) : 0;
        if (d < soonest) soonest = d;
    }
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
        if (r->running && !r->splitPending && !r->txActive && r->rnsdHandle >= 0
            && itsBytesAvailable(r->rnsdHandle) > 0) {
            if (!r->lbt) return 0;
            TickType_t d = r->slotTicks;
            if (r->csmaPhase == CSMA_BACKOFF) {
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
    }
    return soonest;
}

static void loraTaskMain(void*) {
    info("[%s] task up (%d radio%s)", TAG, kNumRadios, kNumRadios == 1 ? "" : "s");

    /* Boot barrier: stay quiet until the RNS universe has settled — clock valid,
     * network up (if configured), and the minimum settle floor elapsed. rnsd
     * publishes rns.ready once all that holds; until then we don't register or
     * transmit (a brownout/boot-loop node must never reach RF TX). Bounded
     * fallback so a wedged rnsd can't pin us. No rnsd, no
     * point — so bail (don't start) if rns.ready never comes. */
    if (!waitForFlag("rns.ready", 120)) {
        err("[%s] rns.ready never set — not starting", TAG);
        killSelf();
    }

    itsClientInit(kNumRadios);
    storageSubscribeChanges("s.lora", onCfgChange);
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
     * it (e.g. hw-tdeck's tdeckPowerInit), not this interface. */
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

    /* Wait for a valid clock before bringing radios on-air, so our first
     * announces aren't 1970-stamped. LoRa has no IP path to SNTP — time comes
     * from GPS/RTC if the board has one. A LoRa-only node with no time source
     * just eats the bounded timeout each boot; set s.sys.time_wait_s=0 there. */
    waitForTime(0);

    /* Seed the stat keys once so consumers see a radio before any traffic; from
     * here publishing is purely event-driven (see the stats block below). */
    for (int i = 0; i < kNumRadios; i++) publishStats(&s_radios[i]);
    s_statsLastPub = xTaskGetTickCount();

    for (;;) {
        if (s_configDirty) {
            s_configDirty = false;
            for (int i = 0; i < kNumRadios; i++) applyConfig(&s_radios[i]);
            for (int i = 0; i < kNumRadios; i++) loraPublishDisplay(i);   /* Hz → MHz/kHz */
        }
        if (s_displayDirty) {
            s_displayDirty = false;
            for (int i = 0; i < kNumRadios; i++) loraApplyDisplay(i);     /* MHz/kHz → Hz */
        }

        /* Sum every published field across radios; a change means a tx/rx event
         * moved a counter (or a last-packet reading) this pass. */
        uint64_t sig = 0;

        for (int i = 0; i < kNumRadios; i++) {
            LoraRadio* r = &s_radios[i];
            if (r->running) serviceRadio(r);

            if (r->splitPending &&
                (int32_t)(xTaskGetTickCount() - r->splitDeadline) >= 0) {
                r->splitPending = false;
                r->splitTimeouts++;
            }
            if (r->running && r->rnsdHandle < 0 && r->enabled) registerWithRnsd(r);
            drainOneOutbound(r);
            sig += r->txBytes + r->rxBytes + r->txFrames + r->rxFrames +
                   r->crcErr + r->splitTimeouts + r->txDropped +
                   (uint32_t)r->rssiLast + (uint32_t)r->snrLast;
        }

        /* Publish only after an event, at most once a second. A change inside the
         * 1 s window sets s_statsPend; nextDeadline() then wakes us at the boundary
         * to flush the latest values (coalescing any changes in between). */
        if (sig != s_statsSig) { s_statsSig = sig; s_statsPend = true; }
        if (s_statsPend) {
            TickType_t nowp = xTaskGetTickCount();
            if ((int32_t)(nowp - s_statsLastPub) >= (int32_t)pdMS_TO_TICKS(LORA_STATS_MIN_MS)) {
                for (int i = 0; i < kNumRadios; i++) publishStats(&s_radios[i]);
                s_statsLastPub = nowp;
                s_statsPend    = false;
            }
        }

        itsPoll(nextDeadline());
    }
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

    /* Larger stack than other interfaces for the LoRa frame buffers and
     * RadioLib state machine. PSRAM stack. */
    s_task = spawnTask(loraTaskMain, TAG, 8192, nullptr, 2, 0, STACK_PSRAM);
}

#else  /* ── no radios configured (CONFIG_LORA_COUNT = 0) ── */

void LoraService::onInit() {
    /* iface-lora staged but inert: no LoRa pins configured for this board.
     * RadioLib links out; set CONFIG_LORA_COUNT and the pins to enable. */
}

#endif
