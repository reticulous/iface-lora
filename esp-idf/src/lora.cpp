/**
 * lora — LoRa transport task (one or more radios).
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
 * RNode-framed packets, and forwards to rnsd.
 *
 * RNode on-air framing (mandatory for interop, see plan §7.1):
 *   [1B header][≤254B payload]
 *   header upper nibble = random sequence id
 *   header bit 0       = SPLIT (this is part of a 2-frame split packet)
 *
 * TX: synchronous radio.transmit() in the task. The radio is half-duplex so
 * we cannot transmit while a split RX is pending — guarded per radio by
 * splitPending.
 *
 * See docs/component-plan.md §7 / §12.
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

static const char* TAG = "lora";

#define LORA_VERSION         2
#define RNS_MTU              500
#define RNODE_MAX_PAYLOAD    254
#define RNODE_FLAG_SPLIT     0x01
#define SPLIT_RX_TIMEOUT_MS  5000

#if defined(CONFIG_LORA0_CS_PIN)   /* ── at least one radio configured ── */

/* ─────────────── Kconfig → descriptor table ─────────────── */

/* Every RadioLib LoRa chip this transport drives, as (enum suffix, RadioLib
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

    /* Split-RX reassembly — one in-flight split at a time per radio. */
    uint8_t         splitBuf[RNS_MTU + 16];
    size_t          splitLen;
    uint8_t         splitSeq;
    bool            splitPending;
    TickType_t      splitDeadline;

    /* Stats — published to ephemeral storage once per task tick. */
    uint64_t        txBytes, rxBytes, txFrames, rxFrames, crcErr, splitTimeouts;
    float           rssiLast, snrLast;
};

static LoraRadio     s_radios[kNumRadios];
static TaskHandle_t  s_task = nullptr;
static volatile bool s_configDirty = true;

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

/* ─────────────── ISR ─────────────── */

static IRAM_ATTR void loraRadioIsr(void) {
    /* Shared across radios — any DIO1 wakes the task, which polls each
     * radio's IRQ flags to find the one(s) that completed. */
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &hp);
    portYIELD_FROM_ISR(hp);
}

/* ─────────────── helpers ─────────────── */

static uint32_t computeBitrate(int sf, int bw_hz, int cr_denom) {
    /* LoRa raw bit rate (bits/s) = SF * (BW / 2^SF) * (4 / cr_denom). */
    if (sf <= 0 || bw_hz <= 0 || cr_denom < 5 || cr_denom > 8) return 0;
    double rb = (double)sf * (double)bw_hz * 4.0
              / ((double)((uint32_t)1 << sf) * (double)cr_denom);
    return (uint32_t)rb;
}

static uint8_t modeFromString(const char* s) {
    if (!s || !*s)                      return RNS_IFACE_MODE_GATEWAY;
    if (strcmp(s, "full")         == 0) return RNS_IFACE_MODE_FULL;
    if (strcmp(s, "access_point") == 0) return RNS_IFACE_MODE_ACCESS_POINT;
    if (strcmp(s, "roaming")      == 0) return RNS_IFACE_MODE_ROAMING;
    if (strcmp(s, "boundary")     == 0) return RNS_IFACE_MODE_BOUNDARY;
    return RNS_IFACE_MODE_GATEWAY;
}

static void publishStats(LoraRadio* r) {
    char b[48];
    storageSet(rk(b, sizeof b, r->idx, "stats.tx_bytes"),  (int)(r->txBytes & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.rx_bytes"),  (int)(r->rxBytes & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.tx_frames"), (int)(r->txFrames & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.rx_frames"), (int)(r->rxFrames & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.crc_err"),   (int)(r->crcErr & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.split_rx_timeout"), (int)(r->splitTimeouts & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.rssi_last"), (int)r->rssiLast);
    storageSet(rk(b, sizeof b, r->idx, "stats.snr_last"),  (int)r->snrLast);
}

static void publishState(LoraRadio* r, const char* state) {
    char b[48];
    storageSet(rk(b, sizeof b, r->idx, "state"), state);
    storageSet(rk(b, sizeof b, r->idx, "up"), r->running ? 1 : 0);
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
    rnsd_transport_t reg = {};
    snprintf(reg.name, sizeof(reg.name), "lora/%d", r->idx);
    reg.mtu     = RNS_MTU;
    reg.bitrate = r->curBitrate;
    reg.mode    = r->curMode;
    reg.in = reg.out = 1;
    reg.fwd = (r->curMode == RNS_IFACE_MODE_FULL || r->curMode == RNS_IFACE_MODE_GATEWAY) ? 1 : 0;
    reg.rpt = 0;
    reg.ifac_size = r->curIfacSize;
    safeStrncpy(reg.ifac_netname, r->curIfacNetname, sizeof(reg.ifac_netname));
    safeStrncpy(reg.ifac_netkey,  r->curIfacNetkey,  sizeof(reg.ifac_netkey));
    /* ref = radio index — onRnsdDisconnect uses it to find the radio. */
    r->rnsdHandle = itsConnect("rnsd", RNSD_PORT_TRANSPORT, &reg, sizeof(reg),
                               pdMS_TO_TICKS(500), r->idx,
                               onRnsdRecv, onRnsdDisconnect);
    if (r->rnsdHandle < 0) {
        warn("lora/%d rnsd register failed", r->idx);
        return false;
    }
    info("registered as iface lora/%d (mtu=%u bitrate=%u mode=0x%02x)",
         r->idx, (unsigned)RNS_MTU, (unsigned)r->curBitrate, (unsigned)r->curMode);
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
        /* Common SX126x return codes: -2 chip not found, -705 SPI cmd timeout
         * (often TCXO/PLL), -12 invalid frequency for the band. radioBegin also
         * applies the DIO2-as-RF-switch option (SX126x) before returning. */
        err("lora/%d %s begin failed: %d", r->idx, chipName(r->slot->chip), (int)st);
        publishState(r, "error");
        return false;
    }

    /* Mode for RNS iface registration. */
    char mode[24] = "gateway";
    storageGetStr(sk(kb, sizeof kb, r->idx, "mode"), mode, sizeof(mode), "gateway");
    r->curMode    = modeFromString(mode);
    r->curBitrate = computeBitrate(sf, bw_hz, cr);

    /* IFAC: network_name is config (s.), passphrase is a secret (secrets.). */
    storageGetStr(sk(kb, sizeof kb, r->idx, "ifac_netname"), r->curIfacNetname, sizeof(r->curIfacNetname), "");
    {
        char skb[48];
        snprintf(skb, sizeof skb, "secrets.lora.%d.ifac_netkey", r->idx);
        storageGetStr(skb, r->curIfacNetkey, sizeof(r->curIfacNetkey), "");
    }
    r->curIfacSize = (uint8_t)storageGetInt(sk(kb, sizeof kb, r->idx, "ifac_size"), 0);

    storageSet(rk(kb, sizeof kb, r->idx, "chip"), chipName(r->slot->chip));
    storageSet(rk(kb, sizeof kb, r->idx, "bitrate_eff"), (int)r->curBitrate);

    /* Arm RX and hook the chip's IRQ line (unified API maps to the right DIO). */
    r->radio->setPacketReceivedAction(loraRadioIsr);
    st = r->radio->startReceive();
    if (st != RADIOLIB_ERR_NONE) {
        err("lora/%d startReceive failed: %d", r->idx, (int)st);
        publishState(r, "error");
        return false;
    }

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
        warn("lora/%d: %s NOT found at cs=%d (begin=%d)",
             r->idx, chip, r->slot->cs, (int)st);
    }
}

/* ─────────────── inbound (radio → rnsd) ─────────────── */

static void deliverInbound(LoraRadio* r, const uint8_t* data, size_t len) {
    if (r->rnsdHandle < 0) return;
    size_t s = itsSend(r->rnsdHandle, data, len, pdMS_TO_TICKS(100));
    if (s == 0) warn("lora/%d rnsd ITS send dropped (%u B)", r->idx, (unsigned)len);
}

static void drainRadioIrq(LoraRadio* r) {
    /* Read-only IRQ check: leaves an in-progress reception on another
     * radio untouched. Only act when this radio actually completed RX. */
    uint32_t flags = r->radio->getIrqFlags();
    if (!(flags & (1UL << RADIOLIB_IRQ_RX_DONE))) return;

    size_t pktLen = r->radio->getPacketLength();
    if (pktLen == 0 || pktLen > 1 + RNODE_MAX_PAYLOAD) {
        r->radio->startReceive();
        gpio_intr_enable((gpio_num_t)r->slot->dio1);
        return;
    }
    uint8_t frame[1 + RNODE_MAX_PAYLOAD];
    int16_t st = r->radio->readData(frame, pktLen);
    if (st != RADIOLIB_ERR_NONE) {
        if (st == RADIOLIB_ERR_CRC_MISMATCH) r->crcErr++;
        r->radio->startReceive();
        gpio_intr_enable((gpio_num_t)r->slot->dio1);
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

    /* Re-arm RX and re-enable the level-triggered DIO1 (the trampoline
     * disables it on each fire; readData() cleared the IRQ so DIO1 has
     * dropped low and the next packet's rising edge will fire again). */
    r->radio->startReceive();
    gpio_intr_enable((gpio_num_t)r->slot->dio1);
}

/* ─────────────── outbound (rnsd → radio) ─────────────── */

static bool sendOneFrame(LoraRadio* r, const uint8_t* frame, size_t flen) {
    int16_t st = r->radio->transmit((uint8_t*)frame, flen);
    /* RadioLib leaves the radio in standby after transmit; re-arm RX. */
    r->radio->startReceive();
    if (st != RADIOLIB_ERR_NONE) {
        warn("lora/%d transmit %u B failed: %d", r->idx, (unsigned)flen, (int)st);
        return false;
    }
    r->txFrames++;
    return true;
}

static void sendRnsPacket(LoraRadio* r, const uint8_t* data, size_t len) {
    if (!r->running || len == 0 || len > RNS_MTU) return;

    /* Random 4-bit seq id in the upper nibble. */
    uint8_t seq = (uint8_t)((esp_random() & 0x0F) << 4);

    if (len <= RNODE_MAX_PAYLOAD) {
        uint8_t frame[1 + RNODE_MAX_PAYLOAD];
        frame[0] = seq;
        std::memcpy(frame + 1, data, len);
        if (sendOneFrame(r, frame, 1 + len)) r->txBytes += len;
    } else {
        size_t first  = RNODE_MAX_PAYLOAD;
        size_t second = len - first;
        uint8_t f1[1 + RNODE_MAX_PAYLOAD];
        f1[0] = seq | RNODE_FLAG_SPLIT;
        std::memcpy(f1 + 1, data, first);
        if (!sendOneFrame(r, f1, 1 + first)) return;

        uint8_t f2[1 + RNODE_MAX_PAYLOAD];
        f2[0] = seq | RNODE_FLAG_SPLIT;
        std::memcpy(f2 + 1, data + first, second);
        if (sendOneFrame(r, f2, 1 + second)) r->txBytes += len;
    }
}

/* Drain one pending outbound packet for this radio if it's free.
 * Half-duplex: while a split RX is in flight we leave bytes sitting in the
 * ITS stream buffer (our outbound TX queue) and revisit once it clears. */
static void drainOneOutbound(LoraRadio* r) {
    if (!r->running || r->splitPending || r->rnsdHandle < 0) return;
    if (itsBytesAvailable(r->rnsdHandle) == 0) return;
    static uint8_t pkt[RNS_MTU + 16];
    size_t n = itsRecv(r->rnsdHandle, pkt, sizeof(pkt), 0);
    if (n > 0) sendRnsPacket(r, pkt, n);
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
    char mode[24]; storageGetStr(sk(kb, sizeof kb, i, "mode"), mode, sizeof mode, "gateway");
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
        for (int i = 0; i < kNumRadios; i++) storageSet(sk(kb, sizeof kb, i, "enable"), v);
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

static TickType_t nextDeadline(void) {
    TickType_t now = xTaskGetTickCount();
    TickType_t soonest = pdMS_TO_TICKS(1000);   /* cap so we publish stats */
    for (int i = 0; i < kNumRadios; i++) {
        LoraRadio* r = &s_radios[i];
        /* Outbound queued and radio free → loop immediately. */
        if (r->running && !r->splitPending && r->rnsdHandle >= 0
            && itsBytesAvailable(r->rnsdHandle) > 0) {
            return 0;
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

    itsClientInit(kNumRadios);
    storageSubscribeChanges("s.lora", onCfgChange);
    storageSubscribeChanges("secrets.lora", onCfgChange);  /* IFAC passphrase */

    /* Construct radio + HAL per slot. The shared SPI bus is brought up
     * idempotently by spi_helper (EspIdfHal::init), so every radio adds
     * its own device on the one bus. begin() is deferred to applyConfig
     * so we only touch RF hardware when a radio is enabled. The board's
     * peripheral power rail (if any) is already up — the buildable owns
     * it (e.g. hw-tdeck's tdeckPowerInit), not this transport. */
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

    for (;;) {
        if (s_configDirty) {
            s_configDirty = false;
            for (int i = 0; i < kNumRadios; i++) applyConfig(&s_radios[i]);
        }

        for (int i = 0; i < kNumRadios; i++) {
            LoraRadio* r = &s_radios[i];
            if (r->running) drainRadioIrq(r);

            if (r->splitPending &&
                (int32_t)(xTaskGetTickCount() - r->splitDeadline) >= 0) {
                r->splitPending = false;
                r->splitTimeouts++;
            }
            if (r->running && r->rnsdHandle < 0 && r->enabled) registerWithRnsd(r);
            drainOneOutbound(r);
            publishStats(r);
        }

        itsPoll(nextDeadline());
    }
}

void loraInit(void) {
    char kb[48];
    if (storageGetInt("s.lora.version", 0) < LORA_VERSION) {
        /* Per-radio defaults. Frequency + TX power are user-must-pick
         * (region / antenna); everything else defaults so an enable-toggle
         * alone gets a radio up. */
        for (int i = 0; i < kNumRadios; i++) {
            storageDefault(sk(kb, sizeof kb, i, "enable"), 0);
            storageDefault(sk(kb, sizeof kb, i, "mode"), "gateway");
            storageDefault(sk(kb, sizeof kb, i, "bandwidth"), 125000);     /* 125 kHz */
            storageDefault(sk(kb, sizeof kb, i, "spreading_factor"), 7);   /* SF7 */
            storageDefault(sk(kb, sizeof kb, i, "coding_rate"), 5);        /* 4/5 */
            storageDefault(sk(kb, sizeof kb, i, "preamble"), 12);
            storageDefault(sk(kb, sizeof kb, i, "sync_word"), "0x42");
        }
        storageSet("s.lora.version", LORA_VERSION);
    }

    cliRegisterCmd("lora", cliLora);

    /* Larger stack than other transports for the LoRa frame buffers and
     * RadioLib state machine. PSRAM stack. */
    s_task = spawnTask(loraTaskMain, TAG, 8192, nullptr, 2, 0, STACK_PSRAM);
}

#else  /* ── no radios configured (CONFIG_LORA_COUNT = 0) ── */

void loraInit(void) {
    /* iface-lora staged but inert: no LoRa pins configured for this board.
     * RadioLib links out; set CONFIG_LORA_COUNT and the pins to enable. */
}

#endif
