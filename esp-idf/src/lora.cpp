/**
 * lora — LoRa (SX1262) transport task.
 *
 * Owns the SX1262 driver via RadioLib + EspIdfHal. DIO1 ISR
 * notifies this task; task-side reads IRQ status, drains FIFO,
 * reassembles RNode-framed packets, forwards to rnsd. Self-registers
 * with rnsd as the `lora` interface.
 *
 * RNode on-air framing (mandatory for interop, see plan §7.1):
 *   [1B header][≤254B payload]
 *   header upper nibble = random sequence id
 *   header bit 0       = SPLIT (this is part of a 2-frame split packet)
 *
 * RX wake mechanism: setDio1Action → HAL attachInterrupt → IRAM_ATTR
 * trampoline → vTaskNotifyGiveFromISR. itsPoll() unblocks on the same
 * task notification, so a single wait point handles ITS, ISR, and
 * deadlines (the diptych canonical loop pattern).
 *
 * TX: synchronous radio.transmit() in the task. SX126x is half-duplex
 * so we cannot transmit while a split RX is pending — guarded by
 * s_splitPending.
 *
 * See docs/component-plan.md §7 / §12.
 */
#include "lora.h"
#include "board.h"
#include "esp_idf_hal.h"
#include "diptych.h"
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

#define LORA_VERSION         1
#define RNS_MTU              500
#define RNODE_MAX_PAYLOAD    254
#define RNODE_FLAG_SPLIT     0x01
#define SPLIT_RX_TIMEOUT_MS  5000
#define LORA_RNSD_CONNECT_REF 1

/* ─────────────── globals (single-task ownership) ─────────────── */

static EspIdfHal*    s_hal     = nullptr;
static Module*       s_mod     = nullptr;
static SX1262*       s_radio   = nullptr;
static TaskHandle_t  s_task    = nullptr;

static int           s_rnsdHandle = -1;

static bool          s_running    = false;   /* radio initialized + RX armed */
static bool          s_enabled    = false;   /* s.lora.enable applied */
static uint8_t       s_curMode    = RNS_IFACE_MODE_GATEWAY;
static uint32_t      s_curBitrate = 0;

/* Split-RX reassembly state. Only one in-flight split at a time — if a
 * second sender's split interleaves, we restart on the new seq nibble. */
static uint8_t       s_splitBuf[RNS_MTU + 16];
static size_t        s_splitLen     = 0;
static uint8_t       s_splitSeq     = 0;
static bool          s_splitPending = false;
static TickType_t    s_splitDeadline = 0;

/* Stats — published to ephemeral storage once per task tick. */
static uint64_t      s_txBytes = 0, s_rxBytes = 0;
static uint64_t      s_txFrames = 0, s_rxFrames = 0;
static uint64_t      s_crcErr = 0, s_splitTimeouts = 0;
static float         s_rssiLast = 0.0f, s_snrLast = 0.0f;

static volatile bool s_configDirty = true;

/* ─────────────── ISR ─────────────── */

static IRAM_ATTR void loraRadioIsr(void) {
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &hp);
    portYIELD_FROM_ISR(hp);
}

/* ─────────────── storage helpers ─────────────── */

static uint32_t computeBitrate(int sf, int bw_hz, int cr_denom) {
    /* LoRa raw bit rate (bits/s) = SF * (BW / 2^SF) * (4 / cr_denom). */
    if (sf <= 0 || bw_hz <= 0 || cr_denom < 5 || cr_denom > 8) return 0;
    double rb = (double)sf * (double)bw_hz * 4.0
              / ((double)((uint32_t)1 << sf) * (double)cr_denom);
    return (uint32_t)rb;
}

static uint8_t modeFromString(const char* s) {
    if (!s || !*s)                   return RNS_IFACE_MODE_GATEWAY;
    if (strcmp(s, "full")         == 0) return RNS_IFACE_MODE_FULL;
    if (strcmp(s, "access_point") == 0) return RNS_IFACE_MODE_ACCESS_POINT;
    if (strcmp(s, "roaming")      == 0) return RNS_IFACE_MODE_ROAMING;
    if (strcmp(s, "boundary")     == 0) return RNS_IFACE_MODE_BOUNDARY;
    return RNS_IFACE_MODE_GATEWAY;
}

static void publishStats(void) {
    storageSet("lora.stats.tx_bytes",   (int)(s_txBytes & 0x7fffffff));
    storageSet("lora.stats.rx_bytes",   (int)(s_rxBytes & 0x7fffffff));
    storageSet("lora.stats.tx_frames",  (int)(s_txFrames & 0x7fffffff));
    storageSet("lora.stats.rx_frames",  (int)(s_rxFrames & 0x7fffffff));
    storageSet("lora.stats.crc_err",    (int)(s_crcErr & 0x7fffffff));
    storageSet("lora.stats.split_rx_timeout", (int)(s_splitTimeouts & 0x7fffffff));
    storageSet("lora.stats.rssi_last",  (int)s_rssiLast);
    storageSet("lora.stats.snr_last",   (int)s_snrLast);
}

static void publishState(const char* state) {
    storageSet("lora.state", state);
    storageSet("lora.up", s_running ? 1 : 0);
}

/* ─────────────── rnsd registration ─────────────── */

static void onRnsdRecv(int handle, size_t /*bytesAvail*/);
static void onRnsdDisconnect(int ref);

static void deregisterFromRnsd(void) {
    if (s_rnsdHandle >= 0) {
        itsDisconnect(s_rnsdHandle);
        s_rnsdHandle = -1;
    }
}

static bool registerWithRnsd(void) {
    deregisterFromRnsd();
    rnsd_transport_t reg = {};
    safeStrncpy(reg.name, "lora", sizeof(reg.name));
    reg.mtu     = RNS_MTU;
    reg.bitrate = s_curBitrate;
    reg.mode    = s_curMode;
    reg.in = reg.out = 1;
    reg.fwd = (s_curMode == RNS_IFACE_MODE_FULL || s_curMode == RNS_IFACE_MODE_GATEWAY) ? 1 : 0;
    reg.rpt = 0;
    s_rnsdHandle = itsConnect("rnsd", RNSD_PORT_TRANSPORT, &reg, sizeof(reg),
                              pdMS_TO_TICKS(500), LORA_RNSD_CONNECT_REF,
                              onRnsdRecv, onRnsdDisconnect);
    if (s_rnsdHandle < 0) {
        warn("rnsd register failed");
        return false;
    }
    info("registered as iface lora (mtu=%u bitrate=%u mode=0x%02x)",
         (unsigned)RNS_MTU, (unsigned)s_curBitrate, (unsigned)s_curMode);
    return true;
}

static void onRnsdDisconnect(int /*ref*/) {
    s_rnsdHandle = -1;
    /* The task loop will re-register if we're still enabled. */
}

/* ─────────────── radio control ─────────────── */

static void radioStop(void) {
    if (!s_radio) return;
    pmGpioWakeDisable(BOARD_LORA_DIO1_PIN);
    s_radio->setDio1Action(nullptr);
    /* Cold-start SLEEP (retainConfig=false): ~160 nA, ~5 ms wake-up on
     * next begin(). We always re-apply config in radioStart, so there's
     * no value in warm-start retention. */
    s_radio->sleep(false);
    s_running = false;
    s_splitPending = false;
    s_splitLen = 0;
    deregisterFromRnsd();
    publishState("down");
}

static bool radioStart(void) {
    /* Required config presence. Frequencies are stored in Hz; bandwidth
     * in Hz; SF/CR/TXP as small ints. With no defaults installed we
     * refuse to bring the radio up — matches the "no preselected
     * defaults" rule in plan §12.4. */
    int freq_hz   = storageGetInt("s.lora.frequency", 0);
    int bw_hz     = storageGetInt("s.lora.bandwidth", 0);
    int sf        = storageGetInt("s.lora.spreading_factor", 0);
    int cr        = storageGetInt("s.lora.coding_rate", 0);
    int txp       = storageGetInt("s.lora.tx_power", 0);
    int preamble  = storageGetInt("s.lora.preamble", 12);

    /* Sync word is stored as a string so the panel can accept hex like
     * "0x42" alongside plain decimal. strtol(base=0) handles both. */
    char syncBuf[16] = "";
    storageGetStr("s.lora.sync_word", syncBuf, sizeof(syncBuf), "0x42");
    int syncWord = (int)strtol(syncBuf, nullptr, 0);
    if (syncWord <= 0 || syncWord > 0xFF) syncWord = 0x42;

    if (freq_hz <= 0 || bw_hz <= 0 || sf < 5 || sf > 12 ||
        cr < 5 || cr > 8 || txp < -9 || txp > 22) {
        info("not started: configure freq/bw/sf/cr/txp first");
        publishState("unconfigured");
        return false;
    }

    /* RadioLib takes frequency in MHz and bandwidth in kHz. */
    float freq_mhz = (float)freq_hz / 1.0e6f;
    float bw_khz   = (float)bw_hz   / 1.0e3f;

    int16_t st = s_radio->begin(freq_mhz, bw_khz, (uint8_t)sf, (uint8_t)cr,
                                (uint8_t)syncWord, (int8_t)txp,
                                (uint16_t)preamble,
                                BOARD_LORA_TCXO_VOLTAGE, /*useRegulatorLDO*/ false);
    if (st != RADIOLIB_ERR_NONE) {
        /* Common SX126x return codes:
         *   -2   = chip not found (no SPI response: power, wiring)
         *   -705 = SPI_CMD_TIMEOUT (chip got a command but couldn't
         *          complete it — often TCXO not stabilised, wrong
         *          frequency band for the matching network, or PLL
         *          failed to lock)
         *   -706 = SPI_CMD_INVALID
         *   -707 = SPI_CMD_FAILED
         *   -12  = invalid frequency for the SX1262 (out of 150–960 MHz) */
        err("SX1262 begin failed: %d", (int)st);
        publishState("error");
        return false;
    }

#if BOARD_LORA_DIO2_RF_SWITCH
    st = s_radio->setDio2AsRfSwitch(true);
    if (st != RADIOLIB_ERR_NONE) warn("setDio2AsRfSwitch: %d", (int)st);
#endif

    /* Mode for RNS iface registration. */
    char mode[24] = "gateway";
    storageGetStr("s.lora.mode", mode, sizeof(mode), "gateway");
    s_curMode    = modeFromString(mode);
    s_curBitrate = computeBitrate(sf, bw_hz, cr);

    storageSet("lora.chip", "SX1262");
    storageSet("lora.bitrate_eff", (int)s_curBitrate);

    /* Arm RX and hook DIO1. */
    s_radio->setDio1Action(loraRadioIsr);
    st = s_radio->startReceive();
    if (st != RADIOLIB_ERR_NONE) {
        err("startReceive failed: %d", (int)st);
        publishState("error");
        return false;
    }

    s_running = true;
    publishState("up");
    info("up: %.3f MHz BW=%.0fkHz SF%d CR4/%d TXP=%ddBm preamble=%d sync=0x%02x",
         (double)freq_mhz, (double)bw_khz, sf, cr, txp, preamble, syncWord);

    /* Register with rnsd as a transport. */
    if (!registerWithRnsd()) {
        publishState("rnsd_unavailable");
        /* Stay up on radio — task loop will retry register. */
    }
    return true;
}

/* ─────────────── inbound (radio → rnsd) ─────────────── */

static void deliverInbound(const uint8_t* data, size_t len) {
    if (s_rnsdHandle < 0) return;
    size_t s = itsSend(s_rnsdHandle, data, len, pdMS_TO_TICKS(100));
    if (s == 0) warn("rnsd ITS send dropped (%u B)", (unsigned)len);
}

static void drainRadioIrq(void) {
    /* Read length; readData fetches data + clears IRQ status. */
    size_t pktLen = s_radio->getPacketLength();
    if (pktLen == 0 || pktLen > 1 + RNODE_MAX_PAYLOAD) {
        s_radio->startReceive();
        return;
    }
    uint8_t frame[1 + RNODE_MAX_PAYLOAD];
    int16_t st = s_radio->readData(frame, pktLen);
    if (st != RADIOLIB_ERR_NONE) {
        if (st == RADIOLIB_ERR_CRC_MISMATCH) s_crcErr++;
        s_radio->startReceive();
        return;
    }
    s_rxFrames++;
    s_rssiLast = s_radio->getRSSI();
    s_snrLast  = s_radio->getSNR();

    if (pktLen < 1) { s_radio->startReceive(); return; }
    uint8_t  header     = frame[0];
    uint8_t  seq        = header & 0xF0;
    bool     isSplit    = (header & RNODE_FLAG_SPLIT) != 0;
    size_t   payloadLen = pktLen - 1;

    if (!isSplit) {
        deliverInbound(frame + 1, payloadLen);
        s_rxBytes += payloadLen;
    } else if (!s_splitPending) {
        std::memcpy(s_splitBuf, frame + 1, payloadLen);
        s_splitLen     = payloadLen;
        s_splitSeq     = seq;
        s_splitPending = true;
        s_splitDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(SPLIT_RX_TIMEOUT_MS);
    } else if (s_splitSeq == seq) {
        if (s_splitLen + payloadLen <= sizeof(s_splitBuf)) {
            std::memcpy(s_splitBuf + s_splitLen, frame + 1, payloadLen);
            s_splitLen += payloadLen;
            deliverInbound(s_splitBuf, s_splitLen);
            s_rxBytes += s_splitLen;
        }
        s_splitPending = false;
    } else {
        /* Different sender's split — restart assembly on the new seq. */
        std::memcpy(s_splitBuf, frame + 1, payloadLen);
        s_splitLen      = payloadLen;
        s_splitSeq      = seq;
        s_splitDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(SPLIT_RX_TIMEOUT_MS);
    }

    /* Re-arm RX for the next frame. Once SX126x clears IRQ (which
     * readData() did internally), DIO1 has dropped low, so it is safe
     * to re-enable the level-triggered GPIO interrupt: it will fire on
     * the next packet's DIO1 rising edge. The trampoline disables it
     * each fire to prevent re-entry in level mode. */
    s_radio->startReceive();
    gpio_intr_enable((gpio_num_t)BOARD_LORA_DIO1_PIN);
}

/* ─────────────── outbound (rnsd → radio) ─────────────── */

static bool sendOneFrame(const uint8_t* frame, size_t flen) {
    int16_t st = s_radio->transmit((uint8_t*)frame, flen);
    /* RadioLib leaves the radio in standby after transmit; re-arm RX. */
    s_radio->startReceive();
    if (st != RADIOLIB_ERR_NONE) {
        warn("transmit %u B failed: %d", (unsigned)flen, (int)st);
        return false;
    }
    s_txFrames++;
    return true;
}

static void sendRnsPacket(const uint8_t* data, size_t len) {
    if (!s_running || len == 0 || len > RNS_MTU) return;

    /* Random 4-bit seq id in the upper nibble. */
    uint8_t seq = (uint8_t)((esp_random() & 0x0F) << 4);

    if (len <= RNODE_MAX_PAYLOAD) {
        uint8_t frame[1 + RNODE_MAX_PAYLOAD];
        frame[0] = seq;
        std::memcpy(frame + 1, data, len);
        if (sendOneFrame(frame, 1 + len)) s_txBytes += len;
    } else {
        size_t first  = RNODE_MAX_PAYLOAD;
        size_t second = len - first;
        uint8_t f1[1 + RNODE_MAX_PAYLOAD];
        f1[0] = seq | RNODE_FLAG_SPLIT;
        std::memcpy(f1 + 1, data, first);
        if (!sendOneFrame(f1, 1 + first)) return;

        uint8_t f2[1 + RNODE_MAX_PAYLOAD];
        f2[0] = seq | RNODE_FLAG_SPLIT;
        std::memcpy(f2 + 1, data + first, second);
        if (sendOneFrame(f2, 1 + second)) s_txBytes += len;
    }
}

/* Drain one pending outbound packet from rnsd if the radio is free.
 * Half-duplex: while a split RX is in flight we leave bytes sitting in
 * the ITS stream buffer (it's our outbound TX queue) and let the task
 * loop revisit once s_splitPending clears. */
static void drainOneOutbound(void) {
    if (!s_running || s_splitPending || s_rnsdHandle < 0) return;
    if (itsBytesAvailable(s_rnsdHandle) == 0) return;
    static uint8_t pkt[RNS_MTU + 16];
    size_t n = itsRecv(s_rnsdHandle, pkt, sizeof(pkt), 0);
    if (n > 0) sendRnsPacket(pkt, n);
}

static void onRnsdRecv(int /*handle*/, size_t /*bytesAvail*/) {
    drainOneOutbound();
}

/* ─────────────── config reload ─────────────── */

static void applyConfig(void) {
    s_enabled = storageGetInt("s.lora.enable", 0) != 0;

    if (!s_enabled) {
        if (s_running) {
            info("disable");
            radioStop();
        }
        return;
    }

    /* If already running, stop and start to pick up new params.
     * Cheap (~30 ms) and avoids tracking which fields changed. */
    if (s_running) radioStop();
    radioStart();
}

static void onCfgChange(const char* /*key*/, const char* /*val*/) {
    s_configDirty = true;
    if (s_task) xTaskNotifyGive(s_task);
}

/* ─────────────── CLI ─────────────── */

static void cliLora(const char* args) {
    if (args && strcmp(args, "help") == 0) {
        cliPrintf("  %-*s LoRa transport status\n",   CLI_HELP_COL, "lora");
        cliPrintf("  %-*s enable/disable LoRa\n",     CLI_HELP_COL, "lora up|down");
        return;
    }
    if (args && strcmp(args, "up") == 0)   { storageSet("s.lora.enable", 1); cliPrintf("enabled\n"); return; }
    if (args && strcmp(args, "down") == 0) { storageSet("s.lora.enable", 0); cliPrintf("disabled\n"); return; }

    cliPrintf("state:    %s\n",       s_running ? "up" : (s_enabled ? "starting" : "down"));
    cliPrintf("chip:     SX1262 (%s)\n", BOARD_NAME);
    int freq_hz = storageGetInt("s.lora.frequency", 0);
    int bw_hz   = storageGetInt("s.lora.bandwidth", 0);
    int sf      = storageGetInt("s.lora.spreading_factor", 0);
    int cr      = storageGetInt("s.lora.coding_rate", 0);
    int txp     = storageGetInt("s.lora.tx_power", 0);
    cliPrintf("freq:     %d Hz\n",    freq_hz);
    cliPrintf("bw:       %d Hz\n",    bw_hz);
    cliPrintf("sf:       %d\n",       sf);
    cliPrintf("cr:       4/%d\n",     cr);
    cliPrintf("txp:      %d dBm\n",   txp);
    cliPrintf("bitrate:  %u bit/s\n", (unsigned)s_curBitrate);
    cliPrintf("rx:       %u frames, %u bytes\n", (unsigned)s_rxFrames, (unsigned)s_rxBytes);
    cliPrintf("tx:       %u frames, %u bytes\n", (unsigned)s_txFrames, (unsigned)s_txBytes);
    cliPrintf("rssi:     %d dBm  snr: %d dB\n",  (int)s_rssiLast, (int)s_snrLast);
    cliPrintf("errors:   crc=%u split_to=%u\n",
              (unsigned)s_crcErr, (unsigned)s_splitTimeouts);
}

/* ─────────────── task ─────────────── */

static TickType_t nextDeadline(void) {
    TickType_t now = xTaskGetTickCount();
    /* Outbound queued in the ITS stream and radio free → loop immediately
     * so we drain the next packet without waiting for an ITS notify. We
     * already drain one per loop turn rather than back-to-back inside the
     * callback, so inbound IRQs in the FIFO still get serviced between
     * transmissions (drainRadioIrq runs before drainOneOutbound). */
    if (s_running && !s_splitPending && s_rnsdHandle >= 0
        && itsBytesAvailable(s_rnsdHandle) > 0) {
        return 0;
    }
    TickType_t soonest = pdMS_TO_TICKS(1000);   /* cap so we publish stats */
    if (s_splitPending) {
        TickType_t d = (s_splitDeadline > now) ? (s_splitDeadline - now) : 0;
        if (d < soonest) soonest = d;
    }
    return soonest;
}

static void loraTaskMain(void*) {
    info("[%s] task up (%s)", TAG, BOARD_NAME);

    itsClientInit(4);
    storageSubscribeChanges("s.lora", onCfgChange);

    /* Drive the board-level peripheral power-enable if there is one.
     * On T-Deck (Plus) GPIO 10 gates the 3.3 V rail to LoRa + SD +
     * display + GPS — without it, SX1262 has no power, SPI MISO is
     * floating, BUSY never drops, begin() returns RADIOLIB_ERR_SPI_CMD_TIMEOUT
     * (-706) or RADIOLIB_ERR_CHIP_NOT_FOUND (-2). */
#if BOARD_POWER_EN_PIN >= 0
    gpio_config_t pwr = {};
    pwr.pin_bit_mask = 1ULL << BOARD_POWER_EN_PIN;
    pwr.mode         = GPIO_MODE_OUTPUT;
    pwr.pull_up_en   = GPIO_PULLUP_DISABLE;
    pwr.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pwr.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&pwr);
    gpio_set_level((gpio_num_t)BOARD_POWER_EN_PIN, BOARD_POWER_EN_ACTIVE ? 1 : 0);
    /* 100 ms settling for the 3.3 V rail. T-Deck Plus's TCXO regulator
     * appears to take longer than the 5 ms RadioLib defaults to wait
     * after setting DIO3 as TCXO control. Easier to fix at the rail. */
    vTaskDelay(pdMS_TO_TICKS(100));
    info("board power: GPIO %d -> %s",
         BOARD_POWER_EN_PIN, BOARD_POWER_EN_ACTIVE ? "HIGH" : "LOW");
#endif

    /* Construct radio + HAL on the task stack/heap. HAL init brings up
     * SPI + GPIO ISR service; radio.begin() is deferred to applyConfig
     * so we only touch hardware when the user enables. */
    s_hal = new EspIdfHal(BOARD_LORA_SPI_HOST,
                         BOARD_LORA_SCK_PIN, BOARD_LORA_MOSI_PIN, BOARD_LORA_MISO_PIN,
                         BOARD_LORA_CS_PIN);
    s_hal->init();
    s_mod = new Module(s_hal, BOARD_LORA_CS_PIN, BOARD_LORA_DIO1_PIN,
                       BOARD_LORA_RST_PIN, BOARD_LORA_BUSY_PIN);
    s_radio = new SX1262(s_mod);

    for (;;) {
        if (s_configDirty) { s_configDirty = false; applyConfig(); }

        /* IRQ was likely the wake source; cheap to always check. RadioLib
         * caches IRQ state after we've issued readData/transmit. */
        if (s_running) drainRadioIrq();

        /* Split-RX timeout. */
        if (s_splitPending && (int32_t)(xTaskGetTickCount() - s_splitDeadline) >= 0) {
            s_splitPending = false;
            s_splitTimeouts++;
        }

        /* Re-register with rnsd if it dropped while we're enabled. */
        if (s_running && s_rnsdHandle < 0 && s_enabled) {
            registerWithRnsd();
        }

        /* If outbound packets queued up while we were busy (split RX or
         * mid-TX), drain one. We do one per loop turn so radio RX gets
         * re-armed between back-to-back transmissions. */
        drainOneOutbound();

        publishStats();
        itsPoll(nextDeadline());
    }
}

#if CONFIG_DIPTYCH_LCD
#include "lcd.h"
/* Settings → Reticulum → Transports → LoRa. Mirrors the web LoraPanel. Selects
 * store the raw wire values (the device atoi's them), same as the browser. */
static void loraSettingsPane(void* arg) {
    lv_obj_t* p = static_cast<lv_obj_t*>(arg);
    lcdSettingSection (p, "LoRa");
    lcdSettingSwitch  (p, "Enable", "s.lora.enable");
    lcdSettingSection (p, "Radio");
    lcdSettingDropdown(p, "Frequency", "s.lora.frequency",
                       "433000000,868100000,869525000,915000000,923000000");
    lcdSettingDropdown(p, "Bandwidth", "s.lora.bandwidth", "125000,250000,500000");
    lcdSettingDropdown(p, "Spreading", "s.lora.spreading_factor", "7,8,9,10,11,12");
    lcdSettingDropdown(p, "Coding rate", "s.lora.coding_rate", "5,6,7,8");
    lcdSettingSlider  (p, "TX power", "s.lora.tx_power", -9, 22);
    lcdSettingSlider  (p, "Preamble", "s.lora.preamble", 6, 32);
    lcdSettingText    (p, "Sync word", "s.lora.sync_word");
    lcdSettingDropdown(p, "Mode", "s.lora.mode",
                       "full,gateway,access_point,roaming,boundary");
    lcdSettingSection (p, "Status");
    lcdSettingValue   (p, "State", "lora.state");
    lcdSettingValue   (p, "Chip", "lora.chip");
    lcdSettingValue   (p, "Bitrate", "lora.bitrate_eff");
    lcdSettingValue   (p, "RSSI", "lora.stats.rssi_last");
}
#endif

void loraInit(void) {
    if (storageGetInt("s.lora.version", 0) < LORA_VERSION) {
        /* Frequency + TX power are user-must-pick (region / antenna).
         * Everything else has a sensible default so an enable-toggle
         * alone gets the radio up. */
        storageDefault("s.lora.enable", 0);
        storageDefault("s.lora.mode", "gateway");
        storageDefault("s.lora.bandwidth", 125000);     /* 125 kHz */
        storageDefault("s.lora.spreading_factor", 7);   /* SF7 */
        storageDefault("s.lora.coding_rate", 5);        /* 4/5 */
        storageDefault("s.lora.preamble", 12);
        storageDefault("s.lora.sync_word", "0x42");
        storageSet("s.lora.version", LORA_VERSION);
    }

#if CONFIG_DIPTYCH_LCD
    lcdRegisterSettings("Reticulum/Transports/LoRa", "LoRa", loraSettingsPane);
#endif

    cliRegisterCmd("lora", cliLora);

    /* Slightly larger stack than other transports because of the LoRa
     * frame buffers and RadioLib state machine. PSRAM stack. */
    s_task = spawnTask(loraTaskMain, TAG, 8192, nullptr, 2, 0, STACK_PSRAM);
}
