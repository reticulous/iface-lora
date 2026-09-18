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
#include "lora_priv.h"

#if CONFIG_IDF_TARGET_LINUX
#include "host/ether_task.h"
#include "host/virtual_hal.h"
#endif

#include "lora_fem.h"
#if CONFIG_STRADDLE_NETGRAPH
#include "netgraph.h"     /* netgraphContributeIface — our `if` line's tail */
#endif

#if defined(CONFIG_LORA0_CS_PIN)

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
      CONFIG_LORA0_FEM_PWR_PIN, CONFIG_LORA0_FEM_EN_PIN,
      CONFIG_LORA0_FEM_TXSEL_A_PIN, CONFIG_LORA0_FEM_TXSEL_B_PIN,
      CONFIG_LORA0_FEM_GAIN_DB, CONFIG_LORA0_TX_CAL, CONFIG_LORA0_RSSI_CAL,
      CONFIG_LORA0_FEM_HF_PWR_PIN,
      CONFIG_LORA0_FEM_HF_GAIN_DB, CONFIG_LORA0_LR_IRQ_DIO,
      { CONFIG_LORA0_LR_RFSW_IDLE, CONFIG_LORA0_LR_RFSW_RX,
        CONFIG_LORA0_LR_RFSW_TX, CONFIG_LORA0_LR_RFSW_RX_HF,
        CONFIG_LORA0_LR_RFSW_TX_HF },
      CONFIG_LORA0_LR_RFSW_TX_BYPASS,
      (LoraChip)CONFIG_LORA0_CHIP_ID },
#if defined(CONFIG_LORA1_CS_PIN)
    { CONFIG_LORA1_CS_PIN, CONFIG_LORA1_DIO1_PIN, CONFIG_LORA1_BUSY_PIN, CONFIG_LORA1_RST_PIN,
      CONFIG_LORA1_TCXO_MV, LORA1_DIO2, CONFIG_LORA1_RFSW_RX_PIN, CONFIG_LORA1_RFSW_TX_PIN,
      CONFIG_LORA1_FEM_PWR_PIN, CONFIG_LORA1_FEM_EN_PIN,
      CONFIG_LORA1_FEM_TXSEL_A_PIN, CONFIG_LORA1_FEM_TXSEL_B_PIN,
      CONFIG_LORA1_FEM_GAIN_DB, CONFIG_LORA1_TX_CAL, CONFIG_LORA1_RSSI_CAL,
      CONFIG_LORA1_FEM_HF_PWR_PIN,
      CONFIG_LORA1_FEM_HF_GAIN_DB, CONFIG_LORA1_LR_IRQ_DIO,
      { CONFIG_LORA1_LR_RFSW_IDLE, CONFIG_LORA1_LR_RFSW_RX,
        CONFIG_LORA1_LR_RFSW_TX, CONFIG_LORA1_LR_RFSW_RX_HF,
        CONFIG_LORA1_LR_RFSW_TX_HF },
      CONFIG_LORA1_LR_RFSW_TX_BYPASS,
      (LoraChip)CONFIG_LORA1_CHIP_ID },
#endif
#if defined(CONFIG_LORA2_CS_PIN)
    { CONFIG_LORA2_CS_PIN, CONFIG_LORA2_DIO1_PIN, CONFIG_LORA2_BUSY_PIN, CONFIG_LORA2_RST_PIN,
      CONFIG_LORA2_TCXO_MV, LORA2_DIO2, CONFIG_LORA2_RFSW_RX_PIN, CONFIG_LORA2_RFSW_TX_PIN,
      CONFIG_LORA2_FEM_PWR_PIN, CONFIG_LORA2_FEM_EN_PIN,
      CONFIG_LORA2_FEM_TXSEL_A_PIN, CONFIG_LORA2_FEM_TXSEL_B_PIN,
      CONFIG_LORA2_FEM_GAIN_DB, CONFIG_LORA2_TX_CAL, CONFIG_LORA2_RSSI_CAL,
      CONFIG_LORA2_FEM_HF_PWR_PIN,
      CONFIG_LORA2_FEM_HF_GAIN_DB, CONFIG_LORA2_LR_IRQ_DIO,
      { CONFIG_LORA2_LR_RFSW_IDLE, CONFIG_LORA2_LR_RFSW_RX,
        CONFIG_LORA2_LR_RFSW_TX, CONFIG_LORA2_LR_RFSW_RX_HF,
        CONFIG_LORA2_LR_RFSW_TX_HF },
      CONFIG_LORA2_LR_RFSW_TX_BYPASS,
      (LoraChip)CONFIG_LORA2_CHIP_ID },
#endif
#if defined(CONFIG_LORA3_CS_PIN)
    { CONFIG_LORA3_CS_PIN, CONFIG_LORA3_DIO1_PIN, CONFIG_LORA3_BUSY_PIN, CONFIG_LORA3_RST_PIN,
      CONFIG_LORA3_TCXO_MV, LORA3_DIO2, CONFIG_LORA3_RFSW_RX_PIN, CONFIG_LORA3_RFSW_TX_PIN,
      CONFIG_LORA3_FEM_PWR_PIN, CONFIG_LORA3_FEM_EN_PIN,
      CONFIG_LORA3_FEM_TXSEL_A_PIN, CONFIG_LORA3_FEM_TXSEL_B_PIN,
      CONFIG_LORA3_FEM_GAIN_DB, CONFIG_LORA3_TX_CAL, CONFIG_LORA3_RSSI_CAL,
      CONFIG_LORA3_FEM_HF_PWR_PIN,
      CONFIG_LORA3_FEM_HF_GAIN_DB, CONFIG_LORA3_LR_IRQ_DIO,
      { CONFIG_LORA3_LR_RFSW_IDLE, CONFIG_LORA3_LR_RFSW_RX,
        CONFIG_LORA3_LR_RFSW_TX, CONFIG_LORA3_LR_RFSW_RX_HF,
        CONFIG_LORA3_LR_RFSW_TX_HF },
      CONFIG_LORA3_LR_RFSW_TX_BYPASS,
      (LoraChip)CONFIG_LORA3_CHIP_ID },
#endif
};
static_assert((int)(sizeof(kSlots) / sizeof(kSlots[0])) == kNumRadios,
              "kSlots out of step with LORA_NUM_RADIOS (lora_priv.h)");

/* ─────────────── shared state owned here ─────────────── */

LoraRadio s_radios[LORA_NUM_RADIOS];

TaskHandle_t  s_task = nullptr;
volatile bool s_stop = false;   /* rns stop → break the work loop and park */
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
static volatile bool       s_cfgPend    = true;
static volatile TickType_t s_cfgDueTick = 0;
/* The settle window (LORA_CFG_SETTLE_MS): true while the pending deadline is a
 * settling one, which is both what lets a further change push it out and what
 * holds the radio off the air meanwhile. An urgent cfgArm clears it — that
 * deadline is immovable and its apply is not a user editing anything. */
static volatile bool       s_settling      = false;
static volatile TickType_t s_settleCapTick = 0;
static volatile bool s_displayDirty = false;   /* an MHz/kHz display key was edited */

/* Ask for a config apply in at most `delayMs`. Keeps the earlier of the
 * requested and any already-pending deadline — see the coalescing note above. */
void cfgArm(uint32_t delayMs) {
    TickType_t due = xTaskGetTickCount() + pdMS_TO_TICKS(delayMs);
    if (s_cfgPend && (int32_t)(due - s_cfgDueTick) >= 0) {
        if (s_task) xTaskNotifyGive(s_task);   /* pending and no sooner: just wake */
        return;
    }
    s_cfgDueTick = due;
    s_cfgPend    = true;
    s_settling   = false;   /* pulled in by something that is not a user's edit */
    if (s_task) xTaskNotifyGive(s_task);
}

/* Ask for an apply once the settings have stopped moving: LORA_CFG_SETTLE_MS
 * from now, pushed out by every further change, and never past the cap the
 * first change of the burst set.
 *
 * This is the one arming path that may move a deadline later, so it may only
 * move its OWN: an apply already pending for any other reason is left where it
 * is, or a stream of edits could hold off a radio the orchestrator is trying to
 * bring up. */
static void cfgArmSettle(void) {
    TickType_t now = xTaskGetTickCount();
    if (s_cfgPend && !s_settling) {
        if (s_task) xTaskNotifyGive(s_task);
        return;
    }
    if (!s_settling) {
        s_settling      = true;
        s_settleCapTick = now + pdMS_TO_TICKS(LORA_CFG_SETTLE_MAX_MS);
    }
    TickType_t due = now + pdMS_TO_TICKS(LORA_CFG_SETTLE_MS);
    if ((int32_t)(due - s_settleCapTick) > 0) due = s_settleCapTick;
    s_cfgDueTick = due;
    s_cfgPend    = true;
    if (s_task) xTaskNotifyGive(s_task);
}

bool loraCfgQuiet(void) { return s_cfgPend && s_settling; }
static volatile bool     s_radioIrq  = false;  /* DIO1 fired; gate the chip SPI poll on it */
static volatile uint32_t s_radioIrqUs = 0;     /* µs stamp of the last DIO1 IRQ (truncated
                                                * esp_timer time; 32-bit so the store is
                                                * atomic) */

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

/* ─────────────── radio control ─────────────── */

static void radioStop(LoraRadio* r) {
    if (!r->radio) return;   /* never came up — see radioStart */
#if !defined(CONFIG_LORA_NO_SUPE)
    /* Take SUPE's lock before touching the chip, not after. Every `s.lora.*`
     * write arrives here on its way to a restart, and a SUPE transaction step
     * runs on the esp_timer task — so without this a config edit sleeps the
     * radio out from under a callback that is mid-sense or mid-transmit. The
     * wait is bounded by that callback (RadioLib's transmit has its own
     * timeout), and it is a leaf lock, so there is nothing to deadlock against. */
    bool supeHeldHere = r->supe != nullptr;
    if (supeHeldHere) supeLock(r);
#endif

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
#if !defined(CONFIG_LORA_NO_SUPE)
    /* A SUPE transaction dies with the radio too, and for the same reason: no
     * RF restore is owed when the chip is going down and the whole modem regime
     * is re-applied on the way up. Queued packets go with it — they were never
     * transmitted, and the layers above deal with that as they always do. */
    supeOnRadioStop(r);
#endif
    loraqFlush(&r->q);   /* queued packets die with the radio */
    r->chNow = LORA_CH_HAIL;
#if !defined(CONFIG_LORA_NO_SUPE)
    if (supeHeldHere) supeUnlock(r);
#endif
    peersAbandonPends(r);  /* proofs can't return while RF is down — drop, uncounted */
    deregisterFromRnsd(r);
    publishState(r, "down");
}

static bool radioStart(LoraRadio* r) {
    /* No radio object means its HAL never got a bus (see the construction loop):
     * there is nothing to configure and nothing to talk to. */
    if (!r->radio) return false;
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

    /* No upper bound on txp here: what a board can reach at the antenna is a
     * board fact (a front end takes it past the chip's own maximum) and it also
     * depends on the band, which is settled a few lines down. A number above
     * the ceiling is clamped with a warning below, not a reason to refuse to
     * start. The floor is the lowest drive any supported part accepts. */
    if (freq_hz <= 0 || bw_hz <= 0 || sf < 5 || sf > 12 ||
        cr < 5 || cr > 8 || txp < -19) {
        info("lora/%d not started: configure freq/bw/sf/cr/txp first", r->idx);
        publishState(r, "unconfigured");
        return false;
    }

    /* The bottom of the range is the PART's, not the standard's: an SX127x
     * reaches neither SF5 nor SF6 through this framing (radioMinSf). Clamped
     * with a warning rather than refused — the same treatment tx_power gets
     * above its ceiling, and for the same reason: a number the operator typed
     * for another board should cost them a line in the log, not a radio that
     * will not come up. */
    {
        uint8_t sfMin = radioMinSf(chipFamily(r->slot->chip));
        if (sf < sfMin) {
            warn("lora/%d %s cannot run SF%d, using SF%u",
                 r->idx, chipName(r->slot->chip), sf, (unsigned)sfMin);
            sf = sfMin;
            storageSet(sk(kb, sizeof kb, r->idx, "spreading_factor"), sf);
        }
    }

    /* RadioLib takes frequency in MHz and bandwidth in kHz; TCXO in volts. */
    float freq_mhz = (float)freq_hz / 1.0e6f;
    float bw_khz   = (float)bw_hz   / 1.0e3f;
    float tcxo_v   = (float)r->slot->tcxo_mv / 1000.0f;

    /* The front end's receive LNA: in the path by default, switchable out on
     * the one part that allows it (~8 mA less while listening, ~20 dB less in
     * front of the chip). Read ahead of femBandSelect, whose calibration
     * rebuild takes the receive-gain correction from it. */
    femRxLna(r, storageGetInt(sk(kb, sizeof kb, r->idx, "fem_rx_lna"), 1) != 0);


    /* Point the front end at the band this carrier is on before anything is
     * measured against it: on a dual-band part the ceiling below, and the
     * conversion radioBegin does, are both the band's. radioBegin repeats this
     * for its own callers; it is idempotent. */
    femBandSelect(r, loraFreqIsHighBand(freq_mhz));

    /* LNA boosted RX gain: ~+3 dB sensitivity for ~0.4 mA more RX current. On by
     * default; radioBegin applies it. Read before begin so it takes effect in
     * the same bring-up.
     *
     * Ignored, and the chip's amplifier left off, while an external amplifier
     * is in the RX path (the calibration femBandSelect just rebuilt says so:
     * a positive receive gain). Behind 17..20 dB of front-end gain the chip's
     * own noise figure is divided by that gain before it reaches the system's,
     * so the boost's 3 dB shrinks to a few tenths — not worth its current or
     * the large-signal headroom it gives up. The setting keeps its value: the
     * moment the front end's LNA is bypassed the chip is the front end again
     * and the switch means what it says. */
    bool extAmp = r->cal.rxGainDb > 0;
    r->rxBoostedGain = !extAmp &&
        storageGetInt(sk(kb, sizeof kb, r->idx, "rx_boosted_gain"), 1) != 0;

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
    csmaNoiseFloorReset(r);

    /* Translate this chip's IRQ bits once, before anything reads flags. */
    radioIrqCache(r);
    /* How long the modem's own reception evidence may stand before it is stale.
     * A preamble has until the header it announces should have arrived — the
     * preamble itself plus the sync word and start-of-frame delimiter, doubled
     * for latency, and floored so a fast configuration still gives the header a
     * usable window. The packet stage's budget is set beside the transmit
     * watchdog below, from the same full-frame airtime. */
    uint32_t preambleMs = (uint32_t)(2.0 * tSymMs * (double)(preamble + 8)) + 1;
    if (preambleMs < 20) preambleMs = 20;
    r->rxPreambleTicks = pdMS_TO_TICKS(preambleMs);
    r->rxActiveStart   = 0;
    r->rxHeaderSeen    = false;

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
    /* A validated header commits the modem to the longest frame it could still
     * be receiving; past that the bit is stale rather than a reception. */
    r->rxPacketTicks = pdMS_TO_TICKS((uint32_t)(maxToa * 1000.0) + 50);

    /* Analog front-end recalibration beat. This is the one periodic wake this
     * task holds without a consumer asking for it, and it is here because the
     * failure it prevents is silent: an SX126x whose gain control has latched
     * stops hearing, and a node that cannot hear also cannot be woken by the
     * traffic that would otherwise trigger a repair. The cost is one wake per
     * interval plus a few ms of chip work; 0 turns it off. */
    r->agcResetMs = (uint32_t)storageGetInt(sk(kb, sizeof kb, r->idx, "agc_reset"),
                                            LORA_AGC_RESET_DEF_S) * 1000u;
    r->agcNext    = xTaskGetTickCount() + pdMS_TO_TICKS(r->agcResetMs);

    /* Mode for RNS iface registration. LoRa defaults to access_point (edge
     * segment); see straddle.yaml for why full/gateway are opt-in-by-hand. */
    char mode[24] = "access_point";
    storageGetStr(sk(kb, sizeof kb, r->idx, "mode"), mode, sizeof(mode), "access_point");
    r->curMode    = modeFromString(mode);
    r->curBitrate = computeBitrate(sf, bw_hz, cr, preamble);
    r->cfgSf = sf; r->cfgBwHz = bw_hz; r->cfgCr = cr; r->cfgPreamble = preamble;
    r->cfgFreqHz = (uint32_t)freq_hz;
    r->airPreamble = preamble; r->airImplicit = false; r->airSf = (uint8_t)sf;
    r->airBwHz = bw_hz;
    r->chNow   = LORA_CH_HAIL;
    /* The config slider ranges over what this board actually reaches at the
     * antenna connector on the band in use, both ends of it. The floor is not
     * a formality: a board whose amplifier cannot be driven below its own
     * output has a quietest transmission, and asking for less than that is
     * asking for something no setting produces. Clamp what the user asked for
     * to the range and say so, rather than accepting a number the hardware
     * will silently ignore. */
    if (txp > r->maxTxDbm) {
        /* Name where the ceiling came from. On a board with no TX_CAL entry it
         * is not a fact about this board at all — it is the part's register
         * ceiling less the margin chipHi() holds back — and reading it as a
         * measurement sends somebody looking for a hardware limit that was
         * never characterised. */
        warn("lora/%d tx_power %d dBm exceeds the %d dBm ceiling (%s) — clamped",
             r->idx, txp, r->maxTxDbm,
             r->cal.grade == CAL_NONE ? "uncalibrated: the part's range, not this board's"
                                      : rfCalName(r->cal.grade));
        txp = r->maxTxDbm;
    }
    if (txp < r->minTxDbm) {
        warn("lora/%d tx_power %d dBm is under this board's %d dBm floor — clamped",
             r->idx, txp, r->minTxDbm);
        txp = r->minTxDbm;
    }
    r->cfgTxp = (int8_t)txp;
    r->cfgSync = (uint8_t)syncWord;
    /* What the register will be on once radioBegin programs it — the setting's
     * own output, not the request, so nothing announces a power before the
     * first frame that a frame would then contradict. */
    r->txPwrNow = rfAntennaDbm(r, rfChipDbm(r, (int8_t)txp));

#if !defined(CONFIG_LORA_NO_SUPE)
    /* The regime in force. The key's value IS the regime number — SUPE's as
     * well as the channel table's, since they are the same quantity. 0 names a
     * regime with no channel plan, which is the same thing as no agility: see
     * the regime table. An unknown number resolves to no agile channels, the
     * safe reading of a value this firmware cannot understand. */
    r->afa = (uint8_t)storageGetInt(sk(kb, sizeof kb, r->idx, "SUPE.afa"), 0);
    /* One interval for everything this node says about itself on this radio:
     * SUPE's ANNOUNCE here, and the Reticulum announces rnsd replays onto this
     * interface on the same beat (rnsdAnnounceBeat, in the task loop). */
    r->annIntervalMin = (uint16_t)storageGetInt(
        sk(kb, sizeof kb, r->idx, "announce_interval"), ANN_INTERVAL_DEF);
#endif

    /* Adaptive TX power. Determinations already made live in the neighbour
     * table, which survives a config cycle, so turning the key back on resumes
     * with what was measured rather than re-probing the mesh. */

#if !defined(CONFIG_LORA_NO_SUPE)
    /* ── SUPE ──
     * The regime is `afa` above; these are the rest. The access-code gate is
     * absolute and comes last: IFAC masks the frame from the flags byte on, so
     * the modem cannot read an address and has nothing to match. An interface
     * with one configured degrades to plain main-channel operation whatever the
     * enable key says, and says so once rather than failing quietly. */
    /* One announce interval. SUPE's own beat and the interface's announce
     * replay are different runs on different timers, but "how often does this
     * node tell the neighbourhood about itself" is one question and deserves
     * one answer; two keys of the same name and the same default were only ever
     * going to be set inconsistently. */
    /* Name ourselves in every START. It costs three bytes and the protocol's
     * default anonymity — a listener learns who is talking to whom, which a
     * Reticulum header never says — and it buys the reverse leg, which cannot
     * exist without it: the tag a START carries is the ANSWERER's address, so
     * an unnamed requester's queued traffic is indistinguishable from anyone
     * else's. Off keeps the anonymity and gives that up (SUPE.md §4). */
    r->supeNameSender = storageGetInt(
        sk(kb, sizeof kb, r->idx, "SUPE.sender_ident"), 1) != 0;
    bool supeWanted = storageGetInt(sk(kb, sizeof kb, r->idx, "SUPE.enable"), 0) != 0;
    bool haveIfac   = storageGetInt(sk(kb, sizeof kb, r->idx, "ifac_size"), 0) != 0;
    /* Two different questions. An access code or an expired dialect means the
     * machinery cannot run at all; the enable key means it runs but does not
     * SPEAK — it still reads announcements, so the picture of the neighbourhood
     * stays current and the switch can be thrown back without waiting to
     * rediscover everyone. That is also what lets a node announce that it does
     * not speak SUPE, which is the only way to tell a neighbour to stop. */
    bool canRun = !haveIfac;
    if (haveIfac)
        info("lora/%d SUPE off: an access code is configured, so the modem cannot "
             "read an address to match", r->idx);
    if (canRun && supeExpired((uint32_t)time(nullptr))) {
        warn("lora/%d SUPE dialect expired — this build stopped speaking it; reflash", r->idx);
        canRun = false;
    }
    r->supeOn = supeWanted && canRun;
    if (canRun && !supeInit(r)) r->supeOn = false;
#endif
    /* After the SUPE switch and outside its guard: the published list is the
     * channels traffic can actually land on, and only a node that speaks SUPE
     * detours off the hailing one — but a build without SUPE still has a
     * hailing channel to name, and the viewers label channel 0 from this. */
    publishChannels(r);
    /* Each store allocates once and keeps its history across config cycles. */
    loraMonInit(r);
    annInit(r);
    peersInit(r);

    /* IFAC: network_name is config (s.), passphrase is a secret (secrets.). */
    storageGetStr(sk(kb, sizeof kb, r->idx, "ifac_netname"), r->curIfacNetname, sizeof(r->curIfacNetname), "");
    {
        char skb[48];
        snprintf(skb, sizeof skb, "s.lora.%d.ifac_netkey", r->idx);
        storageGetStr(skb, r->curIfacNetkey, sizeof(r->curIfacNetkey), "");
    }
    r->curIfacSize = (uint8_t)storageGetInt(sk(kb, sizeof kb, r->idx, "ifac_size"), 0);
    r->curAnnounceCap = (uint8_t)storageGetInt(sk(kb, sizeof kb, r->idx, "announce_cap"), RNS_IFACE_ANNOUNCE_CAP_DEFAULT);
    /* Default 3. This is the expensive edge: this node is custodian of the
     * mesh on the other side of the radio, re-acquiring a neighbour costs
     * ~1.5 s of airtime, and a path response is a signed announce only a node
     * still holding the original bytes can emit — so nodes within the radius
     * are served (stored, answered for, searched for) while a leak from
     * another gateway or the wider network's churn stays out. */
    r->curCommunityRadius = (uint8_t)storageGetInt(sk(kb, sizeof kb, r->idx, "community_radius"), 3);

    storageBegin();
    storageSet(rk(kb, sizeof kb, r->idx, "chip"), chipName(r->slot->chip));
    storageSet(rk(kb, sizeof kb, r->idx, "bitrate_eff"), (int)r->curBitrate);
    /* The same number with its unit, for the settings row that shows it. */
    char bt[32];
    snprintf(bt, sizeof(bt), "%d bit/s", (int)r->curBitrate);
    storageSet(rk(kb, sizeof kb, r->idx, "bitrate_text"), bt);
    storageEnd();

    /* Arm RX and hook the chip's IRQ line (unified API maps to the right DIO). */
    r->radio->setPacketReceivedAction(loraRadioIsr);
    st = radioStartRx(r);
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

/* ─────────────── config reload ─────────────── */

/* Is any radio mid-transmission, or inside a transaction another node is timing
 * against? The apply stops and restarts the radio, so doing it here would cut a
 * frame off the air or strand a detour the far end is still waiting on. The
 * settle window makes this rare — nothing new starts during it — but a frame
 * that began just before the window opened can still be flying, and at SF12 it
 * is flying for seconds. Re-armed a beat later rather than skipped. */
static bool anyRadioOnAir(void) {
    for (int i = 0; i < kNumRadios; i++) {
        LoraRadio* r = &s_radios[i];
        if (!r->running) continue;
        if (r->txActive || r->mtxPhase != MTXP_OFF) return true;
#if !defined(CONFIG_LORA_NO_SUPE)
        if (supeXactLive(r)) return true;
#endif
    }
    return false;
}

/* Which of the pane's conditional rows apply RIGHT NOW — the `when_key` gates
 * the LoRa settings block reads. Each folds two facts into the one truthy/empty
 * value a gate is: what this slot's part can do, and whether the radio is
 * switched on at all. Composed here rather than compared in each UI, which is
 * the same rule as the state wording and the traffic line.
 *
 * A control that does nothing is worse than an absent one — it gets read as an
 * explanation for whatever the radio is doing — and a receive control on a
 * radio that is not receiving is exactly that.
 *
 * Which front end sits on the board is SENSED on the pin rather than declared,
 * so the row for its LNA switch can only be gated once femInit has answered.
 * It always has by the time this matters: femInit runs for every slot as the
 * radio task starts, ahead of the cfgArm that brings applyConfig — and
 * applyConfig is the only thing that can make a gate truthy. */
static void loraPublishRowGates(LoraRadio* r) {
    LoraFamily fam = chipFamily(r->slot->chip);
    char cb[48];
    storageBegin();
    storageSet(rk(cb, sizeof cb, r->idx, "row_rx_boost"),
               r->enabled && radioHasRxBoost(fam) ? "1" : "");
    storageSet(rk(cb, sizeof cb, r->idx, "row_agc_reset"),
               r->enabled && radioHasAgcReset(fam) ? "1" : "");
    /* The LNA switch is the KCT8103L's alone: on the GC1109 the LNA cannot
     * leave the path, and a switch that changed nothing would read as the
     * reason the radio hears what it hears. */
    storageSet(rk(cb, sizeof cb, r->idx, "row_fem_lna"),
               r->enabled && r->femType == FEM_KCT8103L ? "1" : "");
    storageEnd();
}

static void applyConfig(LoraRadio* r) {
    char kb[48];
    r->enabled = storageGetInt(sk(kb, sizeof kb, r->idx, "enable"), 0) != 0;
    /* The status-bar pill answers to the SWITCH, not to the beat: turning the
     * last radio off parks the interface task, so a pill left to the beat would
     * stay on screen showing the neighbourhood of a radio that is no longer
     * listening. The pane's conditional rows go the same way — a disabled radio
     * offers none of them. */
    publishPill();
    loraPublishRowGates(r);

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

#if !defined(CONFIG_LORA_NO_SUPE)
/* Set by the SUPE.enable subscription, read on the radio task. */
static volatile bool s_supeEnDirty = false;
#endif

/* "Announce now" presses, one bit per radio. Set by the sentinel (which runs on
 * whichever task wrote the key) and consumed by the radio task, which is the
 * only context allowed to arm SUPE's ANNOUNCE beside it. */
static volatile uint32_t s_annNowMask = 0;

static void onCfgChange(const char* key, const char* /*val*/) {
    /* Two keys under s.lora.<i> that are not radio settings: nothing about the
     * modem depends on either, and sending them through the settle window
     * would take the radio off the air for ten seconds to change a number.
     *
     *   SUPE.enable        — only whether this node SPEAKS the protocol. The
     *                        toggle exists precisely so a node can be taken out
     *                        of SUPE mid-debug without disturbing anything
     *                        else. It has its own subscription below.
     *   announce_interval  — read live by both beats that use it, so there is
     *                        nothing to re-apply. */
    if (key && (strstr(key, ".SUPE.enable") || strstr(key, ".announce_interval"))) return;
    cfgArmSettle();
}

/* "Announce now" button, one sentinel per radio. Flags the radio task rather
 * than acting here: the rnsd side would be safe from any task, but SUPE's
 * ANNOUNCE goes out beside it and only the radio task may arm that. */
static void onAnnounceNow(const char* key, const char* val) {
    if (!val || atoi(val) == 0) return;   /* the edge write's leading 0 is not a press */
    storageUnset(key);
    int idx = 0;
    if (sscanf(key, "lora.%d.announce_now", &idx) != 1) return;
    if (idx < 0 || idx >= kNumRadios) return;
    s_annNowMask = s_annNowMask | (1u << idx);
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

/* Wake the interface task from another task or a callback: its deadlines just
 * changed (a LoRaMon viewer opened, say) and the blocked itsPoll must not sit
 * out a wait computed for the old state. */
void loraNudge(void) {
    if (s_task) xTaskNotifyGive(s_task);
}

/* ─────────────── task ─────────────── */

/* Stats publishing is event-driven, not timed. Every stat is either a cumulative
 * counter or a last-packet reading, so none of them move without a tx/rx event —
 * republishing on a timer just burns battery. So we publish only after a counter
 * changes, and at most once a second (a change inside the 1 s window is deferred
 * to the boundary, then coalesced). RX is IRQ-woken (loraRadioIsr → task notify;
 * DIO1 is a light-sleep wake source) and outbound wakes via ITS, so with nothing
 * pending the task blocks until a real event and the chip light-sleeps. */

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
        /* A raised IRQ line the ISR has not reported: the interrupt is disabled,
         * so nothing will wake us for it. Service on the next pass rather than
         * sleeping beside a radio holding a completed frame. serviceRadio always
         * either consumes the cause or clears it, so this cannot spin.
         *
         * Not while a transmit is in flight: there the line's only meaning is
         * TxDone, which serviceRadio consumes, and anything else on it belongs
         * to the watchdog — whose deadline is below, and which a zero here would
         * spend at full CPU instead of asleep. */
        if (r->running && !r->txActive && radioIrqLinePending(r)) return 0;
        /* A manual tx request just arrived, or its carrier-sense is mid-backoff:
         * service it now / re-sense at slot pace. */
        if (r->mtxReq) return 0;
        /* The recalibration beat, while one is configured. */
        if (r->running && r->enabled && r->agcResetMs) {
            int32_t rem = (int32_t)(r->agcNext - now);
            TickType_t d = rem > 0 ? (TickType_t)rem : 0;
            if (d < soonest) soonest = d;
        }
        if (r->mtxPhase == MTXP_LBT && r->slotTicks < soonest) soonest = r->slotTicks;
        /* Gating and availability are separate conjunctions: an rnode packet is
         * pending without any rnsd handle, so folding the two together would
         * leave it unable to wake the loop. Undecoded client bytes count too —
         * the pump runs on this task. */
#if !defined(CONFIG_LORA_NO_SUPE)
        /* SUPE's own beats: the announce interval, the pre-offer delay and the
         * airtime verdict. Transaction deadlines are the esp_timer's, so
         * nothing here has to hold one. */
        {
            uint32_t d = supeNextDeadlineMs(r);
            if (d != UINT32_MAX) {
                if (d == 0) return 0;
                /* Sub-tick deadlines round UP: a 3 ms deadline slept as zero
                 * ticks is a busy-wait until it passes, not a sleep. */
                TickType_t t = pdMS_TO_TICKS(d);
                if (t == 0) t = 1;
                if (t < soonest) soonest = t;
            }
        }
#endif
        /* Nothing drains during a settings settle window, so nothing here may
         * ask to be woken for it: with LBT off this term returns 0, which would
         * spend the whole window at full CPU instead of asleep. The window's own
         * deadline is the s_cfgPend term at the top. */
        bool outReady = r->running && !r->splitPending && !r->txActive &&
                        !loraCfgQuiet();
        bool outAvail = (r->rnsdHandle >= 0 && itsBytesAvailable(r->rnsdHandle) > 0) ||
                        loraqDepth(&r->q) > 0 ||
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
        /* An announce replay in progress: it takes the channel one frame at a
         * time, so re-sense at slot pace until it drains. */
        if (r->running && r->enabled && r->annReplay && r->slotTicks < soonest)
            soonest = r->slotTicks;
        /* The channel-RSSI beat — held only while a LoRaMon viewer draws it.
         * Held even on a silent channel then, because an idle radio is exactly
         * when the reading means something; but with no viewer the beat (and
         * this wake) is dropped outright, and an idle task sleeps until a real
         * event. A viewer opening nudges the task (loraNudge), and the by-then
         * stale deadline samples on that very pass. */
        if (r->running && r->enabled && loraMonOpen()) {
            int32_t rem = (int32_t)(r->mon.rssiNext - now);
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

static void loraTaskMain(void*) {
    info("[%s] task up (%d radio%s)", TAG, kNumRadios, kNumRadios == 1 ? "" : "s");

    /* No boot barrier here anymore: the RNS orchestrator only calls loraStart()
     * (which spawns this task) after rnsd is up and past its boot window, so the
     * universe is already settled by the time we run. */
    /* Server before client: the first init call sizes this task's shared inbox,
     * and the server's needs are the larger of the two. */
    itsServerInit();
    /* One conn per radio, plus headroom for a connect that is being reaped.
     *
     * A registration whose ack does not arrive in time is not a registration
     * that failed to happen: itsConnect cancels it and returns -1, but the
     * CANCEL is a message rnsd processes when it next runs, and rnsd is exactly
     * the task that was too busy to ack. So for a moment the slot is still
     * occupied by a conn nobody wants, and a retry with no spare slot is
     * refused — "has 1/1 client conns already" — leaving the radio
     * unregistered until something else shakes it loose. Sized to the radios
     * and it happens on the first boot after a flash, when the config write
     * stalls the storage actor past the ack budget. */
    itsClientInit(kNumRadios + 2);
    /* The RNode endpoint. Both net (a TCP client) and the core serial machinery
     * (a serial client) connect here; onRnodeConnect tells them apart by the
     * connect payload's length. maxHandles=1 plus the explicit reject there is
     * what makes the single-session policy hold across both. */
    itsServerPortOpen(RNODE_ITS_PORT, /*packetBased=*/false, /*maxHandles=*/1, 4096, 4096);
    itsServerOnConnect(RNODE_ITS_PORT, onRnodeConnect);
    itsServerOnRecv(RNODE_ITS_PORT, onRnodeRecv);
    itsServerOnDisconnect(RNODE_ITS_PORT, onRnodeDisconnect);
    storageSubscribeChanges("s.lora", onCfgChange);   /* covers the rnode group too */
    /* The second CDC port only exists while the console is on `usb cdc`, so a
     * serial claim for it has to be re-applied when the transport changes. */
    NOW_AND_ON_CHANGE("sys.usb.serial_ports", { (void)key; (void)val; cfgArm(0); });
    for (int i = 0; i < kNumRadios; i++) {                  /* MHz/kHz pane fields */
        char kb[48];
        storageSubscribeChanges(rk(kb, sizeof kb, i, "freq_mhz"), onDisplayChange);
        storageSubscribeChanges(rk(kb, sizeof kb, i, "bw_khz"),   onDisplayChange);
        storageSubscribeChanges(rk(kb, sizeof kb, i, "announce_now"), onAnnounceNow);
    }
#if !defined(CONFIG_LORA_NO_SUPE)
    /* Speaking SUPE or not, applied where it is read rather than through a
     * radio restart, and announced at once: the neighbourhood learns from the
     * frame, not from silence. Flagged here and acted on by the task, since a
     * storage callback runs on whoever wrote the key. */
    for (int i = 0; i < kNumRadios; i++) {
        char kb[48];
        storageSubscribeChanges(sk(kb, sizeof kb, i, "SUPE.enable"),
                                ON_CHANGE { (void)key; (void)val; s_supeEnDirty = true; });
    }
#endif

    /* Construct radio + HAL per slot. The shared SPI bus is brought up
     * idempotently by spi_helper (EspIdfHal::init), so every radio adds
     * its own device on the one bus. begin() is deferred to applyConfig
     * so we only touch RF hardware when a radio is enabled. The board's
     * peripheral power rail (if any) is already up — the buildable owns
     * it (e.g. hw-lilygo-tdeck's tdeckPowerInit), not this interface. */
#if CONFIG_IDF_TARGET_LINUX
    /* The medium comes up before the radios do, so a model's very first
     * mode and carrier reach it. */
    etherStart();
#endif

    for (int i = 0; i < kNumRadios; i++) {
        LoraRadio* r = &s_radios[i];

        /* CONFIG_LORA_SPI_HOST is the peripheral *name* (1=SPI1 2=SPI2/FSPI
         * 3=SPI3), matching the Kconfig prompt and the BOARD_*_SPI_HOST headers.
         * The IDF spi_host_device_t enum is offset by one (SPI1_HOST=0,
         * SPI2_HOST=1, SPI3_HOST=2), so subtract one — a straight cast put LoRa
         * on SPI3 while the board's shared bus (LCD + SD) lived on SPI2, and the
         * two controllers fought over the same pins (blank panel, SD DMA
         * failures, no LoRa TX). Mirrors fs.cpp's SD-host mapping. */
#if CONFIG_IDF_TARGET_LINUX
        /* The bus is a chip model in this process; there is nothing to claim
         * and nothing that can fail to come up. */
        auto* vhal = new VirtualHal(i, r->slot);
        r->hal = vhal;
        r->hal->init();
#else
        const spi_host_device_t loraHost =
            (spi_host_device_t)(CONFIG_LORA_SPI_HOST - 1);
        auto* ehal = new EspIdfHal(loraHost,
                                   CONFIG_LORA_SCK_PIN, CONFIG_LORA_MOSI_PIN,
                                   CONFIG_LORA_MISO_PIN, r->slot->cs);
        r->hal = ehal;
        r->hal->init();
        /* A HAL that did not come up is a radio that cannot be talked to, and
         * building the rest of the stack on it is how a shortage of internal
         * DMA memory (the bus wants descriptors; WiFi and BLE got there first)
         * turns into a panic inside the SPI driver rather than a line in the
         * log. Shelve the radio and carry on: the device is still a device. */
        if (!ehal->ready()) {
            err("lora/%d: SPI bus unavailable — radio disabled", i);
            delete ehal;
            r->hal  = nullptr;
            r->found = 0;      /* absent, as far as the rest of the straddle is concerned */
            continue;
        }
#endif

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
        /* External FEM (PA/LNA/switch), if the board wires one: detect the
         * part or take the board's declared word for it, install its RF-switch
         * table (which supersedes the two-pin form above — a board wires one or
         * the other), and build the calibration everything downstream converts
         * through. Before begin() and before any power is programmed, like
         * setRfSwitchPins. femBandSelect publishes the range and its grade,
         * here and on every begin, so the figures follow the carrier across the
         * band boundary. */
        femInit(r);
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
        if (s_cfgPend && (int32_t)(xTaskGetTickCount() - s_cfgDueTick) >= 0 &&
            anyRadioOnAir()) {
            /* Due, but something is on air. The deadline must MOVE, not merely
             * fail the test: nextDeadline() turns an overdue apply into a
             * zero-length sleep, so leaving it past-due spins the task for as
             * long as the radio stays busy. */
            s_cfgDueTick = xTaskGetTickCount() + pdMS_TO_TICKS(LORA_CFG_ONAIR_RETRY_MS);
        }
        if (s_cfgPend && (int32_t)(xTaskGetTickCount() - s_cfgDueTick) >= 0) {
            s_cfgPend  = false;
            s_settling = false;
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

        /* The chip first, everything else after: a completed RX or TX is the
         * latency-critical event on this task — a SUPE turnaround is timed in
         * single milliseconds — and nothing unbounded may sit between the IRQ
         * wake and the SPI poll. The RNode pump runs after the radios. */
        if (radioIrq)
            for (int i = 0; i < kNumRadios; i++)
                if (s_radios[i].running) serviceRadio(&s_radios[i]);

        /* Decode whatever the RNode client has sent since the last pass. Also
         * driven from onRnodeRecv; this covers the passes a wake for something
         * else brings us through. */
        rnodePump();

        for (int i = 0; i < kNumRadios; i++) {
            LoraRadio* r = &s_radios[i];
            /* Service on a transmit in flight (the TxDone watchdog), and on a
             * DIO1 that is asserted without the ISR having said so — the line is
             * level-triggered, so a raised line with no notification means the
             * interrupt is disabled, and the frame behind it would otherwise sit
             * there unread. The check is a GPIO read, not an SPI transaction. */
            if (r->running && !radioIrq && (r->txActive || radioIrqLinePending(r)))
                serviceRadio(r);

            if (r->splitPending &&
                (int32_t)(xTaskGetTickCount() - r->splitDeadline) >= 0) {
                r->splitPending = false;
                r->splitTimeouts++;
            }
            if (r->running && r->rnsdHandle < 0 && r->enabled) registerWithRnsd(r);
            /* Say who we are on the air, on this radio's own schedule: rnsd
             * replays every hosted destination's announce onto lora/<i>,
             * pinned, and spends no other interface's airtime doing it. SUPE's
             * ANNOUNCE rides the same interval from its own beat (lora_supe),
             * because it is the same question — which is why the key is read
             * here, live, and handed to both rather than re-applied through a
             * radio cycle. */
            {
                char ib[48];
                int iv = storageGetInt(sk(ib, sizeof ib, i, "announce_interval"),
                                       ANN_INTERVAL_DEF);
#if !defined(CONFIG_LORA_NO_SUPE)
                r->annIntervalMin = (uint16_t)(iv > 0 ? iv : 0);
#endif
                char pfx[16];
                snprintf(pfx, sizeof pfx, "lora/%d", i);
                if (s_annNowMask & (1u << i)) {
                    s_annNowMask = s_annNowMask & ~(1u << i);
                    info("lora/%d announce requested", i);
                    rnsdIfaceAnnounceNow(pfx);
#if !defined(CONFIG_LORA_NO_SUPE)
                    supeAnnSoon(r);   /* our own SUPE announcement beside it */
#endif
                } else if (r->running && r->rnsdHandle >= 0) {
                    rnsdAnnounceBeat(&r->annBeatNextMs, iv, pfx);
                }
            }
            peersExpire(r, millis());
            /* A long stay on one channel is drawn as it accrues rather than
             * only when it ends, so a viewer opened mid-session sees where the
             * radio is now and not a lane that fills in later. Radio task, like
             * every other writer of dwellSince.
             *
             * The watch edge is taken here for the same reason. A window that
             * has just opened must start recording the stay the radio is ALREADY
             * on — waiting for the next retune leaves the lane veiled until
             * something happens to move the radio, so the background appears to
             * begin at the first traffic rather than at the moment the viewer
             * opened. Dropping the anchor on the edge is what makes the first
             * span start at the window rather than reach back before it. */
            {
                bool watched = loraMonOpen();
                if (watched != r->mon.dwellWatched) {
                    r->mon.dwellWatched = watched;
                    r->mon.dwellSince = 0;
                }
                if (watched && (!r->mon.dwellSince ||
                                (uint32_t)(millis() - r->mon.dwellSince) >= 500u))
                    loraMonDwell(r, millis());
            }
            manualTxPoll(r);    /* CLI tx/tx_psa/tx_prot; holds the radio while active */
            peersForgetPoll(r); /* CLI forget; the table is this task's to edit */
            agcResetPoll(r);    /* front-end recalibration; skips a busy radio */
#if !defined(CONFIG_LORA_NO_SUPE)
            /* The enable switch, applied where it is read. Nothing about the
             * modem changes — only whether this node speaks — so the radio
             * stays on the air, and the announcement goes out at once so the
             * neighbourhood is told rather than left to infer it. */
            if (s_supeEnDirty) {
                s_supeEnDirty = false;
                for (int k = 0; k < kNumRadios; k++) {
                    LoraRadio* rr = &s_radios[k];
                    char kb[48];
                    bool want = storageGetInt(
                        sk(kb, sizeof kb, rr->idx, "SUPE.enable"), 0) != 0;
                    if (!supeMounted(rr) || want == rr->supeOn) continue;
                    rr->supeOn = want;
                    info("lora/%d SUPE %s — announcing it", rr->idx,
                         want ? "enabled" : "disabled");
                    /* The agile lanes come and go with the switch, so the list
                     * the viewers draw from is republished here as well as at
                     * config apply — this path never reaches one. */
                    publishChannels(rr);
                    supeAnnArm(rr);
                }
            }
            supePoll(r);        /* the SUPE beat and the offer's channel access */
#endif
            drainOneOutbound(r);
            /* Last, so every claim on the radio above has already been staked:
             * the sample is only taken if nothing else wanted the chip. */
            rssiSamplePoll(r);
        }

        TickType_t dl = nextDeadline();
        /* Hot-loop detector. A zero deadline is legitimate for a pass or two —
         * work is due right now — but hundreds per second is a stuck condition
         * eating the core (`top` reads 90% on this task in bursts). When it
         * trips, name every input the deadline is computed from, so the branch
         * that pins it at zero can be read straight off the log. */
        {
            static uint32_t zeros = 0, winStartMs = 0, lastWarnMs = 0;
            uint32_t nowMs = millis();
            if (dl == 0) {
                if (!zeros) winStartMs = nowMs;
                zeros++;
                if (zeros >= 500) {
                    if (nowMs - winStartMs <= 1000 &&
                        nowMs - lastWarnMs > 5000) {
                        lastWarnMs = nowMs;
                        LoraRadio* r = &s_radios[0];
#if !defined(CONFIG_LORA_NO_SUPE)
                        warn("lora hot loop: 500 zero-deadline passes in %lu ms: "
                             "supeD=%lu airD=%lu offer=%d ann=%d annSoon=%d "
                             "csma=%d q=%u "
                             "txA=%d split=%d mtx=%d its=%u",
                             (unsigned long)(nowMs - winStartMs),
                             (unsigned long)supeNextDeadlineMs(r),
                             (unsigned long)airtimeNextDeadlineMs(r, nowMs),
                             r->supe ? (int)r->supe->eng.offerArmed : -1,
                             r->supe ? (int)r->supe->annPending : -1,
                             r->supe ? (int)r->supe->annSoonPend : -1,
                             (int)r->csmaPhase, (unsigned)loraqDepth(&r->q),
                             (int)r->txActive, (int)r->splitPending,
                             (int)r->mtxReq,
                             (unsigned)(r->rnsdHandle >= 0
                                        ? itsBytesAvailable(r->rnsdHandle) : 0));
#else
                        warn("lora hot loop: 500 zero-deadline passes in %lu ms: "
                             "csma=%d q=%u txA=%d split=%d mtx=%d its=%u",
                             (unsigned long)(nowMs - winStartMs),
                             (int)r->csmaPhase, (unsigned)loraqDepth(&r->q),
                             (int)r->txActive, (int)r->splitPending,
                             (int)r->mtxReq,
                             (unsigned)(r->rnsdHandle >= 0
                                        ? itsBytesAvailable(r->rnsdHandle) : 0));
#endif
                    }
                    zeros = 0;
                }
            } else zeros = 0;
        }
        itsPoll(dl);
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

/* ── netgraph: what a radio says about itself ──
 *
 * The tail of this radio's `if` line in the node's network-graph record:
 * `<MHz>|<SF>|<kHz>|<CR>` and `|s` when SUPE is on. CONFIGURATION ONLY — never
 * RSSI, never a negotiated budget, never a counter. Those move constantly, and
 * a record that moved with them would keep every digest in the community
 * permanently mismatched; the test for a field is whether a change to it
 * deserves waking the whole mesh, and a spreading factor does.
 *
 * netgraph is an optional straddle, so the whole contribution compiles away
 * when it is not in the build — a radio has nothing to say about itself if
 * nobody is composing a record. */
#if CONFIG_STRADDLE_NETGRAPH
static size_t loraNetgraphDetail(const char* iface_name, char* out, size_t outsz) {
    int idx = -1;
    const char* slash = strchr(iface_name, '/');
    if (slash) idx = atoi(slash + 1);
    if (idx < 0 || idx >= kNumRadios) return 0;

    char kb[48], freq[24], bw[24];
    storageGetStr(rk(kb, sizeof kb, idx, "freq_mhz"), freq, sizeof freq, "");
    storageGetStr(rk(kb, sizeof kb, idx, "bw_khz"),   bw,   sizeof bw,   "");
    int sf = storageGetInt(sk(kb, sizeof kb, idx, "spreading_factor"), 0);
    int cr = storageGetInt(sk(kb, sizeof kb, idx, "coding_rate"), 0);
    bool supe = storageGetInt(sk(kb, sizeof kb, idx, "SUPE.enable"), 0) != 0;

    int n = snprintf(out, outsz, "%s|%d|%s|%d%s", freq, sf, bw, cr, supe ? "|s" : "");
    if (n < 0) return 0;
    return (size_t)n < outsz ? (size_t)n : outsz - 1;
}
#endif  /* CONFIG_STRADDLE_NETGRAPH */

/* ── RNS lifecycle hooks (registered with the orchestrator; see rnsServiceRegister) ── */
static void loraStart(void) {
    s_stop = false;
    /* Record queue + interface task — they outlive a stop/start cycle. */
    loraMonStart();
    if (!s_task)
        /* 10 KB PSRAM stack: LoRa frame buffers + RadioLib state, plus the
         * inline Ed25519 announce verification of the neighbour table. */
        s_task = spawnTask(loraTaskMain, TAG, 10240, nullptr, 1, CORE_SECONDARY_NO_LCD, STACK_PSRAM);
    else
        xTaskNotifyGive(s_task);   /* un-park the resident task */
}

static void loraStop(void) {
    if (!s_task || s_stop) return;
    s_stop = true;
    xTaskNotifyGive(s_task);   /* break the work loop; the task parks, not deleted */
    for (int i = 0; i < 300 && !s_parked; i++) delay(10);   /* await park */
    if (!s_parked) warn("[%s] stop timed out", TAG);
    /* The interface task blocks on its queue for up to one beat, so give it
     * comfortably more than a beat to notice the flag and park. */
    for (int i = 0; i < 300 && !loraMonParked(); i++) delay(10);
    if (!loraMonParked()) warn("[%s-if] stop timed out", TAG);
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
        /* How often this radio says who it is — not a SUPE key: it paces the
         * Reticulum announces rnsd replays onto this interface whether or not
         * SUPE is compiled in, and ANNOUNCE when it is. Radio 0's copy comes
         * from the pane row; radios 1.. are seeded here. */
        for (int i = 1; i < kNumRadios; i++)
            storageDefault(sk(kb, sizeof kb, i, "announce_interval"), ANN_INTERVAL_DEF);
#if !defined(CONFIG_LORA_NO_SUPE)
        /* Frequency agility: the regime number, 0 = none. Radio 0's copy comes
         * from the pane row; radios 1.. are seeded in the loop below. */
        for (int i = 1; i < kNumRadios; i++)
            storageDefault(sk(kb, sizeof kb, i, "SUPE.afa"), 0);
#endif
#if !defined(CONFIG_LORA_NO_SUPE)
        /* SUPE. Everything under the prefix defaults **on**, and `enable` is the
         * single thing that is off: a feature nobody can find the switch for is
         * a feature nobody uses, so the only decision to make is whether to
         * speak SUPE at all. Radio 0's copies come from the pane rows.
         *
         * The regime is not here. It is `afa`, the interface's own
         * frequency-agility key, because the regime IS the statement of what is
         * permissible on which channels and a second key would be a second
         * answer to one question. */
        for (int i = 1; i < kNumRadios; i++)
            storageDefault(sk(kb, sizeof kb, i, "SUPE.enable"), 0);
        /* The regime is SUPE's own, so its key sits with the rest of SUPE's;
         * `afa` on its own was the interface-level name it had before. */
        for (int i = 0; i < kNumRadios; i++) {
            int oldAfa = storageGetInt(sk(kb, sizeof kb, i, "afa"), -1);
            if (oldAfa >= 0) {
                storageSet(sk(kb, sizeof kb, i, "SUPE.afa"), oldAfa);
                storageDeleteTree(sk(kb, sizeof kb, i, "afa"));
            }
        }
#endif  /* CONFIG_LORA_NO_SUPE */
        /* RNode endpoint. One endpoint for the device, so the group is global
         * rather than per radio. One switch per door (see lora_rnode.cpp):
         * serial and Bluetooth default on — dormant doors cost nothing — TCP
         * defaults off, since a network listener is a decision. There is no
         * master switch, and `.tcp` is a switch rather than a port number:
         * 7633 is hardcoded in every stock client, so the number carried no
         * information. Values from the earlier shapes of these keys are
         * folded in: a port number in `.tcp` (with the endpoint enabled)
         * becomes the switch, a port number in `.serial` is discarded, and
         * the master `.enable` is deleted. */
        {
            int oldEn  = storageGetInt("s.lora.rnode.enable", -1);
            int oldTcp = storageGetInt("s.lora.rnode.tcp", -1);
            if (oldTcp > 1) storageSet("s.lora.rnode.tcp",
                                       (oldEn != 0 && oldTcp == 7633) ? 1 : 0);
            storageDeleteTree("s.lora.rnode.enable");
            int oldSer = storageGetInt("s.lora.rnode.serial", -1);
            if (oldSer < 0 || oldSer > 1) storageDeleteTree("s.lora.rnode.serial");
        }
        storageDefault("s.lora.rnode.radio",  0);
        storageDefault("s.lora.rnode.serial", 1);
        storageDefault("s.lora.rnode.tcp",    0);
        storageDefault("s.lora.rnode.upnp",   0);
        storageDefault("s.lora.rnode.ble",    1);
        for (int i = 1; i < kNumRadios; i++) {
            storageDefault(sk(kb, sizeof kb, i, "enable"), 0);
            storageDefault(sk(kb, sizeof kb, i, "mode"), "access_point");
            storageDefault(sk(kb, sizeof kb, i, "bandwidth"), 125000);     /* 125 kHz */
            storageDefault(sk(kb, sizeof kb, i, "spreading_factor"), 7);   /* SF7 */
            storageDefault(sk(kb, sizeof kb, i, "coding_rate"), 5);        /* 4/5 */
            storageDefault(sk(kb, sizeof kb, i, "preamble"), 12);
            storageDefault(sk(kb, sizeof kb, i, "sync_word"), "0x42");
        }
        /* Transmit power is not a switch: every power this node derives rests on
         * one the PEER stated, which only a SUPE node ever does, so there is
         * nothing for a setting to govern. The key goes rather than sitting in
         * storage answering a question nobody asks. */
        for (int i = 0; i < kNumRadios; i++)
            storageDeleteTree(sk(kb, sizeof kb, i, "SUPE.adaptive_txpower"));
        storageSet("s.lora.version", LORA_VERSION);
        storageEnd();
    }

    /* Bind each radio to its board slot here, not at task start: the CLI and
     * the settings pane read `s_radios[]` from init onwards, while the task
     * that constructs the hardware only spawns when the RNS orchestrator
     * reaches the interface phase. Pure board data — no hardware is touched.
     * rnsdHandle likewise, since a zeroed one reads as a live handle 0. */
    for (int i = 0; i < kNumRadios; i++) {
        s_radios[i].idx        = i;
        s_radios[i].slot       = &kSlots[i];
        s_radios[i].rnsdHandle = -1;
        /* The SF field's lower bound, so the pane offers what this part can
         * actually run rather than the range the standard defines. A board
         * fact, known from the slot alone, so it is published here rather than
         * waiting for the radio to come up — the pane is readable long before
         * that. */
        char cb[48];
        storageSet(rk(cb, sizeof cb, i, "sf_min"),
                   (int)radioMinSf(chipFamily(kSlots[i].chip)));
        loraPublishRowGates(&s_radios[i]);
    }

    /* Seed the ephemeral MHz/kHz display keys up front, so the settings pane
     * shows current values before the radio task's rns.ready barrier lifts. */
    for (int i = 0; i < kNumRadios; i++) loraPublishDisplay(i);

    cliRegisterCmd("lora", cliLora);

    rnsdPillColor("lora", LORA_PILL_COLOR, LORA_PILL_ORDER, LORA_PILL_TITLE);

    /* Contribute this class's configuration to the network-graph record. We
     * hand over FIELDS; netgraph composes the line and we never see a record —
     * the same division of labour as rnsdPillSet one layer up. */
#if CONFIG_STRADDLE_NETGRAPH
    netgraphContributeIface("lora", loraNetgraphDetail);
#endif

    /* Register with the RNS orchestrator instead of self-spawning: rnsStart()
     * calls loraStart() (which spawns loraTaskMain) once rnsd is up and past its
     * boot window, and rnsStop() calls loraStop(). The larger 10 KB PSRAM stack
     * is for the LoRa frame buffers + RadioLib state machine + the neighbour
     * table's inline announce verification; core placement puts the driver
     * opposite rnsd on a no-LCD build so their RX/TX bursts overlap and both
     * cores idle together for light sleep. */
    rnsServiceRegister(TAG, loraStart, loraStop, RNS_PHASE_IFACE);
}

bool loraPeerSummary(int radio, lora_peer_summary* out) {
    if (radio < 0 || radio >= kNumRadios) return false;
    LoraRadio* r  = &s_radios[radio];
    NeiState*  st = r->nei;
    if (!st) return false;   /* radio has never been up */
    out->peers = out->links = 0;
    for (int k = 0; k < NEI_MAX; k++) {
        Neighbor* e = &st->nei[k];
        if (e->used && !e->isUs && !e->isRnode) out->peers++;
    }
    for (int k = 0; k < NEI_LINKS_MAX; k++)
        if (st->links[k].used) out->links++;
    out->rssi  = (int)r->rssiLast;
    out->snr10 = (int)(r->snrLast * 10.0f);
    return true;
}

static int tagHexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

void loraNameForTag(int radio, const char* tag, char* out, size_t outLen) {
    if (!out || outLen == 0) return;
    out[0] = '\0';
    if (radio < 0 || radio >= kNumRadios || !tag) return;
    /* Six hex characters to three bytes. Anything else is not a tag, and
     * guessing at a malformed one would answer with somebody's name. */
    uint8_t want[3];
    for (int i = 0; i < 3; i++) {
        int hi = tagHexVal(tag[i * 2]), lo = tagHexVal(tag[i * 2 + 1]);
        if (hi < 0 || lo < 0) return;
        want[i] = (uint8_t)((hi << 4) | lo);
    }
    if (tag[6]) return;
    NeiState* st = s_radios[radio].nei;
    if (!st) return;
    /* The same set of prefixes that resolves an address to a node — a
     * destination, a link it opened, the identity behind either, or the bare
     * node key a SUPE announcement left. */
    int num = 0;
    for (int k = 0; k < NEI_MAX; k++) {
        Neighbor* e = &st->nei[k];
        if (!e->used || peersIsLocal(e)) continue;
        num++;                              /* counted as `lora n` counts */
        bool hit = e->haveNode4 && memcmp(e->node4, want, 3) == 0;
        for (int d = 0; d < e->nDests && !hit; d++)
            if (memcmp(e->dests[d].hash, want, 3) == 0) hit = true;
        for (int l = 0; !hit; l++) {
            uint8_t h4[4];
            if (!peersHashAt(st, e, l, h4)) break;
            if (memcmp(h4, want, 3) == 0) hit = true;
        }
        for (int q = 0; q < e->nIds && !hit; q++)
            if (memcmp(e->ids[q], want, 3) == 0) hit = true;
        if (!hit) continue;
        peersNodeNames(e, out, outLen);
        /* A node nobody has named is still a node you can refer to: the number
         * `lora n` prints beside it. The two surfaces then agree, so a bar on
         * the graph and a line in the console are recognisably the same peer. */
        if (!out[0]) snprintf(out, outLen, "#%d", num);
        return;
    }
}

#else  /* ── no radios configured (CONFIG_LORA_COUNT = 0) ── */

void LoraService::onInit() {
    /* iface-lora staged but inert: no LoRa pins configured for this board.
     * RadioLib links out; set CONFIG_LORA_COUNT and the pins to enable.
     *
     * The colour is still published. A board with no radio at all still draws
     * the community's LoRa links on its network graph, and they are still LoRa. */
    rnsdPillColor("lora", LORA_PILL_COLOR, LORA_PILL_ORDER, LORA_PILL_TITLE);
}

bool loraPeerSummary(int, lora_peer_summary*) { return false; }
void loraNameForTag(int, const char*, char* out, size_t outLen) {
    if (out && outLen) out[0] = '\0';
}

#endif
