/* lora_fem.cpp — external RF front-end module support. See lora_fem.h. */

#include "lora_fem.h"

#include "driver/gpio.h"

#include "compat.h"
#include "esp_idf_hal.h"
#include "log.h"
#include "lora_priv.h"

#if defined(CONFIG_LORA0_CS_PIN)

/* Per-dBm TX gain of each FEM, indexed by chip drive 0..21 dBm: gain
 * compresses as the PA saturates. Ported from Meshtastic's LoRaFEMInterface
 * (their measured values for these parts). antenna = chip + gain[chip]. */
static const uint8_t kGainGc1109[22] = {
    11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
    11, 11, 11, 11, 11, 10, 10,  9,  9,  8,  7 };
static const uint8_t kGainKct8103l[22] = {
    13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
    13, 13, 13, 12, 12, 11, 11, 10,  9,  8,  7 };

const char* femName(LoraFemType t)
{
    switch (t) {
    case FEM_GC1109:   return "GC1109";
    case FEM_KCT8103L: return "KCT8103L";
    case FEM_FIXED:    return "fixed";
    default:           return "none";
    }
}

/* RF-switch tables. Both FEMs switch the same way — enable low in standby,
 * enable high + direction low for RX (KCT8103L: LNA path), both high for TX —
 * they just use a different board pin for the direction line, so the pin
 * array is built at runtime after detection. RadioLib applies the row on
 * every mode transition, which is what keeps the scattered
 * standby()/startReceive()/startTransmit() call sites out of this file. */
static const Module::RfSwitchMode_t kFemModeTable[] = {
    { Module::MODE_IDLE, { EspIdfHal::LEVEL_LOW,  EspIdfHal::LEVEL_LOW  } },
    { Module::MODE_RX,   { EspIdfHal::LEVEL_HIGH, EspIdfHal::LEVEL_LOW  } },
    { Module::MODE_TX,   { EspIdfHal::LEVEL_HIGH, EspIdfHal::LEVEL_HIGH } },
    { Module::MODE_END_OF_TABLE, {} },   /* RadioLib's END_OF_MODE_TABLE macro,
                                          * spelled out — the macro is bare and
                                          * its documented Module:: form doesn't
                                          * parse */
};

void femInit(LoraRadio* r)
{
    const LoraSlot* s = r->slot;
    r->femType   = FEM_NONE;
    r->maxTxDbm  = 22;   /* bare SX126x ceiling */

    /* A fixed FEM first: declared, never detected. There is nothing to sense
     * (no enable net) and usually nothing to switch (the chip's own DIO2
     * drives the RF switch on such boards — Station G2), so the declaration
     * alone sets the type and the antenna ceiling. A slot declaring BOTH
     * forms is a board definition contradicting itself; the fixed one wins,
     * because acting on a detect sense that hardware cannot answer would
     * read garbage. */
    if (s->fem_fixed_gain_db > 0) {
        if (s->fem_en >= 0)
            err("lora/%d FEM declared fixed AND given a detect pin — the two are "
                "mutually exclusive; taking the fixed FEM", r->idx);
        /* A fixed FEM may still hang off a firmware-owned rail (fixed means
         * not-detected, not rail-less — the G2's rail happens to be USB-PD
         * hardware, but the next board's may not be). Same drive + light-
         * sleep exemption as the detected path. */
        if (s->fem_pwr >= 0) {
            gpio_config_t pwr = {};
            pwr.pin_bit_mask = 1ULL << s->fem_pwr;
            pwr.mode         = GPIO_MODE_OUTPUT;
            gpio_config(&pwr);
            gpio_set_level((gpio_num_t)s->fem_pwr, 1);
            gpio_sleep_sel_dis((gpio_num_t)s->fem_pwr);
        }
        /* No switch table of ours — but SOMETHING must steer the antenna. A
         * fixed-FEM slot with neither the chip's DIO2 nor RFSW pins declared
         * would transmit a high-power PA into a switch parked for RX; say so
         * at boot rather than let the smoke say it later. */
        if (!s->dio2_rf_switch && s->rfsw_rx < 0 && s->rfsw_tx < 0)
            err("lora/%d fixed FEM declared but no RF switch path is configured "
                "(no DIO2_RF_SWITCH, no RFSW pins) — TX would drive the PA into "
                "a parked switch; check the board's kconfig", r->idx);
        r->femType  = FEM_FIXED;
        r->maxTxDbm = CONFIG_LORA_TX_POWER_MAX;   /* the board's declared FEM ceiling */
        info("lora/%d FEM: fixed +%d dB, declared (max %d dBm at antenna, "
             "chip drive capped at %d dBm)",
             r->idx, s->fem_fixed_gain_db, r->maxTxDbm, s->fem_max_chip_dbm);
        return;
    }

    if (s->fem_en < 0) return;

    /* Rail first: the enable net's pull-up (the detection signal) is powered
     * from the FEM side, so the sense below reads garbage on a dead rail. */
    if (s->fem_pwr >= 0) {
        gpio_config_t pwr = {};
        pwr.pin_bit_mask = 1ULL << s->fem_pwr;
        pwr.mode         = GPIO_MODE_OUTPUT;
        gpio_config(&pwr);
        gpio_set_level((gpio_num_t)s->fem_pwr, 1);
        gpio_sleep_sel_dis((gpio_num_t)s->fem_pwr);   /* rail holds through light sleep */
        delay(5);   /* rail settle before the sense */
    }

    /* Sense the enable net as a floating input: pulled high → KCT8103L board
     * design, floating low → GC1109. */
    gpio_config_t en = {};
    en.pin_bit_mask = 1ULL << s->fem_en;
    en.mode         = GPIO_MODE_INPUT;
    gpio_config(&en);
    delay(1);
    bool kct = gpio_get_level((gpio_num_t)s->fem_en) != 0;

    int txsel = kct ? s->fem_txsel_b : s->fem_txsel_a;
    if (txsel < 0) {
        err("lora/%d FEM detected (%s) but its TX-select pin is not wired — FEM disabled",
            r->idx, kct ? "KCT8103L" : "GC1109");
        return;
    }
    r->femType  = kct ? FEM_KCT8103L : FEM_GC1109;
    r->maxTxDbm = CONFIG_LORA_TX_POWER_MAX;   /* the board's declared FEM ceiling */

    /* RadioLib owns the enable + direction pins from here: it pinModes them at
     * begin() and applies kFemModeTable on every mode change. The light-sleep
     * exemption survives its gpio_config (sleep-sel is a separate register
     * bit), and matters: a radio listening across light sleep needs the FEM
     * held in the RX row, not floating. */
    static uint32_t pinsArr[LORA_NUM_RADIOS][Module::RFSWITCH_MAX_PINS];
    uint32_t (&pins)[Module::RFSWITCH_MAX_PINS] = pinsArr[r->idx];
    pins[0] = (uint32_t)s->fem_en;
    pins[1] = (uint32_t)txsel;
    for (size_t i = 2; i < Module::RFSWITCH_MAX_PINS; i++) pins[i] = RADIOLIB_NC;
    r->mod->setRfSwitchTable(pins, kFemModeTable);
    gpio_sleep_sel_dis((gpio_num_t)s->fem_en);
    gpio_sleep_sel_dis((gpio_num_t)txsel);

    info("lora/%d FEM: %s (max %d dBm at antenna)",
         r->idx, femName((LoraFemType)r->femType), r->maxTxDbm);
}

int8_t femChipDbm(LoraRadio* r, int8_t antennaDbm)
{
    const uint8_t* gain;
    switch (r->femType) {
    case FEM_GC1109:   gain = kGainGc1109;   break;
    case FEM_KCT8103L: gain = kGainKct8103l; break;
    case FEM_FIXED: {
        /* Flat declared gain, no compression table — the board states its
         * conduction figures directly. The top clamp is the safety line: an
         * always-in-path PA's input tolerance (Station G2: P1dB region ends
         * at chip 16 dBm, absolute cap 19) sits far below the chip's own 22,
         * and overdriving it is out of spec however the request came about.
         * Arithmetic in int, not int8_t: antenna −9 minus a 20 dB gain is
         * −29 before the clamp, and an int8 sum has no business wrapping. */
        int cap = r->slot->fem_max_chip_dbm;
        if (cap > 22) cap = 22;              /* SX126x's 22 as the backstop; a
                                              * lower-ceiling chip (SX1261 15,
                                              * SX128x 13) is not bound here —
                                              * RadioLib rejects the drive at
                                              * setOutputPower, radio down, so
                                              * declare fem_max_chip_dbm to the
                                              * CHIP on such slots */
        int chip = (int)antennaDbm - r->slot->fem_fixed_gain_db;
        if (chip > cap) chip = cap;
        if (chip < -9)  chip = -9;
        return (int8_t)chip;
    }
    default:           return antennaDbm;
    }
    /* Walk chip drive upward until chip + gain reaches the request (gain
     * compresses, so the sum is monotonic); top out at chip 21 dBm. */
    for (int chip = 0; chip < 22; chip++) {
        if (chip + gain[chip] >= antennaDbm || chip == 21) {
            int8_t out = (int8_t)(antennaDbm - gain[chip]);
            if (out > 21)  out = 21;
            if (out < -9)  out = -9;
            return out;
        }
    }
    return antennaDbm;   /* unreachable */
}

#endif  /* CONFIG_LORA0_CS_PIN */
