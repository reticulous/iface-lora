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
