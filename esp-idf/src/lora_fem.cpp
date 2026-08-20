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

/* What the chip itself will accept, per port. The LR2021 is the part that makes
 * the two differ enough to matter: -9..+22 dBm on its sub-GHz port and
 * -19..+12 on its 2.4 GHz one, so a drive figure legal on one is refused
 * outright on the other. The floors bound femChipDbm, the ceilings are what a
 * board with no front end can reach. */
#define LORA_CHIP_MAX_DBM_LF   22
#define LORA_CHIP_MIN_DBM_LF   (-9)
#define LORA_CHIP_MAX_DBM_HF   12
#define LORA_CHIP_MIN_DBM_HF   (-19)

const char* femName(LoraFemType t)
{
    switch (t) {
    case FEM_GC1109:   return "GC1109";
    case FEM_KCT8103L: return "KCT8103L";
    case FEM_DECLARED: return "declared";
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

/* One supply gate, active high. -1 is "this board has no such gate", which is
 * the single-band case for the 2.4 GHz one and the no-FEM case for both. */
static void femDriveGate(int pin, int level)
{
    if (pin < 0) return;
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << pin;
    cfg.mode         = GPIO_MODE_OUTPUT;
    gpio_config(&cfg);
    gpio_set_level((gpio_num_t)pin, level);
    gpio_sleep_sel_dis((gpio_num_t)pin);   /* a radio listening across light sleep
                                            * needs its LNA to stay powered */
}

void femBandSelect(LoraRadio* r, bool highBand)
{
    const LoraSlot* s = r->slot;
    r->highBand = highBand;

    /* The ceiling is the band's, always: on a board with no front end at all
     * that is the bare chip's own maximum for the port in use, which is not
     * the same number on the two ports. */
    if (r->femType == FEM_NONE)
        r->maxTxDbm = highBand ? LORA_CHIP_MAX_DBM_HF : LORA_CHIP_MAX_DBM_LF;
    else
        r->maxTxDbm = highBand ? CONFIG_LORA_TX_POWER_MAX_HF : CONFIG_LORA_TX_POWER_MAX;

    /* Only a board that named a second gate has two front ends to choose
     * between; anywhere else the one gate stays as femInit left it. */
    if (s->fem_hf_pwr >= 0) {
        femDriveGate(s->fem_pwr,    highBand ? 0 : 1);
        femDriveGate(s->fem_hf_pwr, highBand ? 1 : 0);
    }

    /* What this radio can actually reach at the antenna, republished on every
     * begin so a UI sizing its power control from the key follows the carrier
     * across the band boundary instead of offering a figure from the other
     * port. */
    char b[48];
    storageSet(rk(b, sizeof b, r->idx, "tx_power_max"), (int)r->maxTxDbm);
}

void femInit(LoraRadio* r)
{
    const LoraSlot* s = r->slot;
    r->femType   = FEM_NONE;
    r->maxTxDbm  = LORA_CHIP_MAX_DBM_LF;   /* bare chip, sub-GHz; femBandSelect refines it */

    /* A declared front end has nothing to detect and nothing to drive: its
     * control lines are the radio's own DIOs, programmed from the board's
     * LORAn_LR_RFSW_* masks once the chip answers (lora_radio.cpp). All this
     * has to do is take the board at its word about what sits in front of the
     * antenna, which is what makes femChipDbm hand the part a drive level it
     * survives. It wins over the pin group below — a board states one or the
     * other, never both. */
    if (s->fem_gain_db > 0 || s->fem_hf_gain_db > 0) {
        r->femType  = FEM_DECLARED;
        /* Both supply gates start up, matching the pull-ups these nets carry on
         * every board seen so far: the band is not known until the first
         * begin(), and a front end powered a moment too long costs current
         * while one powered a moment too late costs the first transmit. The
         * first femBandSelect narrows it to the port actually in use. */
        femDriveGate(s->fem_pwr, 1);
        femDriveGate(s->fem_hf_pwr, 1);
        femBandSelect(r, false);   /* sub-GHz until a carrier says otherwise */
        if (s->fem_hf_gain_db > 0)
            info("lora/%d FEM: declared, %d dB sub-GHz (max %d dBm) / %d dB at 2.4 GHz "
                 "(max %d dBm), supply gates %d/%d",
                 r->idx, s->fem_gain_db, CONFIG_LORA_TX_POWER_MAX,
                 s->fem_hf_gain_db, CONFIG_LORA_TX_POWER_MAX_HF,
                 s->fem_pwr, s->fem_hf_pwr);
        else
            info("lora/%d FEM: declared, %d dB (max %d dBm at antenna, %d dBm at the chip)",
                 r->idx, s->fem_gain_db, r->maxTxDbm, r->maxTxDbm - s->fem_gain_db);
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
    /* The port in use decides what the chip will take. One dB under the
     * ceiling on the sub-GHz side is the long-standing margin against a part
     * that refuses its own stated maximum; the 2.4 GHz numbers are the chip's
     * exactly, because that range is narrow enough that giving one away is
     * giving away a tenth of it. */
    const int hiClamp = r->highBand ? LORA_CHIP_MAX_DBM_HF : LORA_CHIP_MAX_DBM_LF - 1;
    const int loClamp = r->highBand ? LORA_CHIP_MIN_DBM_HF : LORA_CHIP_MIN_DBM_LF;

    const uint8_t* gain;
    switch (r->femType) {
    case FEM_GC1109:   gain = kGainGc1109;   break;
    case FEM_KCT8103L: gain = kGainKct8103l; break;
    case FEM_DECLARED: {
        /* One figure, applied flat: nobody measured this part per rung, so the
         * honest model is the gain the board states for the port in use. The
         * clamp is what makes the bottom of the range behave — asking for less
         * than the chip's floor plus the gain simply lands on the floor. */
        int g = r->highBand ? r->slot->fem_hf_gain_db : r->slot->fem_gain_db;
        int out = antennaDbm - g;
        if (out > hiClamp) out = hiClamp;
        if (out < loClamp) out = loClamp;
        return (int8_t)out;
    }
    default: {
        /* No front end: the request IS the drive, but the chip's own port
         * limits still apply — a 2.4 GHz carrier cannot be handed +22. */
        int out = antennaDbm;
        if (out > hiClamp) out = hiClamp;
        if (out < loClamp) out = loClamp;
        return (int8_t)out;
    }
    }
    /* Walk chip drive upward until chip + gain reaches the request (gain
     * compresses, so the sum is monotonic); top out at chip 21 dBm. */
    for (int chip = 0; chip < 22; chip++) {
        if (chip + gain[chip] >= antennaDbm || chip == 21) {
            int out = antennaDbm - gain[chip];
            if (out > hiClamp) out = hiClamp;
            if (out < loClamp) out = loClamp;
            return (int8_t)out;
        }
    }
    return antennaDbm;   /* unreachable */
}

#endif  /* CONFIG_LORA0_CS_PIN */
