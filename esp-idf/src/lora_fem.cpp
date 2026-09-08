/* lora_fem.cpp — external RF front-end module support. See lora_fem.h. */

#include "lora_fem.h"

#include "driver/gpio.h"

#include "compat.h"
#include "esp_idf_hal.h"
#include "log.h"
#include "lora_priv.h"

#if defined(CONFIG_LORA0_CS_PIN)

/* Receive gain of each detected part, dB, from its datasheet: the GC1109's LNA
 * runs at 17 dB and the KCT8103L's at 20.0 dB on a 3.3 V rail. Both sit in the
 * receive path — what the GC1109 datasheet calls a low-loss bypass is its
 * TRANSMIT bypass — so a level read at the chip is this much above the level at
 * the connector, and rfRssiDbm takes it back off. A board that wires something
 * else in front says so with LORAn_RSSI_CAL. */
static int8_t partRxGainDb(uint8_t femType)
{
    switch (femType) {
    case FEM_GC1109:   return 17;
    case FEM_KCT8103L: return 20;
    default:           return 0;
    }
}

/* What the chip itself will accept, per port. The LR2021 is the part that makes
 * the two differ enough to matter: -9..+22 dBm on its sub-GHz port and
 * -19..+12 on its 2.4 GHz one, so a drive figure legal on one is refused
 * outright on the other. The floors bound rfChipDbm, the ceilings are what a
 * board with no front end can reach. */
#define LORA_CHIP_MAX_DBM_LF   22
#define LORA_CHIP_MIN_DBM_LF   (-9)
#define LORA_CHIP_MAX_DBM_HF   12
#define LORA_CHIP_MIN_DBM_HF   (-19)

/* One dB under the sub-GHz ceiling is the long-standing margin against a part
 * that refuses its own stated maximum; the 2.4 GHz numbers are the chip's
 * exactly, because that range is narrow enough that giving one away is giving
 * away a tenth of it. */
static inline int chipHi(const LoraRadio* r) {
    return r->highBand ? LORA_CHIP_MAX_DBM_HF : LORA_CHIP_MAX_DBM_LF - 1;
}
static inline int chipLo(const LoraRadio* r) {
    return r->highBand ? LORA_CHIP_MIN_DBM_HF : LORA_CHIP_MIN_DBM_LF;
}

const char* femName(LoraFemType t)
{
    switch (t) {
    case FEM_GC1109:   return "GC1109";
    case FEM_KCT8103L: return "KCT8103L";
    case FEM_DECLARED: return "declared";
    default:           return "none";
    }
}

/* RF-switch tables. Both parts take the same two-pin row — enable low in
 * standby, enable high + select low for RX, both high for TX — but the second
 * pin is not the same signal on the two board designs, so the pin array is
 * built at runtime after detection:
 *
 *   GC1109 board    CSD on the enable pin, CPS on the select pin (0 = transmit
 *                   bypass, 1 = through the PA). The TX/RX direction line CTX
 *                   is wired to the radio's own DIO2, so the row never touches
 *                   it and RX reaches the LNA path on its own.
 *   KCT8103L board  CSD on the enable pin, CTX on the select pin (0 = RX via
 *                   the LNA, 1 = TX). CPS has no MCU connection here.
 *
 * RadioLib applies the row on every mode transition, which is what keeps the
 * scattered standby()/startReceive()/startTransmit() call sites out of this
 * file.
 *
 * The KCT8103L has a second RX row: CTX held at 1 while the chip receives
 * routes the signal round the LNA instead of through it. That is the whole of
 * the LNA-off table — the idle and TX rows are the same — and it is the part's
 * only receive path that does not cost the LNA's ~8 mA. The GC1109 has no such
 * row: its second pin selects the transmit path and its LNA is always in line. */
static const Module::RfSwitchMode_t kFemModeTable[] = {
    { Module::MODE_IDLE, { EspIdfHal::LEVEL_LOW,  EspIdfHal::LEVEL_LOW  } },
    { Module::MODE_RX,   { EspIdfHal::LEVEL_HIGH, EspIdfHal::LEVEL_LOW  } },
    { Module::MODE_TX,   { EspIdfHal::LEVEL_HIGH, EspIdfHal::LEVEL_HIGH } },
    { Module::MODE_END_OF_TABLE, {} },   /* RadioLib's END_OF_MODE_TABLE macro,
                                          * spelled out — the macro is bare and
                                          * its documented Module:: form doesn't
                                          * parse */
};
static const Module::RfSwitchMode_t kFemModeTableLnaOff[] = {
    { Module::MODE_IDLE, { EspIdfHal::LEVEL_LOW,  EspIdfHal::LEVEL_LOW  } },
    { Module::MODE_RX,   { EspIdfHal::LEVEL_HIGH, EspIdfHal::LEVEL_HIGH } },
    { Module::MODE_TX,   { EspIdfHal::LEVEL_HIGH, EspIdfHal::LEVEL_HIGH } },
    { Module::MODE_END_OF_TABLE, {} },
};

/* The pin array each slot's table refers to. RadioLib keeps the pointer, not a
 * copy, so it has to outlive femInit — and femRxLna re-installs it with the
 * other table. */
static uint32_t s_femPins[LORA_NUM_RADIOS][Module::RFSWITCH_MAX_PINS];

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

static void calBuild(LoraRadio* r);   /* fwd — defined with the conversion */

void femBandSelect(LoraRadio* r, bool highBand)
{
    const LoraSlot* s = r->slot;
    r->highBand = highBand;

    /* Only a board that named a second gate has two front ends to choose
     * between; anywhere else the one gate stays as femInit left it. */
    if (s->fem_hf_pwr >= 0) {
        femDriveGate(s->fem_pwr,    highBand ? 0 : 1);
        femDriveGate(s->fem_hf_pwr, highBand ? 1 : 0);
    }

    /* The two ports have different register ranges, different front ends and
     * different curves, so the calibration is rebuilt rather than adjusted.
     * The amplifier goes back in the path first: calBuild reads that state, and
     * the begin() this runs inside resets the chip's DIO map to match. */
    r->femTxPa = true;
    calBuild(r);

    /* Both ends of the range come out of that calibration, so they describe
     * what this board can actually put on the connector rather than what the
     * chip would do on its own. A board that can transmit round its amplifier
     * spans BOTH states — the quiet end is the bypass path's, the loud end the
     * amplifier's — because nothing outside here has to know which one a given
     * power will use. A front end's board rating caps the ceiling; that is a
     * rating, not a measurement, and it is allowed to be the tighter. */
    int mx = rfCalMaxDbm(&r->cal), mn = rfCalMinDbm(&r->cal);
    if (femCanBypassPa(r)) {
        const int byMx = rfCalMaxDbm(&r->calBypass), byMn = rfCalMinDbm(&r->calBypass);
        if (byMx > mx) mx = byMx;
        if (byMn < mn) mn = byMn;
    }
    if (r->femType != FEM_NONE) {
        const int cap = highBand ? CONFIG_LORA_TX_POWER_MAX_HF : CONFIG_LORA_TX_POWER_MAX;
        if (mx > cap) mx = cap;
    }
    r->maxTxDbm = (int8_t)mx;
    r->minTxDbm = (int8_t)mn;

    /* What this radio can actually reach at the antenna, republished on every
     * begin so a UI sizing its power control from the key follows the carrier
     * across the band boundary instead of offering a figure from the other
     * port. The floor is published beside it because on an amplified board it
     * is nowhere near the chip's own: a part that cannot be driven below its
     * PA's output has a quietest transmission, and everything that picks a
     * power has to know it. The grade goes out with them so a reader can tell
     * a characterised board from an assumed one. */
    char b[48];
    storageSet(rk(b, sizeof b, r->idx, "tx_power_max"), (int)r->maxTxDbm);
    storageSet(rk(b, sizeof b, r->idx, "tx_power_min"), (int)r->minTxDbm);
    storageSet(rk(b, sizeof b, r->idx, "cal"),          rfCalName(r->cal.grade));
}

/* Everything that follows from knowing which part is in front of the radio.
 * Every exit from femInit runs it, including the ones that found no front end
 * at all — a bare board still has a range and a conversion, and they are the
 * chip's own. Sub-GHz until a carrier says otherwise. */
static void femSettle(LoraRadio* r) { femBandSelect(r, false); }

void femInit(LoraRadio* r)
{
    const LoraSlot* s = r->slot;
    r->femType   = FEM_NONE;
    r->femTxPa   = true;   /* the amplifier is in the path until a board that can
                            * take it out is told to; femTxPa enforces that */
    r->femRxLna  = true;   /* every wiring but the KCT8103L's receives through
                            * whatever LNA it has; the flag is only ever cleared
                            * by femRxLna on that part */

    /* How much the front end amplifies on RECEIVE is femType's to say: both
     * detected parts put an LNA in the RX path — 17 dB on the GC1109, 20 dB on
     * the KCT8103L at 3.3 V — while a declared one is switched by the radio's
     * DIOs with nothing here able to tell what it does. The GC1109's low-loss
     * bypass is its TRANSMIT bypass, not a receive path. loraPublishRowGates
     * reads femType back for the caption that mentions it.
     *
     * A declared front end has nothing to detect and nothing to drive: its
     * control lines are the radio's own DIOs, programmed from the board's
     * LORAn_LR_RFSW_* masks once the chip answers (lora_radio.cpp). All this
     * has to do is take the board at its word about what sits in front of the
     * antenna, which is what makes rfChipDbm hand the part a drive level it
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
        femSettle(r);
        if (s->fem_hf_gain_db > 0)
            info("lora/%d FEM: declared, %d dB sub-GHz (max %d dBm) / %d dB at 2.4 GHz "
                 "(max %d dBm), supply gates %d/%d",
                 r->idx, s->fem_gain_db, CONFIG_LORA_TX_POWER_MAX,
                 s->fem_hf_gain_db, CONFIG_LORA_TX_POWER_MAX_HF,
                 s->fem_pwr, s->fem_hf_pwr);
        else
            info("lora/%d FEM: declared, %d dB (%d..%d dBm at antenna, cal %s)",
                 r->idx, s->fem_gain_db, r->minTxDbm, r->maxTxDbm,
                 rfCalName(r->cal.grade));
        return;
    }

    if (s->fem_en < 0) { femSettle(r); return; }

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
        femSettle(r);
        return;
    }
    r->femType = kct ? FEM_KCT8103L : FEM_GC1109;
    r->femRxLna = true;   /* the LNA table goes in first; applyConfig reads the switch */
    femSettle(r);

    /* RadioLib owns the enable + direction pins from here: it pinModes them at
     * begin() and applies kFemModeTable on every mode change. The light-sleep
     * exemption survives its gpio_config (sleep-sel is a separate register
     * bit), and matters: a radio listening across light sleep needs the FEM
     * held in the RX row, not floating. */
    uint32_t (&pins)[Module::RFSWITCH_MAX_PINS] = s_femPins[r->idx];
    pins[0] = (uint32_t)s->fem_en;
    pins[1] = (uint32_t)txsel;
    for (size_t i = 2; i < Module::RFSWITCH_MAX_PINS; i++) pins[i] = RADIOLIB_NC;
    r->mod->setRfSwitchTable(pins, kFemModeTable);
    gpio_sleep_sel_dis((gpio_num_t)s->fem_en);
    gpio_sleep_sel_dis((gpio_num_t)txsel);

    info("lora/%d FEM: %s (%d..%d dBm at antenna, rx +%d dB, cal %s)",
         r->idx, femName((LoraFemType)r->femType),
         r->minTxDbm, r->maxTxDbm, r->cal.rxGainDb, rfCalName(r->cal.grade));
}

void femRxLna(LoraRadio* r, bool on)
{
    if (r->femType != FEM_KCT8103L) { r->femRxLna = true; return; }
    if (r->femRxLna == on) return;
    r->femRxLna = on;
    r->mod->setRfSwitchTable(s_femPins[r->idx], on ? kFemModeTable : kFemModeTableLnaOff);
    info("lora/%d FEM: receive LNA %s", r->idx, on ? "in the RX path" : "bypassed");
}

bool femCanBypassPa(const LoraRadio* r)
{
    return r->slot->lr_rfsw_tx_bypass != 0 && r->haveBypassCal && !r->highBand;
}

/* Move the front end's transmit amplifier in or out of the path, and swap the
 * calibration with it — the two are one act, because the board is a different
 * transmitter in each state and a register setting means a different power.
 *
 * Called from apApplyPower, which every transmit path runs immediately before
 * staging its frame, so the chip is on its way to standby and TX anyway. The
 * DIO map is what carries the state; the chip reads it when it enters a mode,
 * so rewriting it before the transmit starts is what makes the frame come out
 * of the path we just costed it against. */
void femTxPa(LoraRadio* r, bool on)
{
    if (!femCanBypassPa(r)) { r->femTxPa = true; return; }
    if (r->femTxPa == on) return;
    r->femTxPa = on;

    LoraRfCal swap = r->cal;
    r->cal = r->calBypass;
    r->calBypass = swap;

    lr2021ApplyDio(r);
    if (logIsDebug(TAG))
        dbg("lora/%d FEM: transmit amplifier %s (%d..%d dBm)", r->idx,
            on ? "in the TX path" : "bypassed",
            rfCalMinDbm(&r->cal), rfCalMaxDbm(&r->cal));
}

/* The state a wanted power should go out through: the bypass path whenever it
 * can reach the power at all, the amplifier only when it cannot. Quiet is the
 * common case on a dense mesh and the bypass path is both quieter and cheaper,
 * so it wins ties. The two ranges do not overlap on any board seen so far — the
 * amplifier's floor sits above the bypass path's ceiling — which makes this a
 * single threshold with nothing to oscillate around. */
bool femWantPa(const LoraRadio* r, int8_t antennaDbm)
{
    if (!femCanBypassPa(r)) return true;
    const LoraRfCal* by = r->femTxPa ? &r->calBypass : &r->cal;
    return antennaDbm > rfCalMaxDbm(by);
}

/* ─────────────── the conversion, both directions ───────────────
 *
 * The arithmetic is lora_rfcal's; this is where it meets the hardware. The
 * curve covers the sub-GHz port only: a dual-band board's 2.4 GHz front end is
 * switched by the radio's own DIOs and states one gain figure, so that port
 * keeps the flat model. Both are `r->cal`, rebuilt by femBandSelect whenever
 * the carrier crosses between them. */

int8_t rfChipDbm(LoraRadio* r, int8_t antennaDbm) {
    return rfCalChip(&r->cal, antennaDbm);
}

int8_t rfAntennaDbm(const LoraRadio* r, int8_t chipDbm) {
    return rfCalAntenna(&r->cal, chipDbm);
}

float rfRssiDbm(const LoraRadio* r, float chipRssi) {
    return rfCalRssi(&r->cal, chipRssi);
}

/* Build the calibration for the part now detected on the port now in use.
 *
 * LORAn_TX_CAL holds one entry per part the board can carry, since a board
 * that ships two front ends across revisions has a different curve for each
 * and only learns which at boot. A part the spec does not mention keeps the
 * flat model at grade none, which is what every uncalibrated board does. */
static void calBuild(LoraRadio* r)
{
    const LoraSlot* s = r->slot;
    rfCalReset(&r->cal, chipLo(r), chipHi(r));

    if (r->femType == FEM_DECLARED)
        r->cal.flatGainDb = (int8_t)(r->highBand ? s->fem_hf_gain_db : s->fem_gain_db);

    /* A board that names its own receive gain outranks the part's datasheet
     * figure; that is what the symbol is for. Neither applies while the LNA is
     * switched out of the path: a level read then is the connector's. */
    r->cal.rxGainDb = !r->femRxLna ? 0
                    : s->rssi_cal  ? (int8_t)s->rssi_cal
                                   : partRxGainDb(r->femType);

    if (r->highBand) return;   /* the 2.4 GHz port keeps the flat model */

    const char* name = femName((LoraFemType)r->femType);
    char why[80];
    if (!rfCalParse(&r->cal, s->tx_cal, name, why, sizeof why) && why[0])
        err("lora/%d TX_CAL %s — uncalibrated", r->idx, why);

    /* With the transmit amplifier out of the path the board is a different
     * transmitter, so it carries a second curve — named `<part>-bypass`, and
     * required before the state can be entered at all: a state with no
     * conversion for it is a state that cannot say what it radiates, which is
     * the one thing this must never do. It starts from the same flat model and
     * receive gain, since only the transmit path moved. */
    r->calBypass = r->cal;
    r->calBypass.n = 0;
    r->calBypass.grade = CAL_NONE;
    r->haveBypassCal = false;
    if (!s->lr_rfsw_tx_bypass) return;

    char part[32];
    snprintf(part, sizeof part, "%s-bypass", name);
    if (rfCalParse(&r->calBypass, s->tx_cal, part, why, sizeof why)) {
        r->haveBypassCal = true;
    } else if (why[0]) {
        err("lora/%d TX_CAL %s — bypass unavailable", r->idx, why);
    } else {
        err("lora/%d names a transmit-bypass mask but TX_CAL has no %s entry — "
            "bypass unavailable", r->idx, part);
    }
}

#endif  /* CONFIG_LORA0_CS_PIN */
