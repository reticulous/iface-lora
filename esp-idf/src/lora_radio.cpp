/**
 * lora_radio — chip dispatch and the RadioLib call surface: construction,
 * begin(), the per-chip/per-family setters, channel RSSI, and time-on-air.
 * Knows nothing of Reticulum or SUPE.
 */
#include "lora_priv.h"
#include "lora_fem.h"
#include "lora_toa.h"

#if defined(CONFIG_LORA0_CS_PIN)

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

const char* chipName(LoraChip c) {
    switch (c) {
#define X(name, fam) case CHIP_##name: return #name;
        LORA_CHIPS(X)
#undef X
    }
    return "?";
}

LoraFamily chipFamily(LoraChip c) {
    switch (c) {
#define X(name, fam) case CHIP_##name: return fam;
        LORA_CHIPS(X)
#undef X
    }
    return FAM_SX126X;
}

PhysicalLayer* radioNew(LoraChip c, Module* m) {
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
const char* rlErrName(int16_t st) {
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

/* The PA over-current trip, in mA. RadioLib's SX126x::begin() sets this to 60 mA
 * for every part and setOutputPower() then reads the register and writes it back
 * unchanged, so nothing else ever raises it — an SX1262 asked for +22 dBm draws
 * about 118 mA and trips a limit left at 60. The datasheet's own post-SetPaConfig
 * defaults are the right values: 140 mA for the parts that reach +22 dBm, 60 mA
 * for the SX1261, which tops out at +15 dBm and must not be given a ceiling its
 * PA cannot survive. */
float radioOcpMilliamps(LoraChip c) {
    return c == CHIP_SX1261 ? 60.0f : 140.0f;
}

/* Undocumented SX1262 register bit that Semtech and Heltec both recommend for RX
 * sensitivity; set bit 0 of 0x8B5. Applied through the Module rather than the
 * chip class because SX126x::writeRegister is protected. Re-applied after every
 * recalibration — CALIBRATE_ALL clears it (see radioAgcReset). */
#define LORA_SX126X_RX_SENS_REG   0x8B5

static void sx126xRxSensPatch(LoraRadio* r) {
    if (r->mod->SPIsetRegValue(LORA_SX126X_RX_SENS_REG, 0x01, 0, 0) != RADIOLIB_ERR_NONE)
        warn("lora/%d SX126x RX-sensitivity register patch failed", r->idx);
}

/* The bands SX126x::calibrateImage() has a factory image calibration for,
 * mirroring RadioLib's own table — including its truncation of the frequency to
 * a whole MHz, because a check that disagrees with the call it is describing is
 * worse than no check. A frequency outside every one of them falls back to
 * calibrateImageRejection(freq ± 4 MHz), which the library itself describes as
 * "may or may not work".
 *
 * Landing outside is legal — the part tunes 150-960 MHz — but the common cause
 * is a mistyped frequency, and on the air the two are indistinguishable: the
 * radio comes up, calls itself healthy, reports a quiet noise floor because the
 * band really is empty, and hears nothing anyone says. A digit lost from an
 * 869.475 puts a node on 469.475 with every other setting still matching its
 * neighbours. So the frequency is checked against the same table and named. */
static bool sx126xImageBandKnown(float freqMhz) {
    static const struct { int lo, hi; } bands[] = {
        { 902, 928 }, { 863, 870 }, { 779, 787 }, { 470, 510 }, { 430, 440 },
    };
    int f = (int)freqMhz;
    for (size_t i = 0; i < sizeof bands / sizeof bands[0]; i++)
        if (f >= bands[i].lo && f <= bands[i].hi) return true;
    return false;
}

/* The family's begin(), with nothing applied after it. Split out of radioBegin so
 * the TCXO fallback can run the whole call again with a different argument. */
static int16_t radioBeginOnce(LoraRadio* r, float freq, float bw, uint8_t sf, uint8_t cr,
                              uint8_t sync, int8_t power, uint16_t preamble, float tcxoV) {
    PhysicalLayer* p = r->radio;
    switch (r->slot->chip) {
        case CHIP_SX1261: return static_cast<SX1261*>(p)->begin(freq, bw, sf, cr, sync, power, preamble, tcxoV, false);
        case CHIP_SX1262: return static_cast<SX1262*>(p)->begin(freq, bw, sf, cr, sync, power, preamble, tcxoV, false);
        case CHIP_SX1268: return static_cast<SX1268*>(p)->begin(freq, bw, sf, cr, sync, power, preamble, tcxoV, false);
        case CHIP_LLCC68: return static_cast<LLCC68*>(p)->begin(freq, bw, sf, cr, sync, power, preamble, tcxoV, false);
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
    }
    return RADIOLIB_ERR_UNKNOWN;
}

/* begin() the radio with the common LoRa parameters. Each family's begin() has
 * a different signature: SX126x carries TCXO + regulator; SX127x a LNA-gain arm
 * (0 = AGC) and no TCXO; SX128x is 2.4 GHz and bare; LR11x0 sets freq/power
 * separately (lr11x0Begin); LR2021 takes everything including TCXO. We cast to
 * the concrete class (the pointer really is that class) so dispatch is correct
 * regardless of where each begin() sits in RadioLib's hierarchy.
 *
 * A begin() that fails on the SPI command itself, with a TCXO voltage
 * configured, is the classic symptom of a board whose reference is a plain
 * crystal (or whose DIO3 does not feed the oscillator): the chip is asked to
 * wait for a TCXO that never becomes ready and answers with a command error. The
 * retry on a bare crystal turns that from "radio absent" into a working radio and
 * a warning naming the wrong setting.
 *
 * SX126x parts then take the extras their family needs: DIO2 as the antenna RF
 * switch when the slot asks for it, the LNA boosted-RX-gain option
 * (r->rxBoostedGain, ~+3 dB sensitivity for ~0.4 mA more RX current), the PA
 * over-current trip, and the RX-sensitivity register patch — and the frequency
 * is checked against the image-calibration bands (sx126xImageBandKnown). */
int16_t radioBegin(LoraRadio* r, float freq, float bw, uint8_t sf, uint8_t cr,
                          uint8_t sync, int8_t power, uint16_t preamble, float tcxoV) {
    PhysicalLayer* p = r->radio;
    /* Every begin() rewrites the Rx/Tx fallback register from the driver's own
     * flag, so clear the pair before it runs rather than after: a radio comes up
     * idle, with the reference free to stop between frames, and a chain arms it
     * again (radioHoldOsc) when one starts. */
    if (chipFamily(r->slot->chip) == FAM_SX126X)
        static_cast<SX126x*>(p)->standbyXOSC = false;
    r->oscHeld = false;
    power = femChipDbm(r, power);   /* antenna dBm → chip drive (identity, no FEM) */
    int16_t st = radioBeginOnce(r, freq, bw, sf, cr, sync, power, preamble, tcxoV);
    if (tcxoV > 0.0f &&
        (st == RADIOLIB_ERR_SPI_CMD_TIMEOUT || st == RADIOLIB_ERR_SPI_CMD_INVALID ||
         st == RADIOLIB_ERR_SPI_CMD_FAILED)) {
        int16_t xtal = radioBeginOnce(r, freq, bw, sf, cr, sync, power, preamble, 0.0f);
        if (xtal == RADIOLIB_ERR_NONE) {
            warn("lora/%d %s answered only with the TCXO off — running on the crystal; "
                 "CONFIG_LORA%d_TCXO_MV=%d does not match this board",
                 r->idx, chipName(r->slot->chip), r->idx, r->slot->tcxo_mv);
            st = xtal;
        }
    }
    if (st != RADIOLIB_ERR_NONE) return st;
    if (chipFamily(r->slot->chip) != FAM_SX126X) return st;

    if (!sx126xImageBandKnown(freq))
        warn("lora/%d %.3f MHz is outside every band this part has an image "
             "calibration for (430-440, 470-510, 779-787, 863-870, 902-928 MHz) "
             "— the radio will come up and hear very little. Check the frequency "
             "is the one you meant", r->idx, (double)freq);

    SX126x* sx = static_cast<SX126x*>(p);
    if (r->slot->dio2_rf_switch) st = sx->setDio2AsRfSwitch(true);
    if (st == RADIOLIB_ERR_NONE) st = sx->setRxBoostedGainMode(r->rxBoostedGain);
    if (st == RADIOLIB_ERR_NONE) st = sx->setCurrentLimit(radioOcpMilliamps(r->slot->chip));
    if (st == RADIOLIB_ERR_NONE) sx126xRxSensPatch(r);
    return st;
}

/* Hold the oscillator through the gaps inside a chain of frames.
 *
 * Leaving a transmit or a reception drops the part to STDBY_RC, which powers the
 * TCXO down; the next SetTx then waits out the whole TCXO startup the driver
 * programmed — RadioLib asks for 5 ms — before a carrier appears. That wait is
 * the bulk of the dead air between the two halves of a split packet and between
 * the frames of a train, gaps where the next frame is already known and nothing
 * else may use the medium anyway. Held, the part falls back to STDBY_XOSC
 * instead, the reference never stops, and the next frame starts as soon as the
 * PLL locks.
 *
 * The price is the reference's standing current — a couple of hundred µA of chip
 * plus whatever the board's TCXO draws, which is the larger term by an order of
 * magnitude — so it is armed for the length of a chain and dropped the moment
 * the radio goes back to plain listening. Never held while idle.
 *
 * Two settings, one state: the flag governs the standby RadioLib takes on our
 * behalf, the register governs where the part lands by itself when a frame ends.
 * Both have to say the same thing or the wait is only half avoided.
 *
 * SX126x only — it is the family whose fallback mode RadioLib exposes. Elsewhere
 * this is a no-op and the gaps stand as they are. */
void radioHoldOsc(LoraRadio* r, bool hold) {
    if (chipFamily(r->slot->chip) != FAM_SX126X) return;
    if (r->oscHeld == hold) return;
    uint8_t mode = hold ? RADIOLIB_SX126X_RX_TX_FALLBACK_MODE_STDBY_XOSC
                        : RADIOLIB_SX126X_RX_TX_FALLBACK_MODE_STDBY_RC;
    int16_t st = r->mod->SPIwriteStream(RADIOLIB_SX126X_CMD_SET_RX_TX_FALLBACK_MODE,
                                        &mode, 1);
    if (st != RADIOLIB_ERR_NONE) {
        /* Leave the pair at RC: a fallback the chip did not take, paired with a
         * driver that thinks it did, would be a standby mismatch on every frame
         * rather than a slow one. */
        static_cast<SX126x*>(r->radio)->standbyXOSC = false;
        r->oscHeld = false;
        warn("lora/%d oscillator hold %s refused: %s (%d)", r->idx,
             hold ? "on" : "off", rlErrName(st), (int)st);
        return;
    }
    static_cast<SX126x*>(r->radio)->standbyXOSC = hold;
    r->oscHeld = hold;
}

/* ─────────────── IRQ flags ─────────────── */

/* Cache this chip's raw IRQ bits for the flags the task loop tests. getIrqFlags()
 * returns the chip's own register, not RadioLib's radio-agnostic numbering, so a
 * unified bit position tested against it is only right where the two happen to
 * coincide. getIrqMapped() is the translation, and it reads a table the chip
 * class fills in its constructor — so this costs nothing at run time and makes
 * every family's flags read correctly. */
void radioIrqCache(LoraRadio* r) {
    PhysicalLayer* p = r->radio;
    r->irqTxDone   = p->getIrqMapped(1UL << RADIOLIB_IRQ_TX_DONE);
    r->irqRxDone   = p->getIrqMapped(1UL << RADIOLIB_IRQ_RX_DONE);
    r->irqPreamble = p->getIrqMapped(1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED);
    r->irqHdrValid = p->getIrqMapped(1UL << RADIOLIB_IRQ_HEADER_VALID);
    r->irqHdrErr   = p->getIrqMapped(1UL << RADIOLIB_IRQ_HEADER_ERR);
}

/* Enter continuous RX. Every path back to listening goes through here, so the
 * IRQ flag set is stated in exactly one place.
 *
 * PREAMBLE_DETECTED is added to the *flags* — the chip's IRQ register — and not
 * to the DIO mask, so the modem records that it heard something without raising
 * DIO1 for it. The line keeps its single meaning (a frame completed) and an idle
 * radio still holds no wake; radioRxInProgress reads the record when it wants it.
 * Families whose timeout constant differs take their own arm; SX127x has no
 * preamble-detect IRQ to latch, so it keeps the plain call. */
int16_t radioStartRx(LoraRadio* r) {
    PhysicalLayer* p = r->radio;
    switch (chipFamily(r->slot->chip)) {
        case FAM_SX126X:
            return p->startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, LORA_RX_IRQ_FLAGS,
                                   RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
        case FAM_LR11X0:
            return p->startReceive(RADIOLIB_LR11X0_RX_TIMEOUT_INF, LORA_RX_IRQ_FLAGS,
                                   RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
        case FAM_LR2021:
            return p->startReceive(RADIOLIB_LR2021_RX_TIMEOUT_INF, LORA_RX_IRQ_FLAGS,
                                   RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
        case FAM_SX128X:
            return p->startReceive(RADIOLIB_SX128X_RX_TIMEOUT_INF, LORA_RX_IRQ_FLAGS,
                                   RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
        case FAM_SX127X:
            break;
    }
    return p->startReceive();
}

/* Is a reception under way *right now*, as the modem sees it?
 *
 * Carrier sense answers a different question and answers it worse: LoRa
 * demodulates below the noise floor, so a frame being received perfectly may
 * never rise above `noiseFloor + CSMA_RSSI_MARGIN_DB`, and the sense is a point
 * sample once a slot rather than a continuous watch. The demodulator has the
 * evidence the sense lacks — it has locked onto a preamble, or validated a
 * header — and it costs one register read to ask.
 *
 * Both bits latch until something clears them, so both need a deadline or a
 * preamble that never became a packet would block transmit forever: a preamble
 * has until the header should have arrived, a validated header until the longest
 * frame this modem could still be receiving would have finished. Past that the
 * bit is stale, and clearing it is what lets the next real one be believed.
 *
 * Chips without a preamble-detect IRQ report nothing rather than guess; there,
 * carrier sense and the post-hoc csmaMediumHeld correction stand alone. */
bool radioRxInProgress(LoraRadio* r) {
    if (!r->irqPreamble && !r->irqHdrValid) return false;
    if (r->txActive) return false;      /* the flags belong to the transmit in flight */

    uint32_t flags = r->radio->getIrqFlags();
    TickType_t now = xTaskGetTickCount();
    if (now == 0) now = 1;              /* 0 is the "nothing seen" sentinel */

    auto forget = [&](uint32_t clear) {
        if (clear) r->radio->clearIrqFlags(clear);
        r->rxActiveStart = 0;
        r->rxHeaderSeen  = false;
        return false;
    };

    /* A header that failed its check ends the reception then and there. */
    if (flags & r->irqHdrErr)
        return forget(r->irqPreamble | r->irqHdrValid | r->irqHdrErr);

    if (flags & r->irqHdrValid) {
        if (!r->rxHeaderSeen) { r->rxHeaderSeen = true; r->rxActiveStart = now; }
        if ((TickType_t)(now - r->rxActiveStart) > r->rxPacketTicks)
            return forget(r->irqPreamble | r->irqHdrValid);
        return true;
    }
    /* The header bit went away without an RX_DONE — whatever the modem had, it
     * no longer has. */
    if (r->rxHeaderSeen) return forget(0);

    if (flags & r->irqPreamble) {
        if (r->rxActiveStart == 0) r->rxActiveStart = now;
        if ((TickType_t)(now - r->rxActiveStart) > r->rxPreambleTicks)
            return forget(r->irqPreamble);
        return true;
    }
    r->rxActiveStart = 0;
    return false;
}

/* Is the chip's IRQ line asserted? A GPIO read, no SPI: the backstop for a
 * DIO1 whose interrupt was left disabled, which would otherwise leave the task
 * asleep beside a radio holding a completed frame. */
bool radioIrqLinePending(const LoraRadio* r) {
    return r->slot->dio1 >= 0 && gpio_get_level((gpio_num_t)r->slot->dio1) != 0;
}

/* Clear every IRQ the chip has raised. The recovery arm for a raised line with
 * nothing behind it — see serviceRadio. */
void radioIrqClearAll(LoraRadio* r) {
    r->radio->clearIrqFlags(0xFFFFFFFFu);
}

/* Return the receiver to a known-good analog state.
 *
 * An SX126x that has heard a strong signal can leave its automatic gain control
 * latched at that setting, and a receiver stuck at low gain hears nothing
 * afterwards. Neither standby nor a fresh startReceive resets it: only powering
 * the analog front end down does, which is what the warm sleep here is for.
 * Coming back up in RC standby is the state the datasheet requires for
 * calibration; CALIBRATE_ALL then re-runs every block, and because its image
 * calibration defaults to a band that is probably not ours, calibrateImage
 * follows with the frequency actually in use.
 *
 * Calibration resets settings that were applied on top of begin(), so they are
 * re-applied here — including the RX-sensitivity register patch, whose bit
 * CALIBRATE_ALL clears. Leaving that out would make this beat *cost* sensitivity
 * a minute after boot, which is the opposite of the point.
 *
 * The caller establishes that the radio is idle; this is a few milliseconds of
 * chip work, on the order of the LoRaMon channel sweep's excursion. */
bool radioAgcReset(LoraRadio* r) {
    if (chipFamily(r->slot->chip) != FAM_SX126X) return false;
    SX126x* sx = static_cast<SX126x*>(r->radio);

    /* Each step is named so a failure says which one, and they fail in
     * distinguishable ways: a chip that slept through its wake-up pulse times
     * out on BUSY at `wake`, a calibration that will not run reports at
     * `calibrate`, and a frequency this part cannot image-calibrate reports at
     * `image`. Without the name every one of them reads as the same warning. */
    const char* step = "sleep";
    int16_t st = sx->sleep(/*retainConfig=*/true);
    if (st == RADIOLIB_ERR_NONE) {
        step = "wake";
        st = sx->standby(RADIOLIB_SX126X_STANDBY_RC, /*wakeup=*/true);
    }
    if (st == RADIOLIB_ERR_NONE) {
        step = "calibrate";
        st = sx->calibrate(RADIOLIB_SX126X_CALIBRATE_ALL);
    }
    if (st == RADIOLIB_ERR_NONE) {
        step = "image";
#if !defined(CONFIG_LORA_NO_SUPE)
        st = sx->calibrateImage((float)supeChanFreq(r, r->chNow) / 1.0e6f);
#else
        st = sx->calibrateImage((float)r->cfgFreqHz / 1.0e6f);
#endif
    }
    if (st != RADIOLIB_ERR_NONE) {
        warn("lora/%d AGC reset failed at %s: %s (%d) — recovering",
             r->idx, step, rlErrName(st), (int)st);
        /* The chip may still be asleep, and a sleeping SX126x answers nothing:
         * every command waits out BUSY and fails. Only an NSS pulse brings it
         * back, which is what the wake-up standby is, and it is harmless on a
         * part that is already awake — so recovery starts there rather than
         * going straight for RX and stalling the task a second time. */
        sx->standby(RADIOLIB_SX126X_STANDBY_RC, /*wakeup=*/true);
        radioStartRx(r);                /* deaf beats stranded in standby */
        return false;
    }
    if (r->slot->dio2_rf_switch) sx->setDio2AsRfSwitch(true);
    sx->setRxBoostedGainMode(r->rxBoostedGain);
    sx->setCurrentLimit(radioOcpMilliamps(r->slot->chip));
    sx126xRxSensPatch(r);
    /* The front end that measured the old floor no longer exists. */
    csmaNoiseFloorReset(r);
    radioStartRx(r);
    return true;
}

/* The recalibration beat, driven from the task loop like every other poll.
 *
 * It takes the radio for a few milliseconds, so it waits for a radio that is
 * doing nothing else: no frame in flight either way, no channel access under
 * way, nothing queued to send, and on the hailing channel rather than partway
 * through a detour. A beat that arrives at a busy radio is deferred rather than
 * dropped, and deferred by a whole second — leaving the deadline in the past
 * would turn nextDeadline() into a zero-length sleep and spin the task for as
 * long as the radio stayed busy. */
void agcResetPoll(LoraRadio* r) {
    if (!r->agcResetMs || !r->running || !r->enabled) return;
    TickType_t now = xTaskGetTickCount();
    if ((int32_t)(now - r->agcNext) < 0) return;
    if (r->txActive || r->splitPending ||
#if !defined(CONFIG_LORA_NO_SUPE)
        supeHoldsRadio(r) ||
#endif
        r->chNow != LORA_CH_HAIL || r->csmaPhase != CSMA_IDLE ||
        r->mtxPhase != MTXP_OFF || r->annReplay ||
        loraqDepth(&r->q) > 0 || radioRxInProgress(r)) {
        r->agcNext = now + pdMS_TO_TICKS(1000);
        return;
    }
    /* The chip goes down and comes back up here, so take SUPE's lock for the
     * same reason radioStop does: a transaction step runs on the esp_timer task
     * and would otherwise drive a radio that is mid-sleep. */
#if !defined(CONFIG_LORA_NO_SUPE)
    supeLock(r);
    radioAgcReset(r);
    supeUnlock(r);
#else
    radioAgcReset(r);
#endif
    r->agcNext = xTaskGetTickCount() + pdMS_TO_TICKS(r->agcResetMs);
}

/* ─────────────── helpers ─────────────── */

/* Time-on-air (seconds) for a `payload`-byte LoRa frame. The formula itself is
 * in lora_toa.h — one copy, shared with SUPE's portable core, because two copies
 * are exactly how the sweep's airtime came to be over-stated by half. CRC on:
 * every frame this overload is asked about carries one.
 *
 * `implicitHeader` is NOT optional bookkeeping: a headerless frame drops the
 * 20-bit header from the payload term, and the radio-check sweep also runs a
 * 6-symbol preamble instead of the configured 12. Computing a 4-byte sweep frame
 * as explicit/preamble-12 over-states its airtime by ~11 ms at SF7 — some 47% —
 * which lands in the LoRaMon bar widths, the rx start times (start = end − ToA)
 * and the airtime rollups. */
double loraAirtimeSeconds(int sf, int bw_hz, int cr_denom,
                                        int preamble, int payload, bool implicitHeader) {
    return loraToaSeconds(sf, bw_hz, cr_denom, preamble, payload,
                          implicitHeader, /*crc=*/true);
}

/* Total on-air time (ms) for the frame(s) that carry one RNS packet, at the
 * radio's live SF/BW/CR/preamble. Each LoRa frame carries its own preamble,
 * header and CRC, so a split packet's airtime is the SUM of its frames' — never
 * one airtime over the combined length. frameLens[i] is each frame's full
 * on-air byte count (1-byte split header included). */
double loraPacketAirtimeMs(const LoraRadio* r, const size_t* frameLens, int frames) {
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
uint32_t computeBitrate(int sf, int bw_hz, int cr_denom, int preamble) {
    double toa = loraAirtimeSeconds(sf, bw_hz, cr_denom, preamble, RNS_MTU, false);
    if (toa <= 0.0) return 0;
    double secs = ceil(toa);
    if (secs < 1.0) secs = 1.0;
    return (uint32_t)((double)(RNS_MTU * 8) / secs);
}

/* Instantaneous channel RSSI (dBm), read without leaving continuous RX.
 * getRSSI(false) is the "current channel" overload (vs the base getRSSI() which
 * returns last-packet RSSI); it lives on the concrete chip class, not on
 * PhysicalLayer, so dispatch per chip like radioBegin does. */
float channelRssi(LoraRadio* r) {
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

/* Modem header mode: implicit with a fixed `len` (headerless frames carry no
 * length, so both ends must be told) vs explicit (normal).
 * Lives on the concrete classes, not PhysicalLayer — dispatch like channelRssi. */
int16_t radioHeaderMode(LoraRadio* r, bool implicit, size_t len) {
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
int16_t radioSetCodingRate(LoraRadio* r, uint8_t crDenom) {
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

int16_t radioSyncWord(LoraRadio* r, uint8_t sync) {
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

/* Spreading factor at runtime — a SUPE detour's step is the only caller. Not on
 * PhysicalLayer (it is LoRa-specific), so dispatched per family like the sync
 * word above. */
int16_t radioSetSf(LoraRadio* r, uint8_t sf) {
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

/* Bandwidth at runtime, in kHz. A SUPE detour under a regime with a channel plan
 * is the only caller: regime 0 moves the spreading factor and nothing else.
 * Dispatched per chip like the rest — none of these live on PhysicalLayer.
 *
 * Note the low-data-rate optimisation is deliberately *not* set alongside it.
 * Both ends must hold it identically or neither decodes, and RadioLib's own
 * auto-LDRO applies the same "symbol longer than 16 ms" rule the ladder does, so
 * leaving it automatic is what keeps the two ends agreeing. supeResolve reports
 * the same verdict for the tests to check against; nothing writes the register. */
int16_t radioSetBw(LoraRadio* r, float bwKhz) {
    PhysicalLayer* p = r->radio;
    switch (r->slot->chip) {
        case CHIP_SX1261: case CHIP_SX1262: case CHIP_SX1268: case CHIP_LLCC68:
            return static_cast<SX126x*>(p)->setBandwidth(bwKhz);
        case CHIP_SX1272:
            return static_cast<SX1272*>(p)->setBandwidth(bwKhz);
        case CHIP_SX1276: case CHIP_SX1277: case CHIP_SX1278:
            return static_cast<SX1278*>(p)->setBandwidth(bwKhz);
        case CHIP_SX1280: case CHIP_SX1281: case CHIP_SX1282:
            return static_cast<SX128x*>(p)->setBandwidth(bwKhz);
        case CHIP_LR1110: case CHIP_LR1120: case CHIP_LR1121:
            return static_cast<LR11x0*>(p)->setBandwidth(bwKhz);
        case CHIP_LR2021:
            return static_cast<LR2021*>(p)->setBandwidth(bwKhz);
    }
    return RADIOLIB_ERR_UNKNOWN;
}

#endif  /* CONFIG_LORA0_CS_PIN */
