/**
 * lora_radio — chip dispatch and the RadioLib call surface: construction,
 * begin(), the per-chip/per-family setters, channel RSSI, and time-on-air.
 * Knows nothing of Reticulum or SUPE.
 */
#include "lora_priv.h"

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

/* begin() the radio with the common LoRa parameters. Each family's begin() has
 * a different signature: SX126x carries TCXO + regulator; SX127x a LNA-gain arm
 * (0 = AGC) and no TCXO; SX128x is 2.4 GHz and bare; LR11x0 sets freq/power
 * separately (lr11x0Begin); LR2021 takes everything including TCXO. We cast to
 * the concrete class (the pointer really is that class) so dispatch is correct
 * regardless of where each begin() sits in RadioLib's hierarchy. SX126x also
 * applies DIO2-as-RF-switch when the slot asks for it, and the LNA boosted-RX-gain
 * option (r->rxBoostedGain): ~+3 dB sensitivity for ~0.4 mA more RX current. */
int16_t radioBegin(LoraRadio* r, float freq, float bw, uint8_t sf, uint8_t cr,
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
    /* SX126x only: DIO2 drives the antenna RF switch, and the LNA RX gain mode. */
    if (st == RADIOLIB_ERR_NONE && r->slot->dio2_rf_switch)
        st = static_cast<SX126x*>(p)->setDio2AsRfSwitch(true);
    if (st == RADIOLIB_ERR_NONE && chipFamily(r->slot->chip) == FAM_SX126X)
        st = static_cast<SX126x*>(p)->setRxBoostedGainMode(r->rxBoostedGain);
    return st;
}

/* ─────────────── helpers ─────────────── */

/* Time-on-air (seconds) for a `payload`-byte LoRa frame, per Semtech
 * AN1200.13. Symbol time Tsym = 2^SF / BW; the preamble runs (n+4.25)
 * symbols and the payload rounds up into whole symbols, with low-data-rate
 * optimisation (DE) engaged once a symbol exceeds 16 ms. CRC on — every frame
 * this overload is asked about carries one. The CRC-off form a SUPE frame flies
 * with is the same formula with the parameter cleared, which is why the formula
 * lives in supe.cpp and this is a call rather than a copy: two copies of it are
 * exactly how the sweep's airtime came to be over-stated by half.
 *
 * `implicitHeader` is NOT optional bookkeeping: a headerless frame drops the
 * 20-bit header from the payload term, and the radio-check sweep also runs a
 * 6-symbol preamble instead of the configured 12. Computing a 4-byte sweep frame
 * as explicit/preamble-12 over-states its airtime by ~11 ms at SF7 — some 47% —
 * which lands in the LoRaMon bar widths, the rx start times (start = end − ToA)
 * and the airtime rollups. */
double loraAirtimeSeconds(int sf, int bw_hz, int cr_denom,
                                        int preamble, int payload, bool implicitHeader) {
    return supeAirtimeSeconds(sf, bw_hz, cr_denom, preamble, payload,
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
