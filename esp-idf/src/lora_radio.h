#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

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

/* ─────────────── lora_radio: chip dispatch + RadioLib calls ─────────────── */
const char*    chipName(LoraChip c);
LoraFamily     chipFamily(LoraChip c);
PhysicalLayer* radioNew(LoraChip c, Module* m);
const char*    rlErrName(int16_t st);
int16_t radioBegin(LoraRadio* r, float freq, float bw, uint8_t sf, uint8_t cr,
                   uint8_t sync, int8_t power, uint16_t preamble, float tcxoV);
float   channelRssi(LoraRadio* r);
int16_t radioHeaderMode(LoraRadio* r, bool implicit, size_t len);
int16_t radioSetCodingRate(LoraRadio* r, uint8_t crDenom);
int16_t radioSyncWord(LoraRadio* r, uint8_t sync);
int16_t radioSetSf(LoraRadio* r, uint8_t sf);
int16_t radioSetBw(LoraRadio* r, float bwKhz);
double  loraAirtimeSeconds(int sf, int bw_hz, int cr_denom, int preamble,
                           int payload, bool implicitHeader);
double  loraPacketAirtimeMs(const LoraRadio* r, const size_t* frameLens, int frames);
uint32_t computeBitrate(int sf, int bw_hz, int cr_denom, int preamble);
