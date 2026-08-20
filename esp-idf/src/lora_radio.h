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
    int      fem_pwr;              /* FEM rail-enable GPIO (-1 = none) — see lora_fem.h */
    int      fem_en;               /* FEM chip-enable GPIO (-1 = no FEM); also the detect sense */
    int      fem_txsel_a;          /* TX-select when a GC1109-style FEM is detected */
    int      fem_txsel_b;          /* TX-select when a KCT8103L-style FEM is detected */
    int      fem_gain_db;          /* declared-FEM TX gain, dB (0 = no declared FEM) */
    int      fem_hf_pwr;           /* 2.4 GHz front end's supply GPIO (-1 = single band) */
    int      fem_hf_gain_db;       /* declared-FEM TX gain on the 2.4 GHz path, dB */
    int      lr_irq_dio;           /* LR2021: which chip DIO carries the IRQ line (5..11) */
    uint8_t  lr_rfsw[5];           /* LR2021: DIOs driven high per mode, in the order the
                                    * chip numbers them — idle, rx, tx, rx_hf, tx_hf; bit 0
                                    * = DIO5 … bit 6 = DIO11. All zero = no radio-driven
                                    * front end (see lora_radio.cpp's lr2021ApplyDio) */
    LoraChip chip;
};

/* The IRQ flags the receiver latches. RadioLib's default set plus
 * PREAMBLE_DETECTED, which is what lets radioRxInProgress see a reception the
 * RSSI sense is blind to. These are the chip's IRQ *register* bits, not the DIO
 * mask — nothing extra reaches DIO1, so the line still means "a frame
 * completed" and an idle radio still holds no wake. */
#define LORA_RX_IRQ_FLAGS \
    (RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED))

/* Default period of the analog front-end recalibration, in seconds
 * (s.lora.<i>.agc_reset; 0 = off). Long enough that the wake is nothing next to
 * a receiver's idle draw, short enough that a latched gain control costs minutes
 * rather than the days it would otherwise last. */
#define LORA_AGC_RESET_DEF_S  300

/* ─────────────── lora_radio: chip dispatch + RadioLib calls ─────────────── */
const char*    chipName(LoraChip c);
LoraFamily     chipFamily(LoraChip c);
PhysicalLayer* radioNew(LoraChip c, Module* m);
const char*    rlErrName(int16_t st);
int16_t radioBegin(LoraRadio* r, float freq, float bw, uint8_t sf, uint8_t cr,
                   uint8_t sync, int8_t power, uint16_t preamble, float tcxoV);
float   radioOcpMilliamps(LoraChip c);

/* Above this the dual-band parts (LR2021, SX128x) are on their high-frequency
 * port: a different amplifier, a different chip drive range and a different
 * antenna ceiling. RadioLib's own LR2021 cutoff, spelled here because the FEM
 * and power paths have to agree with it. */
#define LORA_HF_CUTOFF_MHZ  1500.0f
static inline bool loraFreqIsHighBand(float freqMhz) { return freqMhz > LORA_HF_CUTOFF_MHZ; }
void    radioIrqCache(LoraRadio* r);
int16_t radioStartRx(LoraRadio* r);
bool    radioRxInProgress(LoraRadio* r);
bool    radioIrqLinePending(const LoraRadio* r);
void    radioIrqClearAll(LoraRadio* r);
bool    radioAgcReset(LoraRadio* r);
void    agcResetPoll(LoraRadio* r);
void    radioHoldOsc(LoraRadio* r, bool hold);
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
