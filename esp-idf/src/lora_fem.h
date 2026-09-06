/* ─────────────── lora_fem: external RF front-end module (PA/LNA/switch) ───────────────
 *
 * Some boards put a FEM between the radio and the antenna: a TX power
 * amplifier, an RX low-noise amplifier and the antenna switch in one part,
 * driven by a rail-enable plus two logic pins. The radio's own dBm range then
 * stops being the antenna's — the FEM amplifies in both directions — so every
 * number that leaves this interface is referenced to the ANTENNA CONNECTOR and
 * the register-referenced figures live only in here: rfChipDbm converts on the
 * way out, rfAntennaDbm says what a setting actually radiates, rfRssiDbm takes
 * a received level back to the connector. The switch pins must track
 * standby/RX/TX.
 *
 * The model is one measured curve per part, register dBm against connector
 * dBm, because neither half of the error is a constant: the chip's own
 * set-versus-actual output drifts several dB at the extremes, the amplifier
 * compresses, and a board may put a fixed pad between the two. A flat gain
 * figure cannot express any of that, so it is the fallback for a part nobody
 * has characterised, not the model.
 *
 * Every curve states its own provenance — measured, datasheet, or none — and
 * that grade is published (lora.<n>.cal) rather than assumed, so an
 * uncalibrated board is never mistaken for a characterised one.
 *
 * Two wirings exist, and they are told apart by whether the MCU can reach the
 * part at all.
 *
 * DETECTED — the part sits on MCU GPIOs (rail-enable, chip-enable, direction).
 * Two candidates are supported, sensed at boot because boards ship both across
 * revisions on the same enable net (Heltec V4: GC1109 on ≤4.2, KCT8103L on
 * 4.3): the KCT8103L design pulls the enable line up, the GC1109 one leaves it
 * floating — read the pin as an input before driving it. Mode switching rides
 * on RadioLib's RF-switch table, so every standby/startReceive/startTransmit
 * call site in the driver is covered without edits.
 *
 * DECLARED — the control lines hang off the RADIO's own DIOs (LORAn_LR_RFSW_*,
 * programmed in lora_radio.cpp), so there is no pin to sense and no table to
 * install here. Nothing identifies the part at runtime; the board states its
 * gain (LORAn_FEM_GAIN_DB) and its antenna ceiling (LORA_TX_POWER_MAX), and
 * that pair is the whole model.
 *
 * A DUAL-BAND part has one front end per port, and they share nothing: the
 * sub-GHz amplifier and the 2.4 GHz one have separate supply gates
 * (LORAn_FEM_PWR_PIN / LORAn_FEM_HF_PWR_PIN), separate gains and separate
 * ceilings, and the chip itself takes a different drive range on each port.
 * femBandSelect is what makes the carrier decide all of it: it runs from
 * radioBegin, where the frequency is finally known, so tuning across
 * 1500 MHz moves the supply, the ceiling and the conversion together and the
 * amplifier for the band nobody is using draws nothing.
 *
 * femInit must run after the Module exists and before begin(); femBandSelect
 * runs inside every begin.
 */
#pragma once

#include <stdint.h>

#include "lora_rfcal.h"   /* the curve and the arithmetic on it */

struct LoraRadio;

enum LoraFemType : uint8_t {
    FEM_NONE = 0,
    FEM_GC1109,     /* enable (CSD) + TX-select (CPS: 1 = PA, 0 = bypass); the
                     * TX/RX direction line CTX is on the radio's own DIO2 */
    FEM_KCT8103L,   /* enable (CSD) + direction (CTX: 1 = TX, 0 = RX via LNA).
                     * Receiving with CTX held at 1 goes round the LNA: the
                     * only detected part whose receive amplifier can be
                     * switched out (femRxLna) */
    FEM_DECLARED,   /* switched by the radio's own DIOs; known only by its gain */
};

/* Detect the FEM (rail up + enable-pin sense), install the RF-switch table on
 * r->mod, set r->femType / r->maxTxDbm, and exempt every FEM pin from
 * light-sleep isolation. No-op (femType FEM_NONE, maxTxDbm = chip max) when
 * the slot has no FEM pins. */
void femInit(LoraRadio* r);

/* Keep the front end's receive LNA in the RX path (on) or route reception
 * round it (off). The LNA is the front end's standing cost — about 8 mA for as
 * long as the radio listens, on top of the chip's own ~5 — bought for its
 * ~20 dB of gain ahead of the chip; a node on a small solar budget may prefer
 * the range loss. Only a KCT8103L offers the choice: the GC1109's select pin
 * picks the transmit path and its LNA is always in line, so there and on every
 * other wiring this records `on` and changes nothing. Swaps the RF-switch
 * table's RX row, which RadioLib applies on the next mode transition; the
 * receive-gain correction follows at the next femBandSelect (every begin), so
 * call this before radioBegin, as applyConfig does. */
void femRxLna(LoraRadio* r, bool on);

/* Point the front end at the band this carrier is on: raise that port's supply
 * gate and drop the other's, record the band, and set/publish the antenna
 * ceiling it brings. Called from radioBegin with the frequency it is about to
 * program. A single-band board (no HF supply pin, no HF gain) keeps its one
 * front end up and only ever sees the sub-GHz numbers. */
void femBandSelect(LoraRadio* r, bool highBand);

/* Connector dBm → the register setting that comes closest from below, within
 * what the chip's port in use will accept. Where a curve turns over near
 * saturation this returns the LOWEST setting that reaches the request, which
 * is also the cheapest; a request nothing reaches lands on the best there is. */
int8_t rfChipDbm(LoraRadio* r, int8_t antennaDbm);

/* What a register setting actually puts on the connector. This is what a
 * transmission announces and what the record is stamped with — never the
 * request, which the range or the curve may not have been able to honour. */
int8_t rfAntennaDbm(const LoraRadio* r, int8_t chipDbm);

/* A level read at the chip, referred back to the connector: the front end's
 * receive gain comes off. SNR is a ratio and gets no such correction. */
float rfRssiDbm(const LoraRadio* r, float chipRssi);

const char* femName(LoraFemType t);
