/* ─────────────── lora_fem: external RF front-end module (PA/LNA/switch) ───────────────
 *
 * Some boards put a FEM between the radio and the antenna: a TX power
 * amplifier, an RX low-noise amplifier and the antenna switch in one part,
 * driven by a rail-enable plus two logic pins. The radio's own dBm range then
 * stops being the antenna's — the FEM adds its gain on TX — so every
 * user-facing power number is ANTENNA dBm and converts to chip drive at the
 * last moment (femChipDbm), and the switch pins must track standby/RX/TX.
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

struct LoraRadio;

enum LoraFemType : uint8_t {
    FEM_NONE = 0,
    FEM_GC1109,     /* enable (CSD) + TX-select (CPS: 1 = PA, 0 = bypass/RX) */
    FEM_KCT8103L,   /* enable (CSD) + direction (CTX: 1 = TX, 0 = RX via LNA) */
    FEM_DECLARED,   /* switched by the radio's own DIOs; known only by its gain */
};

/* Detect the FEM (rail up + enable-pin sense), install the RF-switch table on
 * r->mod, set r->femType / r->maxTxDbm, and exempt every FEM pin from
 * light-sleep isolation. No-op (femType FEM_NONE, maxTxDbm = chip max) when
 * the slot has no FEM pins. */
void femInit(LoraRadio* r);

/* Point the front end at the band this carrier is on: raise that port's supply
 * gate and drop the other's, record the band, and set/publish the antenna
 * ceiling it brings. Called from radioBegin with the frequency it is about to
 * program. A single-band board (no HF supply pin, no HF gain) keeps its one
 * front end up and only ever sees the sub-GHz numbers. */
void femBandSelect(LoraRadio* r, bool highBand);

/* Antenna dBm → chip drive dBm through the FEM's gain — a measured per-dBm
 * table for a detected part, the board's flat declared figure for a declared
 * one, identity for FEM_NONE — and clamped to what the chip's port in use will
 * accept. Never returns more than the chip's own max. */
int8_t femChipDbm(LoraRadio* r, int8_t antennaDbm);

const char* femName(LoraFemType t);
