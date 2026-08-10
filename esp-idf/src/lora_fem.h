/* ─────────────── lora_fem: external RF front-end module (PA/LNA/switch) ───────────────
 *
 * Some boards put a FEM between the radio and the antenna: a TX power
 * amplifier, an RX low-noise amplifier and the antenna switch in one part,
 * driven by a rail-enable plus two logic pins. The radio's own dBm range then
 * stops being the antenna's — the FEM adds its gain on TX — so every
 * user-facing power number is ANTENNA dBm and converts to chip drive at the
 * last moment (femChipDbm), and the switch pins must track standby/RX/TX.
 *
 * Two FEM parts are supported, auto-detected at boot because boards ship both
 * across revisions on the same enable net (Heltec V4: GC1109 on ≤4.2,
 * KCT8103L on 4.3): the KCT8103L design pulls the enable line up, the GC1109
 * one leaves it floating — read the pin as an input before driving it.
 * Mode switching rides on RadioLib's RF-switch table, so every
 * standby/startReceive/startTransmit call site in the driver is covered
 * without edits. femInit must run after the Module exists and before begin().
 */
#pragma once

#include <stdint.h>

struct LoraRadio;

enum LoraFemType : uint8_t {
    FEM_NONE = 0,
    FEM_GC1109,     /* enable (CSD) + TX-select (CPS: 1 = PA, 0 = bypass/RX) */
    FEM_KCT8103L,   /* enable (CSD) + direction (CTX: 1 = TX, 0 = RX via LNA) */
};

/* Detect the FEM (rail up + enable-pin sense), install the RF-switch table on
 * r->mod, set r->femType / r->maxTxDbm, and exempt every FEM pin from
 * light-sleep isolation. No-op (femType FEM_NONE, maxTxDbm = chip max) when
 * the slot has no FEM pins. */
void femInit(LoraRadio* r);

/* Antenna dBm → chip drive dBm through the detected FEM's per-dBm gain table
 * (identity for FEM_NONE). Never returns more than the chip's own max. */
int8_t femChipDbm(LoraRadio* r, int8_t antennaDbm);

const char* femName(LoraFemType t);
