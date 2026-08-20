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
 *
 * A third kind of front end offers the firmware nothing to detect and nothing
 * to switch: fixed, always in the path, powered with the board (B&Q Station
 * G2 — a 35 dBm TX PA plus an 18.5 dB RX LNA, the antenna switch on the
 * SX1262's own DIO2). Such a FEM is DECLARED per slot
 * (CONFIG_LORAn_FEM_FIXED_GAIN_DB) rather than sensed, so femInit only sets
 * the type and the antenna ceiling and does no GPIO work at all. The one
 * thing the conversion must never do there is pass the request through
 * unchanged: the PA's input compresses far below the chip's own range
 * (Station G2: input P1dB region ends at chip 16 dBm, absolute cap 19, chip
 * max 22), so femChipDbm subtracts the declared flat gain and clamps the
 * chip drive to CONFIG_LORAn_FEM_MAX_CHIP_DBM. The RX side is NOT modeled:
 * an always-on LNA reads RSSI/SNR hot by its gain, uncorrected.
 */
#pragma once

#include <stdint.h>

struct LoraRadio;

enum LoraFemType : uint8_t {
    FEM_NONE = 0,
    FEM_GC1109,     /* enable (CSD) + TX-select (CPS: 1 = PA, 0 = bypass/RX) */
    FEM_KCT8103L,   /* enable (CSD) + direction (CTX: 1 = TX, 0 = RX via LNA) */
    FEM_FIXED,      /* always in the path, no control or detect pins — declared
                     * (fem_fixed_gain_db), never detected. Flat TX gain; chip
                     * drive capped at the slot's fem_max_chip_dbm. The Station
                     * G2 class of hardware. */
};

/* Detect the FEM (rail up + enable-pin sense), install the RF-switch table on
 * r->mod, set r->femType / r->maxTxDbm, and exempt every FEM pin from
 * light-sleep isolation. A slot declaring a fixed FEM (fem_fixed_gain_db > 0)
 * skips all of that: type and ceiling are set on the declaration alone, and
 * the chip's own DIO2 owns the switch. No-op (femType FEM_NONE, maxTxDbm =
 * chip max) when the slot declares no FEM at all. */
void femInit(LoraRadio* r);

/* Antenna dBm → chip drive dBm through the detected FEM's per-dBm gain table;
 * for a fixed FEM the declared flat gain is subtracted and the result clamped
 * to what its input tolerates (fem_max_chip_dbm). Identity for FEM_NONE.
 * Never returns more than the chip's own max. */
int8_t femChipDbm(LoraRadio* r, int8_t antennaDbm);

const char* femName(LoraFemType t);
