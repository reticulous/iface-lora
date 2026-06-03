# tr-lora

## What is this?

**tr-lora** is the LoRa transport for [rns](../rns), driving **any RadioLib LoRa
chip** — SX126x (SX1261/2/8, LLCC68), SX127x / RFM9x (SX1272/6/7/8), SX128x
(2.4 GHz), LR11x0 (LR1110/20/21) and LR2021. It carries the RadioLib ESP-IDF
HAL, owns the radio's IRQ line, and implements **RNode on-air framing** so the
device interoperates with the wider Reticulum LoRa ecosystem. The task loop is
chip-agnostic (RadioLib's `PhysicalLayer`); only `begin()` dispatches per
family.

## What this straddle owns

```
tr-lora/
├── esp-idf/
│   ├── include/lora.h
│   └── src/
│       ├── lora.cpp           the lora transport task
│       └── esp_idf_hal.{cpp,h} custom RadioLib HAL for ESP-IDF
└── browser/
    └── src/
        ├── modules/lora.ts
        └── panels/LoraPanel.vue
```

## How others use it

```cpp
loraInit();    // after rnsdInit, after the board has powered the LoRa rail
```

**Pins + per-radio type** come from this component's own Kconfig
(`menu "LoRa radios"`): a shared SPI bus (`CONFIG_LORA_SPI_HOST` /
`_SCK` / `_MOSI` / `_MISO`) plus up to four radio slots, each with its
own `CONFIG_LORAn_CS_PIN` / `_DIO1_PIN` (the IRQ line — DIO1/DIO0/IRQ per
chip) / `_BUSY_PIN` (−1 on SX127x) / `_RST_PIN` / `_TCXO_MV` /
`_DIO2_RF_SWITCH` (SX126x) / `_RFSW_RX_PIN` + `_RFSW_TX_PIN` (external
antenna RF switch, any chip) and a `chip` (any of the parts above). Boards
set these in their `sdkconfig.defaults`; a build can override with
`spangap build --lora-count N --loraN-cs … --loraN-radio sx1262
--loraN-rfsw-rx … --loraN-rfsw-tx …` or interactively via
`spangap menuconfig`. `CONFIG_LORA_COUNT=0` (the default) leaves tr-lora
staged but inert.

The antenna RF switch is configured one of two ways: SX126x parts can drive it
from the chip's own DIO2 (`_DIO2_RF_SWITCH`); any chip can instead use two MCU
GPIOs via `_RFSW_RX_PIN` / `_RFSW_TX_PIN` (RadioLib `Module::setRfSwitchPins`).
A complex chip-DIO switch *table* (some LR11x0/LR2021 boards) isn't expressible
as two pins yet — that needs a board-supplied table.

Each radio registers with rnsd as its own interface `lora/<slot>`
(`lora/0`, `lora/1`, …); tr-lora owns the name's uniqueness (rnsd takes
it verbatim).

Configuration (runtime, per radio — replace `<n>` with the slot index):

- `s.lora.<n>.enable` — on/off
- `s.lora.<n>.frequency` — Hz (per-region; 868 MHz in EU, 915 MHz in US)
- `s.lora.<n>.bandwidth` — kHz (125/250/500)
- `s.lora.<n>.spreading_factor` — 7..12
- `s.lora.<n>.coding_rate` — 5..8
- `s.lora.<n>.tx_power` — dBm (-9..22)
- `s.lora.<n>.mode` — RNS iface mode (full/gateway/access_point/roaming/boundary)

## Dependencies

- [rns](../rns)
- (no `spangap-net` dep — LoRa is bare-radio)

## What it does NOT own

- Power gating of the LoRa rail — that's the consuming app's board
  HAL (e.g. hw-tdeck's `tdeckPowerInit()`, which brings the shared
  peripheral rail up before `spangapInit()`). tr-lora assumes the rail
  is already live.
- The SX1262 antenna selection (PCB-trace vs IPEX vs SMA) — that's
  hardware, not software.

## Read next

- [INTERNALS.md](INTERNALS.md) — DIO1 ISR rules, RNode framing,
  half-duplex coordination, RadioLib HAL details.
- The hw-tdeck doc:
  [docs/lora.md](../hw-tdeck/docs/lora.md).
