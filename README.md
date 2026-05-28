# reticulous-lora

## What is this?

**reticulous-lora** is the SX1262 LoRa transport for
[reticulous-core](../reticulous-core). It carries the RadioLib ESP-IDF
HAL, owns the SX1262 DIO1 interrupt, and implements **RNode on-air
framing** so the device interoperates with the wider Reticulum LoRa
ecosystem.

## What this straddle owns

```
reticulous-lora/
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

The **board pin map** (`BOARD_LORA_CS`, `BOARD_LORA_DIO1`,
`BOARD_LORA_RST`, `BOARD_LORA_BUSY`, `BOARD_LORA_SPI_*`) comes from the
consuming app-straddle's `main/` (e.g. reticulous-tdeck's
`tdeck.h`). The component's PRIVATE include path includes
`${CMAKE_SOURCE_DIR}/main` so consumers don't have to thread the pin
defs through Kconfig or a function-pointer table.

Configuration:

- `s.lora.enable` — on/off
- `s.lora.frequency` — Hz (per-region; 868 MHz in EU, 915 MHz in US)
- `s.lora.bandwidth` — kHz (125/250/500)
- `s.lora.spreading_factor` — 7..12
- `s.lora.coding_rate` — 5..8
- `s.lora.tx_power_dbm` — bounded by Kconfig per region
- `s.lora.ifac_psk` — optional per-iface PSK (IFAC)

## Dependencies

- [reticulous-core](../reticulous-core)
- (no `spangap-net` dep — LoRa is bare-radio)

## What it does NOT own

- Power gating of the LoRa rail — that's the consuming app's board
  HAL. `loraInit()` re-asserts the power pin (idempotent no-op) but
  does not gate it from cold.
- The SX1262 antenna selection (PCB-trace vs IPEX vs SMA) — that's
  hardware, not software.

## Read next

- [INTERNALS.md](INTERNALS.md) — DIO1 ISR rules, RNode framing,
  half-duplex coordination, RadioLib HAL details.
- The reticulous-tdeck doc:
  [docs/lora.md](../reticulous-tdeck/docs/lora.md).
