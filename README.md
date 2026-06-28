# iface-lora — LoRa interface for Reticulum

**iface-lora** is the LoRa interface for [rns](../rns): it carries Reticulum
packets over a LoRa radio. It drives **any RadioLib LoRa chip** — the SX126x
family (SX1261/2/8, LLCC68), SX127x / RFM9x (SX1272/6/7/8), SX128x (2.4 GHz),
LR11x0 (LR1110/20/21) and LR2021 — and can run **up to four radios** off one
shared SPI bus, each registering with `rnsd` as its own interface `lora/0`,
`lora/1`, … A single task services every radio; the loop is chip-agnostic and
only the per-chip bring-up dispatches by family.

## Origins

The radio driver is [jgromes/RadioLib](https://github.com/jgromes/RadioLib),
pulled in as the `radiolib` IDF component. iface-lora supplies its own RadioLib
HAL (`EspIdfHal`, in `esp_idf_hal.cpp`) implementing RadioLib's GPIO/SPI/timing
surface on ESP-IDF, plus the LoRa interface task (`lora.cpp`) that frames RNS
packets for the air and bridges them to `rnsd`. Driver internals are in
[INTERNALS.md](INTERNALS.md).

## What it does

`rnsd` owns Reticulum but has no radio of its own. iface-lora is one of the
interface straddles that plug into it: each radio opens an ITS connection to
**`RNSD_PORT_IFACE`** with an `rnsd_iface_t` payload (name `lora/<slot>`, MTU
500, an airtime-derived bitrate, the interface mode, and any IFAC credentials).
After that the handle *is* the packet pipe — every inbound LoRa frame is
forwarded to `rnsd` as one RNS packet, and every packet `rnsd` sends back goes
out on the air.

```
  rnsd  ──RNSD_PORT_IFACE──  iface-lora  ──LoRa──  other Reticulum-on-LoRa nodes
        (one RNS packet per ITS send/recv)        (split framing, ≤2 frames/MTU)
```

A node-side announce, an LXMF message, a NomadNet page fetch — anything `rnsd`
routes — can leave over LoRa with no extra wiring. iface-lora has **no compile-
time link to any consumer**; it only talks to `rnsd`.

iface-lora **starts automatically** when the straddle is in the build and at
least one radio is configured (`CONFIG_LORA_COUNT > 0`). With
`CONFIG_LORA_COUNT = 0` (the default) it stages but does nothing and RadioLib is
linked out.

## Configuring radios (Kconfig)

Pins and per-radio chip type come from this straddle's own Kconfig (`LORA_*` /
`LORAn_*`), resolved from the buildable's `sdkconfig`. Boards set them in their
`sdkconfig.defaults`; a build can override with `spangap build --lora-count N`
and the matching `--loraN-*` switches, or interactively via `spangap
menuconfig`.

- **`CONFIG_LORA_COUNT`** (0–4) — how many radios to drive. `0` = inert.
- **Shared SPI bus** — `CONFIG_LORA_SPI_HOST` (1 = SPI1, 2 = SPI2/FSPI,
  3 = SPI3), `_SCK_PIN`, `_MOSI_PIN`, `_MISO_PIN`. One bus carries every radio
  (and, on boards like the T-Deck, the display and SD card too).
- **Per radio `n` (0–3):**
  - `CONFIG_LORAn_CS_PIN` — chip select / NSS.
  - `CONFIG_LORAn_DIO1_PIN` — the chip's **IRQ line** wired to an MCU GPIO:
    DIO1 on SX126x/SX128x, DIO0 on SX127x, IRQ on LR11x0/LR2021.
  - `CONFIG_LORAn_BUSY_PIN` — BUSY line (`-1` on SX127x, which has none).
  - `CONFIG_LORAn_RST_PIN` — reset (`-1` if not wired).
  - `CONFIG_LORAn_TCXO_MV` — TCXO control voltage in mV (`0` = crystal).
    Applies to SX126x / LR11x0 / LR2021; ignored by SX127x and SX128x.
  - `CONFIG_LORAn_DIO2_RF_SWITCH` — SX126x only: let the chip drive the antenna
    RF switch from its own DIO2.
  - `CONFIG_LORAn_RFSW_RX_PIN` / `_RFSW_TX_PIN` — an external antenna RF switch
    driven by two MCU GPIOs (`-1`/`-1` if none). Any chip family.
  - **chip** — the radio part on the slot (`SX1262` default); the choice covers
    all 15 supported parts.

A complex chip-DIO RF-switch *table* (some LR11x0/LR2021 boards) isn't
expressible as two pins and needs board-supplied support; the two-GPIO and
SX126x-DIO2 forms cover the common cases.

## Storage variables

Settings live under `s.lora.<n>.*` per radio (writable by the user, the browser
panel, and the LCD pane); runtime state and telemetry are published under
`lora.<n>.*` for anything to observe. Replace `<n>` with the slot index. Radio 0
defaults come from this straddle's `settings:` block; radios 1.. are seeded by
`loraInit`.

### Settings (read)

| Key | Default | Meaning |
|---|---|---|
| `s.lora.<n>.enable` | `0` | Bring this radio up. Live — toggling it starts/stops the radio. |
| `s.lora.<n>.frequency` | *(none)* | Carrier frequency in **Hz**. No default — region/antenna dependent, user must pick. |
| `s.lora.<n>.bandwidth` | `125000` | Bandwidth in **Hz** (125/250/500 kHz; SX128x also 203/406/812/1625 kHz). |
| `s.lora.<n>.spreading_factor` | `7` | Spreading factor, 5–12. |
| `s.lora.<n>.coding_rate` | `5` | Coding-rate denominator, 5–8 (`5` = 4/5). |
| `s.lora.<n>.tx_power` | *(none)* | TX power in dBm, −9..22. No default — antenna dependent. |
| `s.lora.<n>.preamble` | `12` | Preamble length in symbols, 6–32. |
| `s.lora.<n>.sync_word` | `"0x42"` | Sync word, a string parsed as hex or decimal (`0x42` is the Reticulum-on-LoRa convention). |
| `s.lora.<n>.mode` | `"gateway"` | RNS interface mode: `full`, `gateway`, `access_point`, `roaming`, `boundary`. |
| `s.lora.<n>.ifac_netname` | `""` | IFAC network name. Empty = open (non-IFAC) interface. |
| `s.lora.<n>.ifac_size` | `0` | IFAC access-code length in bytes (`0` = rnsd default). |
| `s.lora.version` | — | Internal defaults-seeding gate; not a user setting. |

A radio refuses to come up until `frequency`, `tx_power`, and a valid
SF/BW/CR/preamble are set; `lora.<n>.state` reads `unconfigured` until then.

### Runtime state & telemetry (written)

| Key | Meaning |
|---|---|
| `lora.<n>.up` | `1` when the radio is on-air, else `0`. |
| `lora.<n>.state` | `unconfigured` / `error` / `up` / `down` / `rnsd_unavailable`. |
| `lora.<n>.chip` | Detected chip name, e.g. `SX1262`. |
| `lora.<n>.bitrate_eff` | Effective bitrate registered with `rnsd`, bits/s (airtime-derived). |
| `lora.<n>.stats.{tx_bytes,rx_bytes,tx_frames,rx_frames,crc_err,split_rx_timeout,rssi_last,snr_last}` | Traffic counters and last-RX RSSI/SNR. |

### Secrets

`secrets.lora.<n>.ifac_netkey` — the IFAC passphrase (a secret; never synced to
the browser). With `ifac_netname` it puts the interface on an access-coded RNS
network; `rnsd` derives the IFAC identity from the pair.

## CLI

```
lora                          status for every radio (chip, pins, config, traffic)
lora <n>                      status for one radio
lora up | down                enable / disable all radios
lora <n> up | down            enable / disable one radio
lora <n> freq <MHz>           set carrier frequency (MHz in, stored as Hz)
lora <n> bw <kHz>             set bandwidth (kHz in, stored as Hz)
lora <n> sf <5..12>           spreading factor
lora <n> cr <5..8>            coding-rate denominator (5 = 4/5)
lora <n> txp <dBm>            TX power
lora <n> preamble <sym>       preamble length
lora <n> sync <word>          sync word (hex or decimal)
lora <n> mode <name>          interface mode
lora help | -h                command summary
```

The `freq`/`bw`/… subcommands write the matching `s.lora.<n>.*` key, which the
task picks up and re-applies live. Run any of these on-device through `spangap
cli "<command>"`.

## Browser

The LoRa Settings panel (`browser/panels/LoraPanel.vue`, registered by
`modules/lora.ts`) edits radio 0 — band, bandwidth, SF, coding rate, TX power,
preamble, sync word, mode, the IFAC pair — and shows live state, chip, bitrate,
last RSSI/SNR, and frame counts. The on-device LoRa pane is generated from this
straddle's `settings:` block.

## Dependencies

- [rns](../rns) — `rnsd` must be ahead of iface-lora in init order so
  `RNSD_PORT_IFACE` is open when a radio registers (`requires:` enforces it).
- No `spangap-net` dependency — LoRa is bare-radio, no IP stack.

## What it does NOT own

- **The LoRa power rail.** Whatever powers the radio (a shared peripheral rail on
  boards like the T-Deck) is brought up by the board HAL before `spangapInit`.
  iface-lora assumes the rail is already live.
- **Antenna selection** (PCB trace vs IPEX vs SMA) — that's hardware.

## Read next

- [INTERNALS.md](INTERNALS.md) — the chip-dispatch table, the RadioLib HAL, the
  IRQ/ISR rules, the on-air split framing, the start/stop lifecycle, and
  maintainer pitfalls.
