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
| `s.lora.<n>.lbt` | `1` | Listen-before-talk: CSMA/CA carrier-sense before each transmit. `0` = blind transmit (no sensing). Live. On a quiet, single-node band you can turn it off to skip the sensing; on a shared band leave it on. |
| `s.lora.<n>.appc` | `1` | Adaptive p-persistent CSMA — see below. Sizes the random backoff by how much of the recent past *this radio* spent transmitting, instead of growing it on collisions. Only has an effect while `lbt` is on. `0` reverts to the exponential-backoff regime. Live. |
| `s.lora.<n>.lbt_timeout` | `5000` | Drop a frame LBT can't clear within this many ms (`0` = never drop, block the queue instead). Note that a fully-loaded `appc` radio can legitimately back off for 5.2 s at SF10 and 5.8 s at SF11/SF12, past this default, so a congested slow link will shed frames — raise it or set `0` if that matters more than queue latency. SF9 and below stay inside it in every band. |
| `s.lora.<n>.ifac_netname` | `""` | IFAC network name. Empty = open (non-IFAC) interface. |
| `s.lora.<n>.ifac_size` | `0` | IFAC access-code length in bytes (`0` = rnsd default). |
| `s.lora.<n>.adaptive_txpwr` | `0` | Transmit to each neighbour node at a power measured for it rather than at `tx_power`. With it on, any node heard recently that has no power yet is probed once (`lora rf`), and the result — or, if the probe found nothing, the `EST` reciprocity estimate plus 5 dB — becomes the power for every hash that node owns. Also enables the **power request** (`0x04`): opening a link to a peer that speaks our air protocol prefixes a 4-byte frame suggesting the power it should answer at, which then governs the whole session; absence of that frame means "use your maximum". Announces always go out at `tx_power`. Answering someone else's probe does not need this key, but *honouring* a request does — it puts your power under a peer's control. A **first slice**: no control loop, nothing re-measures, nothing yet notices a link that broke. Shown as `USE` in `lora n`. See INTERNALS §15. |
| `s.lora.assumed_peer_txp` | `22` | TX power (dBm) credited to a peer whose own power we don't know, for the reciprocity estimate shown as `EST` in `lora n` and compared against the measurement in `lora rf`. Assuming high errs safe (we over-estimate path loss and transmit higher than needed). Set it to match a bench node parked at a low `tx_power`, whose announces go out at *that* power — otherwise the estimate is off by the difference. |
| `s.lora.rnode.enable` | `0` | Expose this device to an RNS `RNodeInterface` client — see [Using the device as an RNode](#using-the-device-as-an-rnode). Live. |
| `s.lora.rnode.radio` | `0` | Which radio the RNode endpoint exposes. Changing it while a client is attached disconnects it. |
| `s.lora.rnode.serial` | `-1` | Serial port the endpoint claims: `0` = the console port, `1` = the second CDC port (only exists after `usb cdc`). `-1` = no serial. |
| `s.lora.rnode.tcp` | `0` | TCP port the endpoint listens on; `0` (the default) or `-1` = no TCP. **Set it to 7633 or leave it off** — an RNS client dials 7633 and nothing else, so any other value opens a port nothing can reach. |
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
| `lora.<n>.stats.{tx_bytes,rx_bytes,tx_frames,rx_frames,crc_err,split_rx_timeout,tx_dropped,rssi_last,snr_last}` | Traffic counters (`tx_dropped` = frames shed by the LBT timeout) and last-RX RSSI/SNR. Published only when a UI can read them — see `uiTelemetryWanted()`. |
| `lora.<n>.stats.{airtime_pct,cw_band}` | With `appc` on: percentage of the last ~15 s this radio spent transmitting, and the contention band (1–4) that percentage currently selects. Absent when `appc` is off. |
| `lora.<n>.packets.<ms>` | LoRaMon: one node per on-air frame, keyed by start-ms — a packed string `r\|rssi\|snr\|dur\|bytes\|type` (rx) or `t\|txp\|dur\|bytes\|type\|wait` (tx); `snr` is deci-dB, `type` is `0` Reticulum / `1` this straddle's own air protocol / `2` traffic from an attached RNode client, and `wait` is the ms the frame spent queued before its first bit went on air (radio held by a probe or linkage frame, a split still landing, then DIFS/backoff) — carried by the first frame of a burst only, and drawn in the viewers as a tick at the moment it queued plus a mid-height line running up to the bar. Written only while the LoRaMon app is open (`sys.stats.{web,lcd}_loramon`) and deleted past 1 h. See INTERNALS §12. |
| `lora.<n>.air1h.{rx,tx}` | Rolling one-hour airtime, **per mille**, per direction. The only airtime figure the device aggregates — viewers compute shorter windows from the frame records themselves. Updated at 1 Hz while a LoRaMon app is open; the underlying rollup runs regardless. |

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
lora [<n>] n[eighbors] [-v]   observed direct neighbours, one numbered block per
                              node: every hash it owns with its aspect and
                              announced name, then a capability line
                              ( TRANSPORT, ROAMING, XXX, TX <dBm>, EST <dBm>,
                                USE <dBm> )
                              — TX is what a probe measured, EST what
                              reciprocity infers from frames overheard, USE the
                              power adaptive_txpwr actually transmits at (`~`
                              = derived from EST, not measured).
                              -v adds identities, signal envelope, proof-based
                              link quality, last-hour traffic and link_ids.
                              Spelled either way; any prefix from `n` works.
                              (see INTERNALS §13)
lora [<n>] rf[probe] <dest>   two-way minimum-TX-power probe against one
                              cooperating neighbour (both ends must run this
                              firmware): one carrier-sensed opener, then a
                              fixed-time slot schedule climbing a 6 dB power
                              ladder from low power up, reporting each
                              direction's lowest heard power and an
                              SNR-interpolated cliff estimate. Also asks the
                              peer for its full hash set when it advertises
                              more hashes than we hold. <dest> is a full dest
                              hash or any prefix of it (4+ bytes are taken
                              as-is; 2-3 complete from the table), a node
                              number from the listing, or any unique substring
                              of an announced name.
                              Ceiling is the radio's own tx_power (see
                              INTERNALS §14).
lora <n> freq <MHz>           set carrier frequency (MHz in, stored as Hz)
lora <n> bw <kHz>             set bandwidth (kHz in, stored as Hz)
lora <n> sf <5..12>           spreading factor
lora <n> cr <5..8>            coding-rate denominator (5 = 4/5)
lora <n> txp <dBm>            TX power
lora <n> preamble <sym>       preamble length
lora <n> sync <word>          sync word (hex or decimal)
lora <n> mode <name>          interface mode
lora <n> lbt <0|1>            listen-before-talk on/off (carrier-sense before TX)
lora <n> appc <0|1>           adaptive contention window on/off (needs lbt on)
lora <n> rx_boosted_gain <0|1>  SX126x RX gain boost on/off
lora help | -h                command summary
```

The `freq`/`bw`/… subcommands write the matching `s.lora.<n>.*` key, which the
task picks up and re-applies live. Run any of these on-device through `spangap
cli "<command>"`.

`lora <n>` reports the channel-access regime in force on its own line — slot and
DIFS times, and with `appc` on, the current own-airtime percentage, the band it
selects and that band's window range.

## Listen-before-talk and APPC

Before any frame goes out, the radio must win the channel. Two things gate it:

- **`lbt`** (default on) is the carrier sense itself: sample channel RSSI, wait
  for an inter-frame quiet period, then count down a random backoff. Off means
  blind transmit.
- **`appc`** (default on, inert without `lbt`) decides *how long that random
  backoff is drawn to be*. With it off, the window starts small and doubles each
  time the channel is snatched away — it reacts only to collisions this radio
  personally lost, and resets after every success. With it on, the window is
  drawn from one of four bands selected by how much of the last 7.5–15 seconds
  this radio spent transmitting: 0–7 % stays in the lowest band, ≥78 % lands in
  the highest, where the backoff averages about eight times longer.

**APPC stands for adaptive p-persistent CSMA (carrier-sense multiple access), an
acronym coined here.** It is not accurate. Textbook p-persistent CSMA gates each
transmit opportunity behind a probability *p*; there is no such coin flip in
this code. What it implements —
copied parameter-for-parameter from [RNode
firmware](https://github.com/markqvist/RNode_Firmware), which is where every
number in it comes from — is an adaptive *contention window*, reaching the same
load-responsive politeness by sizing the backoff rather than by rolling dice.
The name is a label for the feature, not a description of the algorithm. RNode
does not call it anything; upstream it is just how CSMA works there.

Why size the backoff off *our own* transmit time rather than observed channel
busyness: that is also RNode's choice, and it holds up because every radio on a
congested channel is transmitting more, retries included, so own-airtime tracks
aggregate load closely enough to act on — and it costs nothing to measure, since
each frame's time-on-air is already computed. The full parameter set, the exact
band edges, and where this build knowingly departs from upstream are in
[INTERNALS §6a](INTERNALS.md).

## Using the device as an RNode

With `s.lora.rnode.enable = 1` a stock Reticulum `RNodeInterface` client attaches
to this device as if it were RNode hardware — over USB serial, over TCP, or both.
The client becomes a **third endpoint on the same radio segment**: it, the radio,
and this node's own `rnsd` all see the same traffic, and a packet arriving from
any one of them is presented to the other two. So a laptop can run its own
Reticulum stack over this radio while the device keeps running its own.

Pick which radio it exposes with `s.lora.rnode.radio` (default 0).

Both transports default to **off**, so enabling the endpoint on its own does
nothing until you name one — in particular it does not start listening on the
network.

**Over TCP** (on a build with the `spangap-net` straddle staged; without it there
is no network stack and only the serial door exists). Set `s.lora.rnode.tcp =
7633` (that exact number — see the table above) and put this in the client's
`~/.reticulum/config`:

```ini
[[Device RNode]]
  type = RNodeInterface
  interface_enabled = True
  port = tcp://192.168.1.50
  frequency = 868000000
  bandwidth = 125000
  txpower = 14
  spreadingfactor = 7
  codingrate = 5
```

The port number is not configurable on the client side — it dials 7633
regardless of what you write after the host — so give the host only.

**Over USB serial.** Set `s.lora.rnode.serial` to `0` (the console port) or `1`
(the second CDC port, which only exists after `usb cdc`), and point the client at
the device node:

```ini
[[Device RNode]]
  type = RNodeInterface
  interface_enabled = True
  port = /dev/ttyACM0
  frequency = 868000000
  bandwidth = 125000
  txpower = 14
  spreadingfactor = 7
  codingrate = 5
```

Claiming port 0 does not cost you the console straight away: on
USB-Serial-JTAG the port stays an ordinary console until a client actually
speaks, and the console comes back when the client leaves. Two things to know
about a claimed port, though:

- **esptool auto-reset stops working on it.** A host closing a serial port drops
  the same lines, in the same order, that esptool's reset sequence uses — so the
  arming is disabled while the port is claimed, or every clean client exit would
  reboot the device. Use the button, or release the port, to flash.
- **On a CDC port, any terminal that opens it is treated as a client.** DTR is
  the only attach signal CDC gives us. Claim port 1 (after `usb cdc`) if you want
  to keep a usable console alongside.

**One client at a time**, across both transports. A second one is refused; a
serial client cannot take a port over while a TCP client is attached.

> **The client's radio settings overwrite yours, and they persist.** `frequency`,
> `bandwidth`, `txpower`, `spreadingfactor` and `codingrate` from the client's
> config are written to this device's `s.lora.<n>.*` keys — the same keys the
> settings pane edits — and survive a reboot. Point a client with a different
> channel at the device and the device moves to that channel, for `rnsd` too.
> Set them to what you actually want the radio on.
>
> `txpower` above 22 dBm is clamped, and the client checks the value it gets back
> against what it asked for. It will not accept the difference: it closes the
> connection and retries every 5 seconds, indefinitely. If a client never comes
> online, check `txpower` first.

Traffic the client originates is drawn **orange** in LoRaMon (the device's own is
yellow, this straddle's air protocol red), and `lora neighbors` grows a local
`rnode` row beside `us`:

```
lora/0 neighbors: 1 other and us + rnode, 0 open links (observing 4m)

  us    6b87eb8bdbcd51dee010c5a20fd65ef9 rnstransport.probe
  rnode 9a1c4f...                        lxmf.delivery  "laptop"
```

Details, and the protocol reasoning behind them, are in
[INTERNALS.md](INTERNALS.md) §17.

## Browser

The LoRa Settings panel (`browser/panels/LoraPanel.vue`, registered by
`modules/lora.ts`) edits radio 0 — band, bandwidth, SF, coding rate, TX power,
preamble, sync word, mode, the IFAC pair — and shows live state, chip, bitrate,
last RSSI/SNR, and frame counts. The on-device LoRa pane is generated from this
straddle's `settings:` block.

**LoRaMon** (`browser/panels/LoraMonWindow.vue`, and the same app on the device
LCD) is a launcher app rather than a settings pane: one graph per radio showing
every frame that went over the air, placed by transmit power or received signal
on a dBm axis and coloured by protocol, over a window from ten seconds to an
hour. Drag across the plot to zoom into a span; the back pill leaves it again.
Recording only runs while one of the two apps is open, and the graph starts
empty — see [INTERNALS §12](INTERNALS.md).

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
