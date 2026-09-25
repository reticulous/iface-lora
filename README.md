# iface-lora — LoRa interface for Reticulum

**iface-lora** is the LoRa interface for [rns](../rns): it carries Reticulum
packets over a LoRa radio. It drives **any RadioLib LoRa chip** — the SX126x
family (SX1261/2/8, LLCC68), SX127x / RFM9x (SX1272/6/7/8), SX128x (2.4 GHz),
LR11x0 (LR1110/20/21) and LR2021 — and can run **up to four radios** off one
shared SPI bus, each registering with `rnsd` as its own interface `lora/0`,
`lora/1`, … A single task services every radio; the loop is chip-agnostic and
only the per-chip bring-up dispatches by family.

On ESP-IDF's Linux host target there is no SPI bus: `src/host/virtual_hal.cpp`
is RadioLib's HAL over SIMesh's chip library (`SIMesh/radio`, which must sit
beside this straddle in the workspace), a model of an SX1262 with a UDP link
to a virtual medium, and the driver above runs unchanged. That is the
simulated testbed — see [`SIMesh`](../SIMesh/README.md).

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

Where the optional [netgraph](../netgraph) straddle is in the build, it also
tells the network graph what a radio *is*: `netgraphContributeIface` registers
the tail of this class's `if` line — `<MHz>|<SF>|<kHz>|<CR>`, plus `|s` where
SUPE is on — so a node reading the community's graph can see the settings behind
a LoRa link it will never hear itself. Fields only; netgraph composes the line
and this straddle never sees a record. **Configuration only**: RSSI, negotiated
rate steps and counters move constantly, and a record that moved with them would
keep every digest in the community permanently mismatched. The whole
contribution sits behind `CONFIG_STRADDLE_NETGRAPH` and compiles away without
it. See
[netgraph/README](../netgraph/README.md#an-interface-contributes-its-own-fields).

## Which builds have it

**The radio rides on the board.** A board straddle that carries a modem
additional-installs iface-lora — the same way a board with a screen stages the
on-device UI — so no buildable lists it and a board without a radio never stages
it. On such a board there is no LoRa settings section, no `s.lora.*` in storage
and no LoRaMon tile: nothing offers an operator a radio the hardware has not
got. What that board still has is LoRa's colour on its network graph, which
`rnsd` publishes itself (see [rns/README](../rns/README.md#status-line-pills)) —
the community's LoRa links are drawn on a node that will never hear one.

The board's edge is **gated on [rns](../rns)**: this is a Reticulum interface
before it is a radio driver, so a buildable that is not a Reticulum node takes
the board's modem with it only when the mesh is in the build too. Build such an
image on a radio board and the modem stays unclaimed — and the board's
`CONFIG_LORA_*` values, gated the same way, are not applied at all.

iface-lora **starts automatically** when the straddle is in the build and at
least one radio is configured (`CONFIG_LORA_COUNT > 0`). Staged onto a board
that declares no radio (`spangap build --with reticulous/iface-lora`) it keeps
`CONFIG_LORA_COUNT = 0`, does nothing and RadioLib is linked out.

## Configuring radios (Kconfig)

Pins and per-radio chip type come from this straddle's own Kconfig (`LORA_*` /
`LORAn_*`), resolved from the buildable's `sdkconfig`. A board straddle sets
them in its `kconfig:` block — the same block that stages this straddle; a build
can override with `spangap build --lora-count N` and the matching `--loraN-*`
switches, or interactively via `spangap menuconfig`.

- **`CONFIG_LORA_COUNT`** (0–4) — how many radios to drive. `0` = inert, which
  is what a board that declares no radio leaves it at.
- **`CONFIG_LORA_NO_SUPE`** — build without **SUPE**. A build flavour rather than
  a board fact, so it is given per build: `spangap build --kconfig
  CONFIG_LORA_NO_SUPE=y`. The engine, its frames, its console command, its
  settings and its keys in storage all go, and what ships is a plain LoRa
  interface carrying Reticulum on the configured channel. The frequency-agility
  channel plan and the airtime ledger go with it — they exist to serve the
  schedule SUPE negotiates — leaving the configured carrier as the only channel.
  A node built this way neither speaks nor answers SUPE and its neighbours are
  unaffected, since SUPE leaves non-participants unmodified and unaware. See
  INTERNALS §19.1.1.
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
  - `CONFIG_LORAn_FEM_PWR_PIN` / `_FEM_EN_PIN` / `_FEM_TXSEL_A_PIN` /
    `_FEM_TXSEL_B_PIN` — a front-end module on MCU GPIOs, **detected** at boot:
    `_FEM_EN_PIN` doubles as the sense that picks between the two supported
    parts. `-1` on the enable pin means no such FEM.
  - `CONFIG_LORAn_FEM_GAIN_DB` — a front-end module the board **declares**
    instead, in dB of TX gain (`0` = none). For a part the MCU cannot reach
    because its control lines hang off the radio's own DIOs: there is nothing to
    sense, so the board states the gain and `CONFIG_LORA_TX_POWER_MAX` states
    the limit, and the driver never asks the chip for more than the difference.
  - `CONFIG_LORAn_FEM_HF_PWR_PIN` / `_FEM_HF_GAIN_DB` — the same for a
    **dual-band** board's 2.4 GHz front end. `CONFIG_LORAn_FEM_PWR_PIN` is then
    the sub-GHz one's supply gate, and the driver raises whichever the
    configured carrier needs (see the limit note below).
  - `CONFIG_LORAn_LR_IRQ_DIO` — **LR2021 only**: which of the chip's DIO5..DIO11
    is wired to `_DIO1_PIN` (default 5, RadioLib's assumption). A board that
    bonded a different one and does not say so gets a radio that comes up, calls
    itself healthy and never reports a frame.
  - `CONFIG_LORAn_LR_RFSW_IDLE` / `_RX` / `_TX` / `_RX_HF` / `_TX_HF` —
    **LR2021 only**: the chip's internal RF-switch table, one bitmask per mode
    over its own DIOs (bit 0 = DIO5 … bit 6 = DIO11). A DIO named by any of the
    five is programmed as an RF-switch output and driven high in exactly the
    modes whose mask holds it; all five zero means no radio-driven front end.
    The chip applies the row itself on every mode change, so nothing on the host
    follows a transmit.
  - **chip** — the radio part on the slot (`SX1262` default); the choice covers
    all 15 supported parts.

- **Antenna limit** — `CONFIG_LORA_TX_POWER_MAX` (default 22) and
  `CONFIG_LORA_TX_POWER_MAX_HF` (default 12): a front end's board rating below
  and above 1500 MHz. It caps whatever the board's own calibration says the
  hardware reaches, so the published limit is the tighter of the two.

- **Calibration** — `CONFIG_LORAn_TX_CAL` and `CONFIG_LORAn_RSSI_CAL`: what the
  board actually radiates at the connector for a given register setting, and
  what its front end adds on receive. Curves rather than gains, one per part the
  board can carry, each carrying its own provenance. See INTERNALS §4b; a board
  that states none is uncalibrated and says so in `lora.<n>.cal`.

  The pair is separate because the two paths are: different amplifiers,
  different gains, different antenna limits, and a different drive range on
  the chip itself (the LR2021 accepts −9…+22 dBm on its sub-GHz port and
  −19…+12 on its 2.4 GHz one). **Everything follows the carrier**: tuning a
  dual-band radio across 1500 MHz moves the supply gate, the calibration and
  both ends of the range together, re-clamps `tx_power` into the new range with
  a warning, and republishes `lora.<n>.tx_power_{min,max}` so the power control
  re-sizes itself.
  A single-band board never sees any of it.

Between them these forms cover every antenna path seen so far: nothing (the chip
drives its own switch), SX126x's DIO2, two MCU GPIOs, a detected MCU-driven
front-end module, and — on the LR2021 — a table the radio applies to its own
DIOs on the board's behalf.

## Storage variables

Settings live under `s.lora.<n>.*` per radio (writable by the user, the browser
panel, and the LCD pane); runtime state and telemetry are published under
`lora.<n>.*` for anything to observe. Replace `<n>` with the slot index. Radio 0
defaults come from this straddle's `settings:` block; radios 1.. are seeded by
`loraInit`.

Four of the telemetry keys below — `packets.<ms>`, `peers.<slot>`, `rssi` and
`air1h.{rx,tx}` — exist for the [loramon](../loramon) viewers and for nothing
else. They are absent from a build made with `--without loramon`, not merely
idle in one: the code that writes them is gated on `CONFIG_STRADDLE_LORAMON`.
Each row says so.

**Changes settle for ten seconds before they are applied.** Editing a radio is
rarely one write — a frequency, a bandwidth and a spreading factor arrive
seconds apart as you work down the pane — and between them the radio would be
configured for a combination you never asked for. So the apply waits until ten
seconds after the *last* change, and **the radio stays off the air for the whole
window**: nothing is queued out, no SUPE channel switch is started, no announce is sent,
and a manual `lora <n> tx` is refused. Receiving is unaffected, a frame already
in flight is not cut off, and a SUPE transaction already running is left to
finish. The wait is capped at a minute from the first change, so a device being
written to continuously still applies. An attached RNode client is the exception
— its configuration burst ends in an explicit `RADIO_STATE`, which applies at
once, because the client is holding a 0.25 s validation window open.

### Settings (read)

| Key | Default | Meaning |
|---|---|---|
| `s.lora.<n>.enable` | `0` | Bring this radio up. Live — toggling it starts/stops the radio. |
| `s.lora.<n>.frequency` | *(none)* | Carrier frequency in **Hz**. No default — region/antenna dependent, user must pick. |
| `s.lora.<n>.bandwidth` | `125000` | Bandwidth in **Hz** (125/250/500 kHz; SX128x also 203/406/812/1625 kHz). |
| `s.lora.<n>.spreading_factor` | `7` | Spreading factor. The top is 12; the bottom is the **part's**, published as `lora.<n>.sf_min` and enforced on both surfaces — 5 on everything except the SX127x, which reaches neither SF5 (the chip has none) nor SF6 (it wants an implicit header, and this interface's frames are variable length), so its floor is 7. A lower value is clamped with a warning rather than refused. |
| `s.lora.<n>.coding_rate` | `5` | Coding-rate denominator, 5–8 (`5` = 4/5). |
| `s.lora.<n>.tx_power` | `lora.<n>.tx_power_max` | **Maximum** TX power in dBm at the **antenna connector**, within `lora.<n>.tx_power_min` … `lora.<n>.tx_power_max` (a bare chip sub-GHz reaches 21, 12 at 2.4 GHz; a front-end module raises both ends — the Heltec V4 runs roughly 7…27, the Meshnology W12 to 30 sub-GHz / 20 at 2.4 GHz). Seeded once, when the radio first learns its range, with the ceiling that range gives — the board's own claim about its antenna. Unset would read as 0 dBm, which is 22 dB down on a bare part and not even reachable on an amplified one, and it leaves the settings field blank. A power already set is never overwritten. `lora <n> txp max` puts it back. A value outside the range is clamped with a warning at either end; the floor is not a formality, since an amplified board cannot transmit below its amplifier's output. It is a limit and not a level: with SUPE on, a frame to a peer whose signal has been measured goes out at whatever reaches it, which `apDerive` clamps into this range and is often well below (§15). With SUPE off nothing derives anything and every frame goes out at exactly this. |
| `s.lora.<n>.preamble` | `12` | Preamble length in symbols, 6–32. |
| `s.lora.<n>.sync_word` | `"0x42"` | Sync word, a string parsed as hex or decimal (`0x42` is the Reticulum-on-LoRa convention). |
| `s.lora.<n>.announce_interval` | `30` | **Minutes** between everything this node says about itself on this radio, and the one answer to that question: the Reticulum announces `rnsd` replays onto `lora/<n>` (pinned to it — no other interface spends airtime), and SUPE's own ANNOUNCE, the frame publishing this node's identity hashes, what its radio can do and the power the frame went out at. Jittered ±10 %, so a fleet powered up together drifts apart. `0` turns the tick off entirely: the node then says who it is only when an application changes what it advertises, when the pane's **Announce now** is pressed, or on `lora [<n>] a`. It is not a longer interval, it is none. Relayed announces are somebody else's traffic and are unaffected. Live — no radio cycle. |
| `s.lora.<n>.mode` | `"access_point"` | RNS interface mode: `full`, `gateway`, `access_point`, `roaming`, `boundary`. It sets how long a route learned over this radio is kept and whether the node discovers paths on behalf of others; it does not decide whether traffic is forwarded, which is `s.rnsd.transport_enabled` alone. |
| `s.lora.<n>.lbt` | `1` | Listen-before-talk: CSMA/CA carrier-sense before each transmit. `0` = blind transmit (no sensing). Live. On a quiet, single-node band you can turn it off to skip the sensing; on a shared band leave it on. |
| `s.lora.<n>.appc` | `1` | Adaptive p-persistent CSMA — see below. Sizes the random backoff by how much of the recent past *this radio* spent transmitting, instead of growing it on collisions. Only has an effect while `lbt` is on. `0` reverts to the exponential-backoff channel plan. Live. |
| `s.lora.<n>.agc_reset` | `300` | Seconds between recalibrations of the SX126x analog front end (`0` = off; SX126x only). An SX126x that has heard a strong signal can leave its receive gain latched at that setting and stop hearing, and neither standby nor a fresh receive resets it — only powering the front end down does. This tick does that, plus a full block calibration, on an otherwise idle radio; a busy radio defers to the next tick. It is the one periodic wake the radio task holds without a consumer asking for it, which is why the period is minutes rather than the minute other firmwares use: the cost is a wake plus a few ms of chip work, and the bound it buys is how long a latched receiver can stay deaf. |
| `s.lora.<n>.fem_rx_lna` | `1` | Keep the board's external receive amplifier (the front-end module's LNA) in the RX path. It is the front end's standing cost — about 8 mA for as long as the radio listens, more than the SX1262 itself — bought for about 20 dB of gain ahead of the chip. `0` routes reception round it: a solar or small-battery node may prefer the range loss. Only a **KCT8103L** front end (Heltec V4.3) can switch its LNA out, so the row shows only where that part was detected; on a GC1109 board (V4 ≤ 4.2) the LNA is always in line and the setting is inert. Live. |
| `s.lora.<n>.ifac_netname` | `""` | IFAC network name. Empty = open (non-IFAC) interface. |
| `s.lora.<n>.ifac_size` | `0` | IFAC access-code length in bytes (`0` = rnsd default). |
| `s.lora.<n>.community_radius` | `3` | Community Radius: nodes within this many hops on this radio are served — their announces kept and answered for (only a node still holding a neighbour's original announce bytes can answer a path request for it, and re-acquiring one costs ~1.5 s of airtime), searches run on their behalf, and only their announces are re-broadcast onto the air. `0` = endpoint: on-demand only. See `rns/README.md`. **Read at radio start and handed to `rnsd` with the interface registration**, which caches it for the life of that registration — so a change wants a reboot, not a radio cycle: `lora <n> down` / `up` leaves `rnstatus` reporting the old radius. Pick it before the mesh is one; on a network whose diameter exceeds it, the nodes past the edge are not reachable at all, since a path request is never re-forwarded on a single-radio node and an announce is the only thing that crosses. |
| `s.lora.<n>.SUPE.afa` | `0` | Frequency agility: the value **is** the channel plan number, not a flag, and it is the same number SUPE names. `0` is a channel plan with no channel plan — the configured frequency alone — so it reads identically as "no agility". `1` is the EU 863-870 MHz plan (nine 500 kHz channels under polite spectrum access). On its own the channel plan puts no channels up: `lora.<n>.chans` names the agile set only while SUPE is **actually running** on this radio, since only a SUPE channel switch ever leaves the calling channel and channel tracks that are empty by construction cannot be told from a quiet band. Setting a channel plan while SUPE is off — or while it is on in the pane but gated off on the radio by an access code or an expired protocol version — leaves a viewer showing the calling channel alone; `lora supe` says which. **These keys outlive the engine.** They are persisted config, so they stay in the stored tree across a reflash onto an image built `CONFIG_LORA_NO_SUPE=y` (every entry in the `stable` catalogue is), where nothing reads them: `show s.lora` prints `SUPE.enable = 1` and `SUPE.afa = 1`, the pane's SUPE section is absent, `publishChannels` lists the calling channel alone, and a monitor draws one graph. `lora supe` is not registered in such a build at all, which is the quickest way to tell the two apart. **This is where the channel plan is set** — it appears as "Channel plan" in the SUPE section of the settings pane, since that is where you would look for it, but the key is the interface's own and predates SUPE. See INTERNALS §18. Live. |
| `s.lora.<n>.SUPE.enable` | `0` | Speak **SUPE** on this interface: unicast traffic becomes exchanges, with `rnsd` unmodified and unaware. One short HAIL on the shared channel says what is waiting; the party it names answers — a turnaround later on the same channel under channel plan `0`, at two derived slots on a private channel under channel plan `1` — and the frames follow at the best rate the link supports. Under a channel plan every exchange's closing frame seeds the next, so a busy pair touches the shared channel once. Off means the radio behaves exactly as it did, which is what makes it one thing to turn off when comparing. What the exchanges can fly is `afa` above: channel plan `0` moves the spreading factor only, channel plan `1` moves frequency as well. **Inert on an interface with an access code** — IFAC masks the frame end to end, so the modem cannot read an address to match, and the boot log says so. Traffic to a peer that has not announced itself over SUPE is untouched, so a mixed segment needs no detection and no fallback. `lora [<n>] supe` shows what it has learned. See INTERNALS §19 and `plans/SUPE.md`. In development. **`lora [<n>] supe enable` / `disable` sets this**, and the radio acts on it where it is read rather than restarting: nothing about the modem changes, so it stays on the air, and it announces the change at once. A node with this off still **reads** SUPE — it keeps its picture of which neighbours speak it, what their radios can do and what the path loss is — and it announces that it does not speak it, which is what makes a neighbour drop its SUPE bit and go back to plain RNode framing. Silence would not do that: a node that stops speaking is indistinguishable from one that has gone away. |
| `s.lora.<n>.SUPE.sender_ident` | `1` | Name this node in every HAIL. Three bytes and one symbol group, and it gives up the protocol's default anonymity — a listener in radio earshot learns who is talking to whom, which no Reticulum header discloses. It is on because the **hail-back and the return leg depend on it**: the tag a HAIL carries is the *hailed party's* address, so a hailed party that cannot answer has nobody to hail back, and an unnamed hailer's traffic queued at the far end is indistinguishable from a stranger's, so the answer is never a GOT — every reply then buys its own exchange instead of riding the one already running. It also lets the far end file a link identifier our cargo creates against us rather than against nobody. `0` restores the anonymity and gives all of that up; either way this node still understands the longer frame from peers that send it. See `plans/SUPE.md` §4. In development. |
| `s.lora.assumed_peer_txp` | `22` | TX power (dBm) credited to a peer whose own power we don't know, for the estimate shown as `EST` in `lora n` — which is what the **power request** (0x04) asks a peer to transmit at, and nothing else: our own power toward a peer is never estimated (see INTERNALS §15.4). Assuming high errs safe. Set it to match a bench node parked at a low `tx_power`, whose announces go out at *that* power — otherwise the estimate is off by the difference. |
| `s.lora.rnode.radio` | `0` | Which radio the RNode endpoint exposes — see [Using the device as an RNode](#using-the-device-as-an-rnode). Changing it while a client is attached disconnects it. |
| `s.lora.rnode.serial` | `1` | The serial entry point: the endpoint rides the highest serial port that exists — the console port normally, the second CDC port after `usb cdc`. On by default because it costs nothing until a client speaks (see below). Live. |
| `s.lora.rnode.tcp` | `0` | The TCP entry point, on port 7633 — a switch, not a port number: an RNS client dials 7633 and nothing else. Off by default: a listener on the network is a decision, not a side effect. Live. |
| `s.lora.rnode.upnp` | `0` | **Accessible from internet** — ask the router to forward port 7633 in from the WAN, so a client outside the LAN can attach. The flag rides the endpoint's registration with `spangap-net` as `publicFacing`, and [upnp](../upnp) is what acts on it; the switch appears under the TCP entry point only in a build that stages upnp, and only while that entry point is open. Off by default — the other two entry points reach as far as the desk this device is on, this one reaches the whole internet. Live. |
| `s.lora.rnode.ble` | `1` | The Bluetooth entry point, when `reticulous/rnode-ble` is in the build (which owns it — see its README). Live. |
| `s.lora.version` | — | Internal defaults-seeding gate; not a user setting. |

A radio refuses to come up until `frequency`, `tx_power`, and a valid
SF/BW/CR/preamble are set; `lora.<n>.state` reads `unconfigured` until then.

### Runtime state & telemetry (written)

| Key | Meaning |
|---|---|
| `lora.<n>.up` | `1` when the radio is on-air, else `0`. |
| `lora.<n>.state` | `unconfigured` / `error` / `up` / `down` / `rnsd_unavailable`. |
| `lora.<n>.chip` | Detected chip name, e.g. `SX1262`. |
| `lora.<n>.sf_min` | Lowest spreading factor this slot's part can actually run through this framing — `5` everywhere, `7` on the SX127x. A board fact (the slot names the chip, the chip names the family), so it is published at init, long before the radio is touched, and both surfaces take the SF field's lower bound from it. |
| `lora.<n>.row_{rx_boost,agc_reset,fem_lna}` | Which of the pane's conditional rows apply right now — the `when_key` gates the settings block reads. Each folds two facts into the one truthy/empty value a gate is: what this slot's part can do (the chip family answers to boosted RX gain / AGC reset; the front end sensed on the pin can switch its LNA out), **and** whether the radio is switched on at all. Composed here rather than compared in each UI — a `when_key` is one key, and the firmware is where two facts become one gate. A receive control that does nothing is worse than an absent one, since it gets read as an explanation for whatever the radio is doing, and a radio that is off is the plainest case of that. |
| `lora.<n>.announce_now` | Command key: the pane's **Announce now** button. Asks `rnsd` to replay every hosted destination's announce onto this radio and sends SUPE's ANNOUNCE beside it. Self-clearing; written with edge semantics so a second press registers. |
| `lora.<n>.tx_power_max` | Connector-dBm limit this radio can actually reach **on the band it is tuned to**: the peak of the board's own calibration for the port, capped by a front-end module's board rating where there is one. The peak, not the value at the top register setting — a curve that turns over near saturation reaches more a step or two below it. Republished on every begin, so it follows a carrier across 1500 MHz. `tx_power` is clamped to it, and **both** the browser panel and the LCD settings pane take their power field's upper bound from it — so a FEM board whose part did not answer offers what the bare chip reaches rather than a figure it cannot deliver. |
| `lora.<n>.tx_power_min` | Connector-dBm **floor** for the band in use: what the lowest register setting actually radiates. On a bare board that is the chip's own floor, but an amplified one cannot be driven below its amplifier's output — around `+7` on the Heltec V4 against the chip's `−9` — so it has a quietest possible frame. `tx_power` is clamped up to it with a warning, the adaptive controller never asks below it, and the UI sliders take their lower bound from it. Without it a node settles on a power it cannot produce and then announces that power to its neighbours, who compute every path loss to it wrong by the difference. |
| `lora.<n>.cal` | How well the two above, and every level this radio reports, are actually known: `measured` (this board on an analyser), `datasheet` (derived from the part's published figures), or `none` (uncalibrated — the conversion is a flat gain or identity, and says so). Provenance is carried in the `LORAn_TX_CAL` entry beside the numbers themselves, so it cannot drift away from them. A UI showing an uncalibrated figure as though it were measured is the failure this exists to prevent. |
| `lora.<n>.bitrate_eff` | Effective bitrate registered with `rnsd`, bits/s (airtime-derived). |
| `lora.<n>.stats.{tx_bytes,rx_bytes,tx_frames,rx_frames,crc_err,split_rx_timeout,tx_dropped,rssi_last,snr_last}` | Traffic counters (`tx_dropped` = frames shed by the LBT timeout) and last-RX RSSI/SNR. Published only when a UI can read them — see `uiTelemetryWanted()`. |
| `lora.<n>.stats.{airtime_pct,cw_band}` | With `appc` on: percentage of the last ~15 s this radio spent transmitting, and the contention band (1–4) that percentage currently selects. Absent when `appc` is off. |
| `lora.<n>.packets.<ms>` | LoRaMon: one node per on-air frame, keyed by start-ms — a packed string `r\|rssi\|snr\|dur\|bytes\|type\|ch\|desc\|cast[\|tag[\|to\|subj\|hash]]` (rx) or `t\|txp\|dur\|bytes\|type\|wait\|ch\|own\|desc\|cast[\|tag[\|to\|subj\|hash]]` (tx); `snr` is deci-dB, `type` is `0` Reticulum / `1` this straddle's own air protocol, SUPE / `2` traffic from an attached RNode client / `3` a frame whose CRC failed, `bytes` is payload bytes — everything but SUPE carries a 1-byte seq/split header on air and has it stripped, SUPE carries none and is recorded whole, in **both** directions — and the last two are what the frame waited before its first bit went on air, split because they are different facts: `wait` is what the **channel** cost (DIFS/backoff against somebody else's traffic) and `own` is what **we** cost ourselves (the radio held by an announce replay or a SUPE channel switch, a split still landing, or a deliberate pre-offer delay). Both are carried by the first frame of a burst only, and drawn in the viewers as a tick where the frame first wanted the air, then a mid-height run up to the bar — **dotted for ours, solid for contention**, in that order, so the pair reads left to right as the frame experienced it. Conflated, a busy channel and a busy radio look identical, and only one of them is somebody else's fault. `desc` names what the frame is — a code rather than a string, since there may be thousands of these and the name belongs in the viewer's table rather than on every record. It is the Reticulum header read out, not just its packet-type bits: the context byte says what a packet is *for* and the destination type what it is, so a `PATH_REQUEST` is not `DATA` and the `PATH_RESPONSE` answering one is not an ordinary `ANNOUNCE`. Names are the defining protocol's own — RNS's all-caps constants verbatim, this straddle's frames under a `SUPE_` prefix — so a bar on the graph, a Reticulum log line and the source all say the same word. A frame that failed its CRC is named nothing at all, since nothing in it decoded; `cast` is who it was aimed at (`0` broadcast, `1` unicast for us, `2` unicast for somebody else, `3` ours by a link identifier), which the device has to decide because a viewer cannot — it turns on which addresses mean US, and that lives in the peer table. Being aimed at everyone is decided by what the frame IS (`loraMonIsBroadcast`) and never by an address lookup: Reticulum broadcasts in four shapes — an announce, the `PATH_REQUEST` that asks for one, the `PATH_RESPONSE` that answers it, and a tunnel synthesis — and a kind missing from that list reads as somebody else's unicast. The two control destinations carry no `tag` at all, since their address is a constant every node computes and names no node — and which is what both viewers colour by. `cast` and `tag` answer different questions and neither is derived from the other: an exchange frame is *with* the peer and *for* us, so resolving its tag against our own addresses would call our own traffic somebody else's. Broadcast is read off `desc`, a frame belonging to one of our exchanges is ours by construction, and only what is left is looked up. Then `tag` is the three bytes naming the node it concerns, present only where there is one to take, and inside a SUPE exchange naming the **peer the exchange is with** rather than the address on the frame, so a whole burst answers "who was this with" the same way. Together with `lora.<n>.peers.*` those are what let a bar on the graph say `SUPE_GOT · 88B · tdeck` on hover — the tag itself appears only where it resolves to nobody, since once there is a name the hex is the part nobody reads — and what puts a peer's name in a pill beside a whole burst once the view is zoomed wide enough to hold it. `to`, `subj` and `hash` are the **detail** fields, written only while a viewer's `detailed` toggle is on (`sys.stats.{web,lcd}_details`) and absent otherwise — which is why the tag's slot is written empty rather than omitted when they follow it. `to` is where the packet was going and how far it has come (`3f2a11 h2`), which `tag` does not answer: `tag` names the node *this hop* was addressed to, and on a packet in transport that is the relay. `subj` is what the packet is **about**, and what that means is given by `desc` — the address a `PATH_REQUEST` asks for, the packet hash a `PROOF` proves, the link id a `LINKREQUEST` creates, the aspect an `ANNOUNCE` serves, the far end of an established link — and is empty where the kind has nothing of its own to say. `hash` is the packet's **own** hash, six hex characters of it, on everything that is not itself a proof: it is the name a proof for that packet will carry, so a viewer can join the two (the browser draws a line between them). Its own field rather than a case of `subj` because the packets most often proven — a link's data — have a subject worth keeping as well. All of it is cleartext: the header, and the payloads of the packets that carry none. What a packet carries past that is encrypted and stays unsaid. Written only with [loramon](../loramon) staged, and then only while one of its apps is open (`sys.stats.{web,lcd}_loramon` — the web key counted only while that browser's link is up, `webrtc.up`, since a tab that vanished cannot lower its own); deleted past 1 h. See INTERNALS §12. |
| `lora.<n>.peers.<slot>` | The neighbourhood, one node per peer-table slot: `"<num>\|<supe>\|<tags…>\|<names…>\|<loss>\|<rssi>\|<snr>\|<q>\|<heard_s>\|<flags>\|<ratestep>"`, a field left empty where it is not known. `num` is the number `lora n` prints beside the node, so a viewer with no name to show falls back to the same `#4` the console does. Tags are the three-byte prefixes that resolve an address to this node, comma-joined and deduplicated; names are the first word of each announced LXMF name on its destinations. Published rather than derived, because the clustering lives in the peer table and nothing outside this straddle can rebuild it — a viewer sees frames, not the announces and proofs that grouped them. Written only with [loramon](../loramon) staged, and then only while a web reader says it is looking (`sys.stats.web_peers`, counted only while that reader's link is up — `webrtc.up`); deleted when a slot empties. Read by LoRaMon, which names the node behind a frame's tag; and there for graph views. On-device surfaces do not read it — they are in the same binary as the peer table and ask it directly (`loraNameForTag`), since serialising a fact so the same firmware can parse it back is a round trip for nothing. |
| `lora.<n>.meas.<slot>.{tags,name,loss_to,snr_to,txp_to,to_ts,loss_from,snr_from,peer_txp,from_ts,rssi,snr,txp,heard_ts}` | SUPE's measurements of one neighbour, per peer-table slot, republished every 15 s while the table holds anyone and a frame has moved since the last publication, and deleted when the slot empties. `tags` is the comma-joined six-hex prefixes the node answers to (its destination hashes, identities and node key) — a reader that holds a destination hash finds the node by the hash's first six characters; `name` is its announced first-word names. Each direction is a path loss with the reading it was measured from: `loss_to` is us→them in dB from the peer's own report of how our frame landed, with `snr_to` (dB×10) the signal-to-noise it reported, `txp_to` (dBm) the power that frame of ours went out at and `to_ts` the unix second of the reading; `loss_from`/`snr_from`/`peer_txp`/`from_ts` are the same for them→us, a frame heard here against the power the peer stated for it. `rssi`/`snr` (dBm, dB×10) are the LAST level heard from the node and `heard_ts` when — the newest reading rather than the best one, since an envelope describes a row's whole life and says nothing about where the link is now; `txp` is what we last transmitted to it at. A field is absent when it is not known — only a SUPE peer states a power, so a plain Reticulum neighbour carries no loss at all and `rssi`/`snr`/`txp` are the whole of what is known about it. Those three are always present; `txp` falls back to the radio's configured power, which is what every frame leaves at where no per-peer power has been decided (nothing sent yet, or SUPE compiled out). Read by lxmf (Ping, contact bars) and the web contact list. `lora n` prints the same readings with their ages. |
| `rns.pill.lora.*` | The top status line's LoRa pill (yellow `ffd400`, order 4, titled "LoRa"), written through rnsd: `L` and the number of other nodes heard, summed over every slot — a board with two radios still has one LoRa neighbourhood. Published while any slot is enabled, at 0 as readily as at 3, and taken down by the switch rather than by the tick (turning the last radio off stops the interface task). The pill carries the colour and order with it, but the palette entry behind them — those two plus the title — is `rnsd`'s, published from boot on every node, including the boards that stage no radio at all, because the network graph draws LoRa links between other nodes wherever it is read. See [rns/README](../rns/README.md#status-line-pills). |
| `lora.<n>.chans` | The channel list the channel plan puts in force: `<freqHz>,<bwHz>` per channel, `\|`-separated, index = channel, `0` = the configured (calling) frequency. A single entry means no agility, which is how a viewer knows not to draw the extra graphs. Rewritten on a config apply. |
| `lora.<n>.rssi` | The newest channel-RSSI sample set: `<ms>\|<ch0 dBm>\|<ch1 dBm>\|…`, one field per channel in `chans`, taken once a second **while a LoRaMon app is open** — with no viewer the radio is not sampled at all, so an idle node holds no per-second wake for it. The timestamp is in the value so a viewer can tell a fresh reading from a repeat; a channel that could not be measured this tick is an **empty field**, and a tick skipped entirely (the radio was busy, or the configured channel was not quiet enough to leave) republishes nothing at all — both read as gaps. Live only: no history is kept on the device. Absent, along with the sampling tick itself, without [loramon](../loramon) staged. See INTERNALS §18.3. |
| `lora.<n>.air1h.{rx,tx}` | Rolling one-hour airtime, **per mille**, per direction. The only airtime figure the device aggregates — viewers compute shorter windows from the frame records themselves. Updated at 1 Hz while a LoRaMon app is open; the underlying rollup runs whether or not one is, because the hour it covers is longer than a viewer is typically up. Both go with [loramon](../loramon) when it is not staged — nothing else reads the rollup. |

### Secrets

`s.lora.<n>.ifac_netkey` — the IFAC passphrase (a secret; never synced to
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
                              ( TRANSPORT, ROAMING, SUPE BUDGET <k>,
                                EST <dBm>, USE <dBm> )
                              — SUPE BUDGET is how far up the modulation rate
                              table this pair can actually go, from both nodes'
                              announced capabilities; BUDGET 0 is a real answer
                              and means there is no rung above hailing.
                              EST is what
                              reciprocity infers from frames overheard, USE the
                              power a frame to it goes out at — no
                              tilde when the peer itself reported what it heard
                              from us, `~` from a path loss measured the other
                              way round and assumed reciprocal.
                              With SUPE off on this radio there is no
                              derivation to report: every frame goes out at the
                              configured `tx_power` and USE says exactly that,
                              while EST is absent — the request it names rides
                              the protocol this radio is not speaking, so it
                              would be an estimate driving nothing. The path
                              losses stay: they are measurements, and they age
                              in place until something states a power again
                              (the peer's own announces keep them→us current).
                              Then one line per measured direction:
                              `us->them <dB> path loss, SNR <dB> @ tx <dBm>
                              (<mW>), <age> ago` from the peer's own report of
                              how our frame landed, and `them->us …` from a
                              frame heard here against the power the peer
                              stated for it. The loss leads because it is the
                              link's own property whatever either end
                              transmits at; the SNR says whether the link is
                              weak or merely quiet; the power is what the loss
                              was measured against, in watts beside the dBm.
                              A direction nobody has measured has no line.
                              A node outside the protocol states no power, so
                              it can have no loss in either direction — it gets
                              `last heard @ <dBm> / SNR <dB>, <age> ago`
                              instead, which -v adds for every node.
                              This device's own rows (and an attached RNode
                              client's) come last, under their own heading:
                              they are what the radio hears itself saying, not
                              who is out there, and in the numbered list `us`
                              reads as a neighbour. Those rows are built from
                              announces this radio was heard sending and are
                              never retired, so before printing one the listing
                              asks rnsd whether the address is still hosted:
                              an LXMF account handed to a proxy server drops
                              off, rather than the table going on claiming an
                              address this node no longer answers on.
                              Every row says when that hash was last heard —
                              which is what separates a neighbour from a memory,
                              so it is not detail and rides every medium's
                              listing alike.
                              A node nothing has been heard from for two hours
                              is dropped from the table altogether — off this
                              listing, out of the pill's count, withdrawn from
                              rnsd's neighbourhood — and the next frame from it
                              builds a fresh row. Links that have gone quiet
                              for ten minutes are likewise gone from the
                              listing — off the node's hash lines, out of the
                              header's open-link count, out of -v's link
                              section: nothing announces a link ending, so
                              silence is the only evidence there is.
                              -v adds the announce count, identities, signal
                              envelope, proof-based link quality, last-hour
                              traffic and link_ids.
                              Spelled either way; any prefix from `n` works.
                              A SUPE node also lists its `ident`: the
                              three-byte identity prefixes it answers to, which
                              is what a SUPE log line names a node by and what
                              a hail names its sender by. SUPE nodes
                              only — for anyone else these name a protocol they
                              do not speak.
                              (see INTERNALS §13)
                              This is the RICH view of the same nodes every
                              interface answers for through rnsd's shared table
                              (`tcp n`, `auto n`, `ble_if n`). The clustering is
                              handed over rather than kept: this radio sets
                              `rx_origin` and prefixes each announce it forwards
                              with the peer-table ROW that transmitted it, so the
                              shared neighbourhood — and NetGraph — group a
                              node's destinations exactly as this listing does. A
                              packet merely overheard carries no row and no
                              claim. What stays here is what does not fit a shape
                              every medium must fill: link ids, identity
                              prefixes, the negotiated rate step, the derived power.
lora [<n>] p[robe] <num>      one round trip to a neighbour, then the link it
                              measured: `rnprobe` to the node's
                              `rnstransport.probe` address — the hash every
                              node has and the one that answers a probe by
                              itself — narrated as it goes, and then what this
                              radio measured doing it, at the margin:

                                  us->them 57 dB path loss, SNR 12 dB @ tx -9 dBm (126 µW), 0s ago
                                  them->us 53 dB path loss, SNR 12 dB @ tx -9 dBm (126 µW), 0s ago

                              **Everything printed is from this probe.** A loss
                              line appears only where the probe's own exchange
                              produced it, and is then milliseconds old; a
                              reading older than the probe is left to `lora n`,
                              which dates what it prints. Each line carries the
                              power its reading was measured against, so the
                              level at either end follows by subtraction and
                              gets no line of its own.
                              Where there is no loss at all — a node that
                              states no power, or a radio with SUPE off — this
                              radio's own two halves ARE the measurement, and
                              they are printed instead:

                                  sent at -9 dBm (126 µW), heard back at -62 dBm / SNR 12.0 dB

                              the power the probe's frames radiated and how the
                              answer came back, neither of which needs the
                              protocol. A probe over another path transmitted
                              nothing here and was answered elsewhere, so it
                              prints neither half rather than a stale one
                              standing in.
                              The hashes and capabilities are `lora n`'s
                              business and are not repeated.
                              Only when a proof comes back. A probe that draws
                              nothing has said so already, and printing what
                              the radio knew beforehand under it would invite
                              that to be read as the probe's own result.
                              The two halves answer different questions, which
                              is why they are printed together. The round trip
                              says the node is reachable and what the whole
                              path costs; the lines say what the link itself is
                              doing. This is the one command that MAKES a
                              measurement rather than reporting the last one
                              that happened by.
                              The path is Reticulum's to choose, not this
                              radio's, and hearing a node is not the same as
                              having a path to it: with another interface
                              between the two nodes, or a transport node
                              relaying, the probe takes that instead. Anything
                              but one hop says so in a note — the round trip is
                              then that path's and not this link's, and
                              `rnpath <hash>` says which way it went.
                              Blocks the console up to ~15 s, which outlives the
                              ~5 s a `spangap cli` reply is framed for: an
                              answered probe over a quick link lands well
                              inside it, a slow or unanswered one is better run
                              from an interactive session, or its tail is cut.
                              A node that has not announced a transport probe
                              address (or whose announce this radio has not
                              heard) is named as such rather than probed at
                              something else.
lora [<n>] f[orget] <num>     drop a neighbour from the table — `<num>` is the
lora [<n>] f[orget] all       number this radio's `lora n` printed beside it,
                              `all` is every node out there (never this
                              device's own rows). EVERYTHING about it goes: its
                              hashes and identities, its link stubs, its signal
                              and path-loss measurements, the adaptive power's
                              learned offset and failure floor, its outstanding
                              proof expectations, whether it speaks SUPE and at
                              what budget, any hail owed to it or schedule held
                              with it, and the records published about its slot
                              (`lora.<n>.peers.<slot>`, `lora.<n>.meas.<slot>`).
                              rnsd is told the node is gone, so it leaves the
                              shared neighbourhood and NetGraph too. Nothing is
                              kept: a belief that survives this is exactly the
                              belief being corrected — a node that has moved,
                              been reflashed or been reconfigured is described
                              by a table that learned it before any of that.
                              The next frame from it builds a fresh row from
                              what that frame proves, which is also why this is
                              not a block: forgetting is not ignoring.
                              An exchange already on the air is left to finish —
                              it is a conversation the far end is timing
                              against, not a memory, and what it files on its
                              way out is a fresh measurement.
                              With no radio index, `all` takes every radio and a
                              number takes radio 0, since a number belongs to
                              one radio's listing.
                              The numbers are positions in that listing, not
                              names, so they close up behind a forget: read
                              `lora n` again before naming the next one.
lora [<n>] a[nnounce]         repeat every announce this node originated, then
                              announcement — now, rather than on demand only.
                              Each announce takes the channel on its own like
                              any other frame; the run ends with this node's own
                              SUPE announcement. Announces are NOT buffered
                              against, batched or swallowed — they air when rnsd
                              hands them over — so this repeats what already
                              went out rather than releasing anything held
                              (see INTERNALS §14).
lora [<n>] supe               what SUPE has learned and decided on this radio
                              which channel plan is in force and what its steps
                              resolve to, when this build's protocol version expires
                              (a calendar date compiled in — past it a node
                              stops speaking SUPE by itself rather than speaking
                              a stale protocol version at a network that has moved on),
                              the tag set of addresses that mean us, anything
                              currently held for someone else's channel switch, and the
                              counters: offers out, offers answered, probes,
                              channel switchs completed, packets carried, packets
                              dropped as absent (see INTERNALS §19).
lora [<n>] supe e[nable]      speak SUPE on this radio, or stop. Sets
lora [<n>] supe d[isable]     s.lora.<n>.SUPE.enable and announces the change
                              at once, so neighbours are told rather than left
                              to infer it from silence. Disabled, the radio
                              still READS SUPE and keeps its picture of the
                              neighbourhood current; it just answers nothing.
lora [<n>] supe rx 0x<hex>    inject a frame into the receive path as if the
                              radio had decoded it. Paste a line's second column
                              from esp-idf/test/golden.txt, which the host tests
                              regenerate in exactly this form — never
                              hand-written, or the codec and the test can drift
                              apart. This is how the receive side is exercised
                              without a second device.
lora <n> freq <MHz>           set carrier frequency (MHz in, stored as Hz)
lora <n> bw <kHz>             set bandwidth (kHz in, stored as Hz)
lora <n> sf <5..12>           spreading factor
lora <n> cr <5..8>            coding-rate denominator (5 = 4/5)
lora <n> txp <dBm|max|min>    TX power. `max` and `min` are this board's own
                              ends of the range, so a script need not carry a
                              number that is right for only one board.
lora <n> preamble <sym>       preamble length
lora <n> sync <word>          sync word (hex or decimal)
lora <n> mode <name>          interface mode
lora <n> lbt <0|1>            listen-before-talk on/off (carrier-sense before TX)
lora <n> appc <0|1>           adaptive contention window on/off (needs lbt on)
lora <n> rx_boosted_gain <0|1>  RX gain boost on/off (SX126x, LR2021)
lora <n> fem_rx_lna <0|1>     front-end receive LNA in/out of the RX path
                              (KCT8103L only; off saves ~8 mA, loses ~20 dB)
lora <n> tx <string>          blind-transmit <string> as one explicit-header
                              frame at the radio's configured params. `0x<hex>`
                              inserts raw bytes (`0x0a`, `0x48656c6c6f`);
                              everything else is literal ASCII, spaces included.
                              Up to 255 bytes. No carrier-sense.
lora <n> tx_psa <string>      same as tx, but runs the normal listen-before-talk
                              carrier-sense first (honours lbt / appc; gives
                              up after 10 s). "psa" = polite-send-after-sense.
lora <n> tx_prot <ms>         emit an explicit header that announces a long 4/8
                              packet, then cut the carrier before its body — every
                              explicit-header receiver on the channel commits its
                              RX window for ~<ms> while the air only carries the
                              preamble + header. <ms> is the post-header commit
                              time; its limit is the max-length (255 B) 4/8
                              packet at the current SF/BW (~606 ms at SF7/BW125).
lora help | -h                command summary
```

`tx`, `tx_psa` and `tx_prot` are bench/test verbs: `tx`/`tx_psa` put arbitrary
bytes on the air over the live channel, and `tx_prot` is a receiver-capture
primitive — it exploits the LoRa rule that a valid header commits every listener
on that freq/BW/SF/sync-word to receive for the whole announced packet duration,
whether or not the body follows. The header is built by the chip in normal
explicit mode (so its length/CR/CRC fields and header CRC are spec-correct); only
the promised body is withheld. The command reports the actual committed time and
announced length, which may be below the requested `<ms>` when it exceeds what a
255-byte announce reaches at the current SF/BW.

The `freq`/`bw`/… subcommands write the matching `s.lora.<n>.*` key, which the
task picks up and re-applies live. Run any of these on-device through `spangap
cli "<command>"`.

`lora <n>` reports the channel-access channel plan in force on its own line — slot and
DIFS times, and with `appc` on, the current own-airtime percentage, the band it
selects and that band's window range.

## Seeing what is on the air

Two levels, and the split is deliberate:

- **`log lora debug`** — decisions. Config applies, channel-access stalls, and
  every SUPE action: offers with the channel and step that were chosen and what
  chose them, probes and their outcomes, holds, absence decisions, retunes,
  deadline expiries, announcements, power moves. A channel switch reads as a short story.
- **`log lora verbose`** — the above *plus* one line per on-air frame:
  direction, length, channel, airtime, power or rssi/snr, and the wait it paid.

So **a quiet `log lora debug` does not mean nothing is transmitting** — it means
nothing is deciding. Use verbose, or `lora <n>` (whose `tx_frames`/`tx_bytes`
counters are independent of logging), or open [LoRaMon](../loramon). The
per-frame verbose line is the one part of the recorder that survives a
`--without loramon` build — a frame log stands on its own.

Note also that an application asking to announce produces **no RF of its own**.
`RNSD_DEST_ANNOUNCE` sets that destination's stored announce with `rnsd`; the
bytes reach this radio on its own `announce_interval` tick, when somebody
presses the pane's **Announce now**, or in the sweep `rnsd` runs a minute after
any application changes what it advertises. So an `lxmf announce` shows up here
within the minute rather than at once — see INTERNALS §14 and
[rns/INTERNALS.md §4.1](../rns/INTERNALS.md). What DOES go out immediately is a
frame the radio originates: `lora [<n>] a[nnounce]` replays this radio's own
buffer of announces it has already carried.

## Listen-before-talk and adaptive backoff

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

**What `appc` implements is an adaptive contention window** — copied
parameter-for-parameter from [RNode
firmware](https://github.com/markqvist/RNode_Firmware), which is where every
number in it comes from. It reaches the load-responsive politeness that
p-persistent CSMA (carrier-sense multiple access) gets from a per-attempt
probability by sizing the backoff instead; there is no coin flip in this code.
RNode does not call it anything; upstream it is just how CSMA works there.

Why size the backoff off *our own* transmit time rather than observed channel
busyness: that is also RNode's choice, and it holds up because every radio on a
congested channel is transmitting more, retries included, so own-airtime tracks
aggregate load closely enough to act on — and it costs nothing to measure, since
each frame's time-on-air is already computed. The full parameter set, the exact
band edges, and where this build knowingly departs from upstream are in
[INTERNALS §6a](INTERNALS.md).

## Using the device as an RNode

A stock Reticulum `RNodeInterface` client attaches to this device as if it were
RNode hardware — over USB serial, over TCP, over Bluetooth if
`reticulous/rnode-ble` is in the build, or any combination. One client at a
time, whichever gets there first.
The client becomes a **third endpoint on the same radio segment**: it, the radio,
and this node's own `rnsd` all see the same traffic, and a packet arriving from
any one of them is presented to the other two. So a laptop can run its own
Reticulum stack over this radio while the device keeps running its own.

Pick which radio it exposes with `s.lora.rnode.radio` (default 0). Each entry point is
its own switch: serial and Bluetooth are on by default (a dormant entry point costs
nothing until a client speaks), TCP is off — nothing listens on the network
unless asked.

**Over TCP** (on a build with the `spangap-net` straddle staged; without it there
is no network stack and only the serial entry point exists). Set `s.lora.rnode.tcp = 1`
(the port is 7633, fixed on both ends) and put this in the client's
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

To reach the entry point from outside the LAN, set `s.lora.rnode.upnp = 1` as well
(**Accessible from internet**, under the TCP switch in the pane): the router is
then asked to forward 7633 in. It needs the [upnp](../upnp) straddle in the
build and a router that honours the request; the client's host then becomes the
device's public address rather than its LAN one.

From inside the build container the entry point is reachable at
`host.docker.internal:7633` — the workspace bridge fronts the port because the
buildable's straddle.yaml lists it in `bridge_ports:`. That is what
[`esp-idf/test/rnode_test.py`](esp-idf/test/rnode_test.py) uses: an
unattended integration test that opens the TCP entry point over `spangap cli`, runs
the raw KISS detect handshake on the entry point, then attaches the reference Python
RNS stack as a real `RNodeInterface` client and proves a probe round trip in
each direction (client → device `rnstransport.probe`, then device `rnprobe` →
the reference node). It mirrors the device's own radio parameters into the
client config, so a run does not move the radio's channel. With the console on
CDC and the host monitor fronting the spare CDC port (`spangap monitor
<cdc0-dev> --aux <cdc1-dev>`), the same test also exercises the **serial
entry point** — the raw detect and a serial `RNodeInterface` probe through the aux
relay, attaching via the in-band trigger since a relay carries no line state.

**Over USB serial.** On by default (`s.lora.rnode.serial`): the endpoint
listens on the highest
serial port the device presents — the console port normally, the spare CDC port
after `usb cdc`. The port stays a fully ordinary console until a client's first
KISS FEND byte (`0xC0`, which no keystroke produces) arrives in the stream; that
in-band trigger takes the port over, and the console returns when the client
leaves. Point the client at the device node:

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

The in-band trigger is what makes the standing claim free: opening the port in
a terminal changes nothing, typing changes nothing, and esptool auto-reset stays
armed — the console only leaves once a client actually speaks KISS, and it comes
back when the client detaches. Two things to know about an **attached** session,
though:

- **esptool auto-reset is suspended while a client is attached.** A host closing
  a serial port drops the same lines, in the same order, that esptool's reset
  sequence uses — so the arming pauses for the session, or every clean client
  exit would reboot the device. It returns the moment the session ends.
- **On USB-Serial-JTAG there is no line state**, so the device cannot see a
  client close the port: the session ends on the handler's own disconnect or
  `usb down`. On CDC the client's DTR drop releases it.

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

Traffic the client originates is drawn like any other transmit in LoRaMon — the
radio put it on the air, so the graph says so, and clicking a frame names it
`RNode` — and `lora neighbors` grows a local `rnode` row beside `us`:

```
lora/0 neighbors: 1 other, 0 open links (observing 4m)
  table 3 of 64 rows; 0 evicted, 0 gone silent

  …

this device (and the attached RNode client):

  us    6b87eb8bdbcd51dee010c5a20fd65ef9 rnstransport.probe
  rnode 9a1c4f...                        lxmf.delivery  "laptop"
```

Details, and the protocol reasoning behind them, are in
[INTERNALS.md](INTERNALS.md) §17.

## Browser

The LoRa Settings panel edits radio 0 — band, bandwidth, SF, coding rate, TX
power, preamble, sync word, mode, the IFAC pair — and shows live state, chip,
bitrate, last RSSI/SNR, and frame counts. Both it and the on-device LoRa pane
are generated from this straddle's `settings:` block; there is no hand-written
panel component. The **RNode endpoint** — which transports a client
may attach over — is its own menu under Reticulum Mesh rather than a section of
the radio pane: it is a way in to the radio, not a setting of it, and rnode-ble
contributes its Bluetooth switch and settings there.

This straddle has no browser half of its own: the settings pane is generated,
and **LoRaMon** — the per-on-air-frame graph, in the browser and on the device
LCD — is its own straddle, [loramon](../loramon).

## Dependencies

- [rns](../rns) — `rnsd` must be ahead of iface-lora in init order so
  `RNSD_PORT_IFACE` is open when a radio registers (`requires:` enforces it).
- [loramon](../loramon) — the LoRaMon viewers, staged by default
  (`additional_installs:`) and droppable with `--without loramon`. It is the
  only reader of the per-frame recorder here, so dropping it compiles the whole
  recording half out (`CONFIG_STRADDLE_LORAMON`): no per-frame nodes, no
  neighbourhood rows, no channel-RSSI series, no rolling hour, no 1 Hz sample
  tick, and no 16 KB-per-radio expiry FIFO. The radio is otherwise unchanged,
  and the stats, pill, channel-list and state keys stay — the settings pane and
  the status bar read those.
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
