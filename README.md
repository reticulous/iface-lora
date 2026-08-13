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
| `s.lora.<n>.tx_power` | *(none)* | TX power in dBm at the **antenna**, −9 to `lora.<n>.tx_power_max` (22 on a bare chip, higher through a front-end module — 27 on the Heltec V4). No default — antenna dependent. A value above the ceiling is clamped with a warning. |
| `s.lora.<n>.preamble` | `12` | Preamble length in symbols, 6–32. |
| `s.lora.<n>.sync_word` | `"0x42"` | Sync word, a string parsed as hex or decimal (`0x42` is the Reticulum-on-LoRa convention). |
| `s.lora.<n>.mode` | `"gateway"` | RNS interface mode: `full`, `gateway`, `access_point`, `roaming`, `boundary`. |
| `s.lora.<n>.lbt` | `1` | Listen-before-talk: CSMA/CA carrier-sense before each transmit. `0` = blind transmit (no sensing). Live. On a quiet, single-node band you can turn it off to skip the sensing; on a shared band leave it on. |
| `s.lora.<n>.appc` | `1` | Adaptive p-persistent CSMA — see below. Sizes the random backoff by how much of the recent past *this radio* spent transmitting, instead of growing it on collisions. Only has an effect while `lbt` is on. `0` reverts to the exponential-backoff regime. Live. |
| `s.lora.<n>.lbt_timeout` | `5000` | Drop a frame LBT can't clear within this many ms (`0` = never drop, block the queue instead). Note that a fully-loaded `appc` radio can legitimately back off for 5.2 s at SF10 and 5.8 s at SF11/SF12, past this default, so a congested slow link will shed frames — raise it or set `0` if that matters more than queue latency. SF9 and below stay inside it in every band. |
| `s.lora.<n>.agc_reset` | `300` | Seconds between recalibrations of the SX126x analog front end (`0` = off; SX126x only). An SX126x that has heard a strong signal can leave its receive gain latched at that setting and stop hearing, and neither standby nor a fresh receive resets it — only powering the front end down does. This beat does that, plus a full block calibration, on an otherwise idle radio; a busy radio defers to the next beat. It is the one periodic wake the radio task holds without a consumer asking for it, which is why the period is minutes rather than the minute other firmwares use: the cost is a wake plus a few ms of chip work, and the bound it buys is how long a latched receiver can stay deaf. |
| `s.lora.<n>.ifac_netname` | `""` | IFAC network name. Empty = open (non-IFAC) interface. |
| `s.lora.<n>.ifac_size` | `0` | IFAC access-code length in bytes (`0` = rnsd default). |
| `s.lora.<n>.retain_announces` | `1` | Keep the announces heard on this radio, not just forward them. On by default: this node is the sole custodian of the mesh on the other side, re-acquiring a neighbour costs ~1.5 s of airtime, and only a node still holding a neighbour's original announce bytes can answer a path request for it. |
| `s.lora.<n>.policy_manual` | `0` | Set this radio's transit policy by hand instead of inferring it from `mode`. Off = auto, which is stock behaviour and leaves `route_for` unread. Appears as "Set policy manually" in the settings pane. |
| `s.lora.<n>.route_for` | `0` | Read only when `policy_manual = 1`. `1` = we provide transport for the nodes on this radio: we relay announces towards them, we search on their behalf, and their paths get `s.rnsd.path.ttl_custody` rather than the short access-point lifetime. `0` = we still talk to them as an endpoint, we just don't work for them. Answering a path request for a destination we already know is never gated by this. See `rns/README.md`. |
| `s.lora.<n>.SUPE.afa` | `0` | Frequency agility: the value **is** the regime number, not a flag, and it is the same number SUPE names. `0` is a regime with no channel plan — the configured frequency alone — so it reads identically as "no agility". `1` is the EU 863-870 MHz plan (nine 500 kHz channels under polite spectrum access). On its own the regime only puts channels up to be **measured and drawn**; what transmits on them is SUPE, below. **This is where the regime is set** — it appears as "Regime" in the SUPE section of the settings pane, since that is where you would look for it, but the key is the interface's own and predates SUPE. See INTERNALS §18. Live. |
| `s.lora.<n>.SUPE.enable` | `0` | Speak **SUPE** on this interface: unicast traffic leaves the shared channel for short private high-rate detours, negotiated in one seven-byte frame, with `rnsd` unmodified and unaware. Off means the radio behaves exactly as it did, which is what makes it one thing to turn off when comparing. Which detours are possible is `afa` above: regime `0` moves the spreading factor only, regime `1` moves frequency as well. **Inert on an interface with an access code** — IFAC masks the frame end to end, so the modem cannot read an address to match, and the boot log says so. Traffic to a peer that has not announced itself over SUPE is untouched, so a mixed segment needs no detection and no fallback. `lora [<n>] supe` shows what it has learned. See INTERNALS §19 and `plans/SUPE.md`. Live. |
| `s.lora.<n>.SUPE.adaptive_txpower` | `1` | Transmit to each peer at a power measured for it rather than at `tx_power`. Covers both mechanisms: the reciprocity determination with its `0x04` power request (INTERNALS §15), and SUPE's own per-transaction form — `path loss + step margin + offset`, clamped to a learned floor and never above `tx_power` under any failure. **This replaced `adaptive_txpwr`**, which was the same switch under a name one character different; an existing setting is carried across on upgrade. |
| `s.lora.<n>.SUPE.sender_ident` | `1` | Name this node in every SUPE START. Three bytes and one symbol group, and it gives up the protocol's default anonymity — a listener in radio earshot learns who is talking to whom, which no Reticulum header discloses. It is on because the **reverse leg depends on it**: the tag a START carries is the *answerer's* address, so an unnamed requester's traffic queued at the far end is indistinguishable from a stranger's, the GRANT's reverse flag is never set, and every reply buys its own detour instead of riding the one already running. It also lets neighbours hold traffic for us while we are off on a detour, and lets the far end file a link identifier our cargo creates against us rather than against nobody. `0` restores the anonymity and gives those up; either way this node still understands the longer frame from peers that send it. See `plans/SUPE.md` §4. Live. |
| `s.lora.<n>.SUPE.announce_interval` | `30` | Minutes between this node's own SUPE announcements — the frame publishing its identity hashes, what its radio can do, and the power the frame went out at, so a listener turns its own reading into a path loss. It governs nothing a Reticulum announce does: those air when `rnsd` hands them over (INTERNALS §14). `0` turns the beat off entirely, so the node announces only when `lora a` says so; it is not a longer interval, it is none. Relayed announces are somebody else's traffic and are unaffected. **This replaced `announce_interval`**; an existing setting is carried across on upgrade. Live. |
| `s.lora.assumed_peer_txp` | `22` | TX power (dBm) credited to a peer whose own power we don't know, for the reciprocity estimate shown as `EST` in `lora n`. Assuming high errs safe (we over-estimate path loss and transmit higher than needed). Set it to match a bench node parked at a low `tx_power`, whose announces go out at *that* power — otherwise the estimate is off by the difference. |
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
| `lora.<n>.tx_power_max` | Antenna-dBm ceiling this radio can actually reach: `22` on a bare chip, the front-end module's rating on a board with one that was detected at boot. `tx_power` is clamped to it, and **both** the browser panel and the LCD settings pane size their power slider from it — so a FEM board whose part did not answer offers 22 rather than a figure it cannot deliver. |
| `lora.<n>.bitrate_eff` | Effective bitrate registered with `rnsd`, bits/s (airtime-derived). |
| `lora.<n>.stats.{tx_bytes,rx_bytes,tx_frames,rx_frames,crc_err,split_rx_timeout,tx_dropped,rssi_last,snr_last}` | Traffic counters (`tx_dropped` = frames shed by the LBT timeout) and last-RX RSSI/SNR. Published only when a UI can read them — see `uiTelemetryWanted()`. |
| `lora.<n>.stats.{airtime_pct,cw_band}` | With `appc` on: percentage of the last ~15 s this radio spent transmitting, and the contention band (1–4) that percentage currently selects. Absent when `appc` is off. |
| `lora.<n>.packets.<ms>` | LoRaMon: one node per on-air frame, keyed by start-ms — a packed string `r\|rssi\|snr\|dur\|bytes\|type\|ch` (rx) or `t\|txp\|dur\|bytes\|type\|wait\|ch\|own` (tx); `snr` is deci-dB, `type` is `0` Reticulum / `1` this straddle's own air protocol, SUPE (Spectrum Utilization and Performance Enhancements) — the name the viewers' legends use / `2` traffic from an attached RNode client, and note that a SUPE frame's `bytes` reads one lower on the receiving node than on the sending one (the receive path applies the split-header subtraction that transmit exempts our own protocol from), and the last two are what the frame waited before its first bit went on air, split because they are different facts: `wait` is what the **channel** cost (DIFS/backoff against somebody else's traffic) and `own` is what **we** cost ourselves (the radio held by an announce replay or a SUPE detour, a split still landing, or a deliberate pre-offer delay). Both are carried by the first frame of a burst only, and drawn in the viewers as a tick where the frame first wanted the air, then a mid-height run up to the bar — **dotted for ours, solid for contention**, in that order, so the pair reads left to right as the frame experienced it. Conflated, a busy channel and a busy radio look identical, and only one of them is somebody else's fault. Written only while the LoRaMon app is open (`sys.stats.{web,lcd}_loramon`) and deleted past 1 h. See INTERNALS §12. |
| `lora.<n>.chans` | The channel list the regime puts in force: `<freqHz>,<bwHz>` per channel, `\|`-separated, index = channel, `0` = the configured (hailing) frequency. A single entry means no agility, which is how a viewer knows not to draw the extra graphs. Rewritten on a config apply. |
| `lora.<n>.rssi` | The newest channel-RSSI sample set: `<ms>\|<ch0 dBm>\|<ch1 dBm>\|…`, one field per channel in `chans`, taken once a second **while a LoRaMon app is open** — with no viewer the radio is not sampled at all, so an idle node holds no per-second wake for it. The timestamp is in the value so a viewer can tell a fresh reading from a repeat; a channel that could not be measured this beat is an **empty field**, and a beat skipped entirely (the radio was busy, or the configured channel was not quiet enough to leave) republishes nothing at all — both read as gaps. Live only: no history is kept on the device. See INTERNALS §18.3. |
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
                              ( TRANSPORT, ROAMING, SUPE STEP <k>, TX <dBm>,
                                EST <dBm>, USE <dBm> )
                              — SUPE STEP is how far up the modulation ladder
                              this pair can actually go, from both nodes'
                              announced capabilities; STEP 0 is a real answer
                              and means there is no rung above hailing.
                              TX is what a probe measured, EST what
                              reciprocity infers from frames overheard, USE the
                              power SUPE.adaptive_txpower transmits at (`~`
                              = derived from EST, not measured).
                              -v adds identities, signal envelope, proof-based
                              link quality, last-hour traffic and link_ids.
                              Spelled either way; any prefix from `n` works.
                              (see INTERNALS §13)
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
                              which regime is in force and what its steps
                              resolve to, when this build's dialect expires
                              (14 days from the build — past it a node stops
                              speaking SUPE by itself rather than speaking a
                              stale dialect at a network that has moved on),
                              the tag set of addresses that mean us, anything
                              currently held for someone else's detour, and the
                              counters: offers out, offers answered, probes,
                              detours completed, packets carried, packets
                              dropped as absent (see INTERNALS §19).
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
lora <n> txp <dBm>            TX power
lora <n> preamble <sym>       preamble length
lora <n> sync <word>          sync word (hex or decimal)
lora <n> mode <name>          interface mode
lora <n> lbt <0|1>            listen-before-talk on/off (carrier-sense before TX)
lora <n> appc <0|1>           adaptive contention window on/off (needs lbt on)
lora <n> rx_boosted_gain <0|1>  SX126x RX gain boost on/off
lora <n> tx <string>          blind-transmit <string> as one explicit-header
                              frame at the radio's configured params. `0x<hex>`
                              inserts raw bytes (`0x0a`, `0x48656c6c6f`);
                              everything else is literal ASCII, spaces included.
                              Up to 255 bytes. No carrier-sense.
lora <n> tx_psa <string>      same as tx, but runs the normal listen-before-talk
                              carrier-sense first (honours lbt / appc /
                              lbt_timeout). "psa" = polite-send-after-sense.
lora <n> tx_prot <ms>         emit an explicit header that announces a long 4/8
                              packet, then cut the carrier before its body — every
                              explicit-header receiver on the channel commits its
                              RX window for ~<ms> while the air only carries the
                              preamble + header. <ms> is the post-header commit
                              time; its ceiling is the max-length (255 B) 4/8
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

`lora <n>` reports the channel-access regime in force on its own line — slot and
DIFS times, and with `appc` on, the current own-airtime percentage, the band it
selects and that band's window range.

## Seeing what is on the air

Two levels, and the split is deliberate:

- **`log lora debug`** — decisions. Config applies, channel-access stalls, and
  every SUPE action: offers with the channel and step that were chosen and what
  chose them, probes and their outcomes, holds, absence verdicts, retunes,
  deadline expiries, announcements, power moves. A detour reads as a short story.
- **`log lora verbose`** — the above *plus* one line per on-air frame:
  direction, length, channel, airtime, power or rssi/snr, and the wait it paid.

So **a quiet `log lora debug` does not mean nothing is transmitting** — it means
nothing is deciding. Use verbose, or `lora <n>` (whose `tx_frames`/`tx_bytes`
counters are independent of logging), or open LoRaMon.

Note also that **announces this node originates are buffered, not transmitted**
when `rnsd` or an attached client hands them over: they go out on the
`SUPE.announce_interval` beat, or immediately on `lora [<n>] a[nnounce]`. So an
`lxmf announce` produces no RF until one of those happens — see INTERNALS §14.

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
