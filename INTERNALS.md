# iface-lora — internals

Maintainer reference for the LoRa interface. The [README](README.md) is the
operator guide; this document is for changing the code without breaking it. It
is self-authoritative.

## 1. Everything this straddle adds

iface-lora is entirely additive — it sits on top of RadioLib and `rnsd`, and
contributes:

- **The RadioLib HAL** (`EspIdfHal`, `esp_idf_hal.{cpp,h}`) — RadioLib's
  GPIO/SPI/timing/ISR surface implemented on ESP-IDF, sharing the SPI bus through
  spangap-core's `spi_helper` (§3).
- **The LoRa interface task** (`lora.cpp`) — one FreeRTOS task driving every
  configured radio: bring-up, the on-air split framing (§5), the RX/TX paths
  (§6), `rnsd` registration (§7), stats publication, and the `lora` CLI.
- **A chip-agnostic X-macro dispatch table** (§4) — 15 RadioLib LoRa parts
  across 5 families (SX126x, SX127x, SX128x, LR11x0, LR2021), generating the
  chip enum, name table, family map, and constructor/`begin()` switches from one
  list, kept in lockstep with the Kconfig `choice` ordering.
- **Multi-radio support** — up to `CONFIG_LORA_COUNT` (≤4) radios on one shared
  SPI bus, each a separate RNS interface `lora/<slot>` (§7).
- **Airtime-derived bitrate** (§8) — the bitrate registered with `rnsd` is
  computed from real LoRa time-on-air, so RNS's link-establishment timeout
  tracks how long a frame actually takes.
- **Listen-before-talk with an adaptive contention window** (§6, §6a) — RSSI
  carrier sense plus a backoff whose size is set by this radio's own recent
  airtime, reimplementing RNode firmware's CSMA parameters; `s.lora.<n>.appc`.
- **IFAC plumbing** — reading `s.lora.<n>.ifac_netname` / `ifac_size` and the
  `secrets.lora.<n>.ifac_netkey` secret and handing them to `rnsd` in the
  `rnsd_iface_t` connect payload; `rnsd` does the actual access-code crypto.
- **LoRaMon** (§12) — a per-on-air-frame recorder whose storage subtree *is* the
  ring, plus the browser and LCD viewers that plot power, signal and protocol on
  a dBm axis with a click/touch zoom stack.
- **The passive neighbour table** (§13) — who is in RF range, built purely from
  observing rx + tx RNS traffic, with a cryptographic identity join; surfaced as
  `lora n[eighbors]`.
- **`lora rf[probe]`** (§14) — a two-node, fixed-time exchange that measures the
  lowest TX power that still closes a link **in both directions** in ~320 ms at
  SF7, and the 0x02/0x03 hash-linkage frames (§14.2) that let nodes tell each
  other which destination hashes are one device.
- **Adaptive TX power** (§15) — one power determination per neighbour node, and
  the tx-path lookup that transmits at it. A first slice: no control loop yet.
- **The RNode endpoint** (§17) — a stock RNS `RNodeInterface` client attaches
  over USB serial and/or TCP 7633 and becomes the third endpoint of the radio
  segment, executing radio commands by writing the ordinary `s.lora.<n>.*`
  keys.
- **The browser panel and generated LCD/web settings** (`browser/`, the
  `settings:` block in `straddle.yaml`).

## 2. The task

One FreeRTOS task — **priority 2, 10 KB PSRAM stack** (larger than other
interfaces for the LoRa frame buffers, RadioLib state, and the neighbour
table's inline Ed25519 announce verification, §13). It services *all*
radios; per-radio state lives in `s_radios[]` (`LoraRadio`). Its **core** is
`CORE_SECONDARY_NO_LCD` (`compat.h`): on a no-LCD build it runs on core 1,
opposite the `rnsd` it feeds on core 0, so their RX/processing bursts overlap and
both cores idle together for light sleep; an LCD build keeps it on the primary
(core 1 is busy rendering). See
[power-management: core placement](../spangap-core/docs/power-management.md#core-placement--overlap-for-light-sleep).

**Boot order.** There is no boot barrier in the task itself: the RNS
orchestrator spawns it (`rnsServiceRegister`, phase `RNS_PHASE_IFACE`) only once
`rnsd` is up and past its boot window, and `rnsd` resolved the clock before
declaring ready, so neither `rns.ready` nor the time is waited on again here. The
task opens its ITS server + client, opens the RNode endpoint's ITS port (§17.2),
builds the `app.aspect` name-hash dictionary, subscribes to `s.lora`,
`secrets.lora`, `sys.usb.serial_ports` and the per-radio MHz/kHz display keys,
then constructs each radio + HAL and probes for presence (§4).

**Single wait point.** `itsPoll(nextDeadline())` is the only blocking call. It
wakes on an ITS message (an outbound packet from `rnsd`, or a config-change
notify), a task notification from any radio's IRQ ISR, or a computed deadline.
When outbound bytes are queued and a radio is free, `nextDeadline` returns 0 to
drain on the next turn. With nothing pending — no queued outbound, no split-RX
in flight, no deferred stats flush, no unregistered radio, no config apply owed,
no probe running, no proof expectation outstanding, no viewer open — it returns
`portMAX_DELAY`, so an idle link blocks until a real ISR/ITS event and the chip
can light-sleep. RX stays prompt regardless: DIO1 is a light-sleep GPIO wake
source and the ISR notifies the task.

**The chip is polled only on a real IRQ.** The DIO1 ISR sets a flag
(`s_radioIrq`); each turn the loop reads it once (atomic read-and-clear) and calls
`serviceRadio` (a SPI `getIrqFlags` + whatever completed) only when it was set — or
when a transmit is in flight, for the TxDone watchdog. A wake for ITS / config /
stats does **not** touch the chip. Without this gate, `getIrqFlags` ran on every
task wake, so SPI-bus traffic tracked task *wakes* rather than *packets* and idle
housekeeping showed up as phantom radio load.

**Per-turn, per radio:** drain completed RX (§6), expire a stale split, re-
register with `rnsd` if the handle dropped while enabled, expire neighbour-table
proof expectations (§13), start an adaptive-power probe if one is due (§15),
advance a running probe (§14) and any queued linkage frame (§14.2), then drain
one outbound packet. Once per turn: decode whatever the RNode client has sent
(§17.5), and — while a LoRaMon viewer is open — run every radio's 1 Hz frame
expiry and airtime publication (§12).

**Stats are event-driven, not timed.** Every published stat is a cumulative
counter (tx/rx bytes and frames, `crc_err`, `split_rx_timeout`) or a last-packet
reading (`rssi_last`, `snr_last`) — none move without a tx/rx event, so a timed
republish would only burn battery. The task sums the counters each turn; a change
means traffic happened, and stats are published **at most once a second** (a
change inside the 1 s window defers to the boundary, where `nextDeadline` wakes
the task to flush the coalesced values). The keys are seeded once at startup so a
consumer sees a radio before any traffic. A running-but-unregistered radio holds
a 1 Hz retry wake until registration takes.

## 3. SPI bus + the RadioLib HAL

`EspIdfHal : public RadioLibHal` is ~200 lines of stateless plumbing.

- **Shared bus.** `init()` brings the bus up through `spiHelperInitBus`
  (idempotent — multiple radios and a future LCD/SD driver can call it), then
  adds one SPI device per radio. `CONFIG_LORA_SPI_HOST` is the **1-based**
  peripheral name (1 = SPI1, 2 = SPI2/FSPI, 3 = SPI3); the IDF
  `spi_host_device_t` enum is 0-based, so the task subtracts one. A straight cast
  put LoRa on SPI3 while the board's shared bus lived on SPI2 and the two
  controllers fought over the pins — keep the `-1`.
- **CS.** The device is added with `spics_io_num = -1`; RadioLib pulses CS itself
  via `digitalWrite`. RadioLib legitimately holds CS low across what it considers
  two transactions, so letting it own the line avoids surprises.
- **Bus locking.** `spiBeginTransaction` takes `spiHelperBusLock()` *then*
  `spi_device_acquire_bus`, released in reverse. The outer lock serializes
  against the LCD's async-DMA flush on a shared bus — the SPI driver's own bus
  lock isn't enough because `esp_lcd` drops it before its DMA finishes.
  Transfers use `spi_device_polling_transmit` for low latency on small command
  words; the default device clock is 8 MHz.
- **GPIO ISR service** is installed lazily/shared via
  `spiHelperEnsureGpioIsr(ESP_INTR_FLAG_IRAM)`.

**ISR — one notification, re-arm in the task.** Each radio's IRQ line is hooked
with `setPacketReceivedAction(loraRadioIsr)`; the single shared ISR does only
`vTaskNotifyGiveFromISR(s_task)` + `portYIELD_FROM_ISR`. Any radio's IRQ wakes
the task, which then polls each radio's IRQ flags to find the one that completed.
The HAL trampoline (`isrTrampoline`, `IRAM_ATTR`) **disables the GPIO interrupt
before invoking the callback**; the task re-enables it with `gpio_intr_enable`
after draining (§6). Without that disable/re-enable, a level-trigger
interpretation re-fires continuously while the line is asserted.

**ISR rules:** `IRAM_ATTR` mandatory (it may fire during flash access); no SPI
and no logging from the ISR — all IRQ-status reads, FIFO drains, and re-arm
(`startReceive`) happen task-side.

## 4. Chip dispatch (the X-macro)

The whole runtime path is chip-agnostic: `getIrqFlags`, `setPacketReceivedAction`,
`startReceive`, `readData`, `transmit`, `getRSSI`/`getSNR`, `sleep` are all
`PhysicalLayer` virtuals, so each radio holds a `PhysicalLayer*`. Only three
things vary by chip and dispatch through the `LORA_CHIPS(X)` X-macro:

- **construction** (`radioNew`) — which concrete class to `new`;
- **`begin()`** (`radioBegin`) — each family's `begin()` takes a different
  argument set (SX126x carries TCXO + regulator; SX127x has a LNA-gain arm and no
  TCXO; SX128x is 2.4 GHz and bare; LR11x0 sets freq/power *after* `begin()` in
  `lr11x0Begin`; LR2021 takes everything including TCXO). The pointer really is
  the concrete class, so the `static_cast` in the switch is sound;
- **display name** (`chipName`).

The X-macro order **fixes the numeric `CONFIG_LORAn_CHIP_ID`** the Kconfig
`choice` resolves to (id = position from 0). The Kconfig `LORAn_CHIP_ID` defaults
mirror this list — keep the two in lockstep.

The RF switch is uniform and handled at the call site, not in dispatch:
`Module::setRfSwitchPins(rx, tx)` for a two-GPIO external switch (set on the
`Module` before `begin()`, so it covers every family), or — SX126x only —
`setDio2AsRfSwitch(true)` applied inside `radioBegin` when the slot asks for it.

**RX gain (SX126x only).** SX126x/LR `begin()` sets the regulator to **DC-DC**
(passing `useRegulatorLDO = false`). The LNA gain mode is a per-radio setting
`s.lora.<n>.rx_boosted_gain` (default **on**, live via `lora <n> rx_boosted_gain
0|1`): `radioBegin` calls `setRxBoostedGainMode` for the SX126x family — boosted
buys ~+3 dB sensitivity for ~0.4 mA more RX current (~4.2 → ~4.6 mA typ.), worth
it for a receiver that idles in RX. The key is inert on non-SX126x families.

**Presence probe.** `probeRadio` runs a bare `begin()` (safe defaults + the
slot's TCXO voltage) at boot; `RADIOLIB_ERR_NONE` means the radio answered on
SPI. It probes in the chip's **own band** — 2450 MHz/812.5 kHz for SX128x, else
434 MHz/125 kHz — because a sub-GHz probe would make a 2.4 GHz part read as
absent. The result feeds the boot log and the `lora` CLI; `radioStart`
re-`begin()`s with the real config when the radio is enabled.

## 5. On-air split framing

A LoRa frame caps at **255 bytes** (8-bit length register); Reticulum's MTU is
**500 bytes**. So an RNS packet larger than one frame is split across **at most
two** LoRa frames with a 1-byte header per frame:

```
[ 1 byte header ][ ≤254 byte payload ]

header upper nibble (0xF0): random 4-bit sequence id
header bit 0       (0x01): SPLIT — this frame is part of a 2-frame split
```

- RNS packet ≤254 B → one frame, SPLIT clear.
- RNS packet 255–500 B → two frames (first 254 B, then the remainder), both with
  the same random seq nibble, both SPLIT set; the receiver concatenates them.
- The random seq nibble lets a receiver tell one sender's split from another's
  interleaved on the air. A half-assembled split is dropped after
  `SPLIT_RX_TIMEOUT_MS` (5 s), bumping `split_rx_timeout`.

This is a self-contained framing local to this codebase — it is **not** RNode
firmware, HDLC, or KISS, and there is no byte-stuffing. Constants:
`RNS_MTU = 500`, `RNODE_MAX_PAYLOAD = 254`, `RNODE_FLAG_SPLIT = 0x01`.

## 6. RX and TX paths

**RX — `drainRadioIrq(r)`.** On wake the task reads `getIrqFlags()` (read-only,
so polling one radio never disturbs another's in-flight RX) and acts only on
`RADIOLIB_IRQ_RX_DONE`. It range-checks the packet length, `readData`s the frame
(bumping `crc_err` on `RADIOLIB_ERR_CRC_MISMATCH`), caches RSSI/SNR, then parses
the header: not-split frames go straight to `rnsd`; split frames assemble into
the per-radio `splitBuf` (one in-flight split per radio, matched by seq). It ends
by re-arming RX (`startReceive`) and `gpio_intr_enable` on the radio's IRQ pin.

**TX — `sendRnsPacket(r)` / `sendOneFrame`.** Transmission is **synchronous**:
`radio.transmit()` runs inline on the task, then `startReceive` re-arms RX
(RadioLib leaves the radio in standby after a transmit). One or two frames are
sent depending on length; `tx_bytes` counts the RNS payload, `tx_frames` counts
each LoRa frame.

**Half-duplex coordination.** LoRa can't transmit while receiving, so a pending
split RX must not be interrupted. `drainOneOutbound` early-outs while
`r->splitPending` is set (or the radio isn't running, or `rnsd` isn't
connected); the outbound packet stays in the ITS stream buffer and is revisited
once the split completes or times out.

**Listen-before-talk (CSMA/CA).** Before a queued frame is transmitted it must
pass `csmaClear(r)`, a non-blocking channel-access state machine. Carrier
sense is the instantaneous channel RSSI — `channelRssi(r)` reads `getRSSI(false)`
without leaving continuous RX (dispatched per chip, since that overload isn't on
`PhysicalLayer`) — compared against a tracked noise floor (`channelBusy`: the
floor snaps down fast and creeps up slowly, so an active channel can't inflate
its own reference; busy = `rssi > floor + CSMA_RSSI_MARGIN_DB`). Sensing is
shared by two backoff regimes, selected per radio by `s.lora.<n>.appc` inside
the `s.lora.<n>.lbt` gate; both drive the same `CsmaPhase`, so the stall
warning, the `lbt_timeout` valve and `nextDeadline()` are regime-agnostic.

*Exponential regime (`appc=0`).* The classic form:

- `CSMA_IDLE` → begin an inter-frame (DIFS) listen.
- `CSMA_DIFS` → require the channel idle for `difsTicks`; any activity restarts
  the window. Once satisfied, draw a backoff of `[0, 2^cw)` slots.
- `CSMA_BACKOFF` → count the backoff down one slot at a time while the channel
  stays idle; if it goes busy, widen `cw` (exponential, capped at `CSMA_CW_MAX`)
  and re-listen. Backoff drained on a free channel → grant TX, reset `cw`.

`slotTicks` derives from the LoRa symbol time (`2^SF / BW`, clamped
`CSMA_SLOT_MS_MIN..MAX`); `difsTicks` is two slots.

*Adaptive regime (`appc=1`, the default) — see §6a.*

Either way the machine is driven from the task loop: when access is deferred the
frame stays queued and `nextDeadline()` wakes the task at the next slot boundary
to re-sense (never at 0, which would peg the task). `lbt=0` reverts to blind
transmit. The only other TX guard remains `splitPending`.

**On the SPI cost.** Each sense is one `getRSSI(false)` — a single SPI
transaction, read at the DFS floor (the re-sense wakes are timeout-driven, so
they don't boost the CPU). A transmit therefore issues a burst of these across
its DIFS + backoff slots, which makes `spi_master` the dominant SPI source while
traffic flows — but the transfers are ~55 µs APB holds at 80 MHz, ~0.1 % of wall
time, so LBT costs no measurable power (confirmed by an `lbt 0`/`lbt 1` A/B: SPI
halves, light-sleep % is unchanged). The chip *does* have a hardware
alternative — `startChannelScan()` (Channel Activity Detection, CAD), a
LoRa-preamble-aware sense that IRQs on `CadDone` — but it costs **more** SPI per sense (standby → DIO → clear → setCad
→ read result ≈ 6 transactions vs. 1) and drops RX to standby for each sense, so
it is *not* an SPI win. CAD's only edge is sensing quality: it ignores non-LoRa
ambient RF that an RSSI threshold trips on. Reach for it only if a quiet-but-noisy
channel is causing spurious backoffs, not to cut the bus count.

Outbound packets arrive from `rnsd` over the registered handle: `onRnsdRecv`
calls `drainOneOutbound`, which — once LBT clears — `itsRecv`s one packet and
transmits it. One packet per loop turn so RX re-arms between back-to-back sends.

**Per-frame trace.** `log lora debug` turns on a `dbg` line per on-air frame
(`loraTraceFrame`): direction, length, and a 20-byte hex preview — RX lines also
carry `rssi`/`snr`, and a CRC-failed RX logs `rx CRC-FAIL` with rssi/snr (LoRa's
error-check is the CRC; RadioLib exposes no corrected-bit count). The formatting
is guarded by `logIsDebug("lora")`, so the trace costs nothing when off. The tag
is the task name `lora`, so `log lora debug` is what gates it.

## 6a. APPC — the adaptive contention window

**The acronym is ours and it is inaccurate.** APPC expands to *adaptive
p-persistent CSMA*, a name coined for this straddle; no upstream project uses
it. Textbook p-persistent CSMA (Kleinrock & Tobagi, 1975) gates each transmit
opportunity behind a probability *p*, and there is no coin flip anywhere in this
code. What `appc=1` actually implements is an adaptive **contention window**:
the random backoff is drawn from one of four bands, and the band is picked by
how much of the recent past this radio spent transmitting. That reaches the
load-responsive politeness p-persistence aims at by sizing the window instead of
by rolling dice — same goal, different mechanism — so treat the name as a label
for the feature, not a description of the algorithm.

**It is RNode's mechanism, parameter for parameter.** Every constant, band edge,
clamp and quirk below is lifted from [RNode
firmware](https://github.com/markqvist/RNode_Firmware) — `Config.h` (the *CSMA
Parameters* block), `update_csma_parameters()`, `tx_queue_handler()`,
`add_airtime()` and `updateBitrate()`. Upstream has no name for it; it is simply
how CSMA works there. This is a reimplementation on our own task loop and radio
abstraction, not shared code.

**The load signal is our own transmit duty cycle**, not observed channel
occupancy. That is upstream's choice too: `update_csma_parameters()` reads
`airtime` (own time-on-air over the last two bins), not `total_channel_util`
(which folds in carrier-detect sampling and is only reported to the KISS host).
It holds up because every radio on a congested channel transmits more — retries
included — so own-airtime tracks aggregate load closely enough to act on, and it
needs no extra sensing: each frame's time-on-air is already computed for the
LoRaMon record.

**Airtime accounting.** `appcAddAirtime()` credits every frame at TxDone, probe
and linkage frames included. Time-on-air is bucketed into `APPC_BIN_MS` (7500 ms)
bins aligned to the uptime hour, and `appcAirtime()` reads the current plus
previous bin over their combined span — so the figure covers between one and two
bins of history, which is upstream's behaviour and what the band edges are
calibrated against. We keep only those two live bins; upstream's full 480-bin
ring exists to feed a long-term duty-cycle lock that this straddle does not
implement.

**Band selection** (`appcBandFor`), on the integer percentage of that figure:

| Own airtime | Band | Window drawn |
|---|---|---|
| ≤ 7 % | 1 | 0–13 slots |
| 8–38 % | 2 | 15–28 slots |
| 39–77 % | 3 | 30–43 slots |
| ≥ 78 % | 4 | 45–58 slots |

The bands partition one `0 .. APPC_CW_BANDS × APPC_CW_PER_BAND_WINDOWS − 1`
ladder; each band's top value is unreachable because upstream draws with
Arduino's `random(min, max)`, which excludes its upper bound, and `appcDrawWindow`
matches that. `appcMap()` reproduces Arduino's integer `map()` so the edges land
exactly where upstream's do. The expression feeding it is kept verbatim rather
than simplified — upstream adds `APPC_BAND_1_MAX_AIRTIME` to the percentage
*and* uses the same constant as the input floor, which cancels to a plain 0-based
scale; written out, a future upstream change to either constant stays a one-line
diff.

**Slot and DIFS** (`radioStart`) are APPC's own, separate from `slotTicks`: the
slot is `APPC_SLOT_SYMBOLS` (12) symbol times, clamped to
`[APPC_SLOT_MIN_MS, APPC_SLOT_MAX_MS]` = 24–100 ms, with the floor dropping by
`APPC_SLOT_MIN_FAST_DELTA` to 6 ms above `APPC_FAST_THRESHOLD_BPS`; DIFS is
`APPC_SIFS_MS` + 2 slots. Upstream's clamp is asymmetric — it compares against
the 24 ms floor but assigns the possibly-6 ms fast-rate one — and that is
reproduced. The "fast rate" test needs the *nominal* LoRa bitrate, computed
locally in `radioStart`, because `curBitrate` is deliberately distorted to shape
the RNS link timeout and would misclassify every radio.

Those millisecond figures are then quantized by the 100 Hz FreeRTOS tick, which
rounds each of them **down** to a 10 ms multiple. Upstream runs a 1 ms-resolution
poll loop and gets the nominal values; what this build actually runs, at 125 kHz
bandwidth, is:

| SF | 12 symbols | after clamp | realized slot | DIFS | longest backoff (58 slots) |
|---|---|---|---|---|---|
| 7 | 12 ms | 24 ms | 20 ms | 40 ms | 1.16 s |
| 8 | 24 ms | 24 ms | 20 ms | 40 ms | 1.16 s |
| 9 | 49 ms | 49 ms | 40 ms | 80 ms | 2.32 s |
| 10 | 98 ms | 98 ms | 90 ms | 180 ms | 5.22 s |
| 11 | 196 ms | 100 ms | 100 ms | 200 ms | 5.80 s |
| 12 | 393 ms | 100 ms | 100 ms | 200 ms | 5.80 s |

The same tick floor already applies to the exponential regime's slot, so this is
not new behaviour, but it does mean APPC runs up to 17 % more eagerly than
upstream at SF7–SF10. Raising `CONFIG_FREERTOS_HZ` would close the gap; nothing
else here depends on the quantization.

**The machine** (`csmaClearAppc`). The window is a wall-time target
(`appcCw × appcSlotTicks`) accumulated only while the medium reads free, not a
slot countdown:

- Draw a window if none is held, then require the medium free for `appcDifsTicks`
  unbroken.
- DIFS satisfied → start accumulating free-medium time toward the target.
- Medium goes busy → DIFS restarts from scratch, but the accumulated backoff
  **freezes** rather than resetting, so a frame contending on a loaded channel
  keeps its progress and cannot be starved indefinitely.
- Target reached → grant, and the next frame draws a fresh window.

The window is never widened on a busy encounter. All adaptation lives in the
band, which is why a single loud neighbour does not push this radio into longer
backoffs the way the exponential regime would.

**Sensing cadence** stays `slotTicks` (10–20 ms) in both regimes, not APPC's
longer slot — the APPC slot is only the unit its target is counted in. Upstream
senses every 3 ms, far finer than its own 24–100 ms slot, so keeping our existing
cadence preserves the intent; the cost is that a medium going busy is noticed up
to one `slotTicks` late, over-crediting the backoff by at most that much.

**Deliberate divergences from upstream**, beyond the reimplementation itself:

- *`cw_wait_start` is cleared on grant.* Upstream resets `cw_wait_passed`,
  `csma_cw` and `difs_wait_start` after a transmit but leaves `cw_wait_start`
  holding the old timestamp, so the next frame's first accumulation pass credits
  it every millisecond since the previous grant — shortcutting much of the
  backoff on the second and later frames of a burst. That is plainly an
  unintended sentinel leak, and reproducing it would defeat the mechanism, so
  `csmaClearAppc` clears it.
- *Access state is abandoned when the queue drains.* `csmaResetAccess()` discards
  a frozen backoff when nothing is queued, when a frame is shed by
  `lbt_timeout`, and when the probe takes or releases the radio. Upstream would
  carry it into the next frame; here the machine is shared by three producers
  (queued RNS traffic, linkage frames, the probe) and stale progress must not
  leak between them.
- *Time-on-air comes from `loraAirtimeSeconds()`*, the standard Semtech formula
  already used throughout this file, rather than upstream's algebraically
  rearranged variant in `add_airtime()`. The two agree to well under a percent.
- *The band is evaluated when a window is drawn*, not on a 1 Hz timer. Upstream
  recomputes it from `update_airtime()` and can therefore be up to a second
  stale at draw time; the values are identical, ours is just fresher.
- *Bins reset on radio restart*, and any `s.lora.*` write restarts the radio
  (`applyConfig`), so editing a setting clears the airtime history.
- *`CSMA_POST_TX_YIELD_SLOTS` is not implemented* — upstream defines it but never
  references it.

**Interaction with `lbt_timeout`.** Per the table above, a band-4 radio can
legitimately wait 5.22 s at SF10 and 5.80 s at SF11/SF12 — past the 5000 ms
default — so a saturated slow link will shed frames the mechanism intended to
merely delay. SF9 and below stay well inside the budget in every band. Upstream
has no equivalent valve and never faces this. The default is left alone (an
unbounded outbound queue is the worse failure), but `s.lora.<n>.lbt_timeout`
should be raised, or set to `0`, on an SF10+ link expected to run deep into
band 3 or 4.

**Observability.** `lora <n>` prints the regime, slot/DIFS times and, under
`appc`, the live airtime percentage with its band and window range; the same
two figures publish as `lora.<n>.stats.{airtime_pct,cw_band}` on the ordinary
event-driven telemetry flush. The contention stall warning names the band and
airtime instead of the exponential regime's `cw` when `appc` is on.

## 7. rnsd registration

`registerWithRnsd(r)` opens an ITS connection to `RNSD_PORT_IFACE` with an
`rnsd_iface_t`:

- `name` = `lora/<idx>` (iface-lora owns slot-name uniqueness; `rnsd` takes it
  verbatim);
- `mtu` = 500, `bitrate` = the airtime-derived value (§8);
- `mode` from `s.lora.<n>.mode` via `modeFromString` → an `RNS_IFACE_MODE_*`
  value. **Default is `access_point`** — a LoRa segment is almost always the
  edge of the network, and access-point mode stops the node re-broadcasting the
  whole transport network's announces onto the slow RF link. `full`/`gateway`
  remain valid on the key for a deliberate LoRa backbone (set `s.lora.<n>.mode`
  by hand) but are kept out of the settings picker (`straddle.yaml`,
  `LoraPanel.vue`) to avoid footgunning airtime;
- `in = out = 1`, `fwd = 1` for `FULL`/`GATEWAY` (forwarding/transport modes),
  `rpt = 0`;
- `announce_cap` from `s.lora.<n>.announce_cap` (percent, default
  `RNS_IFACE_ANNOUNCE_CAP_DEFAULT` = 2) — the max share of interface bandwidth
  announces may use; `point_to_point` is left 0 (LoRa is a shared radio medium
  with hidden nodes, so announces are still re-broadcast for peers out of range
  of the origin — see `rns/INTERNALS.md` §1.1.1);
- IFAC fields from `s.lora.<n>.ifac_netname` / `ifac_size` and
  `secrets.lora.<n>.ifac_netkey`.

The ITS connect **ref is the radio index**, so `onRnsdDisconnect(ref)` finds the
radio and clears its handle; the task loop re-registers on the next turn if the
radio is still enabled. If registration fails but the radio is on-air, the state
goes `rnsd_unavailable` and the loop keeps retrying — RF stays up.

rnsd is one of **three** endpoints on a radio segment, not the only one. The
others are the radio itself and — when configured — an attached RNode client
(§17). A packet entering from any one is presented to the other two, and the
rnsd handle is no longer a precondition for outbound work: `drainOneOutbound`
computes availability across both software sources before deciding anything.

## 8. Airtime-derived bitrate

`loraAirtimeSeconds` computes the LoRa time-on-air of a frame per Semtech
AN1200.13 (symbol time `2^SF / BW`, preamble `(n + 4.25)` symbols, payload
rounded into whole symbols, low-data-rate optimisation engaged once a symbol
exceeds 16 ms, explicit header + CRC on). `computeBitrate` registers
`bitrate_eff = (500 × 8) / ceil(ToA of one 500-byte frame) = 4000 / ceil(ToA_s)`.

This is deliberate: RNS derives its first-hop link-establishment timeout as
`MTU×8/bitrate + 6 s`, so registering this bitrate makes that term equal
`ceil(airtime) + 6 s` — link setup waits roughly one frame's real airtime plus
margin instead of a fixed budget. It is **not** the LoRa channel symbol rate
(`SF × BW/2^SF × 4/CR`).

## 9. Config lifecycle

A change to any `s.lora.*` or `secrets.lora.*` key fires `onCfgChange`, which
calls `cfgArm(LORA_CFG_COALESCE_MS)` (300 ms). The apply is **coalesced**: a
radio restart is what an apply costs (`radioStop` + `radioStart` + a fresh rnsd
registration), and a configuration burst — an RNode client's frequency,
bandwidth, spreading factor and power arriving as four separate writes, or the
same typed as one `;`-separated CLI line — would otherwise pay that once per key.
One burst becomes one restart and one `registerWithRnsd`.

`cfgArm(delayMs)` **arms once and is never pushed out** by a later change; it can
only be pulled *in*. That asymmetry is the whole design:

- an immovable deadline is what keeps a busy device from starving the apply
  forever (spangap-core's `storage.cpp` save timer documents the same trap:
  re-arming on every write pushed the flush past every realistic idle window);
- pulling in cannot starve anything, and it is what an RNode client's 0.25 s
  echo-validation window needs. `CMD_RADIO_STATE` — which always terminates the
  client's configuration burst — calls `cfgArm(0)`, so the apply and its echo
  land inside that sleep while the burst before it still coalesces (the earlier
  writes armed 300 ms and nothing has applied yet). A CLI or web burst has no
  such terminator and simply takes the 300 ms window.

`nextDeadline()` carries an `s_cfgPend` clause so the task wakes at the deadline.
When it fires, the pass runs — in order — `rnodeSettleOff()`, `applyConfig(r)`
per radio, `loraPublishDisplay(i)` per radio, `rnodeEchoFlush()`, and
`rnodeApplyTransports()`. The echo has to come after the apply because it reports
what was applied; the transport pass sits here because it is also what puts the
net endpoint registration on this task, where net requires it to originate.
`cfgArm(0)` is also used for hardware recovery (`probeRestoreCfg` failing leaves
the radio in the sweep regime — nothing to coalesce with) and at task-loop entry
and every `rns start` resume.

`applyConfig(r)` reads `s.lora.<n>.enable`: disabled → `radioStop`; enabled →
`radioStop` then `radioStart` (a cheap stop/start that avoids tracking which
field changed).

**`radioStart`** reads the radio config, validates it (`freq > 0`, `bw > 0`,
`sf ∈ [5,12]`, `cr ∈ [5,8]`, `txp ∈ [−9,22]`; sync word parsed with
`strtol(base=0)` and falling back to `0x42` outside `(0,0xFF]`), and refuses to
bring an unconfigured radio up (`state = unconfigured`). RadioLib wants frequency
in MHz, bandwidth in kHz, TCXO in volts — the task converts from the Hz/mV
stored values. On a `begin()` error it logs the decoded `RADIOLIB_ERR_*` name
(`rlErrName`) plus the raw code and sets `state = error`. On success it computes
the mode + bitrate, reads IFAC, hooks the IRQ, `startReceive`s, publishes
`state = up` + `chip` + `bitrate_eff`, and registers with `rnsd`.

**`radioStop`** disables the IRQ wake source, clears the packet action, sleeps
the radio (full config is re-applied on the next start, so config retention
doesn't matter), clears split state, deregisters from `rnsd`, and publishes
`state = down`.

## 10. MHz/kHz unit bridge

`s.lora.<n>.frequency` and `.bandwidth` are stored in **Hz**, but the settings
pane and the `lora` CLI speak **MHz / kHz**. So each radio mirrors its two Hz
config keys to a pair of **ephemeral** display keys — `lora.<n>.freq_mhz` and
`lora.<n>.bw_khz` — holding a trimmed decimal (`hzToUnit`). The `straddle.yaml`
pane binds two plain `text` rows to those ephemeral keys (not to `s.lora.*`), so
the operator types any value in human units — no preset dropdown.

Both directions run on the task, never in a storage callback:

- **Hz → display** (`loraPublishDisplay`): after every config apply (and once at
  `onInit`, so the pane has values before the `rns.ready` barrier lifts) each
  radio re-publishes its display keys.
- **display → Hz** (`loraApplyDisplay`): editing a display key fires
  `onDisplayChange`, which raises `s_displayDirty` + notifies. The task parses
  each field (`unitToHz`); a valid, in-range, **changed** value is written back
  to the `s.lora.<n>.*` Hz key (which re-fires `onCfgChange` → radio reconfig +
  re-publish); an unparseable or out-of-range entry is reverted to the stored
  value.

The "write only on a real change" guard on both sides is what keeps the
round-trip from looping. Bounds are int32-safe (`freq ≤ 2 GHz`) — storage ints
are 32-bit, so a 2.4 GHz value would overflow regardless.

## 11. Defaults seeding

`loraInit` registers the `lora` CLI and spawns the task. It seeds per-radio
defaults under a `s.lora.version` gate (`LORA_VERSION = 4`) for radios **1..**
only — radio 0's defaults come from this straddle's `settings:` block in
`straddle.yaml`, **except** two keys with no pane row of their own, seeded for
radio 0 directly: `s.lora.0.bandwidth` (its pane row binds the kHz display key
rather than the Hz config key) and `s.lora.<n>.adaptive_txpwr`, which is seeded
for *every* radio — a key that only exists once someone guesses its name is not
discoverable, and it is otherwise read with an inline default and never
written. Frequency and TX power carry no default
(region/antenna — the user must pick); everything else defaults so an
enable-toggle alone gets a radio up. The **RNode group is global, not per radio**
— there is one endpoint for the device — and is seeded under the same gate with
**both doors shut**: `s.lora.rnode.radio` (0), `.serial` (−1 = claim no serial
port) and `.tcp` (0 = open no socket; 7633 is the only port a stock client can
dial, so it is the only other useful value). `s.lora.rnode.enable` comes from its
pane row in `straddle.yaml`. The existing
`storageSubscribeChanges("s.lora", …)` prefix subscription already covers the
group, so its edits land in the coalesced apply pass like any other. `loraInit`
does **not** touch any power pin — the board owns the LoRa rail.

## 12. LoRaMon (per-frame telemetry)

Every on-air frame is recorded for the LoRaMon viewers (browser + LCD). The
storage subtree **is** the ring — no in-firmware record buffer, no ITS transfer:

- **One node per frame.** `loraMonPush` (from the RX drain and each TxDone, so
  once per on-air frame → two per split RNS packet) writes
  `lora.<n>.packets.<ms>` = a packed string; direction is the leading token, snr
  is deci-dB:
  - rx: `r|<rssi>|<snr>|<dur_ms>|<bytes>|<type>`
  - tx: `t|<txp>|<dur_ms>|<bytes>|<type>|<wait_ms>`
  `<ms>` is the frame's start on the monotonic `millis()` clock, `<dur_ms>` its
  computed time-on-air — computed from the framing the frame *actually* flew
  with, via `LoraRadio.airPreamble` / `airImplicit`, which track the rfprobe
  sweep regime rather than the configured values (see §16), and `<type>` its protocol — `0` Reticulum,
  `1` this straddle's own air protocol (rfprobe frames, 0x02/0x03 linkage),
  `2` a packet the attached RNode client originated (§17). The viewers colour
  them yellow, red and orange. RX is classified by whether `probeOnRx` consumed
  the frame (so the tap runs before the record is written); TX by
  `LoraRadio.txType[]`, which is per-frame because a power request (§15.1) and
  the RNS packet it prefixes share one burst but not one protocol. The record's
  byte count is payload bytes: a tx record strips the 1-byte seq/split header for
  every type **except** `1`, which carries none — RNode-origin packets fly
  through the same framing rnsd's do — while an rx record strips one
  unconditionally, so an inbound type-`1` frame reads a byte short of the air.
  The per-frame `log lora debug` line is emitted here too
  (it replaced the RNS-header trace; `loraTracePacket` is kept but unwired).
- **`<wait_ms>` is queue latency, not airtime** — the wall time between the
  frame reaching the head of the outbound queue and its first bit going on air.
  The clock starts on the first task pass where there is something to send and
  we can't (`drainOneOutbound`), *before* the early returns, so it counts the
  radio being held by a probe or a linkage frame and a split still reassembling,
  not only DIFS/backoff against a busy channel. It belongs to the **first frame
  of a burst** and is zero for the rest — the frames behind it followed
  immediately and waited for nothing. Frames that bypass the outbound queue
  report the wait of the channel access they *do* run (`csmaWaitMs`, read on the
  pass `csmaClear` grants the medium): carrier-sense time for P1 (§14) and for a
  0x02/0x03 linkage frame, zero for a sweep frame, which fires on the schedule
  and senses nothing. With LBT off every figure of that kind is zero. Both viewers
  draw it in the frame's own colour as a **tick at the moment it queued, then a
  hairline at mid-height running up to the bar** — light enough that channel
  occupancy still reads as the filled area alone, so a long wait can't be
  mistaken for airtime.
- **Expiry is the ring.** A per-radio start-ms FIFO (`pktMs`, cap
  `LORA_MON_CAP`) drives deletion: on each push and once a second,
  `loraMonExpire` pops + `storageDeleteTree`s nodes older than 1 h (the cap is a
  flood backstop). `storageDeleteTree` emits an explicit delete op, so the
  browser mirror sees removals — unlike the implicit ~1 Hz ephemeral
  republish-merge, which never nulls a removed key (that asymmetry is why
  add/delete works here but a merge-only scheme couldn't expire on the browser).
- **Gated on a viewer.** Recording runs only while `sys.stats.web_loramon` or
  `sys.stats.lcd_loramon` is set (`loraMonWatched`), the actmon gating pattern.
  On the falling edge the task drops each radio's whole `lora.<n>.packets`
  subtree — so there is no pre-open history; the graph fills from open forward.
- **One aggregate only: the hour.** Every shorter window's airtime % is computed
  by the viewers from the nodes they hold, per direction with boundary overlap.
  The hour is the exception — it spans more history than a viewer is typically
  open for, and recording stops when the last viewer closes — so the firmware
  keeps a rolling 12 × 5-minute rollup (`AirBucket`, fed by every `loraMonPush`
  whether or not anyone is watching) and publishes `lora.<n>.air1h.{rx,tx}` in
  **per mille** on the 1 Hz beat. The browser derives the device clock from the
  newest packet ms it has seen (monotonic anchor, never pulled backward); the
  LCD uses `millis()` directly (same clock as the keys).

Consumers: the browser reads the mirrored `lora.<n>.packets` subtree
(`iface-lora/browser`); the LCD app iterates it via `storageForEach` on a 1 s
redraw (`conditional/spangap-lcd/`, a `when: spangap/spangap-lcd` service). Both
show **one graph carrying both directions** over one selected window, picked
from a pill row (10s / 1m / 5m / 10m / 30m / 1hr). Each frame is a horizontal
line spanning its time-on-air, placed on **one of two dBm axes sharing the same
four gradient bands**: transmit power −10…+30 dBm in **10 dB** steps down the
left gutter, received strength −130…−30 dBm in **25 dB** steps down the right.
The steps differ because the ranges do (100 dB against 40) and both have to land
on the same band edges for one grid to serve them; the axis names stand in the
pill strip over their own gutter, which is what pushes the pills inward.
Direction is read off **the background**: over a transmit's time-on-air — and
not over the wait before it, which is channel access, not transmission — the
bands are repainted in a **reddish cast of the same gradient**. That frees
colour to mean the frame's **protocol**: Reticulum yellow `#E8D040`, the RNode
client orange `#E89040`, ours red `#E84040`. The caption carries **tx airtime**
and **channel busy** (tx + rx — the radio is half duplex, so the two never
overlap) plus a colour-keyed legend naming the three, since a colour with no key
is a decoration. A tab row above the graph selects the radio on a multi-radio
board (discovered from `lora.<n>.state`, hidden when there is only one);
switching resets the zoom stack, because a span selected on one radio's traffic
means nothing on another's.

The two viewers repaint differently, and the difference is visible. The browser
rebuilds from storage at 1 Hz but repaints off `requestAnimationFrame` against an
extrapolated device clock, so a live window *glides*; the LCD repaints on its 1 s
tick (and on a touch event), so the same motion is stepped. Nothing in the model
differs — only how often it is drawn.

**Zoom stack.** Touching/clicking the plot anchors a highlighted span. The
anchor is stored as a **device time, not a pixel** — which is what makes holding
still on a live graph *widen* the highlight: the anchor stays put while "now"
advances, so it drifts left under a stationary pointer. Releasing pushes
`{t0,t1}` on a zoom stack and the view becomes that fixed span; the window pills
give way to a single **back** pill. A zoomed view stands still, so selecting
inside it zooms further, pushing another level. Back pops one level, and
emptying the stack returns to whichever moving window was active. Depth is not
printed anywhere — it is the back pill's colour: **blue while one level is
left** (so it leads back to the live window), grey while another frozen view
stands behind. A release that selected under 1 % of the visible span (under 5 ms
either way in the browser) is a tap, not a zoom, and is discarded — otherwise
every stray touch would freeze the graph on a span nobody can read. Both apps implement the same model (`zoomStack`
/ `sel` in the Vue component, `s.zoom[]` / `s.selActive` behind `canvasEventCb`
on the LCD); the LCD additionally caps the stack at 8 levels, where the browser
lets it grow. In the browser the plot ignores a press taken while the window is *not*
front-most (`focusedWindowId`): that press only raises the window, matching the
click swallow `FloatingWindow` already does — pointer events bypass it, so
without the check, reaching for an occluded LoRaMon would cost a zoom. Airtime follows
the view: a zoomed span is always computed locally, and only the *live* 1-hour
window uses the firmware's published rollup.

**Timescale, frozen views only.** A zoomed view stands still, so it carries a
grid: **1-px vertical lines in `#242424`, the darkest tone of the band
gradient**, on round multiples of one division — dark enough to read as part of
the background rather than as something drawn over it. The span and what a
division is worth are stated beside the back pill in both viewers. A live view
gets none — the grid is anchored to absolute time and would
crawl across a sliding graph. The division is the smallest **1-2-5-10** step at
least `DIV_PX_MIN` pixels wide (70 CSS px on the web, 40 px on the LCD, whose
plot is a third the width), floored at 1 ms because the records are ms-stamped.
That ladder's widest gap is **×2.5** (2 → 5), which is the whole constraint on
the band: any allowed pixels-per-division range spanning a factor of ≥ 2.5
contains a step for *every* span and canvas width, so each viewer only states
its minimum and the 2.5× above it is implied.

This is the data substrate for adaptive TX power: per-frame power, signal and
protocol, which is what a control loop would have to be validated against
before it is allowed to drive the `txp` register.

## 13. Passive neighbour table (`lora n[eighbors]`)

A per-radio picture of the direct radio neighbourhood built **entirely from observing rx + tx RNS
packets on the interface** — no rnsd API, no peer cooperation, works against
every RNS implementation. Surfaced by `lora neighbors` (all radios) /
`lora <n> neighbors`. All state is in-memory (`NeiState`, ~12 KB PSRAM per
radio, `gp_alloc`'d at first `radioStart` and kept across config cycles and
`rns stop`).

- **Tap points.** `neiObserve()` is called with each whole (reassembled) RNS
  packet: from `deliverInbound` (rx, before the rnsd gate, with the same-call
  rssi/snr) and `beginTx` (tx, carrying the packet's `LORA_ORIG_*`). It decodes
  the RNS header only — no payload crypto.
- **Direct = wire hops 0.** The hops byte is incremented by the *receiving*
  transport, so a frame fresh from its originator carries `0` on air (µR's
  `hops()` reports 1 for the same frame). Everything at wire hops ≥ 1 was
  relayed — the originator is not the transmitter — and is ignored.
- **The identity join is cryptographic.** For an announce at hops 0 the entry
  is accepted only after (a) `dest == H(name_hash ‖ H(pubkey)[:16])[:16]` and
  (b) the announce signature verifies (`rnsdVerify`, inline-safe on the lora
  task). Dest hashes announced under one key cluster under one identity;
  known `app.aspect` name-hashes (dictionary in `kNeiNames`) label the rows.
  Our own tx announces feed the identical path and mark the entry `us` — "we"
  need no privileged source. A tx announce that came from the RNode client marks
  the entry `rnode` instead, giving the table a **second local row**. The two
  fold separately: our identities merge into `us`, the client's into `rnode`.
- **Links.** An LR at hops 0 yields `link_id = H([flags&0x0F] ‖ raw[2:])[:16]`
  with LR data trimmed to the 64 ephemeral-key bytes (MTU signalling excluded),
  mapped to its dest; the LRPROOF (context 0xFF, dest = link_id) marks it
  established and — at hops 0 — attributes its signal to the dest, which is
  thereby proven a direct neighbour. Mid-link traffic on an unseen link_id
  creates an *unresolved* entry. `ours` = we transmit on it at hops 0, or its
  LR dest is one of our hashes.
- **Link quality (one byte).** Proof packets are addressed to the proved
  packet's truncated hash, so each elicitor we transmit (an LR, or an
  originated single-dest packet to a known direct neighbour — probes included)
  parks its hash in a small pend table (`NEI_PEND_MAX`, 30 s deadline;
  `nextDeadline()` wakes the task for expiry). A returning proof at hops 0 is
  a hit; expiry a miss. Quality is hit-ratio EWMA (α = 1/4) plus raw counters.
  A miss only counts when a proof was actually owed: always for LRs, and for
  data only once the dest has proven before (`provesData`, learned from its
  first proof — PROVE_ALL is opt-in, so silence from a non-prover is not
  failure). Relayed proofs (hops ≥ 1) never score: proof is end-to-end, power
  is first-hop.
- **Signal + rollup.** Only frames *provably transmitted by* a node sample its
  min/max RSSI/SNR envelope and its last-hour rollup (12 × 5-min buckets:
  count, avg rssi, avg snr): announces at hops 0, proofs of our elicitors,
  LRPROOFs, and inbound hops-0 frames on links we initiated.
- **Transit neighbours.** A rebroadcast announce (hops ≥ 1) is the one relayed
  frame whose transmitter is named: the rebroadcaster stamps its own identity
  hash as the HEADER_2 `transport_id` (how path tables learn `first_hop`). It
  samples that neighbour's envelope/rollup, keyed by identity so the node's
  own hops-0 announces join the same row, and tags the row `transit`. This is
  unverified (the announce signature covers the originator, not the relayer) —
  the same trust rnsd's path table places in the field. On a pure-transport
  neighbourhood this is usually the first row that appears; without it a node
  whose whole horizon is relays sees an empty table. The same frame identifies
  *us* symmetrically: a rebroadcast we transmit stamps our own transport
  identity as transport_id, which becomes (or merges into) a `us … transit`
  row — the identity this node is known by on the air when it relays.
- **Anonymous transit.** Every rx frame at wire hops ≥ 1 was transmitted by an
  in-range transport node even when nothing names it (HEADER_1 relays, relayed
  proofs, a silent access-point bridge). Those sample one aggregate
  "unidentified transit" row per radio (count, envelope, last-heard) —
  anonymous transmitters can't be told apart, so no per-node split is claimed.
  Additionally, the truncated packet hash is hops/transport_id-invariant, so a
  recent-rx ring (`NEI_SEEN_MAX`, 30 s window) catches the same packet
  re-heard one hop higher: an RF→RF repeat by someone in range, counted as
  `in-band relays`. Our own tx never enters the ring, so our own relaying
  doesn't self-count; a bridge relaying to another medium (TCP) shows up in
  the aggregate row but not in the relay counter.
- **IFAC blinds the tap.** IFAC masks everything after the flags byte, so
  IFAC frames (bit 0x80) are skipped; on an IFAC network the table stays empty
  and the CLI says so. Eviction throughout is oldest-first (local entries are
  never evicted).
- **`neiIsLocal()` vs `isUs`.** Every RF-layer guard that means "this traffic
  terminates at our transmitter" tests `neiIsLocal(e)` = `isUs || isRnode`:
  eviction protection, the adaptive-power skips, the own-hash cluster and
  `probeOwnFirst4`, the announce-count and hash-advert walks, and
  `neiDestIsLocal()` behind relay detection and next-hop selection. A packet
  addressed to the client's identities is delivered over the wire, so the radio
  must no more probe, power-adapt or route toward it than toward ourselves.
  Guards that genuinely mean "our own identities" keep plain `isUs`.
  `neiWalk` emits both local rows in pass 0 numbered `0`, and refuses `0` as a
  lookup — `lora rfprobe 0` would be aiming the radio at this device. The
  listing header reads `… and us`, `… and rnode`, or `… and us + rnode`.

### 13.1 What `lora n` prints

```
lora/0 neighbors: 2 others and us, 0 open links (observing 17m)

  us   6b87eb8bdbcd51dee010c5a20fd65ef9 rnstransport.probe
       4e0521019085fd7dc7f9fb53e8c8d1a7 lxmf.delivery  "xiao"

  1    d10d5106bcaa65df4a8c50a56d8f05f7 rnstransport.probe
       6793e13ec79d1c1b1372885105aa5cf7 rnsh
       04e893bce336c889329b89fd61a66ac5 lxmf.delivery  "Rop"
       ( TRANSPORT, XXX, TX -9 )

  2    71cdbfd09e0ea8f0ab17dd06cd0c6e3f rnstransport.probe
       b9351473........................ (not seen yet)
       ( ROAMING, XXX )
```

One numbered block per node — `us` first, then `1`, `2`, … — and **one line per
hash**: full hash, aspect label, then the announced display name in quotes where
the announce carried one. `neiParseName` extracts that name locally (LXMF's
msgpack, optionally behind a 32-byte ratchet, and NomadNet's raw UTF-8);
iface-lora talks only to rnsd, so it cannot borrow lxmf/'s fuller parser. The
transport hash leads each block, being the one hash every node has and the one a
probe is addressed to. A hash a 0x03 linked but we have never heard directly
prints as `<first-4>........ (not seen yet)`.

A capability line closes each non-`us` block:

| tag | means |
|---|---|
| `TRANSPORT` | it relayed someone else's frame to us — a rebroadcast announce naming itself as `transport_id`, or any HEADER_2 frame at hops > 0 that does |
| `ROAMING` | its node-flags bit (a moving node wants more margin) |
| `RF_PROTO_NAME` (`XXX`) | it has spoken our air protocol to us — an rfprobe or linkage frame |
| `TX <dBm>` | the power a probe settled on for it (measured) |
| `EST <dBm>` | the same quantity *inferred* by reciprocity from frames we overheard, crediting the peer with `s.lora.assumed_peer_txp` (default 22). `lora rf` prints both plus their delta — a positive delta means the estimate is conservative, negative that it would under-power the link. Comparing them against a peer we *can* measure is how the estimate earns the right to be trusted against peers we can't (`plans/adaptive-power.md` §4.1). |
| `USE <dBm>` | the determination frames to this node actually go out at under `adaptive_txpwr`, `~` when it came from `EST` plus a margin rather than from a measurement (§15) |

Identities are the **join, not the display**: they build the rows but appear
only under `-v`, which also adds the signal envelope, link quality, the
last-hour rollup and the link_id section.

Node numbers come from `neiWalk`, which the printer and the `lora rf <n>`
resolver share, so the numbers on screen are always the ones the resolver
accepts. `lora rf` takes a node number, a hex hash or prefix, or any unique
substring of an announced name; a 4+ byte hex hash that matches nothing is
still accepted, so an off-table node can be probed. Both verbs abbreviate —
`lora n` … `lora neighbours`, `lora rf` … `lora rfprobe`.


## 14. rfprobe (`lora rf[probe] <dest>`)

Active, cooperative RF-link characterisation against one neighbour: the lowest
TX power that still closes the link, **in both directions**, in one short
exchange (sub-second at SF7; scales ×2^(SF−7)). Both ends must run this
firmware — a vanilla peer never answers and the run ends when the schedule does.
Full wire format and state machine live in the `ProbeState` comment block in
`lora.cpp`; §14.1 records why it is shaped the way it is. The remaining,
unbuilt half of adaptive power — the control loop that would *use* these
measurements, and what to do about neighbours that don't cooperate — is
`plans/adaptive-power.md`. Summary:

- **One carrier-sensed frame, then a fixed schedule.** P1 (12 B, explicit
  header, normal modem cfg, LBT, at our probe max) is the last listen-before-talk
  transmission of the run: `[0x00][us₄][them₄][txp][flags][rsv]`. Its
  **end-of-air is T0**; both ends drop straight into the sweep regime —
  implicit (headerless) frames, preamble 6, sync 0x23, **no carrier sense** —
  and every later transmission happens in a scheduled slot. Responder owns even
  slots, initiator odd.
- **Each slot is only as long as its own frame, and there are two guards.** Slot
  0 is `ToA(8 B) + PROBE_SLOT_GUARD_MS`, every slot after it
  `ToA(4 B) + PROBE_SLOT_GUARD_MS` (`probeSchedule`, `probeSlotOffUs`), derived
  from SF/BW/CR on both ends so they agree without exchanging anything. Sizing
  every slot for the longest frame left ~10 ms of dead air in all but the first.
  The guards differ because they cover different risks: **T0 → slot 0** is
  *task*-latency bound (the responder must take the rx IRQ, wake the priority-2
  task on a 10 ms tick, parse, switch the modem and arm its timer, or P2 never
  goes out) and stays at 15 ms; the inter-slot guard only covers TX-start
  latency and RX→TX turnaround, since those transmits fire from the timer with
  the frame already decided. Both are **15 ms**. The inter-slot one was tried at
  8, 9 and 12 and returned to 15 each time: the few ms per slot don't pay for the
  reliability, on a ladder that climbs its full range anyway. It also can't be
  tuned per board — both ends derive the schedule from the same constant, so it
  has to suit the slowest node in the mesh rather than the one being watched, and
  a build mismatch desynchronizes the two silently (§16). The pressure on it is a
  slot TX held off by SPI-bus contention (an LCD DMA flush on a T-Deck) or the
  callback yielding to a reception still landing; both are counted and reported
  as `forfeit` / `skip` on the ladder line, so if this is ever revisited it is a
  measurement rather than a guess.
- **The ladder climbs the full range on purpose.** It is a search, but the climb
  is also what guarantees we eventually become audible: a tight ladder around
  the opener's prediction, or a binary search, can leave a node too quiet to
  ever be heard on an asymmetric link. The cost is visible — on a link where the
  peer heard our first rung but its own frames only reached us at its max, we
  climbed all 6 rungs before learning we had been heard at −9 dBm, because our
  stopping condition rides *their* frames at *their* power. That entanglement is
  accepted deliberately in exchange for always getting loud enough.
- **Once done, hold at the power the peer echoed.** An earlier version held at
  `found + 1 step`, which is unjustified: they echoed that power, so it
  demonstrably reaches them, and stepping up is just stepping up. On a −9 dBm
  link it showed as one frame at −9 followed by every remaining frame at −3.
- **The tail after "done" is bounded** (`PROBE_DONE_TAIL`). Both ends stop once
  the peer confirms it too, but that confirmation can be lost — and then the
  side that didn't get it used to transmit for the *rest of the schedule* while
  the peer had already finished and restored its normal config. That failure
  mode reads as a wildly lopsided cost report (`sent 8 / heard 1`, tx 237 ms vs
  rx 58 ms, ~850 ms instead of ~330) with both directions nonetheless reported
  correctly. A few tail frames give the peer redundant chances to close its own
  side; the schedule length is the backstop, not the plan.
- **Slot 0 is P2** (8 B, at the responder's probe max):
  `[txp][rssi of P1][snr of P1][flags][4 × rsv]`. No magic and no echoed
  hashes — a frame landing in exactly that slot can only be the answer to our
  P1, which is precisely what those 9 bytes used to buy. Slot 0 is the only
  8-byte frame in the schedule; both ends re-arm the implicit length to 4
  before slot 1 (`probeShorten`, driven from the slot-timer callback, from P2's
  TxDone on the responder, and from P2's arrival on the initiator).
- **Two independent ladders, by reciprocity.** Each side sizes its ladder from
  **its own** measurement of the peer's last stated-power frame — the responder
  from P1, the initiator from P2 — so neither waits on the other and there is no
  shared derivation to keep bit-identical. Start = predicted cliff − 2 steps,
  floor −9 dBm, `PROBE_RUNGS` (6) rungs of 6 dB, clamped to that node's own
  probe max. The slot count is a protocol constant and every frame states the
  power it went out at, so the two ladders need not agree.
- **Sweep frame** `[DONE|AT_MAX|AT_MIN|txp+9] [echo txp+9|heard-cnt] [rssi]
  [snr ¼dB]`: every frame states its own power and echoes the lowest peer power
  heard with our measurement of it — an exact link-budget sample. DONE once the
  peer echoes one of our rungs (we then hold at found + 1 step); both-DONE
  mutually known → one final frame each, early exit. AT_MIN/AT_MAX mark
  chip-floor / probe-max clamping (an SX127x on PA_BOOST can't go below +2 dBm;
  the protocol floor stays −9 and the flag says why the rung moved).
- **Node flags byte** (in P1, P2 and both linkage frames): bit 0 roaming — a
  moving node needs more margin than a fixed one — bits 1–5 how many
  destination hashes the sender believes are its own, bits 6–7 reserved.
- **Timing is ISR/timer-side, not task-side.** T0 is the µs stamp the DIO1 ISR
  takes (`s_radioIrqUs`) for P1's end-of-air IRQ — the initiator's TxDone and
  the responder's RxDone are the same physical instant to within ISR latency, so
  both ends share a µs-accurate anchor regardless of task load. Each owned slot
  fires from an esp_timer one-shot (`probeSlotTimerCb`, esp_timer task): the
  callback builds the frame under `s_probeMux` (byte math only) and starts the
  transmit itself, so the priority-2 lora task and the 10 ms FreeRTOS tick are
  never in the TX timing path. A probe holds a `PM_NO_LIGHT_SLEEP` lock end to
  end: in light sleep the XTAL is off and timekeeping rides the RTC slow clock
  (a ~150 kHz RC oscillator on most boards) whose error dwarfs the slot guard,
  so the µs schedule is only trustworthy while the XTAL-fed systimer runs.
  Awake, inter-node crystal drift over a sweep is ppm-level (µs), far inside the
  guard, and the hold is bounded to the probe's few seconds. The task re-arms
  the timer at each own TxDone (`probeArmSlot`; a slot that can't be hit cleanly
  is skipped, never fired late), the callback forfeits a slot to an rx in
  progress, and `rearmRx` refuses to `startReceive` over a timer-fired transmit
  (`txActive` claim). The schedule close-out is the one task-side deadline left.
- **Probe max.** The radio's configured `tx_power`, chip-clamped — the same
  ceiling real traffic obeys, so a rung the ladder reaches is by construction a
  power an RNS frame may also use. The peer's max is learned from its P1/P2 txp.
- **Radio ownership.** While a probe runs (`probe.phase != PRB_OFF`) normal
  outbound waits in the ITS buffer, the CSMA machine belongs to the probe, and
  every rx frame goes through `probeOnRx`. `probeRestoreCfg` puts back header
  mode, preamble, sync and txp; any failure sets `s_configDirty` and the radio
  restarts clean.
- **Result.** The initiating CLI blocks (polling `resGen`) and prints both
  directions: found rung + the peer-measured rssi/snr there, and a cliff
  estimate interpolated below the rung via SNR headroom to the SF demod floor
  (RSSI vs. sensitivity once SNR saturates) — 6 dB rungs, ~1–2 dB answer. The
  responder logs one `info` line. Both report the run's cost — wall time plus
  the airtime it actually spent, split by direction (`Probe took N ms (tx: …,
  rx: …)`), accumulated in `loraMonPush` while `probe.phase != PRB_OFF` — and
  the CLI states the linkage outcome either way, so a run that had nothing to
  ask for is distinguishable from one that never asked. LoRaMon records every
  probe frame with its true per-frame power (`txPwrNow`) and protocol colour
  (`txOurProto`).

### 14.1 Why the exchange is shaped this way

Each of these was a correction, not a preference — worth keeping written down
because the obvious alternative is wrong in a way that only shows on the air.

- **Ascending ladder, not a binary search.** A binary search over the power
  range converges in fewer frames on paper, but it starts mid-range — shouting
  before it knows it has to — has no same-room fast path, and a lost frame
  corrupts its bracket. The ascending 6 dB ladder is monotone, so a missing
  frame *is* the measurement ("below the cliff") and loss handling and the
  search are the same code path. Resolution is recovered afterwards by
  interpolating below the found rung from measured SNR headroom to the SF demod
  floor, so 6 dB rungs still yield ~1–2 dB.
- **Two independent ladders.** Path loss is reciprocal, so each side can size
  its own ladder from its own measurement of the peer's last stated-power frame.
  An earlier design had both ends derive one shared ladder from the same four
  exchanged bytes; that only works if both compute bit-identically forever, and
  it made every later change a compatibility problem. Since every frame states
  the power it went out at, the two ladders never need to agree.
- **The opener anchors the schedule.** T0 is the ISR µs stamp of P1's
  end-of-air, which is the same physical instant at both ends. That removed a
  third handshake frame whose only job had been to give the responder the
  initiator's measurement before the ladder could start.
- **Timer-driven TX, and wakefulness pinned.** The slot transmit runs from an
  esp_timer callback, never the priority-2 lora task with its 10 ms tick, and
  the run holds `PM_NO_LIGHT_SLEEP` — in light sleep the XTAL is gated and
  timekeeping falls to the RTC slow clock (a ~150 kHz RC oscillator on most
  boards), which is orders of magnitude too coarse for a µs slot schedule. A
  µs-accurate anchor and a slot clock that stops being µs-accurate the moment
  the SoC dozes is the trap here.
- **Cooperation is not a nicety.** Estimating downlink power from uplink RSSI
  assumes the far end's TX power is constant. That holds for a dumb peer and
  fails the moment the peer is also adapting: two loops then chase a reference
  each is moving, and can diverge with both ends too quiet. So between capable
  nodes the power must be *stated*, which is why every frame carries its own
  txp and why the exchange identifies itself at all.

### 14.2 Cooperative hash linkage (0x02 / 0x03)

Which destination hashes belong to one node is otherwise only learnable by
catching an announce per hash (§13's cryptographic join). These two frames let
nodes simply tell each other, in the normal modem regime (explicit header, sync
0x42, LBT) so **any** radio in earshot parses them. Both are
`[magic][sender rnstransport first-4][node flags][4-byte hash]…`:

- **0x02 request** — "for each of these node hashes, send me the rest". Sent at
  the configured `tx_power`. `lora rfprobe` queues one automatically at the end of a
  successful run when the peer's advertised hash count exceeds what we hold for
  it (`probeMaybeAskHashes`) — the RF measurement is already done, so it is the
  cheapest moment to close the linkage gap. The CLI reports which of the three
  outcomes happened (`ProbeAskState`): queued, nothing to ask, or wanted-to-but-
  couldn't. Keep those distinct — collapsing "couldn't queue" into "complete"
  once hid a stuck queue behind a reassuring message.
- **0x03 reply** — the sender's own hashes. **Always** at max power and parsed
  by every listener, not just the requester: one widely-heard 0x03 saves
  everyone else from ever asking. Rate-limited to **one per half hour per
  radio** (`HASHSET_MIN_GAP_MS`), so answering a request is best-effort.

**Channel access is shared with normal outbound, and the order matters.**
`hashPktPoll` runs immediately before `drainOneOutbound` in the task loop, and
CSMA is a multi-pass state machine: `drainOneOutbound`'s "nothing queued →
`csmaResetAccess`" branch will clear a linkage frame's DIFS/backoff progress
every single pass unless it defers, so it returns early while `hashTxPending`.
In exchange the linkage frame carries the same `lbt_timeout` drop valve a queued
RNS frame has — otherwise one frame that can never win the channel would block
all Reticulum traffic behind it.

**A row is a node, not an identity.** One device legitimately runs several
Reticulum identities — its transport, `rnsh` and LXMF identities are all
distinct keys, and each announces separately, so each builds its *own* row via
the §13 cryptographic join. A 0x03 is precisely the assertion "these hashes,
and therefore these identities, are one device", so `neiLink` **merges** the
rows (`neiMergeInto`, shared with the announce path) rather than recording a
stub. `Neighbor` therefore holds `ids[NEI_IDS_MAX]`, not a single identity, and
every one prints as `id:<hash>` on the row. Merging is refused across the
us/them boundary — the frame is unauthenticated, so it may group a peer's
hashes but must never let a peer's claim swallow our own row. Only a hash that
resolves to no existing row is kept as a bare `link4[]` first-4.

Linkage also carries `advHashes` and `roaming` from the flags byte. A hash
already known in full from an announce is not duplicated, and `neiMergeInto`
carries the announced **display name** and the probe's settled **TX power**
across a fold — forgetting either once made a node lose it the moment its rows
were linked.

**The fold works in both time orders.** A 0x03 usually arrives after the
announces it groups, but a stub can equally be created first — the hash-set
follows a probe, while announces are minutes apart, and the sender's own
`node4` is stubbed the moment its 0x03 lands. So the announce path also looks
for a row that *claims* the announced dest's first-4 (`neiFindClaim4`: `node4`
or a `link4` stub, never another row's full dest, which would make a 4-byte
collision fuse two devices) and folds it in, as does `neiEnsureDest` for a dest
that only ever proved. Without that, an aspect announcing after the 0x03 that
already claimed it starts a row of its own and `lora n` shows the same device
two ways at once: the node listing its hash as `(not seen yet)` while a separate
numbered row prints the very same hash in full. A hash is on a row either as a
dest or as a stub, never both — `neiAddDest` drops the stub (`neiDropLink4`),
which also keeps `neiKnownHashes` from double-counting it and so from making
§14.2's `adv <= have` believe the linkage gap is closed.

**Our own rows fold unconditionally.** Each of our identities announces
separately and so builds its own `isUs` row, but they are all one device by
construction — and no 0x03 can ever tell us so, because we never hear
ourselves. So any announce that marks a row `isUs` folds every other `isUs` row
into it, and `lora n` prints exactly one `us` block.

## 15. Adaptive TX power (`s.lora.<n>.adaptive_txpwr`)

The first slice of `plans/adaptive-power.md`, and deliberately only that: **one
power determination per neighbour node, applied to every frame whose first RF
hop is that node.** There is no control loop — nothing walks the power down on
success, nothing jumps it up on failure, nothing re-measures. Off by default.

**Getting the number.** With the key on, `apPoll` walks the neighbour table each
task pass and picks the first node that has no determination yet, then kicks an
`lora rf` against it through the same `probe.req` handshake the CLI uses (one at
a time, ≥ `AP_PROBE_GAP_MS` apart, so a table that fills in one burst does not
probe in one burst). The run yields a determination **either way**:

| outcome | determination |
|---|---|
| the peer echoed one of our rungs (`myDone`) | that rung — measured |
| the probe failed, or ran without ever being echoed | the §13.1 reciprocity estimate **+ `AP_EST_MARGIN_DB` (5 dB)** |

The margin rides the estimate and not the measurement because the estimate
credits an unprobed peer with `s.lora.assumed_peer_txp` rather than knowing its
power, and because noise is not reciprocal even where path loss is. Recording a
determination *on failure too* is what stops this re-probing a vanilla peer
forever: the attempt happens once per node, and a peer that cannot answer is
answered for by reciprocity. Both are then clamped by `apClamp` to the chip's
range and to the configured `tx_power`; a measured rung already satisfies both,
since the ladder climbs to that same ceiling, so it is the estimate-plus-margin
path the clamp is really for.

**The gate on which nodes get probed is that reciprocity can already speak for
the node** (`neiEstimateCliff10` returns something). That one condition does two
jobs: it proves we have heard the node directly inside the bucket ring's hour,
so a probe is worth spending; and it guarantees the fallback has something to
fall back to. A node with nothing recent is simply never auto-probed, and keeps
the configured power.

**Walking the table, not triggering off the rx path.** "A hash we have no power
for" and "a node we have no power for" are the same question once the hash goes
through the table's identity clustering (§13, §14.2) — a hash newly linked to a
node that already settled needs no probe, which is exactly what the lookup
answers. So the determination is per *node* and covers every hash it owns,
including ones learned later by announce or by 0x03.

**Both ends settle.** `probeEnd` runs on initiator and responder alike, and
`p->usTxp` is our own lowest echoed rung in both roles, so a single exchange
gives each side its own number for the other. Answering a probe stays
unconditional (§14) — only *initiating* one and *applying* a determination are
gated on the key, so a node with the key off still lets its neighbours measure
themselves against it.

**Applying it.** `apTxPower` resolves the outbound frame's first-hop node from
the RNS header (`apNextHop4`, the table in `plans/adaptive-power.md` §6):
HEADER_2 names its next hop in the `transport_id`; a HEADER_1 SINGLE frame is
its own dest; link traffic and LRPROOFs are addressed to the `link_id`, whose
peer is the link's destination when that destination is not us. Anything else —
a delivery proof addressed to a packet hash, a link someone else dialed to us —
resolves to nothing and takes the configured power. **An announce always takes
the configured power**: it has no single next hop and must reach everyone.

`apApplyPower` is the one place the chip's power register moves on the tx path,
and `txPwrNow` is authoritative for what the chip is set to as well as what the
LoRaMon record is stamped with. The two must not drift: a frame to a quiet
neighbour would otherwise leave the next frame transmitting at its power while
being recorded at another.

**`lora n` prints it** as `USE <dBm>` on the capability line, with a `~` for a
determination derived from `EST` rather than measured — next to the `TX` a probe
measured and the `EST` reciprocity infers, so all three are comparable at a
glance.

### 15.1 The power request (0x04)

A second, independent half: instead of guessing what power a peer needs, **tell
it**. Four bytes sent back to back in front of the packet they relate to, in the
normal modem regime (sync 0x42, explicit header, preamble 12) so no reconfigure
sits between the two:

```
[0x04][suggested txp int8 dBm][rssi probeEncRssi][snr probeEncSnr]
```

The two payload fields are the two knowledge states, which is why either may be
a sentinel: if we know who the peer is we hold its history and send the
*answer*; if we don't, we send what we measured and let the peer close the loop
itself, since it knows the power it transmitted at. **Absence of the frame means
"use your own maximum"** — the entire fallback, needing no constant agreed
between the ends.

Sending the answer rather than the inputs is what makes it small. The far-end
noise floor is folded in by the only party that can know it, there is no shared
SF-floor model to keep in step, and there is nothing to signal when a peer
cannot comply: it clamps at its own ceiling, our measured path loss then reads
*too large*, so we ask for more rather than less and it parks at the ceiling
without oscillating.

**Precedence: an explicit request outranks our own estimate** (`apLinkSuggest`,
consulted before the node determination in `apTxPower`). The receiver folded in
its own noise floor, antenna and sensitivity, none of which a transmitter sees.

Built at exactly one site so far — **`apPwrReqFor`, on a link we are opening.**
An LR is the one unicast frame where we chose the destination and therefore hold
its history, and where one 4-byte prefix covers a whole session rather than a
single reply. Conditions, all of which must hold:

- the packet is a hops-0 HEADER_1 SINGLE `LINKREQUEST`;
- `adaptive_txpwr` is on;
- the destination resolves to a node with `ourProto` — only a peer that has
  spoken our air protocol will parse the frame, and to anyone else it is 35 ms
  of unparseable noise we cannot detect we wasted. That tag comes from an
  `lora rf` run or either linkage frame, so §14/§14.2 are what bootstrap
  eligibility;
- at least `AP_MIN_SAMPLES` recent frames back the estimate — one frame's RSSI
  moves several dB and has no business dialling a peer down;
- the resulting figure is below `AP_PWR_MAX_DBM`, since asking for a power no
  radio exceeds is what sending nothing already means.

The figure is `neiEstimateCliff10` + `AP_EST_MARGIN_DB`, and note it is a
**direct** measurement here rather than a reciprocal one: the estimate is the
peer's assumed power minus the headroom its frames actually arrived with at our
receiver, i.e. how much of its power was surplus *here* — exactly the quantity
we want to ask it to drop. Its one assumption is the power it transmitted at,
which the no-prefix-means-maximum rule makes true.

**Receiving.** `probeOnRx` consumes the frame at `PRB_OFF` and parks it in
`apRxSuggestPend`; the frame carries no binding field, so it binds **by
adjacency alone** and `handleRxDone` spends it on the very next RNS frame either
way. The `ours` early-return is what carries it across from its own frame to the
one it prefixes. `neiObserve`'s LINKREQUEST branch binds it to the `NeiLink` —
and only for a link dialled to *us*, since a request overheard on someone else's
link addresses nothing we will transmit on. From there `apTxPower` applies it to
the LRPROOF and every later frame of the session alike, because all of them are
addressed to the `link_id`.

A link is the only place this state is legal: we cannot name the initiator (an
LR carries no sender) but the `link_id` is a handle both ends share. An ad-hoc
exchange has no such handle, which is why the responder there must stay
stateless.

**Honouring a request is gated on the key, unlike answering an rfprobe.**
Answering a probe is unconditional because a probe run does not change
steady-state behaviour. Obeying a request does — it puts our transmit power
under someone else's control, observably, which is enough to correlate
identities across links. A node with `adaptive_txpwr` off must stay at its
configured power or the opt-out is not one.

**Framing.** The request and its packet share one channel access and one power —
the peer measures the pair as a single path sample, so sending the prefix louder
"for robustness" would make it a lie. An LR is 83 B and never splits, so the
pair fits the existing two-frame burst; `txOurProto` became per-frame because
the two frames share a burst but not a protocol.

**What is deliberately absent.** Everything in `plans/adaptive-power.md` §4.3
onward: the AIMD loop, the learned offset over a live estimate, the known-bad
floor, evidence-gated walk-down, failure detection, the broadcast regime derived
from the farthest neighbour, and the coupling to SF/BW/CR. Recovery from a bad
request — stop prefixing, and the peer returns to max on the retransmit — is
designed (`plans/adaptive-power.md` §3a) but **not built**: nothing yet notices
that a link failed. RNS retransmission is the only backstop, and its step is
`DEFAULT_PER_HOP_TIMEOUT`, 6 s.

## 16. Pitfalls

- **The LoRa rail is the board's, not this straddle's.** The radio is unreachable
  on SPI until whatever powers it is up and settled; `begin()` then returns
  `CHIP_NOT_FOUND` (−2) or `SPI_CMD_TIMEOUT` (−705). The board HAL brings the
  rail up before `spangapInit`.
- **Re-enable the GPIO interrupt after every RX drain.** The HAL trampoline
  disables it on each fire; `drainRadioIrq` must `gpio_intr_enable` the radio's
  IRQ pin or the radio goes silent.
- **Half-duplex: `splitPending` blocks all TX** until the second frame arrives or
  the 5 s timeout fires. Outbound bytes sit in the ITS stream buffer meanwhile —
  don't drain them in a tight loop.
- **`startReceive` after every `transmit`.** RadioLib leaves the radio in standby;
  without re-arming, RX is dead until the next config reload.
- **The rfprobe slot schedule is a compile-time agreement, and a mismatch is
  silent.** Both ends derive slot lengths from `PROBE_SLOT_GUARD_MS` /
  `PROBE_START_GUARD_MS`, so two nodes running builds with different values
  desynchronize within a few slots (~7 ms of drift per slot between 8 and 15 ms
  is most of a slot by the fourth) — the peer's frames land in your transmit
  slots or in the gaps. It presents as `heard 1 / sent 8` with the ladder
  climbing its full range, *and a plausible-looking result report*, because both
  cliffs still get filled in from whatever did arrive. *Flash both ends from the
  same build before drawing any conclusion from a probe.* The fix worth making:
  P1's byte 11 is reserved — put the initiator's slot length in it and have the
  responder adopt it, so a mismatch degrades to "one side follows the other"
  instead of silent garbage.
- **Airtime depends on framing, not just SF/BW/CR.** A headerless frame drops
  20 bits from the payload term and the rfprobe sweep regime also runs a
  6-symbol preamble instead of the configured 12, so
  `loraAirtimeSeconds(..., implicitHeader)` needs both told to it. Computing a
  4-byte sweep frame as explicit/preamble-12 over-stated it by ~11 ms at SF7
  (~47%), which silently corrupted LoRaMon bar widths, rx start times
  (`start = end − ToA`), the airtime rollups and the probe's own cost report —
  while the protocol timing stayed correct, because `probeToaS` had the implicit
  form. Two copies of one formula is how that happened; there is now one, and
  `airPreamble`/`airImplicit` are updated by `radioStart`, `probeSweepCfg` and
  `probeRestoreCfg` so a record can't be computed against the wrong regime.
- **`sync_word = 0x42` is the Reticulum-on-LoRa convention.** Generic LoRa nets
  use `0x14` (public) / `0x12` (private). A mismatched sync word is a silent
  radio — no frame ever surfaces.
- **Probe in the chip's own band.** A sub-GHz probe makes a 2.4 GHz SX128x read
  as absent (and vice versa) — `probeRadio` already branches on family; keep it.
- **The IRQ line is generic per family** (DIO1 on SX126x/SX128x, DIO0 on SX127x,
  IRQ on LR11x0/LR2021). The `LoraSlot.dio1` field name is historical; it holds
  whichever line the chip uses.
- **SPI host is 1-based in Kconfig.** Subtract one for the IDF enum (see §3); a
  straight cast collides with the board's shared bus.
- **PSRAM-stack task: no `printf`, no file I/O.** Use `info()`/`warn()`/`err()`
  only.
- **Keep the X-macro and Kconfig `choice` in lockstep.** The chip's numeric id is
  its position in `LORA_CHIPS`; reorder one without the other and every radio
  constructs the wrong driver class.
- **An RNode client's `detach()` is a trap for the naive mapping.** It sends
  `RADIO_STATE OFF`, then `CMD_LEAVE`, then closes. Obeying the OFF directly
  takes the radio down for rnsd every time a client shuts down cleanly — hence
  the deferred-off in §17.4.
- **A rejected echo is a churn loop, not an abort.** A client whose validation
  fails (the TX-power clamp is the realistic case) closes the port and re-runs
  the entire handshake every 5 s, forever. If a device looks like it is
  reconnecting endlessly, compare the client's configured parameters against
  what the radio can actually do before looking anywhere else.
- **Never send `CMD_STAT_RX`/`CMD_STAT_TX` or `CMD_ERROR` 0x03/0x04 to a
  client.** They crash its frame handler or raise "Unknown hardware failure" —
  see §17.6. This is the kind of thing a well-meaning "report more telemetry"
  change walks straight into.

## 17. RNode endpoint (`s.lora.rnode.*`)

A stock Reticulum `RNodeInterface` client attaches to this device as if it were
RNode hardware, over USB serial (how RNodes are normally used) and/or
RNode-over-TCP, and becomes the **third endpoint of the radio segment**. All
three — the radio, rnsd, and the client — see the same traffic; a packet
entering from any one is presented to the other two.

Everything here lives in `lora.cpp`'s RNode section. The serial transport rests
on a spangap-core mechanism, the serial-port handler registry
([cli-internals §3](../spangap-core/docs/cli-internals.md)); core knows only
"this port has a handler", never that it is RNode.

Protocol reference:
`RNS/Interfaces/RNodeInterface.py`.

### 17.1 Settings and transports

| Key | Default | Meaning |
|-----|---------|---------|
| `s.lora.rnode.enable` | 0 | master switch (pane row) |
| `s.lora.rnode.radio`  | 0 | which radio the endpoint exposes |
| `s.lora.rnode.serial` | −1 | serial port to claim; −1 = no serial |
| `s.lora.rnode.tcp`    | 0 | TCP listen port; 0 or −1 = no TCP. 7633 is the only useful value |

Global, not per radio: there is one endpoint for the device. **Both doors default
shut**: `.enable` alone opens neither, and each transport is opted into by
naming it. Enabling the endpoint must not put a listener on the network nobody
asked for — a LoRa segment reachable from any host on the LAN is a decision, not
a side effect of flipping a switch.

`.tcp` is therefore a two-value key in practice: **0, or 7633**. The client dials
7633 and nothing else — its `TCPConnection.TARGET_PORT` is hardcoded, and a
`tcp://host:port` config URI does not override it: the whole suffix is handed to
`getaddrinfo` as a hostname and resolution simply fails. Any other number here
opens a socket no stock client can reach.

`rnodeApplyTransports()` runs from the coalesced apply pass (§9) and does three
things: drops the session if the endpoint was switched off or rebound to another
radio; registers the TCP endpoint with net **once** and then drives the listener
by writing `s.net.rnode_port` (net's own `s.net.*` subscriber opens and closes
the socket — the two-step shape sshd uses); and claims or releases the serial
port. A refused serial claim — port 1 while the console presents only one — is
warned once and retried when `sys.usb.serial_ports` changes, which is why the
task also subscribes to that key.

The TCP door is compiled behind `CONFIG_SPANGAP_NET`, which is also what pulls
spangap-net into this component's `REQUIRES` when that straddle is staged: the
radio itself needs no network stack, so on a net-less build the door compiles
away and the serial one still works. `straddle.yaml` therefore does not `require:`
spangap-net — an optional door must not make a whole network stack mandatory.

### 17.2 One session, two doors

`RNODE_ITS_PORT` (0x524E, `'RN'`) is a stream-mode ITS server port on the lora
task, `maxHandles = 1`, 4 KB each way. Both net (a TCP client) and the core
serial machinery (a serial client) connect to it, and `onRnodeConnect`
discriminates by the **connect payload's length** — the serial machinery sends a
one-byte `serial_handler_connect_t`, net a `net_connect_t`. That length is the
only discriminator available, and nothing else needs one.

`maxHandles = 1` plus an explicit reject in `onRnodeConnect` is what enforces the
single-session policy across transports: a serial takeover attempted while a TCP
client is attached is refused, and because the refusal happens before any
takeover, **the console is not disturbed by it**. `onRnodeConnect` also rejects
while the endpoint is disabled or `s_stop` is set; `rns stop` drops an existing
session before parking.

State is one static `RnodeState`: handle, bound radio, the KISS decoder, an
inbound carry (a frame can complete mid-chunk and park a packet — the bytes after
it in the same read must not be lost), one parked decoded packet, and the
`echoPend` / `offPend` / `wantOn` / `txAlternate` flags. `onRnodeConnect` resets
all of it: the client's `TCPConnection.write` buffers frames while disconnected
and flushes them on the next successful write, so a freshly accepted socket may
carry stale pre-drop bytes. The decoder resyncs on `FEND` regardless.

### 17.3 Handshake — and why we claim to be an AVR

The client opens with `CMD_DETECT` (payload `DETECT_REQ`), `CMD_FW_VERSION`,
`CMD_PLATFORM`, `CMD_MCU`, and **over TCP re-sends that entire burst every 3.5 s
of transmit idle for the life of the connection**. Every handshake reply is
therefore stateless and repeatable — answered every time it is asked, not once.

- `CMD_DETECT` must be answered with a `CMD_DETECT` frame carrying
  `DETECT_RESP` (0x46). Any other payload *actively clears* the client's
  `detected` flag. Over TCP it has 5 s.
- `CMD_FW_VERSION` must clear the client's 1.52 floor or `validate_firmware`
  calls `RNS.panic()` — which is `os._exit(255)`. We report **1.78**.
- `CMD_PLATFORM` reports **`PLATFORM_AVR` (0x90)**, deliberately. Platform ESP32
  arms the client's "`CMD_RESET 0xF8` seen while online → `IOError`" teardown,
  and ESP32/NRF52 unlock its framebuffer methods — which makes its own
  `detach()` emit a framebuffer-disable frame. AVR sidesteps both. It is *not*
  about display polling. `CMD_MCU` answers `0x91` (the AVR RNode's 1284P); the
  client only records it.

A command's payload bytes are what the client dispatches on, so **a zero-payload
frame is read and discarded**. `CMD_READY` therefore ships one `0x00` byte.

### 17.4 Configuration, echo, and the validation window

`CMD_FREQUENCY` / `CMD_BANDWIDTH` are 4-byte big-endian Hz; `CMD_TXPOWER` /
`CMD_SF` / `CMD_CR` one byte (dBm / SF / CR denominator); `CMD_ST_ALOCK` /
`CMD_LT_ALOCK` two bytes of percent×100, sent only when the client's config sets
airtime limits. The burst **always ends with `CMD_RADIO_STATE`**.

Each is executed by writing the ordinary `s.lora.<n>.*` key on the bound radio,
so it flows through the normal config path (§9) and re-registers with rnsd —
and persists in NVS. Range checks: frequency and bandwidth against the same
bounds the unit bridge uses (§10), SF 5..12, CR 5..8, TX power clamped to 22 dBm.
Each write calls `rnodeCfgTouched()`, which sets `echoPend` **and** arms the
apply — self-arming even when every write was a no-op, so the echo always fires
rather than waiting on a change storage saw no reason to report.

The client then sleeps — **0.25 s on serial, 1.5 s on TCP** — and validates the
*echoed* values. `bandwidth`, `txpower`, `sf` and `state` are compared
unconditionally, so an absent echo is a mismatch. **Frequency is optional** but,
if present, must be within ±100 Hz — which is why `rnodeEchoFlush()` sends it
from the applied value, not from what was asked. CR is never validated. On a
mismatch the client closes the port and re-runs the whole handshake **every 5 s,
forever**: a churn loop, not a one-shot abort. That is the failure mode behind
the TX-power clamp — a client asking for 23 dBm gets an honest 22 dBm echo and
churns. Lying to it would be worse; the README warns instead.

`rnodeEchoFlush()` runs at the end of the apply pass and emits each frame in one
`itsSend` with whole-frame space checked first: the client never flushes a
command frame it has only part of, so a stream-mode partial write would leave it
waiting forever and desynchronise everything after. If ON was asked for and the
radio failed to start, a `CMD_ERROR 0x01` follows so the client tears down
cleanly and re-dials.

**Deferred radio-off.** `RADIO_STATE OFF` sets `offPend` and nothing else. The
client's `detach()` is OFF → `CMD_LEAVE` → close (with a 0.5 s grace before the
TCP socket close), so a literal OFF → `enable = 0` mapping would take the radio
down for rnsd on every clean client shutdown. `CMD_LEAVE` and disconnect cancel
it; a client that turns the radio off and **stays connected** is honoured at the
apply deadline (`rnodeSettleOff`).

Airtime locks are echoed back as zero and never enforced — governance here is
LBT/APPC plus rnsd's announce cap, and the client parses these echoes without
ever validating them.

### 17.5 Data and the three-way bridge

`CMD_DATA` frames both ways. Inbound stats are optional and sticky: the host may
precede a data frame with `CMD_STAT_RSSI` (rssi + 157) and `CMD_STAT_SNR`
(snr × 4, signed), and the client applies them to the **next** data frame and
clears them after — so they must come *before* it, both of them and in that
order, because its Transport drops the SNR unless an RSSI came too. The client's
`HW_MTU` is 508 with silent truncation; RNS_MTU 500 fits.

The fan-out point for locally-originated packets is **`beginTx`**, and that is
the rule: *"presented to the radio" means transmitted*, so a packet the LBT valve
drops never aired and is bridged nowhere. A packet is bridged if and only if it
went on air.

- **client → radio + rnsd.** `drainOneOutbound` is a two-source drain. It
  computes availability across rnsd's ITS bytes **and** the parked rnode packet
  before anything else, because the "nothing queued" branch calls
  `csmaResetAccess()`: gated on rnsd alone it would wipe channel-access progress
  every pass while an rnode packet waited, and that packet could never win the
  channel — the same failure the `hashTxPending` guard documents. The rnode
  source needs no rnsd handle. With both pending the two alternate
  (`txAlternate`). `beginTx` then calls `rnsdInject()`, factored out of
  `deliverInbound`, with a **synthetic signal: −10 dBm, 10.0 dB SNR** —
  top-of-scale "perfect local", impossible over the air, so an injected packet is
  unmistakable in every signal view.
- **rnsd → client.** `beginTx` calls `rnodeForwardData(..., withStats=false)` —
  our own transmissions carry no measured signal, and stats are optional.
- **radio → client.** `deliverInbound` calls it with `withStats=true`, from
  `rssiLast`/`snrLast`. Only reassembled packets that are not our own air
  protocol reach `deliverInbound`, so the client sees exactly the Reticulum
  traffic. Forwarding is **all-or-nothing**: the stat frames and the data frame
  are space-checked together and skipped together (with a warn), because a
  partial stream write in the middle of a KISS frame corrupts everything after
  it.

`CMD_READY` is sent after every completed client-originated transmit — from
`txRearmRx`, which is exactly where the client's frame is finished with the
radio, and also when the LBT timeout sheds it. Harmless with the client's flow
control off, **mandatory** with it on.

`nextDeadline()`'s outbound clause separates gating from availability for the
same reason the drain does: an rnode packet is pending without any rnsd handle,
and one conjunction would leave it unable to wake the loop.

A client configured with a beacon injects unsolicited ≤32-byte callsign
`CMD_DATA` frames. They are not Reticulum payload; they simply air.

### 17.6 Frames we must never send

These are client-side traps, not style preferences:

- **`CMD_STAT_RX` (0x21) / `CMD_STAT_TX` (0x22)** — the client's handler calls
  `ord()` on an int, raising `TypeError` and taking the interface offline.
- **`CMD_ERROR` 0x03 / 0x04** — unhandled, so they fall through to
  `IOError("Unknown hardware failure")`. `CMD_ERROR 0x01` is the one usable
  code: a clean `IOError` and the 5 s reconnect loop.
- **A spontaneous `CMD_RESET`** — with a platform of ESP32 it is an `IOError`;
  we report AVR, but there is still no reason to send one.

### 17.7 Shared channel

The client's stack governs its own announce rate, so RNode-origin traffic
bypasses rnsd's announce cap; LBT/APPC still gates its airtime like anything
else. And the client's radio settings are **written to NVS**: they survive a
reboot and overwrite what the operator set. That is deliberate — the endpoint is
meant to behave like RNode hardware — but it is the one thing about this feature
an operator has to know, so the README says it too.
