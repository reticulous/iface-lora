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
- **IFAC plumbing** — reading `s.lora.<n>.ifac_netname` / `ifac_size` and the
  `secrets.lora.<n>.ifac_netkey` secret and handing them to `rnsd` in the
  `rnsd_iface_t` connect payload; `rnsd` does the actual access-code crypto.
- **The browser panel and generated LCD/web settings** (`browser/`, the
  `settings:` block in `straddle.yaml`).

## 2. The task

One FreeRTOS task — **core 0, priority 2, 8 KB PSRAM stack** (larger than other
interfaces for the LoRa frame buffers and RadioLib state). It services *all*
radios; per-radio state lives in `s_radios[]` (`LoraRadio`).

**Boot order.** The task first waits on `rns.ready` (`waitForFlag`, 120 s) — a
brownout/boot-loop node must never reach RF TX, and registering before `rnsd`'s
ports exist is pointless. If `rns.ready` never comes, the task **kills itself**
(`killSelf`) rather than spin. It then `itsClientInit(kNumRadios)`, subscribes
to `s.lora` and `secrets.lora` storage changes, constructs each radio + HAL and
probes for presence (§4), and `waitForTime(0)` so the first announces aren't
1970-stamped (a LoRa-only node with no GPS/RTC just eats the bounded wait;
`s.sys.time_wait_s = 0` skips it).

**Single wait point.** `itsPoll(nextDeadline())` is the only blocking call. It
wakes on an ITS message (an outbound packet from `rnsd`, or a config-change
notify), a task notification from any radio's IRQ ISR, or a computed deadline.
When outbound bytes are queued and a radio is free, `nextDeadline` returns 0 to
drain on the next turn. With nothing pending — no queued outbound, no split-RX
in flight, no deferred stats flush, no unregistered radio — it returns
`portMAX_DELAY`, so an idle link blocks until a real ISR/ITS event and the chip
can light-sleep. RX stays prompt regardless: DIO1 is a light-sleep GPIO wake
source and the ISR notifies the task.

**Per-turn, per radio:** drain completed RX (§6), expire a stale split, re-
register with `rnsd` if the handle dropped while enabled, drain one outbound
packet.

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
pass `csmaClear(r)`, a non-blocking channel-access state machine modelled on
RNode's CSMA/CA (airtime-based persistence is not yet implemented). Carrier
sense is the instantaneous channel RSSI — `channelRssi(r)` reads `getRSSI(false)`
without leaving continuous RX (dispatched per chip, since that overload isn't on
`PhysicalLayer`) — compared against a tracked noise floor (`channelBusy`: the
floor snaps down fast and creeps up slowly, so an active channel can't inflate
its own reference; busy = `rssi > floor + CSMA_RSSI_MARGIN_DB`). The machine:

- `CSMA_IDLE` → begin an inter-frame (DIFS) listen.
- `CSMA_DIFS` → require the channel idle for `difsTicks`; any activity restarts
  the window. Once satisfied, draw a backoff of `[0, 2^cw)` slots.
- `CSMA_BACKOFF` → count the backoff down one slot at a time while the channel
  stays idle; if it goes busy, widen `cw` (exponential, capped at `CSMA_CW_MAX`)
  and re-listen. Backoff drained on a free channel → grant TX, reset `cw`.

`slotTicks` derives from the LoRa symbol time (`2^SF / BW`, clamped
`CSMA_SLOT_MS_MIN..MAX`); `difsTicks` is two slots. The machine is driven from
the task loop: when access is deferred the frame stays queued and
`nextDeadline()` wakes the task at the next slot boundary to re-sense (never at
0, which would peg the task). Per-radio toggle `s.lora.<n>.lbt` (default on);
`lbt=0` reverts to blind transmit. The only other TX guard remains
`splitPending`.

Outbound packets arrive from `rnsd` over the registered handle: `onRnsdRecv`
calls `drainOneOutbound`, which — once LBT clears — `itsRecv`s one packet and
transmits it. One packet per loop turn so RX re-arms between back-to-back sends.

**Per-frame trace.** `log lora debug` turns on a `dbg` line per on-air frame
(`loraTraceFrame`): direction, length, and a 20-byte hex preview — RX lines also
carry `rssi`/`snr`, and a CRC-failed RX logs `rx CRC-FAIL` with rssi/snr (LoRa's
error-check is the CRC; RadioLib exposes no corrected-bit count). The formatting
is guarded by `logIsDebug("lora")`, so the trace costs nothing when off. The tag
is the task name `lora`, so `log lora debug` is what gates it.

## 7. rnsd registration

`registerWithRnsd(r)` opens an ITS connection to `RNSD_PORT_IFACE` with an
`rnsd_iface_t`:

- `name` = `lora/<idx>` (iface-lora owns slot-name uniqueness; `rnsd` takes it
  verbatim);
- `mtu` = 500, `bitrate` = the airtime-derived value (§8);
- `mode` from `s.lora.<n>.mode` via `modeFromString` → an `RNS_IFACE_MODE_*`
  value (default `GATEWAY`);
- `in = out = 1`, `fwd = 1` for `FULL`/`GATEWAY` (forwarding/transport modes),
  `rpt = 0`;
- IFAC fields from `s.lora.<n>.ifac_netname` / `ifac_size` and
  `secrets.lora.<n>.ifac_netkey`.

The ITS connect **ref is the radio index**, so `onRnsdDisconnect(ref)` finds the
radio and clears its handle; the task loop re-registers on the next turn if the
radio is still enabled. If registration fails but the radio is on-air, the state
goes `rnsd_unavailable` and the loop keeps retrying — RF stays up.

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
sets `s_configDirty` and notifies the task. On the next turn `applyConfig(r)`
reads `s.lora.<n>.enable`: disabled → `radioStop`; enabled → `radioStop` then
`radioStart` (a cheap ~stop/start that avoids tracking which field changed).

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
defaults under a `s.lora.version` gate (`LORA_VERSION = 2`) for radios **1..**
only — radio 0's defaults come from this straddle's `settings:` block in
`straddle.yaml`, **except** `s.lora.0.bandwidth`: its pane row binds the kHz
display key rather than the Hz config key, so `loraInit` seeds that one default
(125 kHz) for radio 0 directly. Frequency and TX power carry no default
(region/antenna — the user must pick); everything else defaults so an
enable-toggle alone gets a radio up. `loraInit` does **not** touch any power pin
— the board owns the LoRa rail.

## 12. Pitfalls

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
