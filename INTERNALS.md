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
- **The LoRa interface service** — one FreeRTOS task driving every configured
  radio, cut into modules with a single direction of dependency
  (`plans/structuring-lora-code.md`): `lora.cpp` holds only the tasks, the
  config lifecycle and the wiring; `lora_radio` the chip dispatch and RadioLib
  calls; `lora_queue` the packet queue that is the one seam between the bridge
  and the radio; `lora_bridge` the rnsd/RNode packet paths and the outbound
  drain; `lora_peers` the peer table and `lora_observe` the Reticulum
  inspection that fills it; `lora_csma` medium access; `lora_airtime` the
  per-channel budget ledger; `lora_chanplan` the regime channel tables;
  `lora_power` adaptive transmit power; `lora_mon` telemetry; `lora_rnode` the
  RNode endpoint; `lora_cli` the CLI; and `supe_engine` + `lora_supe` the SUPE
  state machine and its platform boundary. `lora_priv.h` carries the shared
  types. `supe.{h,cpp}`, `lora_queue.{h,cpp}` and `supe_engine.{h,cpp}` are
  deliberately free of ESP-IDF, RadioLib and FreeRTOS so `esp-idf/test/` builds
  them with a plain g++.
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
- **The modem's own reception evidence** (§6b) — the latched preamble/header
  IRQ bits read as a channel-busy verdict, which is the only way to see a frame
  arriving below the noise floor, with staleness deadlines so a preamble that
  became nothing cannot block transmit.
- **SX126x hardware corrections** (§4, §4a) — the PA over-current trip RadioLib
  leaves at 60 mA, the 0x8B5 RX-sensitivity register bit, a TCXO-off retry that
  distinguishes a mis-set reference voltage from absent hardware, a check that
  the configured frequency is one the part has an image calibration for, and the
  periodic front-end recalibration that keeps a latched gain control from
  deafening the receiver.
- **IFAC plumbing** — reading `s.lora.<n>.ifac_netname` / `ifac_size` and the
  `secrets.lora.<n>.ifac_netkey` secret and handing them to `rnsd` in the
  `rnsd_iface_t` connect payload; `rnsd` does the actual access-code crypto.
- **LoRaMon** (§12) — a per-on-air-frame recorder whose storage subtree *is* the
  ring, plus the browser and LCD viewers that plot power, signal and protocol on
  a dBm axis with a click/touch zoom stack.
- **The passive neighbour table** (§13) — who is in RF range, built purely from
  observing rx + tx RNS traffic, with a cryptographic identity join; surfaced as
  `lora n[eighbors]`.
- **`lora a[nnounce]`** (§14) — a copy of every announce this node originates,
  kept for an hour and repeated on demand. Announces are not held back, batched
  or paced: they air when `rnsd` hands them over.
- **Adaptive TX power** (§15) — a per-peer offset controller
  (`power = clamp(maximum − offset)`) walked down on evidence from SUPE's
  reverse MANIFEST and back up fast on a miss, plus the reciprocity estimate
  for peers that never detour.
- **Channels and frequency agility** (§18) — a channel index on every record and
  every measurement, the numbered regime table that names a channel set, and the
  per-second channel-RSSI beat that measures it. Instrumentation: what actually
  transmits off the hailing channel is SUPE.
- **SUPE** (§19) — unicast traffic leaves the shared channel for short private
  high-rate detours: the requester states its load, the peer answers with the
  channel and the budget (SUPE_GRANT), both directions ride one detour. Off by
  default. The arithmetic — regime tables, the family-filtered ladder, the
  codec, every deadline — is `supe.{h,cpp}`; the state machine is
  `supe_engine.{h,cpp}`, single-threaded and lock-free behind the `SupeHost`
  interface `lora_supe.cpp` implements; all three are host-tested in
  `esp-idf/test/` (`supe_core_test`, `supe_engine_test`, and the
  `supe-ladder-vectors.txt` conformance file).
- **The RNode endpoint** (§17) — a stock RNS `RNodeInterface` client attaches
  over USB serial and/or TCP 7633 and becomes the third endpoint of the radio
  segment, executing radio commands by writing the ordinary `s.lora.<n>.*`
  keys.
- **The browser panel and generated LCD/web settings** (`browser/`, the
  `settings:` block in `straddle.yaml`).

## 2. The task

One FreeRTOS task — **priority 1, 10 KB PSRAM stack** (larger than other
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
no announce replay running, no proof expectation outstanding, no viewer open — it returns
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

**Per-turn, per radio:** drain completed RX (§6), expire a stale split,
re-register with `rnsd` if the handle dropped while enabled, expire
neighbour-table proof expectations (§13), run the SUPE glue poll (the announce
beat, the offer launch and the transaction watchdog, §19), then run the
outbound drain — ingress into the packet queue, the SUPE classifier on its
head, channel access, transmit (§6). Once per turn: decode whatever the RNode
client has sent (§17.5), and — while a LoRaMon viewer is open — run every
radio's 1 Hz frame expiry and airtime publication (§12).

**Stats are event-driven, not timed.** Every published stat is a cumulative
counter (tx/rx bytes and frames, `crc_err`, `split_rx_timeout`) or a last-packet
reading (`rssi_last`, `snr_last`) — none move without a tx/rx event, so a timed
republish would only burn battery. The task sums the counters each turn; a change
means traffic happened, and stats are published **at most once a second** (a
change inside the 1 s window defers to the boundary, where `nextDeadline` wakes
the task to flush the coalesced values). The keys are seeded once at startup so a
consumer sees a radio before any traffic. A running-but-unregistered radio holds
a 1 Hz retry wake until registration takes.

**An idle node holds almost no standing wake.** Every deadline `nextDeadline`
computes exists only while its work does: the channel-RSSI beat only while a
LoRaMon viewer is open (§18.3), SUPE's airtime verdict only while the window
holds agile airtime (§19.6), proof expectations only while one is outstanding.
With nothing pending the task blocks on `itsPoll(portMAX_DELAY)` and the SoC
light-sleeps until DIO1 or an inbound message; with SUPE enabled a long-period
wake is added by the announce beat (`SUPE.announce_interval`, default 30 min).
This is a battery invariant, not an optimisation: a beat added to this task is a
per-second CPU+SPI wake on every deployed node, so anything periodic must gate
itself on whether its consumer exists.

**The one exception, stated as one.** The front-end recalibration beat
(`s.lora.<n>.agc_reset`, default 5 min, §4a) has no consumer to gate on, because
the failure it prevents removes the evidence that would trigger it: a receiver
whose gain has latched hears nothing, so no traffic arrives to wake anything. It
is a standing wake by necessity, its period is a setting, and `0` turns it off.
Nothing else may claim the same exemption without the same argument.

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

**`delay()` rounds up, and must.** A FreeRTOS tick here is 10 ms, so
`pdMS_TO_TICKS()` truncates anything shorter to zero and `vTaskDelay(0)` is a
bare yield — the wait does not happen. RadioLib's millisecond delays are
hardware timing, not politeness: the ~500 µs the SX126x needs to finish entering
sleep before an NSS edge can wake it again, the reset pulse width, the TCXO
settle. Truncated, the wake-up pulse after `SetSleep` lands while the part is
still on its way down, the chip sleeps through it, BUSY stays high and the next
command burns RadioLib's full 1 s BUSY timeout before returning
`SPI_CMD_TIMEOUT`. Every caller is a bring-up, reset or sleep path asking for a
*minimum*, so `EspIdfHal::delay` rounds up to whole ticks — overshooting is free
there, and it keeps the wait a real sleep rather than a busy-wait.

**The raised-line backstop.** Everything above depends on the task being told
about a raised DIO1. The level trigger makes that robust — a line that goes high
while the interrupt is disabled fires as soon as it is re-enabled, where an edge
would have been lost — but it leaves one hole: a path that disables the interrupt
and fails to re-enable it strands a completed frame behind a line nobody is
watching, and the task then blocks on `portMAX_DELAY` beside a radio that will
never speak again. So the line itself is checked, as a plain `gpio_get_level`
with no SPI behind it (`radioIrqLinePending`): the per-radio pass services a radio
whose line is asserted even without an ISR notification, and `nextDeadline()`
returns 0 rather than sleeping while one is. That pair cannot spin, because
`serviceRadio` either consumes the cause or — finding neither TxDone nor RxDone
behind an asserted line — clears the chip's flags, re-enables the interrupt and
re-arms RX, warning at most once per 10 s. A permanent wake source is the one
outcome this must not have.

**Chip IRQ bits are not RadioLib's.** `getIrqFlags()` returns the chip's own
register. RadioLib's `RADIOLIB_IRQ_*` values are positions in a radio-agnostic
enum, and the two coincide only where a family happens to lay its register out in
the same order — which SX126x does and others do not. `radioIrqCache` translates
the flags this loop tests once per bring-up, through `getIrqMapped()` (a table the
chip class fills in its constructor, so no SPI), into `r->irq*`. Test against
those, never against a bare `1 << RADIOLIB_IRQ_…`.

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

**PA over-current trip (SX126x only).** RadioLib's `SX126x::begin()` writes a
60 mA limit into the OCP register for every part, and its `setOutputPower()`
reads that register and writes it back unchanged — so nothing in the library ever
raises it. An SX1262 driving +22 dBm draws about 118 mA. `radioBegin` therefore
sets the trip explicitly to the datasheet's own post-`SetPaConfig` value:
**140 mA** for the parts that reach +22 dBm, **60 mA** for the SX1261, whose PA
tops out at +15 dBm and must not be handed a ceiling it cannot survive
(`radioOcpMilliamps`). Re-applied after every recalibration (§4a).

**RX-sensitivity register patch (SX126x only).** Bit 0 of register **0x8B5**,
undocumented and recommended by both Semtech and Heltec. Applied through
`Module::SPIsetRegValue` rather than the chip class, because `SX126x::writeRegister`
is protected. It must be re-applied after every `CALIBRATE_ALL`, which clears it
— see §4a, where leaving it out would make the recalibration beat *cost*
sensitivity rather than preserve it.

**Presence probe.** `probeRadio` runs a bare `begin()` (safe defaults + the
slot's TCXO voltage) at boot; `RADIOLIB_ERR_NONE` means the radio answered on
SPI. It probes in the chip's **own band** — 2450 MHz/812.5 kHz for SX128x, else
434 MHz/125 kHz — because a sub-GHz probe would make a 2.4 GHz part read as
absent. The result feeds the boot log and the `lora` CLI; `radioStart`
re-`begin()`s with the real config when the radio is enabled.

**The TCXO fallback.** A `begin()` that fails with a SPI *command* error
(`SPI_CMD_TIMEOUT` / `_INVALID` / `_FAILED`) while a TCXO voltage is configured is
the signature of a board whose reference is a plain crystal, or whose DIO3 does
not feed the oscillator: the chip waits for a TCXO that never reports ready and
answers the next command with an error. `radioBegin` retries the whole call with
the voltage at zero, and on success warns naming `CONFIG_LORAn_TCXO_MV`. Without
it that board reads as **absent** — a wrong Kconfig value and missing hardware
are indistinguishable in the boot log, and the wrong one is far more likely.

**The image-calibration band check (SX126x only).** `SX126x::calibrateImage()`
has factory calibrations for five bands — **430-440, 470-510, 779-787, 863-870,
902-928 MHz** — and falls back to `calibrateImageRejection(freq ± 4 MHz)` for
anything else, which RadioLib itself calls "may or may not work". A frequency
outside all five is legal (the part tunes 150-960 MHz) but is far more often a
typo, and a typo is invisible from the device: the radio comes up, publishes
`state=up`, reports a **quiet** noise floor because the band really is empty, and
simply hears nothing. Every other setting still matches its neighbours, so `lora
<n>` reads healthy on both sides of a link that does not exist. `radioBegin`
therefore checks `freq` against the same table — with the same truncation to
whole MHz RadioLib uses, so the check cannot disagree with the call it describes
— and warns naming the bands. It is a warning, not a refusal: out-of-band
operation is a legitimate thing to ask a part for.

## 4a. Analog front-end recalibration (`s.lora.<n>.agc_reset`)

An SX126x that has heard a strong signal can leave its automatic gain control
latched at that setting, and a receiver stuck at low gain hears nothing
afterwards. Neither `standby()` nor a fresh `startReceive()` clears it; only
powering the analog front end down does. `radioAgcReset` does exactly that and
puts the chip back:

```
sleep(retainConfig=true)          warm sleep — the analog front end loses power
standby(STANDBY_RC, wakeup=true)  the state calibration is specified from
calibrate(CALIBRATE_ALL)          every block: ADC, PLL, image, RC oscillators
calibrateImage(current channel)   CALIBRATE_ALL's image calibration defaults to
                                  a band that is probably not ours
re-apply DIO2-RF-switch, boosted gain, OCP, the 0x8B5 patch
csmaNoiseFloorReset               the front end that measured the floor is gone
radioStartRx
```

The re-apply is not belt and braces: calibration resets settings layered on top
of `begin()`, and the 0x8B5 bit in particular, so a beat that skipped it would
strip the RX sensitivity it exists to protect.

**Scheduling is the interesting part.** This is the one periodic wake the radio
task holds without a consumer asking for it (§2 makes that a battery invariant),
and it is here on purpose: the failure is silent and self-sealing. A deaf
receiver hears no traffic, so nothing wakes the task, so no event-driven repair
can ever fire. Only a timer reaches it. The period is a setting — `agc_reset`,
in seconds, default **300**, `0` = off — and the default is minutes rather than
the ~60 s other LoRa firmwares use because the trade is explicit: a wake plus a
few ms of chip work every five minutes, against a bound of five minutes on how
long a latched receiver can stay deaf.

`agcResetPoll` runs from the task loop and takes the radio only when it is
genuinely idle: nothing in flight either way, no channel access under way,
nothing queued, on the hailing channel rather than mid-detour, and no reception
in progress (§6b). A beat that arrives at a busy radio is **deferred by a full
second**, never left past-due — an overdue deadline makes `nextDeadline()` return
zero and spins the task for as long as the radio stays busy, which is the same
trap `rssiSamplePoll` documents.

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

**TX — `beginTx` / `startTxFrame` (lora_bridge).** Transmission is
**non-blocking**: `startTransmit()` fires the chip and returns; the TxDone IRQ
wakes the task, which finishes the frame in `serviceRadio` and either sends a
split second frame or re-arms RX. One or two frames are sent depending on
length; `tx_bytes` counts the RNS payload, `tx_frames` counts each LoRa frame.
An aborted transmit (the TxDone watchdog) still credits its airtime — the
regulation counts emissions, not successes (SUPE.md §14.4).

**The oscillator across a chain of frames (`radioHoldOsc`).** A frame that ends
drops the part to `STDBY_RC`, which powers the TCXO down, so the next `SetTx`
waits out the whole programmed TCXO startup (RadioLib asks for 5 ms) before a
carrier appears — the bulk of the dead air between the two halves of a split and
between the frames of a train. Where the next frame is already spoken for and no
one else may use the medium anyway, that wait buys nothing, so `startTxFrame`
puts the part's Rx/Tx fallback at `STDBY_XOSC` for the length of the chain —
another frame of this packet, or any frame inside a live transaction — and
`rearmRx` drops it back the moment the radio returns to plain listening. The
driver's `standbyXOSC` flag and the chip's fallback register are set together:
the first governs the standby RadioLib takes on our behalf, the second where the
part lands by itself, and only both together avoid the wait. Never held idle —
the standing cost is the chip's standby delta plus the board's TCXO current, the
larger of the two by an order of magnitude. SX126x only; other families have no
equivalent in RadioLib and keep the gaps they have.

**Ingress is gated on the transaction, not on the radio.** `drainOneOutbound`
pulls from rnsd and the RNode client (`queueFill`) *above* the `txActive` and
`supeHoldsRadio` returns and *below* the transaction one. The distinction is the
protocol's: what a transaction carries is declared before it runs — the
requester's load in the START, the duration everyone else holds for in the GRANT
— so a packet arriving after that cannot join it and must not disturb the queue
the engine is walking. Until then it can, and the polite wait before a START is
hundreds of milliseconds, which is where most of the chances to coalesce live. A
packet pulled in during that wait is still in the queue when `headRun` measures
the load, and rides the very detour being waited for. Pulling while a frame is on
air is safe because `queueSendHead` consumes the head the moment `beginTx` has
copied the bytes out, so the in-flight packet is no longer in the queue for a
later push — or a per-peer cap eviction — to touch.

**Half-duplex coordination.** LoRa can't transmit while receiving, so a pending
split RX must not be interrupted. `drainOneOutbound` early-outs while
`r->splitPending` is set (or the radio isn't running, or `rnsd` isn't
connected); the outbound packet stays in the ITS stream buffer and is revisited
once the split completes or times out.

**Listen-before-talk (CSMA/CA).** Before a queued frame is transmitted it must
pass `csmaClear(r)`, a non-blocking channel-access state machine. A sense is two
questions. First, is the modem already receiving something (§6b) — the only one
of the two that can see a frame arriving below the noise floor. Then carrier
sense: the instantaneous channel RSSI — `channelRssi(r)` reads `getRSSI(false)`
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

**On the SPI cost.** Each sense is one `getIrqFlags()` and one `getRSSI(false)` —
two SPI transactions, read at the DFS floor (the re-sense wakes are timeout-driven,
so they don't boost the CPU). A transmit therefore issues a burst of these across
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

**The packet queue is the one seam.** Everything outbound is enqueued and the
engine (or the plain drain) dequeues; nothing else passes between the bridge
and the radio. `queueFill` pulls from rnsd's packet link zero-copy
(`itsRecvRef` — the heap block lives in the queue untouched) and from the RNode
client's parked packet (copied; that leg is a byte stream), alternating so
neither endpoint starves the other. Each packet is stamped at ingress with its
peer id and its tag — the first three bytes of its first address field — by the
observer, so nothing downstream parses Reticulum. Backpressure has two levels:
a per-peer cap drops that peer's oldest (invisible to rnsd; Reticulum tolerates
loss), and the global cap simply stops consuming — rnsd's send toward us then
blocks ~100 ms and drops with a warning (`rnsd.cpp`, iface out), which is the
one bit it sees. `drainOneOutbound` runs the SUPE classifier on the head
(hold / drop / offer / plain), wins the channel, and transmits; the inbound leg
is zero-copy too (`rnsdInject` builds the rx_signal-prefixed block and hands it
over with `itsSendOwned`, freeing it itself on the one failure path).

**The verbose tier has to be compiled in.** `ESP_LOGV` is dropped at build time
unless `CONFIG_LOG_MAXIMUM_LEVEL` allows it, and the IDF default stops at debug —
so on a build without `CONFIG_LOG_MAXIMUM_LEVEL_VERBOSE=y` (reticulous'
`sdkconfig.defaults` sets it) moving a trace to verbose does not move it, it
deletes it, and `log lora verbose` shows *less* than debug rather than more. It
costs about 20 kB of format strings and emits nothing by default: the ceiling is
what may be printed, `s.log.level` is what is.

**Per-frame trace.** `log lora verbose` turns on a line per on-air frame:
direction, length, channel, and a 20-byte hex preview — RX lines also carry
`rssi`/`snr`, and a CRC-failed RX logs `rx CRC-FAIL` with rssi/snr (LoRa's
error-check is the CRC; RadioLib exposes no corrected-bit count). The formatting
is guarded by `logIsVerbose("lora")`, so the trace costs nothing when off. The
tag is the task name `lora`.

**Verbose, not debug, and the split is a discipline rather than a preference**:
debug carries decisions and verbose carries frames. At debug a SUPE detour
reads as a short story — START, GRANT, MANIFEST, trains, home — with no frame
dumps between the lines; at verbose the same story is interleaved with every
frame that flew. A line that would fire per packet inside a train belongs at
verbose. See §19.

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

**Airtime accounting.** `appcAddAirtime()` credits every frame at TxDone, sweep
included. Time-on-air is bucketed into `APPC_BIN_MS` (7500 ms)
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
  `lbt_timeout`, and when an announce replay takes or releases the radio. Upstream would
  carry it into the next frame; here the machine is shared by three producers
  (queued RNS traffic, the announce replay, manual CLI transmits) and stale progress must not
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

**One floor per channel, carried across retunes (`csmaFloorSwitch`).** The
tracker snaps down to the first sample below it and creeps *up* at 2% of the gap
per sense, so a seed above the real floor costs one sample and a seed below it
costs tens. `csmaNoiseFloorReset` slams the estimate back to
`CSMA_NOISE_FLOOR_DBM` (−105 dBm), which on a bench resting in the mid −90s reads
busy on every sense until it has climbed past −99 — about 25 senses, a few
hundred ms, during which the contention window accrues nothing and a transmit
that was ready waits for a medium that was free the whole time. A node
detouring once a second used to pay that twice a second, because both retune legs
reseeded. Each channel's floor is now parked on the way out and restored on the
way in; the reset constant is only ever an initial value, and the cold-start
paths (a config apply, an AGC recalibration) clear the whole per-channel table
because the front end that measured them is gone.

**Channel access runs under our own waits, never under a reservation
(`csmaPrime`).** A packet held back by our own timing — the absence ladder's
pause between requests, `DETOUR_WAIT` — reserves nothing on the air, so DIFS and
the contention window are served *during* the wait rather than after it: the
machine advances to one step short of the grant, keeps sensing, and the first
`csmaClear` after the wait lifts takes it. A busy medium still restarts the DIFS
and evaporates the withheld grant, so freshness needs no timer of its own. This
is why the classifier distinguishes `SUPE_V_WAIT` from `SUPE_V_HOLD`: the latter
is somebody else's GRANT reserving the medium, and contending underneath it is
exactly what the reservation exists to prevent.

**Observability.** `lora <n>` prints the regime, slot/DIFS times and, under
`appc`, the live airtime percentage with its band and window range, plus the
tracked noise floor for the channel in force and the parked value for each
agile channel — the two numbers every wait is decided against, and previously
the only ones nobody could see; the same
two figures publish as `lora.<n>.stats.{airtime_pct,cw_band}` on the ordinary
event-driven telemetry flush. The contention stall warning names the band and
airtime instead of the exponential regime's `cw` when `appc` is on.

## 6b. Asking the modem whether it is receiving

Carrier sense answers "is there power on this channel", which is not the question
a half-duplex radio needs answered before transmitting. LoRa demodulates *below*
the noise floor — SF7 works at −7.5 dB SNR — so a frame being received perfectly
may never rise above `floor + CSMA_RSSI_MARGIN_DB`, and the sense is a point
sample once a slot rather than a continuous watch, so even a strong frame can
fall between two of them. Transmitting over it destroys both frames.

The demodulator holds the evidence the sense lacks: it has locked onto a
preamble, or validated a header. `radioRxInProgress` reads it for one register
access. This is the **prospective** half of what `csmaMediumHeld` corrects after
the fact — together they mean a neighbour's frame is neither transmitted over nor
credited to us as free medium.

**Latching the evidence costs nothing.** `radioStartRx` — the single place any
path re-enters RX — arms the receiver with `LORA_RX_IRQ_FLAGS`: RadioLib's
default set plus `PREAMBLE_DETECTED`. That goes into the chip's IRQ *register*
and not the DIO mask, so DIO1 keeps its one meaning (a frame completed), no extra
interrupt fires, and an idle radio still holds no wake. `HEADER_VALID` is already
in RadioLib's default set. SX127x has no preamble-detect IRQ to latch and reports
nothing rather than guessing; there, carrier sense and `csmaMediumHeld` stand
alone.

**Both bits latch, so both need a deadline.** A preamble that no packet followed
would otherwise read as a reception forever and block transmit permanently. A
preamble stands until the header it announces should have arrived — `2 × (preamble
+ 8)` symbol times, floored at 20 ms — and a validated header until the longest
frame this modem could still be receiving would have finished (a full 254-byte
frame's airtime + 50 ms). Past that the bit is stale, and *clearing* it is what
lets the next real one be believed. A `HEADER_ERR` ends the reception on the spot.
The two deadlines are computed in `radioStart` from the live modem parameters
(`rxPreambleTicks`, `rxPacketTicks`) beside the transmit watchdog they resemble.

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
- `retain_announces` from `s.lora.<n>.retain_announces` (default 1) — an
  announce heard here is worth *keeping*, not merely forwarding: this node is
  the sole custodian of the mesh on the other side of the radio, re-acquiring a
  neighbour costs ~1.5 s of airtime, and a path response is a signed announce
  that only a node still holding the original bytes can emit (see
  `rns/INTERNALS.md` §1.1.2);
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
defaults under a `s.lora.version` gate (`LORA_VERSION = 6`) for radios **1..**
only — radio 0's defaults come from this straddle's `settings:` block in
`straddle.yaml`, **except** `s.lora.0.bandwidth`, seeded here because its pane
row binds the kHz display key rather than the Hz config key.

The same gate carries three renames, each of which was one setting under two
names: `adaptive_txpwr` → `SUPE.adaptive_txpower`, `afa` → `SUPE.afa`, and
`announce_interval` → `SUPE.announce_interval`. Each moves at its existing
value rather than silently changing a node's behaviour, and the old key is
deleted. Frequency and TX power carry no default
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
  - rx: `r|<rssi>|<snr>|<dur_ms>|<bytes>|<type>|<ch>`
  - tx: `t|<txp>|<dur_ms>|<bytes>|<type>|<wait_ms>|<ch>|<own_ms>`
  `<ch>` is the channel the frame flew on, taken from `LoraRadio.chNow`, `0`
  being the hailing channel; a SUPE detour under a regime with a channel plan
  (§18, §19) is what puts anything else there.
  `<ms>` is the frame's start on the monotonic `millis()` clock, `<dur_ms>` its
  computed time-on-air — computed from the framing the frame *actually* flew
  with, via `LoraRadio.airPreamble` / `airBwHz` / `airImplicit`, which track the
  radio-check sweep regime and a SUPE detour's step rather than the configured
  values (see §16), and `<type>` its protocol — `0` Reticulum,
  `1` this straddle's own air protocol — **SUPE**, Spectrum Utilization and
  Performance Enhancements (`plans/SUPE.md`), which covers its own frames (§19)
  as well as the 0x04 power request, and
  is the name both viewers' legends give it —
  `2` a packet the attached RNode client originated (§17). The viewers colour
  them yellow, red and orange. RX is classified by whether the own-protocol branch consumed
  the frame (so the tap runs before the record is written); TX by
  `LoraRadio.txType[]`, which is per-frame because a power request (§15.1) and
  the RNS packet it prefixes share one burst but not one protocol. The record's
  byte count is payload bytes: a tx record strips the 1-byte seq/split header for
  every type **except** `1`, which carries none — RNode-origin packets fly
  through the same framing rnsd's do — while an rx record strips one
  unconditionally, so an inbound type-`1` frame reads a byte short of the air.
  The per-frame trace is emitted here too, at **`log lora verbose`** rather than
  debug: debug is the decision trace and verbose is the frame trace (§19.8).
  (It replaced the RNS-header trace; `loraTracePacket` is kept but unwired.)
- **`<wait_ms>` is queue latency, not airtime** — the wall time between the
  frame reaching the head of the outbound queue and its first bit going on air.
  The clock starts on the first task pass where there is something to send and
  we can't (`drainOneOutbound`), *before* the early returns, so it counts the
  radio being held by an announce replay and a split still reassembling,
  not only DIFS/backoff against a busy channel. It belongs to the **first frame
  of a burst** and is zero for the rest — the frames behind it followed
  immediately and waited for nothing. Frames that bypass the outbound queue
  report the wait of the channel access they *do* run (`csmaWaitMs`, read on the
  pass `csmaClear` grants the medium): carrier-sense time for an announce replay (§14). With LBT off every figure of that kind is zero. Both viewers
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
  The flag is cached (`s_monWatched`, read as `loraMonOpen()`) and updated by a
  storage subscription on the watch keys the moment a viewer opens or closes —
  the callback wakes both tasks (`loraNudge` + an `IFM_KICK` on the record
  queue), because each may be blocked on the long cadence the old state
  allowed. On the falling edge the interface task drops each radio's whole
  `lora.<n>.packets` subtree — so there is no pre-open history; the graph
  fills from open forward.
- **The viewer's clock is the device's uptime, and uptime restarts.** Records are
  keyed by `millis()`, and the browser panel anchors "now" to the newest record
  and extrapolates from `Date.now()` between updates. That anchor is monotonic
  *within a boot* — a deliberate choice, since pulling it backward on transport
  jitter jerks the bars — so a device reboot under a mounted window leaves it
  marching forward while every new record arrives tens of thousands of seconds
  "in the past", left of every window. Both series then stop drawing while the
  timeline keeps gliding, which looks exactly like a dead device and is not:
  `show lora.<n>.packets` is full. `restarted()`/`reanchor()` treat a record more
  than the one-hour window behind the extrapolation as a new boot and follow it
  back, because the firmware expires its own nodes at an hour and can never
  publish one that old. The RSSI beat checks it too, since on a quiet channel it
  is the only thing publishing.
- **Byte counts differ by one between directions.** A transmit record exempts
  our own air protocol from the split-header subtraction (`doneType ==
  LORA_PKT_OURS ? 0 : 1`); the receive path does not, and hands the record the
  same `payloadLen` it computes for Reticulum framing, which SUPE frames do not
  carry. So a 10-byte GRANT reads 10 on the sender's graph and 9 on the
  receiver's. Cosmetic for the drawing, load-bearing when checking a frame
  against §3 of `plans/SUPE.md`.
- **A second series, live-only: the channel noise floor.** While a viewer is
  open, once a second each radio publishes its channel-RSSI sample set to
  `lora.<n>.rssi` (§18.3), which the viewers draw as a light backdrop under
  the traffic on every channel's graph. The sampling itself is gated on the
  viewer too — see §18.3. Unlike the packet nodes there is no history to
  mirror — only the newest sample is published — so the series starts when the
  window opens and a skipped beat simply reads as a gap.
- **The interface task idles at the rollup cadence, not at 1 Hz.** Its 1 Hz
  maintenance beat (expiry, stats flush, airtime publication) runs while a
  viewer is open or a UI can pull stats (`uiTelemetryWanted`). Dark — no
  viewer, no WiFi, no LCD — its only standing duty is `Rolling1h::shiftAll`,
  one bucket per `kBucketMinutes`, so it blocks on the record queue for
  minutes at a time and a battery node stops paying a per-second wake for
  bookkeeping nothing reads.
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
means nothing on another's. With a frequency-agility regime in force the browser
stacks **one quarter-height graph per agile channel** under that main graph on
the same time axis — see §18.6.

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
- **Claim rows, and what folds them in.** A row can exist on four bytes alone:
  `node4`, the first-4 of a hash asserted by a frame that never carried the hash
  itself. A SUPE announcement is exactly that case — it carries four bytes of
  each identity plus the radio's capabilities, and arrives whenever that node's
  beat says so, which may be long before any Reticulum announce of its own. It
  files against a claim row rather than being discarded, so the capabilities
  survive the gap; the row holds no destination, so nothing routes to it and the
  claim can only ever answer for SUPE. `observeAnnounce` reconciles it on the
  first verified announce, by destination *and* by identity — the identity being
  the key a SUPE claim is filed under — and `peersMergeInto` folds the two.
  Claims never cross the us/them boundary: an unauthenticated assertion must not
  reach our own row.
- **Two lookups, deliberately different.** `peersFindBy4` answers "which node is
  this next hop", searching `node4`, destinations and link identifiers — the
  three things a packet is ever addressed to. `tagNode` (in `lora_supe`) answers
  that *and* "which node does this sender identity name", so it searches stored
  identities too. No packet is ever addressed to an identity, so widening
  `peersFindBy4` to match them would only ever fire on a four-byte collision;
  the asymmetry is the point.
- **Links.** An LR at hops 0 yields `link_id = H([flags&0x0F] ‖ raw[2:])[:16]`
  with LR data trimmed to the 64 ephemeral-key bytes (MTU signalling excluded),
  mapped to its dest; the LRPROOF (context 0xFF, dest = link_id) marks it
  established and — at hops 0 — attributes its signal to the dest, which is
  thereby proven a direct neighbour. A link dialled *to* us records no hash for
  its initiator, so `apNextHop4` cannot name the far end from the link's
  destination — that destination is ours. It hands back the link identifier
  instead, which resolves once a transaction has named who dialled and filed the
  identifier on that node's row (`peersAddLink4`). Without that, our own traffic
  on such a link carries `LORAQ_PEER_NONE` into the queue, and everything keyed
  on the queued peer — the per-peer cap, the power controller, and the reverse
  leg's scan — silently finds nothing. Mid-link traffic on an unseen link_id
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
  lookup — naming node `0` would be aiming the radio at this device. The
  listing header reads `… and us`, `… and rnode`, or `… and us + rnode`.

### 13.1 What `lora n` prints

```
lora/0 neighbors: 2 others and us, 0 open links (observing 17m)

  us   6b87eb8bdbcd51dee010c5a20fd65ef9 rnstransport.probe
       4e0521019085fd7dc7f9fb53e8c8d1a7 lxmf.delivery  "xiao"

  1    d10d5106bcaa65df4a8c50a56d8f05f7 rnstransport.probe
       6793e13ec79d1c1b1372885105aa5cf7 rnsh
       04e893bce336c889329b89fd61a66ac5 lxmf.delivery  "Rop"
       ( TRANSPORT, SUPE, TX -9 )

  2    71cdbfd09e0ea8f0ab17dd06cd0c6e3f rnstransport.probe
       b9351473........................ (not seen yet)
       ( ROAMING, SUPE )
```

One numbered block per node — `us` first, then `1`, `2`, … — and **one line per
hash**: full hash, aspect label, then the announced display name in quotes where
the announce carried one. `neiParseName` extracts that name locally (LXMF's
msgpack, optionally behind a 32-byte ratchet, and NomadNet's raw UTF-8);
iface-lora talks only to rnsd, so it cannot borrow lxmf/'s fuller parser. The
transport hash leads each block, being the one hash every node has. A hash
linked to a node but never heard directly prints as
`<first-4>........ (not seen yet)`.

A capability line closes each non-`us` block:

| tag | means |
|---|---|
| `TRANSPORT` | it relayed someone else's frame to us — a rebroadcast announce naming itself as `transport_id`, or any HEADER_2 frame at hops > 0 that does |
| `ROAMING` | its node-flags bit (a moving node wants more margin) |
| `SUPE` (`RF_PROTO_NAME`) | it has spoken our air protocol to us — a SUPE announcement, or a 0x04 power request |
| `EST <dBm>` | the power this node needs toward that peer, *inferred* by reciprocity from frames we overheard, crediting the peer with `s.lora.assumed_peer_txp` (default 22). It is the only passive source there is now that the power sweep is gone (§14); SUPE's own path-loss pairs replace it for anything it actually detours with (§19.7). |
| `USE <dBm>` | the determination frames to this node actually go out at under `SUPE.adaptive_txpower`, `~` when it came from `EST` plus a margin rather than from a measurement (§15) |

Identities are the **join, not the display**: they build the rows but appear
only under `-v`, which also adds the signal envelope, link quality, the
last-hour rollup and the link_id section.

Node numbers come from `neiWalk`, which the printer and the CLI's node
resolver share, so the numbers on screen are always the ones the resolver
accepts. The resolver takes a node number, a hex hash or prefix, or any unique
substring of an announced name; a 4+ byte hex hash that matches nothing is
still accepted, so an off-table node can be probed. Both verbs abbreviate —
`lora n` … `lora neighbours`, `lora a` … `lora announce`.


## 14. Announce replay (`lora a[nnounce]`)

**Announces are neither buffered against, batched, nor swallowed.** An announce
this node originates goes on the air when `rnsd` hands it over, like any other
packet. What `annRecordTx` does at the `beginTx` tap is keep a *copy* — keyed by
destination hash, replaced when that destination announces again, dropped after
an hour, capped at `ANN_MAX_ENTRIES` (16).

`lora [<n>] a[nnounce]` repeats that buffer: `annReplayStart` arms a run and
`annReplayFill` feeds one buffered announce per drain pass into the packet
queue, where it pays ordinary channel access like any other packet
(`LORAQ_F_REPLAY` keeps `beginTx` from re-recording it). The run closes with
this node's own SUPE announcement. The replay owns nothing: it is queue traffic
end to end, so there is no radio standoff and nothing for a SUPE transaction or
a manual transmit to wait on — the lateral-gate deadlock class this used to
carry is unreachable rather than avoided.

**Two rules hold here, and both are load-bearing:**

- **An announce is the daemon's decision and its timing is part of what it
  decided.** Nothing intercepts one, so nothing can delay somebody else's
  routing decision by holding it — `plans/SUPE.md` §9.
- **Nothing is transmitted in order to measure.** A node's transmit power
  toward a peer comes from traffic that was going to happen anyway: every frame
  a detour sends states the power it went out at, so ordinary exchanges yield a
  *path loss* rather than a bare reading, at two configurations, continuously
  (§19.7, and `plans/SUPE.md` §7's "No power sweep follows it").

The second is why `lora n` has no measured `TX <dBm>` column: the only passive
source outside a detour is `EST` (reciprocity, §13.1), and SUPE's own path-loss
pairs replace it for any peer it actually detours with.

## 15. Adaptive TX power (`s.lora.<n>.SUPE.adaptive_txpower`)

The §15 controller of `plans/SUPE.md`, in `lora_power.cpp`, with its per-peer
state on the peer table's rows. On by default; a node with the key off
transmits everything at `tx_power`, because obeying someone else's power
suggestion puts our transmitter under their control and the opt-out must be
real.

**Power is derived from a learned offset, never computed as an absolute:**

    power = clamp( maximum − offset , floor , maximum )

`apOpenPower` is that expression. The offset starts at zero — a peer the
controller has never adapted to is opened at the configured maximum — and only
ever moves on evidence about that peer. A path loss and a modelled sensitivity
never set the power directly: that arithmetic produces plausible-looking
nonsense (−62 dBm "needed" power on a strong link) that only a floor constant
would be saving, and a floor doing that much work is the design failing.

- **Failure raises the power fast.** `supeApFailed` — no reverse MANIFEST, or
  a delivery-proof miss on plain traffic — cuts the offset by 6 dB and records
  the failed power plus 3 dB as a floor, on a decay (`AP_FLOOR_DECAY_MS`), so
  the loop settles above the cliff instead of oscillating across it.
- **Success lowers it slowly, and only on evidence.** `supeApSucceeded` needs
  `AP_MIN_SAMPLES` clean exchanges since the last change, and the glue only
  calls it when the peer's MANIFEST reported real headroom (margin above the
  target plus slack) — thin margin holds. A controller that dials down on a
  timer walks a quiet link into the ground.
- **The reverse MANIFEST is the evidence loop.** Every detour ends with the
  peer's MANIFEST stating the level our train landed at, so the controller gets
  a complete round trip with a measurement in it on every transaction, at no
  cost. Delivery proofs on plain traffic (`peersQuality`) feed the same two
  functions, slower.
- **SUPE_GRANT is never adapted and SUPE_START always is.** The GRANT is the
  frame every third party holds traffic on, so its reach is the reach of the
  hint; the START is addressed to one node, and §11's absence ladder is the
  power probe run to conclusion — each retry at more power, the third at
  maximum.

**What remains of the passive path.** For a peer that has never detoured with
us there is still the reciprocity estimate (`apSettle`, §13.1): the assumed
peer power minus the headroom its frames arrive with here, plus
`AP_EST_MARGIN_DB` because the assumption may be wrong and because ambient
noise is not reciprocal even where path loss is. It needs `AP_MIN_SAMPLES`
recent frames before it settles anything, settles once per node (through the
identity clustering of §13), and is outranked the moment a filed path-loss
pair exists. The 0x04 power request an LR can carry (§13) is unchanged.

Never on a broadcast: an announce has no single next hop and must reach
everyone, so it always goes out at the configured `tx_power`.

## 16. Pitfalls

- **A record's `wait` is not one clock.** Frames that go out through the drain
  report `wait` (contention) and `own` (everything else since a source first had
  bytes) — the split at the end of `drainOneOutbound`. Frames the SUPE engine
  launches report `wait = csmaGrantWaitMs` and `own = 0`, because `hTxFrame`
  zeroes both marks and the launch restates only the contention part. So the
  interval between rnsd handing a packet over and this interface noticing it is
  invisible on a START's record, and reading its absence as "the packet did not
  exist yet" is wrong. It is also why an armed offer that blocks ingress looks
  exactly like a daemon that has not produced the packet.
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
- **Airtime depends on framing, not just SF/BW/CR.** A headerless frame drops
  20 bits from the payload term and the radio-check sweep also runs a
  6-symbol preamble instead of the configured 12, so
  `loraAirtimeSeconds(..., implicitHeader)` needs both told to it. Computing a
  short headerless frame as explicit/preamble-12 over-stated it by ~11 ms at SF7
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

## 18. Channels and frequency agility (`s.lora.<n>.SUPE.afa`)

Design and the regulatory basis: **`plans/afa.md`** (the channel raster and the
mode ladder) and **`plans/psa.md`** (what must happen before keying up).

Everything in this section is **instrumentation**: channel indices, measurement
and display. What actually transmits off channel 0 is SUPE (§19), and only with
`s.lora.<n>.SUPE.enable` set — with it off, nothing here leaves the hailing
channel.

The order was deliberate. Records, measurements and both viewers were written
against a channel index from the start, so when something did start transmitting
elsewhere there was no retrofit and no format change — a second channel simply
began appearing in data whose shape already had room for it.

### 18.1 Channel 0 is the hailing channel

`LORA_CH_HAIL` (0) is the channel a node camps on: `s.lora.<n>.frequency` and
`s.lora.<n>.bandwidth`, whatever the operator set them to. It is the only
channel that exists until agility is switched on, it is where every frame in
this straddle is transmitted and received unless a SUPE detour is in flight
(§19), and it is what every RSSI reading is referenced to. `LORA_CH_MAX` (10)
bounds the index space at the hailing channel plus the largest regime's agile
set.

**It is flagged never-leave**: no regime may direct a detour onto it, and no
airtime budget of SUPE's is imposed on it. That channel belongs to the Reticulum
network being joined, whose own rules govern it — here, LBT and the APPC
contention band (§6a).

### 18.2 A regime is a numbered statement of what is permissible

A **regime** names the channels and, per channel, the airtime allowance, the
two transmission-length ceilings and the power limit. The number is the
negotiation currency — two nodes agree by naming it, and each resolves the table
locally — which is why `s.lora.<n>.SUPE.afa` **is** the regime number rather than a
flag. It is also the number SUPE (§19) names, because the regime is the
statement of what is permissible on which channels and a second key would be a
second answer to one question.

**`0` means no agile channels, and it is also SUPE's regime 0** — the same thing
read two ways rather than a contradiction. Regime 0 has no channel plan at all:
its whole ladder is the spreading factors above the hailing one, on the channel
the network already hails on. So resolving `0` to an empty channel set is
correct under both readings, and it is the default, so nothing about a node's
on-air behaviour changes until someone sets the key *and* enables SUPE. Regime 1
is the EU 863-870 MHz plan, nine 500 kHz channels under polite spectrum access at
100 s/h each, 25 mW e.r.p. An unrecognised number resolves to no agile channels,
which is the safe reading of a value this firmware cannot understand.

The allowance is a **seconds-per-window pair**, not a percentage, because that
is what makes the table portable across regulators: EU polite spectrum access is
100 s per 3600 s, an EU duty cycle 360 s per 3600 s, US frequency-hopping dwell
0.4 s per 20 s. One field pair, three regulatory shapes, and nothing downstream
special-cases any of them. Nothing enforces the figures yet.

Regime 1's table has no row for channel 0: the hailing channel takes the radio's
configured frequency and bandwidth, which is a user choice and not the table's
to fix.

### 18.3 The per-second channel-RSSI beat

`rssiSamplePoll` runs once a second per radio **while a LoRaMon viewer is
open** and publishes one sample set to `lora.<n>.rssi` (§18.5). The series is
live-only decoration for the graphs — nothing in channel access or SUPE reads
it (carrier sense takes its own samples and tracks its own floor, §5) — so
with no viewer the beat is skipped outright and its deadline is not held in
`nextDeadline`: an idle radio task sleeps until a real event instead of waking
per second for an SPI read nobody sees. The gate is the same cached
`loraMonOpen()` flag that gates frame recording (§12); a viewer opening flips
it via the watch-key subscription and nudges the task, and the stale-by-then
deadline samples on that very pass. The hailing channel is read in place — the
radio is already on it and settled, so it costs one SPI transaction and no
retune — and that reading also decides whether the excursion happens at all.

`rssiSweepAgile` measures the regime's agile channels as **one excursion off the
hailing channel and back**: standby → `setFrequency` → `startReceive` →
`getRSSI` per channel, then home. Nine channels is 2–3 ms away, inside the ~4 ms
an 8-symbol SF7/BW125 preamble allows before a frame could be missed. Four
things about it are load-bearing:

- **The excursion is cancelled when the hailing channel is not quiet.** Leaving
  it mid-reception destroys the frame outright, and unlike a preamble there is no
  partial-recovery argument. The reading just taken is the cheapest evidence
  available that something is on air, so energy above the tracked noise floor
  skips this beat's agile channels — they go unreported and the viewers draw the
  gap. It carries carrier sense's blind spot with it: a frame below the floor is
  invisible to it, and closing that needs the preamble-detect and header-valid
  interrupts the receive path does not currently arm.
- **Bandwidth is deliberately not retuned.** Every channel is measured with the
  receiver the hailing channel is configured for, so all the readings share one
  noise reference and are directly comparable — which is what a graph of nine
  channels needs. Measuring each at its own width would make a 500 kHz channel
  read ~6 dB hotter than a 125 kHz one from thermal noise alone. A regulatory
  Clear Channel Assessment is the opposite case and must match the channel's
  occupied bandwidth; that is a different measurement for a different purpose.
- **A settling delay, and a floor test on the result.** `GetRssiInst` asked
  before the receiver is actually running answers 0xFF, which decodes to
  −127.5 dBm and looks exactly like a very quiet channel. Anything at or below
  `LORA_RSSI_INVALID_DBM` is therefore not reported at all, rather than drawn as
  a floor that isn't one.
- **Coming home is unconditional and unchecked.** A failed retune mid-sweep must
  not strand the radio off the hailing channel, which is the one thing this must
  never do.

Carrier sense outranks the beat: it is skipped while a transmit, a split
reassembly, an announce replay (§14), a SUPE transaction (§19) or any channel-access phase is in progress, so
it never competes for the radio.

### 18.4 Per-channel transmit airtime

`Rolling1h` (`rolling.{h,cpp}`) is a one-hour running total in six ten-minute
buckets, with instances linking themselves into one list at construction so a
single `Rolling1h::shiftAll()` ages every total at once. It knows nothing about
what is being summed. Each radio holds one per channel (`txAir[LORA_CH_MAX]`),
credited at `loraMonPush` against `LoraRadio.chNow` — the channel the radio was
actually tuned to — so a SUPE detour's airtime lands on the channel it flew on
and never on the hailing channel's figure. That separation is load-bearing
rather than tidy: the APPC contention band is chosen from this radio's own
hailing airtime, so detour airtime credited there would make the node contend as
though it had spent the shared channel it deliberately did not, and the detour
would silently stop shortening its own future waits.

**`txAir` measures; it does not enforce.** Six ten-minute buckets cannot defend a
fixed window — a node can spend a budget late in one bucket, have it age out, and
spend it again, approaching twice the cap inside a true hour. Regime 1's cap gets
its own finer ring in `lora_airtime`'s `ChanLedger` (§19.6); do not bolt
enforcement onto the telemetry.

Instances must outlive the program: there is no unlink, so they belong in
statics, globals or long-lived structs, never on a stack or in anything freed.

### 18.5 What is published

| Key | Value |
|---|---|
| `lora.<n>.chans` | `"<freqHz>,<bwHz>\|…"`, index = channel, 0 = hailing. One key, not a subtree: a handful of numbers that change only on a config apply, and the viewers want all of it at once to label their graphs. A list of **one** entry means no agility, so a viewer tells the two cases apart by the entry count and needs no separate flag. |
| `lora.<n>.rssi` | `"<ms>\|<ch0 dBm>\|<ch1 dBm>\|…"`, the newest sample set only. The device timestamp is in the **value**, not the key, so a viewer can tell a fresh reading from a repeated one and place it on the same clock the packet nodes use. A skipped beat republishes nothing, so the key is unchanged, no point is appended, and the gap reads as a gap. One key rather than a node per sample: the series is live-only, so there is no backlog to mirror and nothing to expire. |

The field count is the regime's channel count whether or not every channel
answered — an unmeasured channel is an **empty field**, not a missing one, so a
viewer reads a stable set of columns rather than shifted ones.

### 18.6 What the viewer does with it

The browser LoRaMon draws **one graph per agile channel**, stacked under the
hailing channel's at a quarter its height and the same width — so the same time
axis, and a moment is the same column in every one of them. Same bands, same
dBm scale, same window; only the gutter labels are left off, since repeating one
scale ten times is noise. Each carries its frequency/bandwidth label and its own
transmit airtime over the window on screen.

The channel-RSSI series draws as a **very light grey backdrop** under the
traffic — a bar always wins the pixels it lands on, so the floor reads as
background texture rather than as something drawn over. The series accumulates
live from the newest published sample, the same rule the packet records follow:
it starts when the window opens.

Per-channel captions carry transmit airtime and **not** "channel busy". What
another node is doing on a channel we only visit to measure says little, and the
figure would invite being read as occupancy when it is one instant sampled per
second. The hailing channel's caption keeps both, and its live-hour figures still
come from the firmware's published rollup.

## 19. SUPE (`s.lora.<n>.SUPE.*`)

Protocol: **`plans/SUPE.md`**, authoritative for anything on the air. This
section is what the code does and where it lives.

**SUPE moves unicast traffic off the shared channel onto short private
high-rate detours**, arranged in two short frames on the shared channel, with
rnsd unmodified and unaware. The wire sequence, in full:

```
main channel (the hailing channel — where everyone camps)

  A→*  SUPE_ANNOUNCE2  5+4n B   who I am, what my radio does, at what power
                               └─ once per SUPE.announce_interval, jittered

  A→*  SUPE_START         7 B   "traffic for whoever holds this tag, this
                               much of it (a byte load), and this is what my
                               radio can do (family, ceiling)"
                               └─ carrier-sensed like any other transmission
  B→*  SUPE_GRANT        10 B   "meet me on that channel at that budget, for
                               this long" — or budget 15: refused, with the
                               reason in the channel nibble
                               └─ a turnaround response: no carrier sense
                               └─ carries the REVERSE bit (power byte's free
                                  top bit): B has traffic queued back, so a
                                  reverse MANIFEST will exist at all
                               └─ everyone else holds traffic for the tag for
                                  the stated duration (released one DIFS early,
                                  so the polite node is not last in the draw)

        both retune, observing the 1 ms retune gap

unicast channel (the channel the GRANT named; regime 0: the hailing
                 frequency at a faster modulation)

  A→B  MANIFEST          11 B   this train's power, how the GRANT was heard,
                               capabilities, count, length, the START's hash
  A→B  the packets    × count   ordinary Reticulum frames, ordinary framing

  reverse bit set:
  B→A  MANIFEST          11 B   B's own count and length, how A's train landed
  B→A  the packets    × count
  reverse bit clear:
       nothing — B goes home on A's count/length, A on its own last TxDone

        both return to the main channel
```

One detour carries traffic in **both** directions, ends by arithmetic, and is
not acknowledged. **Nothing is waited for anywhere when things go right**:
ping-pong traffic has its reverse leg (the previous packet's proof, the next
request) buffered before the transaction starts, so the answer is a train or
nothing, decided in the GRANT — never a timed grace, and no close frame. The
only per-frame delays are receiver-turnaround cover (1 ms retune gap, 3 ms
manifest lead, 2 ms between packets), counted into every stated length.
Deadlines exist for misses alone; a packet lost is lost, and the layers above
retry. A count-0 MANIFEST retains one meaning — "nothing after all" — for the
corner where declared reverse traffic vanished (a radio cycle) underneath the
transaction. Silence after an undeclared train is the normal end, so the power
controller reads a missing reverse MANIFEST as loss only when one was
declared. The peer chooses the channel and the budget because it is
the node that knows: it has been camped on the main channel, holds its own
airtime ledger and reuse gaps, and resolves the family-filtered ladder for the
channel it names in the same byte.

### 19.1 What gates it

| Gate | Meaning |
|---|---|
| `s.lora.<n>.SUPE.enable` | off by default. Off means the interface's on-air behaviour is exactly what it was |
| `s.lora.<n>.SUPE.afa` | the regime number (§18). The regime IS the statement of what is permissible on which channels, so SUPE names the interface's own frequency-agility key |
| no access code | IFAC masks the frame from the flags byte on, so the modem cannot read an address and has nothing to match; `radioStart` says so once |

Each regime version expires fourteen days after the build (`supeExpired`); past
it the node neither sends nor accepts frames naming it and says so once.

### 19.2 Where it lives

Three layers, one direction of dependency:

- **`supe.{h,cpp}` — the pure core.** Regime tables, the §14.3 ladder
  (integer-only, family-filtered, channel-bound; `supeLadder` /
  `supeResolveBudget`), the codec for START/GRANT/ANNOUNCE2/MANIFEST, the load
  quantisation (`ceil(Σ(bytes+16)/32)`), every §14.7 deadline, sync words,
  expiry. Conformance for the ladder is `test/supe-ladder-vectors.txt`,
  generated over the full §14.3.4 cross-product by `supe_core_test`; the file
  is the authority when it and a reading of the prose disagree.
- **`supe_engine.{h,cpp}` — the one decider.** The whole state machine, both
  roles, single-threaded by contract with no lock and no blocking anywhere: a
  step that must happen later is `host->schedule`d and the entry returns.
  Everything platform arrives through `SupeHost` (time, randomness, one-shot
  timer, SHA, tune/tx/rx, peer views in, peer notes out, the channel view);
  the packet queue it reads is `lora_queue`, pure itself. The engine also owns
  the tag set ("addresses that mean us", fed by the observer), the third-party
  holds, the proof-return table, and the recent-STARTs cache that lets a GRANT
  be correlated with the START it answers. `shouldDetour` is the one
  deliberately-open policy function (`plans/simulation.md` §7); v0 says NOW
  whenever there is a peer.
- **`lora_supe.cpp` — the boundary.** The recursive mutex every entry point
  takes (radio task, esp_timer task, console, config callbacks — the engine
  itself never locks), the `SupeHost` implementation over
  radio/queue/peers/airtime/chanplan/power, the ANNOUNCE2 beat and its
  peer-table ingest, and the note handlers that file the engine's events into
  `Neighbor` rows (pairs, strikes, absence, refusal backoffs) and the power
  controller (§15).

Host tests: `make -C esp-idf/test` runs the core checks and regenerates
`golden.txt` + the ladder vectors; `make -C esp-idf/test engine` steps whole
transactions — the bidirectional detour, a refusal, the absence ladder, the
reverse flag's no-frame ending, the shared-link-tag crossfire, the third-party
hold and the no-waiting contract — against a stub host.

### 19.3 Frame dispatch

One assumption, stated once: SUPE types are `0xC0`–`0xDF`, never ending in 0
or 1, disjoint from split framing's reachable bytes and from Reticulum flags
on an interface without an access code. `handleRxDone` sorts byte 0 into
framing / SUPE / discard on that rule alone; any change to receive dispatch
preserves it. Assigned: START `0xC2`, ANNOUNCE2 `0xC4`, GRANT `0xC5`,
MANIFEST `0xC9`. `0xC3` (the old ANNOUNCE1) and `0xC8` (the old HERE) are
burned and never reassigned.

### 19.4 The sender path

The classifier (`supeEngVerdict`) runs on the head of the packet queue before
anything contends for the medium: held tag → HOLD (released one DIFS early);
absent peer → DROP; mid-ladder retry wait → HOLD; refusal backoff → PLAIN (the
peer is present, the traffic flies); not a peer → PLAIN, untouched, exactly as
with the feature off. OFFER arms a jittered launch; `supePoll` wins the
channel through ordinary carrier sense and `supeEngLaunch` emits the START.
The absence ladder (§11) is three requests — power up, ceiling down, the third
at maximum and budget 0 — then the peer is absent for a minute and its traffic
drops; one request per minute thereafter; any evidence of life cancels the
record outright (`SUPE_EV_ALIVE`).

**Silence is established from the receiver, not from arithmetic.** The GRANT
deadline is two stages. The first is `turnaround + guard` with no time on air in
it — the instant the answer must have *begun* — and asks the host's `rx_busy`
(`radioRxInProgress`, §6b) what the modem is doing. A frame arriving is the
answer being delivered, so the second stage waits it out; nothing arriving is
silence, reached half a frame earlier and without trusting an estimate of a frame
that was never sent. Silence there retransmits the START once, byte for byte from
`x->startFrame` — a rebuilt frame would carry a different load byte and therefore
a different hash, orphaning the GRANT that names it — inside the same request,
with no strike and no rung. The platform drives that retransmission
(`supeEngResendDue`/`Resend`) rather than the engine firing it, so it pays the
same channel access the original did, which is also its decorrelation; and when
the peer's GRANT does start late, carrier sense sees it and the retransmission
defers instead of transmitting over the reply.

**The requester is named by the START, or not at all.** On the answering side the
tag names one of *our own* addresses, so it says nothing about who is asking:
`sender_ident` is the only handle. It is what `bAnswerStart` resolves into
`x->fromPeer`, and everything that needs to know who the far end is hangs off it
— the reverse leg's scan and staging, and filing a link identifier the cargo
creates against the node that dialled (§10 of `plans/SUPE.md`). Without it
`scanReverse` is handed `LORAQ_PEER_NONE` and returns 0, so the GRANT's reverse
flag is never set and the mechanism is inert. The host tests hid exactly that for
a long time, because their stub `peer_get` resolved *any* tag including the
answerer's own; `testReverseNeedsIdent` is the guard against it reopening.

### 19.5 What is learned

Every measurement is a path-loss pair — a level read here against the power
the other side stated in the frame itself — filed through `SUPE_EV_PAIR` into
the peer table (hailing pair and detour pair separately), or against the link
for the one peer that can never be named (it dialled us; a link-id tag is the
shared handle). The reverse MANIFEST both proves the train landed and states
the level it landed at, which is §15's whole evidence loop. Refusals carry a
reason mapped to a backoff (busy 300 ms, no quiet channel 2 s, out of airtime
5 min, wrong regime / ceiling 1 h) — knowing *how long* not to ask is what a
refusal buys over silence.

### 19.6 Airtime

Detour airtime is accounted separately from the hailing channel's duty figure,
per channel, in `lora_airtime`'s `ChanLedger` (360 × 10 s buckets): credited
at transmit-done *and on the abort path*, recomputed once per bucket into a
per-channel verdict the transmit path reads without arithmetic. The answering
node consults it (and the 100 ms reuse gaps) before granting; hailing-channel
frames feed the APPC contention band instead — two budgets, never one
(SUPE.md §18: credit a detour against the hailing figure and the whole
stays-cheap effect vanishes silently).

**The verdict beat parks when it can't change a verdict.** With no cap to
enforce (regime 0) or an empty agile window, a recompute can only restate
"in budget", so `airtimeRecompute` drops `beatOn` and the beat holds no wake
(`airtimeNextDeadlineMs` → `UINT32_MAX`); the first agile transmit
(`airtimeRecord`, channel ≥ 1) re-arms it. Hailing-only traffic never does —
channel 0 carries no SUPE budget. The dialect-expiry re-check used to ride
this beat and now rides `supePoll` passes directly, at most hourly, holding
no wake of its own. Net: SUPE enabled on an idle node costs the announce beat
and nothing else; the engine's `esp_timer` is armed only inside a
transaction, so with zero packets queued it never fires.

## 20. Known gaps in the LCD viewer

The browser LoRaMon carries the current feature set; the LCD app lags it in two
places. Both are deliberate deferrals rather than oversights, recorded here so
the intent survives.

**Agile channels want a different shape on a small screen.** The LCD currently
just displays the other channels the way the browser does — a stack of
quarter-height graphs on the same dBm bands. On a screen a third the width that
spends its vertical budget on a power dimension nothing much varies in: the
agile channels are visited once a second to be measured, so what they actually
convey is *occupancy over time*, not level. The intended shape is **plain lines
per channel with the power dimension dropped**, leaving the hailing channel as
the only graph that keeps the dBm axis. Not built.

**The two wait marks are browser-only.** `own_ms` (§12) is drawn dotted beside
the solid contention run in the browser; the LCD reads the older six-field form
and draws contention alone. Harmless — the field is appended, so an indexing
parser ignores it — but the LCD therefore cannot distinguish a busy channel from
a busy radio.
