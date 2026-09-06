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
  `lora_power` adaptive transmit power; `lora_fem` the external front-end module
  between chip and antenna; `lora_mon` telemetry; `lora_rnode` the
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
  `s.lora.<n>.ifac_netkey` secret and handing them to `rnsd` in the
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
- **Adaptive TX power** (§15) — a per-peer, per-configuration derivation from
  a measured path loss (`need = loss + sensitivity(cfg) + margin`), walked down
  one dB at a time on evidence and back up immediately on a miss. Four tiers of
  evidence, of which only the guessed one is a setting.
- **Channels and frequency agility** (§18) — a channel index on every record and
  every measurement, the numbered regime table that names a channel set, and the
  per-second channel-RSSI beat that measures it. Instrumentation: what actually
  transmits off the hailing channel is SUPE.
- **SUPE** (§19) — unicast traffic becomes meetings: one HAIL on the shared
  channel says what is waiting, the party it names answers — in place under
  regime 0, at two derived slots on a private channel under a plan — and the
  frames follow at the confirmed budget; both directions ride one meeting, and
  under a plan every goodbye seeds the next. Off by default. The arithmetic — regime tables, the family-filtered ladder, the
  codec, every deadline — is `supe.{h,cpp}`; the state machine is
  `supe_engine.{h,cpp}`, single-threaded and lock-free behind the `SupeHost`
  interface `lora_supe.cpp` implements; all three are host-tested in
  `esp-idf/test/` (`supe_core_test`, `supe_engine_test`, and the
  `supe-ladder-vectors.txt` conformance file).
- **The RNode endpoint** (§17) — a stock RNS `RNodeInterface` client attaches
  over USB serial, TCP 7633 or Bluetooth (the last through
  `reticulous/rnode-ble`) and becomes the third endpoint of the radio segment,
  executing radio commands by writing the ordinary `s.lora.<n>.*` keys. Its
  public door contract is `include/rnode_door.h`.
- **The browser panel and generated LCD/web settings** (`browser/`, the
  `settings:` block in `straddle.yaml`).

## 1a. How this straddle logs

Two levels, and the split is not about how much detail you want — it is about
what a line is *for*.

**Debug is one line per meeting**, plus the failures that cost something: a
schedule that spent a whole horizon carrying nothing, a rendezvous abandoned,
a power floor raised, a peer held off. A busy pair meets several times a
second, so a line per step is a scroll nobody reads, and the one line that
says what happened is buried in it. **Verbose is the steps** — each schedule
installed, each slot attended, each frame, each measurement filed.

Anything that went wrong rides the line of the thing it went wrong in. A
failure is a property of a meeting, not a separate event, and splitting the two
is what makes a log unreadable in the first place.

### The notation

Two forms, used everywhere a level or a channel is stated:

| Form | Means |
|---|---|
| `tx{<txpwr> <rssi> <snr>}` | **we** transmitted at `txpwr` dBm, and the far end reported reading it at `rssi` dBm / `snr` dB |
| `rx{<txpwr> <rssi> <snr>}` | **they** transmitted at `txpwr` dBm, and we read it at `rssi` / `snr` |
| `<ch>/<bw kHz>/<sf>` | the configuration it flew at |

A triple is always *sent at, read as* — never one end's view twice. A reading
that never came back is `?`, which is not the same as a zero: one says nobody
reported, the other is a measurement.

### The meeting line

```
Our hail tx{10 -58 12} 3/500/5: tx{10 -50 12} rx{14 -45 10} - sent 5/5, rcvd 2/2
Their hail rx{14 -45 10} 3/500/5: rx{14 -45 10} tx{10 -50 12} - rcvd 2/2, sent 5/5
```

Whoever opened is named first and their leg is printed first, so the order on
the line is the order on the air. `hail` means the meeting answered a HAIL and
the triple after it is that frame; `rndv` means it came
from a goodbye's rendezvous, which cost no frame and has no levels to report.
The `<ch>/<bw>/<sf>` is what the trains actually flew at — the confirmed budget,
not the slot's. Repairs are transmissions beyond the train and are named as such
(`sent 3/3+1`) rather than pushing a count past its own total.

**A leg that carried nothing is absent, not zeroed.** A side that sent no train
gets no triple, and its count is left off the line entirely rather than printed
as `sent 0/0` — the same rule as the `?` above: `0/0` reads as a measurement of
something, and there was nothing to measure. A one-way meeting is therefore one
triple and one count, and the `-` disappears with both.

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
wake is added by the announce beat (`announce_interval`, default 30 min).
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

The RF switch is handled at the call site, not in dispatch, in one of four
forms: `Module::setRfSwitchPins(rx, tx)` for a two-GPIO external switch (set on
the `Module` before `begin()`, so it covers every family); `setDio2AsRfSwitch(true)`
inside `radioBegin` when the slot asks for it (SX126x only); a detected
front-end module's table installed by `femInit` (§4b); or the LR2021's own DIOs
programmed after `begin()` (§4c).

**RX gain.** SX126x/LR `begin()` sets the regulator to **DC-DC** (passing
`useRegulatorLDO = false`). The LNA gain mode is a per-radio setting
`s.lora.<n>.rx_boosted_gain` (default **on**, live via `lora <n> rx_boosted_gain
0|1`), applied by `radioBegin` — boosted buys ~+3 dB sensitivity for ~0.4 mA more
RX current (~4.2 → ~4.6 mA typ.), worth it for a receiver that idles in RX. Two
families take it and they take it differently: SX126x's `setRxBoostedGainMode`
is a **bool**, the LR2021's is a **level 0..7** that is only accepted from
standby. One switch to the operator either way, with "on" meaning the top of the
LR2021's range — the setting exists to buy sensitivity, so a middle rung would be
a number nobody asked for. `begin()` leaves the chip in standby, which is why the
call sits there rather than anywhere later. Inert on the other families.

**Overridden to off behind an external amplifier.** `radioStart` applies the
setting only while `cal.rxGainDb` is zero — i.e. no front end amplifies on
receive, or the one that does has its LNA bypassed (§4b). Behind 17..20 dB of
front-end gain the chip's noise figure enters the system's divided by that
gain, so the boost's 3 dB becomes two or three tenths, bought with the same
half-milliamp and some large-signal headroom. The key keeps its value rather
than being rewritten: a user who bypasses the front end's LNA to save its
8 mA may want the chip's off too, and a setting the firmware had silently
flipped would deny them that. The status line and the setter both say when
the value is being ignored.

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

## 4b. Front-end modules (`lora_fem`)

Some boards put a PA + LNA + antenna switch between the radio and the antenna.
The radio's own dBm range then stops being the antenna's, so **every number
that leaves this interface is referenced to the antenna connector** and the
register-referenced figures live only inside `lora_fem`. Three entry points
carry it: `rfChipDbm` converts a wanted power to a register setting,
`rfAntennaDbm` says what a setting actually radiates, and `rfRssiDbm` refers a
received level back to the connector. `lora.<n>.tx_power_max` and
`.tx_power_min` publish the range so a UI sizes its slider to the hardware
rather than to a build-time constant.

**The model is a curve, not a gain.** Neither half of the error is constant:
the chip's own set-versus-actual output drifts several dB at the extremes, an
amplifier compresses as it saturates, and a board may sit a fixed pad between
the two — the Heltec V4 has 17 dB of it. `LORAn_TX_CAL` states measured
`register:connector` points per part, straight lines between them and flat
outside; a flat gain figure is what an uncharacterised part falls back to.
Every curve carries its own grade — measured, datasheet or none — published as
`lora.<n>.cal`, so an identity conversion is never read as a characterisation.

Two consequences are worth stating because they are easy to get backwards:

- **The ceiling is the curve's maximum, not its last point.** A curve that
  turns over near saturation peaks below the top register setting, so
  `rfChipDbm` returns the *lowest* setting that reaches a request. On the
  Heltec V4 that means full power is register 20, never 22 — 0.5 dB more output
  for 136 mA less.
- **The floor is real hardware, not a formality.** A part that cannot be driven
  below its own amplifier's output has a quietest transmission, around +7 dBm
  on that board against the chip's −9, and +21 dBm on the Meshnology W12.
  `minTxDbm` is what the adaptive controller clamps to and what a node
  announces, so nothing claims a power no setting produces. Where a front end
  has a transmit-bypass path the floor is a driving choice rather than a limit,
  but reaching it costs a front-end mode change on the transmit path and a
  second curve with a discontinuity between them — scoped, unbuilt, in
  [hw-meshnology-w12's INTERNALS](../hw-meshnology-w12/INTERNALS.md).

**What is transmitted is what is announced.** `apApplyPower` records
`rfAntennaDbm` of the setting it programmed, never the request, and `txPwrNow`
is read by the SUPE frame that states the power and by the LoRaMon record
alike. A request the range cannot honour therefore corrects itself everywhere
at once instead of being reported as though it had been met.

**On receive, one conversion, in one place.** `lora_bridge`'s RX-done path and
`channelRssi` are the only two ways a level enters the driver, so both apply
`rfRssiDbm` there and everything downstream — the neighbour tap, the path
losses, the bucket ring, records, the RNode endpoint — reads one quantity.
**SNR is never corrected**: gain in front raises signal and noise together.
Carrier sense needs nothing either, because it compares against a floor tracked
from those same samples and a constant offset cancels out of a relative test;
only `CSMA_NOISE_FLOOR_DBM`, the seed, is an absolute level.

`SUPE_NOISE_FIGURE_DB` stays a bare radio's figure on purpose. Every caller
asks it about the **far** end — what must reach a peer for it to decode — and a
peer's front end is not ours to assume; crediting one with an amplifier it may
not have would under-power the link. Nothing needs our own figure, because no
decision here compares a level of ours against an absolute floor: margin is an
SNR question and carrier sense tracks its own.

Two wirings, told apart by whether the MCU can reach the part at all.

**Detected** — the part is on MCU GPIOs (`CONFIG_LORAn_FEM_PWR_PIN`,
`_FEM_EN_PIN`, `_FEM_TXSEL_A_PIN`, `_FEM_TXSEL_B_PIN`). Two candidates are
supported, because boards ship both across revisions on the same enable net
(Heltec V4: GC1109 on ≤ 4.2, KCT8103L on 4.3): the KCT8103L design pulls the
enable line up and the GC1109 one leaves it floating, so `femInit` powers the
rail, reads the enable pin as an **input**, and picks the part from what it
finds. The rail has to come up first — the pull-up that is the signal is powered
from the FEM side, so the sense reads garbage on a dead rail. Mode switching
then rides on RadioLib's RF-switch table, which is what keeps every
`standby()`/`startReceive()`/`startTransmit()` call site in the driver ignorant
of the front end.

The two-pin row that table applies does **not** drive the same signal on both
designs, which is why the detect decides more than a pin number. On the GC1109
board the MCU owns `CSD` and `CPS` (the PA / transmit-bypass select) while the
TX/RX direction line `CTX` hangs off the radio's own `DIO2`; on the KCT8103L
board the MCU owns `CSD` and `CTX` itself, and `CPS` has no MCU connection.
Both parts amplify on receive — 17 dB and 20 dB — so both carry a receive-gain
correction; the GC1109's low-loss bypass is its *transmit* bypass, not a
receive path.

**The LNA is the front end's standing cost, and only one part lets it go.** A
KCT8103L's LNA draws about 8 mA for as long as the radio listens — more than
the SX1262 does (a Heltec V4.3 idles near 13 mA with it and near 6 without) —
and the part receives without it when `CTX` is held at 1 while the chip is in
RX. `femRxLna` (`s.lora.<n>.fem_rx_lna`, default on, read by `applyConfig`
ahead of `femBandSelect`) swaps the installed RF-switch table for one whose RX
row is `enable high, CTX high`; RadioLib applies it at the next mode
transition, and the `calBuild` inside `femBandSelect` zeroes `rxGainDb` while
the LNA is out, so a level read then is already the connector's. On a GC1109
the second pin is `CPS`, the transmit-path select — its LNA is in line whatever
the MCU does — so `femRxLna` records `on` and touches nothing there, and
`loraPublishRowGates` shows the switch (`lora.<n>.row_fem_lna`) only where a
KCT8103L was sensed. The transmit path is untouched either way: the TX row is
the same in both tables and the power curve does not move.

**Declared** — `CONFIG_LORAn_FEM_GAIN_DB`, non-zero. The control lines hang off
the **radio's** DIOs (§4c), so there is no pin to sense and no table to install
here: nothing identifies the part at runtime, the board states its gain and
`CONFIG_LORA_TX_POWER_MAX` states its ceiling, and that pair is the whole model.
The gain applies flat — nobody has measured this one per rung — with the clamp
to the chip's own range doing the work at the bottom, where asking for less than
the chip's floor plus the gain simply lands on the floor. It wins over the pin
group above; a board states one or the other, never both.

The pair is a **guard** as much as a conversion. A front end whose TX input is
rated for a few dBm sits behind a chip that will deliver +22 to anything that
asks, and the only thing standing between them is the subtraction. Raising
`CONFIG_LORA_TX_POWER_MAX` without raising the gain raises the chip drive by the
same amount.

### 4b.1 Two bands, and what follows the carrier

A dual-band part (LR2021, SX128x) has two RF ports, and a board that uses both
has a separate amplifier on each. They share **nothing** — not the supply gate,
not the gain, not the antenna ceiling, and not even the drive range the chip
itself accepts (the LR2021 takes −9…+22 dBm on its sub-GHz port and −19…+12 on
its 2.4 GHz one, so a figure legal on one is refused outright on the other).
The only thing in common is the number the operator types.

**`femBandSelect(r, highBand)`** is where all of it is settled, from one fact:
whether the carrier is above `LORA_HF_CUTOFF_MHZ` (1500 MHz, RadioLib's own
LR2021 cutoff, spelled in `lora_radio.h` so the FEM and power paths cannot
disagree with the library). It raises that port's supply gate, drops the other's,
records the band on the radio, sets `maxTxDbm` from the band's ceiling, and
republishes `lora.<n>.tx_power_max`.

It runs from **two** places, and both are needed:

- `radioBegin`, before `rfChipDbm` converts the power — the conversion and its
  clamp are the band's, so the band has to be known first. This is also what
  covers `probeRadio` and any other direct caller.
- `radioStart`, as soon as the configured frequency is in hand — because the
  ceiling it publishes is what the operator's `tx_power` is checked and clamped
  against, and that happens before `begin()`. Without it the clamp would be one
  band behind on every crossing.

It is idempotent, which is what lets both call it.

A single-band board names no HF supply pin and no HF gain, so nothing is
switched and only the sub-GHz numbers are ever used. A board with **no** front
end at all still gets a band-correct ceiling: the bare chip's own maximum for
the port in use, which is not the same number on the two ports.

The user-facing consequence is that tuning across 1500 MHz re-clamps `tx_power`
to the new ceiling with a warning and re-sizes every power control bound to
`lora.<n>.tx_power_max` — the web slider reactively, the LCD one when the pane
is next built.

## 4c. LR2021 DIO wiring (`lr2021ApplyDio`)

The LR2021 bonds out DIO5..DIO11 and lets the board decide what each one is: the
interrupt line, an RF-switch output, or nothing. Both facts are the board's
(`CONFIG_LORAn_LR_IRQ_DIO`, `CONFIG_LORAn_LR_RFSW_*`) and both are programmed
around `begin()`, on **every** begin, because `begin()` resets the part and a
reset returns every DIO to "no function".

- **`irqDioNum` goes in before `begin()`**, because `begin()` is what programs
  the IRQ DIO's function. RadioLib assumes DIO5. A board that bonded a different
  one and does not say so gets a radio that initialises cleanly, reports itself
  found, transmits — and never signals a reception, because the interrupt is
  aimed at a pad that is not connected.
- **The RF-switch table goes in after**, as `SetDioFunction` (0x0112) +
  `SetDioRfSwitchConfig` (0x0113) per DIO, sent through
  `Module::SPIwriteStream` — which is exactly what the library does underneath.
  The board's five per-mode masks are transposed into one mask per DIO on the
  way: the chip is configured per DIO, the board describes itself per mode, and
  those are opposite orderings.

`LR2021::setRfSwitchTable` is deliberately **not** used. Its pin array is
`Module::RFSWITCH_MAX_PINS` — **5** — against this chip's seven DIOs, so a board
using six cannot be described to it at all; and RadioLib 7.7.1 indexes the
per-DIO configuration it builds by the caller's array position rather than by the
DIO number, which silently programs a zero mask into every DIO past the fifth.
The index fix is on the library's master branch and in no release. One datasheet
constraint survives either way and is honoured here: **DIO5 accepts only the
pull-up in sleep**, and any other pull makes the chip refuse the command.

**A frequency change must go through a full `begin()`** on this family.
Crossing the part's LF/HF boundary re-points the whole front end, and the
incremental setters answer a live cross-band move with an error rather than a
retune. §9's config lifecycle already stops and restarts the radio on any
change, so this costs nothing — but it is why that path must not be "optimised"
into a `setFrequency` for the LR2021.

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
- **A SUPE meeting drops its own orphan at the close.** Train frames reach the
  same reassembler (`hTrainDeliver` → `bridgeFrameDeliver`), and a meeting hands
  over everything it collected in one delivery, once. A half still pending after
  that batch is waiting for a frame the repair round already failed to recover,
  so it is dropped there rather than left to the timeout — the counter is the
  same. Holding it was not merely idle: a pending split is state the rest of the
  interface has to reason around, and reasoning around it wrongly is what put
  five seconds of deafness after every lossy train.

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
with `rxContinue`: the driver's in-progress markers reset and `gpio_intr_enable`
on the radio's IRQ pin — and **no `startReceive`**.

**The receiver is never restarted after a frame.** Every family is armed in
continuous receive (`radioStartRx`), and after RX_DONE the chip goes on
receiving; in a SUPE train the next preamble is already being demodulated by
the time the task has read this frame out over SPI. A `startReceive` at that
point begins with standby, which aborts that reception, and at the fastest
budgets that is every second frame of the train. So the RX-done paths — a good
frame, a CRC failure, an impossible length — leave the modem alone, and only
the paths where the chip really is in standby re-arm it (`rearmRx`): a
completed or aborted transmit, a retune, the unhandled-IRQ repair. The one
family whose `readData` itself drops to standby, the SX128x, is started again
inside `radioRxResume`; on the SX126x, SX127x, LR11x0 and LR2021 that call is a
no-op. Reading the frame promptly still matters: the chip's receive buffer is
256 bytes and the next frame is written behind or over this one, so the
readout has to be done before the next payload arrives — that, not a restart,
is what `SUPE_TRAIN_GAP_MS` covers.

**A packet that will not be read is discarded from the chip** (`radioRxDiscard`).
The LR2021 is the one family that reads packets out of a FIFO in order rather
than by offset, and starting receive does not empty that FIFO. A packet left
unread there — an impossible length, a frame that completed just before a
retune cleared the flags — is what the next `readData` returns, under the new
packet's length, with the new packet flushed behind it: a phantom frame with a
stale head delivered in place of a real one. So the FIFO is emptied on every
start of receive and on every discarded RX-done; on the other families the
call does nothing. A SUPE-typed frame that reaches the engine and does not
decode is logged at warn with its first bytes, channel and phase
(`supeOnFrame`), because that is what such a phantom looks like from above and
the bytes are the only evidence of where it came from.

**TX — `beginTx` / `startTxFrame` (lora_bridge).** Transmission is
**non-blocking**: `startTransmit()` fires the chip and returns; the TxDone IRQ
wakes the task, which finishes the frame in `serviceRadio` and either sends a
split second frame or re-arms RX. One or two frames are sent depending on
length; `tx_bytes` counts the RNS payload, `tx_frames` counts each LoRa frame.

**At TxDone the receiver comes first.** The far end answers a frame of ours
after the flip (`SUPE_FLIP_MS`, SUPE.md §14.7), and at the fastest budgets its
preamble is two milliseconds; every millisecond the task spends before the chip
is receiving again — retuned to the budget where the engine asks — is one in
which that answer starts unheard. So the TxDone branch takes a snapshot of what
the record needs (the frame bytes, channel, power, start, waits, the meeting's
peer), hands the radio over — the split's second half, or the engine's
`supeAfterTx`, which retunes and re-arms or fires the next train frame — and
only then classifies, records and accounts the frame from the snapshot. The
snapshot is not optional: the engine may restage `txFrame[0]`, move `chNow`,
and close the meeting whose peer the record is tagged with. The flip as this
board actually makes it, end of air to receiver armed, is measured into
`flipMaxMs`/`flipSumMs`/`flipN` and shown by `lora` stats as `flip tx->rx`;
that is the number `SUPE_FLIP_MS` has to cover at the far end. The other
direction is measured too: `loraNoteAnswer`, called as an engine frame leaves,
scores it against the end of the last frame received when it follows within
`LORA_ANSWER_REACH_MS`, and `lora` stats show it as `answer rx->tx`. That is
what `SUPE_TURNAROUND_MS` has to cover at the far end — the flip, then the
one-shot timer, the task wake and a pass of the loop, which at the hailing
configuration has been seen at 40 ms on the air.
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

**An answer is parked for the flip, never for the train gap.** The peer that
just transmitted the frame we are answering is turning from transmit to receive
— TxDone serviced, chip out of standby, receiver started, after a GIMME retuned
to the budget first — and nothing it does can hear a preamble that starts before
the turn is done. At the hailing configuration the preamble is long enough to
hide the turn; at SF5/500 kHz it is two milliseconds and hides nothing, which is
how a GIMME sent five milliseconds after a HAVE, or a train started four
milliseconds after a GIMME, goes unheard and the meeting dies in silence. So
`deferSend` takes the gap as an argument: an answer — `SUPE_PEND_GIMME` after a
HAVE, `SUPE_PEND_ANSWER` after a THATSIT, `SUPE_PEND_HAIL_ANSWER` in regime 0 —
and the train `startTrain` opens after the peer's GIMME wait `SUPE_FLIP_MS`; a
send that follows our **own** frame — the THATSIT after our train, the close
after our repair, the return train after our answering HAVE — waits only
`SUPE_TRAIN_GAP_MS` or `SUPE_TRAIN_LEAD_MS`, because the peer's receiver has been
open throughout. The GIMME is built in `sendGimme` from what `onHave` settled
(`m->budget`, `m->lastRssi`/`lastSnrQ`), not on the spot.

**A train's next frame is fired from tx-done, not from a deadline.** Both chains
now leave by the same door: a split's second half from `serviceRadio`'s TxDone
branch, and a train's next frame from `supeEngOnTxDone` calling `fireNext`
directly (`SUPE_M_TRAIN_TX` / `SUPE_M_REPAIR_TX`) — which reaches the radio
through `txRearmRx` → `supeAfterTx`, the recursive SUPE lock making the nesting
legal. Parking the frame on `SUPE_TRAIN_GAP_MS` instead did **not** cost one
gap: the wait went to the platform's scheduler, so the frame waited out an
`esp_timer` one-shot (floored at 1 ms), the timer task's turn, `loraNudge`
waking the radio task, and a whole pass of its loop before `supePoll` reached
the engine — several times the gap itself, every millisecond of it dead air
inside an appointment the peer is holding open.

The train gap §14.7 asks for is still there; it is **paid rather than
parked**. Our TxDone and the peer's RxDone land at the same instant and both
sides then do the same order of work — service the interrupt, move a frame over
SPI — before the next carrier appears. It is the spacing the two halves
of a split already fly at, where the receiver does identically the same work
between them. `SUPE_TRAIN_GAP_MS` stays in the budgeted train lengths, where
over-estimating is the safe direction, and stays parked for `deferSend`: a send
that follows the *peer's* transmission has the flip genuinely in front of it,
and firing those at zero gap is what once lost THATSITs and closing answers
while whole trains arrived intact.

**Ingress is gated on the meeting, not on the radio.** `drainOneOutbound`
pulls from rnsd and the RNode client (`queueFill`) *above* the `txActive` and
`supeHoldsRadio` returns and *below* the meeting one. The distinction is the
protocol's: what a train carries is declared before it runs — the HAIL's or
HAVE's count and length — so a packet arriving after the build cannot join it and must
not disturb the frames the engine is firing. Until then it can, and the wait
for a slot is up to hundreds of milliseconds, which is where most of the
chances to coalesce live. A packet pulled in during that wait is still in the
queue when the train is built, and rides the very meeting being waited for. Pulling while a frame is on
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
debug carries decisions and verbose carries frames. At debug a SUPE meeting
reads as a short story — HAIL, the answer, trains, THATSIT, home — with no
frame dumps between the lines; at verbose the same story is interleaved with every
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
and evaporates the withheld grant, so freshness needs no timer of its own.
SUPE's `SUPE_V_WAIT` is the caller: a packet waiting for its slot reserves
nothing, so the medium is served underneath the wait.

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
- `community_radius` from `s.lora.<n>.community_radius` (default 3) — an
  announce from within the radius is worth *keeping*, not merely forwarding:
  this node is the custodian of the mesh on the other side of the radio,
  re-acquiring a neighbour costs ~1.5 s of airtime, and a path response is a
  signed announce that only a node still holding the original bytes can emit
  (see `rns/INTERNALS.md` §1.1.2);
- IFAC fields from `s.lora.<n>.ifac_netname` / `ifac_size` and
  `s.lora.<n>.ifac_netkey`.

The ITS connect **ref is the radio index**, so `onRnsdDisconnect(ref)` finds the
radio and clears its handle; the task loop re-registers on the next turn if the
radio is still enabled. If registration fails but the radio is on-air, the state
goes `rnsd_unavailable` and the loop keeps retrying — RF stays up.

**The ack budget has to clear a flash write, not a scheduling hop.** rnsd acks
on its own task, and a storage flush suspends both cores' cache for as long as
the program windows take — measured at 300 ms on an ordinary save and over
800 ms during the config write that follows a flash, which is precisely when
this registration runs. The connect waits 3 s. And `itsClientInit` is sized to
the radios **plus two**: a registration whose ack does not arrive in time is not
a registration that failed to happen — `itsConnect` cancels it and returns -1,
but the CANCEL is a message rnsd processes when it next runs, and rnsd is the
task that was too busy to ack. For that moment the slot is still held by a conn
nobody wants, and a retry with no spare slot is refused ("has 1/1 client conns
already"), leaving the radio unregistered until something else shakes it loose.

rnsd is one of **three** endpoints on a radio segment, not the only one. The
others are the radio itself and — when configured — an attached RNode client
(§17). A packet entering from any one is presented to the other two, and the
rnsd handle is no longer a precondition for outbound work: `drainOneOutbound`
computes availability across both software sources before deciding anything.

## 8. Airtime-derived bitrate

`loraAirtimeSeconds` computes the LoRa time-on-air of a frame per Semtech
AN1200.13 (symbol time `2^SF / BW`, preamble `(n + 4.25)` symbols, payload
rounded into whole symbols, low-data-rate optimisation engaged once a symbol
exceeds 16 ms, explicit header + CRC on). The formula itself is `loraToaSeconds`
in `lora_toa.h` — header-only and free of ESP-IDF, because both halves of the
straddle need it and neither can reach the other: SUPE's portable core compiles
with a plain g++ (test/), and the driver has to time a frame in a build with no
SUPE in it at all (§19.1.1). One copy on purpose — two are how the radio-check
sweep's airtime came to be over-stated by half. `computeBitrate` registers
`bitrate_eff = (500 × 8) / ceil(ToA of one 500-byte frame) = 4000 / ceil(ToA_s)`.

This is deliberate: RNS derives its first-hop link-establishment timeout as
`MTU×8/bitrate + 6 s`, so registering this bitrate makes that term equal
`ceil(airtime) + 6 s` — link setup waits roughly one frame's real airtime plus
margin instead of a fixed budget. It is **not** the LoRa channel symbol rate
(`SF × BW/2^SF × 4/CR`).

## 9. Config lifecycle

A change to any `s.lora.*` or `secrets.lora.*` key fires `onCfgChange`, which
calls `cfgArmSettle()`. The apply is **coalesced**: a radio restart is what an
apply costs (`radioStop` + `radioStart` + a fresh rnsd registration), and a
configuration burst — an RNode client's frequency, bandwidth, spreading factor
and power arriving as four separate writes, or the same typed as one
`;`-separated CLI line — would otherwise pay that once per key. One burst
becomes one restart and one `registerWithRnsd`.

### 9.1 The settle window

There are two coalescing questions here and they have different answers.
`LORA_CFG_COALESCE_MS` (300 ms) is "four keys written in one breath cost one
apply". `LORA_CFG_SETTLE_MS` (10 s) is **"the person is still typing"** — a
frequency, a bandwidth and a spreading factor arriving seconds apart as somebody
works down a settings pane. Between those writes the radio is configured for a
combination the user never asked for and would not choose, and 300 ms is
nowhere near long enough to span it.

So a change to a user's own settings arms the settle window instead, and the
window is **radio silence as well as a deferred apply** — transmitting on a
half-edited configuration is the part that reaches other people. While
`loraCfgQuiet()` holds:

- `drainOneOutbound` returns before staging anything: the queue and the ITS
  buffers hold what they hold, exactly as they do for every other owner of the
  radio;
- `supePoll` starts no transaction and skips its announce beat. A transaction
  already running is left to finish — the far end is timing against it and it
  is over in well under the window — and an armed offer simply launches when
  the window closes;
- a manual `lora <n> tx` is **refused** rather than held: it is a typed
  one-shot, and a transmit that happens ten seconds later on different
  parameters is not the one that was asked for.

Receive is untouched throughout.

**This is the one deadline that slides.** `cfgArm` arms once and can only be
pulled in (below); "after the last change" means the opposite, so
`cfgArmSettle()` pushes its deadline out on every further change. That needs the
starvation bound the immovable one did not, which is `LORA_CFG_SETTLE_MAX_MS`
(60 s), measured from the first change of the burst: past it the apply happens
whatever is still arriving. And it may only move **its own** deadline — an apply
pending for any other reason is left where it is, or a stream of edits could
hold off a radio the orchestrator is trying to bring up. `cfgArm` clears
`s_settling` for the same reason.

**Two places must not be left past-due.** `nextDeadline()` turns an overdue
deadline into a zero-length sleep, so an apply that is due but held spins the
task at full CPU. The apply that comes due with a frame still on air
(`anyRadioOnAir`, below) therefore *moves* its deadline by
`LORA_CFG_ONAIR_RETRY_MS` rather than merely failing a test; and the outbound
and SUPE wake terms in `nextDeadline` drop out entirely while the window is open
(`outReady` takes `!loraCfgQuiet()`, `supeNextDeadlineMs` returns `UINT32_MAX`),
since an armed offer or a queued packet would otherwise ask to be re-sensed at
slot pace for ten seconds over work that is not going to happen.

**The apply itself waits for the air to clear.** `anyRadioOnAir()` — a transmit
in flight, a manual transmit, or a live SUPE transaction on any radio — defers
the pass, because the apply stops and restarts the radio and would otherwise cut
a frame off the air or strand a detour the far end is still waiting on. The
settle window makes this rare, but a frame that began just before the window
opened is still flying, and at SF12 it flies for seconds.

### 9.2 Why `cfgArm` itself is immovable

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
  such terminator, and takes the settle window of §9.1 instead.

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
defaults under a `s.lora.version` gate (`LORA_VERSION = 10`) for radios **1..**
only — radio 0's defaults come from this straddle's `settings:` block in
`straddle.yaml`, **except** `s.lora.0.bandwidth`, seeded here because its pane
row binds the kHz display key rather than the Hz config key.

The same gate carries a rename that was one setting under two names: `afa` →
`SUPE.afa`, the regime being SUPE's own. It moves at its existing value rather
than silently changing a node's behaviour, and the old key is deleted.
`announce_interval` sits beside it at interface level and is **not** a SUPE key:
it paces the Reticulum announces rnsd replays onto this interface whether or not
SUPE is compiled in, and ANNOUNCE when it is — one question, one answer. It
also deletes `SUPE.adaptive_txpower`:
transmit power is not a setting (§15.4), so it is not an answer to a question
anybody asks. Frequency and TX power carry no default
(region/antenna — the user must pick); everything else defaults so an
enable-toggle alone gets a radio up. The **RNode group is global, not per radio**
— there is one endpoint for the device — and is seeded under the same gate with
one switch per door: `s.lora.rnode.radio` (0), `.serial` (1), `.tcp` (0) and
`.ble` (1) — there is no master enable, and `.tcp` is a switch rather than a
port number, since 7633 is hardcoded in every stock client. The serial and TCP
rows come from this straddle's pane block; rnode-ble contributes the Bluetooth
row into the same section. The existing
`storageSubscribeChanges("s.lora", …)` prefix subscription already covers the
group, so its edits land in the coalesced apply pass like any other. `loraInit`
does **not** touch any power pin — the board owns the LoRa rail.

## 12. LoRaMon (per-frame telemetry)

Every on-air frame is recorded for the LoRaMon viewers, which are their own
straddle — [loramon](../loramon), the browser window and the LCD app. This
section is the **recorder**: what iface-lora writes and when. What the viewers
do with it is loramon's INTERNALS.

The whole of it is gated on `CONFIG_STRADDLE_LORAMON` in `lora_mon.cpp` — the
presence symbol for that straddle. Nothing here reads back what it writes, so a
build made with `--without loramon` has no recorder at all: not the per-frame
nodes, not the neighbourhood rows, not the airtime rollup, not the channel-RSSI
beat (§18.3), and not the 16 KB-per-radio expiry FIFO. The gate is an `#if`
inside the file rather than a `conditional/loramon/` slice dir, because the
radio task calls into the recorder on every frame and a source file that simply
vanished would leave those calls unresolved; `loraMonOpen()` is a compile-time
`false` instead, so every call site asks the same question either way and the
optimiser folds the rest away. What survives the gate is the stats flush, the
status pill, the channel list and the state keys — those have their own readers
in the settings pane and the status bar.

The storage subtree **is** the ring — no in-firmware record buffer, no ITS
transfer:

- **The neighbourhood, published.** `lora.<n>.peers.<slot>` =
  `"<num>|<supe>|<tags…>|<names…>|…"`, one node per peer-table slot, rewritten on
  the stats beat and deleted when a slot empties. `<num>` is the number `lora n`
  prints beside the node, counted the way `lora n` counts it — used, not us, in
  table order — so a viewer with no name to show falls back to it and a person
  reading both surfaces sees the same `#4`. Tags are three-byte prefixes,
  comma-joined and deduplicated — the whole set that resolves an address to this
  node — and names are the first word of each announced LXMF name on its
  destinations, comma-joined, duplicates dropped. **The record is built in a
  `std::string` and has no length cap**, because the tag set has none worth
  betting on: a node reached over links accrues one tag per link identifier
  (`NEI_HASHES_MAX` of them, shared across the table, and they land on whoever
  is actually being talked to), and the tags precede the names. Cut at a fixed
  buffer's end, the peer that loses its names and its trailing fields is by
  construction the one whose traffic a viewer is hovering, and the tags that
  fell off resolve to nobody — the busiest node on the graph reads as `#4` or
  as raw hex while the quiet ones read fine. Published
  rather than derived, because the mapping lives in the peer table and nothing
  outside this straddle can rebuild it: a viewer sees frames, not the announces
  and proofs that clustered them into nodes. LoRaMon reads it to name the node
  behind a frame's tag — on hover in the browser window, on tap in the LCD app;
  a graph view is the reason it is a general key rather than a field on a
  record.

- **One node per frame.** `loraMonPush` (from the RX drain and each TxDone, so
  once per on-air frame → two per split RNS packet) writes
  `lora.<n>.packets.<ms>` = a packed string; direction is the leading token, snr
  is deci-dB:
  - rx: `r|<rssi>|<snr>|<dur_ms>|<bytes>|<type>|<ch>|<desc>|<cast>[|<tag>]`
  - tx: `t|<txp>|<dur_ms>|<bytes>|<type>|<wait_ms>|<ch>|<own_ms>|<desc>|<cast>[|<tag>]`
  - dwell: `a|<ch>|<dur_ms>[|<tag>]`

  A **dwell** is not a frame: it is the radio tuned to that channel and
  listening for that long, written by `loraMonDwell` on every retune and once
  per maintenance beat, and extended in place while the stay continues so an
  idle hour is one record and not one per beat. It is what lets a lane say where
  the radio *was*, which no frame record can — a lane with nothing in it means
  both "nobody spoke" and "we were not listening", and those are the two answers
  a channel view exists to separate.

  **A dwell's `<tag>` is the meeting whose slot the stay is** (`supeMeetingTag`,
  sampled as the record is posted), and it is the reason a detour channel can be
  labelled at all: a slot belongs to its peer for its whole width whether or not
  a frame ever lands in it, and nothing else on the record stream says so. The
  tag is part of what makes two spans one stay — a meeting ending and another
  beginning on the same channel is two slots belonging to two peers, and the
  extend-in-place check compares it alongside the channel so the boundary
  survives into the viewer.

  `<desc>` says what the frame IS — a code, not a string, because every record
  is a storage node and there may be thousands of them, so the name lives once
  in each viewer's own table. The SUPE frames by name, then Reticulum's packet
  type read off the bottom two bits of its first header byte. A CRC-failed frame
  gets no description at all — reading a type out of bytes that failed their
  checksum is reading noise.

  **Which half of a split a frame is, the record stream works out for itself**
  (`loraMonClassify`, one tracker per direction). `loraMonDescribe` and
  `loraMonTagOf` both read the Reticulum header, so both want the head; nothing
  in a frame says which half it is, and — decisively — **the reassembly state
  cannot be asked**. A train's frames are buffered by the meeting and
  reassembled only at its close, so at record time `splitPending` has not moved,
  and on transmit `train_fire` stages every frame as `txFrame[0]`, so the
  completion index cannot say either. Read that way, a tail's second byte is
  payload, and `f[1] & 0x03` on arbitrary bytes yields a *confident* packet
  type: a 467-byte transfer records as `data` followed by `announce`, on a
  detour, where an announce cannot be. Trains are most of the traffic, so that
  is most of the graph. The tracker applies the same seq-and-timeout rule
  reassembly does, to the recorded stream instead.

  A tail therefore reports two things. Its own description is `split`, which is
  what a tail is; the **packet's** description is inherited from the head and is
  what `<cast>` keys off, since an announce too big for one frame is a broadcast
  in both halves and colouring the second as a unicast would say the packet
  changed audience halfway through the air. The head's address is inherited the
  same way.

  A tail that arrives with no head pending — in a meeting the head is resent
  after the tail, in the repair round — is still a tail, and says so: `split`
  for both descriptions and no tag. A head is always the full frame
  (`RNODE_MAX_PAYLOAD` of payload), so a shorter split frame cannot be one, and
  describing it from its payload bytes named a random packet type at a random
  node. It opens no pending of its own; the head, when it comes, does.

  `<tag>` is the three bytes naming the node the frame concerns, present only
  when there is one to take. It is the same prefix the protocol classifies on
  and the one every SUPE log line quotes, which is what lets a bar on a graph
  and a line in a log be recognised as the same event; `lora.<n>.peers.*` above
  is what turns it back into a node with a name. Inside a SUPE meeting the tag
  is the **peer the meeting is with** (`supeMeetingTag`), not the address on the
  frame: a train's frames carry whatever destination their payload was headed
  for, and a viewer asking "who was this exchange with" wants one answer for the
  whole train.

  `<cast>` is who the frame was aimed at — `0` broadcast, `1` a unicast for us,
  `2` a unicast for somebody else, `3` a unicast for us by a **link identifier**
  rather than by a destination or an identity (`LMC_*`). The device decides it because a
  viewer cannot: it turns on which addresses mean US, and that lives in the peer
  table. It is what the viewers colour by, so it has to be on every record rather
  than derived per frame at draw time.

  **`<cast>` and `<tag>` are different questions and must be answered
  separately.** Whose traffic this is decides the colour; who it is with is a
  label for a person. Neither may be derived from the other: inside a meeting
  the tag is the *peer*, which is never one of our own addresses, so resolving
  the tag against the local rows says "somebody else's" about the bulk of our
  own traffic. The three answers, in order:
  - a description of `announce` or `ANNOUNCE` → `LMC_BCAST`. Broadcast is a
    property of the frame, not of an address, so it is read off `<desc>` and
    `loraMonCastOf` never returns it.
  - a frame belonging to a meeting of ours → `LMC_US`. A meeting has two parties
    and we are one of them; this is known, not inferred, and it covers every
    meeting frame that carries no address at all. The meeting is sampled **both
    before and after the SUPE dispatch**, because the dispatch is what opens and
    closes one: asking only after calls the closing frame somebody else's, and
    asking only before does the same to the opener.
  - otherwise `loraMonCastOf` on the frame's own address: a local row's
    destination or identity → `LMC_US`; an address that means us without
    naming us → `LMC_US` — the truncated packet hash a delivery proof is
    addressed to, held in the peer table's pending-proof entries (proofs we
    elicited from direct neighbours) and in the engine's tag set (one per
    single-destination packet we sent or relayed, for the receipt window); a
    link we are an endpoint of → `LMC_US_LINK`; anything else, an address that
    resolves to nobody included, → `LMC_OTHER`. Unknown is not the same as
    ours. A proof's `tag` is resolved the same way: when a pending entry knows
    the destination the proved packet went to, the record is tagged with that
    node rather than with the hash.

  `LMC_US_LINK` is ours by every colouring rule — both viewers paint it as our
  own traffic — and is split out for one reason: it is where an unresolvable
  `<tag>` is a **fact rather than a gap**. A link request carries no sender and
  the session's identify step is encrypted inside it, so the far end of a link
  **dialled to us** is anonymous for the link's whole life; there is nothing to
  file it under and nothing later supplies it (a link *we* dialled has its
  identifier filed on the peer's row by `neiLink`, so it resolves and this never
  shows). Six hex characters with nothing behind them read as a failure of the
  peer table, so the browser's hover says `inbound link` beside them instead.

  On transmit `LMC_US` cannot arise, so `LMC_OTHER` is what the graph reads as
  "our unicast", and our own broadcast carries no tag at all — the address on it
  is ours, and naming ourselves above a bar that is already ours by its colour
  says nothing.

  A **split packet's address lives in its head** and both halves record it, by
  the same inheritance the description uses (see `loraMonClassify` above).

  **Two SUPE frames name their own sender**, and their `<tag>` comes from that
  rather than from an address field (`loraMonSenderOf`). A received **HAIL
  names US** in its address field — it is aimed at this node — so the
  `sender_ident` it carries is what the hail concerns; that is what the field is
  for, and it is why a hail on the graph is labelled with whoever sent it. An
  **ANNOUNCE** has no address field at all: its payload *is* the sender's
  identities, and the first of them is the one `annIngest` resolves the frame's
  node through — so it is the one that resolves through a published tag set too,
  being that row's `node4`, one of its idents, or the front of one of its
  destinations, whichever the row was found by. Without it the announcement —
  the frame that introduces a node — would be the one frame on the graph
  attributed to nobody.
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
  `2` a packet the attached RNode client originated (§17), `3` a frame whose CRC
  failed. Only `3` reaches the viewers' colour: they colour by **direction and
  audience**, not protocol — red is what this radio put on the air and blue what
  arrived, light for a broadcast and dark for a frame with one addressee, grey
  for somebody else's unicast, and purple for a CRC failure, which is neither,
  since nothing in it was readable. `<type>` stays on the record because the
  byte-count convention below turns on it. RX is classified by whether the own-protocol branch consumed
  the frame (so the tap runs before the record is written); TX by
  `LoraRadio.txType[]`, which is per-frame because a power request (§15.1) and
  the RNS packet it prefixes share one burst but not one protocol. The record's
  byte count is payload bytes, and the rule is the same in both directions: the
  1-byte seq/split header is stripped for every type **except** `1`, which
  carries none — RNode-origin packets fly through the same framing rnsd's do.
  It has to be the same both ways, because **the wire length is how a reader
  reaches fields the record does not carry**: a THATSIT is `SUPE_THATSIT_BASE`
  plus one checksum per train frame, so a byte off is a *frame* off, and a repair
  round then reads as a resend of something already received.
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
  fills from open forward. **The in-flight dwell goes with it** — and that takes
  a write on each task, because the two halves belong to different ones.
  - `dwellKeyMs`, interface task, cleared in `loraMonClear`. A stay on one
    channel is one record *extended in place*, so the key it extends has to still
    exist; carried across the clear, the first stay after the next open rewrites
    a node that was just deleted — at a timestamp from before the window opened,
    and outside the FIFO that expires it.
  - `dwellSince`, radio task, dropped when its own poll sees the watch flag
    change (`dwellWatched`). A window that has just opened must start recording
    the stay the radio is **already on**: waiting for the next retune leaves the
    lane veiled until something happens to move the radio, so the background
    appears to begin at the first traffic rather than at the moment the viewer
    opened. Dropping the anchor rather than keeping it is what stops that first
    span reaching back before the window; the poll takes a fresh one on the same
    pass. Note the poll cannot be gated on `dwellSince` being set — only
    `loraMonDwell` assigns it, so a gate like that never re-arms.
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
  carry. So a 10-byte GIMME reads 10 on the sender's graph and 9 on the
  receiver's. Cosmetic for the drawing, load-bearing when checking a frame
  against §3 of `plans/SUPE.md`.
- **A second series, live-only: the channel noise floor.** While a viewer is
  open, once a second each radio publishes its channel-RSSI sample set to
  `lora.<n>.rssi` (§18.3), which the viewers draw as a light backdrop under
  the traffic on every channel's graph. The sampling itself is gated on the
  viewer too — see §18.3. Unlike the packet nodes there is no history to
  mirror — only the newest sample is published — so the series starts when the
  window opens and a skipped beat simply reads as a gap. The hailing lane also
  carries the **floor** drawn from it: one dotted line at the quietest usable
  reading of the last minute, with its figure on a pill sitting ON the line, at
  the right end and clear of the rx scale — the axis the number is quoting. A
  minute rather than the window on screen, because it is a statement about
  conditions now: zooming out to an hour must not turn it into the quietest
  moment of that hour. The quietest rather than the mean, because a mean over a
  channel carrying traffic measures the traffic.

  **Two kinds of non-reading are dropped, and there is no floor at all when
  nothing survives them** — a level nobody can vouch for is worse than none. A
  zero is a beat that produced no reading. Anything at the register's weakest
  rail (`NOISE_RAIL_DBM`, −127) is the *rail*, not the channel: the level is a
  byte read as −value/2 dBm, so −127.5 is as quiet as it can say, and it says
  that whenever the front end has nothing yet — which is exactly the first
  minutes after a radio comes up, where a floor pinned at −128 dBm sat on the
  bottom edge of the plot being wrong. It cannot be a measurement either:
  thermal noise alone is about −123 dBm in 125 kHz before any receiver's own
  noise figure, so no working front end reads below it. The pill is also clamped
  whole inside the lane, since a line along an edge would otherwise cut it in
  half.
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

What the two viewers make of all this — the axes, the colour scheme, the peer
pills, the zoom stack, the live-edge lag — is [loramon's
INTERNALS](../loramon/INTERNALS.md) §2. Nothing there is readable by this
straddle and nothing here depends on it: the contract between them is the
subtree and the two watch keys, and both are written out above.

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
  task). Both reads depend on the header's **context flag** (byte 0 bit
  `0x20`), the only thing that says whether a 32-byte ratchet sits between
  `random_hash` and the signature — the field itself is unmarked, and every
  destination rnsd hosts for a consumer announces one, so a parser that assumes
  the ratchet away verifies nothing but `rnstransport.probe`. The ratchet is
  part of the signed data and is skipped, not stored: this table clusters
  addresses, and key rotation is rnsd's business.
  Dest hashes announced under one key cluster under one identity;
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
- **The transport identity is a key of its own, and nothing announces it.** A
  node's Transport instance holds its own identity, separate from the one its
  destinations hang off, and Reticulum never announces it: it appears on the air
  only as the first address field of a packet in transport — which is to say, as
  the tag of every packet relayed *towards* that node. So the announce join
  cannot supply it, and a relayed announce's claim to it is unverifiable, which
  is why `observeAnnounce` refuses to mint a row from one. The one place it
  arrives attributably is a **SUPE announcement**: a node learns its own
  transport identity from the announces it relays (its `isTx`+HEADER_2 branch)
  and lists it with the rest, so `annIngest` files every announced identity that
  matches no row of its own as a hash meaning that node. Without that, a
  neighbour's transport identity is knowable to nobody, every packet in transit
  through it resolves to `LORAQ_PEER_NONE`, and transit — most of what a gateway
  carries — never leaves the shared channel.
- **Two lookups, deliberately different.** `peersFindBy4` answers "which node is
  this next hop", searching `node4`, destinations and the hash store —
  link identifiers and the transport identity above. `tagNode` (in `lora_supe`)
  answers that *and* "which node does this sender identity name", so it searches
  `ids[]` too. The transport identity is the only identity a packet is ever
  addressed to, and it reaches `peersFindBy4` through the hash store rather than
  through `ids[]`; widening the search to `ids[]` would otherwise only ever fire
  on a four-byte collision, so the asymmetry stays.
- **Links.** An LR yields `link_id = H([flags&0x0F] ‖ raw[2:])[:16]` with LR
  data trimmed to the 64 ephemeral-key bytes (MTU signalling excluded), mapped
  to its dest. The hashed part excludes hops and the transport id, so every hop
  of a relayed request derives the same identifier — which is what lets a relay
  file the link at all. The LRPROOF (context 0xFF, dest = link_id) marks it
  established and — at hops 0 — attributes its signal to the dest, which is
  thereby proven a direct neighbour. Mid-link traffic on an unseen link_id
  creates an *unresolved* entry. `ours` = we transmit on it at hops 0, or its
  LR dest is one of our hashes.

  **Whose link it is, and the three ways of knowing.** A link identifier is the
  address every frame of a session carries once it is up, and it belongs to
  neither end's announced set — so unless it is filed on a node's row
  (`peersAddLink4`) the whole session resolves to nobody: absent from `lora n`,
  unnamed on a graph, and — worse — carrying `LORAQ_PEER_NONE` into the queue,
  where everything keyed on the queued peer (the per-peer cap, the power
  controller, the reverse leg's scan) silently finds nothing. What it is filed
  against is the **first hop**, never the far end of the path — the two are the
  same node only on a link to a direct neighbour. The three handles:
  - **We dialled, or we relay.** Any LR at `isTx` files the identifier against
    the row of the node it goes to: the transport id when the request is in
    transport (a destination behind a gateway is dialled *through* it), else the
    dialled destination when that destination is a neighbour. A relayed request
    counts — the return direction arrives addressed to the identifier rather
    than to our transport identity, one hop before any frame that carries it.
    Every inbound frame at hops 0 on a link we initiated re-files it, which is
    also what catches a link we only picked up mid-session.
  - **It was dialled to us, as a detour's cargo.** A schedule belongs to one
    pair and the frame came under it, so `fromPeer` names the dialler
    (`supeCargoPeer`). Taken on the LR and on any later frame of the session,
    since a responder gets no other handle.
  - **It was dialled to us in the clear.** Anonymous, and stays that way: a link
    request carries no sender, and the session's own identify step is encrypted
    inside the link. `apNextHop4` hands back the identifier itself rather than a
    node, which is the honest answer.
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
- **Path loss, both directions.** A row holds up to three path-loss readings
  (§15.1): the hailing pair and the step pair (them→us — a level heard here
  against the power the peer stated for it) and the peer's report of our own
  frame (us→them). `peersLossFrom` picks the fresher pair for them→us. `lora n`
  prints both directions with the age of each on a `path loss` line; the same
  figures leave the binary as `lora.<n>.meas.<slot>.*` (`publishMeas`, one
  record per slot, every `LORA_MEAS_MS` = 15 s while any radio's table holds a
  neighbour and the frame counters have moved since the last publication —
  `measTrafficSig` — deleted when the slot empties), keyed for outside readers
  by the six-hex `tags` a node answers to — that is how lxmf's Ping and contact
  bars, and the web contact list, find a destination hash's radio link. The
  beat is independent of any UI being present because those readers are
  firmware tasks, and a quiet neighbourhood arms no wake at all; traffic that
  lands during a long idle block is published at the task's next wake.
- **Transit neighbours.** A rebroadcast announce (hops ≥ 1) is the one relayed
  frame whose transmitter is named: the rebroadcaster stamps its own identity
  hash as the HEADER_2 `transport_id` (how path tables learn `first_hop`). It
  samples that neighbour's envelope/rollup, keyed by identity so the node's
  own hops-0 announces join the same row, and tags the row `transit`.
  **Only against a row that already exists**, though: the field is unverified
  (the announce signature covers the originator, not the relayer), which is
  enough to attribute a signal to a node we have otherwise met and not enough to
  MINT one. A row conjured from it holds nothing but that claim — no
  destination, no announce, nothing the node ever signed — and shows up in the
  neighbourhood as a peer that may not exist, most often as a second row for a
  relayer already listed under its own announces. Unattributed, the frame counts
  in the anonymous-transit row below, which is what that row is for; once the
  relayer announces for itself the row is real and every later rebroadcast
  attributes to it. The same frame identifies
  *us* symmetrically: a rebroadcast we transmit stamps our own transport
  identity as transport_id, which becomes (or merges into) a `us … transit`
  row — the identity this node is known by on the air when it relays.
- **Handing the clustering to rnsd.** rnsd builds one neighbourhood for every
  medium and groups a node's destinations by asking the interface who
  transmitted them — free on a point-to-point medium, impossible per packet on a
  radio. But an ANNOUNCE is the one frame this table can name outright: it
  verified the signature and joined the destination to a row, and announces are
  the only frames rnsd's neighbourhood is built from. So the interface registers
  with `rx_origin` and prefixes each forwarded frame with `peersRnsdKey()` — the
  row index, offset by one so an all-zero key stays rnsd's "unknown" — and
  declares the row with `RNSD_IFACE_AUX_PEER`. The ROW is the right key because
  it is this table's notion of one node: every join it makes ends in one, so the
  shared listing (and NetGraph) group exactly as `lora n` does. `peersMergeInto`
  withdraws the absorbed row's key and re-declares the survivor, and `peersAlloc`
  withdraws before reusing a slot, so rnsd never holds a node this table no
  longer has. Everything is fire-and-forget at zero timeout — it runs on the
  radio task in the receive path, and a dropped declaration costs one announce
  interval, since every announce re-declares.
- **Anonymous transit.** Every rx frame at wire hops ≥ 1 was transmitted by an
  in-range transport node even when nothing names it (HEADER_1 relays, relayed
  proofs, a silent access-point bridge, and a rebroadcast announce whose
  `transport_id` we do not already know). Those sample one aggregate
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
the announce carried one. That name and that aspect label both come out of
rnsd — `rnsdAnnounceName` and `rnsdAspectLabel` — rather than out of a decoder
of our own: app_data is bytes an application chose, a name is only what survives
being checked as text, and the node that sees every announce on every medium
owns the rule. A second copy here would drift, and a drifted copy shows a
different name on this pane than on every other surface of the same device. The
transport hash leads each block, being the one hash every node has. A hash
linked to a node but never heard directly prints as
`<first-4>........ (not seen yet)`.

A capability line closes each non-`us` block:

| tag | means |
|---|---|
| `TRANSPORT` | it relayed someone else's frame to us — a rebroadcast announce naming itself as `transport_id`, or any HEADER_2 frame at hops > 0 that does |
| `ROAMING` | its node-flags bit (a moving node wants more margin) |
| `SUPE` (`RF_PROTO_NAME`) | it has spoken our air protocol to us — a SUPE announcement, or a 0x04 power request |
| `EST <dBm>` | the power this node needs toward that peer, *inferred* by reciprocity from frames we overheard over the last three bucket-ring slots, crediting the peer with `s.lora.assumed_peer_txp` (default 22). It is the only source there is for a peer that does not speak our air protocol; for anything that does, a stated power replaces the assumed one and a meeting's reports (GIMME, the answering HAVEDATA) replace the direction (§15.2). |
| `USE <dBm>` | what the last frame to this node went out at, and the tildes say how much of it was guessed: none when the peer itself reported the level our frame landed at, `~` from a path loss measured the other way round, `~~` from `EST` alone (§15). Absent means the configured `tx_power` — no evidence, or none of it fresh |

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
queue (`LORAQ_F_REPLAY` keeps `beginTx` from re-recording it). The run closes
with this node's own SUPE announcement. The replay owns nothing: it is queue
traffic end to end, so there is no radio standoff and nothing for a SUPE
transaction or a manual transmit to wait on.

**Announces go out as a train.** The first pays ordinary channel access; each
one after it chains from the previous one's transmit-done (`annTrainChain`)
with nothing but the receiver's flip gap between them — the spacing a split's
two halves already fly at — and so does the SUPE announcement that closes a
replay (`supeAnnTrainFire`). The polite wait is paid again only where the next
announce would carry the run past `LORA_TX_TRAIN_MAX_MS` (1 s) of continuous
air: `txTrainMs` accumulates on-air time since the last grant and resets at
the next. At SF7/125 kHz a typical announce is about 300 ms, so a run is three
announces, then a wait, then three more; at SF9 and above every announce is a
run of its own. The budget is the single-transmission ceiling EN 300 220 puts
on a channel, and it is also what keeps a replay from monopolising a shared
channel. Ingress marks announces with `LORAQ_F_ANNOUNCE` (own and replayed
alike); nothing else chains.

**Two rules hold here, and both are load-bearing:**

- **An announce is the daemon's decision and its timing is part of what it
  decided.** Nothing intercepts one, so nothing can delay somebody else's
  routing decision by holding it — `plans/SUPE.md` §9. What this radio DOES own
  is how often it asks rnsd to say who we are again: `announce_interval` drives
  `rnsdAnnounceBeat`, which asks rnsd for a replay pinned to `lora/<n>`
  (rns/INTERNALS §4.1). That is a request for the daemon's own bytes on our
  schedule, not an interception of somebody else's packet — the difference is
  whose announce is being held, and the answer here is nobody's.
- **Nothing is transmitted in order to measure.** A node's transmit power
  toward a peer comes from traffic that was going to happen anyway: every frame
  a detour sends states the power it went out at, so ordinary exchanges yield a
  *path loss* rather than a bare reading, at two configurations, continuously
  (§19.7, and `plans/SUPE.md` §7's "No power sweep follows it").

The second is why `lora n` has no measured `TX <dBm>` column: the only passive
source outside a detour is `EST` (reciprocity, §13.1), and SUPE's own path-loss
pairs replace it for any peer it actually detours with (§15.2).

## 15. Adaptive TX power

Every frame whose first RF hop is a known node goes out at a power derived for
that node, at the configuration the frame is about to fly at. `lora_power.cpp`
owns the derivation; the per-node evidence sits on the peer table's rows.

**Nothing is stored as a power.** `apDerive` recomputes the number for every
frame, so evidence going stale, a floor decaying and a link that has moved all
show up on the next transmission rather than at some settling time. What *is*
stored is the ratchet's trim and the failure floor — what the loop has learnt
and no measurement can supply. `Neighbor::apPwr` is a display cache for
`lora <n>` and nothing reads it back.

### 15.1 The evidence is a path loss

A measurement is a level in dBm and the power that produced it; the difference
is the loss, and the loss is one property of the link however it was read. What
a *configuration* changes is the sensitivity that loss has to clear. So:

    need = path loss + sensitivity(cfg) + margin

and one measurement serves the hailing channel and every detour step alike.
This is why `host->txp_open` takes a `SupeCfg`: SF5/500k sits some 15–20 dB
above SF12/125k in sensitivity, and a single per-peer power would be tuned for
one of them and wrong for the other by that much. The main channel resolves
against the hailing configuration (`apOpenPower`); a granted step resolves
against the step (`apOpenPowerAt`), on both sides of the transaction.

### 15.2 Three tiers, best evidence first

Each is gated on `AP_FRESH_MS` (10 minutes). Stale evidence falls to the next
tier, and a node we have heard nothing from opens at the configured `tx_power`.

| tier | evidence | margin |
|---|---|---|
| `AP_SRC_REPORT` | the peer stated the level our own frame landed at — GIMME or HAVE reporting the HAIL, an opening HAVE or the train, whichever it answers: the only measurements of the direction we transmit in | `SUPE_TARGET_MARGIN_DB` |
| `AP_SRC_PAIR` | a frame heard here with the power the peer stated for it (§10's pairs, either the hailing one or the step one, whichever is fresher) | `+ AP_RECIP_MARGIN_DB` |
| `AP_SRC_NONE` | neither is fresh — **and every node outside SUPE, permanently**, since both tiers above need a power the peer stated | the configured `tx_power` |

The reciprocal tiers cost their extra margin because ambient noise is **not**
reciprocal even where path loss is: a node sitting beside an interferer needs
more from us than our own quiet receiver would suggest, and no measurement we
can make from here will say so.

### 15.3 The ratchet: up fast, down slowly

- **A miss raises the power immediately.** `apFailed` — a meeting dying after
  contact with our train unconfirmed, or a delivery proof that never came from
  a peer that has also stopped being heard — files a floor `AP_FLOOR_STEP_DB` above what was tried, on a decay
  (`AP_FLOOR_DECAY_MS`), zeroes the trim and halves the EST walk. The floor
  outranks every tier, because a measurement can be optimistic and the thing
  that proved it optimistic was a frame that never arrived. Successive misses
  climb 6 dB at a time, each filing its floor above what the last one tried.
- **A miss from a peer we can still hear is not one.** `peersQuality` reaches
  `apFailed` only when nothing has been heard from that node for
  `AP_MISS_QUIET_MS`. An unresolved proof expectation from a node whose frames
  keep arriving is a congested medium, a busy far end, or a transfer stalled
  above the radio — all of them made worse by transmitting harder. The quality
  counters score it regardless: that is a statement about the link, and this is
  a statement about the power.
- **A success lowers it one dB, and only on evidence.** `apSucceeded` needs
  `AP_MIN_SAMPLES` clean exchanges, and the glue only calls it when the peer's
  report carried real headroom — thin margin holds. A controller that dials
  down on a timer walks a quiet link into the ground.
- **The trim is capped at `AP_TRIM_MAX_DB` over a measured need.** The
  measurement already aimed at a target margin; the trim is for what
  measurement cannot see, not a second opinion about the link.

Both feedback paths land in the same two functions: a meeting's close
(`SUPE_EV_TRAIN_OK` / `SUPE_EV_TRAIN_LOST`) on every exchange at no cost, and
Reticulum's own delivery proofs on plain traffic (`peersQuality`), slower.

**Every miss note carries the configuration it failed at.** `apMissWasPower`
weighs the loss the peer measured against what that regime needs, and a note
that leaves `cfg` unset defeats it silently rather than loudly: a zero bandwidth
puts zero into a logarithm, and the result lands on 0 dBm — a receiver that
needs a signal stronger than any transmitter can produce, against which every
real link reads as too weak and every miss is scored as too little power.
`supeSensitivityDeci` now answers "unknown" for a blank configuration instead,
with a floor no measurement will beat, so the same omission can only ever hold
the power rather than raise it. What that cost on the bench: an unanswered
closing frame means only that an acknowledgement went missing — the train
itself arrived — and it recurs, so a table-top pair ratcheted −9 dBm to 22 dBm
over four minutes, each end climbing because the other had.

### 15.4 It runs against a SUPE node or not at all, and has no key of its own

Both derived tiers are fed by frames that **state the power they went out at**,
and stating that power is what the protocol is *for* — a node cannot both speak
it and decline to use what it says. So there is nothing to switch, and no key.

**It does follow SUPE's own switch**, and that is not the same statement.
`SUPE.enable=0` leaves announcement ingest running — deliberately, so the
picture of the neighbourhood stays current and the switch can be thrown back
without rediscovering everyone (§19) — so the path-loss pairs keep arriving and
keep being fresh. A controller that read them would go on deriving from a
protocol this node has just announced, in its own ANNOUNCE, that it does not
speak: dialling a frame down to a number no `tx_power` explains, answering a
peer's `0x04` request and prefixing one of its own. `apEnabled` is the single
gate, and `apTxPower` is the one place the tx path can reach a derivation, so
off means every frame goes out at exactly the configured `tx_power`.

A node outside the protocol therefore sits at `AP_SRC_NONE` permanently and is
transmitted to at `tx_power`. That is not a gap to be filled by estimating from
the level its frames arrive with here. Such an estimate is a guess in two
directions at once — the path is measured the wrong way round and the peer's own
power is assumed — and, decisively, **nothing can catch it being wrong**: there
is no return measurement, and Reticulum's delivery proofs are far too sparse to
serve, since nothing inside an established link elicits one.

The case that settles it is two such nodes facing each other. Each reads the
other's surplus off its own receiver, each dials down, and neither has any way to
say *too quiet* — the reciprocity assumption both are relying on is exactly what
fails when both ends move. A link that would have worked at either end's
configured power is walked into the ground by both of them at once, and the only
signal that anything is wrong is traffic that stops. Transmitting at the
configured power is the honest answer, and it is what a peer outside the protocol
gets.

### 15.5 The answering side

**Every meeting frame is adapted to the one node it addresses** — `meetTxp` in
the engine: the controller's derivation for the peer at the configuration the
frame flies at, capped by the channel's regulatory limit. The listener's GIMME
and everything behind it fly at its derivation for the seeker; a seeker that
did not name itself (`sender_ident=0`) leaves no row to resolve, and the answer
takes the cap. Nothing needs third-party reach: no frame carries a hold, a
reservation or a hint for anyone but its addressee, so there is no frame that
must go out at maximum for somebody else's sake.

**The train's power is resolved where the train flies, on the report just
received.** The answer's reading — GIMME's of the HAIL or of our opening HAVE
— is filed through `SUPE_EV_REPORT` into `apFileReport` before the engine asks
`txp_open` at the confirmed budget's configuration — so the freshest
measurement in the protocol, milliseconds old on the very channel, is what the
train power derives from. THATSIT states the result one frame later, which is
what keeps the peer's pairing true. A hailing-rate dialogue has no THATSIT, so
its train flies at the power the hail stated and the controller adapts
between dialogues, not inside one.

### 15.6 What is never adapted

**A broadcast**, always. An announce has no single next hop and must reach
everyone, so it goes out at the configured `tx_power`.

**The second attempt and every one after it.** The ladder's first rung is what
the evidence says the peer needs; a request that drew silence has already shown
that wrong, and the only question left — is this peer reachable at all — is one
maximum answers in a single frame. `supeEngLaunch` takes `pv.txpOpen` at rung 0
and `txpMax` above it, and `supeEngResend` raises `startTxp` to `txpMax` before
retransmitting, so a strike is scored against what was actually tried.

### 15.7 The capability bit that is not one

`SupeCaps` carries no adaptive-power flag. Every node that speaks SUPE derives
its power this way, so a bit announcing it would announce a constant — and the
top bit of the maximum-power byte it used to ride in is back to being part of
the level.

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
  don't drain them in a narrow loop. It must **not** also stand the SUPE engine
  down, and must not reach it as `rx_busy` either. Standing the engine down
  leaves its next event computed from a slot already past, so the deadline pins
  at zero and the main loop spins for the whole timeout while no schedule can
  reach its horizon. Reporting it as `rx_busy` is the opposite error: that
  answer defers attending a slot, which is right for a preamble ending in
  milliseconds and wrong for a timer measured in seconds — every slot inside the
  window is walked past, so the node goes deaf to the peer hailing it. Only a
  transmit of our own (`txActive`) stands the engine down.
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
RNode hardware — over USB serial (how RNodes are normally used), RNode-over-TCP,
or Bluetooth — and becomes the **third endpoint of the radio segment**. All
three — the radio, rnsd, and the client — see the same traffic; a packet
entering from any one is presented to the other two.

Everything here lives in `lora_rnode.cpp`. The public half of it — the ITS port
and the Bluetooth door's connect payload — is `include/rnode_door.h`, which is
what a transport straddle includes; the KISS opcodes stay private. The serial
transport rests on a spangap-core mechanism, the serial-port handler registry
([cli-internals §3](../spangap-core/docs/cli-internals.md)); core knows only
"this port has a handler", never that it is RNode. The Bluetooth door is a
separate straddle, `reticulous/rnode-ble`, which dials the same ITS port and
knows nothing about KISS or the radio.

Protocol reference:
`RNS/Interfaces/RNodeInterface.py`.

### 17.1 Settings and transports

| Key | Default | Meaning |
|-----|---------|---------|
| `s.lora.rnode.radio`  | 0 | which radio the endpoint exposes |
| `s.lora.rnode.serial` | 1 | the serial door (highest existing port, in-band trigger) |
| `s.lora.rnode.tcp`    | 0 | the TCP door on port 7633 — a switch, not a port number |
| `s.lora.rnode.upnp`   | 0 | that door published to the internet — net's `publicFacing` flag |
| `s.lora.rnode.ble`    | 1 | the Bluetooth door, read by `reticulous/rnode-ble` |

Global, not per radio: there is one endpoint for the device, and **one switch
per door — no master enable**: an open door is what "enabled" means, and a
client can only arrive through an open one. TCP is the one door that defaults
shut, because a LoRa segment reachable from any host on the LAN is a decision,
not a default. Serial and Bluetooth default open because dormant they cost
nothing: the serial claim is in-band-triggered — the port is a full console,
esptool auto-reset included, until a client's first KISS FEND takes it over —
and both reach nobody a USB cable or a Bluetooth pairing didn't already admit.
A session records which door it entered through (`RnodeState.door`), so
switching one door off drops its own session and leaves another door's alone.

`.tcp` is a switch because the port is not a choice: the client dials 7633 and
nothing else — its `TCPConnection.TARGET_PORT` is hardcoded, and a
`tcp://host:port` config URI does not override it: the whole suffix is handed to
`getaddrinfo` as a hostname and resolution simply fails. The number lives in
`RNODE_TCP_PORT` on this side.

`rnodeApplyTransports()` runs from the coalesced apply pass (§9) and does three
things: drops the session if its own door was switched off or the endpoint
rebound to another radio; registers the TCP endpoint with net — **once**, plus a
re-send whenever `s.lora.rnode.upnp` moves, because that switch travels as the
registration's `publicFacing` flag and nowhere else (net keys endpoints by
`nvsKey`, so the re-send updates the one already there) — and then drives the
listener by writing `s.net.rnode_port` (net polls its `s.net.*` keys from
`epOpenAll` and opens or closes the socket from there — the two-step shape sshd
uses); and claims or releases the serial port — the highest one `sys.usb.serial_ports`
reports, with the KISS FEND (`0xC0`) as the claim's in-band trigger, so the
claim is dormant until a client speaks. The Bluetooth door needs nothing here:
it dials in from its own straddle like any other client. The task subscribes to
`sys.usb.serial_ports` so a transport switch (`usb cdc` / `usb jtag`) re-runs
this pass and moves the claim to the port that is now highest.

The TCP door is compiled behind `CONFIG_SPANGAP_NET`, which is also what pulls
spangap-net into this component's `REQUIRES` when that straddle is staged: the
radio itself needs no network stack, so on a net-less build the door compiles
away and the serial one still works. `straddle.yaml` therefore does not `require:`
spangap-net — an optional door must not make a whole network stack mandatory.

The same pass publishes `lora.rnode.tcp_on` from the TCP switch: the pane's rows
that are only about that door (today the internet-reachable switch and its
caption) gate on that ephemeral key rather than on `s.lora.rnode.tcp` itself,
the same way `rnode-ble`'s rows gate on `ble.rnode.enabled`. Forwarding a door
that is shut would be an offer of nothing, and the row says so by not being
there. Nothing here knows what a port mapper is: the switch is
`when_kconfig`-gated on `CONFIG_SPANGAP_UPNP` in `straddle.yaml`, and the flag
means the same thing to net whether or not one is in the build.

### 17.2 One session, three doors

`RNODE_ITS_PORT` (0x524E, `'RN'`) is a stream-mode ITS server port on the lora
task, `maxHandles = 1`, 4 KB each way. Three transports connect to it — net (a
TCP client), the core serial machinery (a serial client), and `rnode-ble` (a
Bluetooth client) — and `onRnodeConnect` discriminates by the **connect
payload's length**, which is the only discriminator the port offers:

| transport | payload | size |
|---|---|---|
| serial | `serial_handler_connect_t` (cli.h) | 1 |
| Bluetooth | `rnode_door_connect_t` (rnode_door.h) | 12 |
| TCP | `net_connect_t` (net.h) | 28 with `CONFIG_LWIP_IPV6`, 8 without |

The Bluetooth payload is padded to **12** bytes rather than the 8 that
`{ magic, addr[6], type }` would naturally be: 8 is exactly what `net_connect_t`
collapses to in an IPv6-off build, and a collision there would route a Bluetooth
client down the TCP branch. A `static_assert` in `lora_rnode.cpp` keeps the three
sizes pairwise distinct as any of them changes, and the payload's `magic`
(`0xB7`) is checked on the Bluetooth branch as a second line.

`maxHandles = 1` plus an explicit reject in `onRnodeConnect` is what enforces the
single-session policy across transports: a serial takeover attempted while a TCP
client is attached is refused, and because the refusal happens before any
takeover, **the console is not disturbed by it**. The same refusal is what a
Bluetooth client meets; `rnode-ble` drops the BLE connection on it, and the RNS
client re-dials every 2.5 s until the endpoint frees. `onRnodeConnect` also rejects
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

**A SUPE train carries the release too.** On the plain path the release rides
transmit-done; inside a train the packet has been cut into frames, and a split's
halves are one packet as far as the client is concerned — so `txTRelease` marks
the *last* frame a client packet was cut into, and `hTrainFire` moves that flag
onto `txFromRnode` as it fires. It is spent on the first firing: a repair round
resends the same frame, and a second release would let the client put two
packets in flight.

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
retune.

**And no other channel is measured. The radio never leaves the hailing channel
to take a reading** — not for the regime's agile channels, not for anything. The
rule is worth stating as a rule, because sampling them is a natural-looking idea
(standby → `setFrequency` → `startReceive` → `getRSSI` per channel, then home)
and it costs more than it looks:

- **A frame arriving inside the excursion is lost, invisibly.** Gating the trip
  on the hailing channel reading quiet does not save it: that test is carrier
  sense's and inherits carrier sense's blind spot — a frame below the tracked
  floor does not register, and a preamble that begins *during* the trip cannot.
  Nothing counts what it costs, so the loss surfaces only as a link that
  underperforms for no visible reason.
- **The excursion's cost is per part, and the window it has to fit is per
  configuration.** The ~4 ms an 8-symbol SF7/BW125 preamble allows is not the
  budget at SF12, and an SX126x's retune-and-restart is not an LR2021's. A
  margin that has to hold across both axes is a margin nobody is tracking.
- **Nothing operational would read it.** Channel access takes its own samples,
  SUPE's channel choice reads the airtime ledger alone (`hChanGet`, §19), and
  the power controller works from stated powers (§15). A per-channel noise graph
  is decoration, and decoration does not get to drop frames.

`publishChannels` still lists the agile channels, and the viewer still stacks
a graph per agile channel — **what those graphs draw is the traffic a detour put
there**, from records that exist whether or not anything sampled the noise. The
RSSI series was only a grey backdrop under them, so the lanes now plot their
frames against a plain background. The record's per-channel RSSI fields stay in
the format; a radio publishes one of them.

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
| `lora.<n>.chans` | `"<freqHz>,<bwHz>\|…"`, index = channel, 0 = hailing. One key, not a subtree: a handful of numbers that the viewers want all of at once to label their graphs. A list of **one** entry means no agile lanes, so a viewer tells the two cases apart by the entry count and needs no separate flag. **Two conditions put a lane in the list**: the regime names the channel, *and* `SUPE.enable` is on — a regime says which channels may be used, not that anything will use them, and only a detour ever leaves the hailing channel. So it is republished on the runtime enable toggle as well as at config apply, those being the two things that can change the answer. |
| `lora.<n>.rssi` | `"<ms>\|<ch0 dBm>\|<ch1 dBm>\|…"`, the newest sample set only. The device timestamp is in the **value**, not the key, so a viewer can tell a fresh reading from a repeated one and place it on the same clock the packet nodes use. A skipped beat republishes nothing, so the key is unchanged, no point is appended, and the gap reads as a gap. One key rather than a node per sample: the series is live-only, so there is no backlog to mirror and nothing to expire. |

One field, the hailing channel's: this radio samples nothing else (§18.5), and
the agile lanes draw their traffic against a plain background. The format is
packed rather than singular so that further channels append to it unchanged, and
an unmeasured one is an **empty field** rather than a missing one — a viewer
reads a stable set of columns rather than shifted ones.

### 18.6 What the viewer does with it

The browser LoRaMon draws **one graph per agile channel**, stacked under the
hailing channel's at a quarter its height and the same width — so the same time
axis, and a moment is the same column in every one of them. Same bands, same
dBm scale, same window; only the gutter labels are left off, since repeating one
scale ten times is noise. Each carries its frequency/bandwidth label and its own
transmit airtime over the window on screen. What fills an agile lane is the
**traffic a detour put on that channel** — packet records carry a channel index,
so they land in the right lane with no measurement involved.

The channel-RSSI series draws as a **very light grey backdrop** under the
traffic — a bar always wins the pixels it lands on, so the floor reads as
background texture rather than as something drawn over. The series accumulates
live from the newest published sample, the same rule the packet records follow:
it starts when the window opens. A radio publishes one channel's (§18.3), so the
backdrop appears under the hailing graph and the agile lanes plot against a
plain background.

Per-channel captions carry transmit airtime and **not** "channel busy". A level
sampled once a second invites being read as occupancy, which it is not. The
hailing channel's caption keeps both, and its live-hour figures come from the
firmware's published rollup.

## 19. SUPE (`s.lora.<n>.SUPE.*`)

Protocol: **`plans/SUPE.md`**, authoritative for anything on the air. This
section is what the code does and where it lives.

**SUPE turns unicast traffic into meetings**, with rnsd unmodified and
unaware. One frame on the shared channel asks; the party it names answers;
the traffic follows at the best rate the link supports. What the answer looks
like depends on the regime, so two wire sequences:

```
regime 0 — one channel, one dialogue (everything on the hailing frequency,
           under the interface's own sync word; only the HAIL is carrier-sensed)

  A→*  SUPE_ANNOUNCE   5+4n B   who I am, what my radio does, at what power
                               └─ once per announce_interval, jittered, and
                                  10 s behind any announce this radio had never
                                  put on air, coalesced (supeAnnSoon)
  A→*  HAIL          10/13 B   "n frames for whoever holds this tag, this long,
                               propose this ceiling; answer me, or hail me back"
                               (+ A's identity, sender_ident)
  B→A  GIMME             9 B   a turnaround later: "here; fly at this budget,
                               this many frames at most" + how the hail landed
   -or- HAVE            11 B   the same terms with B's own train behind them —
                               B's train goes first, A's rides the answering turn
  budget 0:                    the frames, at the hailing rate and the hail's
                               power, handed up as they land — nothing else
  budget ≥1:                   both retune to the confirmed SF; the train,
                               THATSIT (3+n: the train's power, a salt, one
                               CRC-8 per frame), then BYE / RESEND / the
                               answering HAVE and its train, as under a plan

regime 1 — a channel plan (nine 500 kHz channels; the hail's hash seeds two
           slots at +100 and +200..223 ms, each a channel and a derived word,
           the second never on the first's channel)

  A→*  HAIL          10/13 B   as above, and also the SEED
  B→A  GIMME / HAVE            at a slot: B speaks at both, A listens at both —
                               the hailed party speaks first, because the hail
                               asked it a question
  A→B  the frames      × n     at the confirmed budget, flip-gap apart
  A→B  THATSIT         3+n B   the checksum list IS the sequence
  B→A  BYE               1 B   -or- RESEND (one repair round) -or- the
                               answering HAVE (11+⌈n/8⌉: reading, repair mask,
                               B's count and length) with B's train behind it
  the goodbye: the final THATSIT's hash seeds the WIDE schedule — slots from
  +150 ms widening to a 350 ms cap, horizon 3 s, the holder of traffic opening
  with HAVE in its own slot. A pair with steady traffic touches the shared
  channel once, ever.
```

**Nothing on a traffic channel flies blind.** The hailed party speaks first
because it is the one whose presence is in question, and the hailer, having
transmitted the seed, knew the schedule before it finished flying and is on
the channel with its receiver open before there is anything to hear. The
receiver names the budget and a count ceiling before the sender sizes
anything; the sender trims to both.

**A hail that cannot be answered is a hail that is owed** (§12 of the spec):
a hailed party that was busy, or missed both slots, hails back with a count
of zero at its first free moment, and the original hailer — hailed now, and
holding the traffic — answers with HAVE. On the air a hail-back is an
ordinary hail. `SupeOwed` holds one per peer, forgotten one patience after
the hail it answers; `owedDue` skips a peer whose fresh schedule we hold,
since we speak there instead.

### 19.1 What gates it

| Gate | Meaning |
|---|---|
| `CONFIG_LORA_NO_SUPE` | build-time. Set, SUPE is not in the image at all — see §19.1.1 |
| `s.lora.<n>.SUPE.enable` | off by default. Off means the interface's on-air behaviour is exactly what it was |
| `s.lora.<n>.SUPE.afa` | the regime number (§18). Regime 0 answers in place; regime 1 derives schedules |
| no access code | IFAC masks the frame from the flags byte on, so the modem cannot read an address and has nothing to match; `radioStart` says so once |

Each regime version expires on the calendar date the build carries —
`SUPE_EXPIRY_Y/M/D` in `supe.h` (`supeExpired`); past it the node neither
sends nor accepts frames naming it and says so once.

#### 19.1.1 Building without SUPE (`CONFIG_LORA_NO_SUPE`)

A build flavour, not a setting: `spangap build --kconfig CONFIG_LORA_NO_SUPE=y`
leaves SUPE out of the image entirely, so what ships is a plain LoRa interface
carrying Reticulum on the configured channel. A node built this way neither
speaks nor answers SUPE, and its neighbours are unaffected — the protocol is
designed to leave non-participants unmodified and unaware, so a mixed segment
needs nothing.

Five sources leave the build (`esp-idf/CMakeLists.txt`): `supe.cpp`,
`supe_engine.cpp`, `lora_supe.cpp`, and with them `lora_chanplan.cpp` and
`lora_airtime.cpp`, which exist only to serve the schedule SUPE negotiates. Every
call site in the sources that stay is gated on the same symbol, so nothing that
remains references them. What that removes along the way:

- **The channel plan.** One channel — the configured carrier — so the RSSI beat
  reports one, `publishChannels` lists one, and every transmission's airtime
  feeds the APPC band directly instead of a per-channel ledger.
- **Adaptive transmit power, entirely.** Both tiers are fed by frames that state
  the power they went out at, and those frames are SUPE's; nothing survives them
  (§15.4). Every peer is transmitted to at `tx_power`.
- **The settings.** Every SUPE row in `straddle.yaml` carries
  `when_kconfig: "!CONFIG_LORA_NO_SUPE"`, which gates the LCD pane row, the
  browser row and the `storageDefault()` at once — so the keys are **absent**
  from storage rather than present and inert.
- **The console.** No `lora [<n>] supe`, no SUPE line in `lora n`, and the help
  text and `lora a` wording say only what this build does.

### 19.2 Where it lives

Three layers, one direction of dependency:

- **`supe.{h,cpp}` — the pure core.** Regime tables, the §14.3 ladder
  (integer-only, family-filtered, channel-bound; `supeLadder` /
  `supeResolveBudget`), the codec for every frame, the CRC-8 frame checksum,
  the §14.5 sync-word list, the §7 schedule derivation
  (`supeDeriveSchedule`, pure integer arithmetic over the seed's two digests),
  the family-4 listening rule (`supeListenSfLow`), and expiry. Conformance:
  `test/supe-ladder-vectors.txt` over the full §14.3.4 cross-product and
  `test/supe-schedule-vectors.txt` over fixed digest patterns, both
  regenerated by `supe_core_test`; the files are the authority when they and a
  reading of the prose disagree.
- **`supe_engine.{h,cpp}` — the one decider.** The schedule table, the owed
  hails, the slot attendance and the whole meeting state machine, both roles
  and both regimes, single-threaded by contract with no lock and no blocking
  anywhere: a step that must happen later is `host->schedule`d and the entry
  returns. Everything platform arrives through `SupeHost` (time, randomness,
  one-shot timer, SHA, tune/tx/rx/CCA, the train build/fire/deliver hooks,
  peer views in, peer notes out, the channel view); the packet queue it reads
  is `lora_queue`, pure itself. The engine also owns the tag set ("addresses
  that mean us", fed by the observer) and the proof-return table.
  `shouldDetour` is the one deliberately-open policy function
  (`plans/simulation.md` §7); v0 says NOW whenever there is a peer.
- **`lora_supe.cpp` — the boundary.** The recursive mutex every entry point
  takes (radio task, esp_timer task, console, config callbacks — the engine
  itself never locks), the `SupeHost` implementation over
  radio/queue/peers/airtime/chanplan/power, the train buffers (outgoing
  frames held whole to the close for the repair round; inbound frames held
  for in-sequence delivery — `supeTrainCapture` diverts them off the live
  receive path and `bridgeFrameDeliver` replays them through it at the close;
  a hailing-rate dialogue's frames are not captured and go up as they land),
  the ANNOUNCE beat and its peer-table ingest, and the note handlers that file
  the engine's events into `Neighbor` rows (pairs, reports, the run and the
  hold) and the power controller (§15).

Host tests: `make -C esp-idf/test` runs the core checks and regenerates
`golden.txt` + both vector files; `make -C esp-idf/test engine` steps whole
meetings — both regime-0 shapes, the hailed party opening with HAVE, the
hail-back a busy hailed party owes, the run and the hold, patience, the
two-slot schedule, the return leg, a repair round, a hole, the wide ride,
crossed hails, the seed-hash gate — against a stub host.

### 19.3 Frame dispatch

One assumption, stated once: SUPE types are `0xC0`–`0xDF`, never ending in 0
or 1, disjoint from split framing's reachable bytes and from Reticulum flags
on an interface without an access code. `handleRxDone` sorts byte 0 into
framing / SUPE / discard on that rule alone; any change to receive dispatch
preserves it. Assigned densely from the bottom: HAIL `0xC2`, ANNOUNCE `0xC3`,
HAVE `0xC4`, GIMME `0xC5`, THATSIT `0xC6`, BYE `0xC7`, RESEND `0xC8`. HAVE is
GIMME with a train behind it — the same nine bytes, then a count and a length,
then a repair mask when it answers a THATSIT — so one layout is written once.
Inside a full meeting a non-SUPE frame is the train's: `supeTrainCapture`
buffers it (checksummed, counted) instead of the live delivery path, and the
close replays the buffer in sequence through `bridgeFrameDeliver` — split
halves reach rnsd adjacent, a repaired frame in its place. Inside a
hailing-rate dialogue the engine counts the frame and declines it, and the
ordinary path delivers it at once.

### 19.4 The sender path

The classifier (`supeEngVerdict`) runs on the head of the packet queue before
anything contends for the medium, in this order: a packet older than
`SUPE_PATIENCE_MS` → DROP, its own age and nothing else; a live schedule with
its peer, or a hail or meeting with it under way → WAIT; not a SUPE peer →
PLAIN, untouched, exactly as with the feature off; the peer in a hold, or in
the interval after an unanswered hail → WAIT; otherwise OFFER, which arms a
jittered launch. `supePoll` wins the channel through ordinary carrier sense
and `supeEngLaunch` emits the HAIL — building the train first, since the hail
describes it, and holding it until the answer or the run's end. A hail-back
launches the same way, from `SupeOwed`, with a count of zero.

**The run** (§12 of the spec): a hail unanswered — no GIMME or HAVE by its
deadline in regime 0, both slots unmet under a plan — is `SUPE_EV_UNANSWERED`,
which opens the interval (`SUPE_HAIL_INTERVAL_MS` + jitter) in which the
hailed party may hail back; after it the next hail goes out louder, the third
at maximum. **The run drops nothing; patience does.** Every queued packet
carries the moment it was queued and leaves at its own age, so a hail-back
that arrives late finds whatever is still young enough.

**Presence and reachability are two records** (`hPeerNote`). `SUPE_EV_ALIVE` —
heard at all: an ANNOUNCE, a hail to anyone, a meeting frame — refreshes
`supeHeardMs` and clears nothing. `SUPE_EV_ANSWERED` — it answered our hail,
or hailed us — and `SUPE_EV_MET` clear the run. Three unanswered hails make the
peer unreachable: `absentUntilMs` holds it for `SUPE_HOLD_BASE_MS`, doubling
per further unanswered run to `SUPE_HOLD_MAX_MS` — or to
`SUPE_HOLD_PRESENT_MAX_MS` while the peer has been heard within
`SUPE_PRESENT_MS`, since a fault at a peer we can hear is likelier transient.
A held peer's traffic is queued, not refused, and waits out its own patience;
what this costs is that other peers' packets behind it in the FIFO wait too,
bounded by that patience. Under an asymmetric link this is the whole
difference between backing off and hailing for ever.

**Which schedule gets a contested moment: the narrow one.** `slotService` walks
the table twice, narrow before wide (§7 of the spec). **A wide schedule nobody
attends is dropped, not spent**: one that has spoken `SUPE_SCHED_GIVEUP_SPOKE`
times without an answer is freed and the traffic hails instead.

**Crossed hails.** A hail from the peer our own hail is out toward says the
peer did not hear ours — a node that had would be waiting at our slots, not
hailing — so `onHail` drops ours, consumed and unscored, and we speak at
theirs with the train we hold. In regime 0 the peer's hail arriving while ours
awaits its answer is treated as that answer.

**Slot telemetry says how late the window opened**, not just which slot it was:
`listen <hash> slot N t=… (open ±ms)`. Negative is the only good answer; a run
of small positives means the first-slot gap is short for this hardware's cold
retune (`SUPE_NARROW_T0_MS`). `slotsLateOpen` counts them.

**The peer is named by the HAIL, or not at all.** On the hailed side the tag
names one of *our own* addresses, so it says nothing about who is hailing:
`sender_ident` is the only handle. It is what resolves the hailer into a peer
id, and everything that needs to know who the far end is hangs off it — the
hail-back and the return leg above all. Without it the meeting still happens;
neither of those can.

### 19.5 What is learned

Every measurement is a path-loss pair — a level read here against the power
the other side stated (the HAIL and every answer state theirs; a train's rides
its THATSIT, one frame after the fact, so it could be chosen on the report; a
hailing-rate train flies at the hail's) — filed through `SUPE_EV_PAIR`, or
against the link for the one peer that can never be named. `SUPE_EV_REPORT`
carries the peer's account of our own transmission: every GIMME and HAVE
reports how the frame it answers was heard — the hail, our opening HAVE, or
the train — which is the direction we transmit in, filed into `apFileReport`
and consumed by the very next `txp_open` ask. The closing BYE/RESEND is the
arrival proof (`SUPE_EV_TRAIN_OK`); a meeting dying after contact with our
train unconfirmed is `SUPE_EV_TRAIN_LOST`, and only that — missed slots, wide
expiries and hail-backs feed nothing.

### 19.6 Airtime

Meeting airtime is accounted separately from the hailing channel's duty
figure, per channel, in `lora_airtime`'s `ChanLedger` (360 × 10 s buckets):
credited at transmit-done *and on the abort path*, recomputed once per bucket
into a per-channel verdict the transmit path reads without arithmetic. The
speaking side consults it (and the 100 ms reuse gaps) before opening a slot;
hailing-channel frames — every regime-0 dialogue included — feed the APPC
contention band instead. Two budgets, never one (SUPE.md §18: credit a
meeting against the hailing figure and the whole stays-cheap effect vanishes
silently).

**The verdict beat parks when it can't change a verdict.** With no cap to
enforce (regime 0) or an empty agile window, a recompute can only restate
"in budget", so `airtimeRecompute` drops `beatOn` and the beat holds no wake
(`airtimeNextDeadlineMs` → `UINT32_MAX`); the first agile transmit
(`airtimeRecord`, channel ≥ 1) re-arms it. The dialect-expiry re-check rides
`supePoll` passes directly, at most hourly, holding no wake of its own. Net:
SUPE enabled on an idle node costs the announce beat and nothing else; the
engine's `esp_timer` is armed only inside a transaction, so with zero packets
queued it never fires.

### 19.7 Not yet built

The family-4 listening set (SUPE.md §14.6, §15): RadioLib's LR2021 module
exposes no side-detector commands, so a W12 listens at the hailing SF alone
and is treated as any other family until the driver can program
`SetLoraSideDetConfig` and its sync words. `supeListenSfLow` is in the core
and tested; nothing reads it yet.

## 20. Known gaps in the LCD viewer

The browser LoRaMon carries the current feature set; the LCD app lags it in
three places. All are deliberate deferrals rather than oversights, recorded here
so the intent survives.

**The two wait marks are browser-only.** `own_ms` (§12) is drawn dotted beside
the solid contention run in the browser; the LCD reads the older six-field form
and draws contention alone. Harmless — the field is appended, so an indexing
parser ignores it — but the LCD therefore cannot distinguish a busy channel from
a busy radio.

**Peer pills still label runs, not moments.** The browser anchors a pill to the
instant a name becomes true and takes a detour channel's name off the dwell, so
a silent slot is still named (§ Pills). The LCD keeps the older rule — one label
per run of same-tag frames, widest four drawn — because its label pool is fixed
and its canvas has no glyph blitter, so "one per attribution change" needs a
different placement pass rather than the same code. Consequence: on a detour
lane the LCD names a slot only if something arrived in it.
