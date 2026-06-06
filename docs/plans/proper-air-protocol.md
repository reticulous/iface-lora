# A proper air protocol for iface-lora (SX1262)

> Status: **design notes / plan**, not implemented. Captures what the SX1262 +
> Reticulum substrate *allows* so a more involved air protocol — a robust common
> channel plus per-pair negotiated high-rate private links — can be built later.
> Counterpart to `iface-espnow`'s `docs/plans/proper-air-protocol.md`; cross-refer
> for the shared rnsd-oblivious layering and link-adaptation reasoning. Nothing
> here is wired up yet.

## 1. Why the SX1262 world is different from ESP-NOW

ESP-NOW rides 802.11: the PHY rate is signalled in the preamble and **auto-detected
per frame**, so a receiver hears all rates at once. **LoRa is the opposite.**
Demodulation requires the receiver to be **pre-tuned to one exact modem config** —
SF, BW, CR, preamble length, sync word, *and* center frequency. The SX1262 has a
**single demodulator**: it listens for exactly one `(SF, BW, freq)` at a time.

Consequences that shape everything below:

- **SF is effectively channelization, not in-band signalling.** Two nodes on
  different SF **cannot hear each other at all** — not slower, *silent*. No "energy
  present", no fallback.
- **No multi-SF simultaneous RX on this part.** That is a *gateway/concentrator*
  feature (SX1301/SX1302 have 8+ parallel demodulators). A T-Deck's SX1262
  transceiver can't, so don't design around it.
- **You own the MAC.** No association, no CSMA forced on you — you can schedule
  transmissions deterministically (see §5), which is a capability ESP-NOW's CSMA
  denies.
- **The SX1262 reports RSSI *and* SNR per received packet** (`GetPacketStatus`:
  `RssiPkt`/`SnrPkt`/`SignalRssiPkt`). SNR is the ideal LoRa link metric — LoRa
  decodes below the noise floor, so SNR tells you how much SF headroom a link has.
  This is the raw material for per-link rate choice (§4).

## 2. The model: robust common channel + per-pair private links

**Not** a network-wide "everybody switch to SF7 now" hop (one collision domain,
everyone must follow, single point of failure). Instead, the better model — closer
to **Bluetooth** (page on known params → private negotiated link between two
devices) or cellular (common control channel → dedicated negotiated resources):

- **Common channel.** One **intermediate, most-robust-feasible** config that *every*
  node camps on a priori (provisioned constant — no chicken-and-egg, because it's
  fixed/known). Carries announces, path discovery, small/control traffic, and the
  link-setup negotiation. This *is* the RNS broadcast interface.
- **Per-pair private links.** When two nodes have enough to exchange, they
  bilaterally negotiate their own fast config and move off the common channel for a
  bounded burst, then return. Only the pair coordinates — the rest of the network is
  undisturbed.

This turns a single shared collision domain into **many parallel point-to-point
links** (§4) — the aggregate-capacity win.

## 3. The "X and Y are gone for ~x ms" reservation

Before a pair leaves, they announce their absence on the common channel. Get the
semantics right — it is **weaker than a channel lock, on purpose**:

- It's a **virtual-carrier-sense / NAV-style hint**: "X and Y won't be on the common
  channel for ~x ms, so **queue anything destined for X or Y**." It is **not**
  "everyone go silent" — other pairs among the still-present nodes keep using the
  common channel freely. Don't let it drift into a full channel reservation; that
  would waste the shared medium.
- It's a **soft optimization, not a guarantee**: (a) **hidden nodes** out of range of
  the announcement may still send to X/Y and get nothing; (b) a node that misses the
  announcement won't hold. That's fine — same principle as the ESP-NOW doc: *the
  reservation is a hint; RNS higher-layer reliability + retry are ground truth.* It
  reduces wasted transmissions, it doesn't eliminate them.
- **Bound it.** The announced window is a **deadline**, not a promise to use it all —
  finish early, return early; hit the bound, return and re-reserve if still needed.
  Others' "hold" timers key off `announced_window + margin`.

## 4. The private bilateral link

The part that changes network capacity, not just one link's speed:

- **Frequency independence ⇒ spectral/code reuse.** X and Y need not share frequency
  with the network *or with other pairs*. Multiple pairs burst **simultaneously** on
  different frequencies (fully orthogonal) — or same freq / different SF (LoRa SFs
  are quasi-orthogonal). Parallel links → aggregate throughput scales with concurrent
  pairs, not capped by one channel.
- **Per-link, measurement-driven parameters.** During the brief common-channel
  negotiation each side reads its RSSI/SNR (§1) and they pick `SF/BW/CR/freq` matched
  to *that* link's budget — including **asymmetric** configs (X→Y at SF7, Y→X at SF9
  if the reverse path is worse), since each direction's SNR is measured
  independently. This is the realization of the ESP-NOW reception-report/asymmetry
  idea, but cleaner because it's a 2-party agreement.
- **Private sync word** on the burst keeps it from being confused with common-channel
  or other pairs' traffic.

## 5. Deterministic timing — a LoRa superpower (recall ESP-NOW couldn't)

On ESP-NOW you can't know *a priori* when a packet goes out (CSMA backoff/retry).
On LoRa **you can**, because airtime is exactly computable from SF/BW/CR/preamble/
payload and you own the schedule. So:

- A scheduled transmission goes out **at its slot, by construction** — the "when did
  it actually go out" question (open on ESP-NOW) **dissolves**: scheduled = known,
  and TxDone/RxDone just confirm it.
- **One transmission can anchor a whole timeline** ("everyone times along"): the
  beacon's **RxDone** is a shared reference instant; a pre-agreed schedule unfolds
  from it (beacon-synchronized TDMA — the LoRaWAN Class B / 802.15.4e-TSCH model).
  Within a **pair** this is trivial (2 nodes, one RxDone anchor — no Class-B
  machinery needed); the full network-wide version is available if ever wanted.

Error budget for any "shared t=0" (increasing importance):
1. **Timestamp jitter** — read RxDone in the ISR after SPI status readback → µs-to-
   tens-of-µs; **hardware-capture the DIO edge** (ESP32-S3 GPTimer + ETM / input
   capture) → sub-µs. Capture the edge for tight slots.
2. **Propagation** — ~3.3 µs/km; single-digit µs at range, usually negligible vs
   guards.
3. **Clock drift between syncs — dominant.** ±10–20 ppm XTAL → ~20 µs/s, ~1.2 ms/min
   of skew. **This sets re-sync interval and guard size.** A TCXO cuts this sharply —
   the part's clock spec drives the whole budget.

Guard intervals absorb `drift-since-sync + jitter + propagation + TX↔RX turnaround +
retune`. SX1262 mode transitions are tens of µs to ~100 µs; PLL settle on a freq
change is a few hundred µs — all deterministic, so budget them exactly. Disciplining
the local clock to the sync cadence (PI-correct the timer rate) stretches how long a
schedule stays valid between syncs.

## 6. Realities to budget

- **Half-duplex deafness.** While on the private link, both nodes are deaf to the
  common channel — exactly what §3's reservation manages. But a node must not vanish
  so long it **misses announces or starves its other neighbors/links**: cap the
  per-session window and the off-channel duty fraction (fairness).
- **Amortization threshold.** The negotiation handshake costs airtime on the *slow*
  common channel. Only go bilateral when the queued burst is big enough to pay back
  the slow setup; small transfers ride the common channel. (Same broadcast-for-small
  / private-link-for-bulk logic as ESP-NOW's unicast threshold.)
- **Return / timeout.** If a side crashes or the burst overruns, both fall back to the
  common channel after a timeout and re-sync there.
- **Regulatory is *friendlier* for the burst.** Fast config (high BW / low SF) →
  short airtime → easy on **dwell-time** limits (e.g. US ~400 ms); spreading pairs
  across sub-bands also spreads the **EU 1% duty-cycle** budget. It's the *slow common
  channel* you must watch — a too-robust (high-SF) common config can blow dwell time
  (e.g. SF12/BW125 4-byte packet ≈ 800 ms > US 400 ms). Pick the common config to
  balance robustness against airtime/dwell; SF10/BW125 or SF9/BW250 is far more
  practical than SF12.
- **CAD for idle listening.** The SX1262's Channel Activity Detection cheaply (low
  power) detects a preamble at the current config without a full RX — good for
  camping on the common channel between bursts.

## 7. Staying rnsd-oblivious (same abstraction as iface-espnow)

The interface presents a "broadcast medium" to rnsd; underneath it:

1. buffers outbound packets whose next hop is Y,
2. when enough accumulate (the §6 threshold), negotiates a private link with Y on the
   common channel and announces the §3 absence window,
3. retunes, drains the buffer over the fast link, returns to the common channel,
4. rnsd just sees the packets delivered.

rnsd never knows a private high-rate link existed — exactly as it never knows about
MAC-unicast on ESP-NOW. The common channel *is* the RNS broadcast interface
(announces, path discovery, control); the private links are ephemeral fast pipes for
bulk next-hop delivery. Hop-count / announce / path-building all happen on the common
channel and read straight from the RNS header — see iface-espnow's doc §5–6, which
applies unchanged here.

## 8. Open items to nail down before building

- **Region target** — drives dwell-time (US ~400 ms) and duty-cycle (EU 1%) limits,
  which bound both the common config and the burst configs. Fill in the airtime/duty
  math once chosen.
- **Clock spec — TCXO vs plain XTAL.** The ±ppm number drives the §5 drift/guard
  budget directly; decide before sizing slots.
- **Negotiation + reservation wire format** — link-setup handshake on the common
  channel (proposed/confirmed `SF/BW/CR/freq`, per-direction if asymmetric, session
  duration, sync word, timing anchor) and the NAV-style absence announcement.
- **Amortization-threshold math** — minimum burst size that pays back the slow
  negotiation, as a function of common-channel rate vs burst rate.
- **SX1262 specifics** — DIO IRQ timing (TxDone/RxDone), hardware DIO-edge capture on
  ESP32-S3 for sub-µs anchors, CAD parameters, `GetPacketStatus` SNR/RSSI plumbing,
  mode-transition + PLL-settle timings.
- **Fairness policy** — max off-channel window and duty fraction so a node doesn't
  miss announces or starve other links.
- **Interaction with current `lora.cpp` / `esp_idf_hal.cpp`** and the shared-radio /
  region config already there.

---

*Captured from a design discussion; verify SX1262 register/IRQ behaviour against the
datasheet + the `lora` HAL, and the RNS header offsets against the `rns` straddle,
before implementing.*
