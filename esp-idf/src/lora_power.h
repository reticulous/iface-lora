#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

/* ── adaptive TX power ──
 * Every frame whose first RF hop is a known node goes out at a power derived
 * for that node, at the configuration the frame is about to fly at. Nothing is
 * stored as a power: the number is resolved per frame from whatever evidence
 * is currently fresh, so a link that changes is followed rather than remembered
 * wrong.
 *
 * **The evidence is a path loss, and the configuration supplies the floor.**
 * A measurement is a level in dBm and the power that produced it; the
 * difference is the loss, which is one property of the link however it was
 * read. What a configuration changes is the sensitivity that loss has to
 * clear. So:
 *
 *     need = path loss + sensitivity(cfg) + margin
 *
 * and one measurement serves the hailing channel and every detour step alike.
 * A power that ignored the configuration would be tuned for one of them and
 * wrong for the other by the 15–20 dB that separates SF12/125k from SF5/500k.
 *
 * **Four tiers, best evidence first** (ApSource, in lora_peers.h):
 *
 *   REPORT  the peer stated the level our own frame landed at, in the MANIFEST
 *           that closes a detour. The only measurement of the us→them
 *           direction there is. Margin: SUPE_TARGET_MARGIN_DB.
 *   PAIR    a frame heard here with the power the peer stated for it. The loss
 *           is reciprocal, so it costs a further AP_RECIP_MARGIN_DB: ambient
 *           noise is not reciprocal even where path loss is, and a node sitting
 *           beside an interferer needs more from us than our own quiet receiver
 *           would suggest.
 *   EST     the same, against s.lora.assumed_peer_txp instead of a stated
 *           power — everything the peer never told us, plus a guess about the
 *           one thing that would have made it a measurement.
 *   NONE    nothing recent: the configured tx_power.
 *
 * Every tier is gated on AP_FRESH_MS. Stale evidence falls to the next tier,
 * and a node we have not heard from at all opens at the configured power.
 *
 * **The ratchet.** Up is immediate and large, down is slow and paid for. A
 * miss — no reverse MANIFEST, or a delivery proof that never came from a peer
 * that has also gone quiet — files a floor AP_FLOOR_STEP_DB above what was
 * tried, on a decay, so the loop settles above the cliff instead of oscillating
 * across it. A success moves the trim one dB, and only after AP_MIN_SAMPLES
 * clean exchanges: a controller that dials down on a timer walks a quiet link
 * into the ground.
 *
 * **A missing delivery signal from a peer we can still hear is not a power
 * failure.** It is a congested medium, a far end that is busy, or a transfer
 * stalled somewhere above the radio, and every one of those is made worse by
 * transmitting harder. The evidence that distinguishes them is already on the
 * row: a peer heard within AP_MISS_QUIET_MS is reachable, whatever else went
 * wrong, so the miss scores against link quality and leaves the power alone.
 *
 * **Nor is a miss the link itself reported hearing well.** A meeting that ends
 * unconfirmed raises the power only when the power could plausibly have been
 * the cause, and the peer's own REPORT of our frames is what settles that: put
 * the power that just failed through the path loss it measured, at the
 * configuration that failed, and compare with what that configuration needs. At
 * AP_MISS_MARGIN_DB or better the frame was lost to something else — a lost
 * turnaround, a modulation at its edge, a receiver that was elsewhere — and
 * raising is not merely useless but expensive: each miss files a floor six dB
 * up for a decay's length, so a handful of them walk a link with thirty dB of
 * margin to maximum and hold it there. Measured margin outranks the fact of a
 * miss, exactly as a measurement outranks a model everywhere else here.
 *
 * **The EST tier is deliberately timid**, and it is the only tier
 * `s.lora.<i>.adaptive_txpwr` governs. It is also the only tier a node that
 * does not speak our air protocol can ever reach, so there is no return
 * measurement to catch it being wrong and nothing but Reticulum's own delivery
 * proofs to notice. It therefore claims only half the surplus it thinks it
 * sees, caps the total cut, and — rather than opening at the estimate — treats
 * it as a target it has to walk to, one dB per AP_EST_WALK_FRAMES frames heard
 * from that peer.
 *
 * **The walk is paid for in frames heard, not in clean exchanges**, and that is
 * what makes the tier reachable at all. The ratchet's currency is a returned
 * delivery signal, which a peer that speaks only Reticulum to us produces
 * rarely — nothing inside an established link elicits a proof — so a walk
 * bounded by the ratchet would never leave zero and the tier would resolve to
 * the configured power forever. Frames heard are the same evidence the estimate
 * itself is built from, they arrive whenever the peer is talking to us at all,
 * and they stop arriving exactly when the link is in trouble. The walk holds
 * still while a failure floor stands, and halves on a miss rather than zeroing:
 * a floor decays, and a walk that came back to the same cut the moment it did
 * would oscillate on the floor's own period.
 * Everything above it is part of SUPE's own operation and is not switchable:
 * the protocol states a power in every frame it sends precisely so that both
 * ends can do this.
 *
 * Never on a broadcast: an announce has no single next hop and must reach
 * everyone, so it always goes out at the configured tx_power. */
#define AP_FRESH_MS        (10u * 60u * 1000u)  /* evidence older than this is
                                                 * not evidence */
#define AP_RECIP_MARGIN_DB    5      /* added when the loss is reciprocal */
#define AP_EST_MARGIN_DB      5      /* added again when the peer's power is a guess */
#define AP_TRIM_MAX_DB        6      /* how far the ratchet may trim a MEASURED need */
#define AP_PASSIVE_MIN_SAMPLES 5     /* recent frames before the EST tier will move */
#define AP_PASSIVE_CUT_MAX_DB  10    /* the EST tier's total cut, proofs returning */
#define AP_PASSIVE_CUT_BLIND_DB 6    /* and when nothing has ever come back */
#define AP_MISS_MARGIN_DB     10  /* a miss with at least this much margin, by
                                   * the peer's OWN report of our frames, is not
                                   * a power failure and must not raise it */
#define AP_EST_WALK_FRAMES    8      /* frames heard from a peer per dB the EST
                                      * tier walks down */
#define AP_MISS_QUIET_MS  (30u * 1000u) /* a delivery signal that never came is
                                      * evidence about POWER only if the peer
                                      * has also stopped being heard for this
                                      * long; sooner than that it is congestion,
                                      * which more power does not fix */
#define AP_FLOOR_STEP_DB      6      /* how far above a miss the floor lands */
#define AP_EST_BUCKETS        3      /* newest bucket-ring slots the estimate
                                      * averages: AP_FRESH_MS, quantised up */

/* ── 0x04: the power request ──
 * A 4-byte frame sent BACK TO BACK in front of the packet it relates to, in the
 * normal modem regime (sync 0x42, explicit header, preamble 12) so it needs no
 * reconfigure between the two. It carries a TX power we suggest the peer use,
 * and/or our measurement of the peer's last frame:
 *
 *   [0x04][suggested txp, int8 dBm][rssi, encoded][snr, encoded]
 *
 * The two payload fields map onto the two knowledge states, which is why either
 * may be absent: if we know who the peer is we hold its history and can compute
 * the answer, so we send a suggestion; if we don't, we can still report what we
 * measured, and the peer closes the loop itself because it knows what power it
 * transmitted at. Sentinels carry the "and/or", so no flags byte is needed.
 *
 * **Absence of the frame means "use your maximum."** That is the whole fallback,
 * and it needs no constant agreed between the ends — so recovery from a bad
 * suggestion is simply to stop sending one, and a peer that cannot comply with a
 * request just clamps at its own ceiling with nothing to signal.
 *
 * An explicit request outranks our own reciprocity estimate: the receiver is the
 * authority on its own reception, having folded in its noise floor, antenna and
 * sensitivity, none of which a transmitter can see.
 *
 * The frame only ever goes to a node that has spoken our air protocol to us,
 * which is what makes it part of that protocol rather than a switchable
 * courtesy: `adaptive_txpwr` does not reach it in either direction. To anyone
 * else it would be 35 ms of unparseable noise on a shared channel.
 *
 * Nothing under 20 B on air can be an RNS packet (HEADER_MINSIZE is 19 and we
 * add a framing byte), which is how our own air frames discriminate; a 4-byte
 * frame is unambiguous on length alone.
 *
 * Design and the reasoning behind each choice: plans/adaptive-power.md §3a. */
#define LORA_MAGIC_PWRREQ     0x04
#define PWRREQ_LEN            4
#define PWRREQ_NO_TXP         ((int8_t)0x7F)   /* "no suggestion" sentinel */
/* The per-radio antenna-dBm ceiling is r->maxTxDbm (22 bare chip, 27 through
 * a FEM — set by femInit); asking a peer for it is what sending nothing
 * already means. */
#define AP_MIN_SAMPLES        3      /* recent frames before we dial a peer down,
                                      * and clean exchanges per ratchet notch */

#define AP_FLOOR_DECAY_MS (10u * 60u * 1000u)  /* the failure floor's decay */
#define AP_FLOOR_DBM      (-9)   /* the low end adaptive power may ask for;
                                  * chips clamp up from it */

/* ─────────────── lora_power: adaptive transmit power ─────────────── */
const uint8_t* apNextHop4(LoraRadio* r, const uint8_t* pkt, size_t len);
int8_t apTxPower(LoraRadio* r, const uint8_t* pkt, size_t len);
void   apApplyPower(LoraRadio* r, int8_t txp);
bool   apPwrReqFor(LoraRadio* r, const uint8_t* pkt, size_t len, int8_t* out);
/* The opening power toward a node at the hailing configuration. */
int8_t apOpenPower(LoraRadio* r, Neighbor* e);
/* The ratchet, from either feedback path: a detour's reverse MANIFEST or a
 * delivery proof on plain traffic. */
void   apFailed(LoraRadio* r, Neighbor* e, int8_t triedDbm, const SupeCfg* cfg);
void   apSucceeded(LoraRadio* r, Neighbor* e);
/* The EST tier's own clock: one frame heard from this node. */
void   apHeard(Neighbor* e, uint32_t now);
#if !defined(CONFIG_LORA_NO_SUPE)
/* The same, resolved where the frame is about to fly — a granted detour step. */
int8_t apOpenPowerAt(LoraRadio* r, Neighbor* e, const SupeCfg* cfg);
void   supeFilePair(LoraRadio* r, Neighbor* e, int16_t rssi, int8_t peerTxp,
                    uint8_t step);
/* The peer's account of our own frame: what it read, at what we sent. */
void   apFileReport(LoraRadio* r, Neighbor* e, int16_t rssi, int8_t ourTxp);
#endif  /* CONFIG_LORA_NO_SUPE */
