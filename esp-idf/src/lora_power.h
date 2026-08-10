#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

/* ── adaptive TX power (s.lora.<i>.SUPE.adaptive_txpower, default on) ──
 * The first slice of plans/adaptive-power.md: one power determination per
 * neighbour node, then every frame whose first RF hop is that node goes out at
 * it. There is no control loop yet — no walk-down, no failure recovery, no
 * re-measurement. One number per node, settled once, applied from then on.
 *
 * Getting the number. Nothing is initiated: the estimate is the reciprocity
 * figure of §13.1 plus AP_EST_MARGIN_DB. The margin exists because that
 * estimate credits an unmeasured peer with s.lora.assumed_peer_txp rather than
 * knowing its power — and because ambient noise is NOT reciprocal even where
 * path loss is. A node sitting beside an interferer needs more from us than our
 * own quiet receiver would suggest, and no measurement we can make from here
 * will say so.
 *
 * A power sweep used to supply a measured figure instead. It is gone, and SUPE
 * replaces it with something better: every frame a detour sends states the
 * power it went out at, so ordinary traffic yields a path loss continuously and
 * at two configurations rather than once every half hour (plans/SUPE.md §7).
 *
 * The determination is per NODE, keyed through the neighbour table's identity
 * clustering, so it covers every hash that node owns — including hashes learned
 * later by announce.
 *
 * Never on a broadcast: an announce has no single next hop and must reach
 * everyone, so it always goes out at the configured tx_power. */
#define AP_EST_MARGIN_DB      5      /* added to the estimate when nothing measured */

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
 * Honouring a request is gated on SUPE.adaptive_txpower: obeying one puts
 * our transmit power under someone else's control, observably, so a node with
 * the key off must stay at its configured power or the opt-out isn't one.
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
#define AP_MIN_SAMPLES        3      /* recent frames before we dial a peer down */

#define AP_FLOOR_DECAY_MS (10u * 60u * 1000u)  /* the failure floor's decay */
#define AP_FLOOR_DBM      (-9)   /* the low end adaptive power may ask for;
                                  * chips clamp up from it */

/* ─────────────── lora_power: adaptive transmit power ─────────────── */
const uint8_t* apNextHop4(LoraRadio* r, const uint8_t* pkt, size_t len);
int8_t apTxPower(LoraRadio* r, const uint8_t* pkt, size_t len);
void   apApplyPower(LoraRadio* r, int8_t txp);
bool   apPwrReqFor(LoraRadio* r, const uint8_t* pkt, size_t len, int8_t* out);
int8_t apOpenPower(LoraRadio* r, Neighbor* e);
void   supeApFailed(LoraRadio* r, Neighbor* e, int8_t triedDbm);
void   supeApSucceeded(LoraRadio* r, Neighbor* e);
void   supeFilePair(LoraRadio* r, Neighbor* e, int16_t rssi, int8_t peerTxp,
                    uint8_t step);
