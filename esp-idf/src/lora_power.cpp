/**
 * lora_power — adaptive transmit power: the per-frame power decision on the
 * transmit path, the reciprocity estimate that settles a node's determination,
 * the 0x04 power request, and SUPE's derived-power control loop (§15).
 */
#include "lora_priv.h"
#include "lora_fem.h"

#if defined(CONFIG_LORA0_CS_PIN)

static void apSettle(LoraRadio* r, Neighbor* e);

/* ── adaptive TX power: the tx-path half ──
 * (overview at AP_EST_MARGIN_DB, near the top of the file) */
static int8_t apClamp(LoraRadio* r, int want);
bool peersEstimateCliff10(const LoraRadio* r, const Neighbor* e,
                               uint32_t now, int* cliff10, uint32_t* samples);

/* The first-4 naming the node an outbound frame's FIRST RF HOP goes to, or null
 * when the frame has no single next hop we can name. Power is a property of
 * that one hop; a proof is end-to-end, so a multi-hop destination says nothing
 * about the power the hop in front of us needs. */
const uint8_t* apNextHop4(LoraRadio* r, const uint8_t* pkt, size_t len) {
    NeiState* st = r->nei;
    if (!st) return nullptr;
    RnsHdr h;
    if (!rnsParse(pkt, len, &h)) return nullptr;
    /* Announces first, and that order matters: a rebroadcast announce is
     * HEADER_2, but its transport_id is the REBROADCASTER's own identity (that
     * is how path tables learn first_hop), not a next hop. It is a broadcast
     * either way and must reach everyone. */
    if (h.ptype == NEI_PT_ANNOUNCE) return nullptr;
    if (h.hdr2) return h.transportId;                 /* relayed: the next hop names itself */
    if (h.ptype == NEI_PT_PROOF || h.dtype == NEI_DT_LINK) {
        /* Link traffic and link proofs are addressed to the link_id, so the
         * peer is the link's destination — and only when that destination is
         * someone else. An inbound dial records no hash for its initiator, so
         * a link they opened to us resolves to nothing. A delivery proof is
         * addressed to a packet hash and simply misses the link table. */
        NeiLink* L = peersLinkFind(st, h.dest);
        if (!L || !L->haveDest || peersDestIsLocal(st, L->dest)) return nullptr;
        return L->dest;
    }
    if (h.dtype == NEI_DT_SINGLE) return h.dest;
    return nullptr;
}

/* Is this outbound frame addressed to a link, and did that link's peer request a
 * power for it? A 0x04 request outranks anything we could work out ourselves:
 * the receiver folded in its own noise floor, antenna and sensitivity, none of
 * which a transmitter can see. Covers the LRPROOF and every later frame of the
 * session alike, since all of them are addressed to the link_id. */
static bool apLinkSuggest(LoraRadio* r, const RnsHdr* h, int8_t* out) {
    if (h->hdr2 || (h->ptype != NEI_PT_PROOF && h->dtype != NEI_DT_LINK)) return false;
    NeiLink* L = peersLinkFind(r->nei, h->dest);
    if (!L || !L->haveSuggest) return false;
    *out = L->suggestDbm;
    return true;
}

/* The power this frame goes out at: a peer's explicit request first, then the
 * next-hop node's own determination, else the configured tx_power. */
int8_t apTxPower(LoraRadio* r, const uint8_t* pkt, size_t len) {
    if (!r->adaptive || !r->nei) return r->cfgTxp;
    RnsHdr h;
    if (rnsParse(pkt, len, &h) && h.ptype != NEI_PT_ANNOUNCE) {
        int8_t want;
        if (apLinkSuggest(r, &h, &want)) return apClamp(r, want);
    }
    const uint8_t* nh = apNextHop4(r, pkt, len);
    if (!nh) return r->cfgTxp;
    Neighbor* e = peersFindBy4(r->nei, nh);
    if (!e || peersIsLocal(e)) return r->cfgTxp;

    /* A filed path-loss pair beats a reciprocity estimate every time: it is a
     * measurement of this link at a stated power, where the estimate is an
     * assumption about a power the peer never gave us. A peer we have exchanged
     * SUPE frames with has one, so a plain unicast frame to it goes out at the
     * same derived power a detour would use, resolved at the hailing
     * configuration because that is where this frame is about to fly. */
    if (e->havePair) return apOpenPower(r, e);
    /* Otherwise the reciprocity estimate, settled on first use. Settling lazily
     * rather than on a timer is what keeps it honest: the estimate is only
     * worth taking once there is signal history to take it from, and the first
     * frame we actually want to send to a node is exactly when to ask. */
    if (!e->haveApPwr) apSettle(r, e);
    return e->haveApPwr ? e->apPwr : r->cfgTxp;
}

/* Put the chip on `txp`. This is the only place the tx path moves the power
 * register, and txPwrNow is what the radio is currently set to as well as what
 * the LoRaMon record is stamped with, so the two can't drift — a frame to a
 * quiet neighbour must not leave the next frame transmitting at its power while
 * being recorded at another. */
void apApplyPower(LoraRadio* r, int8_t txp) {
    if (txp == r->txPwrNow) return;
    int16_t st = r->radio->setOutputPower(femChipDbm(r, txp));
    if (st != RADIOLIB_ERR_NONE) {
        warn("lora/%d setOutputPower(%d): %s (%d)",
             r->idx, (int)txp, rlErrName(st), (int)st);
        return;
    }
    r->txPwrNow = txp;
}

/* Should this outbound packet carry a power request, and what should it ask for?
 * Only a link opener so far — an LR is the one unicast frame where we chose the
 * destination, so we hold its history, and where one 4-byte prefix covers a
 * whole session's traffic coming back rather than a single reply.
 *
 * The number is what the peer needs to reach US, and that is a DIRECT
 * measurement rather than a reciprocal one: peersEstimateCliff10 is their assumed
 * power minus the headroom their frames actually arrived with at our receiver,
 * so it says how much of their power was surplus here. Its one assumption is the
 * power they transmitted at — which "no prefix means maximum" makes true.
 *
 * Nothing is sent when we would ask for a power no radio can exceed: absence
 * already means maximum, so the frame would be 35 ms saying nothing.
 *
 * Every gate names itself under `log lora debug`. Five conditions have to line
 * up before a request goes out, and a silently absent prefix is otherwise
 * indistinguishable from a broken one. */
bool apPwrReqFor(LoraRadio* r, const uint8_t* pkt, size_t len, int8_t* out) {
    const char* why = nullptr;
    int         val = 0;
    RnsHdr      h;

    if (!r->nei) return false;
    if (!rnsParse(pkt, len, &h)) return false;
    if (h.hdr2 || h.ptype != NEI_PT_LINKREQ || h.dtype != NEI_DT_SINGLE || h.hops != 0)
        return false;                       /* not a link opener — say nothing */

    int      cliff10 = 0;
    uint32_t samples = 0;
    Neighbor* e = peersFindBy4(r->nei, h.dest);
    if (!r->adaptive)                     why = "SUPE.adaptive_txpower off";
    else if (!e || peersIsLocal(e))         why = "dest hash is on no node row";
    /* Only a node that has spoken our air protocol to us will parse the frame;
     * to anyone else it is 35 ms of unparseable noise on a shared channel. That
     * is the RF_PROTO_NAME tag in `lora n`, set by a SUPE announcement, which
     * is what bootstraps eligibility. */
    else if (!e->ourProto)                why = "node has not spoken our protocol";
    else if (!peersEstimateCliff10(r, e, millis(), &cliff10, &samples))
                                          why = "no recent signal to estimate from";
    /* One frame's RSSI moves several dB, so a single sample has no business
     * dialling anyone down. */
    else if (samples < AP_MIN_SAMPLES)   { why = "too few samples"; val = (int)samples; }

    int want = 0;
    if (!why) {
        want = (cliff10 >= 0 ? cliff10 / 10 : (cliff10 - 9) / 10) + AP_EST_MARGIN_DB;
        if (want >= r->maxTxDbm)         { why = "would ask for max anyway"; val = want; }
        if (want < AP_FLOOR_DBM) want = AP_FLOOR_DBM;
    }

    if (logIsDebug("lora")) {
        char what[56];
        if (!why)     snprintf(what, sizeof what, "asking for %d dBm", want);
        else if (val) snprintf(what, sizeof what, "%s (%d)", why, val);
        else          safeStrncpy(what, why, sizeof what);
        dbg("lora/%d pwr-req to %02x%02x%02x%02x: %s",
            r->idx, h.dest[0], h.dest[1], h.dest[2], h.dest[3], what);
    }
    if (why) return false;
    *out = (int8_t)want;
    return true;
}

/* ── adaptive TX power: settling one node's determination ──
 * (overview at AP_EST_MARGIN_DB, near the top of the file) */
bool peersEstimateCliff10(const LoraRadio* r, const Neighbor* e,
                               uint32_t now, int* cliff10, uint32_t* samples);
bool peersNodeFirst4(const Neighbor* e, uint8_t out[4]);

/* Clamp a power to what this radio may transmit at: the chip's range, and
 * never above the configured tx_power. A measured rung already satisfies both
 * (the ladder climbs to that same ceiling); the estimate-plus-margin path is
 * what needs the clamp. */
static int8_t apClamp(LoraRadio* r, int want) {
    if (want > r->cfgTxp)    want = r->cfgTxp;
    if (want < AP_FLOOR_DBM) want = AP_FLOOR_DBM;
    int8_t clipped = (int8_t)want;
    r->radio->checkOutputPower((int8_t)want, &clipped);
    return clipped;
}

/* Record the one determination for a node. There is no measured source any
 * more — the power sweep that produced one is gone, and SUPE measures the same
 * quantity continuously instead, every detour stating the power of every frame
 * it sends. What is left is the reciprocity estimate plus AP_EST_MARGIN_DB,
 * because it assumes a power the peer never stated and because noise is not
 * reciprocal even where path loss is. A node with no estimate either — nothing
 * heard from it inside the bucket ring's hour — keeps the configured power.
 *
 * **It takes AP_MIN_SAMPLES before it will settle anything.** One frame's RSSI
 * moves several dB, and this determination is made once and kept: an estimate
 * taken off a single frame would pin a neighbour's transmit power for as long
 * as the table holds the row, with nothing left to re-measure it. The 0x04
 * power request has always required the same evidence before dialling a peer
 * down; this is the same rule applied to dialling ourselves down, which matters
 * more now that it is the only passive source and on by default. */
static void apSettle(LoraRadio* r, Neighbor* e) {
    if (!r->adaptive || !e || peersIsLocal(e) || e->haveApPwr) return;
    int      est10   = 0;
    uint32_t samples = 0;
    if (!peersEstimateCliff10(r, e, millis(), &est10, &samples)) return;
    if (samples < AP_MIN_SAMPLES) return;
    int pwr = (est10 >= 0 ? est10 / 10 : (est10 - 9) / 10) + AP_EST_MARGIN_DB;
    e->apPwr     = apClamp(r, pwr);
    e->apFromEst = true;
    e->haveApPwr = true;
    uint8_t h4[4] = {};
    peersNodeFirst4(e, h4);
    info("lora/%d adaptive: %02x%02x%02x%02x settled at %d dBm "
         "(estimate + margin, %u samples)",
         r->idx, h4[0], h4[1], h4[2], h4[3], (int)e->apPwr, (unsigned)samples);
}

/* ─────────────── adaptive transmit power (§15) ─────────────── */

/* The power the controller opens toward a peer at (SUPE.md §15): derived
 * from a learned offset, never computed as an absolute from a path loss and a
 * modelled sensitivity —
 *
 *     power = clamp( maximum − offset , floor , maximum )
 *
 * The offset starts at zero and only ever moves on evidence about this peer;
 * the floor below remembers where it broke, and decays. Never above the
 * configured tx_power, under any failure, for any reason. */
int8_t apOpenPower(LoraRadio* r, Neighbor* e) {
    if (!r->supeAdaptive || !e) return r->cfgTxp;
    int want = (int)r->cfgTxp - (int)e->apOffsetDb;
    if (e->haveApFloor && want < e->apFloorDbm) want = e->apFloorDbm;
    return apClamp(r, want);
}

/* A miss raises the offset fast — being wrong downward costs connectivity, so
 * recovery is immediate and large — and remembers where it broke, on a floor
 * that decays, so the loop settles above the cliff instead of oscillating
 * across it. */
void supeApFailed(LoraRadio* r, Neighbor* e, int8_t triedDbm) {
    if (!r->supeAdaptive || !e) return;
    e->apOffsetDb = e->apOffsetDb > 6 ? (int8_t)(e->apOffsetDb - 6) : 0;
    e->apFloorDbm = (int8_t)(triedDbm + 3);
    e->haveApFloor = true;
    e->apFloorDecayMs = millis() + AP_FLOOR_DECAY_MS;
    e->apSuccess = 0;
    if (logIsDebug(TAG))
        dbg("lora/%d supe: power back up (offset %d dB, floor %d dBm after a miss at %d)",
            r->idx, (int)e->apOffsetDb, (int)e->apFloorDbm, (int)triedDbm);
}

/* Success lowers it slowly, so any overshoot past the cliff is small and the
 * next failure recovers it — and the decrement is gated on evidence rather than
 * on time, because "nothing went wrong lately" means nothing if nothing was
 * sent. */
void supeApSucceeded(LoraRadio* r, Neighbor* e) {
    if (!r->supeAdaptive || !e) return;
    if (++e->apSuccess < AP_MIN_SAMPLES) return;
    e->apSuccess = 0;
    if (e->apOffsetDb < 40) {
        e->apOffsetDb = (int8_t)(e->apOffsetDb + 1);
        if (logIsDebug(TAG))
            dbg("lora/%d supe: power down a notch (offset %d dB after %d clean exchanges)",
                r->idx, (int)e->apOffsetDb, AP_MIN_SAMPLES);
    }
    if (e->haveApFloor && (int32_t)(millis() - e->apFloorDecayMs) >= 0)
        e->haveApFloor = false;
}

/* File a path-loss pair: a level measured here, and the power the other side
 * states for it. Never a bare level — a path loss stays true while either end
 * adapts its own power (SUPE.md §10). Step 0 is the hailing configuration;
 * anything else files as the detour pair. */
void supeFilePair(LoraRadio* r, Neighbor* e, int16_t rssi, int8_t peerTxp,
                  uint8_t step) {
    if (!e) return;
    if (step == 0) {
        e->havePair = true;
        e->pairRssi = rssi;
        e->pairTxp  = peerTxp;
    } else {
        e->haveStepPair  = true;
        e->stepRssi      = rssi;
        e->stepTxp       = peerTxp;
        e->stepPairStep  = step;
    }
    if (logIsDebug(TAG))
        dbg("lora/%d supe: path-loss pair filed (%s): %d dBm heard, %d dBm sent",
            r->idx, step == 0 ? "hailing" : "detour", (int)rssi, (int)peerTxp);
}

#endif  /* CONFIG_LORA0_CS_PIN */
