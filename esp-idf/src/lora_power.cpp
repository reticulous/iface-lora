/**
 * lora_power — adaptive transmit power (§15): the per-frame, per-configuration
 * derivation on the transmit path, the four tiers of evidence it resolves in,
 * the ratchet that trims it and the floor that recovers it, and the 0x04 power
 * request in which we ask a peer for the same thing.
 */
#include "lora_priv.h"
#include "lora_fem.h"

#if defined(CONFIG_LORA0_CS_PIN)

/* ── adaptive TX power: the tx-path half ──
 * (overview at AP_FRESH_MS, in lora_power.h) */
static int8_t apClamp(LoraRadio* r, int want);

/* Deci-dB to whole dB, rounded UP: the half decibel we would save by rounding
 * the other way is not worth the frame it costs. */
static int ceilDeci(int deci) {
    return deci >= 0 ? (deci + 9) / 10 : -((-deci) / 10);
}

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
        if (L && L->haveDest && !peersDestIsLocal(st, L->dest)) return L->dest;
        /* A link dialled TO us carries no hash for its initiator, so there is no
         * destination to name the far end with — but the identifier itself is
         * filed on that node's row once a transaction has named who dialled
         * (peersAddLink4), so hand back the identifier and let the table
         * resolve it. Both directions of a session then reach the same peer,
         * which is what the per-peer cap, the power controller and the reverse
         * leg's scan all key on. A delivery proof addressed to a packet hash
         * falls through here too and simply matches nothing, as before. */
        return h.dest;
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
 * next-hop node's own derivation, else the configured tx_power. This frame is
 * about to fly on the main channel, so the derivation is at the hailing
 * configuration. */
int8_t apTxPower(LoraRadio* r, const uint8_t* pkt, size_t len) {
    if (!r->nei) return r->cfgTxp;
    RnsHdr h;
    if (rnsParse(pkt, len, &h) && h.ptype != NEI_PT_ANNOUNCE) {
        int8_t want;
        if (apLinkSuggest(r, &h, &want)) return apClamp(r, want);
    }
    const uint8_t* nh = apNextHop4(r, pkt, len);
    if (!nh) return r->cfgTxp;
    Neighbor* e = peersFindBy4(r->nei, nh);
    if (!e || peersIsLocal(e)) return r->cfgTxp;
    return apOpenPower(r, e);
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
 * Every gate names itself under `log lora debug`. Four conditions have to line
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
    if (!e || peersIsLocal(e))              why = "dest hash is on no node row";
    /* Only a node that has spoken our air protocol to us will parse the frame;
     * to anyone else it is 35 ms of unparseable noise on a shared channel. That
     * is the RF_PROTO_NAME tag in `lora n`, set by a SUPE announcement, which
     * is what bootstraps eligibility — and what makes the request part of that
     * protocol rather than something `adaptive_txpwr` governs. */
    else if (!e->ourProto)                why = "node has not spoken our protocol";
    else if (!peersEstimateCliff10(r, e, millis(), &cliff10, &samples, nullptr))
                                          why = "no recent signal to estimate from";
    /* One frame's RSSI moves several dB, so a single sample has no business
     * dialling anyone down. */
    else if (samples < AP_MIN_SAMPLES)   { why = "too few samples"; val = (int)samples; }

    int want = 0;
    if (!why) {
        want = ceilDeci(cliff10) + AP_EST_MARGIN_DB;
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

/* ── adaptive TX power: deriving one node's opening power ──
 * (overview at AP_FRESH_MS, in lora_power.h) */

/* Clamp a power to what this radio may transmit at: the chip's range, and
 * never above the configured tx_power. Every tier needs it: a derived need can
 * land below what the chip will emit, and a floor filed after a miss can land
 * above what the antenna is allowed. */
static int8_t apClamp(LoraRadio* r, int want) {
    if (want > r->cfgTxp)    want = r->cfgTxp;
    if (want < AP_FLOOR_DBM) want = AP_FLOOR_DBM;
    int8_t clipped = (int8_t)want;
    r->radio->checkOutputPower((int8_t)want, &clipped);
    return clipped;
}

#if !defined(CONFIG_LORA_NO_SUPE)
/* The freshest path loss to this node, in deci-dB, and where it came from.
 *
 * A path loss is a level minus the power that produced it, so every source
 * yields the same quantity and they differ only in how much they can be
 * trusted: the peer's report measures the direction we actually transmit in,
 * a pair measures the reverse and assumes reciprocity. Which configuration
 * read it does not enter — that is the caller's business, and it is the whole
 * reason this returns a loss rather than a power. */
static bool apPathLoss10(const Neighbor* e, uint32_t now, int* loss10,
                         ApSource* src) {
    if (e->haveApRpt && (uint32_t)(now - e->apRptMs) <= AP_FRESH_MS) {
        *loss10 = ((int)e->apRptTxp - (int)e->apRptRssi) * 10;
        *src = AP_SRC_REPORT;
        return true;
    }
    bool hail = e->havePair     && (uint32_t)(now - e->pairMs)     <= AP_FRESH_MS;
    bool step = e->haveStepPair && (uint32_t)(now - e->stepPairMs) <= AP_FRESH_MS;
    if (!hail && !step) return false;
    /* Both measure the same link; take whichever was read more recently. */
    bool useStep = step && (!hail || (int32_t)(e->stepPairMs - e->pairMs) > 0);
    *loss10 = useStep ? ((int)e->stepTxp - (int)e->stepRssi) * 10
                      : ((int)e->pairTxp - (int)e->pairRssi) * 10;
    *src = AP_SRC_PAIR;
    return true;
}
#endif  /* CONFIG_LORA_NO_SUPE */

/* ─────────────── the derivation (§15) ─────────────── */

/* The power to open toward a peer at a configuration, from the best evidence
 * that is currently fresh. `sensDeci` is the sensitivity that configuration has
 * to clear; the hailing one is what a caller with no detour in hand passes.
 *
 * The result is never stored as the peer's power — it is recomputed for every
 * frame, so evidence going stale, a floor decaying and a link that has moved
 * all show up on the next transmission rather than at some settling time. What
 * IS stored is the ratchet's trim and the failure floor, which are what the
 * loop has learnt and no measurement can supply. */
static int8_t apDerive(LoraRadio* r, Neighbor* e, int sensDeci, bool hailing) {
    if (!e || peersIsLocal(e)) return r->cfgTxp;
    uint32_t now  = millis();
    int      want = r->cfgTxp;
    ApSource src  = AP_SRC_NONE;
    bool     measured = false;

#if !defined(CONFIG_LORA_NO_SUPE)
    int loss10 = 0;
    measured = apPathLoss10(e, now, &loss10, &src);
    if (measured) {
        /* A measurement is worth acting on outright. The reciprocal one costs
         * a further margin because ambient noise is not reciprocal even where
         * path loss is. */
        int margin = SUPE_TARGET_MARGIN_DB
                     + (src == AP_SRC_REPORT ? 0 : AP_RECIP_MARGIN_DB);
        want = ceilDeci(loss10 + sensDeci + margin * 10);
        int trim = e->apOffsetDb < AP_TRIM_MAX_DB ? e->apOffsetDb : AP_TRIM_MAX_DB;
        want -= trim;
    }
#else
    (void)sensDeci;
#endif
    if (!measured && r->adaptive && hailing) {
        /* The EST tier. Nothing here is a measurement of us: the peer's power
         * is assumed and the direction is reversed, so it opens nothing — it
         * only says how far down there might be room to walk. See the timidity
         * note in lora_power.h. A detour never reaches this tier, because a
         * peer that has granted one has told us a power. */
        int      est10   = 0;
        uint32_t samples = 0, buckets = 0;
        if ((uint32_t)(now - e->lastHeardMs) <= AP_FRESH_MS &&
            peersEstimateCliff10(r, e, now, &est10, &samples, &buckets) &&
            samples >= AP_PASSIVE_MIN_SAMPLES && buckets >= 2) {
            int target  = ceilDeci(est10) + AP_EST_MARGIN_DB;
            int surplus = (int)r->cfgTxp - target;
            if (surplus > 0) {
                int cut = surplus / 2;          /* half of a guess, no more */
                int cap = e->qProved ? AP_PASSIVE_CUT_MAX_DB
                                     : AP_PASSIVE_CUT_BLIND_DB;
                if (cut > cap)             cut = cap;
                if (cut > e->apEstWalkDb)  cut = e->apEstWalkDb; /* walk, don't jump */
                if (cut > 0) { want = (int)r->cfgTxp - cut; src = AP_SRC_EST; }
            }
        }
    }

    /* The floor is where this peer last broke, and it outranks every tier: a
     * measurement can be optimistic, and the thing that proved it optimistic
     * was a frame that never arrived. */
    if (e->haveApFloor) {
        if ((int32_t)(now - e->apFloorDecayMs) >= 0) e->haveApFloor = false;
        else if (want < e->apFloorDbm)               want = e->apFloorDbm;
    }
    int8_t out = apClamp(r, want);
    e->apPwr = out;                      /* for `lora <n>`, and for nothing else */
    e->apSrc = src;
    /* What a delivery signal that never comes back will be evidence against.
     * This is the only place a per-peer power is decided, so it is the only
     * place that can honestly record one. */
    e->haveApLastTxp = true;
    e->apLastTxp     = out;
    return out;
}

/* The main channel, which is where everything that is not a granted detour
 * flies: the hailing configuration's own sensitivity. */
int8_t apOpenPower(LoraRadio* r, Neighbor* e) {
    if (!e) return r->cfgTxp;
#if !defined(CONFIG_LORA_NO_SUPE)
    SupeCfg hail = { (uint8_t)r->cfgSf, (uint32_t)r->cfgBwHz, false, 0 };
    return apDerive(r, e, supeSensitivityDeci(&hail), true);
#else
    /* With SUPE out there is no measured tier to serve and no sensitivity model
     * linked in; the EST tier carries its own floor inside peersHeadroom10. */
    return apDerive(r, e, 0, true);
#endif
}

/* A miss raises the power fast — being wrong downward costs connectivity, so
 * recovery is immediate and large — and remembers where it broke, on a floor
 * that decays, so the loop settles above the cliff instead of oscillating
 * across it. Successive misses climb by AP_FLOOR_STEP_DB each, because each
 * one files its floor above the power the last one tried. */
/* Would raising the power have helped? Only if the power was marginal, and the
 * peer's own report of our transmissions is the one measurement that can say —
 * it is the direction we transmit in, which no transmitter can measure for
 * itself. Turn it into a path loss, put the power that just failed through it
 * at the configuration that failed, and compare with what that configuration
 * needs. Margin to spare means the frame was lost to something power does not
 * fix, and raising is worse than useless: it spends the link's whole range on
 * the wrong variable and holds it there for a floor's decay.
 *
 * No fresh report is no answer, and no answer means raise — the miss stands on
 * its own, which is the conservative reading. */
static bool apMissWasPower(const Neighbor* e, int8_t triedDbm,
                           const SupeCfg* cfg, int* marginDbOut) {
    if (!e->haveApRpt) return true;
    if ((uint32_t)(millis() - e->apRptMs) > AP_FRESH_MS) return true;
    int loss   = (int)e->apRptTxp - (int)e->apRptRssi;
    int expect = (int)triedDbm - loss;                 /* what they should read */
    int sens   = cfg ? supeSensitivityDeci(cfg) / 10 : -120;
    int margin = expect - sens;
    if (marginDbOut) *marginDbOut = margin;
    return margin < AP_MISS_MARGIN_DB;
}

void apFailed(LoraRadio* r, Neighbor* e, int8_t triedDbm, const SupeCfg* cfg) {
    if (!e) return;
    int margin = 0;
    if (!apMissWasPower(e, triedDbm, cfg, &margin)) {
        /* Scored as a miss everywhere else; simply not read as a power one. */
        if (logIsDebug(TAG))
            dbg("lora/%d adaptive: miss at %d dBm with %d dB of margin — "
                "not a power failure, holding", r->idx, (int)triedDbm, margin);
        return;
    }
    int floorDbm = (int)triedDbm + AP_FLOOR_STEP_DB;
    if (floorDbm > r->cfgTxp) floorDbm = r->cfgTxp;
    e->apOffsetDb = 0;
    /* The EST tier's walk retreats rather than resetting: it was bought a frame
     * at a time and the floor below is what actually holds the power up, but a
     * walk left where it stood would reapply the cut that just failed the
     * moment that floor decayed. */
    if (e->apEstWalkDb > 0) e->apEstWalkDb = (int8_t)(e->apEstWalkDb / 2);
    e->apEstHeard = 0;
    e->apFloorDbm = (int8_t)floorDbm;
    e->haveApFloor = true;
    e->apFloorDecayMs = millis() + AP_FLOOR_DECAY_MS;
    e->apSuccess = 0;
    if (logIsDebug(TAG))
        dbg("lora/%d adaptive: power back up (floor %d dBm after a miss at %d)",
            r->idx, (int)e->apFloorDbm, (int)triedDbm);
}

/* Success lowers it slowly, so any overshoot past the cliff is small and the
 * next failure recovers it — and the notch is gated on evidence rather than on
 * time, because "nothing went wrong lately" means nothing if nothing was
 * sent. */
void apSucceeded(LoraRadio* r, Neighbor* e) {
    if (!e) return;
    if (++e->apSuccess < AP_MIN_SAMPLES) return;
    e->apSuccess = 0;
    if (e->apOffsetDb < 40) {
        e->apOffsetDb = (int8_t)(e->apOffsetDb + 1);
        if (logIsDebug(TAG))
            dbg("lora/%d adaptive: power down a notch (trim %d dB after %d clean exchanges)",
                r->idx, (int)e->apOffsetDb, AP_MIN_SAMPLES);
    }
    if (e->haveApFloor && (int32_t)(millis() - e->apFloorDecayMs) >= 0)
        e->haveApFloor = false;
}

/* One frame heard from this node, which is what the EST tier's walk is paid in.
 * The cut it authorises is bounded by the walk, so this is the only thing that
 * lets that tier move at all for a peer whose traffic never returns a delivery
 * signal — and it stops of its own accord the moment the peer stops being
 * heard, which is the condition the tier must not walk through.
 *
 * A live failure floor holds the walk still. The floor already clamps the
 * power, so walking underneath it would buy nothing now and would land the
 * power somewhere unproven the moment it decayed. */
void apHeard(Neighbor* e, uint32_t now) {
    if (!e) return;
    if (e->haveApFloor) {
        if ((int32_t)(now - e->apFloorDecayMs) < 0) return;
        e->haveApFloor = false;
    }
    if (++e->apEstHeard < AP_EST_WALK_FRAMES) return;
    e->apEstHeard = 0;
    if (e->apEstWalkDb < AP_PASSIVE_CUT_MAX_DB) e->apEstWalkDb++;
}

#if !defined(CONFIG_LORA_NO_SUPE)
/* A granted step: the same evidence against a different floor. This is the
 * whole reason the tiers hold a path loss rather than a power — SF5/500k sits
 * some 15–20 dB above SF12/125k in sensitivity, and one number for both would
 * be tuned for one and wrong for the other by that much. */
int8_t apOpenPowerAt(LoraRadio* r, Neighbor* e, const SupeCfg* cfg) {
    if (!e || !cfg) return r->cfgTxp;
    return apDerive(r, e, supeSensitivityDeci(cfg), false);
}

/* The peer's account of our own transmission, from the MANIFEST that closes a
 * detour: the level it read, and the power we sent at. */
void apFileReport(LoraRadio* r, Neighbor* e, int16_t rssi, int8_t ourTxp) {
    if (!e) return;
    e->haveApRpt = true;
    e->apRptRssi = rssi;
    e->apRptTxp  = ourTxp;
    e->apRptMs   = millis();
    if (logIsDebug(TAG))
        dbg("lora/%d supe: peer read our %d dBm at %d dBm (loss %d dB)",
            r->idx, (int)ourTxp, (int)rssi, (int)ourTxp - (int)rssi);
}

/* File a path-loss pair: a level measured here, and the power the other side
 * states for it. Never a bare level — a path loss stays true while either end
 * adapts its own power (SUPE.md §10). Step 0 is the hailing configuration;
 * anything else files as the detour pair. */
void supeFilePair(LoraRadio* r, Neighbor* e, int16_t rssi, int8_t peerTxp,
                  uint8_t step) {
    if (!e) return;
    uint32_t now = millis();
    if (step == 0) {
        e->havePair = true;
        e->pairRssi = rssi;
        e->pairTxp  = peerTxp;
        e->pairMs   = now;
    } else {
        e->haveStepPair  = true;
        e->stepRssi      = rssi;
        e->stepTxp       = peerTxp;
        e->stepPairStep  = step;
        e->stepPairMs    = now;
    }
    if (logIsDebug(TAG))
        dbg("lora/%d supe: path-loss pair filed (%s): %d dBm heard, %d dBm sent",
            r->idx, step == 0 ? "hailing" : "detour", (int)rssi, (int)peerTxp);
}

#endif  /* CONFIG_LORA_NO_SUPE */

#endif  /* CONFIG_LORA0_CS_PIN */
