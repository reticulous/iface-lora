/**
 * lora_peers — the passive neighbour/peer table: per-node aggregates (family,
 * SUPE support, path loss, signal envelope, delivery quality) and the lookups
 * over them. Filled by lora_observe; read by everything.
 */
#include "lora_priv.h"

#if defined(CONFIG_LORA0_CS_PIN)

/* ── table bookkeeping ── */

Neighbor* peersFindByIdentity(NeiState* st, const uint8_t id[16]) {
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used) continue;
        for (int n = 0; n < e->nIds; n++)
            if (memcmp(e->ids[n], id, 16) == 0) return e;
    }
    return nullptr;
}

Neighbor* peersFindByDest(NeiState* st, const uint8_t dest[16]) {
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used) continue;
        for (int d = 0; d < e->nDests; d++)
            if (memcmp(e->dests[d].hash, dest, 16) == 0) return e;
    }
    return nullptr;
}


bool peersDestIsLocal(NeiState* st, const uint8_t dest[16]) {
    Neighbor* e = peersFindByDest(st, dest);
    return e && peersIsLocal(e);
}

/* The row a linkage frame has attributed a first-4 to without the hash itself
 * ever having been heard: its node key, or an entry in a 0x03 list. Narrower
 * than peersFindBy4() on purpose — a full dest that merely shares its first four
 * bytes with an unrelated one is a collision, not the same device, so it must
 * not pull two rows together. */
Neighbor* peersFindClaim4(NeiState* st, const uint8_t b4[4]) {
    if (!st) return nullptr;
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used) continue;
        if (e->haveNode4 && memcmp(e->node4, b4, 4) == 0) return e;
        for (int l = 0; l < e->nLink4; l++)
            if (memcmp(e->link4[l], b4, 4) == 0) return e;
    }
    return nullptr;
}

/* Allocate an entry, evicting the longest-unheard non-us one when full. */
Neighbor* peersAlloc(NeiState* st, uint32_t now) {
    Neighbor* victim = nullptr;
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used) { victim = e; break; }
        if (peersIsLocal(e)) continue;
        if (!victim || (int32_t)(victim->lastHeardMs - e->lastHeardMs) > 0) victim = e;
    }
    if (!victim) return nullptr;
    memset(victim, 0, sizeof(*victim));
    victim->used = true;
    victim->lastHeardMs = now;
    return victim;
}

/* Drop the placeholder a linkage frame left for a first-4. Invariant: a hash is
 * on a row either as a dest or as a `link4` stub, never both — peersKnownHashes()
 * counts the two together, and the printer lists them as separate lines. */
static void peersDropLink4(Neighbor* e, const uint8_t b4[4]) {
    for (int l = 0; l < e->nLink4; l++) {
        if (memcmp(e->link4[l], b4, 4) != 0) continue;
        memmove(e->link4[l], e->link4[l + 1], (size_t)(e->nLink4 - 1 - l) * 4);
        e->nLink4--;
        return;
    }
}

NeiDest* peersAddDest(Neighbor* e, const uint8_t dest[16], uint32_t now) {
    for (int d = 0; d < e->nDests; d++)
        if (memcmp(e->dests[d].hash, dest, 16) == 0) return &e->dests[d];
    peersDropLink4(e, dest);   /* the hash is heard now; the stub is redundant */
    NeiDest* nd;
    if (e->nDests < NEI_DESTS_MAX) nd = &e->dests[e->nDests++];
    else {                                       /* replace the stalest dest */
        nd = &e->dests[0];
        for (int d = 1; d < NEI_DESTS_MAX; d++)
            if ((int32_t)(nd->lastMs - e->dests[d].lastMs) > 0) nd = &e->dests[d];
    }
    memset(nd, 0, sizeof(*nd));
    memcpy(nd->hash, dest, 16);
    nd->lastMs = now;
    return nd;
}

/* Entry for a dest that provably transmitted to us direct but has never
 * announced (LRPROOF at hops 0, a proof of our packet) — identity unknown. */
Neighbor* peersEnsureDest(NeiState* st, const uint8_t dest[16], uint32_t now) {
    Neighbor* e = peersFindByDest(st, dest);
    if (e) return e;
    /* A 0x03 may already have claimed this hash for a node — the dest belongs on
     * that row, not on a fresh one. Never across the us/them boundary: a peer's
     * linkage claim is unauthenticated. */
    e = peersFindClaim4(st, dest);
    if (e && !peersIsLocal(e)) { peersAddDest(e, dest, now); return e; }
    e = peersAlloc(st, now);
    if (e) peersAddDest(e, dest, now);
    return e;
}

/* One rx frame provably transmitted by this node: signal envelope + rollup. */
void peersSample(Neighbor* e, int16_t rssi, int16_t snr10, uint32_t now) {
    if (!e->haveSig || rssi < e->rssiMin)   e->rssiMin  = rssi;
    if (!e->haveSig || rssi > e->rssiMax)   e->rssiMax  = rssi;
    if (!e->haveSig || snr10 < e->snrMin10) e->snrMin10 = snr10;
    if (!e->haveSig || snr10 > e->snrMax10) e->snrMax10 = snr10;
    e->haveSig = true;
    e->lastHeardMs = now;
    e->frames++;
    uint32_t absIdx = now / NEI_BUCKET_MS;
    NeiBucket* b = &e->buck[absIdx % NEI_BUCKETS];
    if (b->absIdx != absIdx) { b->absIdx = absIdx; b->cnt = 0; b->rssiSum = 0; b->snrSum10 = 0; }
    b->cnt++;
    b->rssiSum  += rssi;
    b->snrSum10 += snr10;
}

/* One resolved proof expectation: ratio counters + EWMA (α = 1/4).
 *
 * This is also SUPE's adaptive-power feedback, and it is the only feedback
 * there is: a detour gives no acknowledgement, so within one transaction there
 * is no direct evidence a train landed. What a sender observes instead is
 * whether Reticulum's own delivery signals come back — slower, only for traffic
 * that is proved at all, and costing no airtime whatsoever. */
void peersQuality(LoraRadio* r, Neighbor* e, bool hit) {
    e->qSent++;
    if (hit) e->qProved++;
    uint8_t s = hit ? 255 : 0;
    if (!e->haveQuality) { e->quality = s; e->haveQuality = true; }
    else e->quality = (uint8_t)((3 * (int)e->quality + s + 2) / 4);
    if (hit) supeApSucceeded(r, e);
    else if (e->haveApLastTxp) supeApFailed(r, e, e->apLastTxp);
}

NeiLink* peersLinkFind(NeiState* st, const uint8_t linkId[16]) {
    for (int i = 0; i < NEI_LINKS_MAX; i++)
        if (st->links[i].used && memcmp(st->links[i].linkId, linkId, 16) == 0)
            return &st->links[i];
    return nullptr;
}

/* The link a 3-byte tag names, if any. A START carries only the prefix. */
NeiLink* peersLinkFindBy3(NeiState* st, const uint8_t b3[3]) {
    if (!st) return nullptr;
    for (int i = 0; i < NEI_LINKS_MAX; i++)
        if (st->links[i].used && memcmp(st->links[i].linkId, b3, 3) == 0)
            return &st->links[i];
    return nullptr;
}

NeiLink* peersLinkEnsure(NeiState* st, const uint8_t linkId[16], uint32_t now) {
    NeiLink* L = peersLinkFind(st, linkId);
    if (L) return L;
    NeiLink* victim = nullptr;
    for (int i = 0; i < NEI_LINKS_MAX; i++) {
        L = &st->links[i];
        if (!L->used) { victim = L; break; }
        if (!victim || (int32_t)(victim->lastMs - L->lastMs) > 0) victim = L;
    }
    memset(victim, 0, sizeof(*victim));
    victim->used = true;
    memcpy(victim->linkId, linkId, 16);
    victim->lastMs = now;
    return victim;
}

void peersPendAdd(NeiState* st, const uint8_t phash[16], const uint8_t dest[16],
                       bool isLR, bool counted, uint32_t now) {
    NeiPend* victim = nullptr;
    for (int i = 0; i < NEI_PEND_MAX; i++) {
        NeiPend* pd = &st->pend[i];
        if (!pd->used) { victim = pd; break; }
        if (!victim || (int32_t)(victim->deadlineMs - pd->deadlineMs) > 0) victim = pd;
    }
    victim->used = true;
    victim->isLR = isLR;
    victim->counted = counted;
    memcpy(victim->phash, phash, 16);
    memcpy(victim->dest, dest, 16);
    victim->deadlineMs = now + NEI_PROOF_TIMEOUT_MS;
}

/* Match-and-free a pend entry by the hash a proof is addressed to. The freed
 * slot's fields stay readable until the next peersPendAdd (single task). */
NeiPend* peersPendTake(NeiState* st, const uint8_t phash[16]) {
    for (int i = 0; i < NEI_PEND_MAX; i++) {
        NeiPend* pd = &st->pend[i];
        if (pd->used && memcmp(pd->phash, phash, 16) == 0) { pd->used = false; return pd; }
    }
    return nullptr;
}

Neighbor* peersFindBy4(NeiState* st, const uint8_t b4[4]);

static bool peersHasId(const Neighbor* e, const uint8_t id[16]) {
    for (int n = 0; n < e->nIds; n++)
        if (memcmp(e->ids[n], id, 16) == 0) return true;
    return false;
}

void peersAddId(Neighbor* e, const uint8_t id[16]) {
    if (peersHasId(e, id)) return;
    if (e->nIds < NEI_IDS_MAX) { memcpy(e->ids[e->nIds++], id, 16); return; }
    memmove(e->ids[0], e->ids[1], (NEI_IDS_MAX - 1) * 16);
    memcpy(e->ids[NEI_IDS_MAX - 1], id, 16);
}

static void peersAddLink4Raw(Neighbor* e, const uint8_t b4[4]) {
    for (int d = 0; d < e->nDests; d++)
        if (memcmp(e->dests[d].hash, b4, 4) == 0) return;
    for (int l = 0; l < e->nLink4; l++)
        if (memcmp(e->link4[l], b4, 4) == 0) return;
    if (e->nLink4 < NEI_LINK4_MAX) { memcpy(e->link4[e->nLink4++], b4, 4); return; }
    memmove(e->link4[0], e->link4[1], (NEI_LINK4_MAX - 1) * 4);
    memcpy(e->link4[NEI_LINK4_MAX - 1], b4, 4);
}

/* Fold `src` into `dst` and free it. Used when two rows turn out to be one
 * device: an announce naming a dest-only row's identity, or a 0x03 asserting
 * that several hashes (and so several identities) are the same node. */
void peersMergeInto(Neighbor* dst, Neighbor* src) {
    if (dst == src || !src->used) return;
    for (int n = 0; n < src->nIds; n++) peersAddId(dst, src->ids[n]);
    for (int i = 0; i < src->nDests; i++) {
        NeiDest* nd = peersAddDest(dst, src->dests[i].hash, src->dests[i].lastMs);
        nd->announces += src->dests[i].announces;
        if (src->dests[i].haveName) {
            memcpy(nd->nameHash, src->dests[i].nameHash, 10);
            nd->haveName = true;
        }
        if (!nd->name[0] && src->dests[i].name[0])
            safeStrncpy(nd->name, src->dests[i].name, sizeof nd->name);
    }
    for (int l = 0; l < src->nLink4; l++) peersAddLink4Raw(dst, src->link4[l]);
    if (src->haveSig) {
        if (!dst->haveSig || src->rssiMin < dst->rssiMin)   dst->rssiMin  = src->rssiMin;
        if (!dst->haveSig || src->rssiMax > dst->rssiMax)   dst->rssiMax  = src->rssiMax;
        if (!dst->haveSig || src->snrMin10 < dst->snrMin10) dst->snrMin10 = src->snrMin10;
        if (!dst->haveSig || src->snrMax10 > dst->snrMax10) dst->snrMax10 = src->snrMax10;
        dst->haveSig = true;
    }
    dst->frames  += src->frames;
    dst->qSent   += src->qSent;
    dst->qProved += src->qProved;
    if (!dst->haveQuality && src->haveQuality) { dst->quality = src->quality; dst->haveQuality = true; }
    /* A measured determination outranks an estimated one; otherwise first wins.
     * Losing it across a fold would make a node forget its power the moment a
     * 0x03 linked its rows, exactly as with txPwr above. */
    if (src->haveApPwr && (!dst->haveApPwr || (dst->apFromEst && !src->apFromEst))) {
        dst->apPwr     = src->apPwr;
        dst->apFromEst = src->apFromEst;
        dst->haveApPwr = true;
    }
    dst->provesData |= src->provesData;
    dst->transit    |= src->transit;
    dst->ourProto   |= src->ourProto;
    dst->isUs       |= src->isUs;
    dst->isRnode    |= src->isRnode;
    if (!dst->haveNode4 && src->haveNode4) { memcpy(dst->node4, src->node4, 4); dst->haveNode4 = true; }
    if (src->haveAdv && src->advHashes > dst->advHashes) {
        dst->haveAdv = true;
        dst->advHashes = src->advHashes;
    }
    dst->roaming |= src->roaming;
    if ((int32_t)(src->lastHeardMs - dst->lastHeardMs) > 0) dst->lastHeardMs = src->lastHeardMs;
    for (int i = 0; i < NEI_BUCKETS; i++) {
        NeiBucket* eb = &dst->buck[i];
        NeiBucket* db = &src->buck[i];
        if (!db->cnt) continue;
        if (eb->absIdx == db->absIdx) {
            eb->cnt += db->cnt; eb->rssiSum += db->rssiSum; eb->snrSum10 += db->snrSum10;
        } else if ((int32_t)(db->absIdx - eb->absIdx) > 0) {
            *eb = *db;
        }
    }
    src->used = false;
}

/* Expire overdue proof expectations (driven from the task loop; nextDeadline
 * wakes the task for the soonest outstanding deadline). */
void peersExpire(LoraRadio* r, uint32_t now) {
    NeiState* st = r->nei;
    if (!st) return;
    for (int i = 0; i < NEI_PEND_MAX; i++) {
        NeiPend* pd = &st->pend[i];
        if (!pd->used || (int32_t)(now - pd->deadlineMs) < 0) continue;
        pd->used = false;
        if (pd->counted) {
            Neighbor* e = peersFindByDest(st, pd->dest);
            if (e) peersQuality(r, e, false);
        }
    }
}

/* ── cooperative hash linkage (0x02 / 0x03) ──
 * Everything here keys on a hash's first 4 bytes, which is all the linkage
 * frames carry; a full 16-byte dest already in the table matches on its first
 * 4. See the format block at the top of the file. */

/* Find a neighbour by any first-4 it is known under: its node key, a full
 * dest hash from an announce, or a hash a 0x03 linked to it. */
Neighbor* peersFindBy4(NeiState* st, const uint8_t b4[4]) {
    if (!st) return nullptr;
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used) continue;
        if (e->haveNode4 && memcmp(e->node4, b4, 4) == 0) return e;
        for (int d = 0; d < e->nDests; d++)
            if (memcmp(e->dests[d].hash, b4, 4) == 0) return e;
        for (int l = 0; l < e->nLink4; l++)
            if (memcmp(e->link4[l], b4, 4) == 0) return e;
    }
    return nullptr;
}


/* Find a node by the first four bytes of one of its **identity** hashes.
 *
 * Deliberately separate from peersFindBy4, which indexes the addresses traffic is
 * *sent to* — destination hashes, the transport first-4, and hashes a linkage
 * frame claimed. An identity is none of those: it is what a node *is*, derived
 * from the public key its announce carried, and the table holds it in `ids[]`
 * where nothing else looks.
 *
 * SUPE announces identities rather than destinations precisely because a node
 * typically has more of the latter (plans/SUPE.md §7), so this is the lookup
 * that turns an ANNOUNCE2 into a row. */
Neighbor* peersFindByIdent4(NeiState* st, const uint8_t b4[4]) {
    if (!st) return nullptr;
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used) continue;
        for (int k = 0; k < e->nIds; k++)
            if (memcmp(e->ids[k], b4, 4) == 0) return e;
    }
    return nullptr;
}

/* How many distinct hashes we hold for a node — announced dests plus the ones
 * a 0x03 linked in. This is what we compare against a peer's advertised count. */
int peersKnownHashes(const Neighbor* e) {
    return (int)e->nDests + (int)e->nLink4;
}

/* ── reciprocity estimate: a power determination, guessed for free ──
 * The probe learns the TX power at which our signal lands on the peer's demod
 * floor by *asking* it. The same number can be inferred from a frame we heard
 * FROM them, if we assume a power they transmitted at: their path loss is ours.
 * This computes the us->them cliff from frames we overheard,
 * so the two are directly comparable — which is the point. Against a peer that
 * has been probed we have ground truth to check the estimate against; against a
 * non-cooperating peer the estimate is all there will ever be.
 *
 * `s.lora.assumed_peer_txp` (default 22) is the power we credit an unprobed peer
 * with. Assuming high errs safe — a peer that is actually quieter makes us
 * over-estimate path loss and transmit higher than needed. It also has to be
 * settable, because a bench node parked at a low `tx_power` announces at that
 * power, not at 22, and the estimate would be off by the difference. */
static int peersAssumedPeerTxp(void) {
    int v = storageGetInt("s.lora.assumed_peer_txp", 22);
    if (v < -30 || v > 30) v = 22;
    return v;
}

/* Recent mean signal for a node, from the 5-minute bucket ring, in the same
 * byte encoding the probe uses. Returns the sample count (0 = nothing recent). */
static uint32_t peersRecentSignal(const Neighbor* e, uint32_t now,
                                uint8_t* rssiB, int8_t* snrQ) {
    uint32_t absNow = now / NEI_BUCKET_MS, cnt = 0;
    int64_t rs = 0, ss = 0;
    for (int b = 0; b < NEI_BUCKETS; b++) {
        const NeiBucket* bk = &e->buck[b];
        if (bk->cnt && absNow - bk->absIdx < NEI_BUCKETS) {
            cnt += bk->cnt; rs += bk->rssiSum; ss += bk->snrSum10;
        }
    }
    if (!cnt) return 0;
    int rssi = (int)(rs / (int64_t)cnt);
    int snr10 = (int)(ss / (int64_t)cnt);
    int rb = -rssi;
    *rssiB = (uint8_t)(rb < 0 ? 0 : rb > 255 ? 255 : rb);
    int sq = snr10 * 4 / 10;
    *snrQ = (int8_t)(sq < -128 ? -128 : sq > 127 ? 127 : sq);
    return cnt;
}

/* Link margin above the demodulation floor, deci-dB, from an encoded rssi/snr
 * pair. How much power the sender could have dropped and still been decoded —
 * which, subtracted from its assumed transmit power, is the reciprocity
 * estimate of the power we need toward it. */
static int peersHeadroom10(const LoraRadio* r, uint8_t rssiB, int8_t snrQ) {
    /* Semtech's required SNR per spreading factor, deci-dB: −7.5 dB at SF7 and
     * 2.5 dB worse per factor below it, 2.5 dB better per factor above. */
    int sf = r->cfgSf < 5 ? 5 : (r->cfgSf > 12 ? 12 : r->cfgSf);
    int floor10 = -75 - (sf - 7) * 25;
    int snr10   = ((int)snrQ * 10) / 4;
    int margin  = snr10 - floor10;
    (void)rssiB;                 /* the margin is an SNR question, not a level one */
    return margin > 0 ? margin : 0;
}

/* Estimated us->them cliff, deci-dBm. */
bool peersEstimateCliff10(const LoraRadio* r, const Neighbor* e,
                               uint32_t now, int* cliff10, uint32_t* samples) {
    uint8_t rssiB; int8_t snrQ;
    uint32_t n = peersRecentSignal(e, now, &rssiB, &snrQ);
    if (samples) *samples = n;
    if (!n) return false;
    *cliff10 = peersAssumedPeerTxp() * 10 - peersHeadroom10(r, rssiB, snrQ);
    return true;
}

/* Walk the table in display order — us first, then the others — handing each
 * node its printed number. `want` < 0 visits everything; otherwise the walk
 * stops at that node number. Returns the matched node, or null. The printer and
 * the CLI's node resolver share this so the numbers always agree. */
Neighbor* peersWalk(NeiState* st, int want, PeersVisitFn fn, void* ud) {
    int num = 0;
    for (int pass = 0; pass < 2; pass++) {
        for (int k = 0; k < NEI_MAX; k++) {
            Neighbor* e = &st->nei[k];
            if (!e->used || (pass == 0) != peersIsLocal(e)) continue;
            /* Pass 0 is the local rows — us and the RNode client. Both number 0
             * and neither is addressable: naming `0` would be aiming the
             * radio at this device. */
            int n = peersIsLocal(e) ? 0 : ++num;
            if (want >= 0) { if (n == want && !peersIsLocal(e)) return e; continue; }
            if (fn) fn(e, n, ud);
        }
    }
    return nullptr;
}



/* The first-4 a probe should be addressed to for this node — its transport
 * hash where known, since that is the one every node has. */
bool peersNodeFirst4(const Neighbor* e, uint8_t out[4]) {
    for (int d = 0; d < e->nDests; d++) {
        const char* asp = e->dests[d].haveName ? rnsNameLabel(e->dests[d].nameHash) : nullptr;
        if (asp && strcmp(asp, "rnstransport.probe") == 0) { memcpy(out, e->dests[d].hash, 4); return true; }
    }
    if (e->nDests)   { memcpy(out, e->dests[0].hash, 4); return true; }
    if (e->haveNode4) { memcpy(out, e->node4, 4); return true; }
    if (e->nLink4)   { memcpy(out, e->link4[0], 4); return true; }
    return false;
}

/* Passive peer table: allocated once, history kept across config cycles. A
 * failed alloc just leaves the feature off (every path guards on nei).
 * Published only after init — the CLI reads the pointer cross-task. */
void peersInit(LoraRadio* r) {
    if (r->nei) return;
    NeiState* ns = (NeiState*)gp_alloc(sizeof(NeiState));
    if (!ns) return;
    memset(ns, 0, sizeof(NeiState));
    ns->sinceMs = millis();
    r->nei = ns;
}

/* RF is going down: outstanding proofs can't return — drop them, uncounted. */
void peersAbandonPends(LoraRadio* r) {
    if (!r->nei) return;
    for (int i = 0; i < NEI_PEND_MAX; i++) r->nei->pend[i].used = false;
}

#endif  /* CONFIG_LORA0_CS_PIN */
