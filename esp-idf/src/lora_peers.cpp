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
    }
    if (NeiHash* h = peersHashFind(st, b4, 4))
        if (st->nei[h->node].used) return &st->nei[h->node];
    return nullptr;
}

/* Retire a row: the node it held is no longer one this table has. rnsd's copy
 * goes, and so does every hash the shared store filed against the slot — the
 * slot is about to mean a different device, and a stub left behind would
 * silently attribute the old node's traffic to the new one. */
void peersRetire(NeiState* st, Neighbor* e) {
    if (!st || !e) return;
    if (e->used && e->rnsdDecl) peersRnsdWithdraw(st, e, /*moved=*/false);
    uint8_t node = (uint8_t)(e - st->nei);
    for (int i = 0; i < NEI_HASHES_MAX; i++)
        if (st->hashes[i].used && st->hashes[i].node == node) st->hashes[i].used = false;
    for (int i = 0; i < NEI_LINKS_MAX; i++) {
        NeiLink* L = &st->links[i];
        if (!L->used) continue;
        if (L->initPeer == node) L->initPeer = LORAQ_PEER_NONE;
        if (L->respPeer == node) L->respPeer = LORAQ_PEER_NONE;
    }
    memset(e, 0, sizeof(*e));
}

/* Forget one node outright: everything this radio holds about it goes, and
 * nothing about it is believed afterwards — not its addresses, not its
 * measurements, not that it speaks our air protocol. The next frame from it
 * builds a fresh row from what that frame proves, which is the point: a node
 * that has moved, been reflashed or been reconfigured is described by a table
 * that learned it before any of that happened, and there is no way to correct a
 * belief except to drop it.
 *
 * Order matters. The link rows go first, while the hashes that name them still
 * resolve to this row; then the protocol's own state for the node, then what
 * was published about the slot, then the row itself.
 *
 * A transaction already on the air is NOT torn down: it is a conversation
 * rather than a memory, the far end is timing against it, and what it files on
 * the way out is fresh measurement. */
static void peersForgetRow(LoraRadio* r, Neighbor* e) {
    NeiState* st = r->nei;
    for (int l = 0; ; l++) {
        uint8_t h4[4];
        if (!peersHashAt(st, e, l, h4)) break;
        for (int i = 0; i < NEI_LINKS_MAX; i++)
            if (st->links[i].used && memcmp(st->links[i].linkId, h4, 4) == 0)
                memset(&st->links[i], 0, sizeof st->links[i]);
    }
    /* Proof expectations against this node's destinations: an outstanding one
     * is a question asked of a node this radio no longer holds, and its answer
     * would score against a row rebuilt from a single frame. */
    for (int i = 0; i < NEI_PEND_MAX; i++) {
        NeiPend* pd = &st->pend[i];
        if (!pd->used) continue;
        for (int d = 0; d < e->nDests; d++)
            if (memcmp(pd->dest, e->dests[d].hash, 16) == 0) { pd->used = false; break; }
    }
#if !defined(CONFIG_LORA_NO_SUPE)
    supeForgetPeer(r, e);
#endif
    loraPeerPubForget(r, (int)(e - st->nei));
    peersRetire(st, e);
}

/* `lora [<n>] forget <num>|all`. `num` is the number `lora n` printed, and
 * `num` < 0 is every node out there — never this device's own rows, which are
 * not neighbours and which nothing would rebuild. Returns how many rows went,
 * or -1 when the number names nobody. Task-side: the table is the radio task's
 * and a row freed under a walk is a dangling row. */
int peersForget(LoraRadio* r, int num) {
    NeiState* st = r ? r->nei : nullptr;
    if (!st) return 0;
    if (num >= 0) {
        Neighbor* e = peersWalk(st, num, nullptr, nullptr);
        if (!e) return -1;
        peersForgetRow(r, e);
        return 1;
    }
    int n = 0;
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used || peersIsLocal(e)) continue;
        peersForgetRow(r, e);
        n++;
    }
    return n;
}

/* Allocate an entry, evicting the longest-unheard non-us one when full. */
Neighbor* peersAlloc(NeiState* st, uint32_t now) {
    Neighbor* victim = nullptr;
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used) { victim = e; break; }
        if (peersIsLocal(e)) continue;
        if (!victim || (int32_t)(peersLastHeard(victim) - peersLastHeard(e)) > 0) victim = e;
    }
    if (!victim) return nullptr;
    if (victim->used) st->evicted++;
    peersRetire(st, victim);     /* leaves the slot cleared */
    victim->used = true;
    victim->lastHeardMs = now;
    return victim;
}

NeiDest* peersAddDest(NeiState* st, Neighbor* e, const uint8_t dest[16], uint32_t now) {
    for (int d = 0; d < e->nDests; d++)
        if (memcmp(e->dests[d].hash, dest, 16) == 0) return &e->dests[d];
    /* The hash is heard directly now, so the stub some linkage frame left for
     * it is redundant. Invariant: a hash is a destination OR a stub, never both
     * — peersKnownHashes() counts the two together and the printer lists them
     * as separate lines, so a hash on both would be double. */
    peersHashDrop(st, dest);
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
    if (e && !peersIsLocal(e)) { peersAddDest(st, e, dest, now); return e; }
    e = peersAlloc(st, now);
    if (e) peersAddDest(st, e, dest, now);
    return e;
}

/* One rx frame provably transmitted by this node: signal envelope + rollup. */
void peersSample(Neighbor* e, int16_t rssi, int16_t snr10, uint32_t now) {
    if (!e->haveSig || rssi < e->rssiMin)   e->rssiMin  = rssi;
    if (!e->haveSig || rssi > e->rssiMax)   e->rssiMax  = rssi;
    if (!e->haveSig || snr10 < e->snrMin10) e->snrMin10 = snr10;
    if (!e->haveSig || snr10 > e->snrMax10) e->snrMax10 = snr10;
    e->rssiLast  = rssi;
    e->snrLast10 = snr10;
    e->haveSig = true;
    e->lastHeardMs = now;
    e->frames++;
    apHeard(e, now);            /* retires a failure floor that has decayed */
    uint32_t absIdx = now / NEI_BUCKET_MS;
    NeiBucket* b = &e->buck[absIdx % NEI_BUCKETS];
    if (b->absIdx != absIdx) { b->absIdx = absIdx; b->cnt = 0; b->rssiSum = 0; b->snrSum10 = 0; }
    b->cnt++;
    b->rssiSum  += rssi;
    b->snrSum10 += snr10;
}

/* One resolved proof expectation: ratio counters + EWMA (α = 1/4).
 *
 * This is also the adaptive-power ratchet's feedback: Reticulum's own delivery
 * signals coming back, or not — slower than a MANIFEST, only for traffic that
 * is proved at all, and costing no airtime whatsoever.
 *
 * A miss reaches the power controller only when the peer has ALSO stopped being
 * heard (AP_MISS_QUIET_MS). A proof that never came from a node whose frames
 * are still arriving says the medium is congested or the far end is busy, and
 * transmitting harder makes both worse; the quality counters record it either
 * way, because that is a statement about the link and not about the power. */
void peersQuality(LoraRadio* r, Neighbor* e, bool hit) {
    e->qSent++;
    if (hit) e->qProved++;
    uint8_t s = hit ? 255 : 0;
    if (!e->haveQuality) { e->quality = s; e->haveQuality = true; }
    else e->quality = (uint8_t)((3 * (int)e->quality + s + 2) / 4);
    if (hit) { apSucceeded(r, e); return; }
    uint32_t now = millis();
    if (e->haveApLastTxp && (uint32_t)(now - e->lastHeardMs) > AP_MISS_QUIET_MS)
        apFailed(r, e, e->apLastTxp, nullptr);
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
    /* Every path that touches a link comes through here, so this is the one
     * place that can keep the hash store's idea of "still alive" honest — and
     * it has to, or the ageing pass would time out a link carrying traffic. */
    peersHashTouch(st, linkId, now);
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
    victim->initPeer = victim->respPeer = LORAQ_PEER_NONE;
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
/* Look without taking: the entry stays for the observer to settle. Compares
 * the first `n` bytes, so a viewer's three-byte tag can ask as well as the
 * proof's full address. */
const NeiPend* peersPendPeek(const NeiState* st, const uint8_t* phash, size_t n) {
    if (!st || !phash) return nullptr;
    for (int i = 0; i < NEI_PEND_MAX; i++) {
        const NeiPend* pd = &st->pend[i];
        if (pd->used && memcmp(pd->phash, phash, n) == 0) return pd;
    }
    return nullptr;
}

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

/* ── the shared hash store ──
 * One table, not a slice per node: which node a hash means is a fact about the
 * pair, and a fixed per-node array spends nearly all its bytes empty while the
 * one node that opens a thirteenth link quietly loses its oldest. */
NeiHash* peersHashFind(NeiState* st, const uint8_t* b, int len) {
    if (!st || len < 1 || len > 4) return nullptr;
    for (int i = 0; i < NEI_HASHES_MAX; i++) {
        NeiHash* h = &st->hashes[i];
        if (h->used && memcmp(h->hash4, b, (size_t)len) == 0) return h;
    }
    return nullptr;
}

void peersHashAdd(NeiState* st, Neighbor* e, const uint8_t* hash, uint32_t now) {
    if (!st || !e) return;
    /* A hash a node already owns as a destination needs no stub — the invariant
     * is that it is one or the other, never both. */
    for (int d = 0; d < e->nDests; d++)
        if (memcmp(e->dests[d].hash, hash, 4) == 0) return;
    uint8_t node = (uint8_t)(e - st->nei);
    if (NeiHash* h = peersHashFind(st, hash, 4)) {
        h->node = node; h->lastMs = now; h->timedOut = false;
        return;
    }
    /* Free slot, else the least recently used — and a timed-out row goes before
     * a live one whatever their ages, since the live one is still resolving
     * traffic and the dead one only history. */
    NeiHash* victim = nullptr;
    for (int i = 0; i < NEI_HASHES_MAX; i++) {
        NeiHash* h = &st->hashes[i];
        if (!h->used) { victim = h; break; }
        if (!victim) { victim = h; continue; }
        if (victim->timedOut != h->timedOut) { if (h->timedOut) victim = h; continue; }
        if ((int32_t)(victim->lastMs - h->lastMs) > 0) victim = h;
    }
    if (!victim) return;
    victim->used = true;
    victim->timedOut = false;
    memcpy(victim->hash4, hash, 4);
    victim->node = node;
    victim->lastMs = now;
}

void peersHashDrop(NeiState* st, const uint8_t b4[4]) {
    if (NeiHash* h = peersHashFind(st, b4, 4)) h->used = false;
}

void peersHashTouch(NeiState* st, const uint8_t b4[4], uint32_t now) {
    if (NeiHash* h = peersHashFind(st, b4, 4)) { h->lastMs = now; h->timedOut = false; }
}

void peersHashAge(NeiState* st, uint32_t now) {
    if (!st) return;
    for (int i = 0; i < NEI_HASHES_MAX; i++) {
        NeiHash* h = &st->hashes[i];
        if (h->used && !h->timedOut && (now - h->lastMs) > NEI_LINK_QUIET_MS)
            h->timedOut = true;
    }
}

int peersHashCount(const NeiState* st, const Neighbor* e) {
    if (!st || !e) return 0;
    uint8_t node = (uint8_t)(e - st->nei);
    int n = 0;
    for (int i = 0; i < NEI_HASHES_MAX; i++)
        if (st->hashes[i].used && st->hashes[i].node == node) n++;
    return n;
}

bool peersHashAt(const NeiState* st, const Neighbor* e, int n, uint8_t out[4]) {
    if (!st || !e || n < 0) return false;
    uint8_t node = (uint8_t)(e - st->nei);
    for (int i = 0; i < NEI_HASHES_MAX; i++) {
        const NeiHash* h = &st->hashes[i];
        if (!h->used || h->node != node) continue;
        if (n-- == 0) { memcpy(out, h->hash4, 4); return true; }
    }
    return false;
}

void peersAddLink4(NeiState* st, Neighbor* e, const uint8_t lid[16], uint32_t now) {
    /* Held as a first-4 like every other hash in the store; SUPE's three-byte
     * tag matches against its leading bytes. */
    peersHashAdd(st, e, lid, now);
}

/* Fold `src` into `dst` and free it. Used when two rows turn out to be one
 * device: an announce naming a dest-only row's identity, or a 0x03 asserting
 * that several hashes (and so several identities) are the same node. */
void peersMergeInto(NeiState* st, Neighbor* dst, Neighbor* src) {
    if (!st || dst == src || !src->used) return;
    for (int n = 0; n < src->nIds; n++) peersAddId(dst, src->ids[n]);
    for (int i = 0; i < src->nDests; i++) {
        NeiDest* nd = peersAddDest(st, dst, src->dests[i].hash, src->dests[i].lastMs);
        nd->announces += src->dests[i].announces;
        if (src->dests[i].haveName) {
            memcpy(nd->nameHash, src->dests[i].nameHash, 10);
            nd->haveName = true;
        }
        if (!nd->name[0] && src->dests[i].name[0])
            safeStrncpy(nd->name, src->dests[i].name, sizeof nd->name);
    }
    /* Hashes follow the node, and the store is shared, so this is a change of
     * owner rather than a copy — which is also why the table needs no room for
     * the merge. */
    uint8_t from = (uint8_t)(src - st->nei), to = (uint8_t)(dst - st->nei);
    for (int i = 0; i < NEI_HASHES_MAX; i++)
        if (st->hashes[i].used && st->hashes[i].node == from)
            st->hashes[i].node = to;
    for (int i = 0; i < NEI_LINKS_MAX; i++) {
        NeiLink* L = &st->links[i];
        if (!L->used) continue;
        if (L->initPeer == from) L->initPeer = to;
        if (L->respPeer == from) L->respPeer = to;
    }
    if (src->haveSig) {
        if (!dst->haveSig || src->rssiMin < dst->rssiMin)   dst->rssiMin  = src->rssiMin;
        if (!dst->haveSig || src->rssiMax > dst->rssiMax)   dst->rssiMax  = src->rssiMax;
        if (!dst->haveSig || src->snrMin10 < dst->snrMin10) dst->snrMin10 = src->snrMin10;
        if (!dst->haveSig || src->snrMax10 > dst->snrMax10) dst->snrMax10 = src->snrMax10;
        /* The envelope folds either way; the newest reading is one of the two,
         * and which one is settled by the same clock the listing prints its age
         * from (folded below, so this still sees both rows' own). */
        if (!dst->haveSig || (int32_t)(src->lastHeardMs - dst->lastHeardMs) > 0) {
            dst->rssiLast  = src->rssiLast;
            dst->snrLast10 = src->snrLast10;
        }
        dst->haveSig = true;
    }
    dst->frames  += src->frames;
    dst->qSent   += src->qSent;
    dst->qProved += src->qProved;
    if (!dst->haveQuality && src->haveQuality) { dst->quality = src->quality; dst->haveQuality = true; }
    /* Adaptive power: the evidence folds, the derived number does not — it is
     * recomputed on the next frame anyway. Each measurement keeps whichever
     * copy is more recent, because a fold joins two views of ONE node and the
     * newer reading is the truer one. Losing them here would make a node forget
     * what it had learnt the moment a 0x03 linked its rows.
     *
     * The floor folds the other way: keep the HIGHER, since each was filed
     * where a frame went missing and neither has been disproved. */
    if (src->haveApRpt && (!dst->haveApRpt ||
                           (int32_t)(src->apRptMs - dst->apRptMs) > 0)) {
        dst->haveApRpt  = true;
        dst->apRptRssi  = src->apRptRssi;
        dst->apRptSnr10 = src->apRptSnr10;
        dst->apRptTxp   = src->apRptTxp;
        dst->apRptMs    = src->apRptMs;
    }
    if (src->havePair && (!dst->havePair ||
                          (int32_t)(src->pairMs - dst->pairMs) > 0)) {
        dst->havePair  = true;
        dst->pairRssi  = src->pairRssi;
        dst->pairSnr10 = src->pairSnr10;
        dst->pairTxp   = src->pairTxp;
        dst->pairMs    = src->pairMs;
    }
    if (src->haveStepPair && (!dst->haveStepPair ||
                              (int32_t)(src->stepPairMs - dst->stepPairMs) > 0)) {
        dst->haveStepPair = true;
        dst->stepRssi     = src->stepRssi;
        dst->stepSnr10    = src->stepSnr10;
        dst->stepTxp      = src->stepTxp;
        dst->stepPairStep = src->stepPairStep;
        dst->stepPairMs   = src->stepPairMs;
    }
    if (src->haveApFloor && (!dst->haveApFloor || src->apFloorDbm > dst->apFloorDbm)) {
        dst->haveApFloor    = true;
        dst->apFloorDbm     = src->apFloorDbm;
        dst->apFloorDecayMs = src->apFloorDecayMs;
    }
    if (src->apOffsetDb < dst->apOffsetDb) dst->apOffsetDb = src->apOffsetDb;
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
    /* Two rows became one, so the node rnsd holds for the absorbed one is a
     * node that no longer exists — withdraw it, and re-declare the survivor,
     * whose label may have gained a name from what it just absorbed. The
     * destinations moved into this row a few lines up and are as reachable as
     * they were a moment ago, so the withdrawal says so: nothing left the air,
     * and the routes to them must survive the re-filing. */
    if (src->rnsdDecl) peersRnsdWithdraw(st, src, /*moved=*/true);
    dst->rnsdDecl = false;
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
    /* A link that has carried nothing for long enough is over. Nothing on the
     * air says so — no close is ever observed — so silence is the only evidence
     * there is, and it is only ever used to sort a row first for eviction. The
     * row itself stays: a frame recorded an hour ago still has to resolve
     * through it, and LoRaMon reads back exactly that far. */
    peersHashAge(st, now);
    /* A node nothing has been heard from for NEI_GONE_MS is not a neighbour any
     * more, and a listing that keeps it is describing a neighbourhood that no
     * longer exists. Silence is the only evidence either way — nothing says
     * goodbye — so the row goes and the next frame from that node builds a
     * fresh one. Our own rows stay whatever the air does. */
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used || peersIsLocal(e)) continue;
        if ((uint32_t)(now - peersLastHeard(e)) > NEI_GONE_MS) {
            peersRetire(st, e);
            st->goneSilent++;
        }
    }
}

int peersOtherCount(const NeiState* st) {
    if (!st) return 0;
    int n = 0;
    for (int i = 0; i < NEI_MAX; i++) {
        const Neighbor* e = &st->nei[i];
        if (e->used && !peersIsLocal(e)) n++;
    }
    return n;
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
    }
    if (NeiHash* h = peersHashFind(st, b4, 4))
        if (st->nei[h->node].used) return &st->nei[h->node];
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
 * that turns an ANNOUNCE into a row. */
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
int peersKnownHashes(const NeiState* st, const Neighbor* e) {
    return (int)e->nDests + peersHashCount(st, e);
}

void peersNodeNames(const Neighbor* e, char* out, size_t outLen) {
    if (!out || outLen == 0) return;
    out[0] = '\0';
    if (!e) return;
    size_t o = 0;
    for (int d = 0; d < e->nDests; d++) {
        const char* n = e->dests[d].name;
        if (!n[0]) continue;
        /* First word only — everything after the first space is caption. */
        size_t w = 0;
        while (n[w] && n[w] != ' ' && n[w] != '\t') w++;
        if (!w) continue;
        /* A node's aspects routinely announce the same name, and repeating it
         * once per destination says nothing the first one did not. */
        bool dup = false;
        for (size_t p = 0; p < o && !dup; ) {
            size_t q = p;
            while (q < o && out[q] != ',') q++;
            if (q - p == w && memcmp(out + p, n, w) == 0) dup = true;
            p = q < o ? q + 1 : o;
        }
        if (dup) continue;
        if (o + w + (o ? 1 : 0) + 1 > outLen) break;
        if (o) out[o++] = ',';
        memcpy(out + o, n, w);
        o += w;
        out[o] = '\0';
    }
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
 * byte encoding the probe uses. Returns the sample count (0 = nothing recent),
 * and through `buckets` how many distinct slots those samples fell in — which
 * is the only handle there is on whether they are spread over time or are one
 * burst, and the power controller declines to move on a burst.
 *
 * `span` is how many of the newest slots to average, so a caller can ask a
 * narrower question than the ring's hour. The ring is 5-minute quantised, so a
 * span of n covers somewhere between (n−1) × 5 and n × 5 minutes depending on
 * where in the current slot the question is asked. */
static uint32_t peersRecentSignal(const Neighbor* e, uint32_t now, int span,
                                  uint8_t* rssiB, int8_t* snrQ,
                                  uint32_t* buckets) {
    uint32_t absNow = now / NEI_BUCKET_MS, cnt = 0, slots = 0;
    int64_t rs = 0, ss = 0;
    if (span > NEI_BUCKETS) span = NEI_BUCKETS;
    for (int b = 0; b < NEI_BUCKETS; b++) {
        const NeiBucket* bk = &e->buck[b];
        if (bk->cnt && absNow - bk->absIdx < (uint32_t)span) {
            cnt += bk->cnt; rs += bk->rssiSum; ss += bk->snrSum10; slots++;
        }
    }
    if (buckets) *buckets = slots;
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

/* Estimated us->them cliff, deci-dBm. Averaged over the newest AP_EST_BUCKETS
 * slots: an estimate is about the link as it is now, and a mean taken over the
 * ring's whole hour would still be quoting a neighbour that has since moved. */
bool peersEstimateCliff10(const LoraRadio* r, const Neighbor* e,
                          uint32_t now, int* cliff10, uint32_t* samples,
                          uint32_t* buckets) {
    uint8_t rssiB; int8_t snrQ;
    uint32_t n = peersRecentSignal(e, now, AP_EST_BUCKETS, &rssiB, &snrQ, buckets);
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
    /* No hash of its own, but something linked one to it. */
    return false;
}

const uint8_t* peersNodeTag(const Neighbor* e) {
    if (e->nDests)    return e->dests[0].hash;
    if (e->haveNode4) return e->node4;
    if (e->nIds)      return e->ids[0];
    return nullptr;
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
    ns->radio   = (uint8_t)r->idx;
    r->nei = ns;
}

/* ── the shared neighbourhood: this table's clustering, handed to rnsd ──
 *
 * rnsd builds one neighbourhood for every medium out of announces, and groups a
 * node's destinations by asking the interface who transmitted them. A
 * point-to-point medium answers that for free — there is only one peer. A radio
 * cannot, per packet: everything is overheard. But this table has already done
 * the harder version of the same join — announce identities, 0x03 linkages, a
 * SUPE association all end in ONE row — so the row is the answer, and these two
 * calls are how it crosses over.
 *
 * Zero timeout, result ignored: this runs on the radio task in the receive
 * path, where blocking is the receiver going deaf. A dropped declaration costs
 * one announce interval — the next announce re-declares. */
static void peersRnsdAux(NeiState* st, const Neighbor* e, bool up, bool moved,
                         const char* label) {
    if (!st) return;
    rnsd_iface_peer_t m = {};
    m.op = RNSD_IFACE_AUX_PEER;
    m.up = up ? 1 : 0;
    m.moved = moved ? 1 : 0;
    snprintf(m.iface, sizeof m.iface, "lora/%u", (unsigned)st->radio);
    peersRnsdKey(peersIdOf(st, e), m.key);
    if (label) safeStrncpy(m.label, label, sizeof m.label);
    itsSendAux("rnsd", RNSD_PORT_IFACE, &m, sizeof m, 0);
}

void peersRnsdDeclare(NeiState* st, Neighbor* e) {
    if (!st || !e || !e->used || peersIsLocal(e)) return;
    /* The label is what rnsd shows before an announce names the node, and what
     * identifies it on a graph regardless: the names its destinations announced,
     * else the node key the air identifies it by. */
    char label[NEI_NAME_MAX * 2];
    peersNodeNames(e, label, sizeof label);
    if (!label[0]) {
        if (e->haveNode4)
            snprintf(label, sizeof label, "%02x%02x%02x%02x",
                     e->node4[0], e->node4[1], e->node4[2], e->node4[3]);
        else label[0] = '\0';
    }
    peersRnsdAux(st, e, true, false, label);
    e->rnsdDecl = true;
}

void peersRnsdWithdraw(NeiState* st, const Neighbor* e, bool moved) {
    peersRnsdAux(st, e, false, moved, nullptr);
}

/* RF is going down: outstanding proofs can't return — drop them, uncounted. */
void peersAbandonPends(LoraRadio* r) {
    if (!r->nei) return;
    for (int i = 0; i < NEI_PEND_MAX; i++) r->nei->pend[i].used = false;
}

#endif  /* CONFIG_LORA0_CS_PIN */
