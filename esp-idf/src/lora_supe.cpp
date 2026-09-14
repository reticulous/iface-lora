/**
 * lora_supe — the platform half of SUPE: the SupeHost implementation the pure
 * engine (supe_engine.{h,cpp}) runs against on this device, the boundary lock
 * that serialises every entry point, the train buffers, the ANNOUNCE beat and
 * its peer-table ingest, and the adapters that file the engine's peer notes
 * into the peer table and the power controller.
 */
#include "lora_priv.h"
#include "lora_fem.h"

#if defined(CONFIG_LORA0_CS_PIN)

/* How soon after a radio comes up it first announces itself, jittered over
 * twice this. Nothing can seed a schedule toward a node it has never heard
 * announce, so the first one is worth having promptly. */
#define SUPE_ANN_FIRST_MS    15000

/* How long a first-seen announce going on the air holds the ANNOUNCE behind
 * it (supeAnnSoon). Long enough that the announces a boot or a route change
 * produces arrive as one burst and are answered by one frame; short enough
 * that the answer still belongs to the event that caused it. */
#define SUPE_ANN_SOON_MS     10000

/* The hold (SUPE.md §12): three hails unanswered in one run make the peer
 * unreachable, and no hail is sent to it for a while — its traffic is queued
 * and waits out its own patience. Declaring that is a bet that trying again is
 * not worth the airtime, and a meeting now costs one short frame, so the bet
 * is made in stages: a second at first, doubling per further unanswered run,
 * to a ceiling of a minute. A peer that is present — heard at all lately, by
 * a hail to somebody else, an announce, a meeting overheard — but does not
 * answer us is likelier a transient fault than a terminal one, so its ceiling
 * is ten seconds instead. Presence never clears a hold; only an answer does. */
#define SUPE_HOLD_BASE_MS         1000
#define SUPE_HOLD_MAX_MS         60000
#define SUPE_HOLD_PRESENT_MAX_MS 10000
#define SUPE_PRESENT_MS          60000    /* heard within this: present */

/* The boundary watchdog: a meeting that outlives every deadline inside it
 * holds the radio against the whole outbound queue. Generous — the longest
 * legal meeting is two train ceilings plus repairs and turnarounds. */
#define SUPE_MEET_WATCHDOG_MS 8000

/* ─────────────── tag resolution ───────────────
 *
 * The node (or link) behind a 3-byte tag. Tags are prefixes of the addresses
 * the peer table already indexes; a 3-byte match can collide, and the cost of
 * a collision here is one misfiled measurement, not a wrong delivery. */
static Neighbor* tagNode(LoraRadio* r, const uint8_t tag[SUPE_TAG_LEN]) {
    NeiState* st = r->nei;
    if (!st) return nullptr;
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &st->nei[i];
        if (!e->used) continue;
        if (e->haveNode4 && memcmp(e->node4, tag, SUPE_TAG_LEN) == 0) return e;
        for (int d = 0; d < e->nDests; d++)
            if (memcmp(e->dests[d].hash, tag, SUPE_TAG_LEN) == 0) return e;
        /* Identities, and not only the destinations derived from them: a
         * HAIL's sender_ident names the seeking node by its identity, and
         * nothing is ever *addressed* to an identity, so it appears in none of
         * the lists above. */
        for (int k = 0; k < e->nIds; k++)
            if (memcmp(e->ids[k], tag, SUPE_TAG_LEN) == 0) return e;
    }
    /* Hashes some linkage frame said mean a node — a link identifier above all.
     * One shared store rather than a slice per row, so it is one lookup here
     * instead of a walk inside the loop above. A timed-out row still answers:
     * a frame recorded an hour ago is still a frame for that node. */
    if (NeiHash* h = peersHashFind(st, tag, SUPE_TAG_LEN))
        if (st->nei[h->node].used) return &st->nei[h->node];
    /* A link identifier we initiated resolves to the node the link request was
     * addressed to (SUPE.md §5.1, §11 "links inherit") — this is what keeps a
     * session's traffic meeting after rnsd switches from the destination hash
     * to the link id. A link dialled TO us resolves to nobody by construction
     * (its initiator is anonymous); that case is the link branch of peer_get,
     * not this one. */
    NeiLink* L = peersLinkFindBy3(st, tag);
    if (L && L->haveDest && !peersDestIsLocal(st, L->dest))
        return peersFindByDest(st, L->dest);
    return nullptr;
}

static NeiLink* tagLink(LoraRadio* r, const uint8_t tag[SUPE_TAG_LEN]) {
    return r->nei ? peersLinkFindBy3(r->nei, tag) : nullptr;
}

/* ─────────────── the host ─────────────── */

static uint32_t hNow(void*) { return millis(); }
static uint32_t hRand(void*) { return esp_random(); }

static void hSchedule(void* ctx, uint32_t at_ms) {
    LoraRadio* r = (LoraRadio*)ctx;
    if (!r->supe || !r->supe->timer) return;
    uint32_t now = millis();
    /* Floor at 1 ms: a due event fires on the next millisecond rather than
     * NOW. A zero-delay one-shot re-armed from its own callback never leaves
     * the esp_timer task, and that task outranks the radio task — so a due
     * event that only the radio task can service (a tx completion above all)
     * would be starved by the very timer waiting on it. */
    uint32_t d = (int32_t)(at_ms - now) > 0 ? at_ms - now : 1;
    esp_timer_stop(r->supe->timer);
    esp_timer_start_once(r->supe->timer, (uint64_t)d * 1000ull);
}

static void hSha256(void*, const uint8_t* d, uint16_t n, uint8_t out[32]) {
    rnsdSha256(d, n, out);
}

/* Move to a slot's channel and configuration. The way back is the hailing
 * configuration the radio already holds in cfg*, so nothing needs saving. */
static bool hTune(void* ctx, uint8_t chan, const SupeCfg* c, uint8_t sync) {
    LoraRadio* r = (LoraRadio*)ctx;
    r->radio->standby();
    int16_t st = r->radio->setFrequency((float)supeChanFreq(r, chan) / 1.0e6f);
    if (st == RADIOLIB_ERR_NONE) st = radioSetBw(r, (float)c->bwHz / 1.0e3f);
    if (st == RADIOLIB_ERR_NONE) st = radioSetSf(r, c->sf);
    if (st == RADIOLIB_ERR_NONE) st = radioSyncWord(r, sync);
    if (st != RADIOLIB_ERR_NONE) {
        warn("lora/%d supe: retune failed: %s (%d)", r->idx, rlErrName(st), (int)st);
        return false;
    }
    /* Another channel, another noise reference: the hailing channel's floor says
     * nothing about this one. Its own, from the last visit, says plenty — so the
     * two are exchanged rather than thrown away. Before chNow moves, since that
     * is the channel the floor being parked belongs to. */
    loraMonDwell(r, millis());      /* close the span on the channel being left */
    csmaFloorSwitch(r, r->chNow, chan);
    /* A fresh channel means a fresh receiver: an IRQ flag latched on the old
     * one — a preamble the hailing channel detected moments ago, above all —
     * would read as this channel being busy, and the slot CCA runs right after
     * this returns. */
    radioIrqClearAll(r);
    r->rxActiveStart = 0;
    r->rxHeaderSeen  = false;
    r->airSf   = c->sf;
    r->airBwHz = (int)c->bwHz;
    r->chNow   = chan;
    radioStartRx(r);
    return true;
}

/* Coming home is unconditional and unchecked: a failed retune must never
 * leave the radio stranded off the hailing channel. */
static void hTuneHome(void* ctx) {
    LoraRadio* r = (LoraRadio*)ctx;
    r->radio->standby();
    r->radio->setFrequency((float)r->cfgFreqHz / 1.0e6f);
    radioSetBw(r, (float)r->cfgBwHz / 1.0e3f);
    radioSetSf(r, (uint8_t)r->cfgSf);
    radioSyncWord(r, r->cfgSync);
    r->radio->setPreambleLength((size_t)r->cfgPreamble);
    const int8_t homeChip = rfChipDbm(r, r->cfgTxp);
    r->radio->setOutputPower(homeChip);
    r->airSf       = (uint8_t)r->cfgSf;
    r->airBwHz     = r->cfgBwHz;
    r->airPreamble = r->cfgPreamble;
    r->airImplicit = false;
    r->txPwrNow    = rfAntennaDbm(r, homeChip);
    /* The meeting's floor is not this channel's — but the hailing channel's
     * own, from before the meeting, is the best estimate in the system and the
     * one a node returning home has to contend against straight away. */
    loraMonDwell(r, millis());      /* close the span on the channel being left */
    csmaFloorSwitch(r, r->chNow, LORA_CH_HAIL);
    /* Fresh channel, fresh receiver state — as in hTune. */
    radioIrqClearAll(r);
    r->rxActiveStart = 0;
    r->rxHeaderSeen  = false;
    r->chNow       = LORA_CH_HAIL;
    radioStartRx(r);
    if (logIsVerbose(TAG)) verb("lora/%d supe: home", r->idx);
}

/* One SUPE frame, non-blocking: fired and left; the completion comes back
 * through txRearmRx → supeAfterTx → the engine. */
static bool hTxFrame(void* ctx, const uint8_t* f, uint16_t len, int8_t dbm) {
    LoraRadio* r = (LoraRadio*)ctx;
    if (!r->running || len == 0 || len > sizeof r->txFrame[0]) return false;
    memcpy(r->txFrame[0], f, len);
    r->txFrameLen[0]  = len;
    r->txType[0]      = LORA_PKT_OURS;
    r->txFrameCount   = 1;
    r->txFrameSent    = 0;
    r->txPayloadBytes = 0;
    r->txFromRnode    = false;
    r->txWaitMs       = 0;
    r->txOwnMs        = 0;
    r->txWaitPend     = false;
    r->supe->engineTx = true;
    if (logIsVerbose(TAG))
        verb("lora/%d supe: tx type 0x%02x %uB ch%u txp=%d",
             r->idx, f[0], (unsigned)len, (unsigned)r->chNow, (int)dbm);
    apApplyPower(r, dbm);
    loraNoteAnswer(r);
    startTxFrame(r, 0);
    return r->txActive;
}

static void hRx(void* ctx) {
    rearmRx((LoraRadio*)ctx);
}

/* The modem's own account of whether a frame is arriving: a preamble it has
 * locked, or a header it has validated and is still filling in. Both are
 * measured in milliseconds and end by themselves.
 *
 * A half-assembled split is deliberately NOT one of these, though it looks like
 * the same thing one frame further on. What ends it is a reassembly timeout
 * measured in seconds, and a slot deferred on a clock that long is not deferred
 * at all — every slot inside the window is walked past, so a node holding half
 * a split stops attending its schedules entirely and goes deaf to the peer
 * hailing it. That trade is the wrong way round and §12 already names it: a
 * missed slot means nothing, a missed hail means the peer cannot reach us. The
 * cost of leaving it out is the far smaller one — a retune may lose a partner
 * frame that was still coming, which is a dropped packet the layer above
 * retransmits. */
static bool hRxBusy(void* ctx) {
    return radioRxInProgress((LoraRadio*)ctx);
}

/* One clear-channel read at the current tuning. The appointment grants the
 * peer's attention, never the spectrum: a busy channel skips the slot.
 *
 * A refusal says what it measured. "Busy" is a verdict about the world, and a
 * verdict nothing can contradict is how a broken sense passes for a crowded
 * band — which is exactly what it did here, six slots a schedule, on empty
 * channels. */
static bool hCca(void* ctx) {
    LoraRadio* r = (LoraRadio*)ctx;
    bool clear = csmaSenseClear(r);
    if (!clear && logIsVerbose(TAG))
        verb("lora/%d supe: ch%u reads busy at %.0f dBm (bw %d kHz, rx %s)",
            r->idx, (unsigned)r->chNow, (double)channelRssi(r),
            r->airBwHz / 1000, radioRxInProgress(r) ? "in progress" : "idle");
    return clear;
}

/* ─────────────── the trains ─────────────── */

/* Build the outgoing train from the queue: every packet for the peer, split
 * into on-air frames exactly as the plain path would split it, copied whole —
 * the repair round resends these bytes, so they outlive the queue's view. The
 * queue entries are consumed only on a delivered close (train_done). */
static bool hTrainBuild(void* ctx, uint16_t peerId,
                        const uint8_t tag[SUPE_TAG_LEN], uint8_t maxFrames,
                        SupeTrainInfo* out) {
    LoraRadio* r = (LoraRadio*)ctx;
    SupeState* ss = r->supe;
    memset(out, 0, sizeof *out);
    ss->txTCount = 0;
    if (maxFrames > SUPE_TRAIN_MAX) maxFrames = SUPE_TRAIN_MAX;
    for (uint8_t i = 0; i < loraqDepth(&r->q) && out->count < maxFrames; i++) {
        LoraPkt* p = loraqAt(&r->q, i);
        /* Unicast only. A packet with no tag is a broadcast — an announce, a
         * path request — and it still carries a peer id from the next-hop
         * lookup, so matching on the id alone would sweep it into one node's
         * train and it would reach that node and nobody else. */
        if (!(p->flags & LORAQ_F_HAVE_TAG)) continue;
        bool match = false;
        if (tag && memcmp(p->tag, tag, SUPE_TAG_LEN) == 0) match = true;
        if (peerId != LORAQ_PEER_NONE && p->peer_id == peerId) match = true;
        if (!match) continue;
        uint8_t need = (uint8_t)(p->len > RNODE_MAX_PAYLOAD ? 2 : 1);
        if ((uint8_t)(out->count + need) > maxFrames) break;
        /* The passive neighbour tap, at the moment the packet is committed to
         * the air — the same place beginTx taps the plain path. */
        peersObserve(r, p->bytes, p->len, true, 0, 0,
                     (uint8_t)(p->flags & LORAQ_ORIG_MASK), LORAQ_PEER_NONE);
        uint8_t seq = (uint8_t)((esp_random() & 0x0F) << 4);
        size_t first = p->len > RNODE_MAX_PAYLOAD ? RNODE_MAX_PAYLOAD : p->len;
        for (uint8_t half = 0; half < need; half++) {
            uint8_t* fr = ss->txT[ss->txTCount];
            size_t   nb = half ? p->len - first : first;
            fr[0] = (uint8_t)(seq | (need == 2 ? RNODE_FLAG_SPLIT : 0));
            memcpy(fr + 1, p->bytes + (half ? first : 0), nb);
            ss->txTLen[ss->txTCount] = (uint16_t)(1 + nb);
            ss->txTPkt[ss->txTCount] = p->bytes;
            /* The RNode client's queue is released when its packet is finished
             * with the radio, and a client running flow control transmits
             * nothing more until that release arrives. On the plain path that
             * is transmit-done; here it is the last frame the packet was cut
             * into, since a split's halves are one packet to the client. */
            ss->txTRelease[ss->txTCount] =
                (p->flags & LORAQ_ORIG_MASK) == LORAQ_ORIG_RNODE &&
                half == (uint8_t)(need - 1);
            out->lens[out->count] = (uint16_t)(1 + nb);
            out->csum[out->count] = supeCrc8(fr, 1 + nb);
            ss->txTCount++;
            out->count++;
        }
    }
    return out->count > 0;
}

static bool hTrainFire(void* ctx, uint8_t idx, int8_t dbm) {
    LoraRadio* r = (LoraRadio*)ctx;
    SupeState* ss = r->supe;
    if (!r->running || idx >= ss->txTCount) return false;
    uint16_t len = ss->txTLen[idx];
    memcpy(r->txFrame[0], ss->txT[idx], len);
    r->txFrameLen[0]  = len;
    r->txType[0]      = LORA_PKT_RNS;
    r->txFrameCount   = 1;
    r->txFrameSent    = 0;
    r->txPayloadBytes = (size_t)(len - 1);
    /* Spent on the first firing: a repair round resends the same frame, and a
     * second release would let the client put two packets in flight. */
    r->txFromRnode      = ss->txTRelease[idx];
    ss->txTRelease[idx] = false;
    r->txWaitMs       = 0;
    r->txOwnMs        = 0;
    r->txWaitPend     = false;
    ss->engineTx = true;
    apApplyPower(r, dbm);
    loraNoteAnswer(r);
    startTxFrame(r, 0);
    return r->txActive;
}

static void hTrainDone(void* ctx, bool delivered) {
    LoraRadio* r = (LoraRadio*)ctx;
    SupeState* ss = r->supe;
    if (delivered) {
        /* The peer's answer proved the train landed: consume the queue entries
         * the frames were cut from, found by their stable heap blocks — the
         * queue may have shifted or grown underneath the meeting. */
        for (uint8_t f = 0; f < ss->txTCount; f++) {
            if (!ss->txTPkt[f]) continue;
            for (uint8_t i = 0; i < loraqDepth(&r->q); i++) {
                if (loraqAt(&r->q, i)->bytes == ss->txTPkt[f]) {
                    loraqConsume(&r->q, i);
                    break;
                }
            }
            /* A split cut two frames from one block; consume it once. */
            for (uint8_t g = (uint8_t)(f + 1); g < ss->txTCount; g++)
                if (ss->txTPkt[g] == ss->txTPkt[f]) ss->txTPkt[g] = nullptr;
        }
    }
    /* Unproven: the entries stay queued for the next chance, and Reticulum's
     * duplicate hash list absorbs the rare both-happened case. */
    ss->txTCount = 0;
}

/* Flush the buffered inbound train upward in the engine's sequence order:
 * each frame re-enters the ordinary frame-delivery path — split reassembly,
 * the observer, rnsd — exactly as if it had just left the radio. */
static void hTrainDeliver(void* ctx, const uint8_t* order, uint8_t n) {
    LoraRadio* r = (LoraRadio*)ctx;
    SupeState* ss = r->supe;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t idx = order[i];
        if (idx >= ss->rxTCount) continue;
        r->rssiLast = (float)ss->rxTRssi[idx];
        r->snrLast  = (float)ss->rxTSnr10[idx] / 10.0f;
        bridgeFrameDeliver(r, ss->rxT[idx], ss->rxTLen[idx]);
    }
    ss->rxTCount = 0;
    /* A meeting hands over everything it managed to collect, all at once and
     * once only. So half a split still waiting for its partner after that batch
     * is waiting for a frame this meeting already failed to bring: it was one
     * of the ones the repair round could not recover. Holding it until the
     * reassembly timeout keeps a dead frame in the buffer and, worse, leaves
     * the split marked pending across the quiet stretch that follows, where
     * everything else has to reason about a reception that will never land. */
    if (r->splitPending) {
        r->splitPending = false;
        r->splitTimeouts++;
        if (logIsVerbose(TAG))
            verb("lora/%d split half dropped: the train that carried its "
                 "partner ended without it", r->idx);
    }
}

/* ─────────────── peers ─────────────── */

static bool hPeerGet(void* ctx, const uint8_t tag[SUPE_TAG_LEN], SupePeerView* out) {
    LoraRadio* r = (LoraRadio*)ctx;
    memset(out, 0, sizeof *out);
    out->peerId = LORAQ_PEER_NONE;
    out->txpMax = r->cfgTxp;
    out->txpOpen = r->cfgTxp;
    Neighbor* e = tagNode(r, tag);
    if (e && !peersIsLocal(e) && e->supeSeen) {
        out->known = true;
        out->peerId = peersIdOf(r->nei, e);
        out->fam = e->supeCaps.fam;
        out->topBudget = e->supeCaps.topStep;
        out->maxTxpDbm = e->supeCaps.maxPwrDbm;
        out->txpOpen = apOpenPower(r, e);
        out->unanswered = e->silentCount;
        out->holdUntilMs = e->absentUntilMs;
        out->intervalUntilMs = e->retryWaitUntilMs;
        out->detoured = e->detoured;
        return true;
    }
    /* The one peer we can never name: the node that dialled a link to us. Its
     * measurements were filed against the link (SUPE.md §11, links inherit). */
    NeiLink* L = tagLink(r, tag);
    if (L && L->ours && L->supeSeen) {
        out->known = true;
        out->fam = L->supeCaps.fam;
        out->topBudget = L->supeCaps.topStep;
        out->maxTxpDbm = L->supeCaps.maxPwrDbm;
        return true;
    }
    return false;
}

/* The power to open toward the node behind a tag at a configuration — asked
 * where the frames it covers will actually fly (§15). */
static int8_t hTxpOpen(void* ctx, const uint8_t tag[SUPE_TAG_LEN],
                       const SupeCfg* cfg) {
    LoraRadio* r = (LoraRadio*)ctx;
    Neighbor* e = tagNode(r, tag);
    if (!e || peersIsLocal(e)) return r->cfgTxp;
    return apOpenPowerAt(r, e, cfg);
}

static void hPeerNote(void* ctx, const uint8_t tag[SUPE_TAG_LEN],
                      const SupePeerNote* n) {
    LoraRadio* r = (LoraRadio*)ctx;
    Neighbor* e = tagNode(r, tag);
    /* On the listening side the tag can be one of our own addresses — our own
     * local row must not collect the peer's measurements. What can hold them
     * is the link (below). */
    if (e && peersIsLocal(e)) e = nullptr;
    NeiLink*  L = e ? nullptr : tagLink(r, tag);
    uint32_t now = millis();
    switch (n->ev) {
        case SUPE_EV_ALIVE:
            /* Presence: heard at all. It shortens a hold's ceiling and clears
             * nothing — under an asymmetric link we hear a peer constantly
             * that never hears us, and a rule that let presence stand in for
             * reachability would hail such a peer for ever (§12). */
            if (e) e->supeHeardMs = now;
            break;
        case SUPE_EV_ANSWERED:
            /* Reachability: it answered us, or hailed us. The run ends, the
             * hold lifts, the full run is restored. */
            if (e) {
                e->silentCount = 0;
                e->absentUntilMs = 0;
                e->retryWaitUntilMs = 0;
                e->supeHeardMs = now;
            }
            break;
        case SUPE_EV_UNANSWERED:
            if (e) {
                /* The interval in which the hailed party may hail back; the
                 * next hail of the run waits it out. */
                e->retryWaitUntilMs = now + n->backoffMs;
                if (e->silentCount < 255) e->silentCount++;
                if (e->silentCount >= SUPE_RUN_HAILS) {
                    uint32_t hold = SUPE_HOLD_BASE_MS;
                    uint8_t over = (uint8_t)(e->silentCount - SUPE_RUN_HAILS);
                    if (over > 6) over = 6;            /* 1 s → 64 s, then capped */
                    hold <<= over;
                    bool present = e->supeHeardMs &&
                                   (uint32_t)(now - e->supeHeardMs) < SUPE_PRESENT_MS;
                    uint32_t cap = present ? SUPE_HOLD_PRESENT_MAX_MS : SUPE_HOLD_MAX_MS;
                    if (hold > cap) hold = cap;
                    e->absentUntilMs = now + hold;
                    if (logIsDebug(TAG))
                        dbg("lora/%d supe: %02x%02x%02x unreachable%s, holding off %lums "
                            "(%u unanswered)", r->idx, tag[0], tag[1], tag[2],
                            present ? " but present" : "",
                            (unsigned long)hold, (unsigned)e->silentCount);
                }
            }
            break;
        case SUPE_EV_PAIR:
            /* Never a bare level: a level measured here with the power the
             * other side stated for it. Filed against the hailing pair or the
             * meeting pair by the configuration it was read at. */
            if (e) {
                bool hail = (n->cfg.sf == (uint8_t)r->cfgSf &&
                             n->cfg.bwHz == (uint32_t)r->cfgBwHz);
                supeFilePair(r, e, n->rssiDbm, n->snr10, n->txpDbm, hail ? 0 : 1);
                e->supeHeardMs = now;
            } else if (L && L->ours) {
                L->havePair = true;
                L->pairRssi = n->rssiDbm;
                L->pairTxp  = n->txpDbm;
                L->supeHeardMs = now;
            }
            break;
        case SUPE_EV_REPORT:
            /* The peer's account of our own transmission: what it read,
             * against what we sent — the one measurement of the direction we
             * transmit in, and the freshest input the train power resolves on
             * (§15). */
            if (e) apFileReport(r, e, n->rssiDbm, n->snr10, n->txpDbm);
            break;
        case SUPE_EV_TRAIN_OK:
            if (e) {
                if (n->haveLevel) {
                    apFileReport(r, e, n->rssiDbm, n->snr10, n->txpDbm);
                    /* The ratchet moves only on reported headroom — thin
                     * margin holds (§15). */
                    int marginDeci = (int)n->rssiDbm * 10
                                     - (int)supeSensitivityDeci(&n->cfg);
                    if (marginDeci > (SUPE_TARGET_MARGIN_DB + 3) * 10)
                        apSucceeded(r, e);
                } else {
                    apSucceeded(r, e);
                }
            }
            break;
        case SUPE_EV_TRAIN_LOST:
            if (e) apFailed(r, e, n->triedTxpDbm, &n->cfg);
            break;
        case SUPE_EV_MET:
            if (e) {
                e->detoured = true;
                e->supeHeardMs = now;
                e->silentCount = 0;
                e->absentUntilMs = 0;
            }
            break;
    }
}

static void hChanGet(void* ctx, SupeChanView* out) {
    LoraRadio* r = (LoraRadio*)ctx;
    memset(out, 0, sizeof *out);
    int n = 0;
    supeRegimeChans(r->afa, &n);
    out->nChans = (uint8_t)(n < SUPE_CH_MAX - 1 ? n : SUPE_CH_MAX - 1);
    out->anyBudget = airtimeAnyBudget(r);
    for (uint8_t c = 1; c <= out->nChans; c++)
        out->usable[c] = airtimeMayI(r, c, nullptr) ? 1 : 0;
}

/* One SupeHost per radio, pointing back at it. */
static SupeHost s_hosts[LORA_NUM_RADIOS];

/* What the engine may emit, from what this tag's logger will actually print:
 * asking for verbose lines nobody will show is formatting thrown away. */
static uint8_t supeLogLevel(void) {
    if (logIsVerbose(TAG)) return SUPE_LOG_VERB;
    if (logIsDebug(TAG))   return SUPE_LOG_DBG;
    return SUPE_LOG_NONE;
}

static void hLog(void* ctx, bool verbose, const char* msg) {
    LoraRadio* r = (LoraRadio*)ctx;
    if (verbose) verb("lora/%d %s", r->idx, msg);
    else         dbg("lora/%d %s", r->idx, msg);
}

/* The timer only wakes the radio task; it never runs the engine. Everything a
 * deadline can trigger — a retune, a transmit, a delivery into rnsd — must run
 * on the task that owns the radio and the ITS handles: an owned ITS send from
 * the esp_timer task is refused outright, which on the bench read as every
 * deadline-closed meeting's cargo being dropped ("itsSendOwned on
 * non-packet-link handle"). supePoll services the due deadline on the wake
 * this delivers. */
static void supeTimerCb(void* arg) {
    (void)arg;
    loraNudge();
}

static void hostFill(LoraRadio* r) {
    SupeHost* h = &s_hosts[r->idx];
    memset(h, 0, sizeof *h);
    h->ctx       = r;
    h->now_ms    = hNow;
    h->rand32    = hRand;
    h->schedule  = hSchedule;
    h->sha256    = hSha256;
    h->tune      = hTune;
    h->tune_home = hTuneHome;
    h->tx_frame  = hTxFrame;
    h->rx        = hRx;
    h->rx_busy   = hRxBusy;
    h->cca       = hCca;
    h->train_build   = hTrainBuild;
    h->train_fire    = hTrainFire;
    h->train_done    = hTrainDone;
    h->train_deliver = hTrainDeliver;
    h->peer_get  = hPeerGet;
    h->peer_note = hPeerNote;
    h->txp_open  = hTxpOpen;
    h->chan_get  = hChanGet;
    h->log       = hLog;
    h->logLevel  = SUPE_LOG_NONE;
}

/* ─────────────── lifecycle and the boundary ─────────────── */

uint8_t supeOwnFamily(const LoraRadio* r) {
    switch (chipFamily(r->slot->chip)) {
        case FAM_SX126X: return SUPE_FAM_SX126X;
        case FAM_SX127X: return SUPE_FAM_SX127X;
        case FAM_SX128X: return SUPE_FAM_SX128X;
        case FAM_LR11X0: return SUPE_FAM_LR11X0;
        case FAM_LR2021: return SUPE_FAM_LR2021;
    }
    return SUPE_FAM_SX126X;
}

SupeCaps supeOwnCaps(const LoraRadio* r) {
    SupeCaps c = {};
    c.fam = supeOwnFamily(r);
    /* Our ceiling: the ladder's own reach from this hailing configuration on
     * the widest channel the regime offers, family-bounded on our side. */
    SupeLadderEntry lad[SUPE_LADDER_MAX_ENTRIES];
    uint32_t maxBw = r->cfgBwHz;
    int n = 0;
    const SupeChan* ch = supeRegimeChans(r->afa, &n);
    for (int i = 0; i < n; i++) if (ch[i].bwHz > maxBw) maxBw = ch[i].bwHz;
    int ln = supeLadder(r->afa, SUPE_VERSION, (uint8_t)r->cfgSf, (uint32_t)r->cfgBwHz,
                        maxBw, c.fam, c.fam, lad, SUPE_LADDER_MAX_ENTRIES);
    c.topStep   = (uint8_t)(ln > 0 ? ln - 1 : 0);
    c.maxPwrDbm = r->cfgTxp;
    return c;
}

bool supeInit(LoraRadio* r) {
    if (!r->supe) {
        SupeState* ss = (SupeState*)gp_alloc(sizeof(SupeState));
        if (ss) {
            memset(ss, 0, sizeof(SupeState));
            ss->lock = xSemaphoreCreateRecursiveMutex();
            r->supe  = ss;
        }
        if (!ss || !ss->lock) {
            warn("lora/%d SUPE: no memory for its state — staying off", r->idx);
            r->supe = nullptr;
            return false;
        }
        hostFill(r);
        supeEngInit(&ss->eng, &s_hosts[r->idx], &r->q);
    }
    if (!r->supe->timer) {
        esp_timer_create_args_t a = {};
        a.callback        = supeTimerCb;
        a.arg             = r;
        a.dispatch_method = ESP_TIMER_TASK;
        a.name            = "lora-supe";
        if (esp_timer_create(&a, &r->supe->timer) != ESP_OK) return false;
    }
    if (!airtimeInit(r)) return false;
    SupeCaps caps = supeOwnCaps(r);
    supeEngConfig(&r->supe->eng, r->afa, caps.fam, caps.topStep, r->cfgTxp,
                  (uint8_t)r->cfgSf, (uint32_t)r->cfgBwHz,
                  (uint8_t)r->cfgCr, (uint16_t)r->cfgPreamble, r->cfgSync,
                  RNODE_MAX_PAYLOAD);
    /* The first beat is soon, not one interval out: a node that has just come
     * up is exactly the node its neighbours know nothing about. Jittered so a
     * fleet powered up together does not converge on the same second. */
    if (r->supe->annNextMs == 0)
        r->supe->annNextMs = millis() + SUPE_ANN_FIRST_MS
                             + (esp_random() % SUPE_ANN_FIRST_MS);
    return true;
}

void supeOnRadioStop(LoraRadio* r) {
    if (!r->supe) return;
    if (r->supe->timer) esp_timer_stop(r->supe->timer);
    supeEngReset(&r->supe->eng);
    r->supe->engineTx = false;
    r->supe->annPending = false;
    r->supe->txTCount = 0;
    r->supe->rxTCount = 0;
}

/* Mounted: the machinery is up. Ready: it is up AND this node speaks the
 * protocol. Everything that transmits asks the second; announcement ingest asks
 * the first, because a node that has been switched off still wants a correct
 * picture of who around it speaks SUPE — and still owes its neighbours the
 * announcement that says it does not. */
bool supeMounted(const LoraRadio* r) { return r->supe != nullptr; }
bool supeReady(const LoraRadio* r) { return r->supeOn && r->supe != nullptr; }

bool supeBusy(const LoraRadio* r) {
    return r->supe && supeEngBusy(&r->supe->eng);
}

bool supeXactLive(const LoraRadio* r) {
    return r->supe && supeEngXactLive(&r->supe->eng);
}

uint16_t supeCargoPeer(const LoraRadio* r) {
    return r->supe ? supeEngCargoPeer(&r->supe->eng) : (uint16_t)LORAQ_PEER_NONE;
}

bool supeHoldsRadio(const LoraRadio* r) {
    return r->supe && (supeEngBusy(&r->supe->eng) || r->supe->annPending);
}

bool supeMeetingTag(const LoraRadio* r, uint8_t out[SUPE_TAG_LEN]) {
    if (!r->supe) return false;
    const SupeMeet* m = &r->supe->eng.m;
    if (m->phase == SUPE_M_IDLE || !m->haveTag) return false;
    memcpy(out, m->tag, SUPE_TAG_LEN);
    return true;
}

void supeLock(LoraRadio* r) {
    if (r->supe && r->supe->lock) xSemaphoreTakeRecursive(r->supe->lock, portMAX_DELAY);
}
void supeUnlock(LoraRadio* r) {
    if (r->supe && r->supe->lock) xSemaphoreGiveRecursive(r->supe->lock);
}
static bool supeTryLock(LoraRadio* r) {
    if (!r->supe || !r->supe->lock) return false;
    return xSemaphoreTakeRecursive(r->supe->lock, 0) == pdTRUE;
}

/* ─────────────── ANNOUNCE (SUPE.md §9) ─────────────── */

static void annIngest(LoraRadio* r, const uint8_t* f, size_t len, int16_t rssi,
                      int16_t snr10) {
    SupeAnn a;
    if (!supeDecAnn(f, len, &a)) return;
    if (!r->nei) return;
    uint32_t now = millis();
    int matched = 0;
    /* One frame, one node. The identities in an announcement are a statement
     * that they all belong to the same radio, and until now nothing acted on
     * it: each was resolved on its own and whatever row it landed on was
     * stamped, so a node whose destinations had arrived on separate rows stayed
     * split for as long as it lived — announcing every interval that it was one
     * node, and being re-split every time. Everything downstream reads a row:
     * the SUPE bit, the path-loss pairs, the adaptive power, the absence
     * record. Split rows mean a node met on one of them and addressed at full
     * power on the other.
     *
     * Four bytes is a short name to fuse two rows on, and fusing is not
     * reversible. It is worth it because the alternative is permanent and
     * visible while a collision is neither: two distinct nodes sharing an
     * identity prefix would merge into one row, and what that costs is a power
     * estimate fitted to the nearer of them — the far one is then addressed too
     * quietly, misses, and the ratchet raises the power until it does not. A
     * self-correcting error against a permanent one. */
    Neighbor* keep = nullptr;
    uint8_t   orphan[SUPE_ANN_MAX];      /* ids this table could not place */
    uint8_t   nOrphan = 0;
    for (int i = 0; i < a.count; i++) {
        Neighbor* e = peersFindByIdent4(r->nei, a.ids[i]);
        if (!e) e = peersFindBy4(r->nei, a.ids[i]);
        if (!e) e = peersFindClaim4(r->nei, a.ids[i]);
        if (!e && i > 0 && nOrphan < SUPE_ANN_MAX) orphan[nOrphan++] = (uint8_t)i;
        /* Never met: this frame IS the introduction — it carries the identities
         * and the capabilities together — so keep it rather than wait out an
         * announce interval for the next one. Only four bytes of each identity
         * are on the air, which is exactly what a claim row holds; the node's
         * own Reticulum announce supplies the hash and folds the two together
         * (observeAnnounce). Until then the row has no destination, so nothing
         * routes to it and the claim can only ever answer for SUPE. */
        if (!e && i == 0) {
            e = peersAlloc(r->nei, now);
            if (e) { memcpy(e->node4, a.ids[i], 4); e->haveNode4 = true; }
        }
        if (!e || peersIsLocal(e)) continue;
        /* The first row this announcement resolves to keeps the node; every
         * later one is the same radio arriving under another of its names. */
        if (!keep) keep = e;
        else if (e != keep) {
            if (logIsDebug(TAG))
                dbg("lora/%d supe: %02x%02x%02x folded into one node",
                    r->idx, a.ids[i][0], a.ids[i][1], a.ids[i][2]);
            peersMergeInto(r->nei, keep, e);
            e = keep;
        }
        matched++;
        /* A node renouncing the protocol: drop the belief that it speaks it, so
         * its traffic goes back out as plain framing. The row itself stays —
         * this is still an announcement, and everything else in it (the
         * identities, the level, that the node is alive at all) is as good as
         * any other. What it is NOT is evidence for meeting: no absence record
         * is cleared here, because there is nothing to be absent from. */
        if (a.regime == SUPE_REGIME_NONE) {
            if (e->supeSeen && logIsDebug(TAG))
                dbg("lora/%d supe: %02x%02x%02x no longer speaks SUPE",
                    r->idx, a.ids[i][0], a.ids[i][1], a.ids[i][2]);
            e->supeSeen = false;
            e->ourProto = false;
            e->supeHeardMs = now;
            supeFilePair(r, e, rssi, snr10, a.pwrDbm, 0);
            continue;
        }
        bool first = !e->supeSeen;
        e->supeSeen    = true;
        e->supeCaps    = a.caps;
        /* Presence, not reachability: an announcement says the node is there,
         * not that it can hear us. It shortens a hold and clears none (§12). */
        e->supeHeardMs = now;
        e->ourProto    = true;
        /* The power byte is what makes the frame worth hearing: the reading
         * and the stated power together are path loss, not a bare level. */
        supeFilePair(r, e, rssi, snr10, a.pwrDbm, 0);
        if (first && logIsDebug(TAG))
            dbg("lora/%d supe: %02x%02x%02x speaks SUPE (family %u, ceiling %u, max %d dBm)",
                r->idx, a.ids[i][0], a.ids[i][1], a.ids[i][2],
                (unsigned)a.caps.fam, (unsigned)a.caps.topStep,
                (int)a.caps.maxPwrDbm);
    }
    /* An identity in this frame that no announce ever named is filed on the
     * node anyway, as a hash that means it.
     *
     * THE TRANSPORT IDENTITY IS THE ONE THAT MATTERS, and it is the one no
     * announce can name: it is a key of its own, distinct from the identity a
     * node's destinations hang off, and it is never announced. It appears on
     * the air in exactly two places — the first address field of every packet
     * relayed towards it, which is what makes it a tag, and this frame, which
     * is what makes it attributable. A node's own row holds it (it learns it
     * from its own relaying), so its announcement carries it; dropping it here
     * for want of a row to match leaves every packet in transit through that
     * neighbour resolving to nobody, and transit is most of what a gateway
     * carries. The addresses a node announces resolve on their own and never
     * reach this. */
    if (keep) {
        for (uint8_t o = 0; o < nOrphan; o++)
            peersHashAdd(r->nei, keep, a.ids[orphan[o]], now);
    }
    if (matched == 0 && logIsDebug(TAG))
        dbg("lora/%d supe: announcement from %02x%02x%02x matches no known node",
            r->idx, a.ids[0][0], a.ids[0][1], a.ids[0][2]);
}

/* Give the engine a name to sign its seeds with: this node's first identity,
 * truncated to a tag. The same list the announcement is built from, so what a
 * HAIL claims is what the neighbourhood has already filed against us — and
 * what `tagNode` can resolve it back through. Identities arrive after the radio
 * does (rnsd registers them as it comes up), so this is a poll rather than a
 * one-off, and it stops looking once it has one. */
static void supeIdentRefresh(LoraRadio* r) {
    if (!r->supeNameSender || r->supe->eng.haveOwnIdent || !r->nei) return;
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &r->nei->nei[i];
        if (!e->used || !peersIsLocal(e) || e->nIds == 0) continue;
        supeEngSetIdent(&r->supe->eng, e->ids[0]);
        return;
    }
}

void supeAnnArm(LoraRadio* r) {
    if (!supeMounted(r)) return;
    r->supe->annPending = true;
    r->supe->annTryMs   = millis();
}

void supeAnnCancel(LoraRadio* r) {
    if (r->supe) r->supe->annPending = false;
}

void supeAnnSoon(LoraRadio* r) {
    if (!supeMounted(r)) return;
    SupeState* ss = r->supe;
    /* Already inside a window, or already about to speak: fold in. */
    if (ss->annSoonPend || ss->annPending) return;
    ss->annSoonPend = true;
    ss->annSoonMs   = millis() + SUPE_ANN_SOON_MS;
}

static uint32_t supeAnnGap(const LoraRadio* r) {
    uint32_t base = (uint32_t)r->annIntervalMin * 60u * 1000u;
    if (!base) return 0;
    uint32_t jit = base / 100u * ANN_JITTER_PCT;
    return base - jit + (jit ? (esp_random() % (2u * jit)) : 0u);
}

/* Every identity this node holds, in one frame — ours and the attached
 * client's alike: a packet addressed to either terminates at this
 * transmitter. */
static void supeAnnBeat(LoraRadio* r, uint32_t now);

/* Build this node's announcement: every identity it holds, in one frame.
 * Returns the frame length, 0 when there is nothing to say yet. */
static size_t supeAnnBuild(LoraRadio* r, uint8_t* f, size_t cap, uint8_t* countOut) {
    if (!r->nei) return 0;
    uint8_t ids[SUPE_ANN_MAX][SUPE_ID_LEN];
    uint8_t count = 0;
    for (int i = 0; i < NEI_MAX && count < SUPE_ANN_MAX; i++) {
        Neighbor* e = &r->nei->nei[i];
        if (!e->used || !peersIsLocal(e)) continue;
        for (int k = 0; k < e->nIds && count < SUPE_ANN_MAX; k++)
            memcpy(ids[count++], e->ids[k], SUPE_ID_LEN);
    }
    if (countOut) *countOut = count;
    if (count == 0) return 0;
    return supeEngBuildAnn(&r->supe->eng, f, cap, ids, count, r->cfgTxp, r->supeOn);
}

static void supeAnnFire(LoraRadio* r, const uint8_t* f, size_t n, uint8_t count) {
    SupeState* ss = r->supe;
    ss->annPending = false;
    ss->annSoonPend = false;
    ss->annNextMs = millis() + supeAnnGap(r);   /* only a sent frame paces the beat */
    /* Broadcast, at the configured power, through the ordinary frame path.
     * The engine is idle (the beat stands off on it), so engineTx marks it and
     * the completion simply re-arms receive. */
    hTxFrame(r, f, (uint16_t)n, r->cfgTxp);
    if (logIsVerbose(TAG))
        verb("lora/%d supe: announced %u identit%s (%uB)", r->idx,
            (unsigned)count, count == 1 ? "y" : "ies", (unsigned)n);
}

static void supeAnnSend(LoraRadio* r) {
    SupeState* ss = r->supe;
    ss->annPending = false;
    /* Whatever opened a coalescing window is answered by this frame, however
     * the frame came to be sent. */
    ss->annSoonPend = false;
    uint8_t f[SUPE_MAX_FRAME];
    uint8_t count = 0;
    size_t n = supeAnnBuild(r, f, sizeof f, &count);
    if (!n) {
        ss->annNextMs = millis() + SUPE_ANN_FIRST_MS;
        if (logIsDebug(TAG))
            dbg("lora/%d supe: no identity to announce yet — retrying shortly", r->idx);
        return;
    }
    supeAnnFire(r, f, n, count);
}

/* The announcement that closes a replay run, fired from the last replayed
 * announce's completion without a polite wait of its own (lora_bridge
 * annTrainChain) — if it fits what is left of the run's air budget. */
bool supeAnnTrainFire(LoraRadio* r, uint32_t budgetMs) {
    if (!supeMounted(r) || !r->running) return false;
    SupeState* ss = r->supe;
    if (!ss->annPending || supeEngBusy(&ss->eng)) return false;
    uint8_t f[SUPE_MAX_FRAME];
    uint8_t count = 0;
    size_t n = supeAnnBuild(r, f, sizeof f, &count);
    if (!n) return false;                    /* the beat will retry on its own */
    uint32_t toa = (uint32_t)lround(1000.0 * loraAirtimeSeconds(
                       r->cfgSf, r->cfgBwHz, r->cfgCr, r->cfgPreamble, (int)n, false));
    if (toa > budgetMs) return false;
    supeAnnFire(r, f, n, count);
    return r->txActive;
}

/* The announce beat, which runs whether or not this node speaks the protocol:
 * a silent node's announcement is what tells its neighbours to stop trying to
 * meet it, and it has to keep saying so, because a neighbour that boots later
 * or misses the one frame would otherwise hold the wrong belief indefinitely.
 * One short frame per interval is a cheap price for that convergence. */
static void supeAnnBeat(LoraRadio* r, uint32_t now) {
    SupeState* ss = r->supe;
    /* The coalescing window closed: the announces that opened it are on the
     * air, so say who we are. Ahead of the interval check because this is the
     * same request arriving early — an announcement sent here paces the beat
     * like any other (supeAnnSend), so the interval is not also served. */
    if (ss->annSoonPend && (int32_t)(now - ss->annSoonMs) >= 0) {
        ss->annSoonPend = false;
        supeAnnArm(r);
    }
    if (!ss->annPending && !r->annReplay && r->annIntervalMin &&
        !ss->eng.expired && (int32_t)(now - ss->annNextMs) >= 0)
        supeAnnArm(r);
    if (!ss->annPending) return;
    if (csmaClear(r)) {
        supeAnnSend(r);
        r->txWaitMs = csmaGrantWaitMs(r);   /* same restatement as the hail's */
    } else if (r->lbtTimeoutMs && now - ss->annTryMs > r->lbtTimeoutMs) {
        ss->annPending = false;
        ss->annNextMs  = now + supeAnnGap(r);
        csmaResetAccess(r);
        if (logIsDebug(TAG))
            dbg("lora/%d supe: announce gave up on a busy channel", r->idx);
    }
}

/* ─────────────── the boundary's entry points ─────────────── */

void supeOnFrame(LoraRadio* r, const uint8_t* f, size_t len,
                 int16_t rssi, int16_t snr10) {
    if (!supeMounted(r)) return;
    supeLock(r);
    s_hosts[r->idx].logLevel = supeLogLevel();
    if (len >= 1 && f[0] == SUPE_T_ANNOUNCE) {
        /* Always read: this is the world picture, not an invitation. */
        r->supe->eng.rxFrames++;
        annIngest(r, f, len, rssi, snr10);
    } else if (supeReady(r)) {
        /* Everything else IS an invitation, and answering one is speaking. */
        uint32_t discards = r->supe->eng.rxDiscard;
        supeEngOnRx(&r->supe->eng, f, (uint16_t)len, rssi, snr10);
        /* A frame that carries one of our type bytes and does not decode is
         * either a corrupt frame that passed its CRC or bytes that were never
         * on the air as read — a buffer read at the wrong place. Both are
         * rare and both matter, so the evidence is kept: the bytes, and
         * where the meeting was. */
        if (r->supe->eng.rxDiscard != discards) {
            char hex[3 * 8 + 1];
            size_t n = len < 8 ? len : 8;
            for (size_t i = 0; i < n; i++)
                snprintf(hex + 3 * i, sizeof hex - 3 * i, "%02x ", f[i]);
            warn("lora/%d supe: undecodable type 0x%02x %uB on ch%u phase %u: %s",
                 r->idx, f[0], (unsigned)len, (unsigned)r->chNow,
                 (unsigned)r->supe->eng.m.phase, hex);
        }
    }
    supeUnlock(r);
}

bool supeTrainCapture(LoraRadio* r, const uint8_t* frame, size_t len,
                      int16_t rssi, int16_t snr10) {
    if (!supeReady(r) || !supeXactLive(r)) return false;
    if (len == 0 || len > 1 + RNODE_MAX_PAYLOAD) return false;
    supeLock(r);
    SupeState* ss = r->supe;
    bool took = false;
    if (ss->rxTCount < SUPE_TRAIN_MAX) {
        uint8_t csum = supeCrc8(frame, len);
        s_hosts[r->idx].logLevel = supeLogLevel();
        if (supeEngOnTrainFrame(&ss->eng, csum, rssi, snr10)) {
            memcpy(ss->rxT[ss->rxTCount], frame, len);
            ss->rxTLen[ss->rxTCount]   = (uint16_t)len;
            ss->rxTRssi[ss->rxTCount]  = rssi;
            ss->rxTSnr10[ss->rxTCount] = snr10;
            ss->rxTCount++;
            took = true;
        }
    }
    supeUnlock(r);
    return took;
}

bool supeAfterTx(LoraRadio* r) {
    if (!r->supe || !r->supe->engineTx) return false;
    supeLock(r);
    r->supe->engineTx = false;
    s_hosts[r->idx].logLevel = supeLogLevel();
    if (supeEngBusy(&r->supe->eng)) {
        supeEngOnTxDone(&r->supe->eng, !r->txAborted);
        supeUnlock(r);
        return true;     /* the engine re-armed receive, or fired the next frame */
    }
    supeUnlock(r);
    return false;        /* the announce beat's frame: the caller re-arms */
}

uint8_t supeHeadVerdict(LoraRadio* r) {
    if (!supeReady(r)) return SUPE_V_PLAIN;
    s_hosts[r->idx].logLevel = supeLogLevel();
    return supeEngVerdict(&r->supe->eng);
}

void supePoll(LoraRadio* r) {
    if (!supeMounted(r) || !r->running) return;
    if (!supeTryLock(r)) return;    /* another caller has it; nothing here waits */
    SupeState* ss = r->supe;
    SupeEngine* e = &ss->eng;
    uint32_t now = millis();
    s_hosts[r->idx].logLevel = supeLogLevel();

    /* Switched off: no meetings, no seeds, no slots — but an identity to find
     * and an announcement to keep making, which is the whole of what a silent
     * node owes the neighbourhood. */
    if (!r->supeOn) {
        supeIdentRefresh(r);
        if (r->mtxPhase == MTXP_OFF && !r->txActive) supeAnnBeat(r, now);
        supeUnlock(r);
        return;
    }

    supeEngTagExpire(e, now);
    supeIdentRefresh(r);
    airtimePoll(r);
    /* The dialect deadline is days out, so it re-checks at most hourly, riding
     * whatever pass other work causes — it holds no wake of its own, and a
     * node idle past its expiry catches up on the announce beat's pass before
     * anything would speak. */
    if ((int32_t)(now - ss->expiryNextMs) >= 0) {
        ss->expiryNextMs = now + 3600u * 1000u;
        bool was = e->expired;
        e->expired = supeExpired((uint32_t)time(nullptr));
        if (e->expired && !was)
            warn("lora/%d SUPE dialect expired — no longer speaking it; reflash", r->idx);
    }

    /* Hard watchdog: a meeting that outlives every deadline inside it holds
     * the radio against the whole outbound queue. */
    if (e->m.phase >= SUPE_M_GOT_TX &&
        (uint32_t)(now - e->m.beganMs) > SUPE_MEET_WATCHDOG_MS) {
        warn("lora/%d supe: meeting stuck in phase %u — standing down",
             r->idx, (unsigned)e->m.phase);
        supeEngAbort(e, "watchdog");
    }

    /* A transmit of our own in flight is the one thing the service must not run
     * under: it would retune the chip out from beneath a frame still going out.
     * A half-assembled split is NOT that — it reaches the engine as rx_busy
     * instead, which defers attending a slot while letting schedules expire and
     * dead slots be walked past. Standing the whole service down for it meant
     * the engine's next event stayed pinned at a slot already in the past, so
     * the deadline read zero and the main loop spun on it for the reassembly
     * timeout — five seconds of a hot core, no schedule able to retire, and a
     * peer hailing into a node that had stopped keeping its own appointments. */
    if (r->txActive) { supeUnlock(r); return; }

    /* Service the engine's due deadlines here, on this task (see supeTimerCb),
     * and only with the radio unclaimed — a due slot must not retune the chip
     * out from under a frame of our own still on the air. Bounded: each pass
     * either advances the state or clears the deadline that made it due, and
     * anything left re-arms the timer, whose firing wakes this task straight
     * back into this loop. */
    for (int i = 0; i < 4; i++) {
        uint32_t at = supeEngNextEventMs(e, now);
        if (at == UINT32_MAX || (int32_t)(now - at) < 0) break;
        supeEngOnTimer(e);
        if (r->txActive) break;         /* the service just started a transmit */
        now = millis();
    }

    /* Settings are still moving: start nothing. A meeting already running is
     * left to finish — the far end is timing against it and it is over in well
     * under the window — but no new seed goes out and the announce beat below
     * holds, because both would put this node on the air at a configuration it
     * is about to leave. The offer stays armed and launches when the window
     * closes. */
    if (loraCfgQuiet()) { supeUnlock(r); return; }

    /* The launch: the verdict armed a seed, the jitter has passed, and the
     * HAIL contends for the shared medium like any other frame. */
    if (supeEngLaunchDue(e)) {
        if (csmaClear(r)) {
            supeEngLaunch(e);
            /* The HAIL won the medium for itself; hTxFrame zeroed the wait
             * mark, so what channel access just cost is restated for the
             * record — otherwise the graph shows a seed that never waited. */
            r->txWaitMs = csmaGrantWaitMs(r);
        } else if (r->lbtTimeoutMs && e->offerArmed &&
                   now - e->offerJitterUntilMs > r->lbtTimeoutMs) {
            /* A channel that never frees must not hold the queue behind a
             * hail forever: give it up, the packet takes the main channel on
             * the ordinary path. An owed hail that cannot win the channel is
             * simply left to expire at patience. */
            e->offerArmed = false;
            e->plainOnce = true;
            csmaResetAccess(r);
        }
        supeUnlock(r);
        return;
    }
    if (e->m.phase != SUPE_M_IDLE) { supeUnlock(r); return; }

    /* The announce beat. A replay run ends with our own announcement, so the
     * beat stands off until it is over — otherwise a beat that is already due
     * fires here, and since annPending gates the drain it would speak ahead of
     * the announces the replay has queued. */
    if (r->mtxPhase != MTXP_OFF) { supeUnlock(r); return; }
    supeAnnBeat(r, now);
    supeUnlock(r);
}

uint32_t supeNextDeadlineMs(LoraRadio* r) {
    if (!supeReady(r) || !r->running) return UINT32_MAX;
    /* An armed seed keeps asking to be re-sensed at slot pace, and during a
     * settle window supePoll will do nothing with it — so it must not hold a
     * wake either. The window's own deadline is the task's. */
    if (loraCfgQuiet()) return UINT32_MAX;
    SupeState* ss = r->supe;
    uint32_t now = millis(), best = UINT32_MAX;
    auto soon = [&](uint32_t at) {
        int32_t rem = (int32_t)(at - now);
        uint32_t d = rem > 0 ? (uint32_t)rem : 0;
        if (d < best) best = d;
    };
    uint32_t slotMs = (uint32_t)(r->slotTicks * portTICK_PERIOD_MS);
    if (slotMs == 0) slotMs = 1;
    if (ss->eng.offerArmed) {
        if ((int32_t)(now - ss->eng.offerJitterUntilMs) < 0)
            soon(ss->eng.offerJitterUntilMs);
        else if (slotMs < best) best = slotMs;
    }
    /* The engine's own clock: the next slot edge, window close, meeting
     * deadline or schedule expiry. The esp_timer carries these too; this keeps
     * the task's own wake honest about them. */
    {
        uint32_t at = supeEngNextEventMs(&ss->eng, now);
        if (at != UINT32_MAX) soon(at);
    }
    if (ss->annPending) { if (slotMs < best) best = slotMs; }
    else if (r->annIntervalMin && ss->eng.m.phase == SUPE_M_IDLE) soon(ss->annNextMs);
    /* Paced like annPending once it comes due, not `soon(0)`. supePoll returns
     * ahead of the beat while a transmit is in flight or an announce replay is
     * running, so a window that has expired can stay unserviced for a while —
     * and a deadline of zero over that stretch is the main loop spinning on a
     * request nothing is in a position to grant. */
    if (ss->annSoonPend) {
        if ((int32_t)(now - ss->annSoonMs) >= 0) { if (slotMs < best) best = slotMs; }
        else soon(ss->annSoonMs);
    }
    {
        uint32_t d = airtimeNextDeadlineMs(r, now);
        if (d < best) best = d;
    }
    return best;
}

/* ── the observer's feeds, engine-bound through the boundary ── */

void supeTagAdd(LoraRadio* r, const uint8_t* addr, bool perm, uint32_t ttlMs) {
    if (!supeReady(r)) return;
    supeLock(r);
    supeEngTagAdd(&r->supe->eng, addr, perm, ttlMs);
    supeUnlock(r);
}

void supeTagRelease(LoraRadio* r, const uint8_t* addr) {
    if (!supeReady(r)) return;
    supeLock(r);
    supeEngTagRelease(&r->supe->eng, addr);
    supeUnlock(r);
}

void supeProofRetFile(LoraRadio* r, const uint8_t phash[16], const uint8_t node4[4]) {
    if (!supeReady(r)) return;
    supeLock(r);
    supeEngProofRetFile(&r->supe->eng, phash, node4);
    supeUnlock(r);
}

/* `lora forget` reaching the protocol (lora_supe.h). Every address the row
 * answers to is offered, because the engine files against whichever one it saw:
 * a node key from an announcement, an identity, a destination hash. Not gated
 * on `supeReady` — a radio that has stopped speaking SUPE still HOLDS what it
 * learned, and leaving that behind is exactly the memory this is here to drop. */
void supeForgetPeer(LoraRadio* r, const Neighbor* e) {
    if (!supeMounted(r) || !e) return;
    supeLock(r);
    if (e->haveNode4) supeEngForget(&r->supe->eng, e->node4);
    for (int i = 0; i < e->nIds; i++)   supeEngForget(&r->supe->eng, e->ids[i]);
    for (int d = 0; d < e->nDests; d++) supeEngForget(&r->supe->eng, e->dests[d].hash);
    supeUnlock(r);
}

#endif  /* CONFIG_LORA0_CS_PIN */
