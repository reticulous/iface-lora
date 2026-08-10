/**
 * lora_supe — the platform half of SUPE: the SupeHost implementation the pure
 * engine (supe_engine.{h,cpp}) runs against on this device, the boundary lock
 * that serialises every entry point, the ANNOUNCE2 beat and its peer-table
 * ingest, and the adapters that file the engine's peer notes into the peer
 * table and the power controller.
 */
#include "lora_priv.h"
#include "lora_fem.h"

#if defined(CONFIG_LORA0_CS_PIN)

/* How soon after a radio comes up it first announces itself, jittered over
 * twice this. Nothing can request a detour of a node it has never heard
 * announce, so the first one is worth having promptly. */
#define SUPE_ANN_FIRST_MS    15000

/* The absence ladder's verdict (§11): after three silent requests the peer is
 * absent this long, and its traffic is dropped rather than transmitted into
 * the void. */
#define SUPE_ABSENT_MS       60000
#define SUPE_ABSENT_STRIKES  3

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
        for (int l = 0; l < e->nLink4; l++)
            if (memcmp(e->link4[l], tag, SUPE_TAG_LEN) == 0) return e;
        /* Identities, and not only the destinations derived from them: a
         * START's sender_ident names the asking node by its identity, and
         * nothing is ever *addressed* to an identity, so it appears in none of
         * the lists above. */
        for (int k = 0; k < e->nIds; k++)
            if (memcmp(e->ids[k], tag, SUPE_TAG_LEN) == 0) return e;
    }
    /* A link identifier we initiated resolves to the node the link request was
     * addressed to (SUPE.md §5.1, §10 "links inherit") — this is what keeps a
     * session's traffic detouring after rnsd switches from the destination
     * hash to the link id. A link dialled TO us resolves to nobody by
     * construction (its initiator is anonymous); that case is the link branch
     * of peer_get, not this one. */
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
    uint32_t d = (int32_t)(at_ms - now) > 0 ? at_ms - now : 0;
    esp_timer_stop(r->supe->timer);
    esp_timer_start_once(r->supe->timer, (uint64_t)d * 1000ull);
}

static void hSha256(void*, const uint8_t* d, uint16_t n, uint8_t out[32]) {
    rnsdSha256(d, n, out);
}

/* Move to a granted channel and configuration. The way back is the hailing
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
    csmaFloorSwitch(r, r->chNow, chan);
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
    r->radio->setOutputPower(femChipDbm(r, r->cfgTxp));
    r->airSf       = (uint8_t)r->cfgSf;
    r->airBwHz     = r->cfgBwHz;
    r->airPreamble = r->cfgPreamble;
    r->airImplicit = false;
    r->txPwrNow    = r->cfgTxp;
    /* The detour's floor is not this channel's — but the hailing channel's own,
     * from before the detour, is the best estimate in the system and the one a
     * node returning home has to contend against straight away. */
    csmaFloorSwitch(r, r->chNow, LORA_CH_HAIL);
    r->chNow       = LORA_CH_HAIL;
    radioStartRx(r);
    if (logIsDebug(TAG)) dbg("lora/%d supe: home", r->idx);
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
    startTxFrame(r, 0);
    return r->txActive;
}

/* The train pipeline: the packet's frames, observer tap and fan-out are all
 * built at stage time — during the previous packet's airtime — so the fire
 * is a buffer copy and a startTransmit. The engine consumes the queue entry
 * the moment the stage returns. */
static bool hStagePacket(void* ctx, const LoraPkt* p, int8_t dbm) {
    LoraRadio* r = (LoraRadio*)ctx;
    return stageTx(r, p->bytes, p->len, p->flags & LORAQ_ORIG_MASK, dbm);
}

static bool hFireStaged(void* ctx) {
    LoraRadio* r = (LoraRadio*)ctx;
    r->supe->engineTx = true;
    if (!fireStagedTx(r)) {
        r->supe->engineTx = false;
        return false;
    }
    return true;
}

static void hRx(void* ctx) {
    rearmRx((LoraRadio*)ctx);
}

/* The modem's own account of whether a frame is arriving: a preamble it has
 * locked, or a header it has validated and is still filling in. */
static bool hRxBusy(void* ctx) {
    return radioRxInProgress((LoraRadio*)ctx);
}

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
        out->adaptive = e->supeCaps.adaptive;
        out->txpOpen = apOpenPower(r, e);
        out->absentStrikes = e->silentCount;
        out->absentUntilMs = e->absentUntilMs;
        out->retryWaitUntilMs = e->retryWaitUntilMs;
        out->backoffUntilMs = e->backoffUntilMs;
        out->detoured = e->detoured;
        return true;
    }
    /* The one peer we can never name: the node that dialled a link to us. Its
     * capabilities were filed against the link (SUPE.md §10, links inherit). */
    NeiLink* L = tagLink(r, tag);
    if (L && L->ours && L->supeSeen) {
        out->known = true;
        out->fam = L->supeCaps.fam;
        out->topBudget = L->supeCaps.topStep;
        out->maxTxpDbm = L->supeCaps.maxPwrDbm;
        out->adaptive = L->supeCaps.adaptive;
        return true;
    }
    return false;
}

static void hPeerNote(void* ctx, const uint8_t tag[SUPE_TAG_LEN],
                      const SupePeerNote* n) {
    LoraRadio* r = (LoraRadio*)ctx;
    Neighbor* e = tagNode(r, tag);
    /* On the answering side the tag is one of our own addresses — there is no
     * node row to file against, and our own local row must not collect the
     * requester's measurements. What can hold them is the link (below). */
    if (e && peersIsLocal(e)) e = nullptr;
    NeiLink*  L = e ? nullptr : tagLink(r, tag);
    uint32_t now = millis();
    switch (n->ev) {
        case SUPE_EV_ALIVE:
            if (e) {
                e->silentCount = 0;
                e->absentUntilMs = 0;
                e->retryWaitUntilMs = 0;
                e->supeHeardMs = now;
            }
            break;
        case SUPE_EV_STRIKE:
            if (e) {
                if (e->silentCount < 255) e->silentCount++;
                /* From the request, not from the strike: `agoMs` is what the
                 * GRANT deadline already spent, and only the remainder is owed. */
                e->retryWaitUntilMs = now + (n->backoffMs > n->agoMs
                                             ? n->backoffMs - n->agoMs : 0);
                if (e->silentCount >= SUPE_ABSENT_STRIKES) {
                    e->absentUntilMs = now + SUPE_ABSENT_MS;
                    if (logIsDebug(TAG))
                        dbg("lora/%d supe: %02x%02x%02x absent for %us",
                            r->idx, tag[0], tag[1], tag[2], SUPE_ABSENT_MS / 1000);
                }
            }
            break;
        case SUPE_EV_REFUSED:
            if (e) {
                e->backoffUntilMs = now + n->backoffMs;
                e->silentCount = 0;
                e->absentUntilMs = 0;
            }
            break;
        case SUPE_EV_PAIR:
            /* Never a bare level: a level measured here with the power the
             * other side stated for it. Filed against the hailing pair or the
             * detour pair by the configuration it was read at. */
            if (e) {
                bool hail = (n->cfg.sf == (uint8_t)r->cfgSf &&
                             n->cfg.bwHz == (uint32_t)r->cfgBwHz);
                supeFilePair(r, e, n->rssiDbm, n->txpDbm, hail ? 0 : 1);
                e->supeHeardMs = now;
            } else if (L && L->ours) {
                L->havePair = true;
                L->pairRssi = n->rssiDbm;
                L->pairTxp  = n->txpDbm;
                L->supeHeardMs = now;
            }
            break;
        case SUPE_EV_TRAIN_OK:
            if (e) {
                e->haveApLastTxp = true;
                e->apLastTxp = n->txpDbm;
                /* Walk the offset down only on reported headroom — thin margin
                 * holds (§15). */
                int marginDeci = (int)n->rssiDbm * 10 - (int)supeSensitivityDeci(&n->cfg);
                if (marginDeci > (SUPE_TARGET_MARGIN_DB + 3) * 10)
                    supeApSucceeded(r, e);
            }
            break;
        case SUPE_EV_TRAIN_LOST:
            if (e) supeApFailed(r, e, n->triedTxpDbm);
            break;
        case SUPE_EV_DETOURED:
            if (e) { e->detoured = true; e->supeHeardMs = now; }
            break;
        case SUPE_EV_CAPS:
            /* A MANIFEST carries the sender's capabilities unconditionally
             * (§8). Where the tag was a link identifier, this is the one
             * handle on the peer that dialled us — filing them is what lets
             * reverse traffic on that link detour at all (§10). On a node row
             * it is simply fresher than the last announcement. */
            if (e) {
                e->supeSeen    = true;
                e->supeCaps    = n->caps;
                e->supeHeardMs = now;
                e->ourProto    = true;
            } else if (L && L->ours) {
                L->supeSeen    = true;
                L->supeCaps    = n->caps;
                L->supeHeardMs = now;
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

static void hLog(void* ctx, const char* msg) {
    LoraRadio* r = (LoraRadio*)ctx;
    dbg("lora/%d %s", r->idx, msg);
}

static void supeTimerCb(void* arg) {
    LoraRadio* r = (LoraRadio*)arg;
    supeLock(r);
    if (r->supe) {
        s_hosts[r->idx].dbgLevel = logIsDebug(TAG);
        supeEngOnTimer(&r->supe->eng);
    }
    supeUnlock(r);
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
    h->stage_packet = hStagePacket;
    h->fire_staged  = hFireStaged;
    h->rx        = hRx;
    h->rx_busy   = hRxBusy;
    h->peer_get  = hPeerGet;
    h->peer_note = hPeerNote;
    h->chan_get  = hChanGet;
    h->log       = hLog;
    h->dbgLevel  = false;
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
    c.adaptive  = r->supeAdaptive;
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
                  r->supeAdaptive, (uint8_t)r->cfgSf, (uint32_t)r->cfgBwHz,
                  (uint8_t)r->cfgCr, (uint16_t)r->cfgPreamble, r->cfgSync,
                  RNODE_MAX_PAYLOAD);
    /* The access procedure's fixed interval, for the hold's early release. */
    r->supe->eng.holdEarlyMs = (uint16_t)((r->appc ? r->appcDifsTicks
                                                   : r->difsTicks)
                                          * portTICK_PERIOD_MS);
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
    r->txStageCount = 0;           /* a staged packet dies with the radio */
}

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

/* ─────────────── ANNOUNCE2 (SUPE.md §7) ─────────────── */

static void annIngest(LoraRadio* r, const uint8_t* f, size_t len, int16_t rssi) {
    SupeAnn2 a;
    if (!supeDecAnn2(f, len, &a)) return;
    if (!r->nei) return;
    uint32_t now = millis();
    int matched = 0;
    for (int i = 0; i < a.count; i++) {
        Neighbor* e = peersFindByIdent4(r->nei, a.ids[i]);
        if (!e) e = peersFindBy4(r->nei, a.ids[i]);
        if (!e || peersIsLocal(e)) continue;
        matched++;
        bool first = !e->supeSeen;
        e->supeSeen    = true;
        e->supeCaps    = a.caps;
        e->supeHeardMs = now;
        e->ourProto    = true;
        /* Evidence of life cancels absence outright (§11). */
        e->silentCount = 0;
        e->absentUntilMs = 0;
        e->retryWaitUntilMs = 0;
        /* The power byte is what makes the frame worth hearing: the reading
         * and the stated power together are path loss, not a bare level. */
        supeFilePair(r, e, rssi, a.pwrDbm, 0);
        if (first && logIsDebug(TAG))
            dbg("lora/%d supe: %02x%02x%02x speaks SUPE (family %u, ceiling %u, max %d dBm%s)",
                r->idx, a.ids[i][0], a.ids[i][1], a.ids[i][2],
                (unsigned)a.caps.fam, (unsigned)a.caps.topStep,
                (int)a.caps.maxPwrDbm, a.caps.adaptive ? ", adaptive" : "");
    }
    if (matched == 0 && logIsDebug(TAG))
        dbg("lora/%d supe: announcement from %02x%02x%02x matches no known node",
            r->idx, a.ids[0][0], a.ids[0][1], a.ids[0][2]);
}

/* Give the engine a name to sign its requests with: this node's first identity,
 * truncated to a tag. The same list the announcement is built from, so what a
 * START claims is what the neighbourhood has already filed against us — and
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
    if (!supeReady(r)) return;
    r->supe->annPending = true;
    r->supe->annTryMs   = millis();
}

void supeAnnCancel(LoraRadio* r) {
    if (r->supe) r->supe->annPending = false;
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
static void supeAnnSend(LoraRadio* r) {
    SupeState* ss = r->supe;
    ss->annPending = false;
    auto retrySoon = [&]() {
        ss->annNextMs = millis() + SUPE_ANN_FIRST_MS;
        if (logIsDebug(TAG))
            dbg("lora/%d supe: no identity to announce yet — retrying shortly", r->idx);
    };
    if (!r->nei) { retrySoon(); return; }
    uint8_t ids[SUPE_ANN2_MAX][SUPE_ID_LEN];
    uint8_t count = 0;
    for (int i = 0; i < NEI_MAX && count < SUPE_ANN2_MAX; i++) {
        Neighbor* e = &r->nei->nei[i];
        if (!e->used || !peersIsLocal(e)) continue;
        for (int k = 0; k < e->nIds && count < SUPE_ANN2_MAX; k++)
            memcpy(ids[count++], e->ids[k], SUPE_ID_LEN);
    }
    if (count == 0) { retrySoon(); return; }
    uint8_t f[SUPE_MAX_FRAME];
    size_t n = supeEngBuildAnn(&ss->eng, f, sizeof f, ids, count, r->cfgTxp);
    if (!n) { retrySoon(); return; }
    ss->annNextMs = millis() + supeAnnGap(r);   /* only a sent frame paces the beat */
    /* Broadcast, at the configured power, through the ordinary frame path.
     * The engine is idle (the beat stands off on it), so engineTx marks it and
     * the completion simply re-arms receive. */
    hTxFrame(r, f, (uint16_t)n, r->cfgTxp);
    if (logIsDebug(TAG))
        dbg("lora/%d supe: announced %u identit%s (%uB)", r->idx,
            (unsigned)count, count == 1 ? "y" : "ies", (unsigned)n);
}

/* ─────────────── the boundary's entry points ─────────────── */

void supeOnFrame(LoraRadio* r, const uint8_t* f, size_t len,
                 int16_t rssi, int16_t snr10) {
    if (!supeReady(r)) return;
    supeLock(r);
    s_hosts[r->idx].dbgLevel = logIsDebug(TAG);
    if (len >= 1 && f[0] == SUPE_T_ANNOUNCE2) {
        r->supe->eng.rxFrames++;
        annIngest(r, f, len, rssi);
    } else {
        supeEngOnRx(&r->supe->eng, f, (uint16_t)len, rssi, snr10);
    }
    supeUnlock(r);
}

void supeOnPacketRx(LoraRadio* r, int16_t rssi, int16_t snr10) {
    if (!supeBusy(r)) return;
    supeLock(r);
    s_hosts[r->idx].dbgLevel = logIsDebug(TAG);
    supeEngOnPacketRx(&r->supe->eng, rssi, snr10);
    supeUnlock(r);
}

bool supeAfterTx(LoraRadio* r) {
    if (!r->supe || !r->supe->engineTx) return false;
    supeLock(r);
    r->supe->engineTx = false;
    s_hosts[r->idx].dbgLevel = logIsDebug(TAG);
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
    s_hosts[r->idx].dbgLevel = logIsDebug(TAG);
    return supeEngVerdict(&r->supe->eng);
}

void supePoll(LoraRadio* r) {
    if (!supeReady(r) || !r->running) return;
    if (!supeTryLock(r)) return;    /* the timer task has it; nothing here waits */
    SupeState* ss = r->supe;
    SupeEngine* e = &ss->eng;
    uint32_t now = millis();
    s_hosts[r->idx].dbgLevel = logIsDebug(TAG);

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

    /* Hard watchdog: a transaction that outlives every deadline inside it
     * holds the radio against the whole outbound queue. */
    if (e->x.phase != SUPE_X_IDLE) {
        uint32_t cap = e->x.durMs ? e->x.durMs : SUPE_DUR_MAX_MS;
        if ((uint32_t)(now - e->x.beganMs) > cap + 2000u) {
            warn("lora/%d supe: transaction stuck in phase %u — standing down",
                 r->idx, (unsigned)e->x.phase);
            if (!e->x.role_b) e->plainOnce = true;
            supeEngAbort(e, "watchdog");
        }
    }

    if (r->txActive || r->splitPending) { supeUnlock(r); return; }

    /* A START going back out: nothing was on the air when the answer was due,
     * so the request is repeated inside the same transaction. It owes the
     * medium the same access the first one did — and that backoff is also the
     * decorrelation a retransmission wants, so there is no separate jitter. */
    if (supeEngResendDue(e)) {
        if (csmaClear(r)) {
            supeEngResend(e);
            r->txWaitMs = csmaGrantWaitMs(r);
        } else if (r->lbtTimeoutMs &&
                   (uint32_t)(now - e->x.deadlineMs) > r->lbtTimeoutMs) {
            /* A channel that will not free is not evidence about the peer, but
             * the request cannot wait for it forever either. */
            supeEngResendDrop(e);
            csmaResetAccess(r);
        }
        supeUnlock(r);
        return;
    }

    /* The launch: the verdict armed an offer, the jitter has passed, and the
     * START contends for the shared medium like any other frame. */
    if (supeEngLaunchDue(e)) {
        if (csmaClear(r)) {
            supeEngLaunch(e);
            /* The START won the medium for itself; hTxFrame zeroed the wait
             * mark, so what channel access just cost is restated for the
             * record — otherwise the graph shows a START that never waited. */
            r->txWaitMs = csmaGrantWaitMs(r);
        } else if (r->lbtTimeoutMs && now - e->offerJitterUntilMs > r->lbtTimeoutMs) {
            /* A channel that never frees must not hold the queue behind an
             * offer forever: give the detour up, the packet takes the main
             * channel on the ordinary path. */
            e->offerArmed = false;
            e->plainOnce = true;
            csmaResetAccess(r);
        }
        supeUnlock(r);
        return;
    }
    if (e->x.phase != SUPE_X_IDLE) { supeUnlock(r); return; }

    /* The announce beat. A replay run ends with our own announcement, so the
     * beat stands off until it is over — otherwise a beat that is already due
     * fires here, and since annPending gates the drain it would speak ahead of
     * the announces the replay has queued. */
    if (r->mtxPhase != MTXP_OFF) { supeUnlock(r); return; }
    if (!ss->annPending && !r->annReplay && r->annIntervalMin && !e->expired &&
        (int32_t)(now - ss->annNextMs) >= 0)
        supeAnnArm(r);
    if (ss->annPending) {
        if (csmaClear(r)) {
            supeAnnSend(r);
            r->txWaitMs = csmaGrantWaitMs(r);   /* same restatement as the START's */
        } else if (r->lbtTimeoutMs && now - ss->annTryMs > r->lbtTimeoutMs) {
            ss->annPending = false;
            ss->annNextMs  = now + supeAnnGap(r);
            csmaResetAccess(r);
            if (logIsDebug(TAG))
                dbg("lora/%d supe: announce gave up on a busy channel", r->idx);
        }
    }
    supeUnlock(r);
}

uint32_t supeNextDeadlineMs(LoraRadio* r) {
    if (!supeReady(r) || !r->running) return UINT32_MAX;
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
    if (ss->annPending) { if (slotMs < best) best = slotMs; }
    else if (r->annIntervalMin && ss->eng.x.phase == SUPE_X_IDLE) soon(ss->annNextMs);
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

#endif  /* CONFIG_LORA0_CS_PIN */
