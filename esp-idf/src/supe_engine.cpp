/**
 * supe_engine — implementation. Header carries the design; plans/SUPE.md is
 * authoritative for everything on the air.
 *
 * Pure: <stdint.h>, <string.h>, <stdio.h> (log formatting) and the two pure
 * modules it composes, supe.{h,cpp} and lora_queue.{h,cpp}. The host test in
 * test/supe_engine_test.cpp compiles this file directly; if it ever needs an
 * IDF header, something has leaked in that belongs in lora_supe.cpp.
 */
#include "supe_engine.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* The pre-seed jitter, so two seekers that collide once do not collide again
 * in lockstep. */
#define SUPE_OFFER_JITTER_MS   24

/* The absence ladder's randomised wait between seeds — long enough to outlast
 * somebody else's meeting, which is the likeliest cause of the silence. */
#define SUPE_RETRY_WAIT_MIN_MS 400
#define SUPE_RETRY_WAIT_SPAN_MS 400

/* What the transmitting side adds after a retune before speaking, beyond the
 * synthesizer's own gap — scheduling jitter cover, so the far receiver is
 * armed first. Provisional, pending the measurement SUPE.md §16 asks
 * simulation for. */
#define SUPE_TRAIN_LEAD_MS      3

/* Receiver-side slack per expected train frame, added to the derived receive
 * deadline: each real gap carries IRQ, task and timer latency the sender
 * cannot state. Receiver-local; nothing on the wire derives from it. */
#define SUPE_TRAIN_RX_SLACK_MS  20

/* How many times a slot window extends while the modem reports a frame still
 * arriving at its close. */
#define SUPE_WINDOW_EXTENDS_MAX 3

/* ─────────────── small helpers ─────────────── */

static uint32_t eNow(SupeEngine* e) { return e->host->now_ms(e->host->ctx); }

/* `verbose` is the level this line belongs at, not a flag: false = one of the
 * few lines debug keeps, true = the step-by-step behind them. */
static void eLog(SupeEngine* e, bool verbose, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));
static void eLog(SupeEngine* e, bool verbose, const char* fmt, ...) {
    if (!e->host->log) return;
    if (e->host->logLevel < (verbose ? SUPE_LOG_VERB : SUPE_LOG_DBG)) return;
    char b[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    e->host->log(e->host->ctx, verbose, b);
}

static SupeCfg hailCfgOf(const SupeEngine* e) {
    SupeCfg c;
    c.sf = e->hailSf;
    c.bwHz = e->hailBwHz;
    c.ldro = (1000u << e->hailSf) > 16u * e->hailBwHz;
    c.marginDeci = 0;
    return c;
}

/* The maximum bandwidth a channel permits — what a budget resolves against. */
static uint32_t chanMaxBwOf(const SupeEngine* e, uint8_t chan) {
    if (chan == SUPE_CH_HAIL) return e->hailBwHz;
    int n = 0;
    const SupeChan* c = supeRegimeChans(e->regime, &n);
    if (!c || chan > n) return 0;
    return c[chan - 1].bwHz;
}

/* The most this node may transmit at on a channel: the configured power,
 * capped by the regime's regulatory limit off the hailing frequency. */
static int8_t chanTxpCapOf(const SupeEngine* e, uint8_t chan) {
    if (chan == SUPE_CH_HAIL) return e->txpMax;
    const SupeRegime* g = supeRegime(e->regime);
    if (!g || g->maxTxpDbm == SUPE_TXP_IFACE) return e->txpMax;
    return e->txpMax < g->maxTxpDbm ? e->txpMax : g->maxTxpDbm;
}

static uint32_t toaFrameMs(const SupeEngine* e, const SupeCfg* c, int payload,
                           bool crc) {
    double s = supeAirtimeSeconds(c->sf, (int)c->bwHz, e->crDenom, e->preamble,
                                  payload, false, crc);
    return (uint32_t)(s * 1000.0 + 0.999);
}

static uint32_t preambleMs(const SupeEngine* e, const SupeCfg* c) {
    /* (preamble + 4.25) symbols; whole symbols is close enough for a window. */
    uint64_t num = (uint64_t)(e->preamble + 5) * (1u << c->sf) * 1000u;
    return (uint32_t)((num + c->bwHz - 1) / c->bwHz);
}

/* The time our planned train takes at a configuration, flip gaps included. */
static uint32_t trainLenMs(const SupeEngine* e, const SupeTrainInfo* t,
                           uint8_t count, const SupeCfg* c) {
    uint32_t ms = 0;
    for (uint8_t i = 0; i < count; i++)
        ms += toaFrameMs(e, c, t->lens[i], true) + SUPE_TRAIN_GAP_MS;
    return ms;
}

/* The worst a peer's declared train can take — count is all we hold. */
static uint32_t trainWorstMs(const SupeEngine* e, uint8_t count, const SupeCfg* c) {
    return (uint32_t)count
           * (toaFrameMs(e, c, (int)e->maxFrameLen + 1, true) + SUPE_TRAIN_GAP_MS
              + SUPE_TRAIN_RX_SLACK_MS);
}

/* The answer deadline is sized to the largest of the three frames that may
 * answer a THATSIT, since which arrives is the answer itself. */
static uint32_t answerDeadlineMs(const SupeEngine* e, const SupeCfg* c) {
    return SUPE_TURNAROUND_MS
           + toaFrameMs(e, c, SUPE_HAVEDATA_ANS_BASE + SUPE_MASK_MAX, false)
           + SUPE_GUARD_MS;
}

static uint8_t popcount8(const uint8_t* mask, uint8_t maskLen) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < maskLen; i++)
        for (uint8_t b = 0; b < 8; b++)
            if (mask[i] & (1u << b)) n++;
    return n;
}

static inline bool maskGet(const uint8_t* m, uint8_t i) {
    return (m[i >> 3] & (1u << (i & 7))) != 0;
}
static inline void maskSet(uint8_t* m, uint8_t i) { m[i >> 3] |= (uint8_t)(1u << (i & 7)); }

/* ─────────────── addresses that mean us ─────────────── */

static SupeTag* tagFind(SupeEngine* e, const uint8_t* addr) {
    for (int i = 0; i < SUPE_TAGS_MAX; i++) {
        SupeTag* t = &e->tags[i];
        if (t->used && memcmp(t->tag, addr, SUPE_TAG_LEN) == 0) return t;
    }
    return nullptr;
}

void supeEngTagAdd(SupeEngine* e, const uint8_t* addr, bool perm, uint32_t ttlMs) {
    uint32_t now = eNow(e);
    SupeTag* t = tagFind(e, addr);
    if (!t) {
        for (int i = 0; i < SUPE_TAGS_MAX && !t; i++)
            if (!e->tags[i].used) t = &e->tags[i];
        if (!t) {
            SupeTag* victim = nullptr;
            for (int i = 0; i < SUPE_TAGS_MAX; i++) {
                SupeTag* c = &e->tags[i];
                if (c->perm) continue;
                if (!victim || (int32_t)(c->expiryMs - victim->expiryMs) < 0) victim = c;
            }
            if (!victim) return;              /* every entry is ours: keep them */
            t = victim;
        }
        memset(t, 0, sizeof *t);
        memcpy(t->tag, addr, SUPE_TAG_LEN);
        t->used = true;
        eLog(e, true, "supe: tag %02x%02x%02x learned (%s)",
             addr[0], addr[1], addr[2], perm ? "ours" : "transient");
    }
    if (perm) { t->perm = true; return; }
    if (t->refs < 255) t->refs++;
    uint32_t exp = now + ttlMs;
    if (t->refs == 1 || (int32_t)(exp - t->expiryMs) > 0) t->expiryMs = exp;
}

void supeEngTagRelease(SupeEngine* e, const uint8_t* addr) {
    SupeTag* t = tagFind(e, addr);
    if (!t || t->refs == 0) return;
    t->refs--;
    if (t->refs == 0 && !t->perm) t->used = false;
}

void supeEngTagExpire(SupeEngine* e, uint32_t now) {
    for (int i = 0; i < SUPE_TAGS_MAX; i++) {
        SupeTag* t = &e->tags[i];
        if (!t->used || t->perm || t->refs == 0) continue;
        if ((int32_t)(now - t->expiryMs) >= 0) t->used = false;
    }
}

bool supeEngTagIsOurs(const SupeEngine* e, const uint8_t* addr) {
    const SupeTag* t = tagFind((SupeEngine*)e, addr);
    return t && (t->perm || t->refs > 0);
}

void supeEngProofRetFile(SupeEngine* e, const uint8_t phash[16], const uint8_t node4[4]) {
    uint32_t now = eNow(e);
    SupeProofRet* slot = nullptr;
    for (int i = 0; i < SUPE_PROOFRET_MAX; i++) {
        SupeProofRet* r = &e->pret[i];
        if (r->used && (int32_t)(now - r->expiryMs) >= 0) r->used = false;
        if (r->used && memcmp(r->phash, phash, SUPE_TAG_LEN) == 0) { slot = r; break; }
        if (!r->used && !slot) slot = r;
    }
    if (!slot) slot = &e->pret[0];
    memcpy(slot->phash, phash, SUPE_TAG_LEN);
    memcpy(slot->node4, node4, 4);
    slot->expiryMs = now + 30000;
    slot->used = true;
}

const uint8_t* supeEngProofRetLookup(SupeEngine* e, const uint8_t* addr) {
    uint32_t now = eNow(e);
    for (int i = 0; i < SUPE_PROOFRET_MAX; i++) {
        SupeProofRet* r = &e->pret[i];
        if (!r->used) continue;
        if ((int32_t)(now - r->expiryMs) >= 0) { r->used = false; continue; }
        if (memcmp(r->phash, addr, SUPE_TAG_LEN) == 0) return r->node4;
    }
    return nullptr;
}

/* ─────────────── notes out ─────────────── */

static bool tagUsable(const uint8_t tag[SUPE_TAG_LEN]) {
    static const uint8_t z[SUPE_TAG_LEN] = { 0, 0, 0 };
    return memcmp(tag, z, SUPE_TAG_LEN) != 0;
}

static void noteSimple(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN], uint8_t ev) {
    if (!tagUsable(tag)) return;
    SupePeerNote n = {};
    n.ev = ev;
    e->host->peer_note(e->host->ctx, tag, &n);
}

static void notePair(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN],
                     const SupeCfg* cfg, int16_t rssi, int8_t txp) {
    if (!tagUsable(tag)) return;
    SupePeerNote n = {};
    n.ev = SUPE_EV_PAIR;
    n.cfg = *cfg;
    n.rssiDbm = rssi;
    n.txpDbm = txp;
    e->host->peer_note(e->host->ctx, tag, &n);
}

static void noteReport(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN],
                       const SupeCfg* cfg, int16_t theirReading, int8_t ourTxp) {
    if (!tagUsable(tag)) return;
    SupePeerNote n = {};
    n.ev = SUPE_EV_REPORT;
    n.cfg = *cfg;
    n.rssiDbm = theirReading;
    n.txpDbm = ourTxp;
    e->host->peer_note(e->host->ctx, tag, &n);
}

/* ─────────────── lifecycle ─────────────── */

void supeEngInit(SupeEngine* e, const SupeHost* host, LoraQueue* q) {
    memset(e, 0, sizeof *e);
    e->host = host;
    e->q = q;
}

void supeEngConfig(SupeEngine* e, uint8_t regime, uint8_t ownFam, uint8_t ownTop,
                   int8_t txpMax, uint8_t hailSf, uint32_t hailBwHz,
                   uint8_t crDenom, uint16_t preamble, uint8_t ifaceSync,
                   uint16_t maxFrameLen) {
    e->regime = regime;
    e->ownFam = ownFam;
    e->ownTop = ownTop > 14 ? 14 : ownTop;
    e->txpMax = txpMax;
    e->hailSf = hailSf;
    e->hailBwHz = hailBwHz;
    e->crDenom = crDenom;
    e->preamble = preamble;
    e->ifaceSync = ifaceSync;
    e->maxFrameLen = maxFrameLen;
}

static void armTimer(SupeEngine* e);
static void finishMeeting(SupeEngine* e, bool ok, const char* why);

void supeEngReset(SupeEngine* e) {
    /* The radio went away underneath; no RF restore is owed — the whole modem
     * regime is re-applied on the way up. The learned stores survive; the
     * schedules do not, because their epochs are RF events on a radio that is
     * gone. */
    if (e->m.phase != SUPE_M_IDLE)
        eLog(e, true, "supe: meeting reset (radio went down) in phase %u",
             (unsigned)e->m.phase);
    if (e->m.txBuilt) e->host->train_done(e->host->ctx, false);
    memset(&e->m, 0, sizeof e->m);
    e->m.phase = SUPE_M_IDLE;
    e->m.schedIdx = -1;
    for (int i = 0; i < SUPE_SCHED_MAX; i++) e->sched[i].used = false;
    e->offerArmed = false;
}

void supeEngAbort(SupeEngine* e, const char* why) {
    e->offerArmed = false;
    finishMeeting(e, false, why);
}

bool supeEngBusy(const SupeEngine* e) {
    return e->m.phase != SUPE_M_IDLE || e->offerArmed;
}

bool supeEngXactLive(const SupeEngine* e) {
    return e->m.phase >= SUPE_M_HD_TX;
}

uint16_t supeEngCargoPeer(const SupeEngine* e) {
    return e->m.phase >= SUPE_M_HD_TX ? e->m.peerId : (uint16_t)LORAQ_PEER_NONE;
}

void supeEngSetIdent(SupeEngine* e, const uint8_t id[SUPE_TAG_LEN]) {
    memcpy(e->ownIdent, id, SUPE_TAG_LEN);
    e->haveOwnIdent = true;
}

/* ─────────────── schedules ─────────────── */

static uint32_t schedHorizon(const SupeSched* s) {
    return s->wide ? SUPE_WIDE_HORIZON_MS : SUPE_NARROW_HORIZON_MS;
}

static void schedFree(SupeEngine* e, SupeSched* s) {
    (void)e;
    s->used = false;
}

/* A narrow schedule we seeded expiring unmet is the one silence that scores:
 * its seed was carrier-sensed onto the shared channel and its slots gave the
 * peer several hundred milliseconds of chances (§12). */
static bool queueHasFor(SupeEngine* e, const SupeSched* s);

static void schedExpire(SupeEngine* e, SupeSched* s, uint32_t now) {
    bool struck = false;
    if (s->weSeeded && !s->wide && !s->consumed && s->haveTag) {
        e->strikes++;
        struck = true;
        SupePeerNote nt = {};
        nt.ev = SUPE_EV_STRIKE;
        nt.backoffMs = SUPE_RETRY_WAIT_MIN_MS
            + (e->host->rand32(e->host->ctx) % SUPE_RETRY_WAIT_SPAN_MS);
        nt.agoMs = now - s->epochMs;
        nt.triedTxpDbm = s->seedTxp;
        e->host->peer_note(e->host->ctx, s->tag, &nt);
    }

    /* A horizon that carried nothing while its traffic waited is the one
     * failure no frame on the air records — the ring shows only the fallback
     * that follows — so it is the one schedule ending debug keeps, in a line.
     * An idle horizon is ordinary and stays verbose.
     *
     * Two shapes, told apart by whether this node ever spoke. Silent with
     * traffic waiting means the slots were spent on something else, and the
     * census says which. Spoken and unanswered means the far end is not
     * attending this schedule at all — it derived a different one, or it is
     * gone — and what tells those apart is whether the shared channel still
     * works, which is the very next thing the traffic tries. */
    bool waiting = queueHasFor(e, s);
    const char* kind = s->wide ? "wide" : "narrow";
    if (s->nSpoke == 0 && waiting) {
        eLog(e, false, "supe: sched %02x%02x%02x %s dead — %u slots, %u due "
             "(%u idle, %u busy, %u late, %u no-train, %u no-tune)%s",
             s->d.hash3[0], s->d.hash3[1], s->d.hash3[2], kind,
             (unsigned)s->d.nSlots, (unsigned)s->nOwnDue,
             (unsigned)s->nNoTraffic, (unsigned)s->nBusy, (unsigned)s->nLate,
             (unsigned)s->nNoTrain, (unsigned)s->nNoTune,
             struck ? " — strike" : "");
    } else if (s->nSpoke > 0) {
        eLog(e, false, "supe: sched %02x%02x%02x %s unanswered — spoke %u of %u"
             "%s", s->d.hash3[0], s->d.hash3[1], s->d.hash3[2], kind,
             (unsigned)s->nSpoke, (unsigned)s->nOwnDue,
             struck ? " — strike" : "");
    } else {
        eLog(e, true, "supe: sched %02x%02x%02x %s idle — spoke %u of %u%s",
             s->d.hash3[0], s->d.hash3[1], s->d.hash3[2], kind,
             (unsigned)s->nSpoke, (unsigned)s->nOwnDue,
             struck ? " — strike" : "");
    }
    schedFree(e, s);
}

static SupeSched* schedAlloc(SupeEngine* e) {
    SupeSched* victim = nullptr;
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        SupeSched* s = &e->sched[i];
        if (!s->used) return s;
        if (!victim || (int32_t)(s->epochMs - victim->epochMs) < 0) victim = s;
    }
    return victim;                     /* full: displace the oldest */
}

static bool schedLiveFor(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN],
                         uint16_t peerId) {
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        SupeSched* s = &e->sched[i];
        if (!s->used || s->consumed) continue;
        if (s->haveTag && memcmp(s->tag, tag, SUPE_TAG_LEN) == 0) return true;
        if (peerId != LORAQ_PEER_NONE && s->peerId == peerId) return true;
    }
    return false;
}

/* Derive and install a schedule from a seeding frame both ends hold. */
static SupeSched* schedInstall(SupeEngine* e, const uint8_t* seed, uint16_t seedLen,
                               bool wide, bool weSeeded, bool weTx0,
                               const uint8_t* tag, uint16_t peerId,
                               const SupeCfg* slotCfg, uint32_t epochMs) {
    uint8_t d0[32], d1[32];
    uint8_t buf[SUPE_THATSIT_BASE + SUPE_TRAIN_MAX + 1];
    if ((size_t)seedLen + 1 > sizeof buf) return nullptr;
    e->host->sha256(e->host->ctx, seed, seedLen, d0);
    memcpy(buf, seed, seedLen);
    buf[seedLen] = 0x01;
    e->host->sha256(e->host->ctx, buf, (uint16_t)(seedLen + 1), d1);

    int nChans = 0;
    supeRegimeChans(e->regime, &nChans);

    /* A schedule already held under this hash is superseded, not duplicated:
     * same seed bytes, new epoch. The salt makes this collision rare; this is
     * the belt that makes it harmless — without it the stale copy answers the
     * hash and its slots are ghosts nobody attends. */
    for (int i = 0; i < SUPE_SCHED_MAX; i++)
        if (e->sched[i].used && memcmp(e->sched[i].d.hash3, d0, SUPE_HASH_LEN) == 0)
            e->sched[i].used = false;

    /* One narrow schedule per peer. A second PRIVSYNC naming the same peer says
     * the first went unanswered — that is why it is being retried — so the
     * schedule it seeded is dead to both ends and its slots are appointments
     * nobody will keep. Held alongside the new one they are not merely idle:
     * the two sets interleave, each retune arrives late for the other, and the
     * salt that makes every retry a fresh hash is what stops the rule above
     * from catching this. Both ends apply it to the same frame, so both retire
     * the same schedule. A hail arrives whatever this node already holds, which
     * is what makes the case reachable; goodbyes are not, because a meeting
     * consumes the schedule that produced it and a node already holding one
     * does not hail. An anonymous seeker cannot be attributed to a peer, so it
     * is left to the horizon. */
    bool newHasTag = tag != nullptr && tagUsable(tag);
    if (!wide && (newHasTag || peerId != LORAQ_PEER_NONE)) {
        for (int i = 0; i < SUPE_SCHED_MAX; i++) {
            SupeSched* s = &e->sched[i];
            if (!s->used || s->wide) continue;
            bool same = (newHasTag && s->haveTag &&
                         memcmp(s->tag, tag, SUPE_TAG_LEN) == 0) ||
                        (peerId != LORAQ_PEER_NONE && s->peerId == peerId);
            if (!same) continue;
            eLog(e, true, "supe: schedule %02x%02x%02x superseded by a new hail",
                 s->d.hash3[0], s->d.hash3[1], s->d.hash3[2]);
            s->used = false;
        }
    }

    SupeSched* s = schedAlloc(e);
    memset(s, 0, sizeof *s);
    supeDeriveSchedule(d0, d1, wide, (uint8_t)nChans, &s->d);
    s->used = true;
    s->wide = wide;
    s->weSeeded = weSeeded;
    s->weTx0 = weTx0;
    s->haveTag = tag != nullptr && tagUsable(tag);
    if (s->haveTag) memcpy(s->tag, tag, SUPE_TAG_LEN);
    s->peerId = peerId;
    s->epochMs = epochMs;
    s->slotCfg = *slotCfg;
    e->schedsIn++;
    eLog(e, true, "supe: schedule %02x%02x%02x (%s, %u slots, we tx %s)",
         s->d.hash3[0], s->d.hash3[1], s->d.hash3[2],
         wide ? "wide" : "narrow", (unsigned)s->d.nSlots,
         weTx0 ? "even" : "odd");
    return s;
}

/* ─────────────── the timer ─────────────── */

static uint32_t slotWindowCloseMs(const SupeEngine* e, const SupeSched* s, uint8_t k) {
    return s->epochMs + s->d.slot[k].tMs + SUPE_SLOT_GUARD_MS
           + preambleMs(e, &s->slotCfg);
}

uint32_t supeEngNextEventMs(const SupeEngine* e, uint32_t now) {
    uint32_t best = UINT32_MAX;
    if (e->m.phase != SUPE_M_IDLE) {
        if (e->m.deadlineMs) return e->m.deadlineMs;
        return UINT32_MAX;                 /* waiting on a tx completion */
    }
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        const SupeSched* s = &e->sched[i];
        if (!s->used || s->consumed) continue;
        uint32_t expiry = s->epochMs + schedHorizon(s) + 2 * SUPE_SLOT_GUARD_MS;
        if ((int32_t)(expiry - now) <= 0) expiry = now;   /* overdue: fire now */
        if (expiry < best) best = expiry;
        for (uint8_t k = s->nextSlot; k < s->d.nSlots; k++) {
            bool mine = s->weTx0 == ((k & 1) == 0);
            /* The listener aims to be listening a slot-guard early: late by
             * its own wake slop it is still there before the speaker, which is
             * the whole point of the asymmetry (SUPE_SLOT_GUARD_MS). */
            uint32_t at = s->epochMs + s->d.slot[k].tMs
                          - (mine ? 0 : SUPE_SLOT_GUARD_MS);
            if ((int32_t)(at - now) <= 0) at = now;
            if (at < best) best = at;
            break;                          /* only the soonest slot matters */
        }
    }
    return best;
}

static void armTimer(SupeEngine* e) {
    uint32_t at = supeEngNextEventMs(e, eNow(e));
    if (at != UINT32_MAX) e->host->schedule(e->host->ctx, at);
}

/* ─────────────── going home ─────────────── */

/* Build the delivery order: buffered arrivals sorted by sequence position —
 * repairs included — with unmatched arrivals dropped and holes simply absent. */
static void deliverInbound(SupeEngine* e) {
    SupeMeet* m = &e->m;
    if (m->rxN == 0) return;
    uint8_t order[SUPE_TRAIN_MAX];
    uint8_t n = 0;
    if (!m->rxAligned) {
        /* No THATSIT ever came: the frames arrived in transmitted order, which
         * IS sequence order with the holes unknowable. Deliver as they came. */
        for (uint8_t i = 0; i < m->rxN; i++) order[n++] = i;
    } else {
        for (uint8_t pos = 0; pos < m->peerCount; pos++)
            for (uint8_t i = 0; i < m->rxN; i++)
                if (m->rxPos[i] == pos) { order[n++] = i; break; }
    }
    if (n) e->host->train_deliver(e->host->ctx, order, n);
    m->rxN = 0;
}

/* One direction, as a triple: the power that side transmitted at, and the level
 * and SNR the OTHER side read it as. `tx{…}` is ours going out, `rx{…}` is
 * theirs coming in, and a reading that never came back is `?` rather than a
 * zero that would read as a measurement. */
static void fmtLeg(char* out, size_t cap, const char* dir,
                   int txp, bool haveRead, int rssi, int snrQ) {
    if (haveRead)
        snprintf(out, cap, "%s{%d %d %d}", dir, txp, rssi, supeDecSnr10(snrQ) / 10);
    else
        snprintf(out, cap, "%s{%d ? ?}", dir, txp);
}

/* The meeting, in one line. Whoever hailed is named first and their leg is
 * printed first, so the order on the line is the order on the air:
 *
 *   Our hail tx{10 -58 12} 3/500/5: tx{10 -50 12} rx{14 -45 10} - sent 5/5, rcvd 2/2
 *   Their hail rx{14 -45 10} 3/500/5: rx{14 -45 10} tx{10 -50 12} - rcvd 2/2, sent 5/5
 *
 * `<ch>/<bw kHz>/<sf>` is the configuration the trains actually flew at, which
 * is the confirmed budget rather than the slot's. Anything that went wrong is
 * appended to the same line — a failure is a property of the meeting, not a
 * separate event, and splitting it off is what made the old log unreadable. */
static void meetingLine(SupeEngine* e, uint8_t got, bool ok, const char* why) {
    SupeMeet* m = &e->m;
    bool weOpened = !m->listener;
    char hail[32] = "", ours[32] = "", theirs[32] = "", counts[64] = "";

    /* The opening: a hail is a frame with levels to report, a rendezvous is an
     * appointment that cost nothing and has none. */
    if (m->fromHail && m->haveHail)
        fmtLeg(hail, sizeof hail, weOpened ? "tx" : "rx", m->hailTxp,
               true, m->hailRssi, m->hailSnrQ);

    /* A side that sent no train has no leg — a power of zero there would read
     * as a measurement rather than as nothing having happened. */
    if (m->txFired)
        fmtLeg(ours, sizeof ours, "tx", m->trainTxp, m->haveOurRead,
               m->ourTrainRssi, m->ourTrainSnrQ);
    if (m->havePeerTxp)
        fmtLeg(theirs, sizeof theirs, "rx", m->peerTrainTxp, m->anyRx,
               m->worstRssi, m->worstSnrQ);

    /* The counts, and only for a leg that carried something: "0/0" reads as a
     * measurement of nothing rather than as nothing having happened, which is
     * the same reason a silent side gets no leg above. Repairs are
     * transmissions beyond the train, so they are named as such rather than
     * pushing the count past its own total. */
    {
        char sent[28] = "", rcvd[28] = "";
        if (m->tx.count || m->txFired) {
            if (m->txFired > m->tx.count)
                snprintf(sent, sizeof sent, "sent %u/%u+%u", m->tx.count, m->tx.count,
                         (unsigned)(m->txFired - m->tx.count));
            else
                snprintf(sent, sizeof sent, "sent %u/%u", m->txFired, m->tx.count);
        }
        if (got || m->exCount)
            snprintf(rcvd, sizeof rcvd, "rcvd %u/%u", got, m->exCount);
        /* Whoever opened is named first, so the line reads in the order the
         * meeting happened. */
        const char* a = weOpened ? sent : rcvd;
        const char* b = weOpened ? rcvd : sent;
        if (a[0] && b[0]) snprintf(counts, sizeof counts, " - %s, %s", a, b);
        else if (a[0] || b[0]) snprintf(counts, sizeof counts, " - %s", a[0] ? a : b);
    }

    eLog(e, false, "supe: %s %s%s%s %u/%lu/%u:%s%s%s%s%s%s%s",
         weOpened ? "Our" : "Their", m->fromHail ? "hail" : "rndv",
         hail[0] ? " " : "", hail,
         m->chan, (unsigned long)(m->cfg.bwHz / 1000u), m->cfg.sf,
         (weOpened ? ours : theirs)[0] ? " " : "", weOpened ? ours : theirs,
         (weOpened ? theirs : ours)[0] ? " " : "", weOpened ? theirs : ours,
         counts, ok ? "" : " — ", ok ? "" : why);
}

static void finishMeeting(SupeEngine* e, bool ok, const char* why) {
    SupeMeet* m = &e->m;
    uint8_t was = m->phase;
    uint8_t got = m->rxN;
    if (was == SUPE_M_IDLE) { e->offerArmed = false; return; }

    if (m->retuned) e->host->tune_home(e->host->ctx);
    deliverInbound(e);
    if (m->txBuilt) e->host->train_done(e->host->ctx, m->ourTrainConfirmed);

    /* Every goodbye keys the next schedule, and the seed must be a frame both
     * ends can PROVE the other holds (§7) — holding it ourselves is not enough.
     * Two ways to have that proof, and nothing else counts:
     *
     *   we received it   — the peer sent it, so the peer holds it;
     *   they answered it — a BYE, a RESEND or an answering HAVEDATA came back,
     *                      so the THATSIT we sent arrived.
     *
     * Seeding on a THATSIT we sent and heard nothing about is the asymmetric
     * case, and it is worse than seeding nothing: WE derive a schedule and the
     * peer derives none, so we spend the whole horizon speaking into a node
     * that is not attending — six slots and three seconds — before falling back
     * to the shared channel. One lost frame becomes a dead session that way.
     * Unproven, both ends agree on nothing, and the next traffic buys a
     * PRIVSYNC immediately, which is the recovery that actually works. */
    /* …and it must be the meeting's LAST THATSIT, not merely one we hold. An
     * answering HAVEDATA promises a return train, so a later THATSIT will close
     * the meeting; ours stops being the goodbye the moment that promise is made.
     * Seeding from a superseded one derives a schedule the peer never derives —
     * the same orphan as seeding from an unproven one, reached the other way. */
    bool goodbyeShared = (m->weReceivedFinal || m->ourTrainConfirmed)
                         && !m->laterThatsit;
    if (m->lastThatsitLen && was >= SUPE_M_TRAIN_RX && goodbyeShared) {
        SupeCfg cfg = m->cfg;
        schedInstall(e, m->lastThatsit, m->lastThatsitLen, /*wide=*/true,
                     /*weSeeded=*/false, /*weTx0=*/m->weReceivedFinal,
                     m->haveTag ? m->tag : nullptr, m->peerId, &cfg, eNow(e));
    }
    if (ok) {
        e->meetingsDone++;
        if (m->haveTag) noteSimple(e, m->tag, SUPE_EV_MET);
    }

    SupeEngine::SupeEndRec* er = &e->ends[e->endsAt];
    e->endsAt = (uint8_t)((e->endsAt + 1) % (sizeof e->ends / sizeof e->ends[0]));
    er->why = why;          er->endedMs = eNow(e);
    er->phase = was;        er->chan = m->chan;
    er->listener = m->listener;
    er->ok = ok;
    er->sent = m->txNext;   er->got = got;
    er->expect = m->exCount;

    if (was >= SUPE_M_HD_TX) meetingLine(e, got, ok, why);

    memset(m, 0, sizeof *m);
    m->phase = SUPE_M_IDLE;
    m->schedIdx = -1;
    e->host->rx(e->host->ctx);
    armTimer(e);
}

/* ─────────────── the classifier and the seed ─────────────── */

int shouldDetour(const SupePeerView* peer, const LoraQueue* q,
                 const SupeChanView* chans, uint32_t now,
                 uint32_t* wait_until_ms) {
    /* The one deliberately-open decision (SUPE.md §18): inputs are the peer,
     * the queue and the channels; the answer is a policy question for
     * plans/simulation.md §7. v0: a peer to meet is a meeting worth seeding —
     * the shared-channel cost is one short frame per conversation. */
    (void)peer; (void)q; (void)chans; (void)now; (void)wait_until_ms;
    return DETOUR_NOW;
}

/* Name the door a packet left by, once per distinct refusal. The reasons are
 * genuinely different faults wearing one outcome: no tag at all is a broadcast
 * or a packet the observer could not attribute, while a tag that names no SUPE
 * peer is an attribution that failed to resolve — and a link dialled TO this
 * node is the case that produces the second while the far end sees neither. */
enum : uint8_t {
    PLAIN_NONE = 0,
    PLAIN_EXPIRED,
    PLAIN_NO_TAG,
    PLAIN_NOT_PEER,
    PLAIN_ONCE,
};

static void notePlain(SupeEngine* e, uint8_t why, const LoraPkt* p) {
    uint8_t tag[SUPE_TAG_LEN] = { 0, 0, 0 };
    uint16_t len = 0;
    if (p) {
        len = p->len;
        if (p->flags & LORAQ_F_HAVE_TAG) memcpy(tag, p->tag, SUPE_TAG_LEN);
    }
    /* Keyed on the reason and the peer, NOT the length: a mixed segment carries
     * ordinary traffic to nodes that simply do not speak the protocol, and
     * keying on size would put a line on every distinct packet forever. One
     * line per peer per reason says the same thing and stops. */
    if (why == e->plainWhy && memcmp(tag, e->plainTag, SUPE_TAG_LEN) == 0) return;
    e->plainWhy = why;
    e->plainLen = len;
    memcpy(e->plainTag, tag, SUPE_TAG_LEN);
    static const char* kWhy[] = {
        "", "dialect expired", "no tag — broadcast or unattributable",
        "tag names no SUPE peer", "one plain pass, deliberate",
    };
    eLog(e, false, "supe: %uB plain via %02x%02x%02x — %s", (unsigned)len,
         tag[0], tag[1], tag[2], kWhy[why < 5 ? why : 0]);
}

uint8_t supeEngVerdict(SupeEngine* e) {
    LoraPkt* p = loraqAt(e->q, 0);
    if (e->expired) {
        e->offerArmed = false;
        notePlain(e, PLAIN_EXPIRED, p);
        return SUPE_V_PLAIN;
    }
    if (!p) { e->offerArmed = false; return SUPE_V_PLAIN; }
    if (!(p->flags & LORAQ_F_HAVE_TAG)) {
        e->offerArmed = false;
        notePlain(e, PLAIN_NO_TAG, p);
        return SUPE_V_PLAIN;
    }

    /* A live schedule with this peer: the packet rides the next met slot
     * rather than contending on the shared channel. */
    if (schedLiveFor(e, p->tag, p->peer_id)) {
        e->offerArmed = false;
        return SUPE_V_WAIT;
    }
    if (e->m.phase >= SUPE_M_HD_TX && e->m.peerId != LORAQ_PEER_NONE &&
        e->m.peerId == p->peer_id) {
        e->offerArmed = false;
        return SUPE_V_WAIT;                /* its meeting is running right now */
    }

    SupePeerView pv;
    if (!e->host->peer_get(e->host->ctx, p->tag, &pv) || !pv.known) {
        /* Not a SUPE peer: untouched, exactly as with the feature off. */
        e->offerArmed = false;
        notePlain(e, PLAIN_NOT_PEER, p);
        return SUPE_V_PLAIN;
    }
    uint32_t now = eNow(e);
    if (pv.absentUntilMs && (int32_t)(pv.absentUntilMs - now) > 0) {
        e->offerArmed = false;
        e->dropsAbsent++;
        return SUPE_V_DROP;
    }
    if (pv.retryWaitUntilMs && (int32_t)(pv.retryWaitUntilMs - now) > 0) {
        /* Mid-ladder: the randomised wait between seeds. The packet waits
         * through it — flying plainly here would defeat the ladder. */
        e->offerArmed = false;
        return SUPE_V_WAIT;
    }
    if (e->plainOnce) {
        e->plainOnce = false;
        e->offerArmed = false;
        notePlain(e, PLAIN_ONCE, p);
        return SUPE_V_PLAIN;
    }

    SupeChanView cv;
    e->host->chan_get(e->host->ctx, &cv);
    uint32_t waitUntil = 0;
    int d = shouldDetour(&pv, e->q, &cv, now, &waitUntil);
    if (d == DETOUR_NO)  { e->offerArmed = false; return SUPE_V_PLAIN; }
    if (d == DETOUR_WAIT) { e->offerArmed = false; return SUPE_V_WAIT; }

    if (!e->offerArmed) {
        e->offerArmed = true;
        e->offerJitterUntilMs = now
            + (e->host->rand32(e->host->ctx) % SUPE_OFFER_JITTER_MS);
    }
    return SUPE_V_OFFER;
}

bool supeEngLaunchDue(const SupeEngine* e) {
    if (!e->offerArmed || e->m.phase != SUPE_M_IDLE) return false;
    uint32_t now = e->host->now_ms(e->host->ctx);
    return (int32_t)(now - e->offerJitterUntilMs) >= 0;
}

void supeEngLaunch(SupeEngine* e) {
    SupeMeet* m = &e->m;
    e->offerArmed = false;
    LoraPkt* p = loraqAt(e->q, 0);
    if (!p || !(p->flags & LORAQ_F_HAVE_TAG) || m->phase != SUPE_M_IDLE) return;
    SupePeerView pv;
    if (!e->host->peer_get(e->host->ctx, p->tag, &pv) || !pv.known) return;

    /* The absence ladder's power axis (§12): the first seed at what the
     * evidence says the peer needs, every one after at maximum. There is no
     * ceiling to walk — the budget conversation happens at the meeting, where
     * it is informed by measurement instead of by guessing at silence. */
    /* The seed's power ladder (§12, §15): the first at what the evidence says,
     * the second halfway to maximum, the third and later at maximum. */
    int8_t txp = pv.absentStrikes == 0 ? pv.txpOpen
               : pv.absentStrikes == 1 ? (int8_t)(((int)pv.txpOpen + e->txpMax + 1) / 2)
               : e->txpMax;
    if (txp > e->txpMax) txp = e->txpMax;

    SupePrivsync ps = {};
    ps.regime = e->regime;
    ps.version = SUPE_VERSION;
    memcpy(ps.tag, p->tag, SUPE_TAG_LEN);
    ps.pwrDbm = txp;
    ps.salt = (uint8_t)e->host->rand32(e->host->ctx);
    ps.haveIdent = e->haveOwnIdent;
    if (ps.haveIdent) memcpy(ps.ident, e->ownIdent, SUPE_TAG_LEN);
    uint8_t f[SUPE_PRIVSYNC_ID_LEN];
    size_t n = supeEncPrivsync(f, sizeof f, &ps);
    if (!n) return;

    memset(m, 0, sizeof *m);
    m->phase = SUPE_M_PS_TX;
    m->schedIdx = -1;
    memcpy(m->tag, p->tag, SUPE_TAG_LEN);
    m->haveTag = true;
    m->peerId = pv.peerId;
    m->psTxp = txp;
    memcpy(m->psFrame, f, n);
    m->psLen = (uint8_t)n;
    m->beganMs = eNow(e);
    if (!e->host->tx_frame(e->host->ctx, f, (uint16_t)n, txp)) {
        finishMeeting(e, false, "PRIVSYNC would not transmit");
        e->plainOnce = true;
        return;
    }
    e->seedsOut++;
    eLog(e, true, "supe: PRIVSYNC %02x%02x%02x txp=%d (strikes %u)",
         ps.tag[0], ps.tag[1], ps.tag[2], (int)txp, (unsigned)pv.absentStrikes);
}

/* ─────────────── attending slots ─────────────── */

static bool queueHasFor(SupeEngine* e, const SupeSched* s) {
    for (uint8_t i = 0; i < loraqDepth(e->q); i++) {
        LoraPkt* p = loraqAt(e->q, i);
        if (s->haveTag && (p->flags & LORAQ_F_HAVE_TAG) &&
            memcmp(p->tag, s->tag, SUPE_TAG_LEN) == 0) return true;
        if (s->peerId != LORAQ_PEER_NONE && p->peer_id == s->peerId) return true;
    }
    return false;
}

/* Our meeting frames' power at the slot configuration (§15): the controller's
 * derivation for the peer, never above the channel's cap. */
static int8_t meetTxp(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN], bool haveTag,
                      uint8_t chan, const SupeCfg* cfg) {
    int8_t cap = chanTxpCapOf(e, chan);
    if (!haveTag || !tagUsable(tag)) return cap;
    int8_t open = e->host->txp_open(e->host->ctx, tag, cfg);
    return open < cap ? open : cap;
}

/* The peer's family for the ladder. Both ends resolve the ladder over the
 * same unordered pair {ours, theirs} — membership is symmetric — so each asks
 * its own table; a peer that never announced resolves to our own family, the
 * conservative reading only where the two differ. */
static uint8_t peerFamOf(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN],
                         bool haveTag) {
    if (haveTag && tagUsable(tag)) {
        SupePeerView pv = {};
        if (e->host->peer_get(e->host->ctx, tag, &pv) && pv.known) return pv.fam;
    }
    return e->ownFam;
}

/* Entering a phase that ends with the host's tx completion: no deadline may
 * stand. nextEventMs reads deadlineMs == 0 as "no wake until tx-done"; a stale
 * deadline left behind by the phase before re-arms the host timer at zero
 * delay on every firing — and on a real device that storm outranks the radio
 * task, so the tx-done that would end it is never serviced. */
static void enterTxPhase(SupeMeet* m, uint8_t phase) {
    m->phase = phase;
    m->deadlineMs = 0;
}

/* The transmitting side's slot: sense, then open with HAVEDATA. */
static void slotSpeak(SupeEngine* e, SupeSched* s, uint8_t k) {
    SupeMeet* m = &e->m;
    const SupeSlotD* sl = &s->d.slot[k];
    uint8_t word = supeSyncWordAt(s->slotCfg.sf, e->ifaceSync, sl->sByte);
    if (!e->host->tune(e->host->ctx, sl->chan, &s->slotCfg, word)) {
        if (s->nNoTune < 255) s->nNoTune++;
        s->nextSlot = (uint8_t)(k + 1);
        return;
    }
    m->retuned = true;
    if (e->host->cca && !e->host->cca(e->host->ctx)) {
        /* The appointment grants the peer's attention, never the spectrum:
         * a busy channel skips the slot, which costs and means nothing. */
        eLog(e, true, "supe: slot %u of %02x%02x%02x skipped — ch%u busy",
             (unsigned)k, s->d.hash3[0], s->d.hash3[1], s->d.hash3[2],
             (unsigned)sl->chan);
        if (s->nBusy < 255) s->nBusy++;
        e->slotsSkipped++;
        e->host->tune_home(e->host->ctx);
        m->retuned = false;
        s->nextSlot = (uint8_t)(k + 1);
        armTimer(e);
        return;
    }

    /* The train, built whole now: the frames outlive the queue view, since a
     * repair round resends them at the close. */
    if (!e->host->train_build(e->host->ctx, s->peerId,
                              s->haveTag ? s->tag : nullptr,
                              SUPE_TRAIN_MAX, &m->tx) || m->tx.count == 0) {
        if (s->nNoTrain < 255) s->nNoTrain++;
        e->host->tune_home(e->host->ctx);
        m->retuned = false;
        s->nextSlot = (uint8_t)(k + 1);
        armTimer(e);
        return;
    }
    m->txBuilt = true;

    /* The proposed budget: as far up this channel's ladder as both ceilings
     * allow; GIMME confirms at or below it, from the reading it just took. */
    SupePeerView pv = {};
    uint8_t peerTop = 14;
    if (s->haveTag && e->host->peer_get(e->host->ctx, s->tag, &pv) && pv.known)
        peerTop = pv.topBudget;
    SupeLadderEntry lad[SUPE_LADDER_MAX_ENTRIES];
    int ln = supeLadder(e->regime, SUPE_VERSION, e->hailSf, e->hailBwHz,
                        chanMaxBwOf(e, sl->chan), e->ownFam,
                        peerFamOf(e, s->tag, s->haveTag),
                        lad, SUPE_LADDER_MAX_ENTRIES);
    int top = ln > 0 ? ln - 1 : 0;
    if (top > e->ownTop) top = e->ownTop;
    if (top > peerTop)   top = peerTop;
    SupeCfg propCfg = ln > 0
        ? SupeCfg{ lad[top].sf, lad[top].bwHz, lad[top].ldro, lad[top].marginDeci }
        : s->slotCfg;

    /* Trim the train to the regime's ceiling at the proposed budget. The
     * frames stay built; only the count travels. */
    const SupeRegime* g = supeRegime(e->regime);
    uint32_t ceil = SUPE_LEN_MAX_MS;
    if (g && g->trainCeilMs && g->trainCeilMs < ceil) ceil = g->trainCeilMs;
    while (m->tx.count > 1 && trainLenMs(e, &m->tx, m->tx.count, &propCfg) > ceil)
        m->tx.count--;

    m->listener = false;
    m->schedIdx = (int8_t)(s - e->sched);
    memcpy(m->hash3, s->d.hash3, SUPE_HASH_LEN);
    if (s->haveTag) { memcpy(m->tag, s->tag, SUPE_TAG_LEN); m->haveTag = true; }
    m->peerId  = s->peerId;
    m->chan    = sl->chan;
    m->sByte   = sl->sByte;
    m->slotCfg = s->slotCfg;
    m->cfg     = s->slotCfg;
    m->budget  = (uint8_t)top;
    m->ourTxp  = meetTxp(e, s->tag, s->haveTag, sl->chan, &s->slotCfg);
    m->beganMs = eNow(e);

    SupeHaveData hd = {};
    memcpy(hd.hash, m->hash3, SUPE_HASH_LEN);
    hd.pwrDbm  = m->ourTxp;
    hd.budget  = m->budget;
    hd.count   = m->tx.count;
    hd.lenByte = supeEncLen(trainLenMs(e, &m->tx, m->tx.count, &propCfg));
    uint8_t f[SUPE_HAVEDATA_LEN];
    size_t n = supeEncHaveData(f, sizeof f, &hd);
    if (!n || !e->host->tx_frame(e->host->ctx, f, (uint16_t)n, m->ourTxp)) {
        finishMeeting(e, false, "HAVEDATA would not transmit");
        return;
    }
    e->slotsSpoken++;
    if (s->nSpoke < 255) s->nSpoke++;
    enterTxPhase(m, SUPE_M_HD_TX);
    s->nextSlot = (uint8_t)(k + 1);
    eLog(e, true, "supe: HAVEDATA ch%u %u frames, propose budget %u, txp=%d",
         (unsigned)sl->chan, (unsigned)hd.count, (unsigned)hd.budget,
         (int)m->ourTxp);
}

/* The listening side's slot: a window, stop-on-preamble in spirit. */
static void slotListen(SupeEngine* e, SupeSched* s, uint8_t k) {
    SupeMeet* m = &e->m;
    const SupeSlotD* sl = &s->d.slot[k];
    uint8_t word = supeSyncWordAt(s->slotCfg.sf, e->ifaceSync, sl->sByte);
    if (!e->host->tune(e->host->ctx, sl->chan, &s->slotCfg, word)) {
        s->nextSlot = (uint8_t)(k + 1);
        return;
    }
    e->host->rx(e->host->ctx);
    e->slotsListened++;
    /* How the window opened relative to the moment the far end starts speaking.
     * Negative is the only good answer: a receiver opened after the preamble has
     * passed is deaf for the whole frame however strong it is, so "inside the
     * window" says nothing on its own. A run of small positive numbers here is
     * a first-slot gap too short for this hardware's cold retune, not a peer
     * that has stopped speaking. */
    long open = (long)(int32_t)(eNow(e) - (s->epochMs + s->d.slot[k].tMs));
    if (open > 0 && e->slotsLateOpen < UINT16_MAX) e->slotsLateOpen++;
    eLog(e, true, "supe: listen %02x%02x%02x slot %u t=%u (open %+ld)",
         s->d.hash3[0], s->d.hash3[1], s->d.hash3[2], (unsigned)k,
         (unsigned)s->d.slot[k].tMs, open);
    m->phase = SUPE_M_SLOT_LISTEN;
    m->retuned = true;
    m->schedIdx = (int8_t)(s - e->sched);
    memcpy(m->hash3, s->d.hash3, SUPE_HASH_LEN);
    if (s->haveTag) { memcpy(m->tag, s->tag, SUPE_TAG_LEN); m->haveTag = true; }
    else m->haveTag = false;
    m->peerId  = s->peerId;
    m->chan    = sl->chan;
    m->sByte   = sl->sByte;
    m->slotCfg = s->slotCfg;
    m->cfg     = s->slotCfg;
    m->txNext  = 0;                    /* reused as the window-extension count */
    m->deadlineMs = slotWindowCloseMs(e, s, k);
    s->nextSlot = (uint8_t)(k + 1);
    e->host->schedule(e->host->ctx, m->deadlineMs);
}

/* The IDLE tick: expire schedules, walk past missed slots, act on a due one. */
static void slotService(SupeEngine* e) {
    uint32_t now = eNow(e);
    /* A frame arriving right now outranks every slot. Attending one retunes the
     * radio, and a retune mid-preamble destroys the reception in progress — so a
     * node holding schedules would abandon real frames on the channel it is
     * camped on for slots that may well be empty. The frame in the air is the
     * certain thing; the slot is the speculative one, and a missed slot means
     * nothing (§12) while a missed PRIVSYNC means the peer cannot reach us at
     * all.
     *
     * Only the two acts that touch the radio defer. Expiring a schedule and
     * walking past a slot already gone are bookkeeping, and they must run on
     * every tick whatever the receiver is doing: the next-event time is
     * computed from the earliest unwalked slot and the horizon, both of which
     * report "now" once they are in the past. Leave them unadvanced and the
     * deadline pins at zero for as long as the receiver stays busy, and the
     * main loop spins on it — which also stops the horizon from ever retiring
     * the schedule, so a peer's retries pile up as live schedules that starve
     * each other's slots. A declined slot is therefore walked past, not
     * retried: by the time the reception ends its moment has gone. */
    bool rxBusy = e->host->rx_busy && e->host->rx_busy(e->host->ctx);
    /* Narrow schedules first, and the order is the whole point rather than a
     * detail of iteration. A narrow schedule was bought with a PRIVSYNC on the
     * shared channel moments ago: someone has traffic *now* and paid to say so,
     * and its whole horizon is a few hundred milliseconds. A wide one is a
     * standing appointment from a meeting already closed, three seconds long
     * and quite possibly empty at both ends. Taken in array order the wide one
     * wins as often as not, and each of its retunes lands the node late for the
     * hail — the one appointment with something behind it. On the bench that
     * read as a peer hailing into a node that missed every slot of the schedule
     * that hail had just seeded, then declared it absent.
     *
     * Deferring the other kind's bookkeeping for this tick is safe: acting on a
     * slot leaves the engine in a meeting phase, where the next event is that
     * meeting's own deadline and no schedule is consulted at all. Everything
     * left unwalked is walked on the return to idle. */
    for (int pass = 0; pass < 2; pass++)
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        SupeSched* s = &e->sched[i];
        if (!s->used) continue;
        if (s->wide != (pass == 1)) continue;
        if (s->consumed) { schedFree(e, s); continue; }
        if ((int32_t)(now - (s->epochMs + schedHorizon(s) + 2 * SUPE_SLOT_GUARD_MS)) >= 0) {
            schedExpire(e, s, now);
            continue;
        }
        /* Give up on a wide schedule the far end is plainly not attending. A
         * schedule that meets is freed as consumed, so a live one that has
         * already spoken has had every one of those slots ignored — and a
         * HAVEDATA is not a broadcast, it is a question the peer owes a GIMME
         * to. Two unanswered is enough to stop asking.
         *
         * The two ends cannot always agree that a goodbye was shared — the last
         * frame of any handshake is unacknowledgeable, so a lost answer leaves
         * one side holding a schedule the other never derived. That cannot be
         * designed away, only made cheap: the cost of an orphan is whatever is
         * spent before falling back, and the shared channel is always right
         * there. Three seconds of speaking into silence buys nothing a hail
         * would not have bought at once. Wide only — a narrow horizon is already
         * shorter than the hail that would replace it. */
        if (s->wide && s->nSpoke >= SUPE_SCHED_GIVEUP_SPOKE && queueHasFor(e, s)) {
            eLog(e, false, "supe: sched %02x%02x%02x wide abandoned — %u "
                 "unanswered, hailing", s->d.hash3[0], s->d.hash3[1],
                 s->d.hash3[2], (unsigned)s->nSpoke);
            e->schedsAbandoned++;
            schedFree(e, s);
            continue;
        }
        /* Walk past slots whose moment has passed — a missed slot means
         * nothing, in any direction (§12). */
        while (s->nextSlot < s->d.nSlots) {
            uint8_t k = s->nextSlot;
            bool mine = s->weTx0 == ((k & 1) == 0);
            uint32_t at = s->epochMs + s->d.slot[k].tMs;
            if (mine) {
                /* The late tolerance is twice the guard: the wake for an own
                 * slot rides a timer plus a task switch, and a start a few ms
                 * late still lands its preamble inside the listener's window,
                 * whose rx-busy extension covers the tail. A slot missed by
                 * more than that is walked past — and named, because a silent
                 * walk past queued traffic reads as a radio that never tried. */
                if ((int32_t)(now - (at + SUPE_SLOT_LATE_MS)) > 0) {
                    if (s->nOwnDue < 255) s->nOwnDue++;
                    if (s->nLate < 255) s->nLate++;
                    if (queueHasFor(e, s))
                        eLog(e, true, "supe: own slot %u of %02x%02x%02x missed by %ld ms",
                             (unsigned)k, s->d.hash3[0], s->d.hash3[1], s->d.hash3[2],
                             (long)(now - at));
                    s->nextSlot++;
                    continue;
                }
                if ((int32_t)(now - at) >= 0) {
                    if (s->nOwnDue < 255) s->nOwnDue++;
                    if (rxBusy) { e->slotsYielded++; s->nextSlot++; continue; }
                    if (queueHasFor(e, s)) { slotSpeak(e, s, k); return; }
                    if (s->nNoTraffic < 255) s->nNoTraffic++;
                    s->nextSlot++;
                    continue;
                }
            } else {
                if ((int32_t)(now - slotWindowCloseMs(e, s, k)) > 0) { s->nextSlot++; continue; }
                if ((int32_t)(now - (at - SUPE_SLOT_GUARD_MS)) >= 0) {
                    if (rxBusy) { e->slotsYielded++; s->nextSlot++; continue; }
                    slotListen(e, s, k);
                    return;
                }
            }
            break;
        }
    }
}

/* ─────────────── the trains ─────────────── */

/* The next frame index this side owes: the whole train on leg entry, or the
 * peer-named subset during a repair. 0xFF when done. */
static uint8_t txNextIdx(SupeMeet* m) {
    if (!m->txMaskAny) return m->txNext < m->tx.count ? m->txNext : 0xFF;
    for (uint8_t i = m->txNext; i < m->tx.count; i++)
        if (maskGet(m->txMask, i)) return i;
    return 0xFF;
}

static void fireNext(SupeEngine* e);

static void enterTrainWait(SupeEngine* e, uint32_t gapMs) {
    SupeMeet* m = &e->m;
    m->phase = SUPE_M_TRAIN_WAIT;
    m->deadlineMs = eNow(e) + gapMs;
    e->host->schedule(e->host->ctx, m->deadlineMs);
}

static void sendThatsit(SupeEngine* e);
static void sendClose(SupeEngine* e);
static void answerThatsit(SupeEngine* e);

/* What a TRAIN_WAIT expiry fires when it is not the next train frame. */
enum : uint8_t {
    SUPE_PEND_NONE = 0,
    SUPE_PEND_THATSIT,
    SUPE_PEND_ANSWER,
    SUPE_PEND_CLOSE,
};

/* Park a send for one train gap. The frame that provoked it was this side's
 * own tx-done or the peer's transmission, and the receiver it is aimed at is
 * only now flipping from that frame back to receive — the same flip the train
 * gap already covers between frames. A send fired at zero gap races the flip
 * and loses, which on the bench read as THATSIT and the closing answer being
 * the frames that went missing while whole trains arrived intact. */
static void deferSend(SupeEngine* e, uint8_t what) {
    e->m.pendSend = what;
    enterTrainWait(e, SUPE_TRAIN_GAP_MS);
}

static void fireNext(SupeEngine* e) {
    SupeMeet* m = &e->m;
    uint8_t idx = txNextIdx(m);
    if (idx == 0xFF) {
        if (m->phase == SUPE_M_REPAIR_TX || m->txMaskAny) {
            /* The repair is out. What follows depends on which side we are:
             * the opener's repairs ride ahead of its close; a leg sender's
             * repairs answer a RESEND and wait for the peer's BYE. */
            m->txMaskAny = false;
            if (m->pendClose) { deferSend(e, SUPE_PEND_CLOSE); return; }
            m->phase = SUPE_M_AWAIT_ANSWER;
            m->deadlineMs = eNow(e) + answerDeadlineMs(e, &m->cfg);
            e->host->rx(e->host->ctx);
            e->host->schedule(e->host->ctx, m->deadlineMs);
            return;
        }
        deferSend(e, SUPE_PEND_THATSIT);
        return;
    }
    if (!e->host->train_fire(e->host->ctx, idx, m->trainTxp)) {
        finishMeeting(e, false, "train frame would not transmit");
        return;
    }
    m->txNext = (uint8_t)(idx + 1);
    if (m->txFired < 255) m->txFired++;
    enterTxPhase(m, m->txMaskAny ? SUPE_M_REPAIR_TX : SUPE_M_TRAIN_TX);
    if (m->txMaskAny) e->repairsOut++;
    else              e->framesOut++;
}

static void sendThatsit(SupeEngine* e) {
    SupeMeet* m = &e->m;
    SupeThatsit t = {};
    t.pwrDbm = m->trainTxp;
    /* This goodbye's freshness — the seed of the wide schedule both ends are
     * about to derive (§7). Without it two meetings that close the same way
     * seed the same schedule at two epochs with two roles. */
    t.salt   = (uint8_t)e->host->rand32(e->host->ctx);
    t.count  = m->tx.count;
    memcpy(t.csum, m->tx.csum, m->tx.count);
    size_t n = supeEncThatsit(m->txThatsit, sizeof m->txThatsit, &t);
    if (!n || !e->host->tx_frame(e->host->ctx, m->txThatsit, (uint16_t)n,
                                 m->trainTxp)) {
        finishMeeting(e, false, "THATSIT would not transmit");
        return;
    }
    m->txThatsitLen = (uint8_t)n;
    /* Our THATSIT is the meeting's newest goodbye candidate. */
    memcpy(m->lastThatsit, m->txThatsit, n);
    m->lastThatsitLen = (uint8_t)n;
    m->weReceivedFinal = false;
    enterTxPhase(m, SUPE_M_THATSIT_TX);
    eLog(e, true, "supe: THATSIT %u frames txp=%d", (unsigned)t.count,
         (int)m->trainTxp);
}

/* Start this side's train: resolve its power where it flies, lead in, fire. */
static void startTrain(SupeEngine* e) {
    SupeMeet* m = &e->m;
    m->trainTxp = meetTxp(e, m->tag, m->haveTag, m->chan, &m->cfg);
    m->txNext = 0;
    m->txMaskAny = false;
    enterTrainWait(e, (uint32_t)SUPE_RETUNE_GAP_MS + SUPE_TRAIN_LEAD_MS);
}

/* Send BYE (pendClose 1) or RESEND (pendClose 2). */
static void sendClose(SupeEngine* e) {
    SupeMeet* m = &e->m;
    uint8_t f[SUPE_RESEND_BASE + SUPE_MASK_MAX];
    size_t n = 0;
    if (m->pendClose == 2) {
        SupeResendF rs = {};
        rs.maskLen = supeMaskLen(m->peerCount);
        memcpy(rs.mask, m->missMask, rs.maskLen);
        n = supeEncResend(f, sizeof f, &rs);
        m->closeIsFinal = false;
    } else {
        f[0] = SUPE_T_BYE;
        n = SUPE_BYE_LEN;
        m->closeIsFinal = true;
    }
    m->pendClose = 0;
    if (!n || !e->host->tx_frame(e->host->ctx, f, (uint16_t)n, m->ourTxp)) {
        finishMeeting(e, false, "close would not transmit");
        return;
    }
    enterTxPhase(m, SUPE_M_CLOSE_TX);
}

/* Align the buffered arrivals against a THATSIT's checksum list: a greedy
 * leftmost ordered-subsequence match. Conservative on collisions — when in
 * doubt, ask for more (§8). */
static void alignTrain(SupeEngine* e, const SupeThatsit* t) {
    SupeMeet* m = &e->m;
    m->peerCount = t->count;
    uint8_t assigned[SUPE_TRAIN_MAX] = { 0 };
    uint8_t p = 0;
    for (uint8_t i = 0; i < m->rxN; i++) {
        m->rxPos[i] = 0xFF;
        while (p < t->count && t->csum[p] != m->rxCsum[i]) p++;
        if (p >= t->count) break;
        m->rxPos[i] = p;
        assigned[p] = 1;
        p++;
    }
    memset(m->missMask, 0, sizeof m->missMask);
    m->missCount = 0;
    m->repairExpect = 0;
    for (uint8_t pos = 0; pos < t->count; pos++) {
        if (assigned[pos]) continue;
        maskSet(m->missMask, pos);
        m->repairPos[m->missCount] = pos;
        m->missCount++;
    }
    m->rxAligned = true;
    m->repairGot = 0;
}

/* ─────────────── receive dispatch ─────────────── */

static void onPrivsync(SupeEngine* e, const uint8_t* f, uint16_t len,
                       int16_t rssi, int16_t snr10) {
    SupePrivsync ps;
    if (!supeDecPrivsync(f, len, &ps)) { e->rxDiscard++; return; }
    /* The seed states its power, so any listener's reading is a path loss
     * immediately — and hearing the node at all is evidence of life. */
    if (ps.haveIdent) {
        SupeCfg hail = hailCfgOf(e);
        noteSimple(e, ps.ident, SUPE_EV_ALIVE);
        notePair(e, ps.ident, &hail, rssi, ps.pwrDbm);
    }
    if (!supeEngTagIsOurs(e, ps.tag)) { e->rxForeign++; return; }
    if (e->expired) { e->rxDiscard++; return; }
    if (e->m.phase != SUPE_M_IDLE && e->m.phase != SUPE_M_SLOT_LISTEN) return;

    /* Seed the narrow schedule as its listener: the seeker transmits first.
     * The peer is named by the seed's identity or not at all — an anonymous
     * seeker still gets its meeting, just no return leg (§4). */
    uint16_t peerId = LORAQ_PEER_NONE;
    if (ps.haveIdent) {
        SupePeerView pv;
        if (e->host->peer_get(e->host->ctx, ps.ident, &pv) && pv.known)
            peerId = pv.peerId;
    }
    SupeCfg hail = hailCfgOf(e);
    SupeSched* s = schedInstall(e, f, len, /*wide=*/false, /*weSeeded=*/false,
                                /*weTx0=*/false,
                                ps.haveIdent ? ps.ident : nullptr, peerId,
                                &hail, eNow(e));
    if (s) {
        s->havePs = true;
        s->psRssi = rssi;
        s->psSnrQ = supeEncSnrQ(snr10);
        s->seedTxp = ps.pwrDbm;   /* theirs, stated — ours when we are the seeder */
    }
    armTimer(e);
}

static void onHaveData(SupeEngine* e, const uint8_t* f, uint16_t len,
                       int16_t rssi, int16_t snr10) {
    SupeMeet* m = &e->m;

    if (m->phase == SUPE_M_SLOT_LISTEN) {
        SupeHaveData hd;
        if (!supeDecHaveData(f, len, 0, &hd) || hd.answering || hd.count == 0) {
            e->rxDiscard++;
            return;
        }
        if (memcmp(hd.hash, m->hash3, SUPE_HASH_LEN) != 0) {
            /* Somebody else's meeting, or a stale schedule — either way the
             * sync word's residue dies here, in one frame (§8). */
            e->rxForeign++;
            return;
        }
        SupeSched* s = m->schedIdx >= 0 ? &e->sched[m->schedIdx] : nullptr;
        bool    narrowPs = s && s->havePs && !s->wide;
        int16_t psRssi  = narrowPs ? s->psRssi : 0;
        int8_t  psSnrQ  = narrowPs ? s->psSnrQ : 0;
        notePair(e, m->tag, &m->slotCfg, rssi, hd.pwrDbm);

        /* The budget: the receiver's choice, never above the proposed ceiling,
         * from the freshest reading there is — this very frame — normalised to
         * headroom over the hailing floor via the slot entry's own margin. */
        SupeLadderEntry lad[SUPE_LADDER_MAX_ENTRIES];
        int ln = supeLadder(e->regime, SUPE_VERSION, e->hailSf, e->hailBwHz,
                            chanMaxBwOf(e, m->chan), e->ownFam,
                            peerFamOf(e, m->tag, m->haveTag),
                            lad, SUPE_LADDER_MAX_ENTRIES);
        if (ln <= 0) { e->rxDiscard++; return; }
        int top = ln - 1;
        if (top > hd.budget)  top = hd.budget;
        if (top > e->ownTop)  top = e->ownTop;
        int headroomDeci = (int)snr10 - (int)supeReqSnrDeci(m->slotCfg.sf)
                           + (int)m->slotCfg.marginDeci;
        if (narrowPs) {
            int hHail = (int)supeDecSnr10(psSnrQ) - (int)supeReqSnrDeci(e->hailSf);
            if (hHail < headroomDeci) headroomDeci = hHail;
        }
        int affordDeci = headroomDeci - SUPE_TARGET_MARGIN_DB * 10;
        uint8_t budget = 0;
        for (int i = 1; i <= top; i++)
            if ((int)lad[i].marginDeci <= affordDeci) budget = (uint8_t)i;
        SupeCfg cfg = { lad[budget].sf, lad[budget].bwHz, lad[budget].ldro,
                        lad[budget].marginDeci };

        /* Answering GIMME commits this receiver's memory: the train it invites
         * is held whole to the close (§8). SUPE_TRAIN_MAX is that commitment. */
        m->listener = true;
        m->exCount = hd.count;
        m->peerTrainTxp = hd.pwrDbm;   /* until their THATSIT states the train's */
        m->havePeerTxp = true;
        m->budget  = budget;
        m->cfg     = cfg;
        m->ourTxp  = meetTxp(e, m->tag, m->haveTag, m->chan, &m->slotCfg);
        m->worstRssi = 127;
        m->beganMs = eNow(e);
        if (s) {
            /* Their hail, for the meeting's line: what they said it flew at
             * and what we read it as. Taken now, because consuming the
             * schedule is what frees it. */
            m->fromHail = !s->wide;
            if (s->havePs) {
                m->hailTxp  = s->seedTxp;
                m->hailRssi = s->psRssi;
                m->hailSnrQ = s->psSnrQ;
                m->haveHail = true;
            }
            s->consumed = true;
            schedFree(e, s);
        }
        m->schedIdx = -1;

        SupeGimme g = {};
        memcpy(g.hash, m->hash3, SUPE_HASH_LEN);
        g.pwrDbm = m->ourTxp;
        g.budget = budget;
        g.havePsHeard = narrowPs;
        if (g.havePsHeard) { g.psRssi = psRssi; g.psSnrQ = psSnrQ; }
        g.hdRssi = rssi;
        g.hdSnrQ = supeEncSnrQ(snr10);
        uint8_t out[SUPE_GIMME_LEN];
        size_t n = supeEncGimme(out, sizeof out, &g);
        if (!n || !e->host->tx_frame(e->host->ctx, out, (uint16_t)n, m->ourTxp)) {
            finishMeeting(e, false, "GIMME would not transmit");
            return;
        }
        enterTxPhase(m, SUPE_M_GIMME_TX);
        eLog(e, true, "supe: GIMME budget %u txp=%d for %u frames",
             (unsigned)budget, (int)m->ourTxp, (unsigned)hd.count);
        return;
    }

    if (m->phase == SUPE_M_AWAIT_ANSWER && m->leg == 0 && !m->listener) {
        /* The return leg: the answering HAVEDATA carries the train's reading
         * and any repair request; the return train follows (§8).
         *
         * It also PROMISES a later THATSIT — the one closing that return train
         * — which supersedes ours as the meeting's goodbye. Until it arrives,
         * the newest THATSIT we hold is not the final one, and seeding from it
         * would derive a schedule the peer never derives (§7). */
        m->laterThatsit = true;
        SupeHaveData hd;
        if (!supeDecHaveData(f, len, m->tx.count, &hd) || !hd.answering ||
            hd.count == 0) {
            e->rxDiscard++;
            return;
        }
        if (memcmp(hd.hash, m->hash3, SUPE_HASH_LEN) != 0) { e->rxForeign++; return; }
        m->ourTrainConfirmed = true;
        notePair(e, m->tag, &m->cfg, rssi, hd.pwrDbm);
        noteReport(e, m->tag, &m->cfg, hd.trainRssi, m->trainTxp);
        m->ourTrainRssi = hd.trainRssi;
        m->ourTrainSnrQ = hd.trainSnrQ;
        m->haveOurRead = true;
        {
            SupePeerNote nt = {};
            nt.ev = SUPE_EV_TRAIN_OK;
            nt.cfg = m->cfg;
            nt.rssiDbm = hd.trainRssi;
            nt.txpDbm = m->trainTxp;
            nt.haveLevel = true;
            if (m->haveTag) e->host->peer_note(e->host->ctx, m->tag, &nt);
        }
        memcpy(m->txMask, hd.mask, sizeof m->txMask);
        m->txMaskAny = popcount8(hd.mask, hd.maskLen) > 0;
        m->leg = 1;
        m->exCount = hd.count;
        m->rxAligned = false;
        m->worstRssi = 127;
        m->phase = SUPE_M_TRAIN_RX;
        m->deadlineMs = eNow(e) + SUPE_TURNAROUND_MS
                        + trainWorstMs(e, hd.count, &m->cfg) + SUPE_GUARD_MS;
        e->host->schedule(e->host->ctx, m->deadlineMs);
        return;
    }

    e->rxForeign++;
}

static void onGimme(SupeEngine* e, const uint8_t* f, uint16_t len,
                    int16_t rssi, int16_t snr10) {
    (void)snr10;
    SupeMeet* m = &e->m;
    SupeGimme g;
    if (!supeDecGimme(f, len, &g)) { e->rxDiscard++; return; }
    if (m->phase != SUPE_M_AWAIT_GIMME ||
        memcmp(g.hash, m->hash3, SUPE_HASH_LEN) != 0) {
        e->rxForeign++;
        return;
    }

    /* Attention confirmed; the schedule is consumed — contact is what it was
     * for (§7). */
    if (m->schedIdx >= 0) {
        SupeSched* s = &e->sched[m->schedIdx];
        int8_t seedTxp = s->seedTxp;
        bool wasNarrowSeed = s->weSeeded && !s->wide;
        m->fromHail = !s->wide;
        s->consumed = true;
        schedFree(e, s);
        m->schedIdx = -1;
        if (wasNarrowSeed && m->haveTag && g.havePsHeard) {
            /* The GIMME's PRIVSYNC reading: the direction we transmit in, at
             * the hailing configuration — what no transmitter can measure for
             * itself. */
            SupeCfg hail = hailCfgOf(e);
            noteReport(e, m->tag, &hail, g.psRssi, seedTxp);
        }
        if (wasNarrowSeed) {
            m->hailTxp = seedTxp;
            m->haveHail = g.havePsHeard;
            m->hailRssi = g.psRssi;
            m->hailSnrQ = g.psSnrQ;
        }
    }
    if (m->haveTag) {
        noteSimple(e, m->tag, SUPE_EV_ALIVE);
        notePair(e, m->tag, &m->slotCfg, rssi, g.pwrDbm);
        noteReport(e, m->tag, &m->slotCfg, g.hdRssi, m->ourTxp);
    }
    /* Stands in for the train's own reading until an answering HAVEDATA
     * carries one: same peer, same tuning, one frame earlier. */
    if (!m->haveOurRead) {
        m->ourTrainRssi = g.hdRssi;
        m->ourTrainSnrQ = g.hdSnrQ;
        m->haveOurRead = true;
    }

    /* The confirmed budget: at or below what we proposed, resolved against the
     * slot's channel by both ends identically. */
    if (g.budget > m->budget) { finishMeeting(e, false, "budget above proposal"); return; }
    SupeCfg cfg;
    if (!supeResolveBudget(e->regime, SUPE_VERSION, e->hailSf, e->hailBwHz,
                           chanMaxBwOf(e, m->chan), e->ownFam,
                           peerFamOf(e, m->tag, m->haveTag),
                           g.budget, &cfg)) {
        finishMeeting(e, false, "budget would not resolve");
        return;
    }
    m->budget = g.budget;
    m->cfg = cfg;
    if (cfg.sf != m->slotCfg.sf || cfg.bwHz != m->slotCfg.bwHz) {
        uint8_t word = supeSyncWordAt(cfg.sf, e->ifaceSync, m->sByte);
        if (!e->host->tune(e->host->ctx, m->chan, &cfg, word)) {
            finishMeeting(e, false, "retune to budget failed");
            return;
        }
        m->atTrainCfg = true;
    }
    /* The train's power is resolved where the train flies, on the report just
     * received (§15): the REPORT above reached the controller before this ask. */
    startTrain(e);
}

static void onThatsit(SupeEngine* e, const uint8_t* f, uint16_t len,
                      int16_t rssi, int16_t snr10) {
    (void)snr10;
    SupeMeet* m = &e->m;
    SupeThatsit t;
    if (!supeDecThatsit(f, len, &t)) { e->rxDiscard++; return; }
    if (m->phase != SUPE_M_AWAIT_THATSIT && m->phase != SUPE_M_TRAIN_RX) {
        e->rxForeign++;
        return;
    }
    if (t.count != m->exCount)
        eLog(e, true, "supe: THATSIT count %u against declared %u",
             (unsigned)t.count, (unsigned)m->exCount);

    /* The train's power arrives one frame late, which is what keeps its levels
     * pairable while letting it be chosen on the report (§0.1). */
    if (m->haveTag && m->anyRx)
        notePair(e, m->tag, &m->cfg, m->worstRssi, t.pwrDbm);
    m->peerTrainTxp = t.pwrDbm;
    m->havePeerTxp = true;

    /* Their THATSIT is now the meeting's newest goodbye. */
    if (len <= sizeof m->lastThatsit) {
        memcpy(m->lastThatsit, f, len);
        m->lastThatsitLen = (uint8_t)len;
        m->weReceivedFinal = true;
        m->laterThatsit = false;       /* the one that was owed has arrived */
    }
    alignTrain(e, &t);

    /* The answer waits one train gap (deferSend): the THATSIT's sender is
     * flipping from its own tx-done back to receive, and an answer fired at
     * zero gap races that flip and loses. */
    deferSend(e, SUPE_PEND_ANSWER);
    (void)rssi;
}

/* The answer a THATSIT calls for, one train gap after it arrived: the
 * listener's turn — the return leg riding the answering HAVEDATA, a repair
 * request, or the close — or the opener closing the return leg, its repairs
 * riding ahead of its BYE or RESEND (§8). */
static void answerThatsit(SupeEngine* e) {
    SupeMeet* m = &e->m;
    if (m->listener && m->leg == 0) {
        /* Our turn. Return traffic rides now — GIMME is skipped, the peer's
         * attention is not in question — and the answering HAVEDATA carries
         * both the reading and the repair request (§8). */
        bool haveReturn = false;
        if (m->peerId != LORAQ_PEER_NONE || m->haveTag) {
            SupeTrainInfo ti;
            if (e->host->train_build(e->host->ctx, m->peerId,
                                     m->haveTag ? m->tag : nullptr,
                                     SUPE_TRAIN_MAX, &ti) && ti.count > 0) {
                m->tx = ti;
                m->txBuilt = true;
                haveReturn = true;
            }
        }
        if (haveReturn) {
            const SupeRegime* g = supeRegime(e->regime);
            uint32_t ceil = SUPE_LEN_MAX_MS;
            if (g && g->trainCeilMs && g->trainCeilMs < ceil) ceil = g->trainCeilMs;
            while (m->tx.count > 1 && trainLenMs(e, &m->tx, m->tx.count, &m->cfg) > ceil)
                m->tx.count--;
            m->leg = 1;
            SupeHaveData hd = {};
            memcpy(hd.hash, m->hash3, SUPE_HASH_LEN);
            hd.pwrDbm  = m->ourTxp;
            hd.budget  = m->budget;
            hd.count   = m->tx.count;
            hd.lenByte = supeEncLen(trainLenMs(e, &m->tx, m->tx.count, &m->cfg));
            hd.answering = true;
            hd.trainRssi = m->worstRssi;
            hd.trainSnrQ = m->worstSnrQ;
            hd.maskLen = supeMaskLen(m->peerCount);
            memcpy(hd.mask, m->missMask, hd.maskLen);
            uint8_t out[SUPE_HAVEDATA_ANS_BASE + SUPE_MASK_MAX];
            size_t n = supeEncHaveData(out, sizeof out, &hd);
            if (!n || !e->host->tx_frame(e->host->ctx, out, (uint16_t)n, m->ourTxp)) {
                finishMeeting(e, false, "answering HAVEDATA would not transmit");
                return;
            }
            m->repairExpect = m->missCount;     /* their repairs ride their close */
            enterTxPhase(m, SUPE_M_HD_TX);
            return;
        }
        /* No return traffic: repair or close outright. */
        m->pendClose = m->missCount ? 2 : 1;
        m->repairExpect = m->missCount;
        sendClose(e);
        return;
    }

    /* The opener, closing the return leg. */
    m->pendClose = m->missCount ? 2 : 1;
    m->repairExpect = m->missCount;
    if (m->txMaskAny) {
        m->txNext = 0;
        m->phase = SUPE_M_REPAIR_TX;
        enterTrainWait(e, SUPE_TRAIN_GAP_MS);
        return;
    }
    sendClose(e);
}

static void onBye(SupeEngine* e) {
    SupeMeet* m = &e->m;
    if (m->phase != SUPE_M_AWAIT_ANSWER && m->phase != SUPE_M_REPAIR_RX) {
        e->rxForeign++;
        return;
    }
    m->ourTrainConfirmed = true;
    if (m->haveTag && !m->listener && m->leg == 0) {
        SupePeerNote nt = {};
        nt.ev = SUPE_EV_TRAIN_OK;
        nt.cfg = m->cfg;
        nt.txpDbm = m->trainTxp;
        nt.haveLevel = false;
        e->host->peer_note(e->host->ctx, m->tag, &nt);
    }
    finishMeeting(e, true, "closed");
}

static void onResend(SupeEngine* e, const uint8_t* f, uint16_t len) {
    SupeMeet* m = &e->m;
    if (m->phase != SUPE_M_AWAIT_ANSWER || !m->txBuilt) { e->rxForeign++; return; }
    SupeResendF rs;
    if (!supeDecResend(f, len, m->tx.count, &rs)) { e->rxDiscard++; return; }
    /* One repair round: they hold our THATSIT — the train is confirmed as
     * heard, minus exactly the frames named. */
    m->ourTrainConfirmed = true;
    if (m->haveTag && !m->listener && m->leg == 0) {
        SupePeerNote nt = {};
        nt.ev = SUPE_EV_TRAIN_OK;
        nt.cfg = m->cfg;
        nt.txpDbm = m->trainTxp;
        nt.haveLevel = false;
        e->host->peer_note(e->host->ctx, m->tag, &nt);
    }
    memcpy(m->txMask, rs.mask, sizeof m->txMask);
    m->txMaskAny = true;
    m->pendClose = 0;                  /* after the repair: await their BYE */
    m->txNext = 0;
    m->phase = SUPE_M_REPAIR_TX;
    enterTrainWait(e, SUPE_TRAIN_GAP_MS);
}

void supeEngOnRx(SupeEngine* e, const uint8_t* f, uint16_t len,
                 int16_t rssi, int16_t snr10) {
    if (len < 1) return;
    e->rxFrames++;
    if (e->expired) { e->rxDiscard++; return; }
    switch (f[0]) {
        case SUPE_T_PRIVSYNC: onPrivsync(e, f, len, rssi, snr10); break;
        case SUPE_T_HAVEDATA: onHaveData(e, f, len, rssi, snr10); break;
        case SUPE_T_GIMME:    onGimme(e, f, len, rssi, snr10);    break;
        case SUPE_T_THATSIT:  onThatsit(e, f, len, rssi, snr10);  break;
        case SUPE_T_BYE:
            if (len == SUPE_BYE_LEN) onBye(e);
            else e->rxDiscard++;
            break;
        case SUPE_T_RESEND:   onResend(e, f, len);                break;
        default:              e->rxDiscard++;                     break;
    }
}

bool supeEngOnTrainFrame(SupeEngine* e, uint8_t csum, int16_t rssi, int16_t snr10) {
    SupeMeet* m = &e->m;
    if (m->phase == SUPE_M_TRAIN_RX) {
        if (m->rxN >= SUPE_TRAIN_MAX) return false;
        m->rxCsum[m->rxN] = csum;
        m->rxPos[m->rxN] = m->rxN;     /* provisional until the THATSIT aligns */
        m->rxN++;
        m->anyRx = true;
        e->framesIn++;
        if (rssi < m->worstRssi) { m->worstRssi = rssi; m->worstSnrQ = supeEncSnrQ(snr10); }
        if (m->rxN >= m->exCount) {
            m->phase = SUPE_M_AWAIT_THATSIT;
            m->deadlineMs = eNow(e) + SUPE_TURNAROUND_MS
                + toaFrameMs(e, &m->cfg, SUPE_THATSIT_BASE + m->exCount, false)
                + SUPE_GUARD_MS;
            e->host->schedule(e->host->ctx, m->deadlineMs);
        }
        return true;
    }
    if (m->phase == SUPE_M_REPAIR_RX) {
        if (m->rxN >= SUPE_TRAIN_MAX || m->repairGot >= m->repairExpect) return false;
        m->rxCsum[m->rxN] = csum;
        m->rxPos[m->rxN] = m->repairPos[m->repairGot];
        m->rxN++;
        m->repairGot++;
        e->repairsIn++;
        if (m->repairGot >= m->repairExpect) {
            if (m->listener && m->leg == 1) {
                /* Their repairs precede their close; keep waiting for it. */
                m->phase = SUPE_M_AWAIT_ANSWER;
                m->deadlineMs = eNow(e) + answerDeadlineMs(e, &m->cfg);
            } else {
                /* Our RESEND asked; everything came: answer BYE (§8). */
                m->pendClose = 1;
                sendClose(e);
                return true;
            }
            e->host->schedule(e->host->ctx, m->deadlineMs);
        }
        return true;
    }
    return false;
}

void supeEngOnTxDone(SupeEngine* e, bool ok) {
    SupeMeet* m = &e->m;
    if (m->phase == SUPE_M_IDLE) return;
    if (!ok) {
        if (m->phase == SUPE_M_PS_TX) { finishMeeting(e, false, "seed aborted"); return; }
        finishMeeting(e, false, "transmit aborted");
        return;
    }
    switch (m->phase) {
        case SUPE_M_PS_TX: {
            /* The seed is out; its end is the epoch both radios just timed.
             * The seeker transmits first — slot 0 is one turnaround away. */
            SupeCfg hail = hailCfgOf(e);
            SupeSched* s = schedInstall(e, m->psFrame, m->psLen, /*wide=*/false,
                                        /*weSeeded=*/true, /*weTx0=*/true,
                                        m->haveTag ? m->tag : nullptr, m->peerId,
                                        &hail, eNow(e));
            if (s) s->seedTxp = m->psTxp;
            memset(m, 0, sizeof *m);
            m->phase = SUPE_M_IDLE;
            m->schedIdx = -1;
            e->host->rx(e->host->ctx);
            armTimer(e);
            return;
        }
        case SUPE_M_HD_TX:
            if (m->listener && m->leg == 1) {
                /* The answering HAVEDATA is out; the return train follows. */
                startTrain(e);
                return;
            }
            m->phase = SUPE_M_AWAIT_GIMME;
            m->deadlineMs = eNow(e) + SUPE_TURNAROUND_MS
                + toaFrameMs(e, &m->slotCfg, SUPE_GIMME_LEN, false) + SUPE_GUARD_MS;
            e->host->rx(e->host->ctx);
            e->host->schedule(e->host->ctx, m->deadlineMs);
            return;
        case SUPE_M_GIMME_TX: {
            if (m->cfg.sf != m->slotCfg.sf || m->cfg.bwHz != m->slotCfg.bwHz) {
                uint8_t word = supeSyncWordAt(m->cfg.sf, e->ifaceSync, m->sByte);
                if (!e->host->tune(e->host->ctx, m->chan, &m->cfg, word)) {
                    finishMeeting(e, false, "retune to budget failed");
                    return;
                }
                m->atTrainCfg = true;
            }
            m->phase = SUPE_M_TRAIN_RX;
            m->deadlineMs = eNow(e) + SUPE_RETUNE_GAP_MS + SUPE_TURNAROUND_MS
                            + trainWorstMs(e, m->exCount, &m->cfg) + SUPE_GUARD_MS;
            e->host->rx(e->host->ctx);
            e->host->schedule(e->host->ctx, m->deadlineMs);
            return;
        }
        case SUPE_M_TRAIN_TX:
        case SUPE_M_REPAIR_TX:
            /* The next frame of the train goes out from HERE, not from a
             * deadline. A train is meant to be back to back, and PARKING one
             * gap does not cost one gap: the wait is posted to the platform's
             * scheduler, so the frame waits for a timer to expire, for the
             * timer's task to run, for the radio task to wake, and for a whole
             * pass of its loop — several times the gap itself, all of it dead
             * air inside an appointment the peer is holding open.
             *
             * The flip interval §14.7 asks for is still there; it is simply
             * paid rather than parked. Our tx-done and the peer's rx-done land
             * at the same instant, and both sides then do the same order of
             * work — service the interrupt, move a frame over SPI, re-arm —
             * before this call reaches the radio. It is the same path and the
             * same spacing the two halves of a split RNS packet already fly at,
             * where the receiver does identically the same work between them.
             *
             * Only this send skips the park. fireNext carries the end-of-train
             * case too, and the sends that follow the PEER's transmission —
             * where the flip is genuinely in front of us — still defer. */
            fireNext(e);
            return;
        case SUPE_M_THATSIT_TX:
            m->phase = SUPE_M_AWAIT_ANSWER;
            m->deadlineMs = eNow(e) + answerDeadlineMs(e, &m->cfg);
            e->host->rx(e->host->ctx);
            e->host->schedule(e->host->ctx, m->deadlineMs);
            return;
        case SUPE_M_CLOSE_TX:
            if (m->closeIsFinal) { finishMeeting(e, true, "closed"); return; }
            /* Our RESEND is out; the peer's repairs are owed. */
            m->phase = SUPE_M_REPAIR_RX;
            m->deadlineMs = eNow(e) + SUPE_TURNAROUND_MS
                            + trainWorstMs(e, m->repairExpect, &m->cfg)
                            + SUPE_GUARD_MS;
            e->host->rx(e->host->ctx);
            e->host->schedule(e->host->ctx, m->deadlineMs);
            return;
        default:
            return;
    }
}

void supeEngOnTimer(SupeEngine* e) {
    SupeMeet* m = &e->m;
    uint32_t now = eNow(e);
    switch (m->phase) {
        case SUPE_M_IDLE:
            slotService(e);
            armTimer(e);
            return;
        case SUPE_M_SLOT_LISTEN: {
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            /* A frame mid-air at the window's close is the meeting arriving:
             * extend on evidence, boundedly. */
            if (e->host->rx_busy && e->host->rx_busy(e->host->ctx) &&
                m->txNext < SUPE_WINDOW_EXTENDS_MAX) {
                m->txNext++;
                m->deadlineMs = now
                    + toaFrameMs(e, &m->slotCfg, SUPE_HAVEDATA_LEN, false)
                    + SUPE_GUARD_MS;
                e->host->schedule(e->host->ctx, m->deadlineMs);
                return;
            }
            /* Silence → home, and NOTHING is scored (§12). */
            if (m->retuned) e->host->tune_home(e->host->ctx);
            memset(m, 0, sizeof *m);
            m->phase = SUPE_M_IDLE;
            m->schedIdx = -1;
            e->host->rx(e->host->ctx);
            slotService(e);
            armTimer(e);
            return;
        }
        case SUPE_M_TRAIN_WAIT:
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            switch (m->pendSend) {
                case SUPE_PEND_THATSIT: m->pendSend = 0; sendThatsit(e);   return;
                case SUPE_PEND_ANSWER:  m->pendSend = 0; answerThatsit(e); return;
                case SUPE_PEND_CLOSE:   m->pendSend = 0; sendClose(e);     return;
                default:                fireNext(e);                       return;
            }
        case SUPE_M_AWAIT_GIMME:
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            /* An unanswered slot means nothing — the peer may be mid-frame on
             * the shared channel or busy elsewhere. The traffic keeps its
             * queue place and the next slot is its next chance. */
            finishMeeting(e, false, "no GIMME");
            return;
        case SUPE_M_TRAIN_RX:
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            m->phase = SUPE_M_AWAIT_THATSIT;
            m->deadlineMs = now + SUPE_TURNAROUND_MS
                + toaFrameMs(e, &m->cfg, SUPE_THATSIT_BASE + m->exCount, false)
                + SUPE_GUARD_MS;
            e->host->schedule(e->host->ctx, m->deadlineMs);
            return;
        case SUPE_M_AWAIT_THATSIT:
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            /* The sequence never arrived; what did arrive is still in order.
             * Deliver the provable prefix and surrender the rest (§8, §12). */
            finishMeeting(e, false, "no THATSIT");
            return;
        case SUPE_M_AWAIT_ANSWER: {
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            if (!m->listener && m->leg == 0 && !m->ourTrainConfirmed) {
                /* Contact was made and then the meeting died with our train
                 * unconfirmed: the power may have been too low, or the channel
                 * died under them — more power helps only the first, so the
                 * note carries the configuration it failed at. Without it the
                 * far end cannot weigh the loss against what that regime needs
                 * and has to treat every unanswered close as too little
                 * power (§15). */
                SupePeerNote nt = {};
                nt.ev = SUPE_EV_TRAIN_LOST;
                nt.cfg = m->cfg;
                nt.triedTxpDbm = m->trainTxp;
                if (m->haveTag) e->host->peer_note(e->host->ctx, m->tag, &nt);
                finishMeeting(e, false, "no answer");
                return;
            }
            /* Repairs went unacknowledged, or a close never came: give up —
             * what stands, stands. */
            finishMeeting(e, m->ourTrainConfirmed || m->listener, "gave up");
            return;
        }
        case SUPE_M_REPAIR_RX:
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            if (m->listener && m->leg == 1) { finishMeeting(e, true, "gave up"); return; }
            m->pendClose = 1;              /* what came, came: close the round */
            sendClose(e);
            return;
        default:
            /* A tx-in-flight phase: tx-done is the clock and no deadline may
             * stand (enterTxPhase). If one somehow does, it is stale by
             * definition — clear it rather than re-arming the host timer on
             * it at zero delay forever. */
            if (m->deadlineMs && (int32_t)(now - m->deadlineMs) >= 0)
                m->deadlineMs = 0;
            armTimer(e);
            return;
    }
}

size_t supeEngBuildAnn(SupeEngine* e, uint8_t* out, size_t cap,
                       const uint8_t ids[][SUPE_ID_LEN], uint8_t count,
                       int8_t pwrDbm, bool speaking) {
    SupeAnn2 a = {};
    /* A node that has SUPE turned off still announces — that is the only way to
     * tell a neighbour holding the opposite belief to stop, and going quiet
     * cannot say it: silence from a node that used to speak reads as a node
     * that has gone away, which is a peer to keep trying rather than one to
     * write off. The identities and the power ride the frame as usual, so the
     * neighbourhood keeps a correct picture of a node it simply may not meet. */
    a.regime = speaking ? e->regime : SUPE_REGIME_NONE;
    a.version = SUPE_VERSION;
    a.caps.fam = e->ownFam;
    a.caps.topStep = e->ownTop;
    a.caps.maxPwrDbm = e->txpMax;
    a.pwrDbm = pwrDbm;
    a.count = count > SUPE_ANN2_MAX ? SUPE_ANN2_MAX : count;
    for (int i = 0; i < a.count; i++) memcpy(a.ids[i], ids[i], SUPE_ID_LEN);
    return supeEncAnn2(out, cap, &a);
}
