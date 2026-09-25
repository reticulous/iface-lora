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

/* The pre-hail jitter, so two hailers that collide once do not collide again
 * in lockstep. */
#define SUPE_OFFER_JITTER_MS   24

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

/* The widest channel the regime offers — what a hail's ceiling is proposed
 * against, since the meeting's channel is not known when the hail is built. */
static uint32_t widestBwOf(const SupeEngine* e) {
    uint32_t maxBw = e->hailBwHz;
    int n = 0;
    const SupeChan* c = supeRegimeChans(e->regime, &n);
    for (int i = 0; i < n; i++) if (c[i].bwHz > maxBw) maxBw = c[i].bwHz;
    return maxBw;
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
 * answer an END, since which arrives is the answer itself. */
static uint32_t answerDeadlineMs(const SupeEngine* e, const SupeCfg* c) {
    return SUPE_TURNAROUND_MS
           + toaFrameMs(e, c, SUPE_GOT_ANS_BASE + SUPE_MASK_MAX, false)
           + SUPE_GUARD_MS;
}

/* The word a meeting's frames fly under at a spreading factor: derived under
 * a channel plan, the interface's own in regime 0, which never leaves the
 * hailing channel (§14.5). */
static uint8_t wordFor(const SupeEngine* e, uint8_t sf, uint8_t sByte) {
    return e->plan ? supeSyncWordAt(sf, e->ifaceSync, sByte) : e->ifaceSync;
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
                     const SupeCfg* cfg, int16_t rssi, int16_t snr10, int8_t txp) {
    if (!tagUsable(tag)) return;
    SupePeerNote n = {};
    n.ev = SUPE_EV_PAIR;
    n.cfg = *cfg;
    n.rssiDbm = rssi;
    n.snr10 = snr10;
    n.txpDbm = txp;
    e->host->peer_note(e->host->ctx, tag, &n);
}

static void noteReport(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN],
                       const SupeCfg* cfg, int16_t theirReading, int8_t theirSnrQ,
                       int8_t ourTxp) {
    if (!tagUsable(tag)) return;
    SupePeerNote n = {};
    n.ev = SUPE_EV_REPORT;
    n.cfg = *cfg;
    n.rssiDbm = theirReading;
    n.snr10 = supeDecSnr10(theirSnrQ);
    n.txpDbm = ourTxp;
    e->host->peer_note(e->host->ctx, tag, &n);
}

/* A hail unanswered: no answer by its deadline in regime 0, both slots unmet
 * under a plan, and no hail-back inside the interval that opens now (§12). */
static void noteUnanswered(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN], int8_t txp) {
    e->unanswered++;
    if (!tagUsable(tag)) return;
    SupePeerNote n = {};
    n.ev = SUPE_EV_UNANSWERED;
    n.backoffMs = SUPE_HAIL_INTERVAL_MS
                  + (e->host->rand32(e->host->ctx) % SUPE_HAIL_INTERVAL_JIT_MS);
    n.triedTxpDbm = txp;
    e->host->peer_note(e->host->ctx, tag, &n);
}

/* What we have just put on the air in this meeting, and what it flew at. The
 * peer's END states how our last frame reached it and cannot name which frame
 * that was; this is the other half of that measurement, and without the power
 * behind it a level is not a path loss (§15). */
static void noteOurTx(SupeMeet* m, int8_t txp, const SupeCfg* cfg) {
    m->lastTxp = txp;
    m->lastTxCfg = *cfg;
    m->haveLastTx = true;
}

/* ─────────────── owed hails ─────────────── */

static void owedAdd(SupeEngine* e, const uint8_t ident[SUPE_TAG_LEN], uint16_t peerId,
                    uint32_t hailEpochMs) {
    if (!tagUsable(ident)) return;          /* an anonymous hailer cannot be found */
    SupeOwed* o = nullptr;
    for (int i = 0; i < SUPE_OWED_MAX; i++) {
        SupeOwed* c = &e->owed[i];
        if (c->used && memcmp(c->ident, ident, SUPE_TAG_LEN) == 0) { o = c; break; }
        if (!c->used && !o) o = c;
    }
    if (!o) o = &e->owed[0];
    /* One owed hail per peer: a later hail renews the expiry and nothing else. */
    o->used = true;
    memcpy(o->ident, ident, SUPE_TAG_LEN);
    o->peerId = peerId;
    o->expiryMs = hailEpochMs + SUPE_PATIENCE_MS;
    eLog(e, true, "supe: owe %02x%02x%02x a hail", ident[0], ident[1], ident[2]);
}

static void owedDischarge(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN], uint16_t peerId) {
    for (int i = 0; i < SUPE_OWED_MAX; i++) {
        SupeOwed* c = &e->owed[i];
        if (!c->used) continue;
        if ((tag && memcmp(c->ident, tag, SUPE_TAG_LEN) == 0) ||
            (peerId != LORAQ_PEER_NONE && c->peerId == peerId))
            c->used = false;
    }
}

static SupeSched* schedFindLive(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN],
                                uint16_t peerId, bool narrowOnly);

static SupeOwed* owedDue(SupeEngine* e, uint32_t now) {
    for (int i = 0; i < SUPE_OWED_MAX; i++) {
        SupeOwed* c = &e->owed[i];
        if (!c->used) continue;
        if ((int32_t)(now - c->expiryMs) >= 0) { c->used = false; continue; }
        /* The peer has hailed again and its schedule is live: we speak there
         * instead, and the debt is paid by the meeting. */
        if (schedFindLive(e, c->ident, c->peerId, true)) continue;
        return c;
    }
    return nullptr;
}

/* ─────────────── lifecycle ─────────────── */

void supeEngInit(SupeEngine* e, const SupeHost* host, LoraQueue* q) {
    memset(e, 0, sizeof *e);
    e->host = host;
    e->q = q;
    e->m.schedIdx = -1;
}

void supeEngConfig(SupeEngine* e, uint8_t regime, uint8_t ownFam, uint8_t ownTop,
                   int8_t txpMax, uint8_t hailSf, uint32_t hailBwHz,
                   uint8_t crDenom, uint16_t preamble, uint8_t ifaceSync,
                   uint16_t maxFrameLen) {
    e->regime = regime;
    e->plan = supeRegimeHasPlan(regime);
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

/* A train built for a hail and never flown — the hail went unanswered, the
 * schedule expired, the radio went away — goes back to the queue's keeping. */
static void releaseHeldTrain(SupeEngine* e) {
    if (!e->m.txBuilt) return;
    e->host->train_done(e->host->ctx, false);
    e->m.txBuilt = false;
    e->m.tx.count = 0;
}

void supeEngReset(SupeEngine* e) {
    /* The radio went away underneath; no RF restore is owed — the whole modem
     * regime is re-applied on the way up. The learned stores survive; the
     * schedules do not, because their epochs are RF events on a radio that is
     * gone. */
    if (e->m.phase != SUPE_M_IDLE)
        eLog(e, true, "supe: meeting reset (radio went down) in phase %u",
             (unsigned)e->m.phase);
    releaseHeldTrain(e);
    memset(&e->m, 0, sizeof e->m);
    e->m.phase = SUPE_M_IDLE;
    e->m.schedIdx = -1;
    for (int i = 0; i < SUPE_SCHED_MAX; i++) e->sched[i].used = false;
    for (int i = 0; i < SUPE_OWED_MAX; i++) e->owed[i].used = false;
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
    return e->m.phase >= SUPE_M_GOT_TX;
}

uint16_t supeEngCargoPeer(const SupeEngine* e) {
    return e->m.phase >= SUPE_M_GOT_TX ? e->m.peerId : (uint16_t)LORAQ_PEER_NONE;
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

static bool queueHasFor(SupeEngine* e, const uint8_t* tag, uint16_t peerId);

/* A narrow schedule expiring unmet says one of two things, by role. Ours: the
 * hail went unanswered at its slots, and the interval for a hail-back opens
 * (§12). Theirs: we were hailed and never spoke, so we owe a hail. */
static void schedExpire(SupeEngine* e, SupeSched* s, uint32_t now) {
    (void)now;
    const char* kind = s->wide ? "wide" : "narrow";
    if (!s->wide && !s->consumed) {
        if (s->weHailed) {
            if (e->m.phase == SUPE_M_IDLE) releaseHeldTrain(e);
            noteUnanswered(e, s->haveTag ? s->tag : (const uint8_t*)"\0\0\0", s->hailTxp);
            eLog(e, false, "supe: hail %02x%02x%02x unanswered — %u listened, %u yielded",
                 s->d.hash3[0], s->d.hash3[1], s->d.hash3[2],
                 (unsigned)s->nOwnDue, (unsigned)s->nLate);
        } else {
            if (s->haveTag) owedAdd(e, s->tag, s->peerId, s->epochMs);
            eLog(e, false, "supe: sched %02x%02x%02x %s unmet — %u due "
                 "(%u busy, %u late, %u no-tune)%s",
                 s->d.hash3[0], s->d.hash3[1], s->d.hash3[2], kind,
                 (unsigned)s->nOwnDue, (unsigned)s->nBusy, (unsigned)s->nLate,
                 (unsigned)s->nNoTune, s->haveTag ? " — owing a hail" : "");
        }
    } else if (s->nSpoke > 0) {
        eLog(e, false, "supe: sched %02x%02x%02x %s unanswered — spoke %u of %u",
             s->d.hash3[0], s->d.hash3[1], s->d.hash3[2], kind,
             (unsigned)s->nSpoke, (unsigned)s->nOwnDue);
    } else {
        eLog(e, true, "supe: sched %02x%02x%02x %s idle — %u of %u",
             s->d.hash3[0], s->d.hash3[1], s->d.hash3[2], kind,
             (unsigned)s->nSpoke, (unsigned)s->nOwnDue);
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

/* Forget a node (supe_engine.h): the hail we owe it, the schedules we hold with
 * it, the proof returns filed against it. A proof return names its node by four
 * bytes and a tag is three, so the match is on the three the protocol addresses
 * by — the same prefix every other lookup here uses. The meeting is deliberately
 * untouched. */
void supeEngForget(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN]) {
    if (!e || !tagUsable(tag)) return;
    for (int i = 0; i < SUPE_OWED_MAX; i++) {
        SupeOwed* o = &e->owed[i];
        if (o->used && memcmp(o->ident, tag, SUPE_TAG_LEN) == 0) o->used = false;
    }
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        SupeSched* s = &e->sched[i];
        if (s->used && s->haveTag && memcmp(s->tag, tag, SUPE_TAG_LEN) == 0) schedFree(e, s);
    }
    for (int i = 0; i < SUPE_PROOFRET_MAX; i++) {
        SupeProofRet* r = &e->pret[i];
        if (r->used && memcmp(r->node4, tag, SUPE_TAG_LEN) == 0) r->used = false;
    }
}

static SupeSched* schedFindLive(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN],
                                uint16_t peerId, bool narrowOnly) {
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        SupeSched* s = &e->sched[i];
        if (!s->used || s->consumed) continue;
        if (narrowOnly && s->wide) continue;
        if (s->haveTag && tag && memcmp(s->tag, tag, SUPE_TAG_LEN) == 0) return s;
        if (peerId != LORAQ_PEER_NONE && s->peerId == peerId) return s;
    }
    return nullptr;
}

static bool schedLiveFor(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN], uint16_t peerId) {
    return schedFindLive(e, tag, peerId, false) != nullptr;
}
/* Any narrow schedule still to be met, whoever it is with. Its slots are
 * appointments — the hailed party speaks at them, the hailer listens — and a
 * hail of our own put on the air now would fall into them: the far end
 * answers at slots we are not at, and the appointment we hold is missed for
 * the hail. Wide schedules are not this: their slots are sparse and short. */
static bool schedNarrowLive(const SupeEngine* e) {
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        const SupeSched* s = &e->sched[i];
        if (s->used && !s->consumed && !s->wide) return true;
    }
    return false;
}

/* Derive and install a schedule from a seeding frame both ends hold. */
static SupeSched* schedInstall(SupeEngine* e, const uint8_t* seed, uint16_t seedLen,
                               bool wide, bool weHailed, bool weTx0,
                               const uint8_t* tag, uint16_t peerId,
                               const SupeCfg* slotCfg, uint32_t epochMs) {
    uint8_t d0[32], d1[32];
    uint8_t buf[SUPE_END_BASE + SUPE_TRAIN_MAX + 1];
    if ((size_t)seedLen + 1 > sizeof buf) return nullptr;
    e->host->sha256(e->host->ctx, seed, seedLen, d0);
    memcpy(buf, seed, seedLen);
    buf[seedLen] = 0x01;
    e->host->sha256(e->host->ctx, buf, (uint16_t)(seedLen + 1), d1);

    int nChans = 0;
    supeRegimeChans(e->regime, &nChans);

    /* A schedule already held under this hash is superseded, not duplicated:
     * same seed bytes, new epoch. The salt makes this collision rare; this is
     * the belt that makes it harmless. */
    for (int i = 0; i < SUPE_SCHED_MAX; i++)
        if (e->sched[i].used && memcmp(e->sched[i].d.hash3, d0, SUPE_HASH_LEN) == 0)
            e->sched[i].used = false;

    /* One narrow schedule per peer per direction (§7). A hail retried by the
     * same hailer retires its predecessor at both ends — the hailed party
     * that could not meet the first will not meet the second any sooner. A
     * hail crossing ours is the other direction and stays beside it. */
    bool newHasTag = tag != nullptr && tagUsable(tag);
    if (!wide && (newHasTag || peerId != LORAQ_PEER_NONE)) {
        for (int i = 0; i < SUPE_SCHED_MAX; i++) {
            SupeSched* s = &e->sched[i];
            if (!s->used || s->wide || s->weHailed != weHailed) continue;
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
    s->weHailed = weHailed;
    s->weTx0 = weTx0;
    s->haveTag = newHasTag;
    if (s->haveTag) memcpy(s->tag, tag, SUPE_TAG_LEN);
    s->peerId = peerId;
    s->epochMs = epochMs;
    s->slotCfg = *slotCfg;
    e->schedsIn++;
    eLog(e, true, "supe: schedule %02x%02x%02x (%s, %u slots, we %s)",
         s->d.hash3[0], s->d.hash3[1], s->d.hash3[2],
         wide ? "wide" : "narrow", (unsigned)s->d.nSlots,
         wide ? (weTx0 ? "tx even" : "tx odd") : (weHailed ? "listen" : "speak"));
    return s;
}

/* Which side speaks at slot k. Narrow: the hailed party, at every slot — the
 * hail asked it a question. Wide: alternating from whoever received the last
 * train. */
static bool slotIsMine(const SupeSched* s, uint8_t k) {
    if (!s->wide) return !s->weHailed;
    return s->weTx0 == ((k & 1) == 0);
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
            bool mine = slotIsMine(s, k);
            /* The listener aims to be listening a slot-guard early: late by
             * its own wake slop it is still there before the speaker. */
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
    if (m->rxN == 0 || m->lite) { m->rxN = 0; return; }
    uint8_t order[SUPE_TRAIN_MAX];
    uint8_t n = 0;
    if (!m->rxAligned) {
        /* No END ever came: the frames arrived in transmitted order, which
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
 * printed first, so the order on the line is the order on the air. */
static void meetingLine(SupeEngine* e, uint8_t got, bool ok, const char* why) {
    SupeMeet* m = &e->m;
    bool weOpened = !m->listener;
    char hail[32] = "", ours[32] = "", theirs[32] = "", counts[64] = "";

    if (m->fromHail && m->haveHail)
        fmtLeg(hail, sizeof hail, m->weHailed ? "tx" : "rx", m->hailTxp,
               true, m->hailRssi, m->hailSnrQ);
    if (m->txFired)
        fmtLeg(ours, sizeof ours, "tx", m->trainTxp, m->haveOurRead,
               m->ourTrainRssi, m->ourTrainSnrQ);
    if (m->havePeerTxp)
        fmtLeg(theirs, sizeof theirs, "rx", m->peerTrainTxp, m->anyRx,
               m->worstRssi, m->worstSnrQ);
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
        const char* a = weOpened ? sent : rcvd;
        const char* b = weOpened ? rcvd : sent;
        if (a[0] && b[0]) snprintf(counts, sizeof counts, " - %s, %s", a, b);
        else if (a[0] || b[0]) snprintf(counts, sizeof counts, " - %s", a[0] ? a : b);
    }

    eLog(e, false, "supe: %s %s%s%s %u/%lu/%u%s:%s%s%s%s%s%s%s",
         m->weHailed ? "Our" : "Their", m->fromHail ? "hail" : "rndv",
         hail[0] ? " " : "", hail,
         m->chan, (unsigned long)(m->cfg.bwHz / 1000u), m->cfg.sf,
         m->lite ? " lite" : "",
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
    /* A train flown at the hailing rate is done with — nothing proves it and
     * nothing repairs it; the daemon's retry is the only one there is (§8). */
    if (m->txBuilt) e->host->train_done(e->host->ctx,
                                        m->ourTrainConfirmed || (m->lite && m->txFired));
    m->txBuilt = false;

    /* Every goodbye keys the next schedule, and the seed must be a frame both
     * ends can PROVE the other holds (§7) — received, or answered. And it must
     * be the meeting's LAST END. Only a channel plan derives one. */
    bool goodbyeShared = (m->weReceivedFinal || m->ourTrainConfirmed)
                         && !m->laterEnd;
    if (e->plan && !m->lite && m->lastEndLen && was >= SUPE_M_TRAIN_RX &&
        goodbyeShared) {
        SupeCfg cfg = m->cfg;
        SupeSched* s = schedInstall(e, m->lastEnd, m->lastEndLen, /*wide=*/true,
                                    /*weHailed=*/false, /*weTx0=*/m->weReceivedFinal,
                                    m->haveTag ? m->tag : nullptr, m->peerId, &cfg, eNow(e));
        if (s) {
            s->lastRssi = m->lastRssi;
            s->lastSnrQ = m->lastSnrQ;
            s->haveLastTx = m->haveLastTx;
            s->lastTxp = m->lastTxp;
            s->lastTxCfg = m->lastTxCfg;
        }
    }
    if (ok) {
        e->meetingsDone++;
        if (m->haveTag) noteSimple(e, m->tag, SUPE_EV_MET);
        owedDischarge(e, m->haveTag ? m->tag : nullptr, m->peerId);
        /* Contact consumes every narrow schedule between the pair: a crossed
         * hail's other schedule has been answered by this meeting, and its
         * expiry would otherwise score a peer that just met (§7). */
        for (int i = 0; i < SUPE_SCHED_MAX; i++) {
            SupeSched* s = &e->sched[i];
            if (!s->used || s->wide || s->consumed) continue;
            bool same = (m->haveTag && s->haveTag &&
                         memcmp(s->tag, m->tag, SUPE_TAG_LEN) == 0) ||
                        (m->peerId != LORAQ_PEER_NONE && s->peerId == m->peerId);
            if (same) s->consumed = true;
        }
    } else if (!m->weHailed && m->fromHail && !m->anyRx && was >= SUPE_M_READY_TX &&
               m->haveTag) {
        /* We answered a hail and nothing came of it — the hailer may have been
         * caught by a frame on the hailing channel. It is still owed a hail. */
        owedAdd(e, m->tag, m->peerId, m->beganMs);
    }

    SupeEngine::SupeEndRec* er = &e->ends[e->endsAt];
    e->endsAt = (uint8_t)((e->endsAt + 1) % (sizeof e->ends / sizeof e->ends[0]));
    er->why = why;          er->endedMs = eNow(e);
    er->phase = was;        er->chan = m->chan;
    er->listener = m->listener;
    er->ok = ok;
    er->sent = m->txNext;   er->got = got;
    er->expect = m->exCount;

    if (was >= SUPE_M_GOT_TX) meetingLine(e, got, ok, why);

    memset(m, 0, sizeof *m);
    m->phase = SUPE_M_IDLE;
    m->schedIdx = -1;
    e->host->rx(e->host->ctx);
    armTimer(e);
}

/* ─────────────── the classifier and the hail ─────────────── */

int shouldDetour(const SupePeerView* peer, const LoraQueue* q,
                 const SupeChanView* chans, uint32_t now,
                 uint32_t* wait_until_ms) {
    /* The one deliberately-open decision (SUPE.md §18): inputs are the peer,
     * the queue and the channels, and the answer is policy. A peer to meet is
     * a meeting worth hailing — the shared-channel cost is one short frame per
     * batch. */
    (void)peer; (void)q; (void)chans; (void)now; (void)wait_until_ms;
    return DETOUR_NOW;
}

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
    /* Keyed on the reason and the peer, NOT the length: one line per peer per
     * reason says the same thing and stops. */
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
    uint32_t now = eNow(e);
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

    /* Patience is per packet, from the moment it was queued (§12): the run
     * drops nothing, and a packet leaves at its own age, whatever the run is
     * doing. Well inside the daemon's own timer, so its retry never goes into
     * the air beside a copy the modem still holds. */
    if ((uint32_t)(now - p->first_seen_ms) >= SUPE_PATIENCE_MS) {
        e->dropsPatience++;
        e->offerArmed = false;
        eLog(e, true, "supe: %uB for %02x%02x%02x dropped at patience",
             (unsigned)p->len, p->tag[0], p->tag[1], p->tag[2]);
        return SUPE_V_DROP;
    }

    /* A live schedule with this peer: the packet rides the next met slot
     * rather than contending on the shared channel. */
    if (schedLiveFor(e, p->tag, p->peer_id)) {
        e->offerArmed = false;
        return SUPE_V_WAIT;
    }
    if (e->m.phase != SUPE_M_IDLE && e->m.haveTag &&
        (memcmp(e->m.tag, p->tag, SUPE_TAG_LEN) == 0 ||
         (e->m.peerId != LORAQ_PEER_NONE && e->m.peerId == p->peer_id))) {
        e->offerArmed = false;
        return SUPE_V_WAIT;                /* its hail or meeting is running right now */
    }
    /* Somebody else's slots are still to be met: no hail goes out into them
     * (schedNarrowLive). The offer stays unarmed and is asked again after. */
    if (schedNarrowLive(e)) {
        e->offerArmed = false;
        return SUPE_V_WAIT;
    }

    SupePeerView pv;
    if (!e->host->peer_get(e->host->ctx, p->tag, &pv) || !pv.known) {
        e->offerArmed = false;
        notePlain(e, PLAIN_NOT_PEER, p);
        return SUPE_V_PLAIN;
    }
    /* Unreachable, and in a hold: no hail is sent, and the traffic is queued
     * rather than refused — each packet waits out its own patience (§12). */
    if (pv.holdUntilMs && (int32_t)(pv.holdUntilMs - now) > 0) {
        e->offerArmed = false;
        e->dropsHold++;
        return SUPE_V_WAIT;
    }
    if (pv.intervalUntilMs && (int32_t)(pv.intervalUntilMs - now) > 0) {
        /* Mid-run: the interval in which the hailed party may hail back. */
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
    if (e->m.phase != SUPE_M_IDLE) return false;
    /* Asked at the moment of launch and not only at the verdict, because a
     * hail can wait most of a second for the channel and a peer's hail can
     * seed a schedule meanwhile — its slots come first, and the launch is
     * retried once they are met or void. */
    if (schedNarrowLive(e)) return false;
    uint32_t now = e->host->now_ms(e->host->ctx);
    if (e->offerArmed && (int32_t)(now - e->offerJitterUntilMs) >= 0) return true;
    return owedDue((SupeEngine*)e, now) != nullptr;
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

/* The highest budget this node proposes toward a peer on a channel, and what
 * it resolves to: as far up the channel's ladder as both ceilings allow. */
static uint8_t proposalTop(SupeEngine* e, const uint8_t* tag, bool haveTag,
                           uint32_t chanMaxBw, SupeCfg* cfgOut) {
    SupePeerView pv = {};
    uint8_t peerTop = 14;
    if (haveTag && tagUsable(tag) && e->host->peer_get(e->host->ctx, tag, &pv) && pv.known)
        peerTop = pv.topBudget;
    SupeLadderEntry lad[SUPE_LADDER_MAX_ENTRIES];
    int ln = supeLadder(e->regime, SUPE_VERSION, e->hailSf, e->hailBwHz, chanMaxBw,
                        e->ownFam, peerFamOf(e, tag, haveTag), lad, SUPE_LADDER_MAX_ENTRIES);
    int top = ln > 0 ? ln - 1 : 0;
    if (top > e->ownTop) top = e->ownTop;
    if (top > peerTop)   top = peerTop;
    if (cfgOut) {
        if (ln > 0) *cfgOut = SupeCfg{ lad[top].sf, lad[top].bwHz, lad[top].ldro,
                                       lad[top].marginDeci };
        else        *cfgOut = hailCfgOf(e);
    }
    return (uint8_t)top;
}

/* Trim a train to the regime's ceiling at a configuration. The frames stay
 * built; only the count travels. */
static void trimToCeiling(SupeEngine* e, SupeTrainInfo* t, const SupeCfg* cfg) {
    const SupeRegime* g = supeRegime(e->regime);
    uint32_t ceil = SUPE_LEN_MAX_MS;
    if (g && g->trainCeilMs && g->trainCeilMs < ceil) ceil = g->trainCeilMs;
    while (t->count > 1 && trainLenMs(e, t, t->count, cfg) > ceil) t->count--;
}

void supeEngLaunch(SupeEngine* e) {
    SupeMeet* m = &e->m;
    if (m->phase != SUPE_M_IDLE) return;
    uint32_t now = eNow(e);

    bool offer = e->offerArmed && (int32_t)(now - e->offerJitterUntilMs) >= 0;
    e->offerArmed = false;
    SupeOwed* owed = offer ? nullptr : owedDue(e, now);
    if (!offer && !owed) return;

    SupeHail h = {};
    h.regime = e->regime;
    h.version = SUPE_VERSION;
    h.salt = (uint8_t)e->host->rand32(e->host->ctx);
    h.haveIdent = e->haveOwnIdent;
    if (h.haveIdent) memcpy(h.ident, e->ownIdent, SUPE_TAG_LEN);
    int8_t txp;
    uint16_t peerId;
    uint8_t tag[SUPE_TAG_LEN];
    SupeTrainInfo tx = {};
    bool built = false;

    if (offer) {
        LoraPkt* p = loraqAt(e->q, 0);
        if (!p || !(p->flags & LORAQ_F_HAVE_TAG)) return;
        SupePeerView pv;
        if (!e->host->peer_get(e->host->ctx, p->tag, &pv) || !pv.known) return;
        memcpy(tag, p->tag, SUPE_TAG_LEN);
        peerId = pv.peerId;
        /* The run's power axis (§12, §15): the first hail at what the evidence
         * says, the second halfway to maximum, the third and later at maximum. */
        txp = pv.unanswered == 0 ? pv.txpOpen
            : pv.unanswered == 1 ? (int8_t)(((int)pv.txpOpen + e->txpMax + 1) / 2)
            : e->txpMax;
        if (txp > e->txpMax) txp = e->txpMax;
        /* The hail describes the train it announces, so the train is built
         * now and held until the answer, or the run's end. */
        if (!e->host->train_build(e->host->ctx, peerId, tag, SUPE_TRAIN_MAX, &tx) ||
            tx.count == 0) {
            e->plainOnce = true;
            return;
        }
        built = true;
        SupeCfg ceilCfg;
        h.budgetCeil = proposalTop(e, tag, true, widestBwOf(e), &ceilCfg);
        trimToCeiling(e, &tx, &ceilCfg);
        h.count = tx.count;
        h.lenByte = supeEncLen(trainLenMs(e, &tx, tx.count, &ceilCfg));
        owedDischarge(e, tag, peerId);       /* a hail with traffic pays the debt too */
    } else {
        /* The hail-back: a hail with a count of zero, tagged with the identity
         * the hail it answers carried. The hailed party — the node whose
         * traffic prompted it — answers with GOT (§8). */
        memcpy(tag, owed->ident, SUPE_TAG_LEN);
        peerId = owed->peerId;
        owed->used = false;
        SupeCfg hail = hailCfgOf(e);
        txp = e->host->txp_open(e->host->ctx, tag, &hail);
        if (txp > e->txpMax) txp = e->txpMax;
        h.budgetCeil = proposalTop(e, tag, true, widestBwOf(e), nullptr);
        h.count = 0;
        h.lenByte = 0;
    }
    memcpy(h.tag, tag, SUPE_TAG_LEN);
    h.pwrDbm = txp;

    uint8_t f[SUPE_HAIL_ID_LEN];
    size_t n = supeEncHail(f, sizeof f, &h);
    if (!n) { if (built) e->host->train_done(e->host->ctx, false); return; }

    memset(m, 0, sizeof *m);
    m->phase = SUPE_M_HAIL_TX;
    m->schedIdx = -1;
    m->weHailed = true;
    m->fromHail = true;
    memcpy(m->tag, tag, SUPE_TAG_LEN);
    m->haveTag = true;
    m->peerId = peerId;
    m->hailTxp = txp;
    m->hailCeil = h.budgetCeil;
    m->hailCount = h.count;
    m->hailLen = h.lenByte;
    m->budget = h.budgetCeil;             /* our proposal, for the answer to stay under */
    m->tx = tx;
    m->txBuilt = built;
    memcpy(m->hailFrame, f, n);
    m->hailFrameLen = (uint8_t)n;
    /* The hash both ends quote: the first three bytes of SHA-256 over the
     * hail as transmitted. Under a plan the schedule derives from the same
     * digest; in regime 0 this is all of it. */
    uint8_t d0[32];
    e->host->sha256(e->host->ctx, f, (uint16_t)n, d0);
    memcpy(m->hash3, d0, SUPE_HASH_LEN);
    m->beganMs = now;
    if (!e->host->tx_frame(e->host->ctx, f, (uint16_t)n, txp)) {
        finishMeeting(e, false, "HAIL would not transmit");
        e->plainOnce = offer;
        return;
    }
    {   SupeCfg hc = hailCfgOf(e); noteOurTx(m, txp, &hc); }
    if (built) e->hailsOut++; else { e->hailBacksOut++; e->owedHails++; }
    eLog(e, true, "supe: HAIL %02x%02x%02x %u frames ceiling %u txp=%d%s",
         tag[0], tag[1], tag[2], (unsigned)h.count, (unsigned)h.budgetCeil,
         (int)txp, built ? "" : " (hail-back)");
}

/* ─────────────── opening a meeting ─────────────── */

static bool queueHasFor(SupeEngine* e, const uint8_t* tag, uint16_t peerId) {
    for (uint8_t i = 0; i < loraqDepth(e->q); i++) {
        LoraPkt* p = loraqAt(e->q, i);
        if (tag && tagUsable(tag) && (p->flags & LORAQ_F_HAVE_TAG) &&
            memcmp(p->tag, tag, SUPE_TAG_LEN) == 0) return true;
        if (peerId != LORAQ_PEER_NONE && p->peer_id == peerId) return true;
    }
    return false;
}

/* Our meeting frames' power at a configuration (§15): the controller's
 * derivation for the peer, never above the channel's cap. */
static int8_t meetTxp(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN], bool haveTag,
                      uint8_t chan, const SupeCfg* cfg) {
    int8_t cap = chanTxpCapOf(e, chan);
    if (!haveTag || !tagUsable(tag)) return cap;
    int8_t open = e->host->txp_open(e->host->ctx, tag, cfg);
    return open < cap ? open : cap;
}

/* Entering a phase that ends with the host's tx completion: no deadline may
 * stand. nextEventMs reads deadlineMs == 0 as "no wake until tx-done". */
static void enterTxPhase(SupeMeet* m, uint8_t phase) {
    m->phase = phase;
    m->deadlineMs = 0;
}

static void startTrain(SupeEngine* e, uint32_t leadMs);
static void enterTrainWait(SupeEngine* e, uint32_t gapMs);

/* The receiver's budget: never above the proposal, never above our own
 * ceiling, from the freshest reading there is — the frame being answered —
 * normalised to headroom over the hailing floor. */
static uint8_t chooseBudget(SupeEngine* e, uint8_t chan, uint8_t proposal,
                            const SupeCfg* readCfg, int snr10,
                            const uint8_t* tag, bool haveTag, SupeCfg* cfgOut) {
    SupeLadderEntry lad[SUPE_LADDER_MAX_ENTRIES];
    int ln = supeLadder(e->regime, SUPE_VERSION, e->hailSf, e->hailBwHz,
                        chanMaxBwOf(e, chan), e->ownFam, peerFamOf(e, tag, haveTag),
                        lad, SUPE_LADDER_MAX_ENTRIES);
    if (ln <= 0) { *cfgOut = hailCfgOf(e); return 0; }
    int top = ln - 1;
    if (top > proposal)   top = proposal;
    if (top > e->ownTop)  top = e->ownTop;
    int headroomDeci = snr10 - (int)supeReqSnrDeci(readCfg->sf) + (int)readCfg->marginDeci;
    int affordDeci = headroomDeci - SUPE_TARGET_MARGIN_DB * 10;
    uint8_t budget = 0;
    for (int i = 1; i <= top; i++)
        if ((int)lad[i].marginDeci <= affordDeci) budget = (uint8_t)i;
    *cfgOut = SupeCfg{ lad[budget].sf, lad[budget].bwHz, lad[budget].ldro,
                       lad[budget].marginDeci };
    return budget;
}

/* Move the radio to the confirmed budget, if it differs from the answer's
 * configuration. Regime 0 stays on the hailing frequency under the interface's
 * own word; a plan re-indexes the slot's word byte against the new SF. */
static bool tuneToBudget(SupeEngine* e) {
    SupeMeet* m = &e->m;
    if (m->cfg.sf == m->slotCfg.sf && m->cfg.bwHz == m->slotCfg.bwHz) return true;
    uint8_t word = wordFor(e, m->cfg.sf, m->sByte);
    if (!e->host->tune(e->host->ctx, m->chan, &m->cfg, word)) return false;
    m->retuned = true;
    m->atTrainCfg = true;
    return true;
}

/* The hailed party's answer, from a slot or a turnaround after the hail: GOT
 * where it holds traffic for the hailer, READY otherwise (§8). The meeting
 * record already names the hail — hash, tag, what it declared, how it was
 * heard — and the radio is on the answer's configuration. */
static void openAsHailed(SupeEngine* e, SupeSched* s) {
    SupeMeet* m = &e->m;
    SupeCfg hailCfg = hailCfgOf(e);
    m->fromHail = true;
    m->weHailed = false;
    m->hailTxp = s ? s->hailTxp : m->hailTxp;
    if (s) {
        m->hailCount = s->hailCount;
        m->hailLen = s->hailLen;
        m->hailCeil = s->hailCeil;
        m->haveHail = s->haveHeard;
        m->hailRssi = s->heardRssi;
        m->hailSnrQ = s->heardSnrQ;
    }
    /* The hail is the last thing we heard from this peer, and our own END will
     * quote it back — the schedule carried it across the wait. */
    if (m->haveHail) {
        m->lastRssi = m->hailRssi;
        m->lastSnrQ = m->hailSnrQ;
        m->haveLast = true;
    }
    m->beganMs = eNow(e);
    m->worstRssi = 127;
    m->ourTxp = meetTxp(e, m->tag, m->haveTag, m->chan, &m->slotCfg);

    /* A hail-back's hail names its sender by the same rule as any other, so
     * "do we hold traffic for the hailer" is one question in both cases. The
     * host has one train buffer: a train held for a hail of our own is let go
     * first, and rebuilt when that hail's answer comes (ensureHailerTrain). */
    releaseHeldTrain(e);
    bool haveTrain = false;
    if (m->haveTag || m->peerId != LORAQ_PEER_NONE) {
        SupeTrainInfo ti;
        if (e->host->train_build(e->host->ctx, m->peerId,
                                 m->haveTag ? m->tag : nullptr, SUPE_TRAIN_MAX, &ti) &&
            ti.count > 0) {
            m->tx = ti;
            m->txBuilt = true;
            haveTrain = true;
        }
    }

    SupeReady g = {};
    memcpy(g.hash, m->hash3, SUPE_HASH_LEN);
    g.pwrDbm = m->ourTxp;
    g.countCeil = SUPE_TRAIN_MAX;
    g.heardRssi = m->hailRssi;
    g.heardSnrQ = m->hailSnrQ;

    if (haveTrain) {
        /* GOT: our train goes first, the hailer's rides the answering turn.
         * The budget byte is our proposal for the meeting — as far up this
         * channel's ladder as both ceilings allow; the hailer's READY confirms
         * one at or below it, and below its own hail's ceiling. */
        SupeCfg propCfg;
        uint8_t top = proposalTop(e, m->tag, m->haveTag, chanMaxBwOf(e, m->chan), &propCfg);
        trimToCeiling(e, &m->tx, &propCfg);
        m->listener = false;
        m->budget = top;
        m->cfg = m->slotCfg;
        m->peerCeil = SUPE_TRAIN_MAX;
        SupeGot hv = {};
        hv.g = g;
        hv.g.budget = top;
        hv.count = m->tx.count;
        hv.lenByte = supeEncLen(trainLenMs(e, &m->tx, m->tx.count, &propCfg));
        uint8_t f[SUPE_GOT_LEN];
        size_t n = supeEncGot(f, sizeof f, &hv);
        if (!n || !e->host->tx_frame(e->host->ctx, f, (uint16_t)n, m->ourTxp)) {
            finishMeeting(e, false, "GOT would not transmit");
            return;
        }
        noteOurTx(m, m->ourTxp, &m->slotCfg);
        enterTxPhase(m, SUPE_M_GOT_TX);
        eLog(e, true, "supe: GOT ch%u %u frames, propose budget %u, txp=%d",
             (unsigned)m->chan, (unsigned)hv.count, (unsigned)top, (int)m->ourTxp);
        return;
    }

    /* READY: the receiver's terms. The budget is chosen from how the hail was
     * heard, never above the hail's ceiling. In regime 0 it selects the
     * dialogue's shape: 0 is hail, answer, frames and nothing else. */
    SupeCfg cfg;
    int snr10 = m->haveHail ? (int)supeDecSnr10(m->hailSnrQ) : 0;
    uint8_t budget = chooseBudget(e, m->chan, m->hailCeil, &hailCfg, snr10,
                                  m->tag, m->haveTag, &cfg);
    m->listener = true;
    m->budget = budget;
    m->cfg = cfg;
    m->lite = !e->plan && budget == 0;
    m->exCount = m->hailCount < SUPE_TRAIN_MAX ? m->hailCount : SUPE_TRAIN_MAX;
    m->peerTrainTxp = m->hailTxp;          /* until their END states the train's */
    m->havePeerTxp = true;
    g.budget = budget;
    uint8_t f[SUPE_READY_LEN];
    size_t n = supeEncReady(f, sizeof f, &g);
    if (!n || !e->host->tx_frame(e->host->ctx, f, (uint16_t)n, m->ourTxp)) {
        finishMeeting(e, false, "READY would not transmit");
        return;
    }
    noteOurTx(m, m->ourTxp, &m->slotCfg);
    enterTxPhase(m, SUPE_M_READY_TX);
    eLog(e, true, "supe: READY budget %u txp=%d for %u frames%s",
         (unsigned)budget, (int)m->ourTxp, (unsigned)m->exCount,
         m->lite ? " (lite)" : "");
}

/* The wide schedule's speaker: a holder of traffic, in its own slot, opening
 * with GOT. Nobody asked anything at a goodbye, so this frame is the one
 * that flies blind — sensed first, on a private channel, bounded (§7). */
static void slotSpeakWide(SupeEngine* e, SupeSched* s, uint8_t k) {
    SupeMeet* m = &e->m;
    const SupeSlotD* sl = &s->d.slot[k];
    uint8_t word = wordFor(e, s->slotCfg.sf, sl->sByte);
    if (!e->host->tune(e->host->ctx, sl->chan, &s->slotCfg, word)) {
        if (s->nNoTune < 255) s->nNoTune++;
        s->nextSlot = (uint8_t)(k + 1);
        return;
    }
    m->retuned = true;
    if (e->host->cca && !e->host->cca(e->host->ctx)) {
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
    releaseHeldTrain(e);                  /* one host buffer (openAsHailed) */
    if (!e->host->train_build(e->host->ctx, s->peerId,
                              s->haveTag ? s->tag : nullptr, SUPE_TRAIN_MAX, &m->tx) ||
        m->tx.count == 0) {
        if (s->nNoTrain < 255) s->nNoTrain++;
        e->host->tune_home(e->host->ctx);
        m->retuned = false;
        s->nextSlot = (uint8_t)(k + 1);
        armTimer(e);
        return;
    }
    m->txBuilt = true;
    SupeCfg propCfg;
    uint8_t top = proposalTop(e, s->tag, s->haveTag, chanMaxBwOf(e, sl->chan), &propCfg);
    trimToCeiling(e, &m->tx, &propCfg);

    m->listener = false;
    m->weHailed = false;
    m->fromHail = false;
    m->schedIdx = (int8_t)(s - e->sched);
    memcpy(m->hash3, s->d.hash3, SUPE_HASH_LEN);
    if (s->haveTag) { memcpy(m->tag, s->tag, SUPE_TAG_LEN); m->haveTag = true; }
    m->peerId  = s->peerId;
    m->chan    = sl->chan;
    m->sByte   = sl->sByte;
    m->slotCfg = s->slotCfg;
    m->cfg     = s->slotCfg;
    m->budget  = top;
    m->peerCeil = SUPE_TRAIN_MAX;
    m->ourTxp  = meetTxp(e, s->tag, s->haveTag, sl->chan, &s->slotCfg);
    m->worstRssi = 127;
    m->beganMs = eNow(e);

    SupeGot hv = {};
    memcpy(hv.g.hash, m->hash3, SUPE_HASH_LEN);
    hv.g.pwrDbm  = m->ourTxp;
    hv.g.budget  = top;
    hv.g.countCeil = SUPE_TRAIN_MAX;
    hv.g.heardRssi = s->lastRssi;        /* the last frame heard from the peer */
    hv.g.heardSnrQ = s->lastSnrQ;
    hv.count   = m->tx.count;
    hv.lenByte = supeEncLen(trainLenMs(e, &m->tx, m->tx.count, &propCfg));
    uint8_t f[SUPE_GOT_LEN];
    size_t n = supeEncGot(f, sizeof f, &hv);
    if (!n || !e->host->tx_frame(e->host->ctx, f, (uint16_t)n, m->ourTxp)) {
        finishMeeting(e, false, "GOT would not transmit");
        return;
    }
    noteOurTx(m, m->ourTxp, &m->slotCfg);
    e->slotsSpoken++;
    if (s->nSpoke < 255) s->nSpoke++;
    enterTxPhase(m, SUPE_M_GOT_TX);
    s->nextSlot = (uint8_t)(k + 1);
    eLog(e, true, "supe: GOT ch%u %u frames, propose budget %u, txp=%d (rndv)",
         (unsigned)sl->chan, (unsigned)hv.count, (unsigned)top, (int)m->ourTxp);
}

/* The narrow schedule's speaker: the hailed party, answering at a slot. */
static void slotSpeakNarrow(SupeEngine* e, SupeSched* s, uint8_t k) {
    SupeMeet* m = &e->m;
    const SupeSlotD* sl = &s->d.slot[k];
    uint8_t word = wordFor(e, s->slotCfg.sf, sl->sByte);
    if (!e->host->tune(e->host->ctx, sl->chan, &s->slotCfg, word)) {
        if (s->nNoTune < 255) s->nNoTune++;
        s->nextSlot = (uint8_t)(k + 1);
        return;
    }
    m->retuned = true;
    if (e->host->cca && !e->host->cca(e->host->ctx)) {
        /* The appointment grants the peer's attention, never the spectrum: a
         * busy channel skips the slot, and the second slot is on another. */
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
    m->schedIdx = (int8_t)(s - e->sched);
    memcpy(m->hash3, s->d.hash3, SUPE_HASH_LEN);
    if (s->haveTag) { memcpy(m->tag, s->tag, SUPE_TAG_LEN); m->haveTag = true; }
    else m->haveTag = false;
    m->peerId  = s->peerId;
    m->chan    = sl->chan;
    m->sByte   = sl->sByte;
    m->slotCfg = s->slotCfg;
    m->cfg     = s->slotCfg;
    e->slotsSpoken++;
    if (s->nSpoke < 255) s->nSpoke++;
    s->nextSlot = (uint8_t)(k + 1);
    /* Contact is what the schedule was for: speaking at a slot consumes it
     * either way — a second slot after an answered first is void, and after
     * an unanswered first the hailer is owed a hail rather than a repeat. */
    s->consumed = true;
    openAsHailed(e, s);
    if (m->phase != SUPE_M_IDLE) m->schedIdx = -1;
    schedFree(e, s);
}

/* The listening side's slot: a window, stop-on-preamble in spirit. The
 * hailer, on a narrow schedule; the non-holder's turn on a wide one. */
static void slotListen(SupeEngine* e, SupeSched* s, uint8_t k) {
    SupeMeet* m = &e->m;
    const SupeSlotD* sl = &s->d.slot[k];
    uint8_t word = wordFor(e, s->slotCfg.sf, sl->sByte);
    if (!e->host->tune(e->host->ctx, sl->chan, &s->slotCfg, word)) {
        s->nextSlot = (uint8_t)(k + 1);
        return;
    }
    e->host->rx(e->host->ctx);
    e->slotsListened++;
    long open = (long)(int32_t)(eNow(e) - (s->epochMs + s->d.slot[k].tMs));
    if (open > 0 && e->slotsLateOpen < UINT16_MAX) e->slotsLateOpen++;
    eLog(e, true, "supe: listen %02x%02x%02x slot %u t=%u (open %+ld)",
         s->d.hash3[0], s->d.hash3[1], s->d.hash3[2], (unsigned)k,
         (unsigned)s->d.slot[k].tMs, open);
    /* The hailer's held train and the hail's facts ride into the window. */
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
    m->weHailed = s->weHailed && !s->wide;
    m->fromHail = !s->wide;
    m->hailTxp = s->hailTxp;
    m->hailCeil = s->hailCeil;
    m->hailCount = s->hailCount;
    m->hailLen = s->hailLen;
    m->budget = s->wide ? 0 : s->hailCeil;
    /* A wide opener quotes how it read our last frame of the seeding meeting;
     * nothing is transmitted in this window, so that frame is still our last. */
    m->haveLastTx = s->wide && s->haveLastTx;
    m->lastTxp = s->lastTxp;
    m->lastTxCfg = s->lastTxCfg;
    m->txNext  = 0;                    /* reused as the window-extension count */
    m->deadlineMs = slotWindowCloseMs(e, s, k);
    s->nextSlot = (uint8_t)(k + 1);
    e->host->schedule(e->host->ctx, m->deadlineMs);
}

/* The IDLE tick: expire schedules, walk past missed slots, act on a due one. */
static void slotService(SupeEngine* e) {
    uint32_t now = eNow(e);
    /* A frame arriving right now outranks every slot (§7): only the two acts
     * that touch the radio defer; the bookkeeping runs on every tick. A
     * declined slot is walked past, not retried. */
    bool rxBusy = e->host->rx_busy && e->host->rx_busy(e->host->ctx);
    /* Narrow schedules first: a hail outranks a rendezvous (§7). Among the
     * narrow ones, the schedule of our own hail outranks one from a hail we
     * received: the answer we listen for there is the whole of our exchange,
     * and a READY that finds us away on another schedule's channel is a hail
     * gone unanswered — three of those and the peer holds us. A received
     * hail we walk past costs a hail-back. */
    for (int pass = 0; pass < 3; pass++)
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        SupeSched* s = &e->sched[i];
        if (!s->used) continue;
        int rank = s->wide ? 2 : (s->weHailed ? 0 : 1);
        if (rank != pass) continue;
        if (s->consumed) { schedFree(e, s); continue; }
        if ((int32_t)(now - (s->epochMs + schedHorizon(s) + 2 * SUPE_SLOT_GUARD_MS)) >= 0) {
            schedExpire(e, s, now);
            continue;
        }
        /* A wide schedule the far end is plainly not attending is dropped,
         * not spent (§7). */
        if (s->wide && s->nSpoke >= SUPE_SCHED_GIVEUP_SPOKE &&
            queueHasFor(e, s->haveTag ? s->tag : nullptr, s->peerId)) {
            eLog(e, false, "supe: sched %02x%02x%02x wide abandoned — %u "
                 "unanswered, hailing", s->d.hash3[0], s->d.hash3[1],
                 s->d.hash3[2], (unsigned)s->nSpoke);
            e->schedsAbandoned++;
            schedFree(e, s);
            continue;
        }
        while (s->nextSlot < s->d.nSlots) {
            uint8_t k = s->nextSlot;
            bool mine = slotIsMine(s, k);
            uint32_t at = s->epochMs + s->d.slot[k].tMs;
            if (mine) {
                if ((int32_t)(now - (at + SUPE_SLOT_LATE_MS)) > 0) {
                    if (s->nOwnDue < 255) s->nOwnDue++;
                    if (s->nLate < 255) s->nLate++;
                    eLog(e, true, "supe: own slot %u of %02x%02x%02x missed by %ld ms",
                         (unsigned)k, s->d.hash3[0], s->d.hash3[1], s->d.hash3[2],
                         (long)(now - at));
                    s->nextSlot++;
                    continue;
                }
                if ((int32_t)(now - at) >= 0) {
                    if (s->nOwnDue < 255) s->nOwnDue++;
                    if (rxBusy) { e->slotsYielded++; s->nextSlot++; continue; }
                    if (!s->wide) { slotSpeakNarrow(e, s, k); return; }
                    if (queueHasFor(e, s->haveTag ? s->tag : nullptr, s->peerId)) {
                        slotSpeakWide(e, s, k);
                        return;
                    }
                    if (s->nNoTraffic < 255) s->nNoTraffic++;
                    s->nextSlot++;
                    continue;
                }
            } else {
                if ((int32_t)(now - slotWindowCloseMs(e, s, k)) > 0) { s->nextSlot++; continue; }
                if ((int32_t)(now - (at - SUPE_SLOT_GUARD_MS)) >= 0) {
                    if (s->nOwnDue < 255) s->nOwnDue++;
                    if (rxBusy) { e->slotsYielded++; s->nLate++; s->nextSlot++; continue; }
                    slotListen(e, s, k);
                    return;
                }
            }
            break;
        }
    }
}

/* ─────────────── the trains ─────────────── */

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

static void sendEnd(SupeEngine* e);
static void sendClose(SupeEngine* e);
static void answerEnd(SupeEngine* e);
static void answerHail(SupeEngine* e);

/* What a TRAIN_WAIT expiry fires when it is not the next train frame. */
enum : uint8_t {
    SUPE_PEND_NONE = 0,
    SUPE_PEND_END,
    SUPE_PEND_ANSWER,
    SUPE_PEND_CLOSE,
    SUPE_PEND_HAIL_ANSWER,   /* regime 0: READY or GOT, one gap after the hail */
    SUPE_PEND_TRAIN,         /* the second train of a hailing-rate dialogue */
    SUPE_PEND_READY,         /* the READY answering a GOT at a slot */
};

/* Park a send. An ANSWER to the peer's frame waits the flip (SUPE_FLIP_MS):
 * the peer is only now turning from transmit back to receive, and a send
 * fired before that turn is done is never heard. A send that follows our own
 * frame — the END after our train — waits only the train gap: the peer's
 * receiver has been open throughout. */
static void deferSend(SupeEngine* e, uint8_t what, uint32_t gapMs) {
    e->m.pendSend = what;
    enterTrainWait(e, gapMs);
}
static void sendReady(SupeEngine* e);

/* The end of a hailing-rate train: nothing follows it (§8). Where the hailer's
 * train is still owed — we opened with GOT and the hail declared frames — it
 * comes next, a turnaround after ours; otherwise the dialogue is over. */
static void liteTrainSent(SupeEngine* e) {
    SupeMeet* m = &e->m;
    m->ourTrainConfirmed = true;          /* as confirmed as it will ever be */
    uint8_t theirs = 0;
    if (!m->weHailed && !m->listener) {
        theirs = m->hailCount < SUPE_TRAIN_MAX ? m->hailCount : SUPE_TRAIN_MAX;
    }
    if (theirs == 0) { finishMeeting(e, true, "lite"); return; }
    m->exCount = theirs;
    m->rxN = 0;
    m->anyRx = false;
    m->worstRssi = 127;
    m->phase = SUPE_M_TRAIN_RX;
    m->deadlineMs = eNow(e) + SUPE_TURNAROUND_MS + trainWorstMs(e, theirs, &m->cfg)
                    + SUPE_GUARD_MS;
    e->host->rx(e->host->ctx);
    e->host->schedule(e->host->ctx, m->deadlineMs);
}

/* The end of a hailing-rate train received: ours follows if we hold one
 * (the hailer, after the hailed party's), else the dialogue is over. */
static void liteTrainReceived(SupeEngine* e) {
    SupeMeet* m = &e->m;
    if (m->listener && m->txBuilt && m->tx.count > 0) {
        if (m->tx.count > m->peerCeil) m->tx.count = m->peerCeil;
        m->listener = false;
        m->leg = 1;
        m->trainTxp = m->hailTxp;         /* stated, and nothing will restate it */
        m->txNext = 0;
        m->txMaskAny = false;
        m->pendSend = SUPE_PEND_TRAIN;
        enterTrainWait(e, SUPE_TURNAROUND_MS);
        return;
    }
    finishMeeting(e, true, "lite");
}

static void fireNext(SupeEngine* e) {
    SupeMeet* m = &e->m;
    uint8_t idx = txNextIdx(m);
    if (idx == 0xFF) {
        if (m->lite) { liteTrainSent(e); return; }
        if (m->phase == SUPE_M_REPAIR_TX || m->txMaskAny) {
            m->txMaskAny = false;
            if (m->pendClose) { deferSend(e, SUPE_PEND_CLOSE, SUPE_TRAIN_GAP_MS); return; }
            m->phase = SUPE_M_AWAIT_ANSWER;
            m->deadlineMs = eNow(e) + answerDeadlineMs(e, &m->cfg);
            e->host->rx(e->host->ctx);
            e->host->schedule(e->host->ctx, m->deadlineMs);
            return;
        }
        deferSend(e, SUPE_PEND_END, SUPE_TRAIN_GAP_MS);
        return;
    }
    if (!e->host->train_fire(e->host->ctx, idx, m->trainTxp)) {
        finishMeeting(e, false, "train frame would not transmit");
        return;
    }
    noteOurTx(m, m->trainTxp, &m->cfg);
    m->txNext = (uint8_t)(idx + 1);
    if (m->txFired < 255) m->txFired++;
    enterTxPhase(m, m->txMaskAny ? SUPE_M_REPAIR_TX : SUPE_M_TRAIN_TX);
    if (m->txMaskAny) e->repairsOut++;
    else              e->framesOut++;
}

static void sendEnd(SupeEngine* e) {
    SupeMeet* m = &e->m;
    SupeEnd t = {};
    t.pwrDbm = m->trainTxp;
    t.salt   = (uint8_t)e->host->rand32(e->host->ctx);
    /* How the peer's last frame reached us — its READY or GOT where we hailed,
     * its hail where we answered one. This is the only measurement of the
     * direction the PEER transmits in that the peer will ever get from this
     * exchange, since the frames that quote a level back (READY, GOT) all
     * answer whoever opened the leg (§11). */
    t.haveHeard = m->haveLast;
    t.heardRssi = m->lastRssi;
    t.heardSnrQ = m->lastSnrQ;
    t.count  = m->tx.count;
    memcpy(t.csum, m->tx.csum, m->tx.count);
    size_t n = supeEncEnd(m->txEnd, sizeof m->txEnd, &t);
    if (!n || !e->host->tx_frame(e->host->ctx, m->txEnd, (uint16_t)n,
                                 m->trainTxp)) {
        finishMeeting(e, false, "END would not transmit");
        return;
    }
    noteOurTx(m, m->trainTxp, &m->cfg);
    m->txEndLen = (uint8_t)n;
    memcpy(m->lastEnd, m->txEnd, n);
    m->lastEndLen = (uint8_t)n;
    m->weReceivedFinal = false;
    enterTxPhase(m, SUPE_M_END_TX);
    eLog(e, true, "supe: END %u frames txp=%d", (unsigned)t.count,
         (int)m->trainTxp);
}

/* Start this side's train: resolve its power where it flies, lead in, fire.
 * At the hailing rate the power is the one already stated, since nothing
 * will restate it afterwards (§0.3). */
/* `leadMs` is what the first frame waits: the flip where the train answers
 * the peer's READY — the peer is turning from that transmit to receive, and
 * retuning to the budget on the way — and only the train lead where it
 * follows our own answering GOT, with the peer's receiver open throughout. */
static void startTrain(SupeEngine* e, uint32_t leadMs) {
    SupeMeet* m = &e->m;
    if (m->tx.count > m->peerCeil) m->tx.count = m->peerCeil;
    if (m->tx.count == 0) { finishMeeting(e, true, "nothing to send"); return; }
    m->trainTxp = m->lite ? (m->weHailed ? m->hailTxp : m->ourTxp)
                          : meetTxp(e, m->tag, m->haveTag, m->chan, &m->cfg);
    m->txNext = 0;
    m->txMaskAny = false;
    enterTrainWait(e, (uint32_t)SUPE_RETUNE_GAP_MS + leadMs);
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
    noteOurTx(m, m->ourTxp, &m->cfg);   /* the close rides the train's tuning */
    enterTxPhase(m, SUPE_M_CLOSE_TX);
}

/* Align the buffered arrivals against an END's checksum list: a greedy
 * leftmost ordered-subsequence match. Conservative on collisions — when in
 * doubt, ask for more (§8). */
static void alignTrain(SupeEngine* e, const SupeEnd* t) {
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

static void noteLast(SupeMeet* m, int16_t rssi, int16_t snr10) {
    m->lastRssi = rssi;
    m->lastSnrQ = supeEncSnrQ(snr10);
    m->haveLast = true;
}

/* A hail naming one of our addresses. Regime 0: answer it a turnaround later,
 * in place. Under a plan: derive its schedule and speak at the slots. Either
 * way a hail we cannot answer is a hail we owe (§12). */
static void onHail(SupeEngine* e, const uint8_t* f, uint16_t len,
                   int16_t rssi, int16_t snr10) {
    SupeMeet* m = &e->m;
    SupeHail h;
    if (!supeDecHail(f, len, &h)) { e->rxDiscard++; return; }
    SupeCfg hail = hailCfgOf(e);
    /* The hail states its power, so any listener's reading is a path loss
     * immediately — and hearing the node at all is presence. */
    if (h.haveIdent) {
        noteSimple(e, h.ident, SUPE_EV_ALIVE);
        notePair(e, h.ident, &hail, rssi, snr10, h.pwrDbm);
    }
    if (!supeEngTagIsOurs(e, h.tag)) { e->rxForeign++; return; }
    if (e->expired) { e->rxDiscard++; return; }
    uint32_t now = eNow(e);
    uint16_t peerId = LORAQ_PEER_NONE;
    if (h.haveIdent) {
        SupePeerView pv;
        if (e->host->peer_get(e->host->ctx, h.ident, &pv) && pv.known)
            peerId = pv.peerId;
        /* A hail from a peer naming us is an answer in the run's sense: it can
         * reach us, whatever became of our own hail (§12). */
        noteSimple(e, h.ident, SUPE_EV_ANSWERED);
    }
    uint8_t d0[32];
    e->host->sha256(e->host->ctx, f, len, d0);

    /* Crossed hails (§7): a hail from the peer our own hail is out toward.
     * Under a plan both schedules stay: we listen at ours and speak at
     * theirs, and the first contact consumes every narrow schedule between
     * the pair (finishMeeting). In regime 0 there is no schedule to keep:
     * theirs arriving while ours waits for an answer IS the answer, and ours
     * is dropped for it. */
    if (h.haveIdent && !e->plan && m->phase == SUPE_M_AWAIT_HAIL_ANSWER && m->haveTag &&
        (memcmp(m->tag, h.ident, SUPE_TAG_LEN) == 0 ||
         (peerId != LORAQ_PEER_NONE && m->peerId == peerId))) {
        eLog(e, true, "supe: crossed hail from %02x%02x%02x — answering theirs",
             h.ident[0], h.ident[1], h.ident[2]);
        releaseHeldTrain(e);
        memset(m, 0, sizeof *m);
        m->phase = SUPE_M_IDLE;
        m->schedIdx = -1;
    }

    if (!e->plan) {
        /* The dialogue: answer a turnaround later, unless the radio is spoken
         * for — then the hailer is owed a hail, sent when we are free. */
        if (m->phase != SUPE_M_IDLE) {
            if (h.haveIdent) owedAdd(e, h.ident, peerId, now);
            else e->rxForeign++;
            return;
        }
        releaseHeldTrain(e);
        memset(m, 0, sizeof *m);
        m->phase = SUPE_M_TRAIN_WAIT;
        m->schedIdx = -1;
        memcpy(m->hash3, d0, SUPE_HASH_LEN);
        if (h.haveIdent) { memcpy(m->tag, h.ident, SUPE_TAG_LEN); m->haveTag = true; }
        m->peerId = peerId;
        m->chan = SUPE_CH_HAIL;
        m->sByte = 0;
        m->slotCfg = hail;
        m->cfg = hail;
        m->hailTxp = h.pwrDbm;
        m->hailCount = h.count;
        m->hailLen = h.lenByte;
        m->hailCeil = h.budgetCeil;
        m->haveHail = true;
        m->hailRssi = rssi;
        m->hailSnrQ = supeEncSnrQ(snr10);
        noteLast(m, rssi, snr10);   /* the hail is what our END will report */
        m->beganMs = now;
        deferSend(e, SUPE_PEND_HAIL_ANSWER, SUPE_FLIP_MS);
        return;
    }

    /* A plan: a hail from the peer our own hail is out toward says the peer
     * did not hear ours — a node that had would be waiting at our slots, not
     * hailing — so ours is dropped for theirs, unmet and unscored, and we
     * speak at theirs with what we hold (§7). */
    if (h.haveIdent) {
        SupeSched* ours = schedFindLive(e, h.ident, peerId, true);
        if (ours && ours->weHailed) {
            eLog(e, true, "supe: crossed hail from %02x%02x%02x — theirs keeps",
                 h.ident[0], h.ident[1], h.ident[2]);
            if (m->phase == SUPE_M_SLOT_LISTEN && m->schedIdx == (int8_t)(ours - e->sched)) {
                if (m->retuned) e->host->tune_home(e->host->ctx);
                m->phase = SUPE_M_IDLE;
                m->retuned = false;
                m->deadlineMs = 0;
                m->schedIdx = -1;
            }
            if (m->phase == SUPE_M_IDLE) releaseHeldTrain(e);
            ours->consumed = true;
            schedFree(e, ours);
        }
    }
    /* The hail seeds two slots at which we speak. A window of ours already
     * open is walked out of — a hail outranks a rendezvous. */
    if (m->phase != SUPE_M_IDLE && m->phase != SUPE_M_SLOT_LISTEN) {
        if (h.haveIdent) owedAdd(e, h.ident, peerId, now);
        return;
    }
    SupeSched* s = schedInstall(e, f, len, /*wide=*/false, /*weHailed=*/false,
                                /*weTx0=*/false, h.haveIdent ? h.ident : nullptr,
                                peerId, &hail, now);
    if (s) {
        s->hailTxp = h.pwrDbm;
        s->hailCeil = h.budgetCeil;
        s->hailCount = h.count;
        s->hailLen = h.lenByte;
        s->haveHeard = true;
        s->heardRssi = rssi;
        s->heardSnrQ = supeEncSnrQ(snr10);
    }
    armTimer(e);
}

/* Regime 0: the deferred answer to a hail, on the hailing channel. */
static void answerHail(SupeEngine* e) {
    SupeMeet* m = &e->m;
    m->chan = SUPE_CH_HAIL;
    openAsHailed(e, nullptr);
}

/* Our hail's answer arrived, or a rendezvous opened toward us: the peer's
 * terms, with a train behind them (GOT) or not (READY). */
static bool answerIsOurs(SupeEngine* e, const uint8_t hash[SUPE_HASH_LEN]) {
    SupeMeet* m = &e->m;
    if (m->phase != SUPE_M_SLOT_LISTEN && m->phase != SUPE_M_AWAIT_HAIL_ANSWER) return false;
    return memcmp(hash, m->hash3, SUPE_HASH_LEN) == 0;
}

/* Our hail was answered, or a rendezvous was kept: the schedule is consumed,
 * the run ends, and the hail's reading — the one measurement of the direction
 * we transmit in at the hailing configuration — is filed. */
static void hailAnswered(SupeEngine* e, const SupeReady* g, int16_t rssi, int16_t snr10) {
    SupeMeet* m = &e->m;
    /* The train the hail described. Held since the hail where nothing else
     * needed the host's buffer; rebuilt here where something did — a slot
     * spoken for another peer, a third party's hail answered — and never
     * larger than the hail declared, since that is what the peer expects. */
    if (m->fromHail && m->weHailed && !m->txBuilt) {
        SupeTrainInfo ti;
        if (e->host->train_build(e->host->ctx, m->peerId,
                                 m->haveTag ? m->tag : nullptr, SUPE_TRAIN_MAX, &ti)) {
            m->tx = ti;
            m->txBuilt = true;
        } else {
            m->tx.count = 0;
        }
    }
    if (m->fromHail && m->weHailed && m->tx.count > m->hailCount)
        m->tx.count = m->hailCount;
    if (m->schedIdx >= 0) {
        SupeSched* s = &e->sched[m->schedIdx];
        s->consumed = true;
        schedFree(e, s);
        m->schedIdx = -1;
    }
    if (m->haveTag) {
        noteSimple(e, m->tag, SUPE_EV_ALIVE);
        noteSimple(e, m->tag, SUPE_EV_ANSWERED);
        notePair(e, m->tag, &m->slotCfg, rssi, snr10, g->pwrDbm);
        /* The reading is of a frame of ours, and is a path loss only against
         * that frame's power: the hail's, or — at a wide slot, where we have
         * sent nothing yet — our last of the seeding meeting. */
        if (m->fromHail) {
            SupeCfg hail = hailCfgOf(e);
            noteReport(e, m->tag, &hail, g->heardRssi, g->heardSnrQ, m->hailTxp);
        } else if (m->haveLastTx) {
            noteReport(e, m->tag, &m->lastTxCfg, g->heardRssi, g->heardSnrQ, m->lastTxp);
        }
    }
    if (m->fromHail) {
        m->haveHail = true;
        m->hailRssi = g->heardRssi;
        m->hailSnrQ = g->heardSnrQ;
    }
    m->ourTxp = meetTxp(e, m->tag, m->haveTag, m->chan, &m->slotCfg);
    m->worstRssi = 127;
    if (!m->beganMs) m->beganMs = eNow(e);
}

/* The READY answering a GOT, from what onGot settled: the budget it
 * confirmed and the reading of the GOT it confirmed it from. */
static void sendReady(SupeEngine* e) {
    SupeMeet* m = &e->m;
    SupeReady g = {};
    memcpy(g.hash, m->hash3, SUPE_HASH_LEN);
    g.pwrDbm = m->ourTxp;
    g.budget = m->budget;
    g.countCeil = SUPE_TRAIN_MAX;
    g.heardRssi = m->lastRssi;
    g.heardSnrQ = m->lastSnrQ;
    uint8_t out[SUPE_READY_LEN];
    size_t n = supeEncReady(out, sizeof out, &g);
    if (!n || !e->host->tx_frame(e->host->ctx, out, (uint16_t)n, m->ourTxp)) {
        finishMeeting(e, false, "READY would not transmit");
        return;
    }
    noteOurTx(m, m->ourTxp, &m->slotCfg);
    enterTxPhase(m, SUPE_M_READY_TX);
    eLog(e, true, "supe: READY budget %u txp=%d for %u frames%s",
         (unsigned)m->budget, (int)m->ourTxp, (unsigned)m->exCount,
         m->lite ? " (lite)" : "");
}

static void onGot(SupeEngine* e, const uint8_t* f, uint16_t len,
                   int16_t rssi, int16_t snr10) {
    SupeMeet* m = &e->m;

    if (m->phase == SUPE_M_SLOT_LISTEN || m->phase == SUPE_M_AWAIT_HAIL_ANSWER) {
        SupeGot hv;
        if (!supeDecGot(f, len, 0, &hv) || hv.answering) { e->rxDiscard++; return; }
        if (!answerIsOurs(e, hv.g.hash)) { e->rxForeign++; return; }
        noteLast(m, rssi, snr10);
        hailAnswered(e, &hv.g, rssi, snr10);

        /* The peer's train comes first; ours rides the answering turn. The
         * budget is ours to confirm: never above their proposal, never above
         * our own hail's ceiling where we hailed, from how this frame was
         * heard (§8). */
        uint8_t proposal = hv.g.budget;
        if (m->fromHail && proposal > m->hailCeil) proposal = m->hailCeil;
        SupeCfg cfg;
        uint8_t budget = chooseBudget(e, m->chan, proposal, &m->slotCfg, snr10,
                                      m->tag, m->haveTag, &cfg);
        m->listener = true;
        m->exCount = hv.count < SUPE_TRAIN_MAX ? hv.count : SUPE_TRAIN_MAX;
        m->peerCeil = hv.g.countCeil;
        m->peerTrainTxp = hv.g.pwrDbm;
        m->havePeerTxp = true;
        m->budget = budget;
        m->cfg = cfg;
        m->lite = !e->plan && budget == 0;
        m->leg = 0;
        /* The answer waits the flip: the peer has just transmitted the GOT
         * and is turning back to receive. */
        deferSend(e, SUPE_PEND_READY, SUPE_FLIP_MS);
        return;
    }

    if (m->phase == SUPE_M_AWAIT_ANSWER && m->leg == 0 && !m->listener) {
        /* The return leg: the answering GOT carries the train's reading and
         * any repair request; the return train follows (§8). It also PROMISES
         * a later END, which supersedes ours as the goodbye. */
        SupeGot hv;
        if (!supeDecGot(f, len, m->tx.count, &hv) || !hv.answering || hv.count == 0) {
            e->rxDiscard++;
            return;
        }
        if (memcmp(hv.g.hash, m->hash3, SUPE_HASH_LEN) != 0) { e->rxForeign++; return; }
        noteLast(m, rssi, snr10);
        m->laterEnd = true;
        m->ourTrainConfirmed = true;
        notePair(e, m->tag, &m->cfg, rssi, snr10, hv.g.pwrDbm);
        noteReport(e, m->tag, &m->cfg, hv.g.heardRssi, hv.g.heardSnrQ, m->trainTxp);
        m->ourTrainRssi = hv.g.heardRssi;
        m->ourTrainSnrQ = hv.g.heardSnrQ;
        m->haveOurRead = true;
        {
            SupePeerNote nt = {};
            nt.ev = SUPE_EV_TRAIN_OK;
            nt.cfg = m->cfg;
            nt.rssiDbm = hv.g.heardRssi;
            nt.snr10 = supeDecSnr10(hv.g.heardSnrQ);
            nt.txpDbm = m->trainTxp;
            nt.haveLevel = true;
            if (m->haveTag) e->host->peer_note(e->host->ctx, m->tag, &nt);
        }
        memcpy(m->txMask, hv.mask, sizeof m->txMask);
        m->txMaskAny = popcount8(hv.mask, hv.maskLen) > 0;
        m->leg = 1;
        m->exCount = hv.count < SUPE_TRAIN_MAX ? hv.count : SUPE_TRAIN_MAX;
        m->rxAligned = false;
        m->worstRssi = 127;
        m->phase = SUPE_M_TRAIN_RX;
        m->deadlineMs = eNow(e) + SUPE_TURNAROUND_MS
                        + trainWorstMs(e, m->exCount, &m->cfg) + SUPE_GUARD_MS;
        e->host->schedule(e->host->ctx, m->deadlineMs);
        return;
    }

    e->rxForeign++;
}

static void onReady(SupeEngine* e, const uint8_t* f, uint16_t len,
                    int16_t rssi, int16_t snr10) {
    SupeMeet* m = &e->m;
    SupeReady g;
    if (!supeDecReady(f, len, &g)) { e->rxDiscard++; return; }

    bool answerToHail = answerIsOurs(e, g.hash);
    bool confirmsOurGot = m->phase == SUPE_M_AWAIT_READY &&
                           memcmp(g.hash, m->hash3, SUPE_HASH_LEN) == 0;
    if (!answerToHail && !confirmsOurGot) { e->rxForeign++; return; }
    noteLast(m, rssi, snr10);

    if (answerToHail) {
        /* The hailed party holds nothing for us: our train follows. */
        hailAnswered(e, &g, rssi, snr10);
        m->listener = false;
    } else {
        /* Our GOT is confirmed: attention, budget, ceiling. The schedule it
         * opened is consumed. */
        if (m->schedIdx >= 0) {
            SupeSched* s = &e->sched[m->schedIdx];
            s->consumed = true;
            schedFree(e, s);
            m->schedIdx = -1;
        }
        if (m->haveTag) {
            noteSimple(e, m->tag, SUPE_EV_ALIVE);
            noteSimple(e, m->tag, SUPE_EV_ANSWERED);
            notePair(e, m->tag, &m->slotCfg, rssi, snr10, g.pwrDbm);
            noteReport(e, m->tag, &m->slotCfg, g.heardRssi, g.heardSnrQ, m->ourTxp);
        }
    }
    /* Stands in for the train's own reading until an answering GOT carries
     * one: same peer, same tuning, one frame earlier. */
    if (!m->haveOurRead) {
        m->ourTrainRssi = g.heardRssi;
        m->ourTrainSnrQ = g.heardSnrQ;
        m->haveOurRead = true;
    }

    /* The confirmed budget: at or below what we proposed, resolved against
     * the meeting's channel by both ends identically. */
    if (g.budget > m->budget) { finishMeeting(e, false, "budget above proposal"); return; }
    SupeCfg cfg;
    if (!supeResolveBudget(e->regime, SUPE_VERSION, e->hailSf, e->hailBwHz,
                           chanMaxBwOf(e, m->chan), e->ownFam,
                           peerFamOf(e, m->tag, m->haveTag), g.budget, &cfg)) {
        finishMeeting(e, false, "budget would not resolve");
        return;
    }
    m->budget = g.budget;
    m->cfg = cfg;
    m->peerCeil = g.countCeil;
    m->lite = !e->plan && g.budget == 0;
    if (!m->lite && !tuneToBudget(e)) { finishMeeting(e, false, "retune to budget failed"); return; }
    if (m->lite && m->cfg.sf != m->slotCfg.sf) m->lite = false;   /* cannot happen: budget 0 */
    /* The train's power is resolved where the train flies, on the report just
     * received (§15): the REPORT above reached the controller before this ask. */
    startTrain(e, SUPE_FLIP_MS);
}

static void onEnd(SupeEngine* e, const uint8_t* f, uint16_t len,
                      int16_t rssi, int16_t snr10) {
    SupeMeet* m = &e->m;
    SupeEnd t;
    if (!supeDecEnd(f, len, &t)) { e->rxDiscard++; return; }
    if (m->phase != SUPE_M_AWAIT_END && m->phase != SUPE_M_TRAIN_RX) {
        e->rxForeign++;
        return;
    }
    noteLast(m, rssi, snr10);
    if (t.count != m->exCount)
        eLog(e, true, "supe: END count %u against declared %u",
             (unsigned)t.count, (unsigned)m->exCount);
    if (m->haveTag && m->anyRx)
        notePair(e, m->tag, &m->cfg, m->worstRssi, supeDecSnr10(m->worstSnrQ), t.pwrDbm);
    /* The END's own reading: how OUR last frame reached the peer. It is the
     * answering side's only measurement of the direction it transmits in —
     * READY and GOT quote a level back to whoever opened the leg, and the
     * answerer opens nothing — so without this it would never learn it (§11).
     * Which frame of ours the peer heard it cannot say, so it resolves against
     * the last we transmitted: the frame it must have heard to be answering. */
    if (m->haveTag && t.haveHeard && m->haveLastTx)
        noteReport(e, m->tag, &m->lastTxCfg, t.heardRssi, t.heardSnrQ, m->lastTxp);
    m->peerTrainTxp = t.pwrDbm;
    m->havePeerTxp = true;
    if (len <= sizeof m->lastEnd) {
        memcpy(m->lastEnd, f, len);
        m->lastEndLen = (uint8_t)len;
        m->weReceivedFinal = true;
        m->laterEnd = false;
    }
    alignTrain(e, &t);
    deferSend(e, SUPE_PEND_ANSWER, SUPE_FLIP_MS);
}

/* The answer an END calls for, one train gap after it arrived. */
static void answerEnd(SupeEngine* e) {
    SupeMeet* m = &e->m;
    if (m->listener && m->leg == 0) {
        /* Our turn. Return traffic rides now — the hailer's held train, or one
         * built fresh — in the answering GOT, with the reading and the repair
         * request (§8). */
        bool haveReturn = m->txBuilt && m->tx.count > 0;
        if (!haveReturn && (m->peerId != LORAQ_PEER_NONE || m->haveTag)) {
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
            if (m->tx.count > m->peerCeil) m->tx.count = m->peerCeil;
            trimToCeiling(e, &m->tx, &m->cfg);
            m->leg = 1;
            SupeGot hv = {};
            memcpy(hv.g.hash, m->hash3, SUPE_HASH_LEN);
            hv.g.pwrDbm  = m->ourTxp;
            hv.g.budget  = m->budget;
            hv.g.countCeil = SUPE_TRAIN_MAX;
            hv.g.heardRssi = m->worstRssi;
            hv.g.heardSnrQ = m->worstSnrQ;
            hv.count   = m->tx.count;
            hv.lenByte = supeEncLen(trainLenMs(e, &m->tx, m->tx.count, &m->cfg));
            hv.answering = true;
            hv.maskLen = supeMaskLen(m->peerCount);
            memcpy(hv.mask, m->missMask, hv.maskLen);
            uint8_t out[SUPE_GOT_ANS_BASE + SUPE_MASK_MAX];
            size_t n = supeEncGot(out, sizeof out, &hv);
            if (!n || !e->host->tx_frame(e->host->ctx, out, (uint16_t)n, m->ourTxp)) {
                finishMeeting(e, false, "answering GOT would not transmit");
                return;
            }
            noteOurTx(m, m->ourTxp, &m->cfg);   /* it follows their train, at its tuning */
            m->repairExpect = m->missCount;     /* their repairs ride their close */
            enterTxPhase(m, SUPE_M_GOT_TX);
            return;
        }
        m->pendClose = m->missCount ? 2 : 1;
        m->repairExpect = m->missCount;
        sendClose(e);
        return;
    }

    /* The first sender, closing the return leg. */
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

static void trainConfirmed(SupeEngine* e) {
    SupeMeet* m = &e->m;
    m->ourTrainConfirmed = true;
    if (m->haveTag && !m->listener && m->leg == 0) {
        SupePeerNote nt = {};
        nt.ev = SUPE_EV_TRAIN_OK;
        nt.cfg = m->cfg;
        nt.txpDbm = m->trainTxp;
        nt.haveLevel = false;
        e->host->peer_note(e->host->ctx, m->tag, &nt);
    }
}

static void onBye(SupeEngine* e) {
    SupeMeet* m = &e->m;
    if (m->phase != SUPE_M_AWAIT_ANSWER && m->phase != SUPE_M_REPAIR_RX) {
        e->rxForeign++;
        return;
    }
    trainConfirmed(e);
    finishMeeting(e, true, "closed");
}

static void onResend(SupeEngine* e, const uint8_t* f, uint16_t len) {
    SupeMeet* m = &e->m;
    if (m->phase != SUPE_M_AWAIT_ANSWER || !m->txBuilt) { e->rxForeign++; return; }
    SupeResendF rs;
    if (!supeDecResend(f, len, m->tx.count, &rs)) { e->rxDiscard++; return; }
    trainConfirmed(e);
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
        case SUPE_T_HAIL:     onHail(e, f, len, rssi, snr10);    break;
        case SUPE_T_GOT:     onGot(e, f, len, rssi, snr10);    break;
        case SUPE_T_READY:    onReady(e, f, len, rssi, snr10);   break;
        case SUPE_T_END:  onEnd(e, f, len, rssi, snr10); break;
        case SUPE_T_BYE:
            if (len == SUPE_BYE_LEN) onBye(e);
            else e->rxDiscard++;
            break;
        case SUPE_T_RESEND:   onResend(e, f, len);               break;
        default:              e->rxDiscard++;                    break;
    }
}

bool supeEngOnTrainFrame(SupeEngine* e, uint8_t csum, int16_t rssi, int16_t snr10) {
    SupeMeet* m = &e->m;
    if (m->phase == SUPE_M_TRAIN_RX) {
        if (m->rxN >= SUPE_TRAIN_MAX) return false;
        m->anyRx = true;
        e->framesIn++;
        noteLast(m, rssi, snr10);
        if (rssi < m->worstRssi) { m->worstRssi = rssi; m->worstSnrQ = supeEncSnrQ(snr10); }
        if (m->lite) {
            /* A hailing-rate dialogue's frames are handed up as they land:
             * counted here, delivered by the host as ordinary traffic. */
            m->rxN++;
            if (m->rxN >= m->exCount) liteTrainReceived(e);
            return false;
        }
        m->rxCsum[m->rxN] = csum;
        m->rxPos[m->rxN] = m->rxN;     /* provisional until the END aligns */
        m->rxN++;
        if (m->rxN >= m->exCount) {
            m->phase = SUPE_M_AWAIT_END;
            m->deadlineMs = eNow(e) + SUPE_TURNAROUND_MS
                + toaFrameMs(e, &m->cfg, SUPE_END_BASE + m->exCount, false)
                + SUPE_GUARD_MS;
            e->host->schedule(e->host->ctx, m->deadlineMs);
        }
        return true;
    }
    if (m->phase == SUPE_M_REPAIR_RX) {
        if (m->rxN >= SUPE_TRAIN_MAX || m->repairGot >= m->repairExpect) return false;
        noteLast(m, rssi, snr10);
        m->rxCsum[m->rxN] = csum;
        m->rxPos[m->rxN] = m->repairPos[m->repairGot];
        m->rxN++;
        m->repairGot++;
        e->repairsIn++;
        if (m->repairGot >= m->repairExpect) {
            if (m->listener && m->leg == 1) {
                m->phase = SUPE_M_AWAIT_ANSWER;
                m->deadlineMs = eNow(e) + answerDeadlineMs(e, &m->cfg);
            } else {
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
        if (m->phase == SUPE_M_HAIL_TX) { finishMeeting(e, false, "hail aborted"); return; }
        finishMeeting(e, false, "transmit aborted");
        return;
    }
    switch (m->phase) {
        case SUPE_M_HAIL_TX: {
            /* The hail is out; its end is the epoch both radios just timed. */
            SupeCfg hail = hailCfgOf(e);
            if (!e->plan) {
                /* Regime 0: the answer is owed a turnaround later, here. The
                 * hailed party parks it one train gap, so the deadline
                 * carries that too. */
                m->phase = SUPE_M_AWAIT_HAIL_ANSWER;
                m->slotCfg = hail;
                m->cfg = hail;
                m->chan = SUPE_CH_HAIL;
                m->deadlineMs = eNow(e) + SUPE_TRAIN_GAP_MS + SUPE_TURNAROUND_MS
                                + toaFrameMs(e, &hail, SUPE_GOT_LEN, false) + SUPE_GUARD_MS;
                e->host->rx(e->host->ctx);
                e->host->schedule(e->host->ctx, m->deadlineMs);
                return;
            }
            /* A plan: two slots at which the hailed party speaks and we
             * listen. The held train and the hail's facts stay in the meeting
             * record across the idle stretch. */
            SupeSched* s = schedInstall(e, m->hailFrame, m->hailFrameLen, /*wide=*/false,
                                        /*weHailed=*/true, /*weTx0=*/false,
                                        m->haveTag ? m->tag : nullptr, m->peerId,
                                        &hail, eNow(e));
            if (s) {
                s->hailTxp = m->hailTxp;
                s->hailCeil = m->hailCeil;
                s->hailCount = m->hailCount;
                s->hailLen = m->hailLen;
            }
            m->phase = SUPE_M_IDLE;
            m->deadlineMs = 0;
            e->host->rx(e->host->ctx);
            armTimer(e);
            return;
        }
        case SUPE_M_GOT_TX:
            if (m->listener && m->leg == 1) {
                /* The answering GOT is out; the return train follows. */
                startTrain(e, SUPE_TRAIN_LEAD_MS);
                return;
            }
            m->phase = SUPE_M_AWAIT_READY;
            m->deadlineMs = eNow(e) + SUPE_TURNAROUND_MS
                + toaFrameMs(e, &m->slotCfg, SUPE_READY_LEN, false) + SUPE_GUARD_MS;
            e->host->rx(e->host->ctx);
            e->host->schedule(e->host->ctx, m->deadlineMs);
            return;
        case SUPE_M_READY_TX: {
            if (m->exCount == 0) {
                /* Nothing to receive: a hail-back met with nothing waiting,
                 * or traffic that expired. Two short frames, and home. */
                finishMeeting(e, true, "nothing to receive");
                return;
            }
            if (!m->lite && !tuneToBudget(e)) {
                finishMeeting(e, false, "retune to budget failed");
                return;
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
             * deadline: the flip interval is paid, not parked (§14.7). */
            fireNext(e);
            return;
        case SUPE_M_END_TX:
            m->phase = SUPE_M_AWAIT_ANSWER;
            m->deadlineMs = eNow(e) + answerDeadlineMs(e, &m->cfg);
            e->host->rx(e->host->ctx);
            e->host->schedule(e->host->ctx, m->deadlineMs);
            return;
        case SUPE_M_CLOSE_TX:
            if (m->closeIsFinal) { finishMeeting(e, true, "closed"); return; }
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
            if (e->host->rx_busy && e->host->rx_busy(e->host->ctx) &&
                m->txNext < SUPE_WINDOW_EXTENDS_MAX) {
                m->txNext++;
                m->deadlineMs = now
                    + toaFrameMs(e, &m->slotCfg, SUPE_GOT_LEN, false)
                    + SUPE_GUARD_MS;
                e->host->schedule(e->host->ctx, m->deadlineMs);
                return;
            }
            /* Silence: home, and the schedule's other slot or its expiry
             * decides what it meant. The held train stays held. */
            if (m->retuned) e->host->tune_home(e->host->ctx);
            m->phase = SUPE_M_IDLE;
            m->retuned = false;
            m->deadlineMs = 0;
            m->schedIdx = -1;
            e->host->rx(e->host->ctx);
            slotService(e);
            armTimer(e);
            return;
        }
        case SUPE_M_AWAIT_HAIL_ANSWER:
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            /* Unanswered: the interval opens, in which the hailed party may
             * hail back; the traffic keeps its queue place (§12). */
            noteUnanswered(e, m->haveTag ? m->tag : (const uint8_t*)"\0\0\0", m->hailTxp);
            eLog(e, false, "supe: hail %02x%02x%02x unanswered", m->tag[0], m->tag[1], m->tag[2]);
            finishMeeting(e, false, "no answer");
            return;
        case SUPE_M_TRAIN_WAIT:
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            switch (m->pendSend) {
                case SUPE_PEND_END:     m->pendSend = 0; sendEnd(e);   return;
                case SUPE_PEND_ANSWER:      m->pendSend = 0; answerEnd(e); return;
                case SUPE_PEND_CLOSE:       m->pendSend = 0; sendClose(e);     return;
                case SUPE_PEND_HAIL_ANSWER: m->pendSend = 0; answerHail(e);    return;
                case SUPE_PEND_TRAIN:       m->pendSend = 0; fireNext(e);      return;
                case SUPE_PEND_READY:       m->pendSend = 0; sendReady(e);     return;
                default:                    fireNext(e);                       return;
            }
        case SUPE_M_AWAIT_READY:
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            finishMeeting(e, false, "no READY");
            return;
        case SUPE_M_TRAIN_RX:
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            if (m->lite) {
                /* What came, came: a hailing-rate train has no repair. */
                if (m->anyRx) liteTrainReceived(e);
                else          finishMeeting(e, false, "no train");
                return;
            }
            m->phase = SUPE_M_AWAIT_END;
            m->deadlineMs = now + SUPE_TURNAROUND_MS
                + toaFrameMs(e, &m->cfg, SUPE_END_BASE + m->exCount, false)
                + SUPE_GUARD_MS;
            e->host->schedule(e->host->ctx, m->deadlineMs);
            return;
        case SUPE_M_AWAIT_END:
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            finishMeeting(e, false, "no END");
            return;
        case SUPE_M_AWAIT_ANSWER: {
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            if (!m->listener && m->leg == 0 && !m->ourTrainConfirmed) {
                SupePeerNote nt = {};
                nt.ev = SUPE_EV_TRAIN_LOST;
                nt.cfg = m->cfg;
                nt.triedTxpDbm = m->trainTxp;
                if (m->haveTag) e->host->peer_note(e->host->ctx, m->tag, &nt);
                finishMeeting(e, false, "no answer");
                return;
            }
            finishMeeting(e, m->ourTrainConfirmed || m->listener, "gave up");
            return;
        }
        case SUPE_M_REPAIR_RX:
            if ((int32_t)(now - m->deadlineMs) < 0) { armTimer(e); return; }
            if (m->listener && m->leg == 1) { finishMeeting(e, true, "gave up"); return; }
            m->pendClose = 1;
            sendClose(e);
            return;
        default:
            if (m->deadlineMs && (int32_t)(now - m->deadlineMs) >= 0)
                m->deadlineMs = 0;
            armTimer(e);
            return;
    }
}

size_t supeEngBuildAnn(SupeEngine* e, uint8_t* out, size_t cap,
                       const uint8_t ids[][SUPE_ID_LEN], uint8_t count,
                       int8_t pwrDbm, bool speaking) {
    SupeAnn a = {};
    /* A node that has SUPE turned off still announces — that is the only way to
     * tell a neighbour holding the opposite belief to stop, and going quiet
     * cannot say it. */
    a.regime = speaking ? e->regime : SUPE_REGIME_NONE;
    a.version = SUPE_VERSION;
    a.caps.fam = e->ownFam;
    a.caps.topStep = e->ownTop;
    a.caps.maxPwrDbm = e->txpMax;
    a.pwrDbm = pwrDbm;
    a.count = count > SUPE_ANN_MAX ? SUPE_ANN_MAX : count;
    for (int i = 0; i < a.count; i++) memcpy(a.ids[i], ids[i], SUPE_ID_LEN);
    return supeEncAnn(out, cap, &a);
}
