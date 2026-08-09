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

/* One train's packet cap. The queue holds the packets; this bounds how many
 * of them one MANIFEST describes. */
#define SUPE_TRAIN_MAX          8

/* The pre-offer jitter, so two requesters that collide once do not collide
 * again in lockstep (SUPE.md §11). */
#define SUPE_OFFER_JITTER_MS   24

/* The ladder's randomised wait between requests — long enough to outlast
 * somebody else's detour, which is the likeliest cause of the silence. */
#define SUPE_RETRY_WAIT_MIN_MS 400
#define SUPE_RETRY_WAIT_SPAN_MS 400

/* What the requester waits after its retune before speaking, beyond the
 * synthesizer's own gap. The answerer's TxDone → tune → receive path is an
 * ISR-notified preemptive wake plus a handful of SPI transactions — well
 * under a millisecond when nothing else has the CPU — so this covers ordinary
 * scheduling jitter only. It deliberately does NOT cover a flash-erase cache
 * stall at the far end: no affordable lead could, and that is what the
 * deadline is for — the rare stalled transaction fails cheaply and retries.
 * Provisional, pending the measurement plans/SUPE.md §16 asks simulation
 * for. */
#define SUPE_MAN_LEAD_MS        3

/* The receiver-flip gap before and between train packets: RX_DONE must be
 * serviced, the frame read out and dispatched, and receive re-armed before
 * the next preamble flies — a packet arriving inside that window is simply
 * not heard. The turnaround is sub-millisecond and the fastest budget's
 * preamble is ~0.8 ms, so two milliseconds is slack, not padding. Counted
 * into every stated train length, so the far end's deadline covers it. */
#define SUPE_TRAIN_GAP_MS       2

/* Receiver-side slack per expected train packet, added to the stated
 * length's deadline. The sender's length budgets airtime + the flip gap,
 * but each real gap also carries IRQ, task and timer latency it cannot
 * state, and on a long train at a fast budget that drift outgrows the
 * fixed guard — a deadline firing mid-train walks off while the last
 * packet is still on the air. Count completion ends reception the moment
 * the train is whole; this deadline is only the backstop for loss, so
 * generous costs nothing when things go right. Receiver-local: nothing
 * on the wire derives from it. */
#define SUPE_TRAIN_RX_SLACK_MS  20

/* What a refusal reason is worth in backoff. The reason set is this
 * implementation's (supe.h); the durations are the only thing a requester
 * does with one. */
static uint32_t refusalBackoffMs(uint8_t reason) {
    switch (reason) {
        case SUPE_REFUSE_BUSY:     return 300;
        case SUPE_REFUSE_NO_QUIET: return 2000;
        case SUPE_REFUSE_AIRTIME:  return 5u * 60u * 1000u;
        case SUPE_REFUSE_REGIME:   return 60u * 60u * 1000u;
        case SUPE_REFUSE_CEILING:  return 60u * 60u * 1000u;
        default:                   return 2000;
    }
}

/* ─────────────── small helpers ─────────────── */

static uint32_t eNow(SupeEngine* e) { return e->host->now_ms(e->host->ctx); }

static bool aStageNext(SupeEngine* e);
static bool bStageNext(SupeEngine* e);

static void eLog(SupeEngine* e, bool dbg, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));
static void eLog(SupeEngine* e, bool dbg, const char* fmt, ...) {
    if (!e->host->log || (dbg && !e->host->dbgLevel)) return;
    char b[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    e->host->log(e->host->ctx, b);
}

static SupeCfg hailCfgOf(const SupeEngine* e) {
    SupeCfg c;
    c.sf = e->hailSf;
    c.bwHz = e->hailBwHz;
    c.ldro = (1000u << e->hailSf) > 16u * e->hailBwHz;
    c.marginDeci = 0;
    return c;
}

/* The maximum bandwidth the named channel permits — what the budget resolves
 * against. Regime 0 has only channel 0 at the hailing width. */
static uint32_t chanMaxBwOf(const SupeEngine* e, uint8_t regime, uint8_t chan) {
    if (regime == SUPE_REGIME_SINGLE || chan == SUPE_CH_HAIL) return e->hailBwHz;
    int n = 0;
    const SupeChan* c = supeRegimeChans(regime, &n);
    if (!c || chan > n) return 0;
    return c[chan - 1].bwHz;
}

/* The most this node may transmit at on a granted channel: the configured
 * power, capped by the regime's regulatory limit off the hailing channel. */
static int8_t chanTxpCapOf(const SupeEngine* e, uint8_t regime, uint8_t chan) {
    if (regime == SUPE_REGIME_SINGLE || chan == SUPE_CH_HAIL) return e->txpMax;
    const SupeRegime* g = supeRegime(regime);
    if (!g || g->maxTxpDbm == SUPE_TXP_IFACE) return e->txpMax;
    return e->txpMax < g->maxTxpDbm ? e->txpMax : g->maxTxpDbm;
}

static uint32_t toaFrameMs(const SupeEngine* e, const SupeCfg* c, int payload) {
    double s = supeAirtimeSeconds(c->sf, (int)c->bwHz, e->crDenom, e->preamble,
                                  payload, false, false);
    return (uint32_t)(s * 1000.0 + 0.999);
}

/* Time on air of one queued Reticulum packet at a configuration, in the
 * interface's framing: CRC on, split into maxFrameLen-byte frames each with
 * its own header byte, preamble and checksum. */
static uint32_t pktAirMs(const SupeEngine* e, const SupeCfg* c, uint16_t len) {
    uint32_t ms = 0;
    uint16_t left = len;
    do {
        uint16_t part = left > e->maxFrameLen ? e->maxFrameLen : left;
        double s = supeAirtimeSeconds(c->sf, (int)c->bwHz, e->crDenom, e->preamble,
                                      (int)part + 1, false, true);
        ms += (uint32_t)(s * 1000.0 + 0.999);
        left = (uint16_t)(left - part);
    } while (left);
    return ms;
}

/* The contiguous run of head packets addressed to one tag — what a START's
 * load describes and what the requester's train carries. */
static uint8_t headRun(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN],
                       uint32_t* adjustedBytes) {
    uint8_t n = 0;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < loraqDepth(e->q); i++) {
        LoraPkt* p = loraqAt(e->q, i);
        if (!(p->flags & LORAQ_F_HAVE_TAG)) break;
        if (memcmp(p->tag, tag, SUPE_TAG_LEN) != 0) break;
        sum += (uint32_t)p->len + SUPE_LOAD_PKT_OVERHEAD;
        n++;
    }
    if (adjustedBytes) *adjustedBytes = sum;
    return n;
}

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

/* ─────────────── the hold (virtual carrier sense, §6) ─────────────── */

static void holdAdd(SupeEngine* e, const uint8_t* tag, uint32_t ms) {
    uint32_t now = eNow(e);
    SupeHold* slot = nullptr;
    for (int i = 0; i < SUPE_HOLD_MAX; i++) {
        SupeHold* h = &e->hold[i];
        if (h->used && memcmp(h->tag, tag, SUPE_TAG_LEN) == 0) { slot = h; break; }
        if (!h->used && !slot) slot = h;
    }
    if (!slot) {                                    /* full: displace the soonest */
        slot = &e->hold[0];
        for (int i = 1; i < SUPE_HOLD_MAX; i++)
            if ((int32_t)(e->hold[i].untilMs - slot->untilMs) < 0) slot = &e->hold[i];
    }
    memcpy(slot->tag, tag, SUPE_TAG_LEN);
    slot->used    = true;
    slot->untilMs = now + ms;
    e->holdsTaken++;
    eLog(e, true, "supe: hold %02x%02x%02x for %u ms", tag[0], tag[1], tag[2],
         (unsigned)ms);
}

/* `earlyMs` is the fixed idle interval the contention machine opens with: a
 * hold consulted by the transmit path releases that much early, so the DIFS
 * runs out exactly as the pair is due back and only the random draw remains.
 * The fixed part moves, never the random part (SUPE.md §6). */
static bool heldWithin(SupeEngine* e, const uint8_t* tag, uint32_t earlyMs) {
    uint32_t now = eNow(e);
    for (int i = 0; i < SUPE_HOLD_MAX; i++) {
        SupeHold* h = &e->hold[i];
        if (!h->used) continue;
        if ((int32_t)(now - h->untilMs) >= 0) { h->used = false; continue; }
        if (memcmp(h->tag, tag, SUPE_TAG_LEN) != 0) continue;
        if (h->untilMs - now <= earlyMs) return false;   /* the tail is idle DIFS */
        return true;
    }
    return false;
}

bool supeEngHeld(SupeEngine* e, const uint8_t* tag) {
    return heldWithin(e, tag, 0);
}

/* ─────────────── overheard STARTs, for GRANT correlation ─────────────── */

static void seenAdd(SupeEngine* e, const uint8_t hash[SUPE_HASH_LEN],
                    const SupeStart2* s) {
    uint32_t now = eNow(e);
    SupeStartSeen* slot = &e->seen[0];
    for (int i = 0; i < SUPE_STARTS_SEEN_MAX; i++) {
        SupeStartSeen* c = &e->seen[i];
        if (!c->used) { slot = c; break; }
        if ((int32_t)(c->ms - slot->ms) < 0) slot = c;
    }
    memcpy(slot->hash, hash, SUPE_HASH_LEN);
    memcpy(slot->tag, s->tag, SUPE_TAG_LEN);
    slot->haveIdent = s->haveIdent;
    if (s->haveIdent) memcpy(slot->ident, s->ident, SUPE_TAG_LEN);
    slot->ms = now;
    slot->used = true;
}

static SupeStartSeen* seenFind(SupeEngine* e, const uint8_t hash[SUPE_HASH_LEN]) {
    uint32_t now = eNow(e);
    for (int i = 0; i < SUPE_STARTS_SEEN_MAX; i++) {
        SupeStartSeen* c = &e->seen[i];
        if (!c->used) continue;
        if (now - c->ms > 10000) { c->used = false; continue; }
        if (memcmp(c->hash, hash, SUPE_HASH_LEN) == 0) return c;
    }
    return nullptr;
}

/* ─────────────── notes out ─────────────── */

static void noteSimple(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN], uint8_t ev) {
    SupePeerNote n = {};
    n.ev = ev;
    e->host->peer_note(e->host->ctx, tag, &n);
}

static void notePair(SupeEngine* e, const uint8_t tag[SUPE_TAG_LEN],
                     const SupeCfg* cfg, int16_t rssi, int8_t txp) {
    SupePeerNote n = {};
    n.ev = SUPE_EV_PAIR;
    n.cfg = *cfg;
    n.rssiDbm = rssi;
    n.txpDbm = txp;
    e->host->peer_note(e->host->ctx, tag, &n);
}

/* ─────────────── lifecycle ─────────────── */

void supeEngInit(SupeEngine* e, const SupeHost* host, LoraQueue* q) {
    memset(e, 0, sizeof *e);
    e->host = host;
    e->q = q;
}

void supeEngConfig(SupeEngine* e, uint8_t regime, uint8_t ownFam, uint8_t ownTop,
                   int8_t txpMax, bool adaptive, uint8_t hailSf, uint32_t hailBwHz,
                   uint8_t crDenom, uint16_t preamble, uint8_t ifaceSync,
                   uint16_t maxFrameLen) {
    e->regime = regime;
    e->ownFam = ownFam;
    e->ownTop = ownTop > 14 ? 14 : ownTop;
    e->txpMax = txpMax;
    e->adaptive = adaptive;
    e->hailSf = hailSf;
    e->hailBwHz = hailBwHz;
    e->crDenom = crDenom;
    e->preamble = preamble;
    e->ifaceSync = ifaceSync;
    e->maxFrameLen = maxFrameLen;
}

void supeEngReset(SupeEngine* e) {
    /* The radio went away underneath; no RF restore is owed — the whole modem
     * regime is re-applied on the way up. The learned stores survive. This is
     * the one exit that does not go through home(), so it says so — a
     * transaction that vanishes without a line is undebuggable. */
    if (e->x.phase != SUPE_X_IDLE)
        eLog(e, true, "supe: transaction reset (radio went down) in phase %u",
             (unsigned)e->x.phase);
    memset(&e->x, 0, sizeof e->x);
    e->x.phase = SUPE_X_IDLE;
    e->offerArmed = false;
}

bool supeEngBusy(const SupeEngine* e) {
    return e->x.phase != SUPE_X_IDLE || e->offerArmed;
}

static void home(SupeEngine* e, bool success, const char* why);

void supeEngAbort(SupeEngine* e, const char* why) {
    e->offerArmed = false;
    home(e, false, why);
}

/* ─────────────── going home ─────────────── */

static void home(SupeEngine* e, bool success, const char* why) {
    SupeXact* x = &e->x;
    bool     detoured = x->retuned;
    uint8_t  chan     = x->chan;
    uint8_t  sf       = x->cfg.sf;
    uint32_t bwHz     = x->cfg.bwHz;
    uint8_t  sent     = x->sentCount;
    uint8_t  plan     = x->planCount;
    uint8_t  got      = x->gotCount;
    uint8_t  expect   = x->expectCount;
    if (x->retuned) {
        e->host->tune_home(e->host->ctx);
        x->retuned = false;
    }
    uint8_t was = x->phase;
    memset(x, 0, sizeof *x);
    x->phase = SUPE_X_IDLE;
    e->host->rx(e->host->ctx);
    /* The end record, whatever the log does with the line below. */
    if (was != SUPE_X_IDLE) {
        SupeEngine::SupeEndRec* er = &e->ends[e->endsAt];
        e->endsAt = (uint8_t)((e->endsAt + 1) % (sizeof e->ends / sizeof e->ends[0]));
        er->why = why;         er->endedMs = eNow(e);
        er->phase = was;       er->chan = chan;
        er->ok = success;      er->role_b = (was >= SUPE_X_B_GRANT_TX);
        er->sent = sent;       er->plan = plan;
        er->got = got;         er->expect = expect;
    }
    /* The one line a detour leaves behind: where it ran and what moved. */
    if (was != SUPE_X_IDLE) {
        if (detoured)
            eLog(e, true, "supe: detour ch%u sf%u %luk: sent %u, rcvd %u/%u (%s)",
                 chan, sf, (unsigned long)(bwHz / 1000u),
                 sent, got, expect, why);
        else
            eLog(e, true, "supe: transaction ended (%s)", why);
    }
    (void)success;
}

/* ─────────────── the requester ─────────────── */

int shouldDetour(const SupePeerView* peer, const LoraQueue* q,
                 const SupeChanView* chans, uint32_t now,
                 uint32_t* wait_until_ms) {
    /* The one deliberately-open decision (SUPE.md §18): inputs are the peer,
     * the queue and the channels; the answer is a policy question for
     * plans/simulation.md §7. v0: a peer to detour with is a detour worth
     * asking for — even §8's worst row costs one frame pair against the
     * reverse direction it buys. */
    (void)peer; (void)q; (void)chans; (void)now; (void)wait_until_ms;
    return DETOUR_NOW;
}

uint8_t supeEngVerdict(SupeEngine* e) {
    if (e->expired) { e->offerArmed = false; return SUPE_V_PLAIN; }
    LoraPkt* p = loraqAt(e->q, 0);
    if (!p) { e->offerArmed = false; return SUPE_V_PLAIN; }
    if (!(p->flags & LORAQ_F_HAVE_TAG)) { e->offerArmed = false; return SUPE_V_PLAIN; }

    /* The hold outranks everything below it: a tag announced busy is busy
     * whatever else is known about it. Released one DIFS early, so the polite
     * node is not also the last one in the draw. */
    if (heldWithin(e, p->tag, e->holdEarlyMs)) {
        e->offerArmed = false;
        return SUPE_V_HOLD;
    }

    SupePeerView pv;
    if (!e->host->peer_get(e->host->ctx, p->tag, &pv) || !pv.known) {
        /* Not a SUPE peer: untouched, exactly as with the feature off. */
        e->offerArmed = false;
        return SUPE_V_PLAIN;
    }
    uint32_t now = eNow(e);
    if (pv.absentUntilMs && (int32_t)(pv.absentUntilMs - now) > 0) {
        e->offerArmed = false;
        e->dropsAbsent++;
        return SUPE_V_DROP;
    }
    /* A queue of nothing but proofs bides its time. The proof of a packet can
     * never ride the detour that carried the packet — it exists only after
     * that train has landed — but it can ride the NEXT transaction in either
     * direction: as reverse cargo when this peer opens one to us, or alongside
     * whatever data joins the queue behind it, which releases the hold at
     * once. So it neither launches a detour of its own nor flies plainly
     * until it has aged out. The cost is the far end's send window, which the
     * held proof is keeping shut — the age cap is what bounds that. */
    if (e->holdProofMs && (p->flags & LORAQ_F_PROOF) &&
        (uint32_t)(now - p->first_seen_ms) < e->holdProofMs) {
        bool allProof = true;
        for (int i = 1; allProof; i++) {
            LoraPkt* qp = loraqAt(e->q, i);
            if (!qp) break;
            if (!(qp->flags & LORAQ_F_PROOF)) allProof = false;
        }
        if (allProof) {
            e->offerArmed = false;
            return SUPE_V_HOLD;
        }
    }
    if (pv.retryWaitUntilMs && (int32_t)(pv.retryWaitUntilMs - now) > 0) {
        /* Mid-ladder: the randomised wait between requests. The packet holds
         * through it — flying plainly here would defeat the ladder. */
        e->offerArmed = false;
        return SUPE_V_HOLD;
    }
    if (pv.backoffUntilMs && (int32_t)(pv.backoffUntilMs - now) > 0) {
        /* A refusal said how long not to ask. The peer is present; the
         * traffic flies plainly meanwhile. */
        e->offerArmed = false;
        return SUPE_V_PLAIN;
    }
    if (e->plainOnce) {
        e->plainOnce = false;
        e->offerArmed = false;
        return SUPE_V_PLAIN;
    }

    SupeChanView cv;
    e->host->chan_get(e->host->ctx, &cv);
    uint32_t waitUntil = 0;
    int d = shouldDetour(&pv, e->q, &cv, now, &waitUntil);
    if (d == DETOUR_NO)  { e->offerArmed = false; return SUPE_V_PLAIN; }
    if (d == DETOUR_WAIT) { e->offerArmed = false; return SUPE_V_HOLD; }

    if (!e->offerArmed) {
        e->offerArmed = true;
        e->offerJitterUntilMs = now
            + (e->host->rand32(e->host->ctx) % SUPE_OFFER_JITTER_MS);
    }
    return SUPE_V_OFFER;
}

bool supeEngLaunchDue(const SupeEngine* e) {
    if (!e->offerArmed || e->x.phase != SUPE_X_IDLE) return false;
    uint32_t now = e->host->now_ms(e->host->ctx);
    return (int32_t)(now - e->offerJitterUntilMs) >= 0;
}

void supeEngLaunch(SupeEngine* e) {
    SupeXact* x = &e->x;
    e->offerArmed = false;
    LoraPkt* p = loraqAt(e->q, 0);
    if (!p || !(p->flags & LORAQ_F_HAVE_TAG) || x->phase != SUPE_X_IDLE) return;
    SupePeerView pv;
    if (!e->host->peer_get(e->host->ctx, p->tag, &pv) || !pv.known) return;

    /* The absence ladder's rung decides the power and the ceiling (§11):
     * power up, ceiling down, the third rung at maximum and budget 0. */
    uint8_t rung = pv.absentStrikes > 2 ? 2 : pv.absentStrikes;
    uint8_t ceiling = e->ownTop < pv.topBudget ? e->ownTop : pv.topBudget;
    int8_t  txp = e->adaptive ? pv.txpOpen : e->txpMax;
    if (rung == 1) {
        ceiling = (uint8_t)(ceiling / 2);
        txp = (int8_t)((txp + e->txpMax + 1) / 2);
    } else if (rung >= 2) {
        ceiling = 0;
        txp = e->txpMax;
    }
    if (txp > e->txpMax) txp = e->txpMax;

    uint32_t adjusted = 0;
    headRun(e, p->tag, &adjusted);

    SupeStart2 s = {};
    s.regime = e->regime;
    s.version = SUPE_VERSION;
    memcpy(s.tag, p->tag, SUPE_TAG_LEN);
    s.fam = e->ownFam;
    s.ceiling = ceiling;
    s.load = supeEncLoad(adjusted);
    /* sender_ident ships with the transmitting form and not before (§4). */
    uint8_t f[SUPE_START2_ID_LEN];
    size_t n = supeEncStart2(f, sizeof f, &s);
    if (!n) return;
    uint8_t sha[32];
    e->host->sha256(e->host->ctx, f, (uint16_t)n, sha);

    memset(x, 0, sizeof *x);
    x->phase    = SUPE_X_A_START_TX;
    x->role_b   = 0;
    x->regime   = e->regime;           /* what the START named; a GRANT may
                                        * answer with it or with regime 0 */
    memcpy(x->tag, p->tag, SUPE_TAG_LEN);
    memcpy(x->hash, sha, SUPE_HASH_LEN);
    x->peerId   = pv.peerId;
    x->peerFam  = pv.fam;
    x->attempt  = (uint8_t)(rung + 1);
    x->startTxp = txp;
    x->beganMs  = eNow(e);
    if (!e->host->tx_frame(e->host->ctx, f, (uint16_t)n, txp)) {
        home(e, false, "START would not transmit");
        return;
    }
    e->startsOut++;
    eLog(e, true, "supe: START %02x%02x%02x load=%u ceiling=%u txp=%d (rung %u)",
         s.tag[0], s.tag[1], s.tag[2], (unsigned)s.load, (unsigned)ceiling,
         (int)txp, (unsigned)rung + 1);
}

/* Plan the requester's train: the contiguous same-tag run, capped by the
 * regime's train ceiling and by what MANIFEST's length byte can state. */
static void planTrain(SupeEngine* e, uint8_t maxCount, uint32_t ceilMs,
                      uint8_t* countOut, uint32_t* msOut) {
    uint8_t count = 0;
    uint32_t ms = 0;
    uint8_t run = headRun(e, e->x.tag, nullptr);
    if (run > maxCount) run = maxCount;
    for (uint8_t i = 0; i < run; i++) {
        uint32_t add = pktAirMs(e, &e->x.cfg, loraqAt(e->q, i)->len)
                       + SUPE_TRAIN_GAP_MS;
        if (count > 0 && ms + add > ceilMs) break;
        ms += add;
        count++;
    }
    if (count == 1 && ms > ceilMs) count = 0;   /* one packet that cannot fit */
    *countOut = count;
    *msOut = ms;
}

static void aSendManifest(SupeEngine* e) {
    SupeXact* x = &e->x;
    const SupeRegime* g = supeRegime(x->regime);
    uint32_t ceil = SUPE_LEN_MAX_MS;
    if (g && g->trainCeilMs && g->trainCeilMs < ceil) ceil = g->trainCeilMs;

    uint32_t trainMs = 0;
    planTrain(e, SUPE_TRAIN_MAX, ceil, &x->sendCount, &trainMs);
    x->planCount = x->sendCount;
    x->sentCount = 0;

    SupeManifest2 m = {};
    m.pwrDbm  = x->trainTxp;
    m.rssiDbm = x->grantRssi;
    m.snrQ    = x->grantSnrQ;
    m.caps.fam = e->ownFam;
    m.caps.topStep = e->ownTop;
    m.caps.maxPwrDbm = e->txpMax;
    m.caps.adaptive = e->adaptive;
    m.count   = x->sendCount;
    m.lenByte = x->sendCount ? supeEncLen(trainMs) : 0;
    memcpy(m.hash, x->hash, SUPE_HASH_LEN);
    uint8_t f[SUPE_MANIFEST2_LEN];
    size_t n = supeEncManifest2(f, sizeof f, &m);
    if (!n || !e->host->tx_frame(e->host->ctx, f, (uint16_t)n, x->trainTxp)) {
        home(e, false, "MANIFEST would not transmit");
        return;
    }
    x->phase = SUPE_X_A_MAN_TX;
    /* The first packet is built while the MANIFEST is on the air. */
    if (x->sendCount > 0 && !aStageNext(e)) x->sendCount = 0;
    eLog(e, true, "supe: manifest %u packets, %u ms, txp=%d",
         (unsigned)m.count, (unsigned)trainMs, (int)x->trainTxp);
}

/* Stage the requester's next packet — the head of the run — so the coming
 * gap carries no work: the frames are built, tapped and fanned out during
 * the current packet's airtime, and the fire is a pop. */
static bool aStageNext(SupeEngine* e) {
    SupeXact* x = &e->x;
    LoraPkt* p = loraqAt(e->q, 0);
    if (!p) return false;
    if (!e->host->stage_packet(e->host->ctx, p, x->trainTxp)) return false;
    loraqConsume(e->q, 0);    /* the host holds it staged */
    x->staged = true;
    return true;
}

/* One receiver-flip gap, then the pop: the gap exists for the FAR receiver's
 * re-arm, not for us — our next packet is already built. */
static void trainGap(SupeEngine* e, uint8_t leadPhase) {
    e->x.phase = leadPhase;
    e->x.deadlineMs = eNow(e) + SUPE_TRAIN_GAP_MS;
    e->host->rx(e->host->ctx);
    e->host->schedule(e->host->ctx, e->x.deadlineMs);
}

/* The requester's train is done (or could not continue): what the GRANT
 * declared decides whether a reverse MANIFEST is owed. */
static void aTrainDone(SupeEngine* e) {
    SupeXact* x = &e->x;
    if (!x->reverseExpected) {
        /* The GRANT declared nothing coming back: the transaction ends with
         * our own last frame. No close, no wait — home. */
        e->detoursDone++;
        noteSimple(e, x->tag, SUPE_EV_DETOURED);
        home(e, true, "done, nothing owed");
        return;
    }
    /* From here silence genuinely means loss. */
    x->phase = SUPE_X_A_AWAIT_RMAN;
    x->deadlineMs = eNow(e)
        + supeManifestReverseDeadlineMs(&x->cfg, e->crDenom, e->preamble);
    e->host->rx(e->host->ctx);
    e->host->schedule(e->host->ctx, x->deadlineMs);
}

/* Pop the staged packet onto the air, then immediately stage the one behind
 * it — the build rides this packet's airtime. */
static void aFireNext(SupeEngine* e) {
    SupeXact* x = &e->x;
    if (!x->staged || !e->host->fire_staged(e->host->ctx)) {
        home(e, false, "train packet would not transmit");
        return;
    }
    x->staged = false;
    x->sentCount++;
    e->trainPktsOut++;
    x->phase = SUPE_X_A_TRAIN_TX;
    if (x->sentCount < x->sendCount && !aStageNext(e))
        x->sendCount = x->sentCount;   /* the queue changed underneath: end there */
}

/* ─────────────── the answerer ─────────────── */

static uint8_t scanReverse(SupeEngine* e, uint16_t peerId, uint8_t regime,
                           const SupeCfg* cfg, uint32_t* msOut);

static void bRefuse(SupeEngine* e, uint8_t regime, uint8_t reason,
                    int16_t rssi, int16_t snr10, const uint8_t hash[SUPE_HASH_LEN]) {
    SupeGrant g = {};
    g.regime  = regime;
    g.version = SUPE_VERSION;
    g.chan    = reason;
    g.budget  = SUPE_BUDGET_REFUSED;
    g.durByte = 0;
    g.pwrDbm  = e->txpMax;
    g.rssiDbm = rssi;
    g.snrQ    = supeEncSnrQ(snr10);
    memcpy(g.hash, hash, SUPE_HASH_LEN);
    uint8_t f[SUPE_GRANT_LEN];
    size_t n = supeEncGrant(f, sizeof f, &g);
    if (!n) return;
    SupeXact* x = &e->x;
    memset(x, 0, sizeof *x);
    x->phase = SUPE_X_B_GRANT_TX;
    x->role_b = 1;
    x->budget = SUPE_BUDGET_REFUSED;
    x->beganMs = eNow(e);
    memcpy(x->hash, hash, SUPE_HASH_LEN);
    if (!e->host->tx_frame(e->host->ctx, f, (uint16_t)n, e->txpMax)) {
        home(e, false, "refusal would not transmit");
        return;
    }
    e->refusalsOut++;
    eLog(e, true, "supe: refused (reason %u)", (unsigned)reason);
}

static void bAnswerStart(SupeEngine* e, const SupeStart2* s,
                         const uint8_t hash[SUPE_HASH_LEN],
                         int16_t rssi, int16_t snr10) {
    /* The regime of the detour: the request's where we run it, else regime 0 —
     * the universal floor every node supports (§3). Never above the request's. */
    uint8_t gr = (s->regime == e->regime) ? e->regime : SUPE_REGIME_SINGLE;

    /* The channel: ours to choose, from what the airtime ledger and the reuse
     * gaps allow. A random start in the raster keeps every detour on the
     * segment from landing on the same frequency. */
    uint8_t chan = SUPE_CH_HAIL;
    if (gr != SUPE_REGIME_SINGLE) {
        SupeChanView cv;
        e->host->chan_get(e->host->ctx, &cv);
        if (cv.nChans) {
            uint8_t first = (uint8_t)(1 + (e->host->rand32(e->host->ctx) % cv.nChans));
            for (uint8_t i = 0; i < cv.nChans; i++) {
                uint8_t c = (uint8_t)(1 + ((first - 1 + i) % cv.nChans));
                if (c < SUPE_CH_MAX && cv.usable[c]) { chan = c; break; }
            }
        }
        if (chan == SUPE_CH_HAIL) {
            bRefuse(e, gr, cv.anyBudget ? SUPE_REFUSE_NO_QUIET : SUPE_REFUSE_AIRTIME,
                    rssi, snr10, hash);
            return;
        }
    }

    /* The budget: the family-filtered ladder for this channel, capped by the
     * request's ceiling and ours, chosen from our own reading of the START —
     * the headroom its arrival showed against the hailing demodulation floor
     * is the margin the requester's current power buys. */
    uint32_t chanMax = chanMaxBwOf(e, gr, chan);
    SupeLadderEntry lad[SUPE_LADDER_MAX_ENTRIES];
    int n = supeLadder(gr, SUPE_VERSION, e->hailSf, e->hailBwHz, chanMax,
                       e->ownFam, s->fam, lad, SUPE_LADDER_MAX_ENTRIES);
    if (n <= 0) {
        bRefuse(e, gr, SUPE_REFUSE_REGIME, rssi, snr10, hash);
        return;
    }
    int top = n - 1;
    if (top > s->ceiling) top = s->ceiling;
    if (top > e->ownTop)  top = e->ownTop;
    int headroomDeci = (int)snr10 - (int)supeReqSnrDeci(e->hailSf);
    int affordDeci   = headroomDeci - SUPE_TARGET_MARGIN_DB * 10;
    uint8_t budget = 0;
    for (int i = 1; i <= top; i++)
        if ((int)lad[i].marginDeci <= affordDeci) budget = (uint8_t)i;

    SupeCfg cfg = { lad[budget].sf, lad[budget].bwHz, lad[budget].ldro,
                    lad[budget].marginDeci };

    /* The reverse leg, declared from the queue as it stands: the bit is what
     * tells the requester whether a reverse MANIFEST will exist at all — the
     * one question its own count fields can never answer, since the absent
     * frame is the thing in question. */
    SupePeerView pv;
    bool havePv = e->host->peer_get(e->host->ctx, s->tag, &pv) && pv.known;
    uint16_t revPeer = havePv ? pv.peerId : (uint16_t)LORAQ_PEER_NONE;
    uint32_t revMs = 0;
    bool reverse = scanReverse(e, revPeer, gr, &cfg, &revMs) > 0;

    /* The duration everyone else holds for: the requester's load at the
     * modulation just chosen, the MANIFESTs, the turnarounds, and our own
     * declared train. */
    uint32_t manMs = toaFrameMs(e, &cfg, SUPE_MANIFEST2_LEN);
    uint32_t durMs = SUPE_RETUNE_GAP_MS + SUPE_MAN_LEAD_MS + SUPE_TURNAROUND_MS
                     + manMs + supeLoadAirtimeMs(s->load, &cfg, e->crDenom)
                     + SUPE_TRAIN_MAX * SUPE_TRAIN_GAP_MS
                     + 2 * SUPE_GUARD_MS;
    if (reverse) durMs += SUPE_TURNAROUND_MS + manMs + revMs;
    const SupeRegime* g = supeRegime(gr);
    if (g && g->txnCeilMs && durMs > g->txnCeilMs) durMs = g->txnCeilMs;
    if (durMs > SUPE_DUR_MAX_MS) durMs = SUPE_DUR_MAX_MS;

    SupeGrant gt = {};
    gt.regime  = gr;
    gt.version = SUPE_VERSION;
    gt.chan    = chan;
    gt.budget  = budget;
    gt.reverse = reverse;
    gt.durByte = supeEncDur(durMs);
    gt.pwrDbm  = e->txpMax;       /* a GRANT is never adapted (§15) */
    gt.rssiDbm = rssi;
    gt.snrQ    = supeEncSnrQ(snr10);
    memcpy(gt.hash, hash, SUPE_HASH_LEN);
    uint8_t f[SUPE_GRANT_LEN];
    size_t fn = supeEncGrant(f, sizeof f, &gt);
    if (!fn) return;

    SupeXact* x = &e->x;
    memset(x, 0, sizeof *x);
    x->phase   = SUPE_X_B_GRANT_TX;
    x->role_b  = 1;
    memcpy(x->tag, s->tag, SUPE_TAG_LEN);
    memcpy(x->hash, hash, SUPE_HASH_LEN);
    x->regime  = gr;
    x->chan    = chan;
    x->budget  = budget;
    x->cfg     = cfg;
    x->durMs   = supeDecDur(gt.durByte);
    x->beganMs = eNow(e);
    x->reverseExpected = reverse;
    x->lastPktRssi = rssi;                 /* until a train packet lands */
    x->lastPktSnrQ = supeEncSnrQ(snr10);
    x->trainTxp = chanTxpCapOf(e, gr, chan);
    if (havePv) {
        x->peerId = pv.peerId;
        if (e->adaptive && pv.txpOpen < x->trainTxp) x->trainTxp = pv.txpOpen;
    } else {
        x->peerId = LORAQ_PEER_NONE;
    }
    if (!e->host->tx_frame(e->host->ctx, f, (uint16_t)fn, e->txpMax)) {
        home(e, false, "GRANT would not transmit");
        return;
    }
    e->grantsOut++;
    eLog(e, true, "supe: granted ch%u budget %u for %u ms%s",
         (unsigned)chan, (unsigned)budget, (unsigned)x->durMs,
         reverse ? ", reverse pending" : "");
}

/* The answering side's reverse leg as the queue stands right now: what the
 * GRANT's bit declares and what its own MANIFEST will later describe. The
 * count may grow between the two — anything that enqueues during the
 * requester's train joins the reverse train for free. */
static uint8_t scanReverse(SupeEngine* e, uint16_t peerId, uint8_t regime,
                           const SupeCfg* cfg, uint32_t* msOut) {
    uint8_t count = 0;
    uint32_t ms = 0;
    if (peerId != LORAQ_PEER_NONE) {
        const SupeRegime* g = supeRegime(regime);
        uint32_t ceil = SUPE_LEN_MAX_MS;
        if (g && g->trainCeilMs && g->trainCeilMs < ceil) ceil = g->trainCeilMs;
        for (uint8_t i = 0; i < loraqDepth(e->q) && count < SUPE_TRAIN_MAX; i++) {
            LoraPkt* p = loraqAt(e->q, i);
            if (p->peer_id != peerId) continue;
            uint32_t add = pktAirMs(e, cfg, p->len) + SUPE_TRAIN_GAP_MS;
            if (count > 0 && ms + add > ceil) break;
            ms += add;
            count++;
        }
    }
    if (msOut) *msOut = ms;
    return count;
}

static void bTurn(SupeEngine* e);

/* The requester's train is in (or its length expired). Our declaration in
 * the GRANT decides what follows: the reverse leg, or nothing at all. */
static void bAfterTrain(SupeEngine* e) {
    if (e->x.reverseExpected) { bTurn(e); return; }
    e->detoursDone++;
    /* Count completion and length expiry both land here, and only one of
     * them means a packet went missing — say which, so a shortfall in the
     * end records reads as the deadline it was. */
    home(e, true, e->x.lenExpired ? "train length expired" : "done");
}

/* The answering side's turn: its own train, or — only in the corner where the
 * declared traffic vanished underneath (a radio cycle) — a count-0 MANIFEST,
 * which keeps its one meaning: nothing after all, done. */
static void bTurn(SupeEngine* e) {
    SupeXact* x = &e->x;
    uint32_t ms = 0;
    uint8_t count = scanReverse(e, x->peerId, x->regime, &x->cfg, &ms);

    SupeManifest2 m = {};
    m.pwrDbm  = x->trainTxp;
    m.rssiDbm = x->lastPktRssi;
    m.snrQ    = x->lastPktSnrQ;
    m.caps.fam = e->ownFam;
    m.caps.topStep = e->ownTop;
    m.caps.maxPwrDbm = e->txpMax;
    m.caps.adaptive = e->adaptive;
    memcpy(m.hash, x->hash, SUPE_HASH_LEN);
    if (count > 0) {
        m.count = count;
        m.lenByte = supeEncLen(ms);
        x->pendingKind = 1;                          /* a train follows */
        x->sendCount = count;
        x->planCount = count;
        x->sentCount = 0;
    } else {
        m.count = 0;
        m.lenByte = 0;
        x->pendingKind = 3;                          /* nothing after all */
    }
    uint8_t f[SUPE_MANIFEST2_LEN];
    size_t n = supeEncManifest2(f, sizeof f, &m);
    if (!n || !e->host->tx_frame(e->host->ctx, f, (uint16_t)n, x->trainTxp)) {
        home(e, false, "reverse MANIFEST would not transmit");
        return;
    }
    x->phase = SUPE_X_B_MAN_TX;
    /* The first packet is built while the MANIFEST is on the air. */
    if (x->pendingKind == 1 && !bStageNext(e)) x->pendingKind = 3;
}

/* Stage the answering side's next packet: the first queue entry for the
 * requester, built during the current airtime. */
static bool bStageNext(SupeEngine* e) {
    SupeXact* x = &e->x;
    for (uint8_t i = 0; i < loraqDepth(e->q); i++) {
        LoraPkt* p = loraqAt(e->q, i);
        if (p->peer_id != x->peerId) continue;
        if (!e->host->stage_packet(e->host->ctx, p, x->trainTxp)) return false;
        loraqConsume(e->q, i);
        x->staged = true;
        return true;
    }
    return false;
}

static void bFireNext(SupeEngine* e) {
    SupeXact* x = &e->x;
    if (!x->staged || !e->host->fire_staged(e->host->ctx)) {
        home(e, false, "train packet would not transmit");
        return;
    }
    x->staged = false;
    x->sentCount++;
    e->trainPktsOut++;
    x->phase = SUPE_X_B_TRAIN_TX;
    if (x->sentCount < x->sendCount && !bStageNext(e))
        x->sendCount = x->sentCount;   /* the queue changed underneath */
}

/* ─────────────── receive dispatch ─────────────── */

static void onStart(SupeEngine* e, const uint8_t* f, uint16_t len,
                    int16_t rssi, int16_t snr10) {
    uint8_t sha[32];
    e->host->sha256(e->host->ctx, f, len, sha);

    SupeStart2 s;
    if (!supeDecStart2(f, len, &s)) {
        /* The layout is stable even when the regime is not ours to speak: a
         * START of the right shape for our tag deserves a refusal naming the
         * reason rather than a silence the requester will read as absence. */
        if ((len == SUPE_START2_LEN || len == SUPE_START2_ID_LEN) &&
            f[0] == SUPE_T_START && e->x.phase == SUPE_X_IDLE &&
            supeEngTagIsOurs(e, f + 2)) {
            bRefuse(e, SUPE_REGIME_SINGLE, SUPE_REFUSE_REGIME, rssi, snr10, sha);
            return;
        }
        e->rxDiscard++;
        return;
    }
    seenAdd(e, sha, &s);
    if (!supeEngTagIsOurs(e, s.tag)) return;   /* the hold rides the GRANT */
    if (e->expired) { e->rxDiscard++; return; }
    if (e->x.phase != SUPE_X_IDLE) return;     /* whoever has the radio keeps it */
    bAnswerStart(e, &s, sha, rssi, snr10);
}

static void onGrant(SupeEngine* e, const uint8_t* f, uint16_t len,
                    int16_t rssi, int16_t snr10) {
    SupeGrant g;
    if (!supeDecGrant(f, len, &g)) { e->rxDiscard++; return; }
    SupeXact* x = &e->x;

    if (x->phase == SUPE_X_A_AWAIT_GRANT &&
        memcmp(g.hash, x->hash, SUPE_HASH_LEN) == 0) {
        noteSimple(e, x->tag, SUPE_EV_ALIVE);
        if (supeGrantRefused(&g)) {
            e->refusalsIn++;
            SupePeerNote nt = {};
            nt.ev = SUPE_EV_REFUSED;
            nt.reason = g.chan;
            nt.backoffMs = refusalBackoffMs(g.chan);
            e->host->peer_note(e->host->ctx, x->tag, &nt);
            /* The head still flies — plainly, and without being re-asked. */
            e->plainOnce = true;
            SupeCfg hail = hailCfgOf(e);
            notePair(e, x->tag, &hail, rssi, g.pwrDbm);
            home(e, false, "refused");
            return;
        }
        if (g.regime != x->regime && g.regime != SUPE_REGIME_SINGLE) {
            e->rxDiscard++;
            return;                       /* raised the regime: not a legal answer */
        }
        uint32_t chanMax = chanMaxBwOf(e, g.regime, g.chan);
        SupeCfg cfg;
        if (!chanMax ||
            !supeResolveBudget(g.regime, SUPE_VERSION, e->hailSf, e->hailBwHz,
                               chanMax, e->ownFam, x->peerFam, g.budget, &cfg)) {
            e->rxDiscard++;
            return;                       /* an index the ladder does not reach */
        }
        e->grantsIn++;
        SupeCfg hail = hailCfgOf(e);
        notePair(e, x->tag, &hail, rssi, g.pwrDbm);
        x->regime  = g.regime;
        x->chan    = g.chan;
        x->budget  = g.budget;
        x->cfg     = cfg;
        x->durMs   = supeDecDur(g.durByte);
        x->reverseExpected = g.reverse;
        x->grantRssi = rssi;
        x->grantSnrQ = supeEncSnrQ(snr10);
        x->trainTxp  = chanTxpCapOf(e, g.regime, g.chan);
        if (e->adaptive) {
            SupePeerView pv;
            if (e->host->peer_get(e->host->ctx, x->tag, &pv) && pv.known &&
                pv.txpOpen < x->trainTxp)
                x->trainTxp = pv.txpOpen;
        }
        if (!e->host->tune(e->host->ctx, g.chan, &cfg,
                           supeSyncWordFor(g.regime, &cfg, g.budget, e->ifaceSync))) {
            home(e, false, "retune failed");
            e->plainOnce = true;          /* a retune that fails once will fail again */
            return;
        }
        x->retuned = true;
        x->phase = SUPE_X_A_RETUNE;
        x->deadlineMs = eNow(e) + SUPE_RETUNE_GAP_MS + SUPE_MAN_LEAD_MS;
        e->host->schedule(e->host->ctx, x->deadlineMs);
        return;
    }

    /* Somebody else's exchange. Everyone who hears a GRANT holds traffic for
     * its tag — and for the requester's identity, when the START named one —
     * for the stated duration (§6). The granter holding the tag is alive. */
    e->rxForeign++;
    SupeStartSeen* seen = seenFind(e, g.hash);
    if (seen) {
        uint32_t dur = supeDecDur(g.durByte);
        if (!supeGrantRefused(&g) && dur) {
            holdAdd(e, seen->tag, dur);
            if (seen->haveIdent) holdAdd(e, seen->ident, dur);
        }
        noteSimple(e, seen->tag, SUPE_EV_ALIVE);
        if (x->phase == SUPE_X_A_AWAIT_GRANT &&
            memcmp(seen->tag, x->tag, SUPE_TAG_LEN) == 0) {
            /* Our tag, somebody else's hash: the peer granted the other
             * requester. Direct evidence it is present and busy — never an
             * absence verdict. The hold above paces the retry. */
            eLog(e, true, "supe: lost the collision — peer busy");
            home(e, false, "collision");
        }
    }
}

static void onManifest(SupeEngine* e, const uint8_t* f, uint16_t len,
                       int16_t rssi, int16_t snr10) {
    (void)snr10;
    SupeManifest2 m;
    if (!supeDecManifest2(f, len, &m)) { e->rxDiscard++; return; }
    SupeXact* x = &e->x;
    if (x->phase == SUPE_X_IDLE) { e->rxForeign++; return; }
    if (memcmp(m.hash, x->hash, SUPE_HASH_LEN) != 0) {
        /* The channel is carrying somebody else's exchange (§8). Leave at
         * once rather than counting their train as our own. */
        e->rxForeign++;
        home(e, false, "manifest for another exchange");
        return;
    }

    /* The MANIFEST carries the sender's capabilities unconditionally (§8);
     * the platform files them where the tag resolves — for a link-id tag that
     * is the only handle there will ever be on the node that dialled us. */
    {
        SupePeerNote nc = {};
        nc.ev = SUPE_EV_CAPS;
        nc.caps = m.caps;
        e->host->peer_note(e->host->ctx, x->tag, &nc);
    }

    switch (x->phase) {
        case SUPE_X_B_AWAIT_MAN:
            /* The requester's opening MANIFEST, and pair 3 of §10: its train
             * power against our reading, at the detour's modulation. */
            notePair(e, x->tag, &x->cfg, rssi, m.pwrDbm);
            x->expectCount = m.count;
            x->gotCount = 0;
            if (m.count > 0) {
                x->phase = SUPE_X_B_RECV;
                x->deadlineMs = eNow(e) + supeLenDeadlineMs(m.lenByte)
                              + (uint32_t)m.count * SUPE_TRAIN_RX_SLACK_MS;
                e->host->schedule(e->host->ctx, x->deadlineMs);
            } else {
                /* Count 0 is the close of that direction, whatever the length
                 * byte says: this protocol does not park on a private channel
                 * because a frame asked it to wait for something. */
                if (m.lenByte > 0)
                    eLog(e, true, "supe: peer stated a grace — not staying");
                bAfterTrain(e);        /* nothing coming: our leg, or home */
            }
            return;

        case SUPE_X_A_AWAIT_RMAN: {
            /* The reverse MANIFEST: our train is confirmed, at the level it
             * landed (§15's whole evidence loop), and pair 4 of §10 with it. */
            notePair(e, x->tag, &x->cfg, rssi, m.pwrDbm);
            SupePeerNote nt = {};
            nt.ev = SUPE_EV_TRAIN_OK;
            nt.cfg = x->cfg;
            nt.rssiDbm = m.rssiDbm;      /* how OUR train was heard over there */
            nt.txpDbm = x->trainTxp;
            e->host->peer_note(e->host->ctx, x->tag, &nt);
            if (m.count > 0) {
                x->expectCount = m.count;
                x->gotCount = 0;
                x->phase = SUPE_X_A_RECV;
                x->deadlineMs = eNow(e) + supeLenDeadlineMs(m.lenByte)
                              + (uint32_t)m.count * SUPE_TRAIN_RX_SLACK_MS;
                e->host->schedule(e->host->ctx, x->deadlineMs);
            } else {
                if (m.lenByte > 0)
                    eLog(e, true, "supe: peer stated a grace — not staying");
                e->detoursDone++;
                noteSimple(e, x->tag, SUPE_EV_DETOURED);
                home(e, true, "closed");
            }
            return;
        }

        default:
            e->rxForeign++;
            return;
    }
}

void supeEngOnRx(SupeEngine* e, const uint8_t* f, uint16_t len,
                 int16_t rssi, int16_t snr10) {
    if (len < 1) return;
    e->rxFrames++;
    if (e->expired) { e->rxDiscard++; return; }
    switch (f[0]) {
        case SUPE_T_START:    onStart(e, f, len, rssi, snr10);    break;
        case SUPE_T_GRANT:    onGrant(e, f, len, rssi, snr10);    break;
        case SUPE_T_MANIFEST: onManifest(e, f, len, rssi, snr10); break;
        default:              e->rxDiscard++;                     break;
    }
}

void supeEngOnPacketRx(SupeEngine* e, int16_t rssi, int16_t snr10) {
    SupeXact* x = &e->x;
    if (x->phase != SUPE_X_A_RECV && x->phase != SUPE_X_B_RECV) return;
    x->gotCount++;
    e->trainPktsIn++;
    x->lastPktRssi = rssi;
    x->lastPktSnrQ = supeEncSnrQ(snr10);
    if (x->gotCount < x->expectCount) return;
    if (x->phase == SUPE_X_A_RECV) {
        e->detoursDone++;
        noteSimple(e, x->tag, SUPE_EV_DETOURED);
        home(e, true, "trains complete");
    } else {
        bAfterTrain(e);
    }
}

void supeEngOnTxDone(SupeEngine* e, bool ok) {
    SupeXact* x = &e->x;
    if (x->phase == SUPE_X_IDLE) return;
    if (!ok) {
        /* The frame never left the air; every continuation below would be
         * acting on something that did not happen. */
        home(e, false, "transmit aborted");
        return;
    }
    switch (x->phase) {
        case SUPE_X_A_START_TX:
            x->phase = SUPE_X_A_AWAIT_GRANT;
            x->deadlineMs = eNow(e)
                + supeGrantDeadlineMs(e->hailSf, e->hailBwHz, e->crDenom, e->preamble);
            e->host->rx(e->host->ctx);
            e->host->schedule(e->host->ctx, x->deadlineMs);
            return;

        case SUPE_X_B_GRANT_TX:
            if (x->budget == SUPE_BUDGET_REFUSED) {
                home(e, false, "refusal out");
                return;
            }
            if (!e->host->tune(e->host->ctx, x->chan, &x->cfg,
                               supeSyncWordFor(x->regime, &x->cfg, x->budget,
                                               e->ifaceSync))) {
                home(e, false, "retune failed");
                return;
            }
            x->retuned = true;
            x->phase = SUPE_X_B_AWAIT_MAN;
            x->deadlineMs = eNow(e)
                + supeManifestFirstDeadlineMs(&x->cfg, e->crDenom, e->preamble);
            e->host->rx(e->host->ctx);
            e->host->schedule(e->host->ctx, x->deadlineMs);
            return;

        case SUPE_X_A_MAN_TX:
            /* The first packet was staged while the MANIFEST flew. */
            if (x->sendCount > 0 && x->staged) trainGap(e, SUPE_X_A_TRAIN_LEAD);
            else                               aTrainDone(e);
            return;
        case SUPE_X_A_TRAIN_TX:
            if (x->staged) trainGap(e, SUPE_X_A_TRAIN_LEAD);
            else           aTrainDone(e);
            return;

        case SUPE_X_B_MAN_TX:
            if (x->pendingKind == 1 && x->staged) {
                trainGap(e, SUPE_X_B_TRAIN_LEAD);
                return;
            }
            e->detoursDone++;
            home(e, true, x->pendingKind == 1 ? "own train out" : "closed");
            return;
        case SUPE_X_B_TRAIN_TX:
            if (x->staged) trainGap(e, SUPE_X_B_TRAIN_LEAD);
            else {
                e->detoursDone++;
                noteSimple(e, x->tag, SUPE_EV_DETOURED);
                home(e, true, "own train out");
            }
            return;

        default:
            return;
    }
}

void supeEngOnTimer(SupeEngine* e) {
    SupeXact* x = &e->x;
    switch (x->phase) {
        case SUPE_X_A_AWAIT_GRANT: {
            /* Silence. The ladder advances: a strike, a randomised wait, and
             * the verdict path re-requests — or, at three strikes, the peer is
             * absent and its traffic drops for a minute (§11). */
            e->strikes++;
            SupePeerNote nt = {};
            nt.ev = SUPE_EV_STRIKE;
            nt.backoffMs = SUPE_RETRY_WAIT_MIN_MS
                + (e->host->rand32(e->host->ctx) % SUPE_RETRY_WAIT_SPAN_MS);
            nt.triedTxpDbm = x->startTxp;
            e->host->peer_note(e->host->ctx, x->tag, &nt);
            home(e, false, "no GRANT");
            return;
        }
        case SUPE_X_A_RETUNE:
            aSendManifest(e);
            return;
        case SUPE_X_A_TRAIN_LEAD:
            aFireNext(e);
            return;
        case SUPE_X_B_TRAIN_LEAD:
            bFireNext(e);
            return;
        case SUPE_X_A_AWAIT_RMAN: {
            /* No reverse MANIFEST: the power may have been too low, or the
             * peer may be gone. Both want more power next time (§15). */
            SupePeerNote nt = {};
            nt.ev = SUPE_EV_TRAIN_LOST;
            nt.triedTxpDbm = x->trainTxp;
            e->host->peer_note(e->host->ctx, x->tag, &nt);
            home(e, false, "no reverse MANIFEST");
            return;
        }
        case SUPE_X_A_RECV:
            /* The stated length expired before the count did — an outcome the
             * MANIFEST alone made computable. What arrived, arrived. */
            e->detoursDone++;
            noteSimple(e, x->tag, SUPE_EV_DETOURED);
            home(e, true, "train length expired");
            return;
        case SUPE_X_B_AWAIT_MAN:
            home(e, false, "no MANIFEST");
            return;
        case SUPE_X_B_RECV:
            e->x.lenExpired = true;
            bAfterTrain(e);
            return;
        default:
            return;
    }
}

size_t supeEngBuildAnn(SupeEngine* e, uint8_t* out, size_t cap,
                       const uint8_t ids[][SUPE_ID_LEN], uint8_t count,
                       int8_t pwrDbm) {
    SupeAnn2 a = {};
    a.regime = e->regime;
    a.version = SUPE_VERSION;
    a.caps.fam = e->ownFam;
    a.caps.topStep = e->ownTop;
    a.caps.maxPwrDbm = e->txpMax;
    a.caps.adaptive = e->adaptive;
    a.pwrDbm = pwrDbm;
    a.count = count > SUPE_ANN2_MAX ? SUPE_ANN2_MAX : count;
    for (int i = 0; i < a.count; i++) memcpy(a.ids[i], ids[i], SUPE_ID_LEN);
    return supeEncAnn2(out, cap, &a);
}
