/**
 * supe_engine_test — the engine plus a stub host is a program that steps whole
 * meetings on a laptop, which is the point of SupeHost: nothing in the engine
 * names a radio, a task or a timer, so this file supplies all three as plain
 * data and a loop.
 *
 * What it steps: a PRIVSYNC seeding a schedule and the meeting a slot opens,
 * the return leg riding the answering HAVEDATA, a repair round recovering a
 * dropped frame, in-sequence delivery around a hole, the absence ladder on
 * tight-schedule expiry, the no-evidence rules, contact consuming a schedule,
 * and the seed-hash gate.
 *
 *   make -C iface-lora/esp-idf/test engine
 */
#include "../src/supe_engine.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <string>

static int g_fail = 0;
static int g_run  = 0;

static void ok(bool cond, const char* what) {
    g_run++;
    if (!cond) { g_fail++; printf("FAIL  %s\n", what); }
}

static void eqi(long got, long want, const char* what) {
    g_run++;
    if (got != want) { g_fail++; printf("FAIL  %s: got %ld want %ld\n", what, got, want); }
}

/* Bounds-safe peek into a delivered-frames list: -1 where a test's expectation
 * ran past what actually arrived, so a shortfall reads as a FAIL, not a crash. */
static int dByte(const std::vector<std::vector<uint8_t>>& v, size_t i, size_t j) {
    if (i >= v.size() || j >= v[i].size()) return -1;
    return v[i][j];
}

/* ─────────────── the stub host ─────────────── */

static uint32_t g_now = 1000;
static uint32_t g_rand = 12345;

struct AirItem {
    bool     isTrain;                  /* fired through train_fire */
    std::vector<uint8_t> bytes;
    int8_t   dbm;
};

/* The peer record a real host keeps in its peer table; the note handler below
 * mirrors what lora_supe.cpp's does, because that contract is exactly what is
 * under test. */
struct StubPeer {
    bool     known = false;
    uint16_t peerId = LORAQ_PEER_NONE;
    uint8_t  fam = SUPE_FAM_SX126X, topBudget = 8;
    int8_t   maxTxpDbm = 14;
    int8_t   txpOpen = 14, txpMax = 14;
    uint8_t  strikes = 0;
    uint32_t absentUntilMs = 0, retryWaitUntilMs = 0;
    bool     detoured = false;
};

struct TxFrameCopy {
    std::vector<uint8_t> bytes;
    uint8_t* fromPkt;                  /* the queue heap block it was cut from */
};

struct Node {
    const char* name;
    SupeEngine eng;
    LoraQueue  q;
    SupeHost   host;
    /* radio */
    bool     home = true;
    uint8_t  chan = 0;
    SupeCfg  cfg = {};
    uint8_t  sync = 0x42;
    uint32_t tunes = 0, homes = 0;
    bool     ccaClear = true;
    /* one-shot timer */
    bool     schedArmed = false;
    uint32_t schedAt = 0;
    /* what went on the air, awaiting tx-done + delivery */
    std::vector<AirItem> txq;
    /* the outgoing train, held to the close */
    std::vector<TxFrameCopy> train;
    bool     trainHeld = false;
    uint32_t trainsDelivered = 0, trainsDropped = 0;
    /* the inbound buffer, arrival order; flushed on train_deliver */
    std::vector<std::vector<uint8_t>> rxBuf;
    std::vector<std::vector<uint8_t>> delivered;
    /* the host's peer table, one tag */
    uint8_t  peerTag[3] = {};
    StubPeer peer;
    std::vector<SupePeerNote> notes;
    SupeChanView chans = {};
};

static uint32_t chanKey(const Node* n) {
    if (n->home) return 0x42u;                     /* hailing: chan 0, 0x42 */
    return 1000u + n->chan * 1000u + n->cfg.sf * 100u + n->sync;
}

static uint32_t hNow(void*) { return g_now; }
static uint32_t hRand(void*) { g_rand = g_rand * 1103515245u + 12345u; return g_rand >> 8; }
static void hSched(void* c, uint32_t at) {
    Node* n = (Node*)c;
    n->schedArmed = true;
    n->schedAt = at;
}
static void hSha(void*, const uint8_t* d, uint16_t n, uint8_t out[32]) {
    /* Not SHA-256 — both ends run the same function, which is all the hash is
     * for here (the schedule derivation and its wire id). FNV-1a folded. */
    uint32_t h = 2166136261u;
    for (uint16_t i = 0; i < n; i++) { h ^= d[i]; h *= 16777619u; }
    for (int i = 0; i < 32; i++) { out[i] = (uint8_t)(h >> ((i % 4) * 8)); h = h * 31 + 7; }
}
static bool hTune(void* c, uint8_t chan, const SupeCfg* cfg, uint8_t sync) {
    Node* n = (Node*)c;
    n->home = false;
    n->chan = chan;
    n->cfg = *cfg;
    n->sync = sync;
    n->tunes++;
    return true;
}
static void hTuneHome(void* c) {
    Node* n = (Node*)c;
    n->home = true;
    n->sync = 0x42;
    n->homes++;
}
static bool hTxFrame(void* c, const uint8_t* f, uint16_t len, int8_t dbm) {
    Node* n = (Node*)c;
    n->txq.push_back({ false, std::vector<uint8_t>(f, f + len), dbm });
    return true;
}
static void hRx(void*) {}
/* A reception under way, as the modem would report it: set for as long as a
 * test wants the radio held. */
static bool g_rxBusy = false;
static bool hRxBusy(void*) { return g_rxBusy; }
static bool hCca(void* c) { return ((Node*)c)->ccaClear; }

/* The stub's train pipeline: frames are copies cut from the queue — one frame
 * per packet up to 200 bytes, two above — held until train_done, consumed only
 * on a delivered close. */
static bool hTrainBuild(void* c, uint16_t peerId, const uint8_t tag[3],
                        uint8_t maxFrames, SupeTrainInfo* out) {
    Node* n = (Node*)c;
    n->train.clear();
    memset(out, 0, sizeof *out);
    for (uint8_t i = 0; i < loraqDepth(&n->q) && out->count < maxFrames; i++) {
        LoraPkt* p = loraqAt(&n->q, i);
        bool match = false;
        if (tag && (p->flags & LORAQ_F_HAVE_TAG) && memcmp(p->tag, tag, 3) == 0)
            match = true;
        if (peerId != LORAQ_PEER_NONE && p->peer_id == peerId) match = true;
        if (!match) continue;
        uint16_t first = p->len > 200 ? 200 : p->len;
        for (int half = 0; half < (p->len > 200 ? 2 : 1); half++) {
            if (out->count >= maxFrames) break;
            TxFrameCopy fc;
            fc.fromPkt = p->bytes;
            fc.bytes.push_back((uint8_t)(0x02 | (half ? 0x10 : 0x00)));
            const uint8_t* src = p->bytes + (half ? first : 0);
            uint16_t nb = half ? (uint16_t)(p->len - first) : first;
            fc.bytes.insert(fc.bytes.end(), src, src + nb);
            out->lens[out->count] = (uint16_t)fc.bytes.size();
            out->csum[out->count] = supeCrc8(fc.bytes.data(), fc.bytes.size());
            out->count++;
            n->train.push_back(std::move(fc));
        }
    }
    n->trainHeld = out->count > 0;
    return out->count > 0;
}
static bool hTrainFire(void* c, uint8_t idx, int8_t dbm) {
    Node* n = (Node*)c;
    if (idx >= n->train.size()) return false;
    n->txq.push_back({ true, n->train[idx].bytes, dbm });
    return true;
}
static void hTrainDone(void* c, bool delivered) {
    Node* n = (Node*)c;
    if (delivered) {
        n->trainsDelivered++;
        /* Consume the queue entries the frames were cut from, by heap block. */
        for (auto& fc : n->train) {
            for (uint8_t i = 0; i < loraqDepth(&n->q); i++) {
                if (loraqAt(&n->q, i)->bytes == fc.fromPkt) { loraqConsume(&n->q, i); break; }
            }
        }
    } else {
        n->trainsDropped++;
    }
    n->train.clear();
    n->trainHeld = false;
}
static void hTrainDeliver(void* c, const uint8_t* order, uint8_t cnt) {
    Node* n = (Node*)c;
    for (uint8_t i = 0; i < cnt; i++)
        if (order[i] < n->rxBuf.size())
            n->delivered.push_back(n->rxBuf[order[i]]);
    n->rxBuf.clear();
}

static bool hPeerGet(void* c, const uint8_t tag[3], SupePeerView* out) {
    Node* n = (Node*)c;
    memset(out, 0, sizeof *out);
    out->peerId = LORAQ_PEER_NONE;
    out->txpMax = 14;
    out->txpOpen = 14;
    if (!n->peer.known || memcmp(tag, n->peerTag, 3) != 0) return false;
    out->known = true;
    out->peerId = n->peer.peerId;
    out->fam = n->peer.fam;
    out->topBudget = n->peer.topBudget;
    out->maxTxpDbm = n->peer.maxTxpDbm;
    out->txpOpen = n->peer.txpOpen;
    out->txpMax = n->peer.txpMax;
    out->absentStrikes = n->peer.strikes;
    out->absentUntilMs = n->peer.absentUntilMs;
    out->retryWaitUntilMs = n->peer.retryWaitUntilMs;
    out->detoured = n->peer.detoured;
    return true;
}
static int8_t hTxpOpen(void* c, const uint8_t tag[3], const SupeCfg*) {
    Node* n = (Node*)c;
    if (!n->peer.known || memcmp(tag, n->peerTag, 3) != 0) return 14;
    return n->peer.txpOpen;
}
static void hPeerNote(void* c, const uint8_t tag[3], const SupePeerNote* nt) {
    Node* n = (Node*)c;
    n->notes.push_back(*nt);
    if (memcmp(tag, n->peerTag, 3) != 0) return;
    switch (nt->ev) {
        case SUPE_EV_ALIVE:
        case SUPE_EV_MET:
            n->peer.strikes = 0;
            n->peer.absentUntilMs = 0;
            n->peer.retryWaitUntilMs = 0;
            if (nt->ev == SUPE_EV_MET) n->peer.detoured = true;
            break;
        case SUPE_EV_STRIKE:
            n->peer.strikes++;
            n->peer.retryWaitUntilMs = g_now
                + (nt->backoffMs > nt->agoMs ? nt->backoffMs - nt->agoMs : 0);
            if (n->peer.strikes >= 3) n->peer.absentUntilMs = g_now + 60000;
            break;
        default:
            break;
    }
}
static void hChanGet(void* c, SupeChanView* out) { *out = ((Node*)c)->chans; }
static void hLog(void* c, const char* msg) {
    if (getenv("VERBOSE")) printf("  [%s %6u] %s\n", ((Node*)c)->name, g_now, msg);
}

/* Each node's own identity, which its PRIVSYNCs carry. The listening side has
 * nothing else to name a seeker by: the tag a PRIVSYNC asks about is one of
 * the LISTENER'S addresses. */
static const uint8_t IDENT_A[3] = { 0xa1, 0xa2, 0xa3 };
static const uint8_t IDENT_B[3] = { 0xb1, 0xb2, 0xb3 };
static const uint8_t TAG[3]     = { 0xd1, 0x0d, 0x51 };

static void nodeInit(Node* n, const char* name, uint8_t regime) {
    n->name = name;
    n->host = {};
    n->host.ctx = n;
    n->host.now_ms = hNow;
    n->host.rand32 = hRand;
    n->host.schedule = hSched;
    n->host.sha256 = hSha;
    n->host.tune = hTune;
    n->host.tune_home = hTuneHome;
    n->host.tx_frame = hTxFrame;
    n->host.rx = hRx;
    n->host.rx_busy = hRxBusy;
    n->host.cca = hCca;
    n->host.train_build = hTrainBuild;
    n->host.train_fire = hTrainFire;
    n->host.train_done = hTrainDone;
    n->host.train_deliver = hTrainDeliver;
    n->host.peer_get = hPeerGet;
    n->host.peer_note = hPeerNote;
    n->host.txp_open = hTxpOpen;
    n->host.chan_get = hChanGet;
    n->host.log = hLog;
    n->host.dbgLevel = true;
    loraqInit(&n->q);
    supeEngInit(&n->eng, &n->host, &n->q);
    supeEngConfig(&n->eng, regime, SUPE_FAM_SX126X, 8, 14,
                  7, 125000, 5, 12, 0x42, 254);
    supeEngSetIdent(&n->eng, name[0] == 'A' ? IDENT_A : IDENT_B);
    n->chans = {};
    n->chans.nChans = (regime == SUPE_REGIME_EU863) ? 9 : 0;
    n->chans.anyBudget = true;
    for (int c = 1; c <= n->chans.nChans; c++) n->chans.usable[c] = 1;
}

static void pushPkt(Node* n, const uint8_t tag[3], uint16_t peerId, uint16_t len,
                    uint8_t fill) {
    uint8_t* b = (uint8_t*)malloc(len);
    memset(b, fill, len);
    ok(loraqPush(&n->q, b, len, g_now, peerId, tag, 1,
                 LORAQ_ORIG_RNSD | (tag ? LORAQ_F_HAVE_TAG : 0)),
       "test packet enqueues");
}

/* ─────────────── the air ─────────────── */

/* Drop filter for the loss tests: drop the Nth train frame seen (1-based),
 * once; 0 = drop nothing. dropRepairs also drops every LATER transmission of
 * a frame already dropped — the repair round's resend of the same bytes. */
static int  g_dropNthTrain = 0;
static uint8_t g_dropType = 0;   /* drop the first SUPE frame of this type */
static uint8_t g_dropNthType = 0; /* …or the g_dropNth'th of this type */
static int     g_dropNth = 0;
static int     g_typeSeen = 0;
static bool g_dropRepairs = false;
static int  g_trainSeen = 0;
static std::vector<uint8_t> g_droppedCsums;

static bool airPump(std::vector<Node*>& nodes) {
    bool moved = false;
    for (Node* src : nodes) {
        if (src->txq.empty()) continue;
        moved = true;
        AirItem it = src->txq.front();
        src->txq.erase(src->txq.begin());
        uint32_t key = chanKey(src);
        supeEngOnTxDone(&src->eng, true);
        bool drop = false;
        if (it.isTrain) {
            g_trainSeen++;
            uint8_t csum = supeCrc8(it.bytes.data(), it.bytes.size());
            if (g_dropNthTrain && g_trainSeen == g_dropNthTrain) {
                drop = true;
                g_droppedCsums.push_back(csum);
            } else if (g_dropRepairs) {
                for (uint8_t c : g_droppedCsums) if (c == csum) drop = true;
            }
        }
        if (!it.isTrain && g_dropType && !it.bytes.empty() &&
            it.bytes[0] == g_dropType) {
            g_dropType = 0;
    g_dropNthType = 0; g_dropNth = 0; g_typeSeen = 0;                       /* one frame, once */
            continue;
        }
        if (!it.isTrain && g_dropNthType && !it.bytes.empty() &&
            it.bytes[0] == g_dropNthType && ++g_typeSeen == g_dropNth) {
            continue;
        }
        if (drop) continue;
        for (Node* dst : nodes) {
            if (dst == src || chanKey(dst) != key) continue;
            if (it.isTrain) {
                uint8_t csum = supeCrc8(it.bytes.data(), it.bytes.size());
                if (supeEngOnTrainFrame(&dst->eng, csum, (int16_t)(it.dbm - 80), 60))
                    dst->rxBuf.push_back(it.bytes);
            } else {
                supeEngOnRx(&dst->eng, it.bytes.data(), (uint16_t)it.bytes.size(),
                            (int16_t)(it.dbm - 80), 60);
            }
        }
    }
    return moved;
}

/* Wake latency, in ms: what a real task pays between a timer firing and the
 * work running — the timer task, the notify, the scheduler, the retune. Zero
 * here would model a machine this code never runs on, and a slot window sized
 * for zero is exactly the bug the bench found. */
static uint32_t g_wakeLateMs = 0;

static void fireTimers(std::vector<Node*>& nodes) {
    for (Node* n : nodes) {
        if (n->schedArmed && (int32_t)(g_now - (n->schedAt + g_wakeLateMs)) >= 0) {
            n->schedArmed = false;
            supeEngOnTimer(&n->eng);
        }
    }
}

/* Run the air and the clocks until everything is idle or `maxMs` passes. */
static uint32_t drive(std::vector<Node*> nodes, uint32_t maxMs) {
    uint32_t began = g_now, end = g_now + maxMs;
    while ((int32_t)(end - g_now) > 0) {
        bool moved = airPump(nodes);
        fireTimers(nodes);
        if (moved) continue;
        /* A timer armed at-or-before now during this very pass must fire
         * before the clock may jump. */
        bool due = false;
        for (Node* n : nodes)
            if (n->schedArmed && (int32_t)(g_now - (n->schedAt + g_wakeLateMs)) >= 0) due = true;
        if (due) continue;
        /* Jump the clock to the next armed timer's effective firing; +1 when
         * none is. */
        uint32_t next = end;
        for (Node* n : nodes) {
            uint32_t at = n->schedAt + g_wakeLateMs;
            if (n->schedArmed && (int32_t)(at - g_now) > 0 &&
                (int32_t)(next - at) > 0) next = at;
        }
        bool anyIdle = true;
        for (Node* n : nodes)
            if (!n->txq.empty()) anyIdle = false;
        if (!anyIdle) continue;
        g_now = next > g_now ? next : g_now + 1;
    }
    return g_now - began;
}

/* Drive until `watch` has closed `wantDone` meetings and every engine is
 * quiet — schedules may still be live, which is the point: the checks run at
 * the goodbye, not after the reseeded schedule has expired. */
static void driveUntilDone(std::vector<Node*> nodes, Node* watch,
                           uint32_t wantDone, uint32_t maxMs) {
    uint32_t end = g_now + maxMs;
    while ((int32_t)(end - g_now) > 0) {
        bool moved = airPump(nodes);
        fireTimers(nodes);
        if (moved) continue;
        /* Never jump the clock over a frame still in flight. */
        bool inFlight = false;
        for (Node* n : nodes)
            if (!n->txq.empty()) inFlight = true;
        if (inFlight) continue;
        bool due = false;
        for (Node* n : nodes)
            if (n->schedArmed && (int32_t)(g_now - (n->schedAt + g_wakeLateMs)) >= 0) due = true;
        if (due) continue;
        bool anyBusy = false;
        for (Node* n : nodes)
            if (n->eng.m.phase >= SUPE_M_HD_TX) anyBusy = true;
        if (!anyBusy && watch->eng.meetingsDone >= wantDone) return;
        uint32_t next = end;
        for (Node* n : nodes) {
            uint32_t at = n->schedAt + g_wakeLateMs;
            if (n->schedArmed && (int32_t)(at - g_now) > 0 &&
                (int32_t)(next - at) > 0) next = at;
        }
        if (!anyBusy && next == end) break;
        g_now = next > g_now ? next : g_now + 1;
    }
}

static void launchFrom(Node* a) {
    uint8_t v = supeEngVerdict(&a->eng);
    eqi(v, SUPE_V_OFFER, "the verdict offers");
    while (!supeEngLaunchDue(&a->eng)) g_now++;
    supeEngLaunch(&a->eng);
}

/* Wire A→B: A holds traffic for tag TAG (B's address, peer id 5); B knows A
 * by A's identity (peer id 9). */
static void wire(Node* A, Node* B) {
    memcpy(A->peerTag, TAG, 3);
    A->peer.known = true;
    A->peer.peerId = 5;
    memcpy(B->peerTag, IDENT_A, 3);
    B->peer.known = true;
    B->peer.peerId = 9;
    supeEngTagAdd(&B->eng, TAG, true, 0);
}

static void resetAir(void) {
    g_rxBusy = false;
    g_wakeLateMs = 0;
    g_dropNthTrain = 0;
    g_dropType = 0;
    g_dropNthType = 0; g_dropNth = 0; g_typeSeen = 0;
    g_dropRepairs = false;
    g_trainSeen = 0;
    g_droppedCsums.clear();
}

/* ─────────────── the tests ─────────────── */

static void testMeeting(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);

    pushPkt(&A, TAG, 5, 300, 0x11);
    pushPkt(&A, TAG, 5, 120, 0x22);

    launchFrom(&A);
    driveUntilDone(air, &A, 1, 20000);

    eqi(A.eng.m.phase, SUPE_M_IDLE, "A is home");
    eqi(B.eng.m.phase, SUPE_M_IDLE, "B is home");
    eqi(A.eng.seedsOut, 1, "one PRIVSYNC on the shared channel — and only one");
    eqi(A.eng.meetingsDone, 1, "A counts one meeting");
    eqi(B.eng.meetingsDone, 1, "B counts one meeting");
    eqi(A.eng.framesOut, 3, "a 300 B packet split: three frames out");
    eqi(B.eng.framesIn, 3, "…three frames in");
    eqi((long)B.delivered.size(), 3, "B delivered the whole train upward");
    ok(dByte(B.delivered, 0, 0) == 0x02 && dByte(B.delivered, 1, 0) == 0x12,
       "…in sequence: the split's halves are adjacent");
    eqi(loraqDepth(&A.q), 0, "A's queue was consumed on the proven close");
    eqi(A.trainsDelivered, 1, "A's train copies were released as delivered");
    ok(A.peer.detoured, "A's peer record remembers the meeting");
    ok(A.homes >= 1 && B.homes >= 1, "both radios came home");

    /* Every goodbye keys the next schedule: both ends hold a wide one now. */
    bool aWide = false, bWide = false;
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        if (A.eng.sched[i].used && A.eng.sched[i].wide) aWide = true;
        if (B.eng.sched[i].used && B.eng.sched[i].wide) bWide = true;
    }
    ok(aWide && bWide, "the final THATSIT seeded a wide schedule at both ends");
    /* The receiver of the final train transmits first on it. */
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        if (A.eng.sched[i].used && A.eng.sched[i].wide)
            ok(!A.eng.sched[i].weTx0, "A sent the final train, so B speaks first");
        if (B.eng.sched[i].used && B.eng.sched[i].wide)
            ok(B.eng.sched[i].weTx0, "…and B holds the mirror of that");
    }
    bool aReport = false, aPair = false, aOk = false;
    for (auto& nt : A.notes) {
        if (nt.ev == SUPE_EV_REPORT) aReport = true;
        if (nt.ev == SUPE_EV_PAIR) aPair = true;
        if (nt.ev == SUPE_EV_TRAIN_OK) aOk = true;
    }
    ok(aReport, "A got the GIMME's report of its own transmission");
    ok(aPair, "A filed a path-loss pair");
    ok(aOk, "A's power controller heard its train confirmed");
}

static void testReturnLeg(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);

    pushPkt(&A, TAG, 5, 150, 0x11);
    pushPkt(&B, nullptr, 9, 100, 0x33);       /* B's reply, already queued */
    pushPkt(&B, nullptr, 9, 80, 0x44);

    launchFrom(&A);
    driveUntilDone(air, &A, 1, 20000);

    eqi(A.eng.meetingsDone, 1, "one meeting carried both directions");
    eqi(B.eng.meetingsDone, 1, "…on B's count too");
    eqi(B.eng.framesOut, 2, "B's return train rode the answering HAVEDATA");
    eqi((long)A.delivered.size(), 2, "A delivered B's return frames");
    eqi(loraqDepth(&B.q), 0, "B's queue was consumed on the proven close");
    eqi(A.eng.seedsOut, 1, "the return leg cost the shared channel nothing");
}

static void testRepair(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);

    pushPkt(&A, TAG, 5, 150, 0x11);
    pushPkt(&A, TAG, 5, 150, 0x22);
    pushPkt(&A, TAG, 5, 150, 0x33);
    g_dropNthTrain = 2;                        /* the middle frame is lost */

    launchFrom(&A);
    driveUntilDone(air, &A, 1, 20000);

    eqi(B.eng.framesIn, 2, "two frames of three arrived");
    eqi(A.eng.repairsOut, 1, "the RESEND named exactly the missing one");
    eqi(B.eng.repairsIn, 1, "…and it arrived on the repair round");
    eqi((long)B.delivered.size(), 3, "delivery is whole after one repair round");
    ok(dByte(B.delivered, 0, 1) == 0x11 && dByte(B.delivered, 1, 1) == 0x22 &&
       dByte(B.delivered, 2, 1) == 0x33,
       "…and in sequence: the repaired frame took its place, not the end");
    eqi(A.eng.meetingsDone, 1, "the meeting closed as done");
}

static void testHole(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);

    pushPkt(&A, TAG, 5, 150, 0x11);
    pushPkt(&A, TAG, 5, 150, 0x22);
    pushPkt(&A, TAG, 5, 150, 0x33);
    g_dropNthTrain = 2;
    g_dropRepairs = true;                      /* the repair round is lost too */

    launchFrom(&A);
    driveUntilDone(air, &A, 1, 30000);

    eqi((long)B.delivered.size(), 2, "what one repair round cannot save is surrendered");
    ok(dByte(B.delivered, 0, 1) == 0x11 && dByte(B.delivered, 1, 1) == 0x33,
       "…the hole stays a hole and the order stays the order");
}

static void testAbsenceLadder(void) {
    resetAir();
    Node A = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A };           /* nobody is listening */
    memcpy(A.peerTag, TAG, 3);
    A.peer.known = true;
    A.peer.peerId = 5;
    A.peer.txpOpen = 2;                        /* what the evidence says B needs */
    pushPkt(&A, TAG, 5, 200, 0x11);

    int8_t seedTxp[3] = {};
    for (int attempt = 0; attempt < 3; attempt++) {
        /* Between seeds the ladder's randomised wait runs, and the expiring
         * schedule needs its timers fired to score the strike. */
        while (supeEngVerdict(&A.eng) == SUPE_V_WAIT) drive(air, 50);
        launchFrom(&A);
        /* The PRIVSYNC is the first air item. */
        seedTxp[attempt] = A.txq.front().dbm;
        drive(air, 800);                       /* the tight horizon passes in silence */
    }
    eqi(A.eng.strikes, 3, "three tight schedules expired unmet — three strikes");
    eqi(seedTxp[0], 2, "the first seed at what the evidence said");
    eqi(seedTxp[1], 8, "the second halfway to maximum (§12: more power)");
    eqi(seedTxp[2], 14, "the third at maximum");
    ok(A.peer.absentUntilMs > g_now, "the peer is absent for a minute");
    eqi(supeEngVerdict(&A.eng), SUPE_V_DROP,
        "its traffic drops rather than transmitting into the void");
    /* Any evidence of life cancels the record outright. */
    SupePeerNote alive = {};
    alive.ev = SUPE_EV_ALIVE;
    hPeerNote(&A, TAG, &alive);
    ok(A.peer.absentUntilMs == 0 && A.peer.strikes == 0,
       "evidence of life cancels absence and restores the full ladder");
}

static void testWideExpiryScoresNothing(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 150, 0x11);
    launchFrom(&A);
    driveUntilDone(air, &A, 1, 20000);
    eqi(A.eng.meetingsDone, 1, "the meeting completed");
    size_t notesBefore = A.notes.size();
    uint32_t strikesBefore = A.eng.strikes;
    drive(air, 5000);                          /* the wide schedules expire unmet */
    bool anyWide = false;
    for (int i = 0; i < SUPE_SCHED_MAX; i++)
        if (A.eng.sched[i].used) anyWide = true;
    ok(!anyWide, "the wide schedule expired at its horizon");
    eqi(A.eng.strikes, (long)strikesBefore,
        "a wide schedule expiring unmet scores nothing");
    bool struck = false;
    for (size_t i = notesBefore; i < A.notes.size(); i++)
        if (A.notes[i].ev == SUPE_EV_STRIKE) struck = true;
    ok(!struck, "…not even a note");
}

/* The wide schedule is the reply's ride (§7): a reply born a few hundred ms
 * after the goodbye meets its peer at a wide slot — no PRIVSYNC, nothing on
 * the shared channel. */
static void testWideRide(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 200, 0x11);
    launchFrom(&A);
    driveUntilDone(air, &A, 1, 20000);
    eqi(A.eng.meetingsDone, 1, "the first meeting completed");
    eqi(A.eng.seedsOut, 1, "…for one seed");

    /* The reply, born 200 ms after the goodbye. */
    g_now += 200;
    pushPkt(&B, nullptr, 9, 120, 0x33);
    driveUntilDone(air, &B, 2, 20000);
    eqi(B.eng.meetingsDone, 2, "the reply's meeting completed");
    eqi(B.eng.seedsOut, 0, "…at a wide slot: B never touched the shared channel");
    eqi((long)A.delivered.size(), 1, "A holds the reply");
    eqi((long)B.delivered.size(), 1, "B holds the first train");
}

/* The bench's own failure: a slot is reached from an idle task, so it opens
 * late — 16-20 ms of timer, notify, scheduler and retune. The window that has
 * to cover that cannot borrow its cover from the preamble, which shrinks with
 * the budget the wide schedule flies at. At zero wake latency every schedule
 * works and this passes vacuously; at the measured latency only a window sized
 * to the software's slop does. */
static void testWideRideWhenWokenLate(void) {
    /* The measured figure, not the constant under test: a test whose
     * coverage shrinks when the constant is weakened tests nothing. */
    for (uint32_t late = 0; late <= 20; late += 10) {
        resetAir();
        Node A = {}, B = {};
        nodeInit(&A, "A", SUPE_REGIME_EU863);
        nodeInit(&B, "B", SUPE_REGIME_EU863);
        std::vector<Node*> air = { &A, &B };
        wire(&A, &B);
        pushPkt(&A, TAG, 5, 200, 0x11);
        launchFrom(&A);
        driveUntilDone(air, &A, 1, 20000);
        eqi(A.eng.meetingsDone, 1, "the seeded meeting completed");

        /* Everything from here is reached from an idle task. */
        g_wakeLateMs = late;
        g_now += 200;
        pushPkt(&B, nullptr, 9, 120, 0x33);
        driveUntilDone(air, &B, 2, 20000);
        eqi(B.eng.meetingsDone, 2, "the reply met at a wide slot despite the late wake");
        eqi(B.eng.seedsOut, 0, "…without touching the shared channel");
        eqi((long)A.delivered.size(), 1, "and A holds the reply");
    }
    g_wakeLateMs = 0;
}

/* Two meetings that close the same way — same one-frame train, same power —
 * must not seed the same schedule. Without a salt a THATSIT is a type, a power
 * that rarely moves and one checksum, so ordinary sessions collide; and a
 * collision is worse than it sounds, because each end derives its role from
 * ITS OWN view of who sent the final THATSIT. Two ends holding the same-named
 * schedule from different meetings take opposite roles and both fall silent. */
static void testGoodbyesAreUnique(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);

    uint8_t seen[2][SUPE_HASH_LEN];
    for (int round = 0; round < 2; round++) {
        pushPkt(&A, TAG, 5, 90, (uint8_t)0x11);     /* the same train each time */
        launchFrom(&A);
        driveUntilDone(air, &A, (uint32_t)(round + 1), 20000);
        bool got = false;
        for (int i = 0; i < SUPE_SCHED_MAX; i++) {
            if (!A.eng.sched[i].used || !A.eng.sched[i].wide) continue;
            memcpy(seen[round], A.eng.sched[i].d.hash3, SUPE_HASH_LEN);
            got = true;
        }
        ok(got, "the goodbye seeded a wide schedule");
        drive(air, 4000);                            /* let it expire, then again */
    }
    ok(memcmp(seen[0], seen[1], SUPE_HASH_LEN) != 0,
       "two identical meetings seeded two different schedules");
}

/* A lost THATSIT must not leave the two ends disagreeing about whether a
 * schedule exists. The sender holds the frame and the receiver does not, so a
 * sender that seeds from it derives a schedule its peer has never heard of —
 * and then spends the whole horizon speaking into a node that is not
 * attending. On the bench that turned one lost frame into a dead session: six
 * slots unanswered, the shared channel, the absence ladder, absent for 60 s. */
static void testLostThatsitLeavesNoOrphanSchedule(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 120, 0x11);
    g_dropType = SUPE_T_THATSIT;               /* A's goodbye never lands */
    launchFrom(&A);
    /* Long enough for the meeting to give up, short enough that an orphan
     * schedule would still be inside its horizon — checking after it expired
     * would pass no matter what the code did. */
    drive(air, 1500);

    int aWide = 0, bWide = 0;
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        if (A.eng.sched[i].used && A.eng.sched[i].wide) aWide++;
        if (B.eng.sched[i].used && B.eng.sched[i].wide) bWide++;
    }
    eqi(aWide, 0, "the sender of a THATSIT nobody answered seeds nothing");
    eqi(bWide, 0, "…and the peer that never received it seeds nothing either");
    eqi((long)B.delivered.size(), 1, "the train that did arrive is still delivered");
    /* Both ends agreeing on no schedule is what lets the traffic seed one on
     * the shared channel straight away, rather than after a wasted horizon:
     * the unconfirmed train stays queued, and the verdict offers rather than
     * waiting on a schedule the peer has never heard of. */
    eqi(supeEngVerdict(&A.eng), SUPE_V_OFFER, "A seeds afresh rather than waiting");
}

/* A bidirectional meeting whose RETURN THATSIT is lost. The opener holds only
 * its own THATSIT — superseded the moment the answering HAVEDATA promised a
 * return train — while the listener holds the later one it sent and never had
 * answered. Neither may seed: on the bench the opener did, derived a schedule
 * the listener had never heard of, and then sat listening on agile channels
 * through three of the listener's PRIVSYNC retries before declaring it absent. */
static void testSupersededGoodbyeSeedsNothing(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 120, 0x11);
    pushPkt(&B, nullptr, 9, 90, 0x33);          /* B answers: a return leg runs */
    launchFrom(&A);
    /* Two THATSITs fly; the second — B's, closing the return train — is lost. */
    g_dropNth = 2; g_dropNthType = SUPE_T_THATSIT;
    drive(air, 1500);

    int aWide = 0, bWide = 0;
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        if (A.eng.sched[i].used && A.eng.sched[i].wide) aWide++;
        if (B.eng.sched[i].used && B.eng.sched[i].wide) bWide++;
    }
    eqi(aWide, 0, "the opener does not seed from a THATSIT already superseded");
    eqi(bWide, 0, "…nor the listener from one nothing answered");
}

/* Put one hail from A on B's receiver, seeded by `salt` so each call derives a
 * different schedule — the salt is what makes every retry of a hail a fresh
 * seed on the air. */
static void hailB(Node* B, uint8_t salt, const uint8_t* toTag = TAG,
                  const uint8_t* fromIdent = IDENT_A) {
    SupePrivsync ps = {};
    ps.regime  = SUPE_REGIME_EU863;
    ps.version = supeRegime(SUPE_REGIME_EU863)->version;
    memcpy(ps.tag, toTag, SUPE_TAG_LEN);        /* the target's own address */
    ps.pwrDbm  = 14;
    ps.salt    = salt;
    ps.haveIdent = true;
    memcpy(ps.ident, fromIdent, SUPE_TAG_LEN);
    uint8_t f[SUPE_PRIVSYNC_ID_LEN];
    size_t n = supeEncPrivsync(f, sizeof f, &ps);
    ok(n == SUPE_PRIVSYNC_ID_LEN, "the hail encodes");
    supeEngOnRx(&B->eng, f, (uint16_t)n, -44, 80);
}

/* A busy receiver defers the two acts that touch the radio and nothing else.
 * Expiry and the walk past dead slots are bookkeeping, and the next-event time
 * is computed from both: leave them frozen and an overdue slot reports "now"
 * for as long as the reception lasts, so the deadline pins at zero, the main
 * loop spins on it, and the horizon never retires the schedule. On the bench
 * that read as 500 zero-deadline passes in 49 ms and a 400 ms schedule still
 * live 1.8 s later. */
static void testBusyReceiverStillRetiresSchedules(void) {
    resetAir();
    Node B = {};
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    memcpy(B.peerTag, IDENT_A, 3);
    B.peer.known = true;
    B.peer.peerId = 9;
    supeEngTagAdd(&B.eng, TAG, true, 0);
    hailB(&B, 0x77);                     /* one hail: B holds a tight schedule */
    int held = 0;
    for (int i = 0; i < SUPE_SCHED_MAX; i++) if (B.eng.sched[i].used) held++;
    eqi(held, 1, "B holds a schedule from the hail");

    g_rxBusy = true;                     /* the modem is mid-frame from here on */
    uint32_t stop = g_now + 3000;        /* well past the tight horizon */
    while ((int32_t)(stop - g_now) > 0) { supeEngOnTimer(&B.eng); g_now += 5; }

    int live = 0;
    for (int i = 0; i < SUPE_SCHED_MAX; i++)
        if (B.eng.sched[i].used && !B.eng.sched[i].consumed) live++;
    eqi(live, 0, "the horizon still retires a schedule while the receiver is busy");
    uint32_t at = supeEngNextEventMs(&B.eng, g_now);
    ok(at == UINT32_MAX || (int32_t)(at - g_now) > 0,
       "…and the engine asks for no wake at-or-before now");
}

/* One tight schedule per peer. A hail retried is a hail whose schedule went
 * unanswered, so that schedule is dead to both ends — held alongside the new
 * one its slots are appointments nobody keeps, and the two sets interleave
 * until each retune arrives late for the other. The salt makes every retry a
 * fresh seed, which is exactly why the identical-hash rule cannot catch it. */
static void testHailRetryReplacesSchedule(void) {
    resetAir();
    Node B = {};
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    memcpy(B.peerTag, IDENT_A, 3);
    B.peer.known = true;
    B.peer.peerId = 9;
    supeEngTagAdd(&B.eng, TAG, true, 0);

    for (int retry = 0; retry < 3; retry++) {
        hailB(&B, (uint8_t)(0x10 + retry));     /* each retry a different seed */
        g_now += 40;                            /* inside the tight horizon */
    }
    int tight = 0;
    for (int i = 0; i < SUPE_SCHED_MAX; i++)
        if (B.eng.sched[i].used && !B.eng.sched[i].wide) tight++;
    eqi(tight, 1, "three hails from one peer leave one tight schedule");
}

/* A train that went unconfirmed reports the configuration it failed at. The
 * far end weighs the loss the peer measured against what that regime needs and
 * raises the power only when the margin was thin; a note with no configuration
 * reads as a receiver needing 0 dBm, against which every real signal looks weak
 * and every unanswered close is scored as too little power. On the bench that
 * ratcheted a table-top pair from −9 dBm to 22 dBm over four minutes, each side
 * climbing because the other had. */
static void testTrainLostCarriesItsConfig(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 120, 0x11);
    g_dropType = SUPE_T_BYE;                  /* A's close is never answered */
    launchFrom(&A);
    drive(air, 4000);

    const SupePeerNote* lost = nullptr;
    for (const SupePeerNote& n : A.notes)
        if (n.ev == SUPE_EV_TRAIN_LOST) lost = &n;
    ok(lost != nullptr, "an unconfirmed train is reported lost");
    if (lost) {
        ok(lost->cfg.bwHz != 0, "…and the note names the bandwidth it flew at");
        ok(lost->cfg.sf != 0, "…and the spreading factor");
        /* The whole point of carrying it: a real sensitivity, not the 0 dBm a
         * blank configuration collapses to. */
        ok(supeSensitivityDeci(&lost->cfg) < -900,
           "…so the regime's sensitivity is a real figure");
    }
}

/* An orphaned wide schedule is abandoned for the shared channel rather than
 * spent. The last frame of any handshake cannot be acknowledged, so a lost
 * answer will always be able to leave one end holding a schedule the other
 * never derived — that is not designable-away, only make-it-cheap. Here the
 * peer simply never attends: the node must stop asking after a couple of
 * unanswered slots and offer a fresh seed, not speak into three seconds of
 * silence first. */
/* A hail outranks a rendezvous. A node holding a wide schedule that is hailed
 * must attend the hail: that schedule was paid for on the shared channel by a
 * peer with traffic in hand and lasts a few hundred milliseconds, while the
 * rendezvous is a standing appointment that may be empty at both ends. Taken in
 * array order the rendezvous wins about half the time and its retunes land the
 * node late for every slot of the hail — which is what the bench showed, three
 * own slots late out of three, followed by the peer giving up. */
static void testHailOutranksRendezvous(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 90, 0x11);
    launchFrom(&A);
    driveUntilDone(air, &A, 1, 20000);        /* both ends now hold a rendezvous */

    int wideA = 0, tightA = 0;
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        if (!A.eng.sched[i].used) continue;
        if (A.eng.sched[i].wide) wideA++; else tightA++;
    }
    eqi(wideA, 1, "A holds the rendezvous the goodbye seeded");

    /* A hail lands on top of it. Both are live; the tight one must be served. */
    supeEngTagAdd(&A.eng, IDENT_A, true, 0);   /* A's own address, so it is for A */
    hailB(&A, 0x5a, IDENT_A, IDENT_B);
    tightA = 0;
    for (int i = 0; i < SUPE_SCHED_MAX; i++)
        if (A.eng.sched[i].used && !A.eng.sched[i].wide) tightA++;
    eqi(tightA, 1, "…and the hail installs a tight one beside it");

    /* Line the two up so their first slots fall at the same instant — the
     * contended case, which chance alone produces only now and then. The
     * rendezvous sits earlier in the table, so array order would take it. */
    SupeSched* tight = nullptr; SupeSched* wideS = nullptr;
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        SupeSched* s = &A.eng.sched[i];
        if (!s->used) continue;
        if (s->wide) { if (!wideS) wideS = s; } else if (!tight) tight = s;
    }
    ok(tight && wideS, "both kinds are live");
    ok(wideS && tight && wideS < tight, "…with the rendezvous found first");
    if (!tight || !wideS) return;
    uint32_t both = tight->epochMs + tight->d.slot[0].tMs;
    wideS->epochMs = both - wideS->d.slot[0].tMs;

    g_now = both;
    supeEngOnTimer(&A.eng);
    eqi(tight->nextSlot, 1, "the hail's slot is the one taken");
    eqi(wideS->nextSlot, 0, "…and the rendezvous still has its slot to come");
}

static void testOrphanWideScheduleIsAbandoned(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 90, 0x11);
    launchFrom(&A);
    driveUntilDone(air, &A, 1, 20000);        /* a real meeting, a real goodbye */
    int wide = 0;
    for (int i = 0; i < SUPE_SCHED_MAX; i++)
        if (A.eng.sched[i].used && A.eng.sched[i].wide) wide++;
    eqi(wide, 1, "the goodbye seeded a wide schedule");

    /* B stops attending — the shape a lost final answer leaves behind, without
     * needing to lose one: A holds the schedule and nobody else derived it. */
    std::vector<Node*> alone = { &A };
    pushPkt(&A, TAG, 5, 90, 0x22);            /* traffic that wants the schedule */

    uint32_t stop = g_now + SUPE_WIDE_HORIZON_MS;
    while ((int32_t)(stop - g_now) > 0 && A.eng.schedsAbandoned == 0) {
        airPump(alone);
        fireTimers(alone);
        g_now += 5;
    }
    eqi((long)A.eng.schedsAbandoned, 1, "the unanswered wide schedule is abandoned");
    ok((int32_t)(stop - g_now) > 0, "…well inside the horizon it would have spent");
    int live = 0;
    for (int i = 0; i < SUPE_SCHED_MAX; i++)
        if (A.eng.sched[i].used && !A.eng.sched[i].consumed) live++;
    eqi(live, 0, "…and nothing is left holding the traffic back");
    eqi(supeEngVerdict(&A.eng), SUPE_V_OFFER,
        "…so the traffic seeds afresh on the shared channel");
}

static void testSeedHashGate(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 150, 0x11);
    launchFrom(&A);
    /* Pump the PRIVSYNC across, then corrupt A's next frame's hash. */
    airPump(air);
    /* B holds a listening schedule now. Drive to just before slot 0 and let
     * A's HAVEDATA fly with a poisoned hash byte. */
    drive(air, 20);
    fireTimers(air);
    /* A is at HD_TX or beyond; intercept its HAVEDATA on the air. */
    bool poisoned = false;
    uint32_t end = g_now + 2000;
    while ((int32_t)(end - g_now) > 0) {
        for (Node* n : air)
            if (n == &A && !n->txq.empty() && !poisoned &&
                n->txq.front().bytes[0] == SUPE_T_HAVEDATA) {
                n->txq.front().bytes[1] ^= 0xFF;
                poisoned = true;
            }
        bool moved = airPump(air);
        fireTimers(air);
        if (!moved) g_now++;
        if (poisoned && B.eng.rxForeign) break;
    }
    ok(poisoned, "the HAVEDATA was poisoned on the air");
    ok(B.eng.rxForeign >= 1, "a mismatched seed hash dies in one frame");
    eqi(B.eng.meetingsDone, 0, "…and no meeting follows it");
}

static void testVerdicts(void) {
    resetAir();
    Node A = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    memcpy(A.peerTag, TAG, 3);
    A.peer.known = true;
    A.peer.peerId = 5;
    pushPkt(&A, TAG, 5, 100, 0x11);
    eqi(supeEngVerdict(&A.eng), SUPE_V_OFFER, "a known peer draws an offer");
    launchFrom(&A);
    std::vector<Node*> air = { &A };
    airPump(air);                              /* the seed is out; a schedule lives */
    eqi(supeEngVerdict(&A.eng), SUPE_V_WAIT,
        "a live schedule holds the packet for its slot");
    drive(air, 800);                           /* horizon passes */
    /* Unknown peer: plain, exactly as with the feature off. */
    Node C = {};
    nodeInit(&C, "C", SUPE_REGIME_EU863);
    pushPkt(&C, TAG, LORAQ_PEER_NONE, 100, 0x11);
    eqi(supeEngVerdict(&C.eng), SUPE_V_PLAIN, "an unknown peer flies plainly");
}

/* Deliver exactly one frame off the air — unlike airPump, which services every
 * node's queue in one pass and so completes a turnaround answer's tx in the
 * same call that provoked it. Mid-flight states are invisible to it, and the
 * stale-deadline storm below lives precisely there. */
static bool pumpOne(std::vector<Node*>& nodes) {
    for (Node* src : nodes) {
        if (src->txq.empty()) continue;
        AirItem it = src->txq.front();
        src->txq.erase(src->txq.begin());
        uint32_t key = chanKey(src);
        supeEngOnTxDone(&src->eng, true);
        for (Node* dst : nodes) {
            if (dst == src || chanKey(dst) != key) continue;
            if (it.isTrain) {
                uint8_t csum = supeCrc8(it.bytes.data(), it.bytes.size());
                if (supeEngOnTrainFrame(&dst->eng, csum, (int16_t)(it.dbm - 80), 60))
                    dst->rxBuf.push_back(it.bytes);
            } else {
                supeEngOnRx(&dst->eng, it.bytes.data(), (uint16_t)it.bytes.size(),
                            (int16_t)(it.dbm - 80), 60);
            }
        }
        return true;
    }
    return false;
}

/* A frame in flight is the one state whose clock is the tx completion and
 * nothing else. A deadline left standing there — the listen window's, on the
 * side that just answered with GIMME — re-arms the host timer at zero delay on
 * every firing, and on the device that storm outranks the radio task that
 * would deliver the tx-done ending it. */
static void testNoStaleDeadlineInTxPhase(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 200, 0x11);
    launchFrom(&A);

    uint32_t guard = g_now + 20000;
    while (B.eng.m.phase != SUPE_M_GIMME_TX && (int32_t)(guard - g_now) > 0) {
        if (pumpOne(air)) continue;
        bool due = false;
        for (Node* n : air)
            if (n->schedArmed && (int32_t)(g_now - n->schedAt) >= 0) due = true;
        if (due) { fireTimers(air); continue; }
        uint32_t next = guard;
        for (Node* n : air)
            if (n->schedArmed && (int32_t)(n->schedAt - g_now) > 0 &&
                (int32_t)(next - n->schedAt) > 0) next = n->schedAt;
        g_now = next > g_now ? next : g_now + 1;
    }
    eqi(B.eng.m.phase, SUPE_M_GIMME_TX, "B's GIMME is on the air");
    eqi((long)B.eng.m.deadlineMs, 0, "no deadline stands while a tx is in flight");
    uint32_t at = supeEngNextEventMs(&B.eng, g_now);
    ok(at == UINT32_MAX || (int32_t)(at - g_now) > 0,
       "the engine asks for no wake at-or-before now");
    for (int i = 0; i < 5; i++) supeEngOnTimer(&B.eng);
    eqi(B.eng.m.phase, SUPE_M_GIMME_TX, "hammering the timer changes nothing");
    ok(!B.schedArmed || (int32_t)(B.schedAt - g_now) > 0,
       "…and never re-arms it at or before now");

    driveUntilDone(air, &A, 1, 20000);
    eqi(A.eng.meetingsDone, 1, "the meeting still completes");
}

int main(void) {
    testMeeting();
    testReturnLeg();
    testRepair();
    testHole();
    testAbsenceLadder();
    testWideExpiryScoresNothing();
    testWideRide();
    testWideRideWhenWokenLate();
    testGoodbyesAreUnique();
    testLostThatsitLeavesNoOrphanSchedule();
    testSupersededGoodbyeSeedsNothing();
    testBusyReceiverStillRetiresSchedules();
    testHailRetryReplacesSchedule();
    testTrainLostCarriesItsConfig();
    testHailOutranksRendezvous();
    testOrphanWideScheduleIsAbandoned();
    testSeedHashGate();
    testVerdicts();
    testNoStaleDeadlineInTxPhase();
    printf("%d checks, %d failed\n", g_run, g_fail);
    return g_fail ? 1 : 0;
}
