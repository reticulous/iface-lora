/**
 * supe_engine_test — the engine plus a stub host is a program that steps whole
 * transactions on a laptop, which is the point of SupeHost: nothing in the
 * engine names a radio, a task or a timer, so this file supplies all three as
 * plain data and a loop.
 *
 * What it steps: full bidirectional transactions, a refusal, the absence
 * ladder, the reverse flag's no-frame ending, the crossfire of two requesters
 * sharing one link tag, the third-party hold, and the no-waiting contract —
 * a whole exchange is turnarounds only.
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

/* ─────────────── the stub host ─────────────── */

static uint32_t g_now = 1000;
static uint32_t g_rand = 12345;

struct AirItem {
    bool     isPacket;
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
    uint32_t absentUntilMs = 0, retryWaitUntilMs = 0, backoffUntilMs = 0;
    bool     detoured = false;
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
    uint32_t tunes = 0, homes = 0;
    /* one-shot timer */
    bool     schedArmed = false;
    uint32_t schedAt = 0;
    /* what went on the air, awaiting tx-done + delivery */
    std::vector<AirItem> txq;
    /* the host's peer table, one tag */
    uint8_t  peerTag[3] = {};
    StubPeer peer;
    std::vector<SupePeerNote> notes;
    SupeChanView chans = {};
    uint32_t framesSent = 0;
    std::vector<uint8_t> stagedPkt;
    int8_t  stagedDbm = 0;
    bool    hasStaged = false;
};

static uint32_t chanKey(const Node* n) {
    if (n->home) return 0;
    return 1000u + n->chan * 20u + n->cfg.sf + n->cfg.bwHz / 1000u;
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
     * for here (the transaction id). FNV-1a folded across the output. */
    uint32_t h = 2166136261u;
    for (uint16_t i = 0; i < n; i++) { h ^= d[i]; h *= 16777619u; }
    for (int i = 0; i < 32; i++) { out[i] = (uint8_t)(h >> ((i % 4) * 8)); h = h * 31 + 7; }
}
static bool hTune(void* c, uint8_t chan, const SupeCfg* cfg, uint8_t) {
    Node* n = (Node*)c;
    n->home = false;
    n->chan = chan;
    n->cfg = *cfg;
    n->tunes++;
    return true;
}
static void hTuneHome(void* c) {
    Node* n = (Node*)c;
    n->home = true;
    n->homes++;
}
static bool hTxFrame(void* c, const uint8_t* f, uint16_t len, int8_t dbm) {
    Node* n = (Node*)c;
    n->framesSent++;
    n->txq.push_back({ false, std::vector<uint8_t>(f, f + len), dbm });
    return true;
}
/* The stub's stage/fire pipeline: stage parks one packet, fire puts it on
 * the air — mirroring the device's prebuilt-frames pop. */
static bool hStagePacket(void* c, const LoraPkt* p, int8_t dbm) {
    Node* n = (Node*)c;
    n->stagedPkt.assign(p->bytes, p->bytes + p->len);
    n->stagedDbm = dbm;
    n->hasStaged = true;
    return true;
}

static bool hFireStaged(void* c) {
    Node* n = (Node*)c;
    if (!n->hasStaged) return false;
    n->txq.push_back({ true, n->stagedPkt, n->stagedDbm });
    n->hasStaged = false;
    return true;
}
static void hRx(void*) {}
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
    out->backoffUntilMs = n->peer.backoffUntilMs;
    out->detoured = n->peer.detoured;
    return true;
}
static void hPeerNote(void* c, const uint8_t tag[3], const SupePeerNote* nt) {
    Node* n = (Node*)c;
    n->notes.push_back(*nt);
    if (memcmp(tag, n->peerTag, 3) != 0) return;
    switch (nt->ev) {
        case SUPE_EV_ALIVE:
            n->peer.strikes = 0;
            n->peer.absentUntilMs = 0;
            n->peer.retryWaitUntilMs = 0;
            break;
        case SUPE_EV_STRIKE:
            n->peer.strikes++;
            n->peer.retryWaitUntilMs = g_now + nt->backoffMs;
            if (n->peer.strikes >= 3) n->peer.absentUntilMs = g_now + 60000;
            break;
        case SUPE_EV_REFUSED:
            n->peer.backoffUntilMs = g_now + nt->backoffMs;
            n->peer.strikes = 0;
            break;
        case SUPE_EV_DETOURED:
            n->peer.detoured = true;
            break;
        default:
            break;
    }
}
static void hChanGet(void* c, SupeChanView* out) { *out = ((Node*)c)->chans; }
static void hLog(void* c, const char* msg) {
    if (getenv("VERBOSE")) printf("  [%s] %s\n", ((Node*)c)->name, msg);
}

/* Each node's own identity, which its STARTs carry. The answering side has
 * nothing else to name a requester by: the tag a START asks about is one of the
 * ANSWERER's addresses, so resolving it says nothing about who is asking. */
static const uint8_t IDENT_A[3] = { 0xa1, 0xa2, 0xa3 };
static const uint8_t IDENT_B[3] = { 0xb1, 0xb2, 0xb3 };

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
    n->host.stage_packet = hStagePacket;
    n->host.fire_staged = hFireStaged;
    n->host.rx = hRx;
    n->host.peer_get = hPeerGet;
    n->host.peer_note = hPeerNote;
    n->host.chan_get = hChanGet;
    n->host.log = hLog;
    n->host.dbgLevel = true;
    loraqInit(&n->q);
    supeEngInit(&n->eng, &n->host, &n->q);
    supeEngConfig(&n->eng, regime, SUPE_FAM_SX126X, 8, 14, true,
                  7, 125000, 5, 12, 0x42, 254);
    supeEngSetIdent(&n->eng, name[0] == 'A' ? IDENT_A : IDENT_B);
    n->chans = {};
    n->chans.nChans = (regime == SUPE_REGIME_EU863) ? 9 : 0;
    n->chans.anyBudget = true;
    for (int c = 1; c <= n->chans.nChans; c++) n->chans.usable[c] = 1;
}

static void pushPkt(Node* n, const uint8_t tag[3], uint16_t peerId, uint16_t len) {
    uint8_t* b = (uint8_t*)malloc(len);
    memset(b, 0xA5, len);
    ok(loraqPush(&n->q, b, len, g_now, peerId, tag, 1, LORAQ_ORIG_RNSD),
       "test packet enqueues");
}

/* ─────────────── the air ─────────────── */

/* One pass: every node's oldest transmission completes (tx-done) and lands on
 * every other node sharing its channel. Frames whose first byte is a SUPE type
 * go to the engine's rx; packet bodies count as train packets. */
static bool airPump(std::vector<Node*>& nodes) {
    bool moved = false;
    for (Node* src : nodes) {
        if (src->txq.empty()) continue;
        moved = true;
        AirItem it = src->txq.front();
        src->txq.erase(src->txq.begin());
        uint32_t key = chanKey(src);
        supeEngOnTxDone(&src->eng, true);
        for (Node* dst : nodes) {
            if (dst == src || chanKey(dst) != key) continue;
            if (it.isPacket)
                supeEngOnPacketRx(&dst->eng, (int16_t)(it.dbm - 80), 60);
            else
                supeEngOnRx(&dst->eng, it.bytes.data(), (uint16_t)it.bytes.size(),
                            (int16_t)(it.dbm - 80), 60);
        }
    }
    return moved;
}

static void fireTimers(std::vector<Node*>& nodes) {
    for (Node* n : nodes) {
        if (n->schedArmed && (int32_t)(g_now - n->schedAt) >= 0) {
            n->schedArmed = false;
            supeEngOnTimer(&n->eng);
        }
    }
}

/* Run the air and the clocks until everything is idle or `maxMs` passes. */
/* Run until every engine is idle (or `maxMs` passes) and return the elapsed
 * stub time. Unlike drive(), this does not wait out stale deadline timers a
 * finished transaction left armed — they fire into IDLE and are ignored, and
 * counting them would misread bookkeeping as waiting. */
static uint32_t driveUntilIdle(std::vector<Node*> nodes, uint32_t maxMs) {
    uint32_t began = g_now, end = g_now + maxMs;
    auto allIdle = [&]() {
        for (Node* n : nodes)
            if (n->eng.x.phase != SUPE_X_IDLE || !n->txq.empty()) return false;
        return true;
    };
    while (!allIdle() && (int32_t)(end - g_now) > 0) {
        bool moved = airPump(nodes);
        fireTimers(nodes);
        if (!moved) g_now += 1;
    }
    return g_now - began;
}

static void drive(std::vector<Node*> nodes, uint32_t maxMs) {
    uint32_t end = g_now + maxMs;
    while ((int32_t)(end - g_now) > 0) {
        bool moved = airPump(nodes);
        fireTimers(nodes);
        /* The platform's half of a retransmission: on a real node supePoll puts
         * it back through channel access, which this harness has none of, so it
         * goes out the moment it is armed. */
        for (Node* n : nodes)
            if (supeEngResendDue(&n->eng)) { supeEngResend(&n->eng); moved = true; }
        if (!moved) {
            bool anyBusy = false, anySched = false;
            for (Node* n : nodes) {
                if (!n->txq.empty()) anyBusy = true;
                if (n->schedArmed) anySched = true;
            }
            if (!anyBusy && !anySched) return;
            g_now += 1;
        }
    }
}

/* ─────────────── the transactions ─────────────── */

static const uint8_t TAG[3] = { 0xd1, 0x0d, 0x51 };

static void launchFrom(Node* a) {
    uint8_t v = supeEngVerdict(&a->eng);
    eqi(v, SUPE_V_OFFER, "the verdict offers");
    while (!supeEngLaunchDue(&a->eng)) g_now++;
    supeEngLaunch(&a->eng);
}

static void testBidirectional(void) {
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };

    /* A knows the peer behind the tag; B holds the tag (it is B's address)
     * and can identify the requester's reverse traffic (a link-id tag). */
    memcpy(A.peerTag, TAG, 3);
    A.peer.known = true;
    A.peer.peerId = 5;
    memcpy(B.peerTag, IDENT_A, 3);      /* B knows A by A's identity */
    B.peer.known = true;
    B.peer.peerId = 9;
    supeEngTagAdd(&B.eng, TAG, true, 0);

    pushPkt(&A, TAG, 5, 300);
    pushPkt(&A, TAG, 5, 300);
    pushPkt(&A, TAG, 5, 120);
    pushPkt(&B, nullptr, 9, 200);      /* B's reverse traffic for the requester */
    pushPkt(&B, nullptr, 9, 80);

    launchFrom(&A);
    drive(air, 20000);

    eqi(A.eng.x.phase, SUPE_X_IDLE, "A came home");
    eqi(B.eng.x.phase, SUPE_X_IDLE, "B came home");
    eqi(loraqDepth(&A.q), 0, "A's train consumed its queue");
    eqi(loraqDepth(&B.q), 0, "B's train consumed its queue");
    eqi(A.eng.detoursDone, 1, "A counts one detour");
    eqi(B.eng.detoursDone, 1, "B counts one detour");
    eqi(A.eng.trainPktsOut, 3, "A sent three packets");
    eqi(B.eng.trainPktsIn, 3, "B counted three packets");
    eqi(B.eng.trainPktsOut, 2, "B sent two packets back");
    eqi(A.eng.trainPktsIn, 2, "A counted two packets back");
    ok(A.homes >= 1 && B.homes >= 1, "both radios returned to the hailing channel");
    ok(A.peer.detoured, "A's peer record remembers the completed detour");
    bool aPair = false, aTrainOk = false;
    for (auto& nt : A.notes) {
        if (nt.ev == SUPE_EV_PAIR) aPair = true;
        if (nt.ev == SUPE_EV_TRAIN_OK) aTrainOk = true;
    }
    ok(aPair, "A filed a path-loss pair");
    ok(aTrainOk, "A's power controller heard its train confirmed");
    bool bCaps = false;
    for (auto& nt : B.notes) if (nt.ev == SUPE_EV_CAPS) bCaps = true;
    ok(bCaps, "B was handed the requester's MANIFEST capabilities to file");
}

static void testRefusal(void) {
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    memcpy(A.peerTag, TAG, 3);
    A.peer.known = true;
    memcpy(B.peerTag, TAG, 3);
    supeEngTagAdd(&B.eng, TAG, true, 0);
    /* B is out of airtime on every channel. */
    for (int c = 0; c < SUPE_CH_MAX; c++) B.chans.usable[c] = 0;
    B.chans.anyBudget = false;

    pushPkt(&A, TAG, LORAQ_PEER_NONE, 200);
    launchFrom(&A);
    drive(air, 5000);

    eqi(A.eng.refusalsIn, 1, "A heard the refusal");
    eqi(B.eng.refusalsOut, 1, "B sent it");
    bool refused = false;
    for (auto& nt : A.notes)
        if (nt.ev == SUPE_EV_REFUSED && nt.reason == SUPE_REFUSE_AIRTIME) refused = true;
    ok(refused, "the reason travelled: out of airtime");
    ok(A.peer.backoffUntilMs > g_now, "the backoff was filed");
    eqi(supeEngVerdict(&A.eng), SUPE_V_PLAIN,
        "the packet flies plainly, once, without being re-asked");
    eqi(A.eng.x.phase, SUPE_X_IDLE, "A never left the main channel");
}

static void testAbsenceLadder(void) {
    Node A = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A };    /* nobody is listening */
    memcpy(A.peerTag, TAG, 3);
    A.peer.known = true;
    pushPkt(&A, TAG, LORAQ_PEER_NONE, 200);

    int8_t txp[3] = {};
    uint8_t ceil[3] = {};
    for (int attempt = 0; attempt < 3; attempt++) {
        /* Wait out the ladder's randomised interval, then re-request. */
        /* WAIT, not HOLD: the ladder's pause is our own timing and reserves
         * nothing — a real node contends underneath it. */
        while (supeEngVerdict(&A.eng) == SUPE_V_WAIT) g_now += 50;
        launchFrom(&A);
        txp[attempt] = A.txq.back().dbm;
        ceil[attempt] = (uint8_t)(A.txq.back().bytes[5] & 0x0F);
        drive(air, 2000);               /* the GRANT deadline passes in silence */
    }
    eqi(A.eng.strikes, 3, "three requests drew three silences");
    ok(txp[1] >= txp[0] && txp[2] >= txp[1], "each retry went out at more power");
    eqi(txp[2], 14, "the third at the configured maximum");
    ok(ceil[1] <= ceil[0], "each retry asked for a lower ceiling");
    eqi(ceil[2], 0, "the third asked for budget 0");
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

static void testCountZeroClose(void) {
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    memcpy(A.peerTag, TAG, 3);
    A.peer.known = true;
    supeEngTagAdd(&B.eng, TAG, true, 0);
    /* B cannot name the requester (no peer record): its GRANT declares no
     * reverse traffic, and the transaction ends on the train with no frame. */
    pushPkt(&A, TAG, LORAQ_PEER_NONE, 250);
    launchFrom(&A);
    drive(air, 10000);

    eqi(A.eng.x.phase, SUPE_X_IDLE, "A came home on its own last frame");
    eqi(B.eng.x.phase, SUPE_X_IDLE, "B went home on the count");
    eqi(B.eng.trainPktsIn, 1, "B took the one packet");
    eqi(B.eng.trainPktsOut, 0, "B had nothing to say back");
    eqi(B.framesSent, 1, "B transmitted only the GRANT — no close frame");
    eqi(A.eng.detoursDone, 1, "the detour still completed");
}

/* No waiting anywhere when things go right: an answering side with nothing
 * queued declares so in its GRANT and goes home on the train's end, and the
 * whole exchange is back-to-back turnarounds. The stub's air is instant, so
 * the wall clock of a complete bidirectional detour is just the retune gap,
 * the manifest lead and the packet gaps: a couple of tens of milliseconds,
 * not hundreds. */
static void testNoWaiting(void) {
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    memcpy(A.peerTag, TAG, 3);
    A.peer.known = true;
    memcpy(B.peerTag, IDENT_A, 3);  /* B knows A by A's identity, not by its own tag */
    B.peer.known = true;
    B.peer.peerId = 9;              /* B could answer — it just has nothing */
    supeEngTagAdd(&B.eng, TAG, true, 0);
    pushPkt(&A, TAG, LORAQ_PEER_NONE, 250);

    launchFrom(&A);
    uint32_t took = driveUntilIdle(air, 10000);

    eqi(A.eng.x.phase, SUPE_X_IDLE, "A came home");
    eqi(B.eng.x.phase, SUPE_X_IDLE, "B came home on the train's end");
    eqi(B.eng.trainPktsIn, 1, "B took the packet");
    eqi(B.eng.trainPktsOut, 0, "B had nothing to say back");
    eqi(B.framesSent, 1, "B transmitted exactly one frame: the GRANT — no close");
    eqi(A.eng.detoursDone, 1, "the detour completed");
    eqi(B.eng.detoursDone, 1, "on both sides");
    ok(took < 60, "no timed wait anywhere: the exchange is turnarounds only");

    /* With the reverse leg buffered BEFORE the transaction — the ping-pong
     * case — it rides the same detour with no wait either. */
    pushPkt(&A, TAG, LORAQ_PEER_NONE, 250);
    pushPkt(&B, nullptr, 9, 80);    /* the proof of the previous packet */
    while (supeEngVerdict(&A.eng) != SUPE_V_OFFER) g_now += 10;
    while (!supeEngLaunchDue(&A.eng)) g_now++;
    supeEngLaunch(&A.eng);
    took = driveUntilIdle(air, 10000);
    eqi(B.eng.trainPktsOut, 1, "the buffered proof rode the same detour");
    eqi(A.eng.trainPktsIn, 1, "…and A collected it");
    ok(took < 60, "the round trip is still turnarounds only");
}

static void testThirdPartyHold(void) {
    Node A = {}, B = {}, C = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    nodeInit(&C, "C", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B, &C };
    memcpy(A.peerTag, TAG, 3);
    A.peer.known = true;
    supeEngTagAdd(&B.eng, TAG, true, 0);
    memcpy(C.peerTag, TAG, 3);
    C.peer.known = true;
    pushPkt(&A, TAG, LORAQ_PEER_NONE, 250);
    pushPkt(&C, TAG, LORAQ_PEER_NONE, 100);   /* C wants the same tag */

    launchFrom(&A);
    /* Let the START and the GRANT fly; C is in earshot of both. */
    airPump(air);
    airPump(air);
    ok(C.eng.holdsTaken >= 1, "a third party held on the GRANT");
    eqi(supeEngVerdict(&C.eng), SUPE_V_HOLD,
        "its own traffic for the tag waits out the detour");
    /* The hold releases one fixed interval early, so the polite node's DIFS
     * runs while the tail of the hold is still idle (SUPE.md §6). */
    uint32_t until = 0;
    for (int i = 0; i < SUPE_HOLD_MAX; i++)
        if (C.eng.hold[i].used) until = C.eng.hold[i].untilMs;
    ok(until > g_now, "the hold has a deadline");
    C.eng.holdEarlyMs = 40;
    uint32_t save = g_now;
    g_now = until - 20;                 /* inside the early-release window */
    ok(supeEngVerdict(&C.eng) != SUPE_V_HOLD,
       "the fixed interval runs through the hold's idle tail");
    ok(supeEngHeld(&C.eng, TAG), "…while the hold itself still stands");
    g_now = save;
    C.eng.holdEarlyMs = 0;
    drive(air, 15000);
    eqi(A.eng.x.phase, SUPE_X_IDLE, "the pair finished despite the audience");
}

/* The bench case that killed the rnsh handshake: a link identifier means us
 * at BOTH endpoints, both sides queue traffic tagged with it, and both request
 * at once. The STARTs cross while each side is deaf in its own wait. */
static void testLinkCrossfire(void) {
    Node X = {}, Y = {};
    nodeInit(&X, "X", SUPE_REGIME_EU863);
    nodeInit(&Y, "Y", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &X, &Y };
    memcpy(X.peerTag, TAG, 3);
    X.peer.known = true;  X.peer.peerId = 5;
    memcpy(Y.peerTag, TAG, 3);
    Y.peer.known = true;  Y.peer.peerId = 9;
    supeEngTagAdd(&X.eng, TAG, false, 600000);
    supeEngTagAdd(&Y.eng, TAG, false, 600000);
    pushPkt(&X, TAG, 5, 200);
    pushPkt(&Y, TAG, 9, 200);

    /* Both launch before either START lands. */
    ok(supeEngVerdict(&X.eng) == SUPE_V_OFFER, "X offers");
    ok(supeEngVerdict(&Y.eng) == SUPE_V_OFFER, "Y offers");
    while (!supeEngLaunchDue(&X.eng)) g_now++;
    supeEngLaunch(&X.eng);
    while (!supeEngLaunchDue(&Y.eng)) g_now++;
    supeEngLaunch(&Y.eng);
    drive(air, 30000);

    /* Whatever the interleaving, neither side may wedge, and neither may talk
     * itself into an absence verdict about a peer that is demonstrably there
     * and transmitting. */
    eqi(X.eng.x.phase, SUPE_X_IDLE, "X is idle after the crossfire");
    eqi(Y.eng.x.phase, SUPE_X_IDLE, "Y is idle after the crossfire");
    ok(X.peer.absentUntilMs == 0, "X did not declare Y absent");
    ok(Y.peer.absentUntilMs == 0, "Y did not declare X absent");

    /* And the traffic must eventually move: keep driving through the retry
     * waits until both queues drain (plain fallback counts as moving — the
     * stub cannot transmit plainly, so here it must be by detour). */
    uint32_t guard = g_now + 120000;
    while ((loraqDepth(&X.q) || loraqDepth(&Y.q)) && (int32_t)(guard - g_now) > 0) {
        uint8_t vx = supeEngVerdict(&X.eng);
        uint8_t vy = supeEngVerdict(&Y.eng);
        if (supeEngLaunchDue(&X.eng)) supeEngLaunch(&X.eng);
        if (supeEngLaunchDue(&Y.eng)) supeEngLaunch(&Y.eng);
        (void)vx; (void)vy;
        airPump(air);
        fireTimers(air);
        g_now += 1;
    }
    eqi(loraqDepth(&X.q), 0, "X's packet moved despite the crossfire");
    eqi(loraqDepth(&Y.q), 0, "Y's packet moved despite the crossfire");
}

/* A proof takes the channel like anything else. The hold that used to keep one
 * waiting for a ride cost the far end its send window for the whole wait, and
 * the ride it waited for takes it as reverse cargo regardless. */
static void testProofGoesOut(void) {
    Node A = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    memcpy(A.peerTag, TAG, 3);
    A.peer.known = true;
    A.peer.peerId = 5;

    uint8_t* b = (uint8_t*)malloc(60);
    memset(b, 0xA5, 60);
    ok(loraqPush(&A.q, b, 60, g_now, 5, TAG, 1, LORAQ_ORIG_RNSD), "proof enqueues");
    eqi(supeEngVerdict(&A.eng), SUPE_V_OFFER, "a lone proof offers at once");
    loraqConsume(&A.q, 0);
    eqi(supeEngVerdict(&A.eng), SUPE_V_PLAIN, "an empty queue disarms");
}

/* The reverse leg needs the requester named, and only the START's sender
 * identity names it. Without one the answerer cannot tell its own queued
 * traffic for that node from anyone else's, and declares no reverse leg —
 * which is what a node that never shipped an identity did, always. */
static void testReverseNeedsIdent(void) {
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    A.eng.haveOwnIdent = false;        /* a node that does not name itself */

    memcpy(A.peerTag, TAG, 3);
    A.peer.known = true;
    A.peer.peerId = 5;
    memcpy(B.peerTag, IDENT_A, 3);
    B.peer.known = true;
    B.peer.peerId = 9;
    supeEngTagAdd(&B.eng, TAG, true, 0);

    pushPkt(&A, TAG, 5, 300);
    pushPkt(&B, nullptr, 9, 200);      /* B has traffic for A, and cannot say so */

    launchFrom(&A);
    drive(air, 20000);

    eqi(A.eng.trainPktsOut, 1, "A's train still flies");
    eqi(B.eng.trainPktsOut, 0, "B declared no reverse leg");
    eqi(loraqDepth(&B.q), 1, "B's packet is still queued");
}

int main(void) {
    testBidirectional();
    testRefusal();
    testAbsenceLadder();
    testCountZeroClose();
    testNoWaiting();
    testThirdPartyHold();
    testLinkCrossfire();
    testProofGoesOut();
    testReverseNeedsIdent();
    printf("%d checks, %d failed\n", g_run, g_fail);
    return g_fail ? 1 : 0;
}
