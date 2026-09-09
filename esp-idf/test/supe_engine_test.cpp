/**
 * supe_engine_test — the engine plus a stub host is a program that steps whole
 * meetings on a laptop, which is the point of SupeHost: nothing in the engine
 * names a radio, a task or a timer, so this file supplies all three as plain
 * data and a loop.
 *
 * What it steps: regime 0's dialogue in both shapes (hail, answer, frames at
 * the hailing rate; the full meeting at a lowered one), the hailed party
 * opening with GOT, the hail-back a busy hailed party owes, the run and the
 * hold, patience; and under a channel plan the two-slot schedule, the return
 * leg, a repair round, a hole, the wide schedule's ride, crossed hails, the
 * seed-hash gate, and the no-stale-deadline rule.
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

static int dByte(const std::vector<std::vector<uint8_t>>& v, size_t i, size_t j) {
    if (i >= v.size() || j >= v[i].size()) return -1;
    return v[i][j];
}

/* ─────────────── the stub host ─────────────── */

static uint32_t g_now = 1000;
static uint32_t g_rand = 12345;
/* The SNR every frame lands with. 60 (6 dB) affords one budget step from
 * SF7; 0 affords none, which is what a hailing-rate dialogue needs. */
static int16_t g_airSnr10 = 60;

struct AirItem {
    bool     isTrain;
    std::vector<uint8_t> bytes;
    int8_t   dbm;
};

/* The peer record a real host keeps in its peer table; the note handler below
 * mirrors what lora_supe.cpp's does, because that contract is under test. */
struct StubPeer {
    bool     known = false;
    uint16_t peerId = LORAQ_PEER_NONE;
    uint8_t  fam = SUPE_FAM_SX126X, topBudget = 8;
    int8_t   maxTxpDbm = 14;
    int8_t   txpOpen = 14, txpMax = 14;
    uint8_t  unanswered = 0;
    uint32_t holdUntilMs = 0, intervalUntilMs = 0;
    bool     detoured = false;
    uint32_t heardMs = 0;
};

struct TxFrameCopy {
    std::vector<uint8_t> bytes;
    uint8_t* fromPkt;
};

struct Node {
    const char* name;
    SupeEngine eng;
    LoraQueue  q;
    SupeHost   host;
    bool     home = true;
    uint8_t  chan = 0;
    SupeCfg  cfg = {};
    uint8_t  sync = 0x42;
    uint32_t tunes = 0, homes = 0;
    bool     ccaClear = true;
    bool     schedArmed = false;
    uint32_t schedAt = 0;
    std::vector<AirItem> txq;
    std::vector<TxFrameCopy> train;
    bool     trainHeld = false;
    uint32_t trainsDelivered = 0, trainsDropped = 0;
    std::vector<std::vector<uint8_t>> rxBuf;
    std::vector<std::vector<uint8_t>> delivered;
    uint32_t plainRx = 0;             /* train frames handed up as they landed */
    uint8_t  peerTag[3] = {};
    uint8_t  peerTag2[3] = {};        /* a second name for the same node — its
                                       * identity beside its destination, as the
                                       * real peer table resolves both */
    StubPeer peer;
    std::vector<SupePeerNote> notes;
    SupeChanView chans = {};
};

static uint32_t chanKey(const Node* n) {
    if (n->home) return 0x42u;
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
static bool g_rxBusy = false;
static bool hRxBusy(void*) { return g_rxBusy; }
static bool hCca(void* c) { return ((Node*)c)->ccaClear; }

static bool hTrainBuild(void* c, uint16_t peerId, const uint8_t tag[3],
                        uint8_t maxFrames, SupeTrainInfo* out) {
    Node* n = (Node*)c;
    n->train.clear();
    memset(out, 0, sizeof *out);
    for (uint8_t i = 0; i < loraqDepth(&n->q) && out->count < maxFrames; i++) {
        LoraPkt* p = loraqAt(&n->q, i);
        if (!(p->flags & LORAQ_F_HAVE_TAG)) continue;   /* unicast only, as the host */
        bool match = false;
        if (tag && memcmp(p->tag, tag, 3) == 0) match = true;
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

static bool peerNamed(const Node* n, const uint8_t tag[3]) {
    return n->peer.known && (memcmp(tag, n->peerTag, 3) == 0 ||
                             memcmp(tag, n->peerTag2, 3) == 0);
}

static bool hPeerGet(void* c, const uint8_t tag[3], SupePeerView* out) {
    Node* n = (Node*)c;
    memset(out, 0, sizeof *out);
    out->peerId = LORAQ_PEER_NONE;
    out->txpMax = 14;
    out->txpOpen = 14;
    if (!peerNamed(n, tag)) return false;
    out->known = true;
    out->peerId = n->peer.peerId;
    out->fam = n->peer.fam;
    out->topBudget = n->peer.topBudget;
    out->maxTxpDbm = n->peer.maxTxpDbm;
    out->txpOpen = n->peer.txpOpen;
    out->txpMax = n->peer.txpMax;
    out->unanswered = n->peer.unanswered;
    out->holdUntilMs = n->peer.holdUntilMs;
    out->intervalUntilMs = n->peer.intervalUntilMs;
    out->detoured = n->peer.detoured;
    return true;
}
static int8_t hTxpOpen(void* c, const uint8_t tag[3], const SupeCfg*) {
    Node* n = (Node*)c;
    if (!peerNamed(n, tag)) return 14;
    return n->peer.txpOpen;
}
/* The run and the hold, as lora_supe.cpp keeps them: presence never clears a
 * hold, an answer does; three unanswered hails and the peer is held. */
static void hPeerNote(void* c, const uint8_t tag[3], const SupePeerNote* nt) {
    Node* n = (Node*)c;
    n->notes.push_back(*nt);
    if (!peerNamed(n, tag)) return;
    switch (nt->ev) {
        case SUPE_EV_ALIVE:
            n->peer.heardMs = g_now;
            break;
        case SUPE_EV_ANSWERED:
        case SUPE_EV_MET:
            n->peer.unanswered = 0;
            n->peer.holdUntilMs = 0;
            n->peer.intervalUntilMs = 0;
            if (nt->ev == SUPE_EV_MET) n->peer.detoured = true;
            break;
        case SUPE_EV_UNANSWERED:
            n->peer.unanswered++;
            n->peer.intervalUntilMs = g_now + nt->backoffMs;
            if (n->peer.unanswered >= 3) n->peer.holdUntilMs = g_now + 60000;
            break;
        default:
            break;
    }
}
static void hChanGet(void* c, SupeChanView* out) { *out = ((Node*)c)->chans; }
static void hLog(void* c, bool verbose, const char* msg) {
    const char* v = getenv("VERBOSE");
    if (!v) return;
    if (verbose && atoi(v) < 2) return;
    printf("  [%s %6u]%s %s\n", ((Node*)c)->name, g_now, verbose ? " ." : "", msg);
}

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
    n->host.logLevel = SUPE_LOG_VERB;
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

static int  g_dropNthTrain = 0;
static uint8_t g_dropType = 0;
static bool g_dropRepairs = false;
static int  g_trainSeen = 0;
static std::vector<uint8_t> g_droppedCsums;
static int  g_typeCount[256];

static void deliverTo(Node* dst, const AirItem& it) {
    if (it.isTrain) {
        uint8_t csum = supeCrc8(it.bytes.data(), it.bytes.size());
        if (supeEngOnTrainFrame(&dst->eng, csum, (int16_t)(it.dbm - 80), g_airSnr10))
            dst->rxBuf.push_back(it.bytes);
        else if (dst->eng.m.phase == SUPE_M_TRAIN_RX || dst->eng.m.phase == SUPE_M_TRAIN_WAIT ||
                 dst->eng.m.phase == SUPE_M_IDLE)
            dst->plainRx++;
    } else {
        supeEngOnRx(&dst->eng, it.bytes.data(), (uint16_t)it.bytes.size(),
                    (int16_t)(it.dbm - 80), g_airSnr10);
    }
}

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
        } else if (!it.bytes.empty()) {
            g_typeCount[it.bytes[0]]++;
            if (g_dropType && it.bytes[0] == g_dropType) { g_dropType = 0; continue; }
        }
        if (drop) continue;
        for (Node* dst : nodes) {
            if (dst == src || chanKey(dst) != key) continue;
            deliverTo(dst, it);
        }
    }
    return moved;
}

static bool pumpOne(std::vector<Node*>& nodes) {
    for (Node* src : nodes) {
        if (src->txq.empty()) continue;
        AirItem it = src->txq.front();
        src->txq.erase(src->txq.begin());
        uint32_t key = chanKey(src);
        supeEngOnTxDone(&src->eng, true);
        for (Node* dst : nodes) {
            if (dst == src || chanKey(dst) != key) continue;
            deliverTo(dst, it);
        }
        return true;
    }
    return false;
}

static uint32_t g_wakeLateMs = 0;

static void fireTimers(std::vector<Node*>& nodes) {
    for (Node* n : nodes) {
        if (n->schedArmed && (int32_t)(g_now - (n->schedAt + g_wakeLateMs)) >= 0) {
            n->schedArmed = false;
            supeEngOnTimer(&n->eng);
        }
    }
}

/* The glue's launch service, as supePoll does it: a hail contends for the
 * hailing channel when the engine says it wants it. */
static void launchService(std::vector<Node*>& nodes) {
    for (Node* n : nodes)
        if (supeEngLaunchDue(&n->eng)) supeEngLaunch(&n->eng);
}

static uint32_t drive(std::vector<Node*> nodes, uint32_t maxMs) {
    uint32_t began = g_now, end = g_now + maxMs;
    while ((int32_t)(end - g_now) > 0) {
        bool moved = airPump(nodes);
        fireTimers(nodes);
        launchService(nodes);
        if (moved) continue;
        bool due = false;
        for (Node* n : nodes)
            if (n->schedArmed && (int32_t)(g_now - (n->schedAt + g_wakeLateMs)) >= 0) due = true;
        if (due) continue;
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

static void driveUntilDone(std::vector<Node*> nodes, Node* watch,
                           uint32_t wantDone, uint32_t maxMs) {
    uint32_t end = g_now + maxMs;
    while ((int32_t)(end - g_now) > 0) {
        bool moved = airPump(nodes);
        fireTimers(nodes);
        launchService(nodes);
        if (moved) continue;
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
            if (n->eng.m.phase != SUPE_M_IDLE) anyBusy = true;
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
 * by A's identity (peer id 9), and B's identity is one of A's addresses so a
 * hail-back from B resolves at A. */
static void wire(Node* A, Node* B) {
    memcpy(A->peerTag, TAG, 3);
    memcpy(A->peerTag2, IDENT_B, 3);
    A->peer.known = true;
    A->peer.peerId = 5;
    memcpy(B->peerTag, IDENT_A, 3);
    B->peer.known = true;
    B->peer.peerId = 9;
    supeEngTagAdd(&B->eng, TAG, true, 0);
    supeEngTagAdd(&A->eng, IDENT_A, true, 0);
    supeEngTagAdd(&B->eng, IDENT_B, true, 0);
}

static void resetAir(void) {
    g_rxBusy = false;
    g_wakeLateMs = 0;
    g_dropNthTrain = 0;
    g_dropType = 0;
    g_dropRepairs = false;
    g_trainSeen = 0;
    g_droppedCsums.clear();
    g_airSnr10 = 60;
    memset(g_typeCount, 0, sizeof g_typeCount);
}

static bool holdsWide(const Node* n) {
    for (int i = 0; i < SUPE_SCHED_MAX; i++)
        if (n->eng.sched[i].used && n->eng.sched[i].wide) return true;
    return false;
}

/* ─────────────── regime 0: the dialogue ─────────────── */

/* Hail, READY, frames — and nothing else. The frames reach the daemon as they
 * land, the queue is consumed on transmit, and no goodbye seeds anything. */
static void testDialogueLite(void) {
    resetAir();
    g_airSnr10 = 0;                            /* no headroom: budget 0 */
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_SINGLE);
    nodeInit(&B, "B", SUPE_REGIME_SINGLE);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 300, 0x11);
    pushPkt(&A, TAG, 5, 120, 0x22);

    launchFrom(&A);
    driveUntilDone(air, &A, 1, 5000);

    eqi(A.eng.m.phase, SUPE_M_IDLE, "A is home");
    eqi(B.eng.m.phase, SUPE_M_IDLE, "B is home");
    eqi(A.eng.hailsOut, 1, "one hail on the shared channel");
    eqi(g_typeCount[SUPE_T_READY], 1, "one READY answered it");
    eqi(g_typeCount[SUPE_T_GOT], 0, "no GOT: the hailed party held nothing");
    eqi(g_typeCount[SUPE_T_END], 0, "no END at the hailing rate");
    eqi(g_typeCount[SUPE_T_BYE], 0, "…and no BYE");
    eqi(A.eng.framesOut, 3, "a 300 B packet split: three frames out");
    eqi(B.eng.framesIn, 3, "…three frames counted in");
    eqi((long)B.plainRx, 3, "…each handed up as it landed");
    eqi((long)B.delivered.size(), 0, "…none buffered for the close");
    eqi(A.eng.meetingsDone, 1, "A counts one meeting");
    eqi(B.eng.meetingsDone, 1, "B counts one meeting");
    eqi(loraqDepth(&A.q), 0, "A's queue was consumed: the frames flew");
    eqi(A.trainsDelivered, 1, "A's train copies were released as done with");
    ok(!holdsWide(&A) && !holdsWide(&B), "regime 0 seeds no schedule");
    eqi((long)A.tunes, 0, "A never retuned");
    eqi((long)B.tunes, 0, "…nor did B");
}

/* Hail, READY at budget 1, the train at SF6, END, BYE. */
static void testDialogueFull(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_SINGLE);
    nodeInit(&B, "B", SUPE_REGIME_SINGLE);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 300, 0x11);

    launchFrom(&A);
    driveUntilDone(air, &A, 1, 5000);

    eqi(A.eng.meetingsDone, 1, "A counts one meeting");
    eqi(B.eng.meetingsDone, 1, "B counts one meeting");
    eqi(g_typeCount[SUPE_T_END], 1, "the train was closed by a END");
    eqi(g_typeCount[SUPE_T_BYE], 1, "…and answered by a BYE");
    eqi((long)B.delivered.size(), 2, "B delivered the whole train at the close");
    ok(dByte(B.delivered, 0, 0) == 0x02 && dByte(B.delivered, 1, 0) == 0x12,
       "…in sequence");
    eqi(B.cfg.sf, 6, "B retuned to SF6 for the train");
    eqi((long)B.chan, 0, "…on the hailing frequency");
    eqi((long)B.sync, 0x42, "…under the interface's own word");
    ok(A.homes >= 1 && B.homes >= 1, "both radios came home");
    ok(!holdsWide(&A) && !holdsWide(&B), "regime 0 seeds no schedule");
    eqi(loraqDepth(&A.q), 0, "A's queue was consumed on the proven close");
    bool aReport = false, aOk = false, aAnswered = false;
    for (auto& nt : A.notes) {
        if (nt.ev == SUPE_EV_REPORT) aReport = true;
        if (nt.ev == SUPE_EV_TRAIN_OK) aOk = true;
        if (nt.ev == SUPE_EV_ANSWERED) aAnswered = true;
    }
    ok(aReport, "A got the READY's report of its hail");
    ok(aOk, "A's power controller heard its train confirmed");
    ok(aAnswered, "A's peer record shows B answered");
}

/* The hailed party holds traffic too: it answers with GOT, its train goes
 * first, and the hailer's rides the answering turn. */
static void testDialogueGotFirst(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_SINGLE);
    nodeInit(&B, "B", SUPE_REGIME_SINGLE);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 150, 0x11);
    pushPkt(&B, IDENT_A, 9, 100, 0x33);
    pushPkt(&B, IDENT_A, 9, 80, 0x44);

    launchFrom(&A);
    driveUntilDone(air, &A, 1, 5000);

    eqi(g_typeCount[SUPE_T_GOT], 2, "an opening GOT and an answering one");
    eqi(g_typeCount[SUPE_T_READY], 1, "one READY confirmed the terms for both");
    eqi(A.eng.meetingsDone, 1, "one meeting carried both directions");
    eqi(B.eng.meetingsDone, 1, "…on B's count too");
    eqi(B.eng.framesOut, 2, "B's train went first");
    eqi((long)A.delivered.size(), 2, "A delivered B's frames");
    eqi((long)B.delivered.size(), 1, "B delivered A's frame");
    eqi(loraqDepth(&B.q), 0, "B's queue was consumed on the proven close");
    eqi(loraqDepth(&A.q), 0, "…and A's");
    eqi(A.eng.hailsOut, 1, "the return leg cost the shared channel nothing");
}

/* The same, at the hailing rate: GOT, READY, B's frames, A's frames, done. */
static void testDialogueGotFirstLite(void) {
    resetAir();
    g_airSnr10 = 0;
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_SINGLE);
    nodeInit(&B, "B", SUPE_REGIME_SINGLE);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 150, 0x11);
    pushPkt(&B, IDENT_A, 9, 100, 0x33);

    launchFrom(&A);
    driveUntilDone(air, &A, 1, 5000);

    eqi(g_typeCount[SUPE_T_GOT], 1, "one opening GOT");
    eqi(g_typeCount[SUPE_T_READY], 1, "one READY");
    eqi(g_typeCount[SUPE_T_END], 0, "no END at the hailing rate");
    eqi(B.eng.framesOut, 1, "B's frame went first");
    eqi(A.eng.framesOut, 1, "…then A's");
    eqi((long)A.plainRx, 1, "A handed B's frame up as it landed");
    eqi((long)B.plainRx, 1, "B handed A's frame up as it landed");
    eqi(A.eng.meetingsDone, 1, "A counts one meeting");
    eqi(B.eng.meetingsDone, 1, "B counts one meeting");
    eqi(loraqDepth(&A.q), 0, "A's queue was consumed");
    eqi(loraqDepth(&B.q), 0, "…and B's");
}

/* A hailed party that cannot answer owes a hail: it hails back with a count
 * of zero, the hailer answers with GOT, and the traffic flows. */
static void testHailBackRegime0(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_SINGLE);
    nodeInit(&B, "B", SUPE_REGIME_SINGLE);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 150, 0x11);

    /* B's radio is spoken for when the hail lands. */
    B.eng.m.phase = SUPE_M_AWAIT_READY;
    B.eng.m.deadlineMs = g_now + 100000;
    launchFrom(&A);
    airPump(air);                              /* the hail crosses */
    ok(B.eng.owed[0].used, "B owes A a hail");
    supeEngAbort(&B.eng, "test");              /* B comes free */
    driveUntilDone(air, &A, 1, 5000);

    eqi(B.eng.hailBacksOut, 1, "B hailed back");
    eqi(A.eng.hailsOut + A.eng.hailBacksOut, 1, "A hailed once");
    eqi(A.eng.meetingsDone, 1, "A's traffic met");
    eqi(g_typeCount[SUPE_T_GOT], 1, "A answered the hail-back with GOT");
    eqi((long)B.delivered.size(), 1, "B delivered A's frame");
    eqi(loraqDepth(&A.q), 0, "A's queue was consumed");
    ok(!B.eng.owed[0].used, "the debt is discharged");
}

/* Nobody answers: three hails at rising power, an interval after each, then
 * the hold; the packet leaves at its own patience. */
static void testRunAndHold(void) {
    resetAir();
    Node A = {};
    nodeInit(&A, "A", SUPE_REGIME_SINGLE);
    memcpy(A.peerTag, TAG, 3);
    A.peer.known = true;
    A.peer.peerId = 5;
    A.peer.txpOpen = 2;
    std::vector<Node*> air = { &A };
    pushPkt(&A, TAG, 5, 100, 0x11);
    uint32_t queuedAt = g_now;

    std::vector<int8_t> powers;
    uint32_t end = g_now + 1900;
    while ((int32_t)(end - g_now) > 0) {
        uint8_t v = supeEngVerdict(&A.eng);
        if (v == SUPE_V_OFFER && supeEngLaunchDue(&A.eng)) {
            supeEngLaunch(&A.eng);
            powers.push_back(A.txq.back().dbm);
        }
        airPump(air);
        fireTimers(air);
        g_now++;
    }
    eqi(A.eng.hailsOut, 3, "three hails in the run");
    ok(powers.size() == 3 && powers[0] == 2 && powers[1] == 8 && powers[2] == 14,
       "…at the opening power, halfway, then maximum");
    eqi(A.peer.unanswered, 3, "three unanswered");
    ok(A.peer.holdUntilMs && (int32_t)(A.peer.holdUntilMs - g_now) > 0, "the peer is held");
    eqi(supeEngVerdict(&A.eng), SUPE_V_WAIT, "held: the packet waits, not dropped");
    g_now = queuedAt + SUPE_PATIENCE_MS;
    eqi(supeEngVerdict(&A.eng), SUPE_V_DROP, "…until its own patience runs out");
    eqi(A.eng.dropsPatience, 1, "counted as a patience drop");
    eqi(A.trainsDropped, 3, "each hail's train was released unflown");
}

/* ─────────────── a channel plan ─────────────── */

static void testMeetingPlan(void) {
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
    eqi(A.eng.hailsOut, 1, "one hail on the shared channel — and only one");
    eqi(B.eng.slotsSpoken, 1, "B spoke at the first slot");
    eqi(A.eng.slotsListened, 1, "A listened at it");
    eqi(A.eng.meetingsDone, 1, "A counts one meeting");
    eqi(B.eng.meetingsDone, 1, "B counts one meeting");
    eqi(A.eng.framesOut, 3, "three frames out");
    eqi(B.eng.framesIn, 3, "…three frames in");
    eqi((long)B.delivered.size(), 3, "B delivered the whole train upward");
    ok(dByte(B.delivered, 0, 0) == 0x02 && dByte(B.delivered, 1, 0) == 0x12,
       "…in sequence: the split's halves are adjacent");
    ok(B.chan >= 1 && B.chan <= 9, "the meeting was on an agile channel");
    eqi(loraqDepth(&A.q), 0, "A's queue was consumed on the proven close");
    ok(A.peer.detoured, "A's peer record remembers the meeting");
    ok(holdsWide(&A) && holdsWide(&B), "the final END seeded a wide schedule at both ends");
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        if (A.eng.sched[i].used && A.eng.sched[i].wide)
            ok(!A.eng.sched[i].weTx0, "A sent the final train, so B speaks first");
        if (B.eng.sched[i].used && B.eng.sched[i].wide)
            ok(B.eng.sched[i].weTx0, "…and B holds the mirror of that");
    }
}

static void testGotFirstPlan(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 150, 0x11);
    pushPkt(&B, IDENT_A, 9, 100, 0x33);
    pushPkt(&B, IDENT_A, 9, 80, 0x44);

    launchFrom(&A);
    driveUntilDone(air, &A, 1, 20000);

    eqi(A.eng.meetingsDone, 1, "one meeting carried both directions");
    eqi(B.eng.meetingsDone, 1, "…on B's count too");
    eqi(B.eng.framesOut, 2, "B's train went first, opening with GOT");
    eqi((long)A.delivered.size(), 2, "A delivered B's frames");
    eqi((long)B.delivered.size(), 1, "B delivered A's frame from the answering turn");
    eqi(loraqDepth(&B.q), 0, "B's queue was consumed");
    eqi(loraqDepth(&A.q), 0, "…and A's");
    eqi(A.eng.hailsOut, 1, "the return leg cost the shared channel nothing");
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
    g_dropNthTrain = 2;

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
    g_dropRepairs = true;

    launchFrom(&A);
    driveUntilDone(air, &A, 1, 30000);

    eqi((long)B.delivered.size(), 2, "what one repair round cannot save is surrendered");
    ok(dByte(B.delivered, 0, 1) == 0x11 && dByte(B.delivered, 1, 1) == 0x33,
       "…the hole stays a hole and the order stays the order");
}

static void testWideRide(void) {
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
        eqi(A.eng.meetingsDone, 1, "the first meeting completed");

        g_wakeLateMs = late;
        g_now += 200;
        pushPkt(&B, IDENT_A, 9, 120, 0x33);
        driveUntilDone(air, &B, 2, 20000);
        eqi(B.eng.meetingsDone, 2, "the reply's meeting completed at a wide slot");
        eqi(B.eng.hailsOut, 0, "…B never touched the shared channel");
        eqi((long)A.delivered.size(), 1, "A holds the reply");
        eqi((long)B.delivered.size(), 1, "B holds the first train");
    }
    g_wakeLateMs = 0;
}

/* The hailed party cannot take either slot (both channels read busy): it owes
 * a hail, sends it when the schedule has expired, and the hailer answers with
 * GOT. */
static void testOwedHailPlan(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 150, 0x11);
    B.ccaClear = false;

    launchFrom(&A);
    drive(air, 300);                           /* both slots read busy */
    eqi(A.eng.meetingsDone, 0, "nothing met at the slots");
    eqi(B.eng.slotsSkipped, 2, "B skipped both slots as busy");
    B.ccaClear = true;
    driveUntilDone(air, &A, 1, 20000);
    eqi(B.eng.hailBacksOut, 1, "B owed a hail and hailed back");
    eqi(A.eng.meetingsDone, 1, "A's traffic met at B's schedule");
    eqi((long)B.delivered.size(), 1, "B delivered A's frame");
    eqi(A.eng.hailsOut, 1, "A hailed once — the hail-back came inside the interval");
}

/* Crossed hails: A hails while B is deaf to it, then B hails A. A holds both
 * schedules and speaks at B's; one meeting carries both directions, and A's
 * own schedule is consumed by it rather than scoring. */
static void testCrossedHails(void) {
    resetAir();
    Node A = {}, B = {};
    nodeInit(&A, "A", SUPE_REGIME_EU863);
    nodeInit(&B, "B", SUPE_REGIME_EU863);
    std::vector<Node*> air = { &A, &B };
    std::vector<Node*> aAlone = { &A };
    wire(&A, &B);
    pushPkt(&A, TAG, 5, 150, 0x11);
    pushPkt(&B, IDENT_A, 9, 100, 0x33);
    loraqAt(&B.q, 0)->flags |= LORAQ_F_HAVE_TAG;
    memcpy(loraqAt(&B.q, 0)->tag, IDENT_A, 3);

    launchFrom(&A);
    pumpOne(aAlone);                           /* A's hail: B never hears it */
    launchFrom(&B);                            /* B hails A */
    pumpOne(air);                              /* …and A hears that */
    driveUntilDone(air, &B, 1, 20000);

    eqi(A.eng.meetingsDone, 1, "one meeting at A");
    eqi(B.eng.meetingsDone, 1, "…and at B");
    eqi((long)A.delivered.size(), 1, "A holds B's frame");
    eqi((long)B.delivered.size(), 1, "B holds A's frame");
    eqi(loraqDepth(&A.q), 0, "A's queue was consumed");
    eqi(loraqDepth(&B.q), 0, "…and B's");
    drive(air, 400);                           /* A's own schedule runs out */
    eqi(A.eng.unanswered, 0, "A's crossed schedule was consumed, not scored");
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
    airPump(air);
    drive(air, 20);
    bool poisoned = false;
    uint32_t end = g_now + 2000;
    while ((int32_t)(end - g_now) > 0) {
        for (Node* n : air)
            if (n == &B && !n->txq.empty() && !poisoned &&
                n->txq.front().bytes[0] == SUPE_T_READY) {
                n->txq.front().bytes[1] ^= 0xFF;
                poisoned = true;
            }
        bool moved = airPump(air);
        fireTimers(air);
        if (!moved) g_now++;
        if (poisoned && A.eng.rxForeign) break;
    }
    ok(poisoned, "the READY was poisoned on the air");
    ok(A.eng.rxForeign >= 1, "a mismatched seed hash dies in one frame");
    eqi(A.eng.meetingsDone, 0, "…and no meeting follows it");
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
    airPump(air);
    eqi(supeEngVerdict(&A.eng), SUPE_V_WAIT, "a live schedule holds the packet for its slots");
    drive(air, 400);                           /* horizon and interval pass */
    Node C = {};
    nodeInit(&C, "C", SUPE_REGIME_EU863);
    pushPkt(&C, TAG, LORAQ_PEER_NONE, 100, 0x11);
    eqi(supeEngVerdict(&C.eng), SUPE_V_PLAIN, "an unknown peer flies plainly");
}

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
    while (B.eng.m.phase != SUPE_M_READY_TX && (int32_t)(guard - g_now) > 0) {
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
    eqi(B.eng.m.phase, SUPE_M_READY_TX, "B's READY is on the air");
    eqi((long)B.eng.m.deadlineMs, 0, "no deadline stands while a tx is in flight");
    uint32_t at = supeEngNextEventMs(&B.eng, g_now);
    ok(at == UINT32_MAX || (int32_t)(at - g_now) > 0,
       "the engine asks for no wake at-or-before now");
    for (int i = 0; i < 5; i++) supeEngOnTimer(&B.eng);
    eqi(B.eng.m.phase, SUPE_M_READY_TX, "hammering the timer changes nothing");
    ok(!B.schedArmed || (int32_t)(B.schedAt - g_now) > 0,
       "…and never re-arms it at or before now");

    driveUntilDone(air, &A, 1, 20000);
    eqi(A.eng.meetingsDone, 1, "the meeting still completes");
}

int main(void) {
    testDialogueLite();
    testDialogueFull();
    testDialogueGotFirst();
    testDialogueGotFirstLite();
    testHailBackRegime0();
    testRunAndHold();
    testMeetingPlan();
    testGotFirstPlan();
    testRepair();
    testHole();
    testWideRide();
    testOwedHailPlan();
    testCrossedHails();
    testSeedHashGate();
    testVerdicts();
    testNoStaleDeadlineInTxPhase();
    printf("%d checks, %d failed\n", g_run, g_fail);
    return g_fail ? 1 : 0;
}
