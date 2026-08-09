/**
 * supe_engine — the SUPE state machine on the revised protocol (plans/SUPE.md):
 * the requester states its load, the peer chooses the channel and the budget
 * (SUPE_GRANT), both sides send MANIFEST and a train, and the whole thing ends
 * by arithmetic. One machine, both roles, and the one decider in the system —
 * everything else answers questions.
 *
 * **Single-threaded by contract, no lock anywhere.** Every entry point is
 * called from one context; the host serialises at the boundary (on ESP-IDF a
 * mutex in lora_supe.cpp, on a bare host nothing at all). No entry point
 * blocks: a step that needs to happen later is scheduled via host->schedule
 * and the entry returns.
 *
 * **Pure.** This file and supe_engine.cpp know nothing of ESP-IDF, RadioLib,
 * FreeRTOS or Reticulum — everything platform arrives through SupeHost, peers
 * arrive as views, and packets arrive already tagged (the observer computed
 * the tag at ingress). That is what makes the engine host-runnable:
 * test/supe_engine_test.cpp steps whole transactions against a stub host.
 */
#ifndef IFACE_LORA_SUPE_ENGINE_H
#define IFACE_LORA_SUPE_ENGINE_H

#include "supe.h"
#include "lora_queue.h"

/* ─────────────── what the platform says about a peer ───────────────
 *
 * A snapshot, filled by the host's peer_get for the node behind a tag. The
 * peer table owns all of it; the engine never stores any of it. */
struct SupePeerView {
    bool     known;          /* its SUPE_ANNOUNCE2 has been heard — a peer at all */
    uint16_t peerId;         /* the queue's peer_id for it; LORAQ_PEER_NONE if none */
    uint8_t  fam;
    uint8_t  topBudget;      /* the ceiling nibble it announced */
    int8_t   maxTxpDbm;      /* what it announced as its maximum */
    bool     adaptive;
    /* ours toward it */
    int8_t   txpOpen;        /* what the power controller opens at (§15) */
    int8_t   txpMax;         /* our configured maximum, channel-capped */
    /* the absence ladder's record (§11) */
    uint8_t  absentStrikes;  /* silences since last evidence of life */
    uint32_t absentUntilMs;  /* while in the future: its traffic is dropped */
    uint32_t retryWaitUntilMs; /* the ladder's randomised wait between requests —
                                * the packet HOLDS through it */
    uint32_t backoffUntilMs; /* a refusal said how long not to ask — traffic
                              * flies plainly meanwhile */
    bool     detoured;       /* a detour with it has completed before */
};

/* ─────────────── what the engine tells the platform ───────────────
 *
 * One event, delivered through host->peer_note against the transaction's tag.
 * The platform files it where the tag resolves — a node row, a link, or
 * nowhere (an anonymous requester), which is the platform's call. */
enum SupePeerEvent : uint8_t {
    SUPE_EV_ALIVE = 1,       /* any evidence of life — cancels absence outright */
    SUPE_EV_STRIKE,          /* a request drew silence; the ladder advances */
    SUPE_EV_REFUSED,         /* a refusal arrived; backoffMs says how long not to ask */
    SUPE_EV_PAIR,            /* a path-loss pair: level + stated power at cfg */
    SUPE_EV_TRAIN_OK,        /* the peer's MANIFEST confirmed our train (§15) */
    SUPE_EV_TRAIN_LOST,      /* no reverse MANIFEST — raise power, remember where */
    SUPE_EV_DETOURED,        /* a detour completed — lifts first-contact caution */
    SUPE_EV_CAPS,            /* the peer's MANIFEST capabilities — filed against
                              * the link for the peer that can never be named */
};

struct SupePeerNote {
    uint8_t  ev;
    uint8_t  reason;         /* REFUSED: the SupeRefusal nibble */
    uint32_t backoffMs;      /* REFUSED/STRIKE: how long not to ask */
    /* PAIR / TRAIN_OK: the measurement */
    SupeCfg  cfg;            /* the configuration the level was read at */
    int16_t  rssiDbm;
    int8_t   txpDbm;         /* the power the other side stated */
    int8_t   triedTxpDbm;    /* TRAIN_LOST: what we transmitted at */
    SupeCaps caps;           /* CAPS: what the MANIFEST carried */
};

/* ─────────────── what the platform says about the channels ─────────────── */
struct SupeChanView {
    uint8_t nChans;                 /* the regime's agile channels, 1..n */
    uint8_t usable[SUPE_CH_MAX];    /* 1 = airtime budget + reuse gap allow it */
    bool    anyBudget;              /* false: every channel is out of airtime */
};

/* ─────────────── the host ───────────────
 *
 * Everything the engine needs from the platform, in one struct. Nothing in it
 * names ITS, FreeRTOS, esp_timer or RadioLib. The queue is deliberately NOT
 * behind it: lora_queue is itself pure and single-threaded under the same
 * boundary, so the engine holds a LoraQueue* and calls it directly.
 *
 * tx_frame / tx_packet fire the radio and return; the completion arrives as
 * supeEngOnTxDone. tune/tune_home are synchronous register work. */
struct SupeHost {
    void*    ctx;
    uint32_t (*now_ms)(void* ctx);
    uint32_t (*rand32)(void* ctx);
    void     (*schedule)(void* ctx, uint32_t at_ms);   /* one-shot; re-arms */
    void     (*sha256)(void* ctx, const uint8_t* d, uint16_t n, uint8_t out[32]);
    /* radio */
    bool     (*tune)(void* ctx, uint8_t chan, const SupeCfg* c, uint8_t sync);
    void     (*tune_home)(void* ctx);                  /* unconditional, unchecked */
    bool     (*tx_frame)(void* ctx, const uint8_t* f, uint16_t len, int8_t dbm);
    /* The train pipeline: stage builds the packet's on-air frames — split,
     * observer tap, fan-out, everything expensive — during the PREVIOUS
     * packet's airtime; fire is a buffer copy and a startTransmit, so the
     * inter-packet gap carries no work at all. */
    bool     (*stage_packet)(void* ctx, const LoraPkt* p, int8_t dbm);
    bool     (*fire_staged)(void* ctx);
    void     (*rx)(void* ctx);                         /* (re)arm the receiver */
    /* stores */
    bool     (*peer_get)(void* ctx, const uint8_t tag[SUPE_TAG_LEN], SupePeerView* out);
    void     (*peer_note)(void* ctx, const uint8_t tag[SUPE_TAG_LEN], const SupePeerNote* n);
    void     (*chan_get)(void* ctx, SupeChanView* out);
    /* diagnostics — may be null; the engine formats, the host routes it to its
     * own logger. `dbgLevel` gates the chatty lines. */
    void     (*log)(void* ctx, const char* msg);
    bool     dbgLevel;
};

/* ─────────────── addresses that mean us (SUPE.md §5) ─────────────── */
#define SUPE_TAGS_MAX        256
#define SUPE_HOLD_MAX          8
#define SUPE_PROOFRET_MAX      8
#define SUPE_LINK_TTL_MS   (10u * 60u * 1000u)   /* a link with no traffic on it */
#define SUPE_PROOF_TTL_MS  30000                 /* the receipt/reverse-table window */
#define SUPE_STARTS_SEEN_MAX   8     /* recent overheard STARTs, for GRANT holds */

struct SupeTag {
    uint8_t  tag[SUPE_TAG_LEN];
    bool     used;
    bool     perm;
    uint8_t  refs;
    uint32_t expiryMs;
};

struct SupeHold {
    uint8_t  tag[SUPE_TAG_LEN];
    bool     used;
    uint32_t untilMs;
};

struct SupeProofRet {
    uint8_t  phash[SUPE_TAG_LEN];
    uint8_t  node4[4];
    uint32_t expiryMs;
    bool     used;
};

/* An overheard START, kept until its GRANT arrives (or a few seconds pass):
 * the GRANT quotes the START's hash, and a third party holding for the
 * requester's identity needs the START it named itself in. */
struct SupeStartSeen {
    uint8_t  hash[SUPE_HASH_LEN];
    uint8_t  tag[SUPE_TAG_LEN];
    bool     haveIdent;
    uint8_t  ident[SUPE_TAG_LEN];
    uint32_t ms;
    bool     used;
};

/* ─────────────── the transaction ─────────────── */
enum SupeXPhase : uint8_t {
    SUPE_X_IDLE = 0,
    /* requester */
    SUPE_X_A_START_TX,     /* START handed to the radio */
    SUPE_X_A_AWAIT_GRANT,
    SUPE_X_A_RETUNE,       /* grant taken; the retune gap is running */
    SUPE_X_A_MAN_TX,
    SUPE_X_A_TRAIN_LEAD,   /* the receiver-flip gap before/between packets */
    SUPE_X_A_TRAIN_TX,
    SUPE_X_A_AWAIT_RMAN,   /* our train is out; the reverse MANIFEST is owed */
    SUPE_X_A_RECV,         /* counting the peer's train */
    /* answerer */
    SUPE_X_B_GRANT_TX,
    SUPE_X_B_AWAIT_MAN,
    SUPE_X_B_RECV,
    SUPE_X_B_MAN_TX,       /* the reverse MANIFEST — its train, or the close */
    SUPE_X_B_TRAIN_LEAD,
    SUPE_X_B_TRAIN_TX,
};

struct SupeXact {
    uint8_t  phase;
    uint8_t  role_b;               /* answering side */
    uint8_t  tag[SUPE_TAG_LEN];
    uint8_t  hash[SUPE_HASH_LEN];  /* the START's — the transaction id */
    uint8_t  regime;               /* the detour's */
    uint8_t  chan, budget;
    SupeCfg  cfg;
    uint32_t durMs;                /* what the GRANT announced */
    bool     reverseExpected;      /* the GRANT's bit: a reverse MANIFEST will
                                    * follow the requester's train. Clear
                                    * means both sides go home on the train's
                                    * end — no frame, nothing waited on. */
    uint32_t beganMs;
    uint32_t deadlineMs;           /* what the armed timer means, per phase */
    bool     retuned;
    /* requester */
    uint16_t peerId;
    uint8_t  peerFam;              /* the peer's family, for resolving the GRANT */
    uint8_t  attempt;              /* the absence ladder's rung, 1.. */
    int8_t   startTxp;
    int16_t  grantRssi;            /* our reading of the GRANT */
    int8_t   grantSnrQ;
    /* trains */
    int8_t   trainTxp;
    uint8_t  sendCount, sentCount;
    uint8_t  planCount;            /* what the MANIFEST promised — sendCount
                                    * can be trimmed after the promise (a
                                    * staging failure), and the difference is
                                    * a packet the far end waits out */
    bool     lenExpired;           /* the receive deadline ended the train */
    bool     staged;               /* the next packet is built and ready to pop */
    uint8_t  expectCount, gotCount;
    int16_t  lastPktRssi;          /* the peer's train as heard, for our MANIFEST */
    int8_t   lastPktSnrQ;
    uint8_t  pendingKind;          /* what the frame in flight was (internal) */
};

/* ─────────────── the engine ─────────────── */
struct SupeEngine {
    const SupeHost* host;
    LoraQueue*      q;

    /* the interface's own facts, refreshed by supeEngConfig */
    uint8_t  regime;               /* the frequency-agility key's number */
    uint8_t  ownFam;
    uint8_t  ownTop;               /* our ceiling nibble */
    int8_t   txpMax;               /* configured tx_power (the hailing cap) */
    bool     adaptive;
    uint8_t  hailSf;
    uint32_t hailBwHz;
    uint8_t  crDenom;
    uint16_t preamble;
    uint8_t  ifaceSync;
    uint16_t maxFrameLen;          /* the interface's per-frame payload quantum
                                    * (how a packet splits), for train airtime */
    uint16_t holdEarlyMs;          /* the access procedure's fixed interval
                                    * (DIFS): a hold releases this early, so a
                                    * node that politely held enters the random
                                    * draw on the same footing as one that never
                                    * heard the hint (SUPE.md §6) */
    uint32_t holdProofMs;          /* how long a queue of nothing but proofs
                                    * bides its time before taking the channel
                                    * on its own — a proof exists only after
                                    * the train that earned it, so its best
                                    * ride is the NEXT transaction in either
                                    * direction. 0 disables the hold. */
    bool     expired;              /* the dialect is past its date */

    SupeTag       tags[SUPE_TAGS_MAX];
    SupeHold      hold[SUPE_HOLD_MAX];
    SupeProofRet  pret[SUPE_PROOFRET_MAX];
    SupeStartSeen seen[SUPE_STARTS_SEEN_MAX];

    SupeXact x;
    bool     plainOnce;            /* the head packet goes plainly, once, without
                                    * being re-classified into another offer */
    bool     offerArmed;           /* verdict said OFFER; the launch waits for the
                                    * pre-offer jitter, then the channel */
    uint32_t offerJitterUntilMs;

    /* what `lora <n> supe` prints */
    uint32_t rxFrames, rxDiscard, rxForeign;
    uint32_t startsOut, grantsIn, grantsOut, refusalsIn, refusalsOut;
    uint32_t detoursDone, trainPktsOut, trainPktsIn;
    uint32_t holdsTaken, dropsAbsent, strikes;

    /* The last transaction ends, newest last — ground truth for `lora <n>
     * supe` that survives a lossy debug log. `why` points at the static
     * literals home() is called with, never at anything owned. */
    struct SupeEndRec {
        const char* why;               /* nullptr = slot unused */
        uint32_t endedMs;
        uint8_t  phase;                /* the phase the end came in */
        uint8_t  chan;
        bool     role_b, ok;
        uint8_t  sent, plan;           /* fired vs what the MANIFEST promised */
        uint8_t  got, expect;
    } ends[8];
    uint8_t endsAt;                    /* next slot to write (ring) */
};

/* The classifier's verdicts, decided on the head of the queue before anything
 * contends for the medium. */
enum : uint8_t { SUPE_V_PLAIN = 0, SUPE_V_HOLD, SUPE_V_DROP, SUPE_V_OFFER };

/* The one deliberately-unspecified decision (SUPE.md §18): whether to detour.
 * One call site (the classifier); inputs are the peer, the queue and the
 * channels; output is no / now / wait-until. plans/simulation.md §7 owns what
 * it should decide; the v0 policy detours whenever there is a peer to detour
 * with. */
enum { DETOUR_NO = 0, DETOUR_NOW, DETOUR_WAIT };
int shouldDetour(const SupePeerView* peer, const LoraQueue* q,
                 const SupeChanView* chans, uint32_t now,
                 uint32_t* wait_until_ms);

/* lifecycle */
void supeEngInit(SupeEngine* e, const SupeHost* host, LoraQueue* q);
void supeEngConfig(SupeEngine* e, uint8_t regime, uint8_t ownFam, uint8_t ownTop,
                   int8_t txpMax, bool adaptive, uint8_t hailSf, uint32_t hailBwHz,
                   uint8_t crDenom, uint16_t preamble, uint8_t ifaceSync,
                   uint16_t maxFrameLen);
void supeEngReset(SupeEngine* e);         /* the radio went away underneath */
void supeEngAbort(SupeEngine* e, const char* why);   /* the watchdog's exit */
bool supeEngBusy(const SupeEngine* e);    /* a transaction owns the radio */

/* callbacks in — the whole surface (plan §5) */
void supeEngOnRx(SupeEngine* e, const uint8_t* f, uint16_t len,
                 int16_t rssi, int16_t snr10);
void supeEngOnPacketRx(SupeEngine* e, int16_t rssi, int16_t snr10);
void supeEngOnTxDone(SupeEngine* e, bool ok);
void supeEngOnTimer(SupeEngine* e);

/* the sender path: the drain classifies the head, wins the channel, launches */
uint8_t supeEngVerdict(SupeEngine* e);
bool    supeEngLaunchDue(const SupeEngine* e);   /* jitter passed; wants the channel */
void    supeEngLaunch(SupeEngine* e);            /* channel won: emit the START */

/* addresses that mean us — fed by the observer through the glue */
void supeEngTagAdd(SupeEngine* e, const uint8_t* addr, bool perm, uint32_t ttlMs);
void supeEngTagRelease(SupeEngine* e, const uint8_t* addr);
void supeEngTagExpire(SupeEngine* e, uint32_t now);
bool supeEngTagIsOurs(const SupeEngine* e, const uint8_t* addr);
void supeEngProofRetFile(SupeEngine* e, const uint8_t phash[16], const uint8_t node4[4]);
const uint8_t* supeEngProofRetLookup(SupeEngine* e, const uint8_t* addr);
bool supeEngHeld(SupeEngine* e, const uint8_t* tag);

/* build our own announcement (the glue paces and transmits it) */
size_t supeEngBuildAnn(SupeEngine* e, uint8_t* out, size_t cap,
                       const uint8_t ids[][SUPE_ID_LEN], uint8_t count,
                       int8_t pwrDbm);

#endif /* IFACE_LORA_SUPE_ENGINE_H */
