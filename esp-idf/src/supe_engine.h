/**
 * supe_engine — the SUPE state machine (plans/SUPE.md): PRIVSYNC seeds a
 * derived schedule of slots, the pair meets at the first slot that works,
 * HAVEDATA/GIMME confirm attention and budget, trains of LoRa frames flow with
 * THATSIT's checksum list naming the sequence after the fact, one RESEND round
 * repairs, and every meeting's final THATSIT seeds the next schedule. One
 * machine, both roles, and the one decider in the system — everything else
 * answers questions.
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
 * test/supe_engine_test.cpp steps whole meetings against a stub host.
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
    /* ours toward it */
    int8_t   txpOpen;        /* what the power controller opens at, at the
                              * HAILING configuration (§15); any other
                              * configuration asks host->txp_open */
    int8_t   txpMax;         /* our configured maximum, channel-capped */
    /* the absence ladder's record (§12) */
    uint8_t  absentStrikes;  /* narrow schedules expired unmet since evidence of life */
    uint32_t absentUntilMs;  /* while in the future: its traffic is dropped */
    uint32_t retryWaitUntilMs; /* the ladder's randomised wait between seeds —
                                * the packet WAITS through it */
    bool     detoured;       /* a meeting with it has completed before */
};

/* ─────────────── what the engine tells the platform ───────────────
 *
 * One event, delivered through host->peer_note against a tag. The platform
 * files it where the tag resolves — a node row, a link, or nowhere (an
 * anonymous peer), which is the platform's call. */
enum SupePeerEvent : uint8_t {
    SUPE_EV_ALIVE = 1,       /* any evidence of life — cancels absence outright */
    SUPE_EV_STRIKE,          /* a narrow schedule expired unmet; the ladder advances */
    SUPE_EV_PAIR,            /* a path-loss pair: level measured here + stated power */
    SUPE_EV_REPORT,          /* the peer's account of how OUR transmission landed:
                              * its reading + the power we sent at, at cfg */
    SUPE_EV_TRAIN_OK,        /* our train was confirmed (§15) */
    SUPE_EV_TRAIN_LOST,      /* a meeting died after our train flew — raise power,
                              * remember where */
    SUPE_EV_MET,             /* a meeting completed — lifts first-contact caution */
};

struct SupePeerNote {
    uint8_t  ev;
    uint32_t backoffMs;      /* STRIKE: how long not to seed again */
    uint32_t agoMs;          /* STRIKE: how long ago the seed went out. The wait
                              * paces SEEDS, so it runs from the seed; the
                              * horizon just waited out is already time spent
                              * not asking. */
    /* PAIR / REPORT / TRAIN_OK: the measurement */
    SupeCfg  cfg;            /* the configuration the level was read at */
    int16_t  rssiDbm;
    int8_t   txpDbm;         /* PAIR: the power the other side stated;
                              * REPORT/TRAIN_OK: the power WE transmitted at */
    bool     haveLevel;      /* TRAIN_OK: rssiDbm carries the peer's reading */
    int8_t   triedTxpDbm;    /* TRAIN_LOST: what we transmitted at */
};

/* ─────────────── what the platform says about the channels ─────────────── */
struct SupeChanView {
    uint8_t nChans;                 /* the regime's agile channels, 1..n */
    uint8_t usable[SUPE_CH_MAX];    /* 1 = airtime budget + reuse gap allow it */
    bool    anyBudget;              /* false: every channel is out of airtime */
};

/* ─────────────── the outgoing train ───────────────
 *
 * Built by the host from the queue at meeting time and held until the meeting
 * closes — the frames are what a repair round resends, so they outlive the
 * queue entries they were built from (consumed at build; a meeting that dies
 * loses them to the layers above, exactly as the air would have). */
struct SupeTrainInfo {
    uint8_t  count;                     /* LoRa frames */
    uint16_t lens[SUPE_TRAIN_MAX];      /* each frame's on-air byte count */
    uint8_t  csum[SUPE_TRAIN_MAX];      /* supeCrc8 over each frame */
};

/* ─────────────── the host ───────────────
 *
 * Everything the engine needs from the platform, in one struct. Nothing in it
 * names ITS, FreeRTOS, esp_timer or RadioLib. The queue is deliberately NOT
 * behind it: lora_queue is itself pure and single-threaded under the same
 * boundary, so the engine holds a LoraQueue* and calls it directly.
 *
 * tx_frame / train_fire fire the radio and return; the completion arrives as
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
    void     (*rx)(void* ctx);                         /* (re)arm the receiver */
    /* Is a frame arriving at this instant — a preamble or a valid header the
     * modem is still working on? What lets a slot window close on evidence
     * rather than walk off mid-preamble. */
    bool     (*rx_busy)(void* ctx);
    /* One clear-channel read at the current tuning. The appointment grants the
     * peer's attention, never the spectrum: a busy channel skips the slot. */
    bool     (*cca)(void* ctx);
    /* the outgoing train: build whole from the queue (consuming what it takes),
     * fire one frame, release at close */
    bool     (*train_build)(void* ctx, uint16_t peerId,
                            const uint8_t tag[SUPE_TAG_LEN], uint8_t maxFrames,
                            SupeTrainInfo* out);
    bool     (*train_fire)(void* ctx, uint8_t idx, int8_t dbm);
    /* delivered=true: the peer's answer proved the train (and its THATSIT)
     * landed — the queue entries it was built from are consumed. false: the
     * meeting died unproven; the entries stay queued for the next chance, and
     * Reticulum's duplicate hash list absorbs the rare both-happened case. */
    void     (*train_done)(void* ctx, bool delivered);
    /* the incoming train: the host buffered each frame as it arrived (it told
     * the engine through supeEngOnTrainFrame); at close the engine hands back
     * the delivery order — indices into the host's buffer, sequence-sorted,
     * holes simply absent — and the host flushes them upward */
    void     (*train_deliver)(void* ctx, const uint8_t* order, uint8_t n);
    /* stores */
    bool     (*peer_get)(void* ctx, const uint8_t tag[SUPE_TAG_LEN], SupePeerView* out);
    void     (*peer_note)(void* ctx, const uint8_t tag[SUPE_TAG_LEN], const SupePeerNote* n);
    /* The power to open toward the node behind a tag AT a configuration —
     * asked where the frames will actually fly (§15). */
    int8_t   (*txp_open)(void* ctx, const uint8_t tag[SUPE_TAG_LEN], const SupeCfg* cfg);
    void     (*chan_get)(void* ctx, SupeChanView* out);
    /* Diagnostics — may be null; the engine formats, the host routes it to its
     * own logger at the level the engine asked for.
     *
     * Debug is **one line per meeting** and the failures that cost something.
     * Everything a meeting is made of — the schedule, each slot, each frame —
     * is verbose, because a line per step turns a busy pair into a scroll
     * nobody reads and buries the one line that says what happened. The
     * meeting's line carries what those steps would have said anyway: what
     * each side transmitted at, how the other read it, and the counts. */
    void     (*log)(void* ctx, bool verbose, const char* msg);
    uint8_t  logLevel;                  /* 0 none, 1 debug, 2 verbose */
};
#define SUPE_LOG_NONE  0
#define SUPE_LOG_DBG   1
#define SUPE_LOG_VERB  2

/* ─────────────── addresses that mean us (SUPE.md §5) ─────────────── */
#define SUPE_TAGS_MAX        256
#define SUPE_PROOFRET_MAX      8
#define SUPE_LINK_TTL_MS   (10u * 60u * 1000u)   /* a link with no traffic on it */
#define SUPE_PROOF_TTL_MS  30000                 /* the receipt/reverse-table window */

struct SupeTag {
    uint8_t  tag[SUPE_TAG_LEN];
    bool     used;
    bool     perm;
    uint8_t  refs;
    uint32_t expiryMs;
};

struct SupeProofRet {
    uint8_t  phash[SUPE_TAG_LEN];
    uint8_t  node4[4];
    uint32_t expiryMs;
    bool     used;
};

/* ─────────────── schedules (SUPE.md §7) ─────────────── */
#define SUPE_SCHED_MAX  4

struct SupeSched {
    bool      used;
    bool      wide;           /* seeded by a goodbye rather than a PRIVSYNC */
    bool      weSeeded;       /* narrow only: our PRIVSYNC — its expiry strikes */
    bool      weTx0;          /* we transmit in slot 0, 2, 4…; else 1, 3, 5… */
    bool      consumed;       /* a slot was met: every later slot is void */
    uint8_t   tag[SUPE_TAG_LEN];  /* how peer_get/peer_note reach the node — the
                                   * packet tag we seeded on, or the seeker's
                                   * identity; zeros for an anonymous seeker */
    bool      haveTag;
    uint16_t  peerId;
    uint32_t  epochMs;        /* the end of the seeding frame, as timed here */
    SupeCfg   slotCfg;        /* what the slot frames fly at: the hailing
                               * configuration (narrow) or the budget the seeding
                               * meeting confirmed (wide) */
    SupeSchedD d;
    uint8_t   nextSlot;
    int8_t    seedTxp;        /* narrow+weSeeded: what the PRIVSYNC flew at */
    bool      havePs;         /* narrow+listener: our reading of the PRIVSYNC */
    int16_t   psRssi;
    int8_t    psSnrQ;
    /* Why a schedule carried nothing, counted as it happens. A schedule that
     * expires unmet while its traffic waited is the one failure that costs a
     * whole horizon, and no frame on the air records it: the packet ring shows
     * only the fallback that follows. These say which branch spent the slots. */
    uint8_t   nOwnDue;        /* own slots whose moment arrived */
    uint8_t   nSpoke;         /* …opened with HAVEDATA */
    uint8_t   nNoTraffic;     /* …nothing queued for this peer at that instant */
    uint8_t   nBusy;          /* …the channel was not clear */
    uint8_t   nLate;          /* …reached past the lateness tolerance */
    uint8_t   nNoTrain;       /* …the queue matched but built no frames */
    uint8_t   nNoTune;        /* …the retune failed */
};

/* ─────────────── the meeting ─────────────── */
enum SupeMPhase : uint8_t {
    SUPE_M_IDLE = 0,
    SUPE_M_PS_TX,          /* PRIVSYNC on the air; its end is the epoch */
    SUPE_M_SLOT_LISTEN,    /* a slot window is open */
    SUPE_M_HD_TX,          /* our HAVEDATA (opening or answering) on the air */
    SUPE_M_AWAIT_GIMME,
    SUPE_M_GIMME_TX,
    SUPE_M_TRAIN_WAIT,     /* the flip gap / retune lead before our next frame */
    SUPE_M_TRAIN_TX,       /* one of our train frames on the air */
    SUPE_M_THATSIT_TX,
    SUPE_M_TRAIN_RX,       /* counting the peer's frames */
    SUPE_M_AWAIT_THATSIT,
    SUPE_M_AWAIT_ANSWER,   /* our THATSIT is out; BYE / RESEND / answering
                            * HAVEDATA decides what follows */
    SUPE_M_REPAIR_TX,      /* resending the frames the peer named */
    SUPE_M_REPAIR_RX,      /* the peer is resending the frames we named */
    SUPE_M_CLOSE_TX,       /* our BYE or RESEND on the air */
};

struct SupeMeet {
    uint8_t  phase;
    bool     listener;             /* we answered the HAVEDATA */
    int8_t   schedIdx;             /* the schedule this meeting consumed */
    uint8_t  hash3[SUPE_HASH_LEN];
    uint8_t  tag[SUPE_TAG_LEN];
    bool     haveTag;
    uint16_t peerId;
    uint8_t  chan;
    uint8_t  sByte;                /* the slot's word byte — the train re-indexes it */
    SupeCfg  slotCfg, cfg;         /* the slot's modulation and the confirmed one */
    uint8_t  budget;
    bool     retuned;              /* off the hailing configuration */
    bool     atTrainCfg;           /* the confirmed budget is on the radio */
    int8_t   ourTxp;               /* our meeting power (HAVEDATA/GIMME stated) */
    int8_t   trainTxp;             /* our train's power — THATSIT states it */
    uint8_t  leg;                  /* 0: opener's train; 1: the return leg */
    /* outgoing */
    SupeTrainInfo tx;
    bool     txBuilt;
    uint8_t  txNext;               /* next frame index to fire */
    uint8_t  txMask[SUPE_MASK_MAX];/* a repair: which frames to refire */
    bool     txMaskAny;
    uint8_t  txThatsit[SUPE_THATSIT_BASE + SUPE_TRAIN_MAX];
    uint8_t  txThatsitLen;
    bool     ourTrainConfirmed;    /* anything of theirs answered our THATSIT */
    bool     laterThatsit;         /* an answering HAVEDATA promised a return
                                    * train, so a THATSIT later than ours will
                                    * close the meeting: the one we hold is not
                                    * the goodbye until that one arrives */
    /* incoming */
    uint8_t  exCount;              /* what the peer's HAVEDATA declared */
    uint8_t  rxN;                  /* frames buffered by the host, arrival order */
    uint8_t  rxCsum[SUPE_TRAIN_MAX];
    uint8_t  rxPos[SUPE_TRAIN_MAX];   /* arrival → sequence position (after align) */
    bool     rxAligned;
    uint8_t  peerCount;            /* the peer train's THATSIT count */
    uint8_t  missMask[SUPE_MASK_MAX];
    uint8_t  missCount;
    uint8_t  repairPos[SUPE_TRAIN_MAX];  /* positions the peer will resend, in order */
    uint8_t  repairExpect, repairGot;
    int16_t  worstRssi;            /* our reading of THEIR train, worst frame */
    int8_t   worstSnrQ;
    bool     anyRx;
    /* The meeting's one line, gathered as it happens. Each side states the
     * power it transmits at and reports what it read, so every direction is a
     * triple: the power sent, the level the far end read, the SNR it read. */
    int8_t   hailTxp;              /* the PRIVSYNC's power — whoever seeded */
    int16_t  hailRssi;             /* and how the other end read it */
    int8_t   hailSnrQ;
    bool     haveHail;
    int16_t  ourTrainRssi;         /* THEIR reading of OUR train */
    int8_t   ourTrainSnrQ;
    bool     haveOurRead;
    int8_t   peerTrainTxp;         /* what their train flew at (their THATSIT) */
    bool     havePeerTxp;
    uint8_t  txFired;              /* frames of ours actually put on air, repairs
                                    * included — so it may exceed tx.count */
    bool     fromHail;             /* reached through a PRIVSYNC's schedule
                                    * rather than a goodbye's rendezvous */
    /* the goodbye */
    uint8_t  lastThatsit[SUPE_THATSIT_BASE + SUPE_TRAIN_MAX];
    uint8_t  lastThatsitLen;
    bool     weReceivedFinal;      /* the final train was theirs → we transmit
                                    * first on the reseeded schedule */
    bool     closeIsFinal;         /* the CLOSE_TX on the air ends the meeting */
    uint8_t  pendClose;            /* what to send once repairs are out: 1 BYE,
                                    * 2 RESEND */
    uint8_t  pendSend;             /* what this TRAIN_WAIT fires instead of a
                                    * train frame (SUPE_PEND_*): a send parked
                                    * one train gap so the receiver it is aimed
                                    * at has flipped back from its own frame */
    uint32_t deadlineMs;
    uint32_t beganMs;
    /* the pending seed (PS_TX) */
    uint8_t  psFrame[SUPE_PRIVSYNC_ID_LEN];
    uint8_t  psLen;
    int8_t   psTxp;
};

/* ─────────────── the engine ─────────────── */
struct SupeEngine {
    const SupeHost* host;
    LoraQueue*      q;

    /* the interface's own facts, refreshed by supeEngConfig */
    uint8_t  regime;
    uint8_t  ownFam;
    uint8_t  ownIdent[SUPE_TAG_LEN];
    bool     haveOwnIdent;
    uint8_t  ownTop;
    int8_t   txpMax;               /* configured tx_power (the hailing cap) */
    uint8_t  hailSf;
    uint32_t hailBwHz;
    uint8_t  crDenom;
    uint16_t preamble;
    uint8_t  ifaceSync;
    uint16_t maxFrameLen;
    bool     expired;              /* the dialect is past its date */

    SupeTag       tags[SUPE_TAGS_MAX];
    SupeProofRet  pret[SUPE_PROOFRET_MAX];
    SupeSched     sched[SUPE_SCHED_MAX];

    SupeMeet m;
    bool     plainOnce;            /* the head packet goes plainly, once */
    bool     offerArmed;           /* verdict said OFFER; the launch waits for the
                                    * pre-seed jitter, then the channel */
    uint32_t offerJitterUntilMs;

    /* what `lora <n> supe` prints */
    uint32_t rxFrames, rxDiscard, rxForeign;
    uint32_t seedsOut, schedsIn, meetingsDone;
    /* Why the head packet last went plain. The verdict is polled continuously,
     * so this is the last one NAMED rather than a count: a packet declining to
     * meet leaves no frame saying so, and the ring shows only an ordinary
     * transmission on the shared channel — identical to the one a node with the
     * feature switched off would make. Repeats are suppressed on the triple,
     * so a steady stream of the same refusal costs one line. */
    uint8_t  plainWhy;
    uint8_t  plainTag[SUPE_TAG_LEN];
    uint16_t plainLen;
    uint32_t schedsAbandoned; /* wide schedules dropped for the shared channel
                               * after their own slots went unanswered (§12) */
    uint32_t framesOut, framesIn, repairsOut, repairsIn;
    uint32_t slotsListened, slotsSpoken, slotsSkipped;
    uint32_t slotsYielded;   /* deferred: a frame was arriving (§7) */
    uint16_t slotsLateOpen;  /* windows opened after the speaker's start — the
                              * preamble was already gone, so the slot was spent
                              * listening to the middle of a frame it could not
                              * lock. A rising count means the first-slot gap is
                              * shorter than this hardware's cold retune. */
    uint32_t strikes, dropsAbsent;

    /* The last meeting ends, newest last — ground truth for `lora <n> supe`
     * that survives a lossy debug log. `why` points at static literals. */
    struct SupeEndRec {
        const char* why;
        uint32_t endedMs;
        uint8_t  phase;
        uint8_t  chan;
        bool     listener, ok;
        uint8_t  sent, got, expect;
    } ends[8];
    uint8_t endsAt;
};

/* The classifier's verdicts, decided on the head of the queue before anything
 * contends for the medium. WAIT leaves the packet queued: a schedule with its
 * peer is live (the packet rides the next met slot), or the absence ladder's
 * randomised pause between seeds is running. */
enum : uint8_t { SUPE_V_PLAIN = 0, SUPE_V_DROP, SUPE_V_OFFER, SUPE_V_WAIT };

/* The one deliberately-unspecified decision (SUPE.md §18): whether to seed.
 * One call site (the classifier); inputs are the peer, the queue and the
 * channels; output is no / now / wait-until. plans/simulation.md §7 owns what
 * it should decide; the v0 policy seeds whenever there is a peer to meet. */
enum { DETOUR_NO = 0, DETOUR_NOW, DETOUR_WAIT };
int shouldDetour(const SupePeerView* peer, const LoraQueue* q,
                 const SupeChanView* chans, uint32_t now,
                 uint32_t* wait_until_ms);

/* lifecycle */
void supeEngInit(SupeEngine* e, const SupeHost* host, LoraQueue* q);
void supeEngConfig(SupeEngine* e, uint8_t regime, uint8_t ownFam, uint8_t ownTop,
                   int8_t txpMax, uint8_t hailSf, uint32_t hailBwHz,
                   uint8_t crDenom, uint16_t preamble, uint8_t ifaceSync,
                   uint16_t maxFrameLen);
void supeEngReset(SupeEngine* e);         /* the radio went away underneath */
void supeEngAbort(SupeEngine* e, const char* why);   /* the watchdog's exit */
bool supeEngBusy(const SupeEngine* e);    /* the engine owns the radio right now */
bool supeEngXactLive(const SupeEngine* e);/* a meeting is under way */
uint16_t supeEngCargoPeer(const SupeEngine* e);
void supeEngSetIdent(SupeEngine* e, const uint8_t id[SUPE_TAG_LEN]);

/* callbacks in */
void supeEngOnRx(SupeEngine* e, const uint8_t* f, uint16_t len,
                 int16_t rssi, int16_t snr10);
/* A non-SUPE frame arrived while the engine held the radio. True: it belongs
 * to the meeting's inbound train — the host buffers it (in arrival order,
 * repairs appended) and will flush on train_deliver. False: not the engine's;
 * the host handles it as ordinary traffic. */
bool supeEngOnTrainFrame(SupeEngine* e, uint8_t csum, int16_t rssi, int16_t snr10);
void supeEngOnTxDone(SupeEngine* e, bool ok);
void supeEngOnTimer(SupeEngine* e);

/* the sender path: the drain classifies the head, wins the channel, seeds */
uint8_t supeEngVerdict(SupeEngine* e);
bool    supeEngLaunchDue(const SupeEngine* e);   /* jitter passed; wants the channel */
void    supeEngLaunch(SupeEngine* e);            /* channel won: emit the PRIVSYNC */

/* The soonest instant the engine wants the clock for — a slot edge, a window
 * close, a meeting deadline, a schedule expiry. UINT32_MAX for none. */
uint32_t supeEngNextEventMs(const SupeEngine* e, uint32_t now);

/* addresses that mean us — fed by the observer through the glue */
void supeEngTagAdd(SupeEngine* e, const uint8_t* addr, bool perm, uint32_t ttlMs);
void supeEngTagRelease(SupeEngine* e, const uint8_t* addr);
void supeEngTagExpire(SupeEngine* e, uint32_t now);
bool supeEngTagIsOurs(const SupeEngine* e, const uint8_t* addr);
void supeEngProofRetFile(SupeEngine* e, const uint8_t phash[16], const uint8_t node4[4]);
const uint8_t* supeEngProofRetLookup(SupeEngine* e, const uint8_t* addr);

/* build our own announcement (the glue paces and transmits it) */
size_t supeEngBuildAnn(SupeEngine* e, uint8_t* out, size_t cap,
                       const uint8_t ids[][SUPE_ID_LEN], uint8_t count,
                       int8_t pwrDbm, bool speaking);

#endif /* IFACE_LORA_SUPE_ENGINE_H */
