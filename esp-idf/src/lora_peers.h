#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

#include "supe.h"

/* ─────────────── passive neighbour table ("Eve") ───────────────
 * Built entirely from observing rx + tx RNS packets on the interface — no rnsd
 * API, no peer cooperation. Only frames whose ORIGINATOR is the transmitting RF
 * neighbour are used: the wire hops byte is incremented by the receiving
 * transport, so a frame fresh from its originator carries hops == 0 on air
 * (µR's hops() reports 1 for the same frame). Announces at hops 0 name the
 * transmitter cryptographically (signature-verified identity key; dest ==
 * H(name_hash ‖ identity_hash) groups dest hashes per device); LR/LRPROOF pairs
 * name links; proofs we elicited close a per-neighbour delivery-quality loop.
 * IFAC frames are masked end-to-end and are skipped (the table stays empty on
 * an IFAC network). Surfaced by `lora [<n>] neighbors`. */
#define NEI_MAX              24      /* neighbour entries per radio */
#define NEI_DESTS_MAX        8       /* dest hashes clustered per node */
#define NEI_IDS_MAX          8       /* identities clustered per node — one
                                      * device legitimately runs several (its
                                      * transport, rnsh, lxmf, rlpg and nomad
                                      * identities are all distinct), and an
                                      * announcement naming them together is
                                      * what folds them into one row.
                                      *
                                      * Overflow evicts the OLDEST, and the
                                      * eviction is what makes this a size worth
                                      * getting right rather than a cap: the
                                      * identity dropped is one a neighbour may
                                      * still be holding, and it is dropped from
                                      * the list this node's own announcement is
                                      * built from — so the pair loses the very
                                      * name that would have folded their rows
                                      * together. Four was set before rnsh and
                                      * lxmf each carried one. Costs 16 bytes
                                      * per slot per row: 2 KB a radio at
                                      * NEI_MAX. */
#define NEI_LINKS_MAX        12      /* observed links per radio */
#define NEI_PEND_MAX         8       /* outstanding proof expectations per radio */
#define NEI_PROOF_TIMEOUT_MS 30000   /* elicited proof must return within this */
#define NEI_BUCKETS          12      /* last-hour rollup: 12 × 5 min */
#define NEI_BUCKET_MS        (5u * 60u * 1000u)
#define NEI_HASHES_MAX       48      /* shared: hashes linked to a node by 0x03 */
/* Silence that means a link is over. Nothing announces a teardown, so this is
 * the only evidence available; generous, because the cost of calling a live
 * link dead is only that it sorts first for eviction. */
#define NEI_LINK_QUIET_MS    (10u * 60u * 1000u)

struct NeiBucket {                  /* one 5-minute rollup slot */
    uint32_t absIdx;                /* millis()/NEI_BUCKET_MS this slot holds */
    uint16_t cnt;
    int32_t  rssiSum;
    int32_t  snrSum10;
};

#define NEI_NAME_MAX 20             /* announced display name, truncated */

struct NeiDest {                    /* one destination hash in a node's cluster */
    uint8_t  hash[16];
    uint8_t  nameHash[10];          /* from the announce; labels the row */
    bool     haveName;
    char     name[NEI_NAME_MAX];    /* display name from the announce app_data */
    uint32_t announces;
    uint32_t lastMs;
};

/* Where an opening power came from, best evidence first. The order is the
 * precedence the controller resolves in, so it compares. */
enum ApSource : uint8_t {
    AP_SRC_NONE = 0,        /* nothing recent enough — the configured tx_power.
                             * Also every node outside SUPE, permanently: both
                             * tiers below need a power the PEER stated */
    AP_SRC_PAIR,            /* reciprocity against a STATED peer power */
    AP_SRC_REPORT,          /* the peer stated the level our own frame landed at */
};

struct Neighbor {
    bool     used;
    bool     isUs;                  /* built from our own tx announces */
    bool     isRnode;               /* built from the RNode client's tx announces —
                                     * a second local row, distinct from ours */
    uint8_t  ids[NEI_IDS_MAX][16];
    uint8_t  nIds;
    NeiDest  dests[NEI_DESTS_MAX];
    uint8_t  nDests;
    /* Signal envelope over rx frames provably transmitted by this node. */
    bool     haveSig;
    int16_t  rssiMin, rssiMax;
    int16_t  snrMin10, snrMax10;
    uint32_t lastHeardMs;
    uint32_t frames;
    /* Link quality: EWMA (0–255) of elicited proofs that came back (LR→LRPROOF,
     * data to a dest that has proven before) — direct dests only. */
    bool     haveQuality;
    uint8_t  quality;
    uint16_t qSent, qProved;
    bool     provesData;            /* has proven a plain data packet (PROVE_ALL) */
    bool     transit;               /* seen rebroadcasting announces (a transport node) */
    /* Hashes learned to belong to this node, held as first-4. `node4` is the
     * node's rnstransport first-4. Both are populated by the identity join. */
    uint8_t  node4[4];
    bool     haveNode4;
    /* Link stubs are NOT here: they live in one shared table on NeiState. A
     * hash that means a node is a property of the pair, and a fixed slice per
     * node spends most of its bytes empty while the one node that opens many
     * links silently loses its oldest. See NeiHash. */
    bool     haveAdv;               /* peer stated a hash count / roaming bit */
    uint8_t  advHashes;
    bool     roaming;
    bool     ourProto;              /* has spoken our air protocol to us */
    /* Adaptive TX power, as last resolved: what `lora <n>` prints, and nothing
     * else. The number is derived per frame at the configuration the frame is
     * about to fly at (lora_power.cpp), so nothing may read it back as state. */
    int8_t   apPwr;
    ApSource apSrc;

#if !defined(CONFIG_LORA_NO_SUPE)
    /* ── SUPE ──
     * A node becomes a SUPE peer by its SUPE_ANNOUNCE2 being heard and by
     * nothing else, which is the whole of the mixed-segment story: traffic to
     * anything that has not announced takes the main channel untouched, so
     * there is no detection to do and no fallback to arrange. */
    bool     supeSeen;
    SupeCaps supeCaps;
    uint32_t supeHeardMs;           /* last ANNOUNCE2 or answered offer — the
                                     * five-minute staleness gate reads this */
#endif
    /* Path loss, never a bare level (SUPE.md §10). Every reading is a pair: a
     * level measured here, and the transmit power the other side states for it
     * one frame later. One pair per configuration — the hailing one, which an
     * ANNOUNCE2 alone supplies, and the step last used.
     *
     * A pair measures THEM→US. The difference between its two numbers is the
     * path loss, which is one property of the link whatever configuration read
     * it, so either pair answers for either direction; what a configuration
     * changes is the floor that loss has to clear, and that the power
     * controller adds itself. The two are kept apart because the step chooser
     * wants to know which is which, and because the fresher of them wins. */
    bool     havePair;
    int16_t  pairRssi;
    int8_t   pairTxp;
    uint32_t pairMs;
    bool     haveStepPair;
    int16_t  stepRssi;
    int8_t   stepTxp;
    uint8_t  stepPairStep;
    uint32_t stepPairMs;
    /* The peer's own account of what OUR frame landed at, from the MANIFEST
     * that closes a detour: the level it read, and the power we sent at. The
     * only measurement of the us→them direction that exists — everything else
     * is reciprocal — so it outranks every pair. */
    bool     haveApRpt;
    int16_t  apRptRssi;
    int8_t   apRptTxp;
    uint32_t apRptMs;
    /* Absence is a provisional verdict, not a finding: it expires, and it is
     * suppressed outright while an overheard START says the node is merely
     * busy. The counter decays so a silent peer is retried occasionally rather
     * than never. */
    uint32_t absentUntilMs;
    uint8_t  silentCount;           /* the ladder's strikes since evidence of life */
    uint32_t retryWaitUntilMs;      /* the randomised wait between requests */
    uint32_t backoffUntilMs;        /* a refusal said how long not to ask */
    bool     detoured;              /* a detour to it has been answered at least
                                     * once — what lifts the first-offer cap */
    /* The ratchet (§15): dB of trim below a MEASURED need. It moves one dB per
     * AP_MIN_SAMPLES clean exchanges and returns to zero on a miss, where the
     * floor takes over. It is what the loop learns and measurement cannot
     * reach: the far end's noise floor, antenna and front-end differences. */
    int8_t   apOffsetDb;
    bool     haveApFloor;
    int8_t   apFloorDbm;
    uint32_t apFloorDecayMs;
    uint16_t apSuccess;             /* successful exchanges since the offset moved */
    bool     haveApLastTxp;         /* the power the last train to it went out at,
                                     * which is what a missing delivery signal is
                                     * evidence against */
    int8_t   apLastTxp;

    NeiBucket buck[NEI_BUCKETS];    /* last-hour rollup ring */
};

struct NeiLink {
    bool     used;
    uint8_t  linkId[16];
    bool     haveDest;
    uint8_t  dest[16];              /* the LR's destination */
    bool     ours;                  /* we are an endpoint (initiated or host) */
    bool     established;           /* LRPROOF seen */
    bool     unresolved;            /* discovered from mid-link traffic, LR missed */
    bool     haveSig;
    int16_t  lastRssi, lastSnr10;   /* last rx frame on this link */
    uint32_t lastMs;
    uint32_t frames;
    /* A 0x04 power request the peer bound to this link's setup. A link is
     * identifiable, so unlike an ad-hoc exchange it may legally hold state
     * about a peer it cannot otherwise name — the request covers the session. */
    bool     haveSuggest;
    int8_t   suggestDbm;
    /* SUPE state filed against the *link* rather than a node, for the one peer
     * we cannot name: the node that dialled this link to us. An inbound request
     * carries no sender, so there is no neighbour row to hang capabilities on —
     * but the link identifier is a handle both ends share, and a MANIFEST
     * carries the sender's capabilities unconditionally. That is enough to
     * offer a detour back, which is what SUPE.md §10's "links inherit" asks
     * for and what otherwise leaves reverse traffic on the shared channel. */
#if !defined(CONFIG_LORA_NO_SUPE)
    bool     supeSeen;
    SupeCaps supeCaps;
    uint32_t supeHeardMs;
#endif
    bool     havePair;
    int16_t  pairRssi;
    int8_t   pairTxp;
};

struct NeiPend {                    /* an outstanding proof expectation */
    bool     used;
    bool     isLR;
    bool     counted;               /* a miss scores against quality */
    uint8_t  phash[16];             /* proofs are addressed to this (truncated
                                     * packet hash; link_id for an LR) */
    uint8_t  dest[16];              /* the elicitor's destination */
    uint32_t deadlineMs;
};

/* Unattributed relayed traffic: every rx frame at wire hops ≥ 1 was, by
 * definition, transmitted by an in-range transport node — even when nothing
 * names it (HEADER_1 relays, relayed proofs, a silent access-point bridge).
 * One aggregate row per radio; anonymous transmitters can't be told apart. */
struct NeiAnon {
    uint32_t frames;
    uint32_t inbandRelays;          /* packet re-heard one hop later: RF→RF repeat */
    bool     haveSig;
    int16_t  rssiMin, rssiMax;
    int16_t  snrMin10, snrMax10;
    uint32_t lastMs;
};

#define NEI_SEEN_MAX     16         /* recent-rx ring for relay coupling */
#define NEI_SEEN_WIN_MS  30000      /* relay must re-appear within this */

struct NeiSeen {
    uint8_t  hash[16];              /* truncated packet hash (hops-invariant) */
    uint8_t  hops;
    uint32_t ms;
};

/* One hash that means one node — a link identifier, above all. Shared across
 * every node rather than sliced per node: which node a hash belongs to is a
 * fact about the pair, and a per-node array of twelve spends nearly all of its
 * bytes empty while the single node that opens a thirteenth link quietly loses
 * its oldest. One table sized to what a radio actually sees is both smaller
 * and never drops a resolution it still needs.
 *
 * `timedOut` is a link that has gone silent long enough to call it over.
 * Nothing on the air announces a link ending — no close is ever observed — so
 * silence is the only signal there is. The row STAYS when it trips: a frame
 * recorded an hour ago still has to resolve to the node it was for, and LoRaMon
 * reads back exactly that far. Only a full table evicts anything, and it takes
 * the timed-out rows first. */
struct NeiHash {
    bool     used;
    bool     timedOut;              /* silent long enough to call it over */
    uint8_t  hash4[4];
    uint8_t  node;                  /* index into nei[] */
    uint32_t lastMs;
};

struct NeiState {
    Neighbor nei[NEI_MAX];
    NeiHash  hashes[NEI_HASHES_MAX];
    NeiLink  links[NEI_LINKS_MAX];
    NeiPend  pend[NEI_PEND_MAX];
    NeiAnon  anon;
    NeiSeen  seen[NEI_SEEN_MAX];
    uint8_t  seenNext;
    uint32_t sinceMs;               /* millis() at first allocation */
};

typedef void (*PeersVisitFn)(Neighbor* e, int num, void* ud);

/* A row that is one of this device's own two local endpoints — us, or the
 * attached RNode client — rather than a node out on the air. Every RF-layer
 * guard that means "this traffic terminates at our transmitter" tests this. */
static inline bool peersIsLocal(const Neighbor* e) { return e->isUs || e->isRnode; }

/* The stable id a queued packet carries for its peer: the row's index. */
static inline uint16_t peersIdOf(const NeiState* st, const Neighbor* e) {
    return (uint16_t)(e - st->nei);
}

/* The row that id names, or null for LORAQ_PEER_NONE and anything out of range
 * or since retired. */
static inline Neighbor* peersById(NeiState* st, uint16_t id) {
    if (!st || id >= NEI_MAX) return nullptr;
    Neighbor* e = &st->nei[id];
    return e->used ? e : nullptr;
}

/* ─────────────── lora_peers: the neighbour/peer table ─────────────── */
Neighbor* peersFindByIdentity(NeiState* st, const uint8_t id[16]);
Neighbor* peersFindByDest(NeiState* st, const uint8_t dest[16]);
bool      peersDestIsLocal(NeiState* st, const uint8_t dest[16]);
Neighbor* peersFindClaim4(NeiState* st, const uint8_t b4[4]);
Neighbor* peersAlloc(NeiState* st, uint32_t now);
NeiDest*  peersAddDest(NeiState* st, Neighbor* e, const uint8_t dest[16], uint32_t now);
Neighbor* peersEnsureDest(NeiState* st, const uint8_t dest[16], uint32_t now);
void      peersSample(Neighbor* e, int16_t rssi, int16_t snr10, uint32_t now);
void      peersQuality(LoraRadio* r, Neighbor* e, bool hit);
NeiLink*  peersLinkFind(NeiState* st, const uint8_t linkId[16]);
NeiLink*  peersLinkFindBy3(NeiState* st, const uint8_t b3[3]);
NeiLink*  peersLinkEnsure(NeiState* st, const uint8_t linkId[16], uint32_t now);
void      peersPendAdd(NeiState* st, const uint8_t phash[16], const uint8_t dest[16],
                     bool isLR, bool counted, uint32_t now);
NeiPend*  peersPendTake(NeiState* st, const uint8_t phash[16]);
void      peersAddId(Neighbor* e, const uint8_t id[16]);
/* File a link identifier on a row: an address that resolves to this node for as
 * long as the entry survives, which is what lets traffic on the link detour. */
/* A linkage frame says this hash means that node. Kept in the shared store, so
 * it needs the state rather than the row. */
void      peersAddLink4(NeiState* st, Neighbor* e, const uint8_t lid[16], uint32_t now);
void      peersMergeInto(NeiState* st, Neighbor* dst, Neighbor* src);

/* ── the shared hash store ──
 * A hash that means a node — a link identifier above all. `len` is how many
 * leading bytes to match, so a SUPE 3-byte tag and a 4-byte stub hit the same
 * rows. Adding when the table is full evicts the least recently used, and a
 * dead row goes before a live one. */
NeiHash*  peersHashFind(NeiState* st, const uint8_t* b, int len);
void      peersHashAdd(NeiState* st, Neighbor* e, const uint8_t hash[16], uint32_t now);
void      peersHashDrop(NeiState* st, const uint8_t b4[4]);
void      peersHashTouch(NeiState* st, const uint8_t b4[4], uint32_t now);
/* Mark every hash that has been silent past NEI_LINK_QUIET_MS. */
void      peersHashAge(NeiState* st, uint32_t now);
int       peersHashCount(const NeiState* st, const Neighbor* e);
/* Copy the `n`th hash held for a node into `out`; false past the end. */
bool      peersHashAt(const NeiState* st, const Neighbor* e, int n, uint8_t out[4]);
Neighbor* peersFindBy4(NeiState* st, const uint8_t b4[4]);
Neighbor* peersFindByIdent4(NeiState* st, const uint8_t b4[4]);
int       peersKnownHashes(const NeiState* st, const Neighbor* e);

/* What to call a node: the FIRST WORD of each announced name it holds, comma
 * joined, duplicates dropped. An LXMF name is written for a human reading one
 * message ("tdeck — lab bench, do not unplug"); what identifies the node across
 * a graph or a packet list is the head of it, and the rest is caption. Empty
 * when nothing has announced a name. */
void      peersNodeNames(const Neighbor* e, char* out, size_t outLen);
void      peersExpire(LoraRadio* r, uint32_t now);
Neighbor* peersWalk(NeiState* st, int want, PeersVisitFn fn, void* ud);
bool      peersNodeFirst4(const Neighbor* e, uint8_t out[4]);
bool      peersEstimateCliff10(const LoraRadio* r, const Neighbor* e,
                             uint32_t now, int* cliff10, uint32_t* samples,
                             uint32_t* buckets);
void      peersInit(LoraRadio* r);
void      peersAbandonPends(LoraRadio* r);
