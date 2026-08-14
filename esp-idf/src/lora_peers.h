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
#define NEI_IDS_MAX          4       /* identities clustered per node — one
                                      * device legitimately runs several (its
                                      * transport, rnsh and lxmf identities are
                                      * all distinct), and a 0x03 is what folds
                                      * them into one row */
#define NEI_LINKS_MAX        12      /* observed links per radio */
#define NEI_PEND_MAX         8       /* outstanding proof expectations per radio */
#define NEI_PROOF_TIMEOUT_MS 30000   /* elicited proof must return within this */
#define NEI_BUCKETS          12      /* last-hour rollup: 12 × 5 min */
#define NEI_BUCKET_MS        (5u * 60u * 1000u)
#define NEI_LINK4_MAX        12      /* first-4 hashes linked to a node by 0x03 */

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
    uint8_t  link4[NEI_LINK4_MAX][4];
    uint8_t  nLink4;
    bool     haveAdv;               /* peer stated a hash count / roaming bit */
    uint8_t  advHashes;
    bool     roaming;
    bool     ourProto;              /* has spoken our air protocol to us */
    /* Adaptive TX power: the single power every frame to this node goes out at,
     * and where that number came from. Settled once and kept (see AP_ below). */
    bool     haveApPwr;
    int8_t   apPwr;
    bool     apFromEst;             /* derived from reciprocity, not measured */

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
     * ANNOUNCE2 alone supplies, and the step last used. */
    bool     havePair;
    int16_t  pairRssi;
    int8_t   pairTxp;
    bool     haveStepPair;
    int16_t  stepRssi;
    int8_t   stepTxp;
    uint8_t  stepPairStep;
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
    /* Adaptive power over SUPE (§15). Power is derived, not stored:
     * clamp(path loss + step margin + offset, floor, maximum). The offset is
     * what the loop learns and measurement cannot reach — the far end's noise
     * floor, antenna and front-end differences. */
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

struct NeiState {
    Neighbor nei[NEI_MAX];
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
NeiDest*  peersAddDest(Neighbor* e, const uint8_t dest[16], uint32_t now);
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
void      peersAddLink4(Neighbor* e, const uint8_t lid[16]);
void      peersMergeInto(Neighbor* dst, Neighbor* src);
Neighbor* peersFindBy4(NeiState* st, const uint8_t b4[4]);
Neighbor* peersFindByIdent4(NeiState* st, const uint8_t b4[4]);
int       peersKnownHashes(const Neighbor* e);
void      peersExpire(LoraRadio* r, uint32_t now);
Neighbor* peersWalk(NeiState* st, int want, PeersVisitFn fn, void* ud);
bool      peersNodeFirst4(const Neighbor* e, uint8_t out[4]);
bool      peersEstimateCliff10(const LoraRadio* r, const Neighbor* e,
                             uint32_t now, int* cliff10, uint32_t* samples);
void      peersInit(LoraRadio* r);
void      peersAbandonPends(LoraRadio* r);
