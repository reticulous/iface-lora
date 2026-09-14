/**
 * lora_observe — Reticulum packet inspection: the wire-header parse, the
 * per-packet observation tap that fills the peer table and the SUPE tag set,
 * and the announce ingest with its cryptographic identity join. One direction,
 * no decisions. The one Reticulum-specific module.
 */
#include "lora_priv.h"

#if defined(CONFIG_LORA0_CS_PIN)

/* ─────────────── per-packet debug trace (`log lora debug`) ───────────────
 *
 * Decodes the Reticulum header of a whole (reassembled) RNS frame into a
 * one-line summary at debug level; at verbose level the entire frame is also
 * dumped as hex. This traces the RNS packet, not the on-air split frame — so
 * it runs at the reassembly boundary (deliverInbound for rx, beginTx for
 * tx), free of the local 1-byte split header.
 *
 * RNS wire header (RNS/Packet.py):
 *   byte0 flags: [IFAC 0x80][hdr2 0x40][ctxflag 0x20][transport 0x10]
 *                [dest-type 0x0C][packet-type 0x03]
 *   byte1 hops
 *   [IFAC access code: ifac_size bytes, present iff IFAC flag set]
 *   [transport-id: 16 bytes, present iff hdr2 (HEADER_2)]
 *   [destination hash: 16 bytes]
 *   [context: 1 byte]
 *   [data ...]
 */

void loraHex(char* out, const uint8_t* d, size_t n) {
    static const char H[] = "0123456789abcdef";
    for (size_t i = 0; i < n; i++) { out[2*i] = H[d[i] >> 4]; out[2*i+1] = H[d[i] & 0xF]; }
    out[2*n] = '\0';
}

static const char* loraPktType(uint8_t t) {   /* flags & 0x03 */
    switch (t) { case 0: return "data"; case 1: return "announce";
                 case 2: return "linkreq"; default: return "proof"; }
}
static const char* loraDestType(uint8_t t) {   /* (flags >> 2) & 0x03 */
    switch (t) { case 0: return "single"; case 1: return "group";
                 case 2: return "plain"; default: return "link"; }
}
static const char* loraCtx(uint8_t c) {        /* context byte; NULL = unknown */
    switch (c) {
        case 0x00: return nullptr;                 /* none — omitted from the line */
        case 0x01: return "resource";
        case 0x02: return "resource-adv";
        case 0x03: return "resource-req";
        case 0x04: return "resource-hmu";
        case 0x05: return "resource-prf";
        case 0x06: return "resource-icl";
        case 0x07: return "resource-rcl";
        case 0x08: return "cache-req";
        case 0x09: return "request";
        case 0x0a: return "response";
        case 0x0b: return "path-resp";
        case 0x0c: return "command";
        case 0x0d: return "command-status";
        case 0x0e: return "channel";
        case 0xfa: return "keepalive";
        case 0xfb: return "link-identify";
        case 0xfc: return "link-close";
        case 0xfd: return "link-proof";
        case 0xfe: return "link-rtt";
        case 0xff: return "link-req-proof";
        default:   return nullptr;                 /* unknown → flag the line */
    }
}

/* Full frame as an offset-prefixed hexdump, 16 bytes/line, no ASCII. Verbose
 * only — the level check guards the per-row formatting cost. */
static void loraHexdump(int idx, const char* dir, const uint8_t* p, size_t len) {
    if (esp_log_level_get(TAG) < ESP_LOG_VERBOSE) return;
    for (size_t off = 0; off < len; off += 16) {
        char row[16 * 3 + 8];
        int o = snprintf(row, sizeof row, "%04x", (unsigned)off);
        for (size_t i = 0; i < 16 && off + i < len; i++)
            o += snprintf(row + o, sizeof row - (size_t)o, " %02x", p[off + i]);
        verb("lora/%d %s %s", idx, dir, row);
    }
}

/* Trace one whole RNS frame, gated on `log lora debug`. haveQual: fold in the
 * radio's last rx rssi/snr (rx); false on tx. logIsDebug short-circuits the
 * decode when off, so this is free on the hot path in normal operation.
 * Retained for future RNS-level inspection but no longer wired into rx/tx —
 * LoRaMon logs per on-air frame instead (loraMonPush). */
__attribute__((unused))
static void loraTracePacket(LoraRadio* r, const char* dir,
                            const uint8_t* p, size_t len, bool haveQual,
                            double airMs, int frames) {
    if (!logIsVerbose(TAG)) return;   /* a frame trace, so it lives at verbose */

    char qual[24] = "";
    if (haveQual)
        snprintf(qual, sizeof qual, " %ddBm snr%.1f",
                 (int)r->rssiLast, (double)r->snrLast);

    /* Computed airtime of the on-air frame(s) at the live modem params; a
     * 2-frame split shows its per-frame count so the doubled preamble is clear. */
    char air[24] = "";
    if (frames == 2) snprintf(air, sizeof air, " air=%.0fms/2frm", airMs);
    else             snprintf(air, sizeof air, " air=%.0fms", airMs);

    bool   ifac   = len >= 1 && (p[0] & 0x80);
    bool   hdr2   = len >= 1 && (p[0] & 0x40);
    size_t ifacB  = ifac ? (r->curIfacSize ? r->curIfacSize : 1u) : 0;
    size_t addrB  = hdr2 ? 32 : 16;                 /* [transport 16] + dest 16 */
    size_t hdrEnd = 2 + ifacB + addrB + 1;          /* through the context byte */

    /* Too short to hold a header — decode nothing, show the first ≤10 bytes so
     * a stray/foreign frame is still visible. */
    if (len < 2 || len < hdrEnd) {
        size_t s = len < 10 ? len : 10;
        char hx[21];
        loraHex(hx, p, s);
        dbg("lora/%d %s%s%s <unparsed %uB> %s", r->idx, dir, qual, air, (unsigned)len, hx);
        loraHexdump(r->idx, dir, p, len);
        return;
    }

    uint8_t  flags     = p[0];
    uint8_t  hops      = p[1];
    bool     transport = flags & 0x10;              /* 0 broadcast, 1 transport */
    uint8_t  dtype     = (flags >> 2) & 0x03;
    uint8_t  ptype     = flags & 0x03;
    const uint8_t* dest = p + 2 + ifacB + (hdr2 ? 16 : 0);
    uint8_t  ctx       = p[2 + ifacB + addrB];

    char destHex[33]; loraHex(destHex, dest, 16);

    char via[48] = "";
    if (hdr2) { char v[33]; loraHex(v, p + 2 + ifacB, 16); snprintf(via, sizeof via, " via %s", v); }

    bool anomaly = hops > 128;                      /* RNS caps hops at 128 */
    char ctxbuf[24] = "";
    if (ctx != 0x00) {
        const char* cn = loraCtx(ctx);
        if (cn) snprintf(ctxbuf, sizeof ctxbuf, " ctx=%s", cn);
        else { snprintf(ctxbuf, sizeof ctxbuf, " ctx=0x%02x", ctx); anomaly = true; }
    }

    dbg("lora/%d %s%s%s %s %s %s %s%s%s hops=%u%s",
        r->idx, dir, qual, air,
        loraPktType(ptype), transport ? "to" : "bcast",
        loraDestType(dtype), destHex, via, ctxbuf,
        (unsigned)hops, anomaly ? " ?" : "");

    loraHexdump(r->idx, dir, p, len);
}

/* ─────────────── passive neighbour table: implementation ───────────────
 *
 * Everything below runs on the lora task (observe/expire) except the CLI
 * printer, which — like cliPrintSlot — reads live state cross-task without a
 * lock; a torn row during heavy traffic is acceptable for a diagnostic view. */

/* Decoded RNS wire header (layout in the loraTracePacket comment above). */
/* IFAC frames (flag 0x80) are masked from byte 2 on — unparseable, skipped. */
bool rnsParse(const uint8_t* p, size_t len, RnsHdr* h) {
    if (len < 2 || (p[0] & 0x80)) return false;
    h->hdr2 = (p[0] & 0x40) != 0;
    size_t hdrEnd = 2 + (h->hdr2 ? 32 : 16) + 1;
    if (len < hdrEnd) return false;
    h->hops    = p[1];
    h->ptype   = p[0] & 0x03;
    h->dtype   = (p[0] >> 2) & 0x03;
    h->ctxflag = (p[0] & 0x20) != 0;
    h->transportId = h->hdr2 ? p + 2 : nullptr;
    h->dest    = p + 2 + (h->hdr2 ? 16 : 0);
    h->ctx     = p[hdrEnd - 1];
    h->data    = p + hdrEnd;
    h->dataLen = len - hdrEnd;
    return true;
}

/* Truncated packet hash — what a proof is addressed to. Hashable part is
 * [flags & 0x0F] + raw[2:] (HEADER_1) / raw[18:] (HEADER_2, transport_id
 * excluded); for an LR the link_id additionally drops any data beyond the
 * 64-byte ephemeral keys (MTU signalling). Static buffer: lora task only. */
void rnsPacketHash(const RnsHdr* h, const uint8_t* p, size_t len,
                   bool isLr, uint8_t out[16]) {
    static uint8_t buf[1 + RNS_MTU + 16];
    size_t skip = h->hdr2 ? 18 : 2;
    size_t n = len - skip;
    if (isLr && h->dataLen > NEI_ECPUBSIZE) n -= h->dataLen - NEI_ECPUBSIZE;
    buf[0] = p[0] & 0x0F;
    memcpy(buf + 1, p + skip, n);
    uint8_t sha[RNSD_HASH_LEN];
    rnsdSha256(buf, 1 + n, sha);
    memcpy(out, sha, 16);
}

/* The aspect behind an announce's name hash. rnsd owns the dictionary — it sees
 * every announce on every medium, and a name hash is one-way, so the table of
 * names this firmware speaks is the only way back. Keeping a second copy here
 * is how `lora n` came to print a raw hash for netgraph.discovery while every
 * other medium named it: the copy was never updated. There is one table now. */
const char* rnsNameLabel(const uint8_t nameHash[10]) {
    return rnsdAspectLabel(nameHash);
}


/* Display name out of an announce's app_data — rnsd's decoder, for the same
 * reason the aspect dictionary above is: app_data is bytes an application chose
 * and a name is only what survives being checked as text, so the node that sees
 * every announce on every medium owns the rule. A second copy here drifts, and
 * a drifted copy shows a different name on this pane than on every other
 * surface of the same device. */
static inline void rnsParseName(const uint8_t* p, size_t n, char* out, size_t outsz) {
    rnsdAnnounceName(p, n, out, outsz);
}


/* ── announce ingest: the identity join ── */

static void observeAnnounce(LoraRadio* r, const RnsHdr* h, bool isTx,
                        int16_t rssi, int16_t snr10, uint32_t now,
                        uint8_t txOrigin) {
    NeiState* st = r->nei;
    /* pubkey(64) | name_hash(10) | random_hash(10) | [ratchet(32)] |
     * signature(64) | app_data. The ratchet is present iff the header's
     * context flag is set, and every destination rnsd hosts for a consumer
     * announces one — so a parser that assumes it away sees nothing but the
     * transport probe. */
    const size_t   ratLen = h->ctxflag ? NEI_RATCHETSIZE : 0;
    if (h->dataLen < 64 + 10 + 10 + ratLen + 64) return;
    const uint8_t* pub    = h->data;
    const uint8_t* nameH  = h->data + 64;
    const uint8_t* randH  = h->data + 74;
    const uint8_t* rat    = ratLen ? h->data + 84 : nullptr;
    const uint8_t* sig    = h->data + 84 + ratLen;
    const uint8_t* appD   = h->data + 148 + ratLen;
    size_t         appLen = h->dataLen - 148 - ratLen;

    /* The join is cryptographic or it is nothing: identity = H(pubkey)[:16],
     * the dest must equal H(name_hash ‖ identity)[:16], and the announce
     * signature must verify under that key — same checks as µR's
     * validate_announce, minus the cache. */
    uint8_t sha[RNSD_HASH_LEN], idh[16];
    rnsdSha256(pub, 64, sha);
    memcpy(idh, sha, 16);
    uint8_t mat[26];
    memcpy(mat, nameH, 10);
    memcpy(mat + 10, idh, 16);
    rnsdSha256(mat, 26, sha);
    if (memcmp(sha, h->dest, 16) != 0) return;
    /* signed_data; lora task only */
    static uint8_t sd[16 + 64 + 10 + 10 + NEI_RATCHETSIZE + RNS_MTU];
    size_t o = 0;
    memcpy(sd + o, h->dest, 16); o += 16;
    memcpy(sd + o, pub, 64);     o += 64;
    memcpy(sd + o, nameH, 10);   o += 10;
    memcpy(sd + o, randH, 10);   o += 10;
    if (rat) { memcpy(sd + o, rat, NEI_RATCHETSIZE); o += NEI_RATCHETSIZE; }
    memcpy(sd + o, appD, appLen); o += appLen;
    if (!rnsdVerify(pub, sd, o, sig)) return;

    Neighbor* e = peersFindByIdentity(st, idh);
    Neighbor* d = peersFindByDest(st, h->dest);
    if (!e && d && d->nIds == 0) {
        /* Dest-only entry (seen via LRPROOF/proof before any announce) — the
         * announce names its identity now. */
        peersAddId(d, idh);
        e = d;
    } else if (e && d && e != d && d->nIds == 0) {
        /* Same device split across an identity entry and a dest-only entry. */
        peersMergeInto(st, e, d);
    }
    if (!e) {
        e = peersAlloc(st, now);
        if (!e) return;
    }
    peersAddId(e, idh);
    if (isTx) {
        if (txOrigin == LORA_ORIG_RNODE) e->isRnode = true;
        else                             e->isUs    = true;
    }
    if (peersIsLocal(e)) {
        /* Each of our own identities announces separately and so builds its own
         * row, but they are all one device by construction — no 0x03 needed,
         * and none would ever arrive, since we don't hear ourselves. The RNode
         * client's identities fold the same way into its own row. Per flag, so
         * the two local rows stay two. */
        for (int i = 0; i < NEI_MAX; i++) {
            Neighbor* o = &st->nei[i];
            if (o == e || !o->used) continue;
            if ((e->isUs && o->isUs) || (e->isRnode && o->isRnode)) peersMergeInto(st, e, o);
        }
    }

    /* A 0x03 may have claimed this dest before it ever announced, in which case
     * the claiming row and this one are one device: the linkage frame said so,
     * and the announce has now supplied the hash it only held a stub for. Fold,
     * so the aspect joins the node instead of starting a row of its own. Same
     * us/them guard as neiLink(): a peer's claim must not reach our row. */
    if (!peersIsLocal(e)) {
        Neighbor* c = peersFindClaim4(st, h->dest);
        if (c && c != e && !peersIsLocal(c)) peersMergeInto(st, e, c);
        /* And the same by identity: a SUPE announcement heard before this one
         * files what it knows — four bytes of an identity and the radio's
         * capabilities — against a claim row, and this is the frame that
         * supplies the identity itself. */
        c = peersFindClaim4(st, idh);
        if (c && c != e && !peersIsLocal(c)) peersMergeInto(st, e, c);
    }

    NeiDest* nd = peersAddDest(st, e, h->dest, now);
    /* Before the counter moves: zero means this interface has never carried an
     * announce for this destination. A dest row may already exist from a
     * linkage frame or a claim, so it is the announce count and not the row
     * that says whether the announcement itself is new. */
    bool firstAnn = nd->announces == 0;
    memcpy(nd->nameHash, nameH, 10);
    nd->haveName = true;
    nd->announces++;
    nd->lastMs = now;
    {   /* Only the naming aspects carry a human name in app_data. */
        const char* lbl = rnsNameLabel(nameH);
        if (lbl && (strcmp(lbl, "lxmf.delivery") == 0 ||
                    strcmp(lbl, "nomadnetwork.node") == 0)) {
            char nm[NEI_NAME_MAX];
            rnsParseName(appD, appLen, nm, sizeof nm);
            if (nm[0]) safeStrncpy(nd->name, nm, sizeof nd->name);
        }
    }
    if (isTx) e->lastHeardMs = now;   /* keep the us row fresh; no rx signal */
    else      peersSample(e, rssi, snr10, now);
    if (!isTx && !peersIsLocal(e)) {
        /* THE ATTRIBUTION. An announce is the one frame whose transmitter this
         * table can name outright — it verified the signature and joined the
         * destination to a row — and it is the only frame rnsd's neighbourhood
         * is built from. So this is where a shared radio can tell rnsd what
         * every point-to-point medium gets for free: which node these
         * destinations belong to. Every join this table makes ends in this one
         * row, so grouping by it groups exactly as `lora n` does. */
        st->lastObs = peersIdOf(st, e);
        st->lastObsValid = true;
        /* The name may have moved with this announce, and re-declaring is how
         * that reaches rnsd's label. */
        e->rnsdDecl = false;
    }
    /* An announce this radio had never put on air just went out — a
     * destination of ours announcing for the first time, or somebody else's
     * that we relayed. Say who we are behind it, once the burst has passed. */
#if !defined(CONFIG_LORA_NO_SUPE)
    if (isTx && firstAnn) supeAnnSoon(r);
#else
    (void)firstAnn;
#endif
}

/* ── the per-packet observation tap ── */

void peersObserve(LoraRadio* r, const uint8_t* p, size_t len, bool isTx,
                       int16_t rssi, int16_t snr10, uint8_t txOrigin,
                       uint16_t fromPeer) {
    NeiState* st = r->nei;
    if (!st) return;
    /* Nothing attributed until something is. The caller reads this back to tell
     * rnsd who transmitted the packet it is about to hand on, and an unparsed or
     * merely overheard frame must not inherit the last one's answer. */
    st->lastObsValid = false;
    RnsHdr h;
    if (!rnsParse(p, len, &h)) return;
    uint32_t now = millis();

    /* Any HEADER_2 frame arriving at hops > 0 names its relayer in the
     * transport_id — the node forwarded someone else's packet to us, which is
     * transport behaviour whether or not it was an announce. */
    if (!isTx && h.hdr2 && h.hops > 0) {
        Neighbor* tr = peersFindBy4(st, p + 2);
        if (tr && !tr->isUs) tr->transit = true;
    }

    if (!isTx) {
        /* In-band relay coupling: the truncated packet hash is invariant
         * across relaying (hops and transport_id are excluded from the
         * hashable part), so the same hash re-heard one hop higher means a
         * node in range repeated it RF→RF — transport mode, confirmed from
         * the outside. Our own tx never enters the ring, so relaying we do
         * ourselves doesn't self-count. */
        uint8_t ph[16];
        rnsPacketHash(&h, p, len, false, ph);
        for (int i = 0; i < NEI_SEEN_MAX; i++) {
            NeiSeen* sn = &st->seen[i];
            if (sn->ms && (uint8_t)(sn->hops + 1) == h.hops &&
                now - sn->ms < NEI_SEEN_WIN_MS && memcmp(sn->hash, ph, 16) == 0) {
                st->anon.inbandRelays++;
                break;
            }
        }
        NeiSeen* sn = &st->seen[st->seenNext];
        st->seenNext = (uint8_t)((st->seenNext + 1) % NEI_SEEN_MAX);
        memcpy(sn->hash, ph, 16);
        sn->hops = h.hops;
        sn->ms = now ? now : 1;

        /* A relayed frame's transmitter is an in-range transport node even
         * when nothing names it. A rebroadcast announce (HEADER_2) is
         * attributed to the relayer's own row below — but only when we already
         * know that identity from something it signed; when we do not, it lands
         * here with every other relayed frame. */
        bool attributed = h.ptype == NEI_PT_ANNOUNCE && h.hdr2 && !isTx &&
                          peersFindByIdentity(st, h.transportId) != nullptr;
        if (h.hops >= 1 && !attributed) {
            NeiAnon* a = &st->anon;
            if (!a->haveSig || rssi < a->rssiMin)    a->rssiMin  = rssi;
            if (!a->haveSig || rssi > a->rssiMax)    a->rssiMax  = rssi;
            if (!a->haveSig || snr10 < a->snrMin10)  a->snrMin10 = snr10;
            if (!a->haveSig || snr10 > a->snrMax10)  a->snrMax10 = snr10;
            a->haveSig = true;
            a->frames++;
            a->lastMs = now;
        }
    }

    switch (h.ptype) {

    case NEI_PT_ANNOUNCE:
        if (h.hops == 0) {
            /* An announce we transmit at hop zero names one of our own
             * destinations — the first and largest class of addresses that mean
             * us, and the one that is effectively never retired because every
             * re-announcement refreshes it. */
#if !defined(CONFIG_LORA_NO_SUPE)
            if (isTx) supeTagAdd(r, h.dest, /*perm=*/true, 0);
#endif
            observeAnnounce(r, &h, isTx, rssi, snr10, now, txOrigin);
        } else if (!isTx && h.hdr2) {
            /* A rebroadcast announce is the one hops>0 frame whose transmitter
             * IS named: the rebroadcaster stamps its own identity hash as the
             * HEADER_2 transport_id (that is how path tables learn first_hop).
             * Attribute the signal to that transit neighbour, keyed by identity
             * so its own hops-0 announces (if any) land in the same row. The
             * announce signature covers the originator, not the relayer, so
             * this is unverified — the same trust the path table places in it. */
            /* Only against a row that already exists. The announce signature
             * covers the ORIGINATOR, not the relayer, so the transport_id is an
             * unverified claim about who transmitted — enough to attribute a
             * signal to a node we have otherwise met, not enough to mint one.
             * A row conjured from it holds nothing but that claim: no
             * destination, no announce, nothing it ever signed, and it appears
             * in the neighbourhood as a node that may not exist. Unattributed,
             * the frame counts in the anonymous-transit row above, which is
             * exactly what that row is for. Once the relayer announces for
             * itself the row is real, and every later rebroadcast attributes. */
            Neighbor* e = peersFindByIdentity(st, h.transportId);
            if (e && !e->isUs) {
                e->transit = true;
                peersSample(e, rssi, snr10, now);
            }
        } else if (isTx && h.hdr2) {
            /* Our own rebroadcast stamps OUR transport identity as the
             * transport_id — the exact frame neighbours identify us by, so
             * learn "who we are" from it symmetrically. This is the only way
             * the transport identity surfaces here at all: it hangs off no
             * destination rnsd announces, so no announce of ours ever names
             * it. It belongs on the local row as one more name this device
             * answers to, not on a row of its own. */
            Neighbor* e = peersFindByIdentity(st, h.transportId);
            if (!e) {
                /* It is not a second node: the transport identity is another
                 * name for the endpoint this frame came from. File it on that
                 * endpoint's row. Minting one instead would list a second `us`
                 * holding nothing but an identity nobody has announced — a row
                 * with no hash to print — until our next own announce folded it
                 * away. Only if the endpoint has no row yet does one get made:
                 * we can relay before we have ever announced, and the fold in
                 * observeAnnounce joins the two when we do. */
                bool rnode = (txOrigin == LORA_ORIG_RNODE);
                for (int i = 0; i < NEI_MAX; i++) {
                    Neighbor* o = &st->nei[i];
                    if (o->used && (rnode ? o->isRnode : o->isUs)) { e = o; break; }
                }
                if (!e) e = peersAlloc(st, now);
                if (e) peersAddId(e, h.transportId);
            }
            if (e) {
                if (txOrigin == LORA_ORIG_RNODE) e->isRnode = true;
                else                             e->isUs    = true;
                e->transit = true;
                e->lastHeardMs = now;
            }
            /* An announce we relay has our own transport identity in its first
             * address field. That identity is the address every neighbour
             * relaying through us sends to, so it is the single most valuable
             * entry in the set — and, like our destinations, never retired. */
#if !defined(CONFIG_LORA_NO_SUPE)
            supeTagAdd(r, h.transportId, /*perm=*/true, 0);
#endif
        }
        break;

    case NEI_PT_LINKREQ: {
        if (h.dtype != NEI_DT_SINGLE) break;
        uint8_t lid[16];
        rnsPacketHash(&h, p, len, true, lid);
        NeiLink* L = peersLinkEnsure(st, lid, now);
        memcpy(L->dest, h.dest, 16);
        L->haveDest = true;
        L->unresolved = false;
        L->lastMs = now;
        L->frames++;
        if (isTx) {
            if (h.hops == 0) {
                L->ours = true;
                /* We initiated: the LRPROOF will be addressed to the link_id.
                 * Counted only if the dest is already a known direct
                 * neighbour — proof is end-to-end, quality is first-hop. */
                Neighbor* d = peersFindByDest(st, h.dest);
                peersPendAdd(st, lid, h.dest, true, d && !d->isUs, now);
            }
            /* The identifier belongs on the row of the FIRST HOP, which is the
             * node we dialled only when we dialled a neighbour. A destination
             * behind a gateway is dialled *through* it: the request goes out in
             * transport and its first address field names the relay, while the
             * destination names a node nothing here can reach. Every later
             * frame of the session is addressed to the identifier and to no
             * destination at all, so a session filed on no row resolves to
             * nobody — absent from `lora n`, unnamed on a graph, and carrying
             * LORAQ_PEER_NONE into the queue, where the per-peer cap, the power
             * controller and SUPE all find nothing and the whole session stays
             * on the shared channel.
             *
             * Relaying somebody else's request files it too, and against the
             * node we relay it TO: the return direction arrives addressed to
             * the identifier rather than to our transport identity, and this is
             * one hop earlier than the first frame that carries it. */
            Neighbor* nh = h.hdr2 ? peersFindBy4(st, h.transportId)
                                  : peersFindByDest(st, h.dest);
            if (nh && !peersIsLocal(nh)) peersAddLink4(st, nh, lid, now);
            /* A link identifier we terminate or relay for. Held for as long as
             * the link plausibly lives; a link that goes quiet takes its entry
             * with it. */
#if !defined(CONFIG_LORA_NO_SUPE)
            supeTagAdd(r, lid, /*perm=*/false, SUPE_LINK_TTL_MS);
#endif
        } else {
            bool toUs = peersDestIsLocal(st, h.dest);
            if (toUs) {
                L->ours = true;                            /* inbound dial to us */
                /* **We terminate this link, so its identifier means us.** Every
                 * later frame of the session — the peer's data, its proofs —
                 * is addressed to the link identifier rather than to any
                 * destination of ours, so without this the whole session is
                 * unrecognisable: a detour offered for it would be read as
                 * somebody else's business and merely held.
                 *
                 * Learned from the *inbound* request rather than from the
                 * LRPROOF we send back, because the peer may offer a detour for
                 * the link the moment it is established, which is before our
                 * proof has necessarily left. */
#if !defined(CONFIG_LORA_NO_SUPE)
                supeTagAdd(r, lid, /*perm=*/false, SUPE_LINK_TTL_MS);
#endif
                /* And, when the request came out of a transaction, whose link
                 * it is. A link request carries no sender, so a link dialled to
                 * us in the clear is anonymous and stays that way: our whole
                 * side of the session — the proof first — flies plainly for
                 * want of a node to meet. Arriving
                 * as a detour's cargo, it is not anonymous at all: the node that
                 * asked for the detour is the node that dialled. Filing the
                 * identifier on its row makes the very first frame back
                 * detourable. */
                Neighbor* from = peersById(st, fromPeer);
                if (from && !peersIsLocal(from)) peersAddLink4(st, from, lid, now);
            }
            L->haveSig = true;                             /* initiator's setup signal */
            L->lastRssi = rssi;
            L->lastSnr10 = snr10;
            /* A power request prefixed to this LR is the initiator telling us
             * how loud our side of the session needs to be. We cannot name the
             * initiator — an LR carries no sender — but the link_id is a handle
             * both ends share, so the request rides it for the whole session.
             * Only for a link dialled to us: a request overheard on someone
             * else's link is not addressed to anything we will transmit on. */
            if (toUs && r->apRxSuggestPend) {
                L->suggestDbm  = r->apRxSuggest;
                L->haveSuggest = true;
                info("lora/%d power request: link %02x%02x%02x%02x -> reply at %d dBm",
                     r->idx, lid[0], lid[1], lid[2], lid[3], (int)r->apRxSuggest);
            }
        }
        break;
    }

    case NEI_PT_PROOF: {
        if (h.ctx == NEI_CTX_LRPROOF) {
            /* dest field = link_id; the transmitter is the link's destination. */
            NeiLink* L = peersLinkFind(st, h.dest);
            if (L) {
                L->established = true;
                L->lastMs = now;
                L->frames++;
                if (!isTx) { L->haveSig = true; L->lastRssi = rssi; L->lastSnr10 = snr10; }
            }
            /* An LRPROOF we transmit is us accepting a link: its identifier is
             * an address that means us from here on. Belt to the inbound
             * request's brace — a link whose LR we somehow missed still gets
             * its identifier learned here. */
#if !defined(CONFIG_LORA_NO_SUPE)
            if (isTx) supeTagAdd(r, h.dest, /*perm=*/false, SUPE_LINK_TTL_MS);
#endif
            if (!isTx) {
                NeiPend* pd = peersPendTake(st, h.dest);
                if (h.hops == 0) {
                    const uint8_t* dh = (L && L->haveDest) ? L->dest
                                      : (pd ? pd->dest : nullptr);
                    if (dh && !peersDestIsLocal(st, dh)) {
                        Neighbor* e = peersEnsureDest(st, dh, now);
                        if (e) {
                            peersSample(e, rssi, snr10, now);
                            if (pd) peersQuality(r, e, true);
                        }
                    }
                }
                /* hops > 0: the LRPROOF came relayed — the dest is not a direct
                 * neighbour, so it neither samples nor scores quality (§5 of
                 * plans/adaptive-power.md: proof is end-to-end, power is
                 * first-hop). */
            }
        } else if (!isTx) {
            /* A delivery proof is addressed to the proved packet's truncated
             * hash — match it against what we elicited. The proof arriving is
             * also what retires that hash from the set of addresses that mean
             * us; anything else still holding it keeps it alive. */
#if !defined(CONFIG_LORA_NO_SUPE)
            supeTagRelease(r, h.dest);
#endif
            NeiPend* pd = peersPendTake(st, h.dest);
            if (pd && !pd->isLR && h.hops == 0 && !peersDestIsLocal(st, pd->dest)) {
                Neighbor* e = peersEnsureDest(st, pd->dest, now);
                if (e) {
                    e->provesData = true;   /* this dest proves plain data */
                    peersSample(e, rssi, snr10, now);
                    peersQuality(r, e, true);
                }
            }
        }
        break;
    }

    case NEI_PT_DATA: {
        if (h.dtype == NEI_DT_LINK) {
            NeiLink* L = peersLinkFind(st, h.dest);
            if (!L) {
                L = peersLinkEnsure(st, h.dest, now);
                L->unresolved = true;       /* mid-link traffic, setup missed */
            }
            if (isTx && h.hops == 0) L->ours = true;   /* we originate on it */
            /* Any frame we put on air addressed to a link identifier makes that
             * identifier one that means us — whether we terminate the link or
             * merely relay for it. The relayed case is the one that is easy to
             * miss and expensive to miss: the return direction of a relayed
             * link arrives addressed to the link identifier rather than to our
             * transport identity, so a relay that skips this sleeps through it
             * and the link dies. */
#if !defined(CONFIG_LORA_NO_SUPE)
            if (isTx) supeTagAdd(r, h.dest, /*perm=*/false, SUPE_LINK_TTL_MS);
#endif
            L->lastMs = now;
            L->frames++;
            if (!isTx) {
                L->haveSig = true;
                L->lastRssi = rssi;
                L->lastSnr10 = snr10;
                /* On a link we initiated to a direct peer, every inbound frame
                 * at hops 0 is provably the peer (the dest) transmitting. */
                if (h.hops == 0 && L->ours && L->haveDest && !peersDestIsLocal(st, L->dest)) {
                    Neighbor* e = peersEnsureDest(st, L->dest, now);
                    if (e) {
                        peersSample(e, rssi, snr10, now);
                        /* The identifier too, and not only at setup: a link we
                         * picked up mid-session never saw its request, and this
                         * is the first frame that proves whose it is. */
                        peersAddLink4(st, e, h.dest, now);
                    }
                }
                /* A link DIALLED to us is anonymous in the clear — a link
                 * request carries no sender, and the session's own identify
                 * step is encrypted inside it — so the far end of every link we
                 * host would stay unknown. Arriving as a detour's cargo it is
                 * not anonymous at all: the schedule belongs to one pair, and
                 * this came under it. That is the only handle the responder
                 * ever gets, so it is taken on any frame and not just at setup. */
                Neighbor* from = peersById(st, fromPeer);
                if (from && !peersIsLocal(from)) peersAddLink4(st, from, h.dest, now);
            }
        } else if (h.dtype == NEI_DT_SINGLE && isTx) {
            /* Every single-dest data packet we send or relay may attract a
             * delivery proof, and that proof is addressed to the packet's
             * truncated hash — so the hash is an address that means us, for as
             * long as the receipt or reverse-table window lasts. Exactly two
             * nodes hold it, ours being one, so the tag is as precise here as
             * anywhere else. */
            uint8_t ph[16];
            rnsPacketHash(&h, p, len, false, ph);
#if !defined(CONFIG_LORA_NO_SUPE)
            supeTagAdd(r, ph, /*perm=*/false, SUPE_PROOF_TTL_MS);
#endif
            /* Our own origination (probes included): if the dest is a known
             * direct neighbour, expect a proof back — the quality elicitor. A
             * miss counts only once the dest has proven before. */
            if (h.hops == 0) {
                Neighbor* e = peersFindByDest(st, h.dest);
                if (e && !e->isUs) peersPendAdd(st, ph, h.dest, false, e->provesData, now);
            }
        } else if (h.dtype == NEI_DT_SINGLE && !isTx && h.hdr2) {
            /* An inbound packet that may attract a proof, relayed to us by a
             * node that named itself. The proof we send back is addressed to
             * the packet's hash, which resolves to no destination and no link —
             * so file the pair now, while the relayer is still on the wire, and
             * the proof has a next hop when it goes out. */
            uint8_t ph[16];
            rnsPacketHash(&h, p, len, false, ph);
#if !defined(CONFIG_LORA_NO_SUPE)
            supeProofRetFile(r, ph, h.transportId);
#endif
        }
        break;
    }
    }
}

#endif  /* CONFIG_LORA0_CS_PIN */
