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
static void rnsPacketHash(const RnsHdr* h, const uint8_t* p, size_t len,
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

/* Known app.aspect name-hashes, to label dest rows. Computed once (name_hash =
 * SHA-256(expanded name)[:10], matching µR's Destination::expand_name). */
static const char* const kRnsNames[] = {
    "lxmf.delivery", "lxmf.propagation", "nomadnetwork.node",
    "rnstransport.probe", "rnsh",
};
static constexpr int kRnsNameCount = (int)(sizeof(kRnsNames) / sizeof(kRnsNames[0]));
static uint8_t s_rnsNameHash[kRnsNameCount][10];

void rnsNamesInit(void) {
    for (int i = 0; i < kRnsNameCount; i++) {
        uint8_t sha[RNSD_HASH_LEN];
        rnsdSha256((const uint8_t*)kRnsNames[i], strlen(kRnsNames[i]), sha);
        memcpy(s_rnsNameHash[i], sha, 10);
    }
}

const char* rnsNameLabel(const uint8_t nameHash[10]) {
    for (int i = 0; i < kRnsNameCount; i++)
        if (memcmp(s_rnsNameHash[i], nameHash, 10) == 0) return kRnsNames[i];
    return nullptr;
}

/* Display name out of an announce's app_data. LXMF wraps it in msgpack
 * (optionally behind a 32-byte ratchet); NomadNet and very old clients send raw
 * UTF-8. We only need the first element, so this is a deliberately small subset
 * of the parser lxmf/ carries — iface-lora talks to rnsd alone and must not
 * depend on a consumer straddle. */
static void rnsParseName(const uint8_t* p, size_t n, char* out, size_t outsz) {
    out[0] = '\0';
    if (!p || !n) return;

    auto plausible = [&](size_t off, size_t len) {
        if (off >= n || !len) return false;
        for (size_t k = 0; k < len && off + k < n; k++) {
            uint8_t b = p[off + k];
            if (b == 0x7F || (b < 0x20 && b != '\t' && b != '\n' && b != '\r')) return false;
        }
        return true;
    };
    auto copy = [&](const uint8_t* q, size_t len) {
        if (len >= outsz) len = outsz - 1;
        memcpy(out, q, len);
        out[len] = '\0';
    };
    /* msgpack array whose first element is the name (str/bin/nil). */
    auto tryArray = [&](size_t i) {
        if (i >= n) return false;
        uint8_t b = p[i++];
        if (b >= 0x90 && b <= 0x9F) { if (!(b & 0x0F)) return false; }
        else if (b == 0xDC) { if (i + 2 > n) return false; i += 2; }
        else return false;
        if (i >= n) return false;
        uint8_t t = p[i++];
        size_t len;
        if (t == 0xC0) return true;                       /* nil name — valid, empty */
        else if (t >= 0xA0 && t <= 0xBF) len = t & 0x1F;   /* fixstr */
        else if (t == 0xD9 || t == 0xC4) { if (i >= n) return false; len = p[i++]; }
        else if (t == 0xDA || t == 0xC5) { if (i + 2 > n) return false; len = ((size_t)p[i] << 8) | p[i+1]; i += 2; }
        else return false;
        if (i + len > n) return false;
        copy(p + i, len);
        return true;
    };
    if (n >= 34 && tryArray(32)) return;
    if (tryArray(0)) return;
    if (n > 32 && plausible(32, n - 32)) { copy(p + 32, n - 32); return; }
    if (plausible(0, n)) copy(p, n);
}


/* ── announce ingest: the identity join ── */

static void observeAnnounce(LoraRadio* r, const RnsHdr* h, bool isTx,
                        int16_t rssi, int16_t snr10, uint32_t now,
                        uint8_t txOrigin) {
    NeiState* st = r->nei;
    /* pubkey(64) | name_hash(10) | random_hash(10) | signature(64) | app_data */
    if (h->dataLen < 64 + 10 + 10 + 64) return;
    const uint8_t* pub    = h->data;
    const uint8_t* nameH  = h->data + 64;
    const uint8_t* randH  = h->data + 74;
    const uint8_t* sig    = h->data + 84;
    const uint8_t* appD   = h->data + 148;
    size_t         appLen = h->dataLen - 148;

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
    static uint8_t sd[16 + 64 + 10 + 10 + RNS_MTU];   /* signed_data; lora task only */
    size_t o = 0;
    memcpy(sd + o, h->dest, 16); o += 16;
    memcpy(sd + o, pub, 64);     o += 64;
    memcpy(sd + o, nameH, 10);   o += 10;
    memcpy(sd + o, randH, 10);   o += 10;
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
        peersMergeInto(e, d);
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
            if ((e->isUs && o->isUs) || (e->isRnode && o->isRnode)) peersMergeInto(e, o);
        }
    }

    /* A 0x03 may have claimed this dest before it ever announced, in which case
     * the claiming row and this one are one device: the linkage frame said so,
     * and the announce has now supplied the hash it only held a stub for. Fold,
     * so the aspect joins the node instead of starting a row of its own. Same
     * us/them guard as neiLink(): a peer's claim must not reach our row. */
    if (!peersIsLocal(e)) {
        Neighbor* c = peersFindClaim4(st, h->dest);
        if (c && c != e && !peersIsLocal(c)) peersMergeInto(e, c);
        /* And the same by identity: a SUPE announcement heard before this one
         * files what it knows — four bytes of an identity and the radio's
         * capabilities — against a claim row, and this is the frame that
         * supplies the identity itself. */
        c = peersFindClaim4(st, idh);
        if (c && c != e && !peersIsLocal(c)) peersMergeInto(e, c);
    }

    NeiDest* nd = peersAddDest(e, h->dest, now);
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
}

/* ── the per-packet observation tap ── */

void peersObserve(LoraRadio* r, const uint8_t* p, size_t len, bool isTx,
                       int16_t rssi, int16_t snr10, uint8_t txOrigin,
                       uint16_t fromPeer) {
    NeiState* st = r->nei;
    if (!st) return;
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
         * when nothing names it. Rebroadcast announces (HEADER_2) are
         * attributed to their named transit row below; everything else
         * relayed lands in the aggregate anonymous-transit row. */
        if (h.hops >= 1 && !(h.ptype == NEI_PT_ANNOUNCE && h.hdr2)) {
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
            Neighbor* e = peersFindByIdentity(st, h.transportId);
            if (!e) {
                e = peersAlloc(st, now);
                if (e) {
                    peersAddId(e, h.transportId);
                }
            }
            if (e && !e->isUs) {
                e->transit = true;
                peersSample(e, rssi, snr10, now);
            }
        } else if (isTx && h.hdr2) {
            /* Our own rebroadcast stamps OUR transport identity as the
             * transport_id — the exact frame neighbours identify us by, so
             * learn "who we are" from it symmetrically. This is the only way
             * the transport identity surfaces here unless rnsd also hosts an
             * announcing destination on it (rnstransport.probe usually does,
             * in which case this merges into that us row and tags it). */
            Neighbor* e = peersFindByIdentity(st, h.transportId);
            if (!e) {
                e = peersAlloc(st, now);
                if (e) {
                    peersAddId(e, h.transportId);
                }
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
        if (h.dtype != NEI_DT_SINGLE || h.hops != 0) break;   /* relayed LR: transit, out of scope */
        uint8_t lid[16];
        rnsPacketHash(&h, p, len, true, lid);
        NeiLink* L = peersLinkEnsure(st, lid, now);
        memcpy(L->dest, h.dest, 16);
        L->haveDest = true;
        L->unresolved = false;
        L->lastMs = now;
        L->frames++;
        if (isTx) {
            L->ours = true;
            /* We initiated: the LRPROOF will be addressed to the link_id.
             * Counted only if the dest is already a known direct neighbour. */
            Neighbor* e = peersFindByDest(st, h.dest);
            peersPendAdd(st, lid, h.dest, true, e && !e->isUs, now);
            /* A link identifier we terminate. Held for as long as the link
             * plausibly lives; a link that goes quiet takes its entry with it. */
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
                 * us is normally anonymous and our whole side of the session —
                 * the proof first — flies plainly until a MANIFEST for the link
                 * identifier eventually files capabilities against it. Arriving
                 * as a detour's cargo, it is not anonymous at all: the node that
                 * asked for the detour is the node that dialled. Filing the
                 * identifier on its row makes the very first frame back
                 * detourable. */
                Neighbor* from = peersById(st, fromPeer);
                if (from && !peersIsLocal(from)) peersAddLink4(from, lid);
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
                    if (e) peersSample(e, rssi, snr10, now);
                }
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
