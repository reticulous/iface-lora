/**
 * lora_mon — telemetry: the radio→interface record queue, the LoRaMon
 * per-frame ring and its storage nodes, the stats flush, the channel-RSSI
 * series, and the interface task that owns every storage write.
 */
#include "lora_priv.h"

#include <cstdarg>
#include <string>

#if defined(CONFIG_LORA0_CS_PIN)

/* ─────────────── the two tasks ───────────────
 *
 * `lora` is the RADIO task: it owns the chip and nothing else may call
 * RadioLib. Sharing one task across all radios is deliberate — it serialises
 * the SPI bus by construction, so no sibling radio can slip a transaction into
 * a channel-access sequence. Everything it does is bounded, in-RAM work.
 *
 * `lora-if` is the INTERFACE task: the storage side. Publishing packet nodes,
 * expiring them, the stats flush, the RSSI series. It runs below the radio task
 * so it can never delay it.
 *
 * The line between them is **flash**. A storage write can erase a sector, and an
 * erase blocks the instruction cache for milliseconds — which is survivable for
 * telemetry and fatal for a channel-access deadline. Keeping every storage op
 * off the radio task is what makes the timing arguments in plans/psa.md hold.
 *
 * Traffic is one way, radio → interface, over a bounded queue that is never
 * allowed to block: a full queue drops the record and counts it. Telemetry
 * yielding under pressure is correct; a recorder that can stall a transmit is
 * not.
 *
 * The neighbour table — including its inline Ed25519 announce verification —
 * is still on the radio task. It wants to move for the same reason, but it is
 * read cross-task by the CLI, the probe and the adaptive-power path, so its
 * ownership has to be settled first. */

enum : uint8_t {
    IFM_MON  = 0,   /* one on-air frame → a packet node */
    IFM_RSSI = 1,   /* one channel-RSSI sample */
    IFM_KICK = 2,   /* no payload: a watch transition — wake and re-block on the
                     * cadence the new state calls for */
};

struct IfMsg {
    uint8_t  kind;
    uint8_t  radio;
    uint8_t  dir;        /* MON: 0 rx, 1 tx */
    uint8_t  type;       /* MON: LORA_PKT_* */
    uint8_t  desc;       /* MON: LMD_* — what the frame is */
    uint8_t  cast;       /* MON: LMC_* — who it was aimed at */
    uint8_t  tag[3];     /* MON: the node the frame concerns, 0 if unattributable */
    uint8_t  ch;         /* channel index; 0 = the reticulum hailing channel */
    int8_t   txp;        /* MON tx: power of the frame */
    uint32_t t_ms;       /* MON: frame start; RSSI: sample time */
    uint16_t dur_ms;     /* MON: time on air */
    uint16_t bytes;      /* MON: payload bytes */
    uint16_t wait_ms;    /* MON tx: time spent contending for the channel */
    uint16_t own_ms;     /* MON tx: time the frame waited on US — the radio
                          * busy with our own work, or a deliberate delay */
    int16_t  rssi;       /* MON rx: dBm; RSSI: channel 0's reading */
    int16_t  snr10;      /* MON rx: deci-dB */
    uint8_t  nch;        /* RSSI: how many of chRssi carry a reading */
    int16_t  chRssi[LORA_CH_MAX];   /* RSSI: dBm per channel, index = channel */
};
/* Deep enough for a burst of frames to land
 * inside ~500 ms without the storage side having to keep up frame for frame.
 * At 24 it overflowed at the end of every run, dropping the last records —
 * which reads as "the last frames were never sent". */
#define LORA_IFQ_DEPTH 48
static QueueHandle_t s_ifq = nullptr;

/* Cached so the radio task can gate recording — and the sampling beat — on it
 * without a storage read of its own. Updated by the watch-key subscription the
 * moment a viewer opens or closes, and refreshed at each maintenance beat as a
 * belt against a missed callback. */
static volatile bool s_monWatched = false;

/* The radio task's read of it: true while a LoRaMon viewer (web or LCD) is
 * open. What hangs off this is not just recording but wake cycles — see
 * rssiSamplePoll and nextDeadline. */
bool loraMonOpen(void) { return s_monWatched; }

/* Post to the interface task. Never blocks: a full queue means the storage side
 * is behind, and dropping telemetry is the correct answer. Returns false so the
 * caller can count the loss against the radio it belongs to. */
static bool ifPost(const IfMsg* m) {
    if (!s_ifq) return false;
    return xQueueSend(s_ifq, m, 0) == pdTRUE;
}

/* True while something is actually reading the neighbourhood — today the web
 * LoRaMon's packet hover, tomorrow a graph view. Its own key, not LoRaMon's:
 * these are different appetites. LoRaMon wants frames and can run on an LCD
 * that needs no published peers at all; a graph wants the peer table and no
 * frames. Publishing on the wrong one costs a rewrite of every peer row on
 * every stats beat for a reader that is not there. */
static bool loraPeersWatched(void) {
    return storageGetInt("sys.stats.web_peers", 0) != 0;
}

/* The neighbourhood, one node per peer-table slot, for anything that has to
 * turn a tag or an address back into a node — and to say what has been learned
 * about it. Published rather than derived, because all of it lives in the peer
 * table and nothing outside this straddle can rebuild it: a viewer sees frames,
 * not the announces, proofs and measurements that clustered and characterised
 * them.
 *
 *   lora.<n>.peers.<slot> =
 *     "<num>|<supe>|<tags…>|<names…>|<loss>|<rssi>|<snr>|<q>|<heard_s>|<flags>|<budget>"
 *
 * Empty means not known rather than zero — a peer with no path-loss pair yet is
 * a different thing from one measured at 0 dB, and a graph that cannot tell
 * them apart draws a confident line where there is no measurement.
 *
 * On-device viewers do NOT read this. They are inside the same binary as the
 * peer table and ask it directly (see loraPeerSummary in lora.h); serialising a
 * fact so the same firmware can parse it back is a round trip for nothing.
 *
 * Built in a std::string and not a fixed buffer, because the tag set has no
 * useful bound: a node reached over links accrues one tag per link identifier
 * (NEI_HASHES_MAX of them, shared, and they land on whoever is actually being
 * talked to), and tags precede the names. Cut off at a buffer's end, that peer
 * — the ONLY one whose traffic a viewer is hovering — loses its names and its
 * later fields, and the tags that fell off resolve to nobody at all: exactly
 * the node the record exists to name shows as `#4` or as raw hex. */
static void appendf(std::string& s, const char* fmt, ...)
    __attribute__((format(printf, 2, 3)));
static void appendf(std::string& s, const char* fmt, ...) {
    char b[32];                       /* one field, never a whole record */
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    s += b;
}

static void publishPeers(LoraRadio* r) {
    if (!r->nei) return;
    char k[48];
    std::string v;
    uint32_t now = millis();
    /* The number `lora n` prints beside a node, counted the same way it counts:
     * used, not us, in table order. A viewer that has no name to show falls
     * back to it, and a person reading both surfaces sees the same #4. */
    int num = 0;
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &r->nei->nei[i];
        snprintf(k, sizeof k, "lora.%d.peers.%d", r->idx, i);
        if (!e->used || peersIsLocal(e)) { storageDeleteTree(k); continue; }
        num++;
        uint8_t tg[NEI_DESTS_MAX + NEI_HASHES_MAX + NEI_IDS_MAX + 1][3];
        int nt = 0;
        auto addTag = [&](const uint8_t* b) {
            for (int j = 0; j < nt; j++) if (memcmp(tg[j], b, 3) == 0) return;
            if (nt < (int)(sizeof tg / sizeof tg[0])) memcpy(tg[nt++], b, 3);
        };
        if (e->haveNode4) addTag(e->node4);
        for (int d = 0; d < e->nDests; d++) addTag(e->dests[d].hash);
        for (int q = 0; q < e->nIds; q++)   addTag(e->ids[q]);
        for (int l = 0; ; l++) {
            uint8_t h4[4];
            if (!peersHashAt(r->nei, e, l, h4)) break;
            addTag(h4);
        }

        v.clear();
        appendf(v, "%d|", num);
#if !defined(CONFIG_LORA_NO_SUPE)
        appendf(v, "%d|", e->supeSeen ? 1 : 0);
#else
        v += "0|";
#endif
        for (int j = 0; j < nt; j++)
            appendf(v, "%s%02x%02x%02x", j ? "," : "", tg[j][0], tg[j][1], tg[j][2]);
        v += '|';
        {
            char names[NEI_NAME_MAX * 3];
            peersNodeNames(e, names, sizeof names);
            v += names;
        }
        /* What has been learned about it. Path loss is the one number a graph
         * actually wants — it is symmetric, it does not move when either end
         * changes power, and it is what an edge length should be drawn from. */
        v += '|';
        if (e->havePair) appendf(v, "%d", (int)e->pairTxp - (int)e->pairRssi);
        v += '|';
        if (e->haveSig)  appendf(v, "%d", (int)e->rssiMax);
        v += '|';
        if (e->haveSig)  appendf(v, "%d", (int)e->snrMax10);
        v += '|';
        if (e->haveQuality) appendf(v, "%u", (unsigned)e->quality);
        appendf(v, "|%u|%s%s", (unsigned)((now - e->lastHeardMs) / 1000),
                e->transit ? "T" : "", e->roaming ? "R" : "");
#if !defined(CONFIG_LORA_NO_SUPE)
        v += '|';
        if (e->supeSeen) appendf(v, "%u", (unsigned)e->supeCaps.topStep);
#endif
        storageSet(k, v);
    }
}

void publishStats(LoraRadio* r) {
    /* Skip the churn on a headless, WiFi-down node — nothing pulls these keys
     * there. A UI (web over WiFi, or an LCD build) re-populates them when it
     * appears; see uiTelemetryWanted(). */
    if (!uiTelemetryWanted()) return;
    char b[48];
    /* One bracket → one storage op. Unbracketed this fired ~8 separate sync
     * round-trips to the storage task every second; under an inbound-message
     * burst those pile up on the storage op port and stall the radio task. */
    storageBegin();
    if (loraPeersWatched()) publishPeers(r);
    storageSet(rk(b, sizeof b, r->idx, "stats.tx_bytes"),  (int)(r->txBytes & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.rx_bytes"),  (int)(r->rxBytes & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.tx_frames"), (int)(r->txFrames & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.rx_frames"), (int)(r->rxFrames & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.crc_err"),   (int)(r->crcErr & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.split_rx_timeout"), (int)(r->splitTimeouts & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.tx_dropped"), (int)(r->txDropped & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "stats.rssi_last"), (int)r->rssiLast);
    storageSet(rk(b, sizeof b, r->idx, "stats.snr_last"),  (int)r->snrLast);
    /* APPC: what the contention window is being drawn from right now. Both move
     * only on a transmit, so they belong in the same event-driven flush. */
    if (r->appc) {
        storageSet(rk(b, sizeof b, r->idx, "stats.airtime_pct"),
                   (int)(appcAirtime(r) * 100.0f));
        storageSet(rk(b, sizeof b, r->idx, "stats.cw_band"), (int)appcLiveBand(r));
    }
    /* The composed lines a settings row shows: the last reception's quality and
     * the frame counters. Units and separators belong with the numbers, here,
     * not re-assembled by each surface that displays them. */
    char txt[64];
    snprintf(txt, sizeof(txt), "RSSI %d dBm \xC2\xB7 SNR %d dB", (int)r->rssiLast, (int)r->snrLast);
    storageSet(rk(b, sizeof b, r->idx, "rx_text"), txt);
    snprintf(txt, sizeof(txt), "rx %u \xC2\xB7 tx %u",
             (unsigned)(r->rxFrames & 0x7fffffff), (unsigned)(r->txFrames & 0x7fffffff));
    storageSet(rk(b, sizeof b, r->idx, "traffic"), txt);
    storageEnd();
}

/* The status-bar pill: `L` and how many other nodes this radio hears — the same
 * number `lora n` heads its listing with, summed over every slot, because the
 * pill names the MEDIUM and a board with two radios still has one LoRa
 * neighbourhood. Published while any slot is enabled, at 0 as readily as at 3:
 * "enabled and hearing nobody" is exactly what an operator needs to see, and it
 * is not the same as no pill at all. rnsd owns the keys and both status lines
 * read them; the letter and the colour are this straddle's to state. */
void publishPill(void) {
    int peers = 0;
    bool on = false;
    for (int i = 0; i < kNumRadios; i++) {
        LoraRadio* r = &s_radios[i];
        if (!r->enabled) continue;
        on = true;
        peers += peersOtherCount(r->nei);
    }
    if (on) rnsdPillSet("lora", 'L', peers, LORA_PILL_COLOR, LORA_PILL_ORDER);
    else    rnsdPillClear("lora");
}

/* ─────────────── LoRaMon: recording, windows, publish, ITS server ─────────────── */

/* True while a LoRaMon viewer (web or LCD) is open — gates recording. */
static bool loraMonWatched(void) {
    return storageGetInt("sys.stats.web_loramon", 0) ||
           storageGetInt("sys.stats.lcd_loramon", 0);
}

/* Delete published packet nodes older than the 1-hour window, and enforce the
 * FIFO cap. The FIFO holds start-ms oldest-first; pop + delete from the front.
 * Interface task only — it owns the FIFO and every packet node in storage. */
static void loraMonExpire(LoraRadio* r, uint32_t now) {
    if (!r->mon.pktMs) return;
    while (r->mon.pktCount) {
        uint32_t oldest = r->mon.pktMs[r->mon.pktHead];
        bool aged = (now - oldest) > 3600u * 1000;      /* > 1 h (u32 diff, wrap-safe) */
        bool full = r->mon.pktCount >= r->mon.pktCap;           /* backstop against a flood */
        if (!aged && !full) break;
        char key[40];
        snprintf(key, sizeof key, "lora.%d.packets.%u", r->idx, (unsigned)oldest);
        storageDeleteTree(key);
        /* Aged out from under the run being extended: stop extending it, or the
         * next span would recreate a node the window has already dropped. */
        if (r->mon.dwellKeyMs == oldest) r->mon.dwellKeyMs = 0;
        r->mon.pktHead = (uint16_t)((r->mon.pktHead + 1) % r->mon.pktCap);
        r->mon.pktCount--;
    }
}

/* Publish one packet node `lora.<n>.packets.<ms>` holding a packed string:
 *
 *   r|rssi|snr|dur|bytes|type|ch|desc|cast[|tag]
 *   t|txp|dur|bytes|type|wait|ch|own|desc|cast[|tag]
 *   a|ch|dur[|tag]
 *
 * The last is a DWELL: the radio was tuned to that channel and listening for
 * that long, ending where the next one begins. Its tag is the meeting whose
 * slot the stay is — present only inside one. The leading token is the
 * direction; snr is deci-dB; ch is the channel, 0 being the reticulum hailing
 * channel; desc is what the frame is (LMD_*); cast is who it was aimed at
 * (LMC_*); tag is six hex characters naming the peer, absent when unknown.
 * Then age old nodes out.
 *
 * The dwell is what makes a viewer able to say where the radio *was*, which no
 * frame record can: a slot attended in silence, a meeting's channel held open,
 * the hailing channel between them. Without it a lane can only show what
 * arrived, and a lane with nothing in it means both "nobody spoke" and "we were
 * not listening" — the two answers a channel view exists to separate.
 *
 * INTERFACE TASK ONLY — this is the storage half of a record. */
static void loraMonRecordOne(LoraRadio* r, const IfMsg* m) {
    if (!r->mon.pktMs || !r->mon.pktCap) return;
    char key[40], val[56];

    /* A stay on one channel is ONE record that grows, not one per beat. The
     * radio sits on the hailing channel whenever it is doing nothing, so a node
     * per beat would be a permanent write every second and — worse — would push
     * real frames out of the capped ring, emptying the very history the graph
     * exists to show. Extending in place costs one rewrite of a node already
     * there and keeps a whole idle hour as a single record. */
    if (m->dir == 2) {
        LoraMonState& mo = r->mon;
        /* The tag is part of what makes two spans one stay: a meeting ending
         * and another beginning on the same channel is two slots belonging to
         * two peers, and a viewer that draws one label per slot has to see the
         * boundary. */
        if (mo.dwellKeyMs && mo.dwellCh == m->ch && mo.dwellEndMs == m->t_ms &&
            memcmp(mo.dwellTag, m->tag, 3) == 0 &&
            (uint32_t)mo.dwellDur + m->dur_ms <= 0xFFFF) {
            mo.dwellDur = (uint16_t)(mo.dwellDur + m->dur_ms);
            mo.dwellEndMs = m->t_ms + m->dur_ms;
            snprintf(key, sizeof key, "lora.%d.packets.%u", r->idx, (unsigned)mo.dwellKeyMs);
            snprintf(val, sizeof val, "a|%u|%u", (unsigned)mo.dwellCh, (unsigned)mo.dwellDur);
            if (mo.dwellTag[0] | mo.dwellTag[1] | mo.dwellTag[2]) {
                size_t o = strlen(val);
                snprintf(val + o, sizeof val - o, "|%02x%02x%02x",
                         mo.dwellTag[0], mo.dwellTag[1], mo.dwellTag[2]);
            }
            storageSet(key, val);
            return;                      /* the FIFO already holds this node */
        }
        mo.dwellKeyMs = m->t_ms ? m->t_ms : 1;
        mo.dwellCh    = m->ch;
        mo.dwellDur   = m->dur_ms;
        mo.dwellEndMs = m->t_ms + m->dur_ms;
        memcpy(mo.dwellTag, m->tag, 3);
    } else {
        /* Any frame ends the run: the next dwell starts a record of its own, so
         * a span never reads as covering traffic that happened inside it. */
        r->mon.dwellKeyMs = 0;
    }

    snprintf(key, sizeof key, "lora.%d.packets.%u", r->idx, (unsigned)m->t_ms);
    if (m->dir == 2) snprintf(val, sizeof val, "a|%u|%u",
                              (unsigned)m->ch, (unsigned)m->dur_ms);
    else if (m->dir) snprintf(val, sizeof val, "t|%d|%u|%u|%u|%u|%u|%u|%u|%u",
                         (int)m->txp, (unsigned)m->dur_ms, (unsigned)m->bytes,
                         (unsigned)m->type, (unsigned)m->wait_ms, (unsigned)m->ch,
                         (unsigned)m->own_ms, (unsigned)m->desc, (unsigned)m->cast);
    else        snprintf(val, sizeof val, "r|%d|%d|%u|%u|%u|%u|%u|%u",
                         (int)m->rssi, (int)m->snr10, (unsigned)m->dur_ms,
                         (unsigned)m->bytes, (unsigned)m->type, (unsigned)m->ch,
                         (unsigned)m->desc, (unsigned)m->cast);
    /* The tag rides last and only when there is one. Six characters on every
     * record of a thousand is worth not spending on zeros, and a viewer that
     * finds the field absent has the same answer as one that finds it empty.
     * A dwell carries one too — it is a meeting's slot, and the slot belongs to
     * the peer for its whole width whether or not a frame landed in it. */
    if (m->tag[0] | m->tag[1] | m->tag[2]) {
        size_t o = strlen(val);
        snprintf(val + o, sizeof val - o, "|%02x%02x%02x",
                 m->tag[0], m->tag[1], m->tag[2]);
    }
    storageSet(key, val);

    loraMonExpire(r, m->t_ms);                           /* age out + free a slot if full */
    r->mon.pktMs[(r->mon.pktHead + r->mon.pktCount) % r->mon.pktCap] = m->t_ms;
    r->mon.pktCount++;
}

/* Packet nodes batch: a train is many frames in a blink, and one storage op
 * per frame is one port-44 round-trip per frame into an actor that may be
 * frozen by a concurrent flash window — under a burst the ops pile up and
 * stall everything behind that port. So records accumulate here and go out
 * bracketed, one atomic storage op per flush (the expiry deletes ride the
 * same bracket). Flushed when the batch fills, when the inbound queue goes
 * quiet, and at the maintenance beat — so a lone frame still shows up
 * promptly and a storm costs one op per batch instead of one per frame. */
#define MON_BATCH_MAX 32
static IfMsg  s_monPend[MON_BATCH_MAX];
static uint8_t s_monPendN = 0;

static void monFlushPending(void) {
    if (!s_monPendN) return;
    storageBegin();
    for (uint8_t i = 0; i < s_monPendN; i++) {
        const IfMsg* m = &s_monPend[i];
        if (m->radio < kNumRadios) loraMonRecordOne(&s_radios[m->radio], m);
    }
    storageEnd();
    s_monPendN = 0;
}

static void loraMonRecord(LoraRadio* r, const IfMsg* m) {
    (void)r;
    s_monPend[s_monPendN++] = *m;
    if (s_monPendN >= MON_BATCH_MAX) monFlushPending();
}

/* Record one on-air frame. RADIO TASK: the in-RAM rollups, the debug line, and
 * a hand-off to the interface task for the storage node. Nothing here touches
 * flash.
 *
 * `wait_ms` and `own_ms` are tx-only and belong to the FIRST frame of a burst —
 * the frames behind it followed immediately and waited for nothing. They are
 * separated because they are different facts about the same delay: `wait_ms` is
 * what the *channel* made us wait, and `own_ms` is what *we* did — the radio
 * held by our own traffic, a split still landing, or a deliberate pre-offer
 * delay. Conflated, a busy channel and a busy radio look identical, and only
 * one of them is somebody else's fault. */
/* What a frame is, from the frame itself. Our own protocol names itself in its
 * first byte; Reticulum's packet type is the bottom two bits of its first
 * header byte. A split half is left unread beyond that — its second frame
 * carries no header at all, and guessing which half this is from a record that
 * does not know would put a confident wrong name on the graph. */
/* The three bytes of the address a frame was AIMED AT — the same prefix the
 * protocol classifies on, so a record and a log line agree about who a frame
 * was for. Our own frames carry it where their own codec puts it; a Reticulum
 * packet's is the front of its first address field, which sits behind this
 * interface's framing byte and the two header bytes. Zero where there is none
 * to take: a frame too short, or one that names no address at all.
 *
 * A split packet's address is in its head — the only half with a header — and
 * the caller passes that half, because nothing in a frame says which half it
 * is. */
void loraMonTagOf(const uint8_t* f, size_t len, uint8_t type, uint8_t out[3]) {
    out[0] = out[1] = out[2] = 0;
    if (!f || len < 1) return;
    if (type == LORA_PKT_OURS) {
        /* PRIVSYNC is the one that names a peer: type, regime/version, then the
         * tag it is asking about. The meeting frames name a schedule, not a
         * node, so they have none to give. */
        if (f[0] == SUPE_T_PRIVSYNC && len >= 2 + SUPE_TAG_LEN)
            memcpy(out, f + 2, SUPE_TAG_LEN);
        return;
    }
    if (len >= 1 + 2 + 3) memcpy(out, f + 1 + 2, 3);
}

/* The two SUPE frames that name their own sender.
 *
 * PRIVSYNC's sender_ident sits behind the tag, the power and the salt. It is
 * optional on the wire — a hail from a node with nothing to say about itself
 * omits it — so the length is what says whether there is one.
 *
 * ANNOUNCE2 is nothing BUT a statement of who is speaking: the identities are
 * its payload, and the first of them is the one annIngest resolves the frame's
 * node through, so it is the one that resolves through a published tag set too
 * (it is that row's node4, one of its idents, or the front of one of its
 * destinations, whichever the row was found by). */
bool loraMonSenderOf(const uint8_t* f, size_t len, uint8_t type, uint8_t out[3]) {
    if (!f || type != LORA_PKT_OURS || len < 1) return false;
    if (f[0] == SUPE_T_ANNOUNCE2) {
        if (len < SUPE_ANN2_BASE + SUPE_ID_LEN) return false;
        memcpy(out, f + SUPE_ANN2_BASE, SUPE_TAG_LEN);
        return true;
    }
    if (f[0] != SUPE_T_PRIVSYNC || len < SUPE_PRIVSYNC_ID_LEN) return false;
    memcpy(out, f + SUPE_PRIVSYNC_LEN, SUPE_TAG_LEN);
    return true;
}

/* What one recorded frame IS and who it concerns, resolving which half of a
 * split it holds from the record stream itself.
 *
 * Both loraMonDescribe and loraMonTagOf read the Reticulum header, so both need
 * the half that HAS one. Nothing in a frame says which half it is, and the
 * reassembly state cannot answer either — a train's frames are buffered by the
 * meeting and reassembled at its close, so at record time none of it has moved.
 * The stream tracks itself instead, by the seq-and-timeout rule reassembly uses:
 * a split frame whose seq matches a pending head is that head's tail, and takes
 * the head's answers rather than inventing its own out of payload bytes.
 *
 * Two descriptions come back because they answer different questions. `desc` is
 * what this FRAME is, and a tail is honestly a `split`. `whole` is what the
 * PACKET is, which a tail inherits — an announce too big for one frame is a
 * broadcast in both its halves, and colouring the second one as a unicast would
 * say the packet changed audience halfway through the air.
 *
 * RADIO TASK — called from the rx drain and from TxDone. */
void loraMonClassify(LoraRadio* r, uint8_t dir, const uint8_t* f, size_t len,
                     uint8_t type, uint32_t now,
                     uint8_t* desc, uint8_t* whole, uint8_t tag[3]) {
    LoraRadio::MonSplit* ms = &r->monSplit[dir ? 1 : 0];
    bool split = type != LORA_PKT_OURS && len >= 1 && (f[0] & RNODE_FLAG_SPLIT);
    /* A head whose tail never came does not get to claim the next one. */
    if (ms->pend && (uint32_t)(now - ms->atMs) > SPLIT_RX_TIMEOUT_MS) ms->pend = false;
    if (split && ms->pend && ms->seq == (uint8_t)(f[0] & 0xF0)) {
        *desc  = LMD_RNS_SPLIT;             /* what this FRAME is: the rest of one */
        *whole = ms->desc;                  /* what the PACKET is: its head's answer */
        memcpy(tag, ms->tag, 3);
        ms->pend = false;
        return;
    }
    *desc  = loraMonDescribe(f, len, type);
    *whole = *desc;
    loraMonTagOf(f, len, type, tag);
    if (split) {
        ms->pend = true;
        ms->seq  = (uint8_t)(f[0] & 0xF0);
        ms->atMs = now;
        ms->desc = *desc;
        memcpy(ms->tag, tag, 3);
    }
}

uint8_t loraMonCastOf(LoraRadio* r, const uint8_t tag[3]) {
    if (!r->nei || !tag || !(tag[0] | tag[1] | tag[2])) return LMC_OTHER;
    /* Ours means a LOCAL row's — this node's own destinations and identities,
     * and the attached RNode client's, since an RNode is us (§17). */
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &r->nei->nei[i];
        if (!e->used || !peersIsLocal(e)) continue;
        if (e->haveNode4 && memcmp(e->node4, tag, 3) == 0) return LMC_US;
        for (int d = 0; d < e->nDests; d++)
            if (memcmp(e->dests[d].hash, tag, 3) == 0) return LMC_US;
        for (int k = 0; k < e->nIds; k++)
            if (memcmp(e->ids[k], tag, 3) == 0) return LMC_US;
    }
    /* A link we are an endpoint of carries our traffic even though its
     * identifier belongs to neither side's announced set. `ours` is the whole
     * test — we only ever track links we are one end of. Answered apart from
     * the rows above so a viewer can tell an unnameable far end from a missing
     * one; see LMC_US_LINK. */
    for (int i = 0; i < NEI_LINKS_MAX; i++) {
        NeiLink* L = &r->nei->links[i];
        if (L->used && L->ours && memcmp(L->linkId, tag, 3) == 0) return LMC_US_LINK;
    }
    return LMC_OTHER;
}

uint8_t loraMonDescribe(const uint8_t* f, size_t len, uint8_t type) {
    if (!f || len < 1) return LMD_NONE;
    if (type == LORA_PKT_OURS) {
        switch (f[0]) {
            case SUPE_T_PRIVSYNC:  return LMD_PRIVSYNC;
            case SUPE_T_ANNOUNCE2: return LMD_ANNOUNCE2;
            case SUPE_T_HAVEDATA:  return LMD_HAVEDATA;
            case SUPE_T_GIMME:     return LMD_GIMME;
            case SUPE_T_THATSIT:   return LMD_THATSIT;
            case SUPE_T_BYE:       return LMD_BYE;
            case SUPE_T_RESEND:    return LMD_RESEND;
            default: return LMD_NONE;
        }
    }
    /* Everything else flew behind this interface's own framing byte, and the
     * Reticulum header begins right after it. A split's TAIL carries no header
     * and must not reach here — the caller knows which half it holds and names
     * a tail LMD_RNS_SPLIT itself. */
    if (len < 2) return type == LORA_PKT_RNODE ? LMD_RNODE : LMD_NONE;
    switch (f[1] & 0x03) {
        case 0x00: return LMD_RNS_DATA;
        case 0x01: return LMD_RNS_ANNOUNCE;
        case 0x02: return LMD_RNS_LINKREQ;
        default:   return LMD_RNS_PROOF;
    }
}

void loraMonPush(LoraRadio* r, uint8_t dir, uint32_t t_ms, uint16_t dur_ms,
                        uint16_t bytes, int16_t rssi, int16_t snr10, int8_t txp,
                        uint8_t type, uint16_t wait_ms, uint16_t own_ms,
                        uint8_t desc, const uint8_t tag[3], uint8_t cast) {
    /* Airtime rollup runs whether or not a viewer is open — the hour it covers
     * is longer than a viewer is typically up, so it can't be built on demand. */
    {
        uint32_t absIdx = t_ms / AIR_BUCKET_MS;
        AirBucket* b = &r->mon.air[absIdx % AIR_BUCKETS];
        if (b->absIdx != absIdx) { b->absIdx = absIdx; b->rxMs = b->txMs = 0; }
        if (dir) b->txMs += dur_ms; else b->rxMs += dur_ms;
    }
    /* Transmit seconds per channel over the rolling hour, on the channel the
     * frame actually flew. Detour airtime must not be credited to the hailing
     * channel: the contention band is chosen from this radio's own hailing
     * airtime, so a detour that landed there would make the node contend as
     * though it had spent the shared channel it deliberately did not — and the
     * detour would silently stop shortening its own future waits. */
    if (dir) r->mon.txAir[r->chNow < LORA_CH_MAX ? r->chNow : LORA_CH_HAIL]
                 .add((float)dur_ms / 1000.0f);
    /* The per-frame trace is verbose, not debug. Two levels, one discipline:
     * debug carries decisions and verbose carries frames, so at debug a detour
     * reads as a short story — offer, HERE, MANIFEST, train, home — with no
     * frame dumps between the lines, and at verbose the same story is
     * interleaved with every frame that flew. */
    if (logIsVerbose(TAG)) {
        if (dir) verb("lora/%d tx %u..%u (%ums) %uB ch%u txp=%ddBm waited=%ums",
                      r->idx, (unsigned)t_ms, (unsigned)(t_ms + dur_ms),
                      (unsigned)dur_ms, (unsigned)bytes, (unsigned)r->chNow,
                      (int)txp, (unsigned)wait_ms + (unsigned)own_ms);
        else     verb("lora/%d rx %u..%u (%ums) %uB ch%u rssi=%d snr=%.1f",
                      r->idx, (unsigned)t_ms, (unsigned)(t_ms + dur_ms),
                      (unsigned)dur_ms, (unsigned)bytes, (unsigned)r->chNow,
                      (int)rssi, (double)snr10 / 10.0);
    }
    if (!s_monWatched) return;

    IfMsg m = {};
    m.kind = IFM_MON;  m.radio = (uint8_t)r->idx;
    m.dir  = dir;      m.type  = type;
    m.ch   = r->chNow;
    m.txp  = txp;      m.t_ms  = t_ms;
    m.dur_ms = dur_ms; m.bytes = bytes; m.wait_ms = wait_ms; m.own_ms = own_ms;
    m.rssi = rssi;     m.snr10 = snr10;
    m.desc = desc;
    m.cast = cast;
    if (tag) memcpy(m.tag, tag, 3);
    if (!ifPost(&m)) r->mon.monDropped++;
}

/* The radio has left a channel (or is about to): record how long it listened
 * there. Called from the radio task on every retune, and once per maintenance
 * beat so a long stay on one channel is drawn as it accrues rather than only
 * when it ends.
 *
 * A dwell shorter than a millisecond is not a listening window, it is a
 * retune passing through — those would out-number the frames and say nothing. */
void loraMonDwell(LoraRadio* r, uint32_t now) {
    uint8_t ch = r->chNow < LORA_CH_MAX ? r->chNow : LORA_CH_HAIL;
    if (r->mon.dwellSince == 0) { r->mon.dwellSince = now ? now : 1; return; }
    uint32_t span = now - r->mon.dwellSince;
    if ((int32_t)span <= 0) return;
    uint32_t start = r->mon.dwellSince;
    r->mon.dwellSince = now ? now : 1;
    if (!s_monWatched || span == 0) return;
    if (span > 0xFFFF) span = 0xFFFF;
    IfMsg m = {};
    m.kind = IFM_MON;  m.radio = (uint8_t)r->idx;
    m.dir  = 2;        m.ch    = ch;
    m.t_ms = start;    m.dur_ms = (uint16_t)span;
    /* Whose slot this stay is. A meeting's slots belong to one peer for their
     * whole width whether or not a frame ever lands in them, and that is the
     * only place the fact exists: a viewer looking at a detour channel sees a
     * lane that may be entirely empty, and "we were listening HERE, for THEM"
     * is not derivable from frames that did not arrive. */
#if !defined(CONFIG_LORA_NO_SUPE)
    supeMeetingTag(r, m.tag);
#endif
    if (!ifPost(&m)) r->mon.monDropped++;
}

/* Publish the rolling one-hour airtime, per mille, per direction. The apps
 * compute every shorter window from the frame records; the hour needs more
 * history than a viewer holds, so it is the one figure the device publishes. */
static void loraPublishAirtime(LoraRadio* r, uint32_t now) {
    uint32_t absNow = now / AIR_BUCKET_MS;
    uint64_t rx = 0, tx = 0;
    for (int i = 0; i < AIR_BUCKETS; i++) {
        const AirBucket* b = &r->mon.air[i];
        if ((b->rxMs || b->txMs) && absNow - b->absIdx < AIR_BUCKETS) {
            rx += b->rxMs;
            tx += b->txMs;
        }
    }
    char kb[48];
    storageBegin();
    storageSet(rk(kb, sizeof kb, r->idx, "air1h.rx"), (int)(rx * 1000 / 3600000u));
    storageSet(rk(kb, sizeof kb, r->idx, "air1h.tx"), (int)(tx * 1000 / 3600000u));
    storageEnd();
}

/* Drop all of a radio's published nodes + FIFO (last viewer closed). */
static void loraMonClear(LoraRadio* r) {
    char pfx[32];
    snprintf(pfx, sizeof pfx, "lora.%d.packets", r->idx);
    storageDeleteTree(pfx);
    r->mon.pktHead = r->mon.pktCount = 0;
    /* The dwell that was still growing goes with them. A stay on one channel is
     * ONE record extended in place, so the key it extends has to exist: kept
     * across a clear, the next stay would rewrite a node that was just deleted —
     * at a timestamp from before the viewer opened, outside the ring that
     * expires it, and with nothing recorded at the time the radio is actually
     * listening. The lane then reads as "not here" from the moment the window
     * opens until the next retune breaks the chain, which is exactly the span a
     * viewer has its eyes on.
     *
     * Only this half. `dwellSince` is radio-task state and its own task drops it
     * on the watch edge — reaching across from here to clear it while a span is
     * being closed would be a cross-task write for nothing. */
    r->mon.dwellKeyMs = 0;
}

/* Publish the channel list the regime puts in force:
 * `lora.<n>.chans` = "<freqHz>,<bwHz>|…", index = channel, 0 = hailing.
 *
 * One key rather than a subtree: it is a handful of numbers that only change on
 * a config apply, and the viewers want all of it at once to label their graphs.
 * With no agility in force it is just the hailing channel, so a viewer can tell
 * the two cases apart by the entry count alone and needs no separate flag.
 *
 * **The list is the regime's channels, not the measured ones.** What a viewer
 * draws per channel is the TRAFFIC on it — the frames a detour put there, with
 * their airtime — and those records exist whether or not anything ever took a
 * noise reading there. The RSSI series is only a backdrop under that traffic,
 * and this radio publishes one (rssiSamplePoll), so the agile lanes draw their
 * frames against a plain background. Trimming this list to what is measured
 * would take the detour traffic off the screen with it. */
void publishChannels(LoraRadio* r) {
    char val[24 * LORA_CH_MAX];
    int  w = snprintf(val, sizeof val, "%u,%u",
                      (unsigned)r->cfgFreqHz, (unsigned)r->cfgBwHz);
#if !defined(CONFIG_LORA_NO_SUPE)
    int n = 0;
    const RegimeChan* ch = regimeChans(r->afa, &n);
    for (int i = 0; i < n && i + 1 < LORA_CH_MAX && w > 0 && w < (int)sizeof val; i++)
        w += snprintf(val + w, sizeof val - w, "|%u,%u",
                      (unsigned)ch[i].freqHz, (unsigned)ch[i].bwHz);
#else
    (void)w;    /* no agility: the hailing channel is the whole list */
#endif
    char kb[48];
    storageSet(rk(kb, sizeof kb, r->idx, "chans"), val);
}

/* The words a settings row shows for each state. Published beside the state
 * itself so neither UI carries a state->wording table; they render the string. */
static const char* stateWords(const char* state) {
    if (strcmp(state, "up") == 0)           return "up";
    if (strcmp(state, "starting") == 0)     return "starting";
    if (strcmp(state, "error") == 0)        return "error";
    if (strcmp(state, "unconfigured") == 0) return "unconfigured";
    return *state ? state : "down";
}

void publishState(LoraRadio* r, const char* state) {
    char b[48];
    storageBegin();
    storageSet(rk(b, sizeof b, r->idx, "state"), state);
    storageSet(rk(b, sizeof b, r->idx, "state_text"), stateWords(state));
    storageSet(rk(b, sizeof b, r->idx, "up"), r->running ? 1 : 0);
    storageEnd();
}


/* Channel-RSSI sample for the LoRaMon floor: one reading a second, handed to
 * the interface task to publish. Radio task; the reading is the same
 * getRSSI(false) carrier sense uses, so it costs one SPI transaction.
 *
 * **The radio never leaves the hailing channel to take a reading.** A frame
 * arriving during the retune is lost with nothing to show it existed, the cost
 * of the trip is per part (an LR2021's retune and receive restart is not an
 * SX126x's) and the window it must fit inside is per configuration, and no
 * consumer would read the result: channel access samples for itself, SUPE's
 * channel choice reads the airtime ledger, and the power controller works from
 * stated powers. The agile channels keep their graphs — those draw the traffic
 * a detour put there, which is recorded regardless — and simply have no noise
 * backdrop under it. See INTERNALS §18.3.
 *
 * **Only while a viewer is open.** The series is live-only decoration for the
 * LoRaMon graphs; nothing in channel access or SUPE reads it (carrier sense
 * takes its own samples and tracks its own floor). Unwatched, the beat is
 * skipped here and its deadline is not held in nextDeadline(), so an idle
 * radio task truly sleeps — a once-a-second wake with an SPI read is exactly
 * the standing battery cost this interface must not carry for a graph nobody
 * is looking at. A viewer opening flips s_monWatched and nudges the task; the
 * stale-by-then deadline samples on that very pass.
 *
 * **Carrier sense outranks measurement.** While the CSMA machine holds the
 * radio — or a transmit is on air, or a probe owns the chip, or a split is
 * still reassembling — no sample is taken and none is published. The series
 * goes absent for the duration and the viewers draw the gap. A reading taken
 * mid-contention would describe the transmission we are queued behind, not the
 * channel's resting noise, and channel access is the radio's actual job.
 *
 * The beat is not advanced when a sample is skipped, so sampling resumes as
 * soon as the radio is idle again rather than waiting out the rest of a second. */
void rssiSamplePoll(LoraRadio* r) {
    if (!s_monWatched) return;
    if (!r->running || !r->enabled) return;
    if ((int32_t)(xTaskGetTickCount() - r->mon.rssiNext) < 0) return;
    if (r->txActive || r->splitPending ||
#if !defined(CONFIG_LORA_NO_SUPE)
        supeHoldsRadio(r) ||
#endif
        r->csmaPhase != CSMA_IDLE || r->mtxPhase == MTXP_LBT) {
        /* Due, but the radio is spoken for. The deadline must move anyway:
         * nextDeadline() turns an overdue beat into a zero-length sleep, so
         * leaving it past-due here spun the task at full speed for as long
         * as the radio stayed busy — a detour storm read as 90% CPU. The
         * sample is 1 Hz telemetry; trying again shortly loses nothing. */
        r->mon.rssiNext = xTaskGetTickCount() + pdMS_TO_TICKS(100);
        return;
    }

    r->mon.rssiNext = xTaskGetTickCount() + pdMS_TO_TICKS(LORA_RSSI_SAMPLE_MS);

    IfMsg m = {};
    m.kind  = IFM_RSSI;
    m.radio = (uint8_t)r->idx;
    m.ch    = LORA_CH_HAIL;
    m.t_ms  = millis();
    for (int i = 0; i < LORA_CH_MAX; i++) m.chRssi[i] = LORA_RSSI_NONE;
    m.nch = 1;

    /* The hailing channel, in place: the radio is already on it and settled, so
     * the reading costs one transaction and no retune. **The radio does not
     * leave this channel to measure anything.** */
    m.chRssi[LORA_CH_HAIL] = (int16_t)lround(channelRssi(r));
    m.rssi = m.chRssi[LORA_CH_HAIL];

    if (!ifPost(&m)) r->mon.rssiDropped++;
}

/* ─────────────── interface task ───────────────
 *
 * The storage side of the interface: packet nodes, their expiry, the stats
 * flush, and the channel-RSSI series. Every storage write iface-lora makes on a
 * per-frame or per-second cadence happens here and not on the radio task — see
 * the note at IfMsg for why that separation is the point rather than tidiness.
 *
 * It blocks on the record queue with a short timeout, so it wakes for work and
 * otherwise ticks its own 1 Hz maintenance beat. Priority sits below the radio
 * task's, so a storage op that stalls on the storage task can never delay a
 * channel-access decision. */
/* Parked, it has nothing to block on, so it polls the stop flag. Only the
 * unpark latency depends on this. */
#define LORA_IF_PARK_POLL_MS 100

static TaskHandle_t  s_ifTask   = nullptr;
static volatile bool s_ifParked = false;

/* Publish the newest channel-RSSI sample as one key per radio:
 * `lora.<n>.rssi` = "<ms>|<ch0 dBm>". The device timestamp is in the value
 * rather than the key so a viewer can tell a fresh reading from a repeated one
 * and place it on the same clock the packet nodes use — and so a skipped beat
 * (carrier sense had the radio) simply leaves the key unchanged and reads as a
 * gap. One key rather than a node per sample: the series is live-only, so there
 * is no backlog to mirror and nothing to expire.
 *
 * The channel list is packed rather than singular because the agile channels
 * append to it unchanged once they exist. */
static void loraPublishRssi(LoraRadio* r, const IfMsg* m) {
    char kb[48], val[8 + 6 * LORA_CH_MAX];
    int  w = snprintf(val, sizeof val, "%u", (unsigned)m->t_ms);
    for (int i = 0; i < m->nch && i < LORA_CH_MAX && w > 0 && w < (int)sizeof val; i++)
        w += (m->chRssi[i] == LORA_RSSI_NONE)
                 ? snprintf(val + w, sizeof val - w, "|")
                 : snprintf(val + w, sizeof val - w, "|%d", (int)m->chRssi[i]);
    storageSet(rk(kb, sizeof kb, r->idx, "rssi"), val);
}

static void loraIfTaskMain(void*) {
    info("[%s-if] task up", TAG);
    bool       prevWatch = false;
    TickType_t lastBeat  = 0;
    TickType_t lastShift = 0;
    uint64_t   statsSig   = 0;
    const TickType_t shiftTicks =
        pdMS_TO_TICKS(Rolling1h::kBucketMinutes * 60u * 1000u);
    for (;;) {
        while (!s_stop) {
            /* Block until the next record or the next standing duty. With a
             * viewer open (or any UI pulling stats) the maintenance beat is
             * 1 Hz; with neither, the only duty left is aging the one-hour
             * rollups, one bucket per kBucketMinutes — so a dark idle node
             * wakes this task a few times an hour, not once a second. A watch
             * transition posts an IFM_KICK, so opening a viewer never waits
             * out the long block. */
            TickType_t now = xTaskGetTickCount();
            if (lastShift == 0) lastShift = now;
            bool active = s_monWatched || uiTelemetryWanted();
            TickType_t due = active ? lastBeat + pdMS_TO_TICKS(LORA_STATS_MIN_MS)
                                    : lastShift + shiftTicks;
            int32_t    rem  = (int32_t)(due - now);
            TickType_t wait = rem > 0 ? (TickType_t)rem : 0;

            IfMsg m;
            if (s_ifq && xQueueReceive(s_ifq, &m, wait) == pdTRUE) {
                if (m.radio < kNumRadios) {
                    LoraRadio* r = &s_radios[m.radio];
                    if (m.kind == IFM_MON)       loraMonRecord(r, &m);
                    else if (m.kind == IFM_RSSI) loraPublishRssi(r, &m);
                }
                /* A lull is a flush point: mid-storm the batch cap governs,
                 * and the lone frame of a quiet minute publishes right away. */
                if (uxQueueMessagesWaiting(s_ifq) == 0) monFlushPending();
            }

            /* A close acts on the wake that carried it, not on the next beat —
             * unwatched, the next beat may be minutes out, and the packets
             * subtree would sit there the whole wait. */
            if (prevWatch && !s_monWatched) {
                monFlushPending();
                for (int i = 0; i < kNumRadios; i++) loraMonClear(&s_radios[i]);
            }
            prevWatch = s_monWatched;

            /* Checked whether or not a record arrived: a steady stream of them
             * must not be able to starve expiry and the stats flush. */
            now = xTaskGetTickCount();
            if ((int32_t)(now - due) < 0) continue;
            lastBeat = now;
            monFlushPending();      /* nothing pending outlives a beat */

            /* Age every one-hour running total in the system, ours included.
             * One call covers them all; they linked themselves up at
             * construction. shiftAll ages exactly one bucket, so a wake that
             * arrives late (never by much — the idle block above is set to
             * this very cadence) catches up bucket by bucket. */
            while ((int32_t)(now - lastShift) >= (int32_t)shiftTicks) {
                lastShift += shiftTicks;
                Rolling1h::shiftAll();
            }

            /* Belt for the cached watch flag — the change subscription is the
             * prompt path. A transition it reveals is handled like any other:
             * a close drops the published subtree. */
            bool w = loraMonWatched();
            s_monWatched = w;
            if (prevWatch && !w)
                for (int i = 0; i < kNumRadios; i++) loraMonClear(&s_radios[i]);
            prevWatch = w;

            /* Stats: counters only move on a tx/rx event, so publish only when
             * the sum of them has changed since the last beat. */
            uint64_t sig = 0;
            for (int i = 0; i < kNumRadios; i++) {
                LoraRadio* r = &s_radios[i];
                sig += r->txBytes + r->rxBytes + r->txFrames + r->rxFrames +
                       r->crcErr + r->splitTimeouts + r->txDropped +
                       (uint32_t)r->rssiLast + (uint32_t)r->snrLast;
            }
            if (sig != statsSig) {
                statsSig = sig;
                for (int i = 0; i < kNumRadios; i++) publishStats(&s_radios[i]);
            }

            /* The pill rides this beat rather than the stats gate above: a
             * neighbour appearing or ageing out moves no byte counter, and the
             * enable switch moves none either. rnsdPillSet writes the same
             * values idempotently, so a beat that changes nothing costs the
             * storage actor one deduped op. */
            publishPill();

            /* LoRaMon expiry — 1 Hz while a viewer is open, so nodes age out of
             * the 1 h window even on an idle channel. */
            if (w) {
                uint32_t nowMs = millis();
                for (int i = 0; i < kNumRadios; i++) {
                    loraMonExpire(&s_radios[i], nowMs);
                    loraPublishAirtime(&s_radios[i], nowMs);
                }
            }
        }

        /* rns stop: the radio task parks too, so nothing more will be queued.
         * Drop whatever is still in flight and park on the same flag. */
        if (s_ifq) xQueueReset(s_ifq);
        s_monWatched = false;
        s_ifParked = true;
        while (s_stop) vTaskDelay(pdMS_TO_TICKS(LORA_IF_PARK_POLL_MS));
        s_ifParked = false;
    }
}

/* A viewer opened or closed (or WiFi came up, putting a stats-pulling UI in
 * reach): update the cached flag now and wake both tasks, because each may be
 * blocked on the long idle cadence the OLD state allowed — the radio task
 * resumes (or stops holding) the RSSI beat, the interface task re-blocks on
 * the cadence the new state calls for. */
static void onWatchChange(const char* /*key*/, const char* /*val*/) {
    s_monWatched = loraMonWatched();
    IfMsg m = {};
    m.kind  = IFM_KICK;
    m.radio = 0xFF;              /* matches no radio: wake, dispatch nothing */
    ifPost(&m);
    loraNudge();
}

/* Record queue + interface task, created once; both outlive a stop/start
 * cycle. The interface task sits one priority below the radio task: a storage
 * op that stalls must never be able to delay a channel-access decision. Its
 * stack is the smaller of the two — it holds no frame buffers, only the
 * record it popped and the key/value it formats. */
void loraMonStart(void) {
    if (!s_ifq) s_ifq = xQueueCreate(LORA_IFQ_DEPTH, sizeof(IfMsg));
    if (!s_ifTask) {
        s_ifTask = spawnTask(loraIfTaskMain, "lora-if", 4096, nullptr, 1,
                             CORE_SECONDARY_NO_LCD, STACK_PSRAM);
        storageSubscribeChanges("sys.stats.web_loramon", onWatchChange);
        storageSubscribeChanges("sys.stats.lcd_loramon", onWatchChange);
        storageSubscribeChanges("wifi.sta.up",           onWatchChange);
        storageSubscribeChanges("wifi.ap.up",            onWatchChange);
    }
}

bool loraMonParked(void) { return s_ifParked; }

/* LoRaMon expiry FIFO: allocated once, kept across config cycles. */
void loraMonInit(LoraRadio* r) {
    if (r->mon.pktMs) return;
    r->mon.pktMs   = (uint32_t*)gp_alloc((size_t)LORA_MON_CAP * sizeof(uint32_t));
    r->mon.pktCap  = r->mon.pktMs ? LORA_MON_CAP : 0;
    r->mon.pktHead = r->mon.pktCount = 0;
}

#endif  /* CONFIG_LORA0_CS_PIN */
