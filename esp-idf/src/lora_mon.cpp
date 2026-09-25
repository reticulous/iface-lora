/**
 * lora_mon — telemetry: the radio→interface record queue, the LoRaMon
 * per-frame ring and its storage nodes, the stats flush, the channel-RSSI
 * series, and the interface task that owns every storage write.
 *
 * Half of this file is the LoRaMon recorder, and the only thing that ever reads
 * what it writes is the loramon straddle — the LCD app and the browser window.
 * So it is gated on that straddle being staged (CONFIG_STRADDLE_LORAMON), the
 * same way SUPE is gated on CONFIG_LORA_NO_SUPE: `--without loramon` and the
 * per-frame nodes, the neighbourhood rows, the channel-RSSI series, the rolling
 * hour, the 1 Hz sample beat and the expiry FIFO (16 KB per radio) are all
 * absent from the image rather than merely idle. What is left is the stats
 * flush, the status pill, the channel list and the state keys — what the
 * settings pane and the status bar read, which have their own audience.
 *
 * Nothing outside this file needs the gate: every call site the rest of the
 * straddle makes (loraMonPush, loraMonDwell, rssiSamplePoll) already asks
 * loraMonOpen() first, and with no viewer in the build that is a constant
 * false.
 */
#include "lora_priv.h"

#include <cstdarg>
#include <ctime>
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
 * off the radio task is what keeps the radio's channel-access deadlines.
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
#if CONFIG_STRADDLE_LORAMON
    /* MON: the two detail fields, empty unless a viewer asked for them. They
     * ride in the message rather than being looked up on the interface task
     * because the frame they are read from exists only on the radio task, and
     * only for as long as the record takes to post. */
    LoraMonExt ext;
#endif
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
 * belt against a missed callback.
 *
 * With no viewer straddle in the build it is a constant, and every gate that
 * reads it — here, in loraMonPush, in loraMonDwell, in rssiSamplePoll, in
 * nextDeadline — folds away with it. That is what keeps the gate out of the
 * radio task's code: a call site asks the same question either way. */
#if CONFIG_STRADDLE_LORAMON
static volatile bool s_monWatched = false;
/* The same, for the `detailed` toggle. Its own flag because it is its own
 * appetite: a viewer may watch the traffic for hours and want the detail for a
 * minute of it, and the fields cost a deeper read of every frame here and ~34
 * bytes in every record node for as long as the ring holds it. */
static volatile bool s_monDetail = false;
#else
static constexpr bool s_monWatched = false;
static constexpr bool s_monDetail  = false;
#endif

/* The radio task's read of it: true while a LoRaMon viewer (web or LCD) is
 * open. What hangs off this is not just recording but wake cycles — see
 * rssiSamplePoll and nextDeadline. */
bool loraMonOpen(void) { return s_monWatched; }
bool loraMonDetail(void) { return s_monDetail; }

/* Post to the interface task. Never blocks: a full queue means the storage side
 * is behind, and dropping telemetry is the correct answer. Returns false so the
 * caller can count the loss against the radio it belongs to. */
static bool ifPost(const IfMsg* m) {
    if (!s_ifq) return false;
    return xQueueSend(s_ifq, m, 0) == pdTRUE;
}

#if CONFIG_STRADDLE_LORAMON
/* True while something is actually reading the neighbourhood — today the web
 * LoRaMon's packet hover, tomorrow a graph view. Its own key, not LoRaMon's:
 * these are different appetites. LoRaMon wants frames and can run on an LCD
 * that needs no published peers at all; a graph wants the peer table and no
 * frames. Publishing on the wrong one costs a rewrite of every peer row on
 * every stats beat for a reader that is not there. */
static bool loraPeersWatched(void) {
    /* A browser-only key, so it is gated on the browser's link for the same
     * reason the web half of loraMonWatched() is: the tab that raised it is the
     * only party that can lower it, and a vanished tab never gets to. */
    return storageGetInt("sys.stats.web_peers", 0) != 0 &&
           storageGetInt("webrtc.up", 0) != 0;
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
#endif  /* CONFIG_STRADDLE_LORAMON — the neighbourhood publisher */

/* SUPE's per-peer measurements, one record per peer-table slot, every
 * LORA_MEAS_MS while the table holds anyone — for a reader outside this binary
 * that holds a destination hash and wants to know the link to it: lxmf's Ping
 * and its contact bars, the web contact list. The measurement is a path loss
 * (§15): a level read here against the power the other side stated for it,
 * in each direction one exists for. `loss_from` is them→us, the fresher of the
 * hailing pair and the step pair; `loss_to` is us→them, the peer's own account
 * of how our frame landed. A field is absent when it is not known — only a
 * SUPE peer states a power — and a slot that empties is deleted. A reader
 * resolves a peer by the first six hex characters of its destination hash in
 * `tags`.
 *
 *   lora.<n>.meas.<slot>.tags       6-hex prefixes the node answers to
 *   lora.<n>.meas.<slot>.name       announced first-word names
 *   lora.<n>.meas.<slot>.loss_to    dB, us→them          .snr_to   dB×10 the peer reported
 *   lora.<n>.meas.<slot>.txp_to     dBm we sent it at    .to_ts    unix s of that reading
 *   lora.<n>.meas.<slot>.loss_from  dB, them→us          .snr_from dB×10 read here
 *   lora.<n>.meas.<slot>.peer_txp   dBm the peer stated  .from_ts  unix s of that reading
 *   lora.<n>.meas.<slot>.rssi       dBm, LAST heard      .snr      dB×10, last heard
 *   lora.<n>.meas.<slot>.heard_ts   unix s last heard    .txp      dBm we last sent to it at
 *
 * Only the two losses need SUPE — they are a level and a power the OTHER end
 * stated, and nobody outside the protocol states one. `rssi`, `snr` and `txp`
 * are always there: a level this radio read, and the power this radio sent at.
 *
 * Each direction is a loss with the reading it was measured from — the frame's
 * signal-to-noise and the power it went out at — because a loss alone cannot
 * say whether a link is weak or merely quiet, and the two numbers it is the
 * difference of are what a person reads. rssi/snr are the LAST reading and not
 * the best one: an envelope describes a row's whole life, so a node that walked
 * out of range an hour ago would go on publishing the level it once managed.
 *
 * Separate fields rather than a packed string: the readers are other firmware
 * tasks and the browser, each after one or two of them, and storage deduplicates
 * an unchanged value, so a beat that measured nothing new costs the storage
 * actor almost nothing. Timestamps rather than ages for the same reason — an
 * age changes on every beat, a timestamp only when something was heard. */
static void publishMeas(LoraRadio* r) {
    if (!r->nei) return;
    char k[48];
    uint32_t nowMs = millis();
    time_t   nowS  = time(nullptr);
    auto tsOf = [&](uint32_t ms) { return (int)(nowS - (time_t)((nowMs - ms) / 1000u)); };
    for (int i = 0; i < NEI_MAX; i++) {
        Neighbor* e = &r->nei->nei[i];
        snprintf(k, sizeof k, "lora.%d.meas.%d", r->idx, i);
        if (!e->used || peersIsLocal(e)) { storageDeleteTree(k); continue; }
        std::string base = k;

        uint8_t tg[NEI_DESTS_MAX + NEI_IDS_MAX + 1][3];
        int nt = 0;
        auto addTag = [&](const uint8_t* b) {
            for (int j = 0; j < nt; j++) if (memcmp(tg[j], b, 3) == 0) return;
            if (nt < (int)(sizeof tg / sizeof tg[0])) memcpy(tg[nt++], b, 3);
        };
        if (e->haveNode4) addTag(e->node4);
        for (int d = 0; d < e->nDests; d++) addTag(e->dests[d].hash);
        for (int q = 0; q < e->nIds; q++)   addTag(e->ids[q]);
        std::string tags;
        for (int j = 0; j < nt; j++) {
            char t[8];
            snprintf(t, sizeof t, "%s%02x%02x%02x", j ? "," : "", tg[j][0], tg[j][1], tg[j][2]);
            tags += t;
        }
        char names[NEI_NAME_MAX * 3];
        peersNodeNames(e, names, sizeof names);

        auto setOr = [&](const char* f, bool have, int v) {
            std::string key = base + "." + f;
            if (have) storageSet(key.c_str(), v);
            else      storageUnset(key.c_str());
        };
        int      lossFrom = 0;
        uint32_t fromMs   = 0;
        int16_t  snrFrom  = 0;
        int8_t   peerTxp  = 0;
        bool     haveFrom = peersLossFrom(e, &lossFrom, &fromMs, &snrFrom, &peerTxp);

        storageBegin();
        storageSet((base + ".tags").c_str(), tags);
        storageSet((base + ".name").c_str(), names);
        setOr("loss_from", haveFrom,        lossFrom);
        setOr("snr_from",  haveFrom,        snrFrom);
        setOr("peer_txp",  haveFrom,        peerTxp);
        setOr("from_ts",   haveFrom,        haveFrom ? tsOf(fromMs) : 0);
        setOr("loss_to",   e->haveApRpt,    (int)e->apRptTxp - (int)e->apRptRssi);
        setOr("snr_to",    e->haveApRpt,    e->apRptSnr10);
        setOr("txp_to",    e->haveApRpt,    e->apRptTxp);
        setOr("to_ts",     e->haveApRpt,    e->haveApRpt ? tsOf(e->apRptMs) : 0);
        setOr("rssi",      e->haveSig,      e->rssiLast);
        setOr("snr",       e->haveSig,      e->snrLast10);
        /* The power our last frame to this peer went out at. Always present:
         * where no per-peer power has been decided — nothing sent yet, or SUPE
         * compiled out, which is every peer — every frame leaves at the radio's
         * configured power, and that IS the answer. It is the only half of a
         * link a node outside the protocol can state, so it must not go
         * missing with the protocol. */
        storageSet((base + ".txp").c_str(),
                   e->haveApLastTxp ? (int)e->apLastTxp : (int)r->cfgTxp);
        setOr("heard_ts",  e->lastHeardMs != 0, e->lastHeardMs ? tsOf(e->lastHeardMs) : 0);
        storageEnd();
    }
}

/* One forgotten slot, off both publications at once (lora_mon.h). Neither
 * publisher's own beat can be relied on to do it: the neighbourhood record is
 * written only while a LoRaMon viewer is open, and the measurements only when a
 * frame has moved — and a radio that has just forgotten everyone may hear
 * nothing for hours. */
void loraPeerPubForget(LoraRadio* r, int slot) {
    char k[48];
    snprintf(k, sizeof k, "lora.%d.peers.%d", r->idx, slot);
    storageDeleteTree(k);
    snprintf(k, sizeof k, "lora.%d.meas.%d", r->idx, slot);
    storageDeleteTree(k);
}

/* True while any radio's table holds a neighbour worth publishing. */
static bool measWanted(void) {
    for (int i = 0; i < kNumRadios; i++) {
        LoraRadio* r = &s_radios[i];
        if (r->nei && peersOtherCount(r->nei) > 0) return true;
    }
    return false;
}

/* The traffic the measurements can have changed on: frames in and out, over
 * every radio. A measurement is made of frames — nothing is transmitted in
 * order to measure — so an unchanged sum means the last publication still
 * stands, and the 15 s beat is armed only while this differs from the sum at
 * the last publication. A quiet neighbourhood costs no wake at all. */
static uint64_t measTrafficSig(void) {
    uint64_t sig = 0;
    for (int i = 0; i < kNumRadios; i++)
        sig += s_radios[i].rxFrames + s_radios[i].txFrames;
    return sig;
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
#if CONFIG_STRADDLE_LORAMON
    if (loraPeersWatched()) publishPeers(r);
#endif
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

/* ─────────────── LoRaMon: recording, windows, publish ───────────────
 *
 * Everything from here to the close of this region exists to feed the loramon
 * straddle's two viewers, and is in the image only when one of them is. */
#if CONFIG_STRADDLE_LORAMON

/* True while a LoRaMon viewer (web or LCD) is open — gates recording.
 *
 * The browser's flag counts only while its link is up. A tab raises that flag
 * about itself and can only lower it while it is still there: one that crashed,
 * slept or lost its WiFi leaves it standing, and recording for a viewer that is
 * gone costs a 1 Hz radio sample, a 1 Hz interface beat against light sleep and
 * a subtree growing to LORA_MON_CAP — until something reboots. `webrtc.up` is
 * the link itself, so the falling edge arrives within seconds of the tab
 * vanishing, whether or not the tab got a word in. An LCD viewer is local and
 * needs no link. The same gate is what the CPU sampler applies to its own web
 * flag (spangap-core pm.cpp).
 *
 * It costs one thing worth knowing: a web flag set by hand on a node with no
 * browser session (`store set sys.stats.web_loramon 1`) no longer records.
 * `sys.stats.lcd_loramon` is the flag to set from the CLI. */
static bool loraMonWatched(void) {
    return (storageGetInt("sys.stats.web_loramon", 0) &&
            storageGetInt("webrtc.up", 0)) ||
           storageGetInt("sys.stats.lcd_loramon", 0);
}

/* A viewer wants the detail fields too. Same shape and the same link gate; a
 * detail flag left standing by a vanished tab would cost a third more heap per
 * record for a reader that is gone. Meaningless without a viewer open, and
 * cheap enough to ask that way round rather than tracking the pair. */
static bool loraMonDetailWatched(void) {
    return (storageGetInt("sys.stats.web_details", 0) &&
            storageGetInt("webrtc.up", 0)) ||
           storageGetInt("sys.stats.lcd_details", 0);
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
 *   r|rssi|snr|dur|bytes|type|ch|desc|cast[|tag[|to|subj|hash]]
 *   t|txp|dur|bytes|type|wait|ch|own|desc|cast[|tag[|to|subj|hash]]
 *   a|ch|dur[|tag]
 *
 * The last is a DWELL: the radio was tuned to that channel and listening for
 * that long, ending where the next one begins. Its tag is the meeting whose
 * slot the stay is — present only inside one. The leading token is the
 * direction; snr is deci-dB; ch is the channel, 0 being the reticulum hailing
 * channel; desc is what the frame is (LMD_*); cast is who it was aimed at
 * (LMC_*); tag is six hex characters naming the peer, absent when unknown.
 * `to`, `subj` and `hash` are the detail fields (LoraMonExt) and ride only
 * while a viewer has the `detailed` toggle on — which is why the tag's slot is
 * written empty rather than omitted when they follow it. Then age old nodes out.
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
    /* Big enough for the longest record plus both detail fields at their full
     * width and the tag slot they need in front of them. */
    char key[40], val[120];

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
    size_t o = strlen(val);
    bool haveTag = (m->tag[0] | m->tag[1] | m->tag[2]) != 0;
    bool haveExt = m->ext.to[0] != '\0' || m->ext.subj[0] != '\0' ||
                   m->ext.hash[0] != '\0';
    if (haveTag)
        o += snprintf(val + o, sizeof val - o, "|%02x%02x%02x",
                      m->tag[0], m->tag[1], m->tag[2]);
    /* The detail fields sit behind the tag, so the tag's slot has to be there
     * to be counted past — empty where there was no tag to put in it. They are
     * absent altogether while nobody asked for them, which is what makes the
     * toggle worth having: a record written with the detail off is the record
     * it always was. */
    else if (haveExt)
        o += snprintf(val + o, sizeof val - o, "|");
    if (haveExt)
        snprintf(val + o, sizeof val - o, "|%s|%s|%s",
                 m->ext.to, m->ext.subj, m->ext.hash);
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
#endif  /* CONFIG_STRADDLE_LORAMON — the recorder's storage half */

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
        /* HAIL is the one that names a peer: type, regime/version, then the
         * tag it is asking about. The meeting frames name a schedule, not a
         * node, so they have none to give. */
        if (f[0] == SUPE_T_HAIL && len >= 2 + SUPE_TAG_LEN)
            memcpy(out, f + 2, SUPE_TAG_LEN);
        return;
    }
    if (len >= 1 + 2 + 3) memcpy(out, f + 1 + 2, 3);
}

/* The two SUPE frames that name their own sender.
 *
 * HAIL's sender_ident sits behind the tag, the power and the salt. It is
 * optional on the wire — a hail from a node with nothing to say about itself
 * omits it — so the length is what says whether there is one.
 *
 * ANNOUNCE is nothing BUT a statement of who is speaking: the identities are
 * its payload, and the first of them is the one annIngest resolves the frame's
 * node through, so it is the one that resolves through a published tag set too
 * (it is that row's node4, one of its idents, or the front of one of its
 * destinations, whichever the row was found by). */
bool loraMonSenderOf(const uint8_t* f, size_t len, uint8_t type, uint8_t out[3]) {
    if (!f || type != LORA_PKT_OURS || len < 1) return false;
    if (f[0] == SUPE_T_ANNOUNCE) {
        if (len < SUPE_ANN_BASE + SUPE_ID_LEN) return false;
        memcpy(out, f + SUPE_ANN_BASE, SUPE_TAG_LEN);
        return true;
    }
    if (f[0] != SUPE_T_HAIL || len < SUPE_HAIL_ID_LEN) return false;
    memcpy(out, f + SUPE_HAIL_LEN, SUPE_TAG_LEN);
    return true;
}

/* The two well-known Reticulum control destinations, hashed the way RNS hashes
 * a destination that has no identity behind it: SHA-256 of "<app>.<aspects>",
 * the first ten bytes of that hashed again, the first sixteen of the result.
 * Computed once and kept — they are constants of the protocol, not of this
 * node, and they are the only way to tell a path request from any other plain
 * broadcast: nothing in the header says so, the ADDRESS does. */
static uint8_t s_ctrlPathReq[16], s_ctrlTunnel[16];
static bool    s_ctrlHashed = false;
static void monCtrlHashes(void) {
    if (s_ctrlHashed) return;
    static const char* const kNames[2] = { "rnstransport.path.request",
                                           "rnstransport.tunnel.synthesize" };
    uint8_t* const out[2] = { s_ctrlPathReq, s_ctrlTunnel };
    for (int i = 0; i < 2; i++) {
        uint8_t h1[RNSD_HASH_LEN], h2[RNSD_HASH_LEN];
        rnsdSha256((const uint8_t*)kNames[i], strlen(kNames[i]), h1);
        rnsdSha256(h1, 10, h2);          /* NAME_HASH_LENGTH is 80 bits */
        memcpy(out[i], h2, 16);
    }
    s_ctrlHashed = true;
}

#if CONFIG_STRADDLE_LORAMON
/* Up to `max` characters of a cleartext payload, for the packets that have one:
 * a PLAIN destination is unencrypted by definition. Anything unprintable ends
 * the copy rather than becoming a dot — a field that stops early says "that is
 * all the text there was", which is true, where a run of dots would suggest
 * bytes worth looking at. */
static void monText(char* dst, size_t max, const uint8_t* p, size_t n) {
    size_t w = 0;
    for (size_t i = 0; i < n && w + 1 < max; i++) {
        if (p[i] < 0x20 || p[i] > 0x7e) break;
        dst[w++] = (char)p[i];
    }
    dst[w] = '\0';
}

/* The two detail fields, read out of one frame. Everything here is CLEARTEXT —
 * the header, which is never encrypted, and the payloads of the packets that
 * carry none (an announce, a path request, a plain destination). What a packet
 * carries past that is behind a key this node does not have, and the fields
 * stay empty rather than guess.
 *
 * `complete` is false for the head of a split packet: its header is all there,
 * so everything read out of the header still holds, but a hash taken over half
 * a packet is not that packet's hash and must not be offered as one.
 *
 * RADIO TASK, and only while a viewer asked for the detail. */
static void monExtFill(LoraRadio* r, const uint8_t* f, size_t len,
                       uint8_t type, uint8_t desc, bool complete,
                       LoraMonExt* ext) {
    char hx[7];
    if (type == LORA_PKT_OURS) {
        /* Our own protocol has no Reticulum header to read. What it does carry,
         * where it carries anything, is who is speaking — and that is the fact
         * a person watching an exchange wants beside it. */
        uint8_t s[3];
        if (loraMonSenderOf(f, len, type, s)) {
            loraHex(hx, s, 3);
            snprintf(ext->subj, sizeof ext->subj, "%s", hx);
        }
        return;
    }
    if (type == LORA_PKT_BAD) return;        /* nothing in it was readable */

    RnsHdr h;
    if (len < 2 || !rnsParse(f + 1, len - 1, &h)) return;
    monCtrlHashes();

    /* Where the packet is going, and how far it has come. The record's `tag`
     * names the node THIS HOP was addressed to — on a packet in transport that
     * is the relay, which is the neighbour a person watching the air is dealing
     * with and what the peer pills want. This is the other end of the journey,
     * which nothing else in the record carries. */
    loraHex(hx, h.dest, 3);
    snprintf(ext->to, sizeof ext->to, "%s h%u", hx, (unsigned)h.hops);

    switch (desc) {
    case LMD_RNS_PATHREQ: {
        /* The whole point of the packet, and not its destination: the address
         * being asked about rides in the payload, followed by the asking
         * transport's own identity where the request carries one. */
        if (h.dataLen < 16) break;
        char req[7]; loraHex(req, h.data, 3);
        if (h.dataLen >= 32) {
            char by[7]; loraHex(by, h.data + 16, 3);
            snprintf(ext->subj, sizeof ext->subj, "%s by %s", req, by);
        } else {
            snprintf(ext->subj, sizeof ext->subj, "%s", req);
        }
        break;
    }
    case LMD_RNS_ANNOUNCE:
    case LMD_RNS_PATHRESP: {
        /* What the announced destination IS — the aspect behind its name hash,
         * which sits in the clear behind the public key. rnsd holds the only
         * table that can turn one back into a name, and it is the one fact
         * about an announce that a person reads rather than decodes. */
        if (h.dataLen < 74) break;
        const uint8_t* nameHash = h.data + 64;
        const char* label = rnsNameLabel(nameHash);
        if (label) snprintf(ext->subj, sizeof ext->subj, "%s", label);
        else { loraHex(hx, nameHash, 3); snprintf(ext->subj, sizeof ext->subj, "?%s", hx); }
        break;
    }
    case LMD_RNS_LINKREQ: {
        /* The link this packet creates. Every later packet of that session is
         * addressed to it, so it is what joins a link's setup to its traffic —
         * the one thread through an otherwise opaque conversation. */
        if (!complete) break;
        uint8_t lid[16];
        rnsPacketHash(&h, f + 1, len - 1, true, lid);
        loraHex(hx, lid, 3);
        snprintf(ext->subj, sizeof ext->subj, "%s", hx);
        break;
    }
    case LMD_RNS_PROOF:
    case LMD_RNS_LRPROOF:
    case LMD_RNS_LINKPROOF:
    case LMD_RNS_RESPROOF: {
        /* The packet being proven. An explicit proof carries its full 32-byte
         * hash ahead of the signature, which is exactly what every other record
         * here puts in this field — so a proof and the frame it answers meet on
         * the graph. An implicit proof is signature alone and names nothing. */
        if (h.dataLen >= 96) { loraHex(hx, h.data, 3);
                               snprintf(ext->subj, sizeof ext->subj, "%s", hx); }
        break;
    }
    case LMD_RNS_PLAIN:
    case LMD_RNS_TUNNEL:
        monText(ext->subj, sizeof ext->subj, h.data, h.dataLen);
        break;
    case LMD_RNS_CACHEREQ:
        /* The packet it is asking for, by the same hash every other record
         * here carries — so a cache request and what answers it meet. */
        if (h.dataLen >= 16) { loraHex(hx, h.data, 3);
                               snprintf(ext->subj, sizeof ext->subj, "%s", hx); }
        break;
    case LMD_RNS_LINKDATA:  case LMD_RNS_CHANNEL:   case LMD_RNS_REQUEST:
    case LMD_RNS_RESPONSE:  case LMD_RNS_KEEPALIVE: case LMD_RNS_LINKIDENT:
    case LMD_RNS_LINKCLOSE: case LMD_RNS_LINKRTT:   case LMD_RNS_COMMAND:
    case LMD_RNS_CMDSTATUS: case LMD_RNS_RESPART:   case LMD_RNS_RESADV:
    case LMD_RNS_RESREQ:    case LMD_RNS_RESHMU:
    case LMD_RNS_RESCANCEL: case LMD_RNS_RESCANCEL_RX: {
        /* A packet on an established link is addressed to the link and says
         * nothing else about who it is with — the payload that would is
         * encrypted. The peer table watched the link being set up, though, so
         * the destination it was dialled to is on file. */
        NeiLink* L = r->nei ? peersLinkFindBy3(r->nei, h.dest) : nullptr;
        if (L && L->haveDest) { loraHex(hx, L->dest, 3);
                                snprintf(ext->subj, sizeof ext->subj, "%s", hx); }
        break;
    }
    default:
        break;                       /* the kind has nothing of its own to say */
    }

    /* And what this packet IS, for everything that is not a proof: the hash a
     * proof for it will carry, so a viewer can join the two. Its own field, not
     * a case of `subj`, because the packets most often proven — a link's data —
     * already have a subject worth keeping. A hash taken over the head of a
     * split is not the packet's hash, so a split head carries none. */
    if (complete && !loraMonIsProof(desc)) {
        uint8_t ph[16];
        rnsPacketHash(&h, f + 1, len - 1, false, ph);
        loraHex(hx, ph, 3);
        snprintf(ext->hash, sizeof ext->hash, "%s", hx);
    }
}

#endif  /* CONFIG_STRADDLE_LORAMON — the detail fields */

/* Aimed at everyone, by what the frame IS — see the header. */
bool loraMonIsBroadcast(uint8_t desc) {
    return desc == LMD_RNS_ANNOUNCE  || desc == LMD_ANNOUNCE ||
           desc == LMD_RNS_PATHRESP  || desc == LMD_RNS_PATHREQ ||
           desc == LMD_RNS_TUNNEL;
}

/* A proof of something, in any of its shapes — see the header. */
bool loraMonIsProof(uint8_t desc) {
    return desc == LMD_RNS_PROOF     || desc == LMD_RNS_LRPROOF ||
           desc == LMD_RNS_LINKPROOF || desc == LMD_RNS_RESPROOF;
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
                     uint8_t* desc, uint8_t* whole, uint8_t tag[3],
                     LoraMonExt* ext) {
    if (ext) { ext->to[0] = '\0'; ext->subj[0] = '\0'; }
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
    /* A split frame with no head pending is a tail whose head did not arrive —
     * inside a meeting the head is resent after the tail, in the repair round.
     * A head is always the full frame, so a shorter one cannot be a head, and
     * reading a packet header out of its payload bytes would name a random
     * packet type and a random node. It waits for nothing: the head, when it
     * comes, opens a pending of its own. */
    if (split && len < 1 + RNODE_MAX_PAYLOAD) {
        *desc  = LMD_RNS_SPLIT;
        *whole = LMD_RNS_SPLIT;
        tag[0] = tag[1] = tag[2] = 0;
        return;
    }
    *desc  = loraMonDescribe(f, len, type);
    *whole = *desc;
    loraMonTagOf(f, len, type, tag);
    /* The control destinations name no node. Their address is a constant every
     * node computes the same way, so carrying it as the frame's tag would put
     * one meaningless six-hex "peer" above every path request on the graph, and
     * group them into a run as though they were a conversation with it. */
    if (*desc == LMD_RNS_PATHREQ || *desc == LMD_RNS_TUNNEL)
        tag[0] = tag[1] = tag[2] = 0;
#if CONFIG_STRADDLE_LORAMON
    if (ext && s_monDetail) monExtFill(r, f, len, type, *desc, !split, ext);
#endif
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
    /* Addresses that mean us without naming us. A delivery proof is addressed
     * to the truncated hash of the packet it proves (SUPE.md §5), and that
     * hash names this node for as long as the receipt window lasts: the
     * engine holds one per single-destination packet we sent or relayed, and
     * the peer table one per proof we elicited from a direct neighbour. Asked
     * apart from the rows above because a hash is not a row. */
    if (peersPendPeek(r->nei, tag, 3)) return LMC_US;
#if !defined(CONFIG_LORA_NO_SUPE)
    if (r->supe && supeEngTagIsOurs(&r->supe->eng, tag)) return LMC_US;
#endif
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
            case SUPE_T_HAIL:  return LMD_HAIL;
            case SUPE_T_ANNOUNCE: return LMD_ANNOUNCE;
            case SUPE_T_GOT:  return LMD_GOT;
            case SUPE_T_READY:     return LMD_READY;
            case SUPE_T_END:   return LMD_END;
            case SUPE_T_BYE:       return LMD_BYE;
            case SUPE_T_RESEND:    return LMD_RESEND;
            default: return LMD_NONE;
        }
    }
    /* A frame that failed its CRC decoded to nothing. Reading a packet type out
     * of bytes the air corrupted puts a confident name on the graph that is
     * true only by luck — and the bar is already purple, which says the whole
     * of what can honestly be said about it. */
    if (type == LORA_PKT_BAD) return LMD_NONE;

    /* Everything else flew behind this interface's own framing byte, and the
     * Reticulum header begins right after it. A split's TAIL carries no header
     * and must not reach here — the caller knows which half it holds and names
     * a tail LMD_RNS_SPLIT itself.
     *
     * Three cleartext fields decide the name, in this order: the CONTEXT byte,
     * which says what the packet is for and is the same whatever carries it;
     * then the DESTINATION TYPE for the cases with no context of their own; and
     * the packet-type bits underneath both. A packet type alone cannot tell a
     * path request from any other data packet, nor a path response from an
     * ordinary announce — which is how both used to read. */
    RnsHdr h;
    if (len < 2 || !rnsParse(f + 1, len - 1, &h))
        return type == LORA_PKT_RNODE ? LMD_RNODE : LMD_NONE;

    switch (h.ptype) {
    case NEI_PT_ANNOUNCE:
        return h.ctx == NEI_CTX_PATH_RESP ? LMD_RNS_PATHRESP : LMD_RNS_ANNOUNCE;
    case NEI_PT_LINKREQ:
        return LMD_RNS_LINKREQ;
    case NEI_PT_PROOF:
        switch (h.ctx) {
            case NEI_CTX_LRPROOF:   return LMD_RNS_LRPROOF;
            case NEI_CTX_LINKPROOF: return LMD_RNS_LINKPROOF;
            case NEI_CTX_RES_PRF:   return LMD_RNS_RESPROOF;
            default:                return LMD_RNS_PROOF;
        }
    default: break;                                   /* NEI_PT_DATA */
    }

    switch (h.ctx) {
        case NEI_CTX_RESOURCE:  return LMD_RNS_RESPART;
        case NEI_CTX_RES_ADV:   return LMD_RNS_RESADV;
        case NEI_CTX_RES_REQ:   return LMD_RNS_RESREQ;
        case NEI_CTX_RES_HMU:   return LMD_RNS_RESHMU;
        case NEI_CTX_RES_ICL:   return LMD_RNS_RESCANCEL;
        case NEI_CTX_RES_RCL:   return LMD_RNS_RESCANCEL_RX;
        case NEI_CTX_CACHE_REQ: return LMD_RNS_CACHEREQ;
        case NEI_CTX_REQUEST:   return LMD_RNS_REQUEST;
        case NEI_CTX_RESPONSE:  return LMD_RNS_RESPONSE;
        case NEI_CTX_COMMAND:   return LMD_RNS_COMMAND;
        case NEI_CTX_CMD_STAT:  return LMD_RNS_CMDSTATUS;
        case NEI_CTX_CHANNEL:   return LMD_RNS_CHANNEL;
        case NEI_CTX_KEEPALIVE: return LMD_RNS_KEEPALIVE;
        case NEI_CTX_LINKIDENT: return LMD_RNS_LINKIDENT;
        case NEI_CTX_LINKCLOSE: return LMD_RNS_LINKCLOSE;
        case NEI_CTX_LRRTT:     return LMD_RNS_LINKRTT;
        default: break;                               /* no context of its own */
    }

    switch (h.dtype) {
    case NEI_DT_PLAIN:
        /* A plain destination is unencrypted by definition, so what it is
         * turns on WHICH destination — and the two that matter are constants
         * of the protocol every node computes the same way. */
        monCtrlHashes();
        if (memcmp(h.dest, s_ctrlPathReq, 16) == 0) return LMD_RNS_PATHREQ;
        if (memcmp(h.dest, s_ctrlTunnel,  16) == 0) return LMD_RNS_TUNNEL;
        return LMD_RNS_PLAIN;
    case NEI_DT_GROUP: return LMD_RNS_GROUP;
    case NEI_DT_LINK:  return LMD_RNS_LINKDATA;
    default:           return LMD_RNS_DATA;           /* SINGLE, no context */
    }
}

void loraMonPush(LoraRadio* r, uint8_t dir, uint32_t t_ms, uint16_t dur_ms,
                        uint16_t bytes, int16_t rssi, int16_t snr10, int8_t txp,
                        uint8_t type, uint16_t wait_ms, uint16_t own_ms,
                        uint8_t desc, const uint8_t tag[3], uint8_t cast,
                        const LoraMonExt* ext) {
#if CONFIG_STRADDLE_LORAMON
    /* Airtime rollup runs whether or not a viewer is open — the hour it covers
     * is longer than a viewer is typically up, so it can't be built on demand.
     * It is still only ever read by a viewer, so it goes when they do. */
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
#endif
    /* The per-frame trace is verbose, not debug. It is also the one thing here
     * that is not the viewer's — a frame log stands on its own — so it stays in
     * a build with no LoRaMon in it. Two levels, one discipline:
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
#if CONFIG_STRADDLE_LORAMON
    if (ext && s_monDetail) m.ext = *ext;
#else
    (void)ext;
#endif
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

#if CONFIG_STRADDLE_LORAMON
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
#endif  /* CONFIG_STRADDLE_LORAMON — the hour and the subtree drop */

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
 * would take the detour traffic off the screen with it.
 *
 * **Two conditions, not one.** A regime names channels this node MAY use; it
 * does not say that it will. Only a SUPE detour ever leaves the hailing
 * channel, so with the protocol off no frame can reach an agile lane however
 * high the regime number is set — and drawing the lanes anyway gives a viewer
 * a screen of graphs that are empty by construction and no way to tell that
 * from a quiet band. The published list is what this radio can actually put
 * traffic on. */
void publishChannels(LoraRadio* r) {
    char val[24 * LORA_CH_MAX];
    int  w = snprintf(val, sizeof val, "%u,%u",
                      (unsigned)r->cfgFreqHz, (unsigned)r->cfgBwHz);
#if !defined(CONFIG_LORA_NO_SUPE)
    int n = 0;
    const RegimeChan* ch = r->supeOn ? regimeChans(r->afa, &n) : nullptr;
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

#if CONFIG_STRADDLE_LORAMON
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
#endif  /* CONFIG_STRADDLE_LORAMON — the channel-RSSI series */

static void loraIfTaskMain(void*) {
    info("[%s-if] task up", TAG);
#if CONFIG_STRADDLE_LORAMON
    bool       prevWatch = false;
#endif
    TickType_t lastBeat  = 0;
    TickType_t lastShift = 0;
    TickType_t lastMeas  = 0;
    uint64_t   measSig   = 0;   /* traffic signature at the last publication */
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
            /* The measurement publication has its own 15 s beat, UI or not —
             * its readers are other firmware tasks (lxmf's Ping) as much as
             * the browser — armed only while there is a neighbour to publish
             * AND a frame has moved since the last publication. Traffic that
             * arrives during a long idle block is picked up at this task's next
             * wake, whatever brings it. */
            if (lastMeas == 0) lastMeas = now;
            bool       meas    = measWanted() && measTrafficSig() != measSig;
            TickType_t measDue = lastMeas + pdMS_TO_TICKS(LORA_MEAS_MS);
            if (meas && (int32_t)(measDue - due) < 0) due = measDue;
            int32_t    rem  = (int32_t)(due - now);
            TickType_t wait = rem > 0 ? (TickType_t)rem : 0;

            IfMsg m;
            if (s_ifq && xQueueReceive(s_ifq, &m, wait) == pdTRUE) {
#if CONFIG_STRADDLE_LORAMON
                if (m.radio < kNumRadios) {
                    LoraRadio* r = &s_radios[m.radio];
                    if (m.kind == IFM_MON)       loraMonRecord(r, &m);
                    else if (m.kind == IFM_RSSI) loraPublishRssi(r, &m);
                }
                /* A lull is a flush point: mid-storm the batch cap governs,
                 * and the lone frame of a quiet minute publishes right away. */
                if (uxQueueMessagesWaiting(s_ifq) == 0) monFlushPending();
#endif
            }

#if CONFIG_STRADDLE_LORAMON
            /* A close acts on the wake that carried it, not on the next beat —
             * unwatched, the next beat may be minutes out, and the packets
             * subtree would sit there the whole wait. */
            if (prevWatch && !s_monWatched) {
                monFlushPending();
                for (int i = 0; i < kNumRadios; i++) loraMonClear(&s_radios[i]);
            }
            prevWatch = s_monWatched;
#endif

            /* Checked whether or not a record arrived: a steady stream of them
             * must not be able to starve expiry and the stats flush. */
            now = xTaskGetTickCount();
            if ((int32_t)(now - due) < 0) continue;
            lastBeat = now;
#if CONFIG_STRADDLE_LORAMON
            monFlushPending();      /* nothing pending outlives a beat */
#endif

            /* Age every one-hour running total in the system, ours included.
             * One call covers them all; they linked themselves up at
             * construction. shiftAll ages exactly one bucket, so a wake that
             * arrives late (never by much — the idle block above is set to
             * this very cadence) catches up bucket by bucket. */
            while ((int32_t)(now - lastShift) >= (int32_t)shiftTicks) {
                lastShift += shiftTicks;
                Rolling1h::shiftAll();
            }

#if CONFIG_STRADDLE_LORAMON
            /* Belt for the cached watch flag — the change subscription is the
             * prompt path. A transition it reveals is handled like any other:
             * a close drops the published subtree. */
            bool w = loraMonWatched();
            s_monWatched = w;
            s_monDetail  = loraMonDetailWatched();
            if (prevWatch && !w)
                for (int i = 0; i < kNumRadios; i++) loraMonClear(&s_radios[i]);
            prevWatch = w;
#endif

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

            /* The per-peer measurements, on their own cadence, and only when
             * frames have moved since the last publication. */
            if (meas && (int32_t)(now - measDue) >= 0) {
                lastMeas = now;
                measSig  = measTrafficSig();
                for (int i = 0; i < kNumRadios; i++) publishMeas(&s_radios[i]);
            }

            /* The pill rides this beat rather than the stats gate above: a
             * neighbour appearing or ageing out moves no byte counter, and the
             * enable switch moves none either. rnsdPillSet writes the same
             * values idempotently, so a beat that changes nothing costs the
             * storage actor one deduped op. */
            publishPill();

#if CONFIG_STRADDLE_LORAMON
            /* LoRaMon expiry — 1 Hz while a viewer is open, so nodes age out of
             * the 1 h window even on an idle channel. */
            if (w) {
                uint32_t nowMs = millis();
                for (int i = 0; i < kNumRadios; i++) {
                    loraMonExpire(&s_radios[i], nowMs);
                    loraPublishAirtime(&s_radios[i], nowMs);
                }
            }
#endif
        }

        /* rns stop: the radio task parks too, so nothing more will be queued.
         * Drop whatever is still in flight and park on the same flag. */
        if (s_ifq) xQueueReset(s_ifq);
#if CONFIG_STRADDLE_LORAMON
        s_monWatched = false;
#endif
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
#if CONFIG_STRADDLE_LORAMON
    s_monWatched = loraMonWatched();
    s_monDetail  = loraMonDetailWatched();
#endif
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
#if CONFIG_STRADDLE_LORAMON
        storageSubscribeChanges("sys.stats.web_loramon", onWatchChange);
        storageSubscribeChanges("sys.stats.lcd_loramon", onWatchChange);
        /* The `detailed` toggle on either surface. Its own keys: it is asked
         * for and dropped mid-session, and every record written between the two
         * carries a third more than the rest. */
        storageSubscribeChanges("sys.stats.web_details", onWatchChange);
        storageSubscribeChanges("sys.stats.lcd_details", onWatchChange);
        /* The browser half of the watch is gated on the link, so the link going
         * down closes a web viewer as surely as the viewer closing does. */
        storageSubscribeChanges("webrtc.up",             onWatchChange);
#endif
        /* Not a viewer's key: the stats flush and the pill are held back on a
         * node no UI can reach (uiTelemetryWanted), so the interface task has
         * to be woken when one comes into reach. That is true with or without
         * LoRaMon in the build. */
        storageSubscribeChanges("wifi.sta.up",           onWatchChange);
        storageSubscribeChanges("wifi.ap.up",            onWatchChange);
    }
}

bool loraMonParked(void) { return s_ifParked; }

#if CONFIG_STRADDLE_LORAMON
/* LoRaMon expiry FIFO: allocated once, kept across config cycles. 16 KB per
 * radio, which is the single largest thing the viewer costs a node that never
 * opens one — hence the gate rather than a lazy allocation on first watch. */
void loraMonInit(LoraRadio* r) {
    if (r->mon.pktMs) return;
    r->mon.pktMs   = (uint32_t*)gp_alloc((size_t)LORA_MON_CAP * sizeof(uint32_t));
    r->mon.pktCap  = r->mon.pktMs ? LORA_MON_CAP : 0;
    r->mon.pktHead = r->mon.pktCount = 0;
}
#endif

#endif  /* CONFIG_LORA0_CS_PIN */
