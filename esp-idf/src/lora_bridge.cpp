/**
 * lora_bridge — the three-way segment bridge and the packet paths: rnsd
 * registration, inbound delivery (radio → rnsd/RNode), the transmit machinery
 * (beginTx, frame completion, the TxDone service), the announce buffer and its
 * replay, and the outbound drain with SUPE's classifier gate.
 */
#include "lora_priv.h"

#if defined(CONFIG_LORA0_CS_PIN)

uint8_t modeFromString(const char* s) {
    if (!s || !*s)                      return RNS_IFACE_MODE_GATEWAY;
    if (strcmp(s, "full")         == 0) return RNS_IFACE_MODE_FULL;
    if (strcmp(s, "access_point") == 0) return RNS_IFACE_MODE_ACCESS_POINT;
    if (strcmp(s, "roaming")      == 0) return RNS_IFACE_MODE_ROAMING;
    if (strcmp(s, "boundary")     == 0) return RNS_IFACE_MODE_BOUNDARY;
    return RNS_IFACE_MODE_GATEWAY;
}

static const char* modeName(uint8_t m) {
    switch (m) {
        case RNS_IFACE_MODE_FULL:         return "full";
        case RNS_IFACE_MODE_GATEWAY:      return "gateway";
        case RNS_IFACE_MODE_ACCESS_POINT: return "access_point";
        case RNS_IFACE_MODE_ROAMING:      return "roaming";
        case RNS_IFACE_MODE_BOUNDARY:     return "boundary";
        default:                          return "?";
    }
}


static void onRnsdRecv(int handle, size_t bytesAvail);
static void onRnsdDisconnect(int ref);

static LoraRadio* radioByHandle(int h) {
    for (int i = 0; i < kNumRadios; i++)
        if (s_radios[i].rnsdHandle == h) return &s_radios[i];
    return nullptr;
}

void deregisterFromRnsd(LoraRadio* r) {
    if (r->rnsdHandle >= 0) {
        itsDisconnect(r->rnsdHandle);
        r->rnsdHandle = -1;
    }
}

bool registerWithRnsd(LoraRadio* r) {
    deregisterFromRnsd(r);
    rnsd_iface_t reg = {};
    snprintf(reg.name, sizeof(reg.name), "lora/%d", r->idx);
    reg.mtu     = RNS_MTU;
    reg.bitrate = r->curBitrate;
    reg.mode    = r->curMode;
    reg.in = reg.out = 1;
    reg.fwd = (r->curMode == RNS_IFACE_MODE_FULL || r->curMode == RNS_IFACE_MODE_GATEWAY) ? 1 : 0;
    reg.rpt = 0;
    reg.ifac_size = r->curIfacSize;
    reg.announce_cap = r->curAnnounceCap;
    reg.rx_signal = 1;   /* inbound data frames carry the 4-byte RSSI/SNR prefix */
    reg.retain_announces = r->curRetainAnnounces;
    reg.policy_manual = r->curPolicyManual;
    reg.route_for     = r->curRouteFor;
    safeStrncpy(reg.ifac_netname, r->curIfacNetname, sizeof(reg.ifac_netname));
    safeStrncpy(reg.ifac_netkey,  r->curIfacNetkey,  sizeof(reg.ifac_netkey));
    /* ref = radio index — onRnsdDisconnect uses it to find the radio. */
    r->rnsdHandle = itsConnect("rnsd", RNSD_PORT_IFACE, &reg, sizeof(reg),
                               pdMS_TO_TICKS(500), r->idx,
                               onRnsdRecv, onRnsdDisconnect);
    if (r->rnsdHandle < 0) {
        warn("lora/%d rnsd register failed", r->idx);
        return false;
    }
    info("registered as iface lora/%d (mtu=%u bitrate=%u mode=%s)",
         r->idx, (unsigned)RNS_MTU, (unsigned)r->curBitrate, modeName(r->curMode));
    return true;
}

static void onRnsdDisconnect(int ref) {
    if (ref >= 0 && ref < kNumRadios) s_radios[ref].rnsdHandle = -1;
    /* The task loop will re-register if the radio is still enabled. */
}

/* ─────────────── inbound (radio → rnsd) ─────────────── */

/* Hand one packet to rnsd, prefixed with its signal telemetry (rnsd strips the
 * prefix and sets it on the received packet): int16 rssi(dBm) | int16 snr(dB*10),
 * big-endian. Shared by the radio's receive path, which passes the readings
 * captured in the same synchronous RX call, and by the RNode bridge, which
 * passes a synthetic pair because its packets never crossed the air.
 *
 * Zero-copy: the block is allocated here with the prefix in front and handed
 * to rnsd whole (itsSendOwned). On backpressure the call returns 0 and we
 * still own the block — that is the drop point, not a retry loop. */
static void rnsdInject(LoraRadio* r, const uint8_t* data, size_t len,
                       int16_t rssi, int16_t snr10) {
    if (r->rnsdHandle < 0) return;
    if (len > RNS_MTU + 16) len = RNS_MTU + 16;     /* defensive clamp */
    uint8_t* f = (uint8_t*)malloc(4 + len);
    if (!f) return;
    f[0] = (uint8_t)(rssi  >> 8); f[1] = (uint8_t)rssi;
    f[2] = (uint8_t)(snr10 >> 8); f[3] = (uint8_t)snr10;
    memcpy(f + 4, data, len);
    /* Zero timeout: this runs on the radio task, in the receive path, between
     * train packets — blocking here is the receiver going deaf. Backpressure
     * is the drop point, not a retry loop; the layers above own recovery. */
    if (itsSendOwned(r->rnsdHandle, f, 4 + len, 0) == 0) {
        free(f);
        warn("lora/%d rnsd ITS send dropped (%u B)", r->idx, (unsigned)len);
    }
}

static void deliverInbound(LoraRadio* r, const uint8_t* data, size_t len,
                           double airMs, int frames) {
    (void)airMs; (void)frames;   /* per-frame recording/logging now happens in handleRxDone */
    /* Passive neighbour tap — before the rnsd gate, so the table fills even
     * while the interface is unregistered. rssiLast/snrLast were captured in
     * the same synchronous RX call, so they belong to exactly this packet. */
    auto rnd = [](float x) { return (int16_t)(x < 0 ? x - 0.5f : x + 0.5f); };
    int16_t rssi = rnd(r->rssiLast);
    int16_t snr  = rnd(r->snrLast * 10.0f);
    /* Inside a transaction the sender is not a guess: the GRANT went to one
     * node and this arrived under it, so the tap can attribute what the packet
     * establishes — a link identifier above all — to that peer instead of
     * waiting to overhear the association in the clear. */
    peersObserve(r, data, len, false, rssi, snr, LORA_ORIG_RNSD, supeCargoPeer(r));
    /* A whole packet, which is what a train is counted in: a split packet
     * counts once, when both halves are in, which is exactly here. */
    supeOnPacketRx(r, rssi, snr);
    /* One segment, two other endpoints. Only reassembled packets that are not
     * our own air protocol reach here, so the client sees exactly the Reticulum
     * traffic — with the signal this radio measured for it. */
    rnodeForwardData(r, data, len, /*withStats=*/true);
    rnsdInject(r, data, len, rssi, snr);
}

/* Re-arm continuous RX and re-enable the level-triggered DIO1 (the trampoline
 * disables it on each fire; a completed readData()/finishTransmit() has cleared
 * the chip IRQ so the line has dropped low and the next edge fires again).
 * Shared by the RX drain and the post-TX return to listening. */
void rearmRx(LoraRadio* r) {
    /* A transmit fired meanwhile (the radio-check slot timer runs off-task) — a
     * startReceive now would abort it. TxDone re-arms RX when it completes. */
    if (r->txActive) {
        gpio_intr_enable((gpio_num_t)r->slot->dio1);
        return;
    }
    /* Back to plain listening with nothing chained behind it: the reference may
     * stop again. A transaction still in flight is not idle — its next frame is
     * as close behind as a split's second half — so it keeps the hold. Before
     * the receiver is armed, since arming takes the standby this governs. */
    if (!supeBusy(r)) radioHoldOsc(r, false);
    radioStartRx(r);
    /* A fresh receiver has no reception in progress; the evidence radioRxInProgress
     * tracks was cleared with the chip's flags. */
    r->rxActiveStart = 0;
    r->rxHeaderSeen  = false;
    gpio_intr_enable((gpio_num_t)r->slot->dio1);
}

/* Drain a completed reception. serviceRadio has already confirmed RX_DONE from
 * the chip's IRQ flags, so go straight to reading the packet. */
static void handleRxDone(LoraRadio* r) {
    /* End-of-air timestamp for the LoRaMon record, captured before any of the
     * processing below: the SUPE dispatch can transmit and retune, and a `now`
     * taken after it charges that work to the frame's position on the air. */
    uint32_t rxEndMs = millis();
    size_t pktLen = r->radio->getPacketLength();
    if (pktLen == 0 || pktLen > 1 + RNODE_MAX_PAYLOAD) {
        rearmRx(r);
        return;
    }
    /* Time on air of what just landed, from the framing it actually flew with.
     * Computed before the CRC verdict because channel occupancy does not depend
     * on the frame decoding — a corrupt frame held the medium exactly as long. */
    uint32_t airMs = (uint32_t)lround(1000.0 * loraAirtimeSeconds(
                         r->airSf, r->airBwHz, r->cfgCr, r->airPreamble,
                         (int)pktLen, r->airImplicit));

    uint8_t frame[1 + RNODE_MAX_PAYLOAD];
    int16_t st = r->radio->readData(frame, pktLen);
    if (st != RADIOLIB_ERR_NONE) {
        if (st == RADIOLIB_ERR_CRC_MISMATCH) {
            r->crcErr++;
            csmaMediumHeld(r, airMs);
            /* On the record: a corrupt frame is a real event on the air, and
             * the graph must show the hole in a train where it died. */
            loraMonPush(r, 0 /*rx*/, (airMs <= rxEndMs ? rxEndMs - airMs : rxEndMs),
                        (uint16_t)airMs, (uint16_t)pktLen,
                        (int16_t)lround(r->radio->getRSSI()),
                        (int16_t)lround(r->radio->getSNR() * 10.0), 0,
                        LORA_PKT_BAD, 0, 0);
            if (logIsVerbose(TAG))        /* a frame, so it belongs at verbose */
                verb("lora/%d rx CRC-FAIL %uB rssi=%.0f snr=%.1f",
                     r->idx, (unsigned)pktLen,
                     (double)r->radio->getRSSI(), (double)r->radio->getSNR());
        }
        rearmRx(r);
        return;
    }
    csmaMediumHeld(r, airMs);
    r->rxFrames++;
    r->rssiLast = r->radio->getRSSI();
    r->snrLast  = r->radio->getSNR();

    /* The frame is out of the chip — put the receiver back on the air before
     * any of the processing below. In a train the next preamble can start
     * under a millisecond after this RX_DONE at the fastest budget, and the
     * dispatch, records, fan-out and inject below all fit inside its flight.
     * Anything downstream that transmits or retunes overrides this arm. */
    rearmRx(r);

    uint8_t  header     = frame[0];
    uint8_t  seq        = header & 0xF0;
    bool     isSplit    = (header & RNODE_FLAG_SPLIT) != 0;
    size_t   payloadLen = pktLen - 1;

    /* Our own air protocol is consumed
     * here — it never enters split framing or rnsd. Classified before the
     * LoRaMon record so the record can carry its protocol colour. */
    /* Radio check, before the probe tap: while we are listening to somebody's
     * sweep the modem is on the 0x23 implicit regime, so a 4-byte frame here is
     * it never enters split framing or rnsd. */
    bool ours = false;
    if (pktLen == PWRREQ_LEN && frame[0] == LORA_MAGIC_PWRREQ) {
        /* A power request binds to the frame it prefixes by adjacency alone, so
         * it is parked here and spent by the very next RNS frame either way. */
        r->apRxSuggest     = (int8_t)frame[1];
        r->apRxSuggestPend = (int8_t)frame[1] != PWRREQ_NO_TXP;
        ours = true;
    }

    /* SUPE's receive dispatch, and the whole of it (plans/SUPE.md §0.1). On this
     * interface the first on-air byte is the split header — a random 4-bit
     * sequence in the high nibble and a split flag in bit 0 — which reaches
     * every byte whose low nibble is 0 or 1 and nothing else. So byte 0 sorts
     * into exactly three cases:
     *
     *   low nibble 0 or 1                      → a split header: a Reticulum packet
     *   0xC2–0xDF, low nibble neither          → a SUPE frame
     *   anything else                          → neither, and discarded
     *
     * The third branch is stated on purpose: discard is the designed response to
     * everything unrecognised, and dispatch is where that discipline starts.
     * It is also a one-line assumption sitting between this file and the
     * protocol — if the framing above ever changes, check the rule still holds,
     * because it is the receive path's only basis for telling the two apart. */
    /* The SUPE dispatch below may retune the radio (a GRANT starts the move
     * to the detour); the record of THIS frame belongs to the channel it
     * arrived on, so that is captured first. */
    uint8_t rxCh = r->chNow;
    bool foreign = false;
    if (!ours && !supeIsFramingByte(header)) {
        if (supeIsTypeByte(header)) {
            supeOnFrame(r, frame, pktLen, (int16_t)lround(r->rssiLast),
                        (int16_t)lround(r->snrLast * 10.0));
            ours = true;
        } else {
            foreign = true;
        }
    }

    /* Record this on-air frame (RX_DONE marks end-of-air, so start = end − ToA),
     * against the channel it arrived on — chNow may already be the detour's. */
    {
        uint32_t now = rxEndMs;
        uint32_t dur = airMs;
        uint8_t  chLive = r->chNow;
        r->chNow = rxCh;
        loraMonPush(r, 0 /*rx*/, (dur <= now ? now - dur : now), (uint16_t)dur,
                    (uint16_t)payloadLen, (int16_t)lround(r->rssiLast),
                    (int16_t)lround(r->snrLast * 10.0), 0,
                    ours ? LORA_PKT_OURS : LORA_PKT_RNS,
                    0, 0 /*rx never waits*/);
        r->chNow = chLive;
    }

    if (ours || foreign) {
        if (foreign && logIsVerbose(TAG))
            verb("lora/%d rx discarded: byte 0 0x%02x is neither framing nor SUPE",
                 r->idx, header);
        return;
    }

    if (!isSplit) {
        size_t fl = pktLen;                        /* whole on-air frame (incl. header) */
        deliverInbound(r, frame + 1, payloadLen, loraPacketAirtimeMs(r, &fl, 1), 1);
        r->rxBytes += payloadLen;
    } else if (!r->splitPending) {
        std::memcpy(r->splitBuf, frame + 1, payloadLen);
        r->splitLen      = payloadLen;
        r->splitSeq      = seq;
        r->splitPending  = true;
        r->splitDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(SPLIT_RX_TIMEOUT_MS);
    } else if (r->splitSeq == seq) {
        if (r->splitLen + payloadLen <= sizeof(r->splitBuf)) {
            /* Two on-air frames, each with its own preamble/header/CRC. */
            size_t fl[2] = { r->splitLen + 1, payloadLen + 1 };
            std::memcpy(r->splitBuf + r->splitLen, frame + 1, payloadLen);
            r->splitLen += payloadLen;
            deliverInbound(r, r->splitBuf, r->splitLen,
                           loraPacketAirtimeMs(r, fl, 2), 2);
            r->rxBytes += r->splitLen;
        }
        r->splitPending = false;
    } else {
        /* Different sender's split — restart assembly on the new seq. */
        std::memcpy(r->splitBuf, frame + 1, payloadLen);
        r->splitLen      = payloadLen;
        r->splitSeq      = seq;
        r->splitDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(SPLIT_RX_TIMEOUT_MS);
    }

    /* A pending power request binds to the one RNS frame that follows it and to
     * nothing else — whether that frame claimed it or not, it is spent here. The
     * `ours` path above returns early on purpose: that is how the request
     * survives from its own frame to the one it prefixes. */
    r->apRxSuggestPend = false;
}

/* ─────────────── outbound (rnsd → radio) ─────────────── */

/* Return the radio to continuous RX after a transmit finishes or aborts. */
/* Back to listening after a packet — completed, abandoned by the watchdog, or
 * never started. If the packet came from the RNode client, this is where its
 * frame is finished with the radio, so its queue is released here: CMD_READY is
 * harmless with flow control off and mandatory with it on. */
static void txRearmRx(LoraRadio* r) {
    r->txActive = false;
    if (r->txFromRnode) { r->txFromRnode = false; rnodeSendReady(); }
    /* Mid-train, the next packet follows this one back to back with nothing
     * between them — which is what a train is, and why it chains from here
     * rather than from the outbound drain. When the count or the announced
     * duration runs out the sender returns, which is one of the two ways a
     * train ends; the peer's is the count or the stated length. */
    if (r->supe) {
        supeLock(r);
        bool handled = supeAfterTx(r);
        supeUnlock(r);
        if (handled) return;          /* it re-armed, or fired the next frame */
    }
    rearmRx(r);
}

/* Fire frame `idx` of the current outbound packet. startTransmit() writes the
 * FIFO and issues SetTx, then returns — the chip modulates on its own and raises
 * TxDone on DIO1 when done (serviceRadio handles it). Non-blocking: the task is
 * free for the whole airtime, so nothing on its core is starved at high SF. */
void startTxFrame(LoraRadio* r, int idx) {
    /* Is another frame already spoken for behind this one — the second half of a
     * split, or any frame inside a live transaction? Then the oscillator must
     * not be allowed to stop when this frame ends, because restarting it is
     * several ms of dead air in a gap nothing else may use. Decided here, before
     * the frame flies, because the fallback it sets is what the chip acts on the
     * moment TxDone arrives. */
    radioHoldOsc(r, idx + 1 < (int)r->txFrameCount || supeBusy(r));
    int16_t st = r->radio->startTransmit(r->txFrame[idx], r->txFrameLen[idx]);
    if (st != RADIOLIB_ERR_NONE) {
        warn("lora/%d startTransmit %u B failed: %s (%d)",
             r->idx, (unsigned)r->txFrameLen[idx], rlErrName(st), (int)st);
        txRearmRx(r);
        return;
    }
    r->txActive       = true;
    r->txFrameStartMs = millis();                  /* start-of-air, for the LoRaMon record */
    r->txDeadline     = xTaskGetTickCount() + r->txWatchTicks;
    gpio_intr_enable((gpio_num_t)r->slot->dio1);   /* arm DIO1 for this frame's TxDone */
}

/* ─────────────── announce buffer ───────────────
 * (overview at ANN_MAX_ENTRIES, near the top of the file) */

/* Drop entries past their hour. Radio task only. */
static void annExpire(LoraRadio* r, uint32_t now) {
    if (!r->ann) return;
    for (int i = 0; i < ANN_MAX_ENTRIES; i++) {
        AnnRec* e = &r->ann->e[i];
        if (e->used && (uint32_t)(now - e->ms) > ANN_TTL_MS) e->used = false;
    }
}

int annCount(const LoraRadio* r) {
    if (!r->ann) return 0;
    int n = 0;
    for (int i = 0; i < ANN_MAX_ENTRIES; i++) if (r->ann->e[i].used) n++;
    return n;
}

/* Offer an outgoing packet to the buffer. Stores it only if it is an announce
 * this node ORIGINATED — wire hops 0, which is what distinguishes our own
 * announcement from one we are relaying for the network. Relayed announces
 * belong to somebody else and replaying them would be speaking for them.
 *
 * Keyed by destination hash: a fresh announce for a destination we already hold
 * replaces it in place rather than adding an entry, so a busy destination
 * cannot crowd the buffer. At the cap, the oldest entry is evicted — with a
 * one-hour TTL that only bites on a node with more than sixteen live
 * destinations, where something has to give regardless.
 *
 * **Returns true when it has SWALLOWED the packet**, which is the normal case
 * for an announce we originate. This interface announces at its own pace and
 * nobody else's: rnsd and the RNode client hand announces over whenever their
 * own logic fires, and we sit on them until the beat comes round or `lora a`
 * says so. Nothing else is intercepted — a relayed announce is somebody else's
 * traffic and goes straight out.
 *
 * **Nothing is held back.** The announce goes on the air when rnsd hands it
 * over; this only keeps a copy so `lora a` can repeat it later. Buffering and
 * swallowing an announce delayed somebody else's decision by up to an announce
 * interval, which is exactly what plans/SUPE.md §9 says not to do. */
static void annRecordTx(LoraRadio* r, const uint8_t* pkt, size_t len) {
    if (!r->ann || len == 0 || len > ANN_MAX_LEN) return;
    RnsHdr h;
    if (!rnsParse(pkt, len, &h)) return;
    if (h.ptype != NEI_PT_ANNOUNCE || h.hops != 0 || h.hdr2 || !h.dest) return;

    uint32_t now = millis();
    annExpire(r, now);

    AnnRec* slot = nullptr;
    for (int i = 0; i < ANN_MAX_ENTRIES; i++) {          /* same destination → replace */
        AnnRec* e = &r->ann->e[i];
        if (e->used && std::memcmp(e->dest, h.dest, 16) == 0) { slot = e; break; }
    }
    if (!slot) for (int i = 0; i < ANN_MAX_ENTRIES; i++)  /* else a free slot */
        if (!r->ann->e[i].used) { slot = &r->ann->e[i]; break; }
    if (!slot) {                                          /* else evict the oldest */
        slot = &r->ann->e[0];
        for (int i = 1; i < ANN_MAX_ENTRIES; i++)
            if ((int32_t)(r->ann->e[i].ms - slot->ms) < 0) slot = &r->ann->e[i];
    }

    std::memcpy(slot->dest, h.dest, 16);
    std::memcpy(slot->data, pkt, len);
    slot->len  = (uint16_t)len;
    slot->ms   = now;
    slot->used = true;
}

/* Begin transmitting one RNS packet from `origin` (LORA_ORIG_*).
 * >RNODE_MAX_PAYLOAD splits into two frames sharing a seq nibble; the second is
 * fired from serviceRadio once the first completes. Returns immediately — the
 * airtime runs on the chip, not the task.
 *
 * This is also the fan-out point of the three-way bridge, and deliberately so:
 * "presented to the radio" means transmitted, so a packet the LBT valve drops
 * never aired and is bridged nowhere. A packet is bridged if and only if it
 * went on air. */
void beginTx(LoraRadio* r, const uint8_t* data, size_t len, uint8_t origin,
                    bool fromBuffer, const int8_t* forcePwr) {
    if (!r->running || len == 0 || len > RNS_MTU) return;

    /* Keep a copy of every announce we originate so `lora a` can repeat it.
     * Not when this call IS that repeat, or the buffer would refresh its own
     * timestamps and never age out. */
    if (!fromBuffer) annRecordTx(r, data, len);

    peersObserve(r, data, len, true, 0, 0, origin, LORAQ_PEER_NONE);   /* passive neighbour tap (tx side) */

    r->txSeq          = (uint8_t)((esp_random() & 0x0F) << 4);   /* 4-bit seq, upper nibble */
    r->txPayloadBytes = len;
    r->txFrameSent    = 0;

    /* A power request rides in front of the packet it relates to, in the same
     * channel access, and binds to it by adjacency alone. Only link openers
     * carry one so far; an LR never splits (83 B, far under RNODE_MAX_PAYLOAD),
     * so the pair fits the existing two-frame burst. */
    int      base = 0;
    int8_t   suggest;
    /* Not on a detour: the manifest already stated the packet count, and a
     * prefix frame would put a frame on the unicast channel that the count does
     * not describe. The power for a detour comes from `forcePwr` regardless. */
    if (!forcePwr && apPwrReqFor(r, data, len, &suggest)) {
        r->txFrame[0][0] = LORA_MAGIC_PWRREQ;
        r->txFrame[0][1] = (uint8_t)suggest;
        r->txFrame[0][2] = 0;               /* no rssi report: we know this peer, */
        r->txFrame[0][3] = 0;               /* so the suggestion is the useful half */
        r->txFrameLen[0] = PWRREQ_LEN;
        r->txType[0]     = LORA_PKT_OURS;
        base = 1;
    }

    uint8_t pktType = (origin == LORA_ORIG_RNODE) ? LORA_PKT_RNODE : LORA_PKT_RNS;
    if (len <= RNODE_MAX_PAYLOAD) {
        r->txFrame[base][0] = r->txSeq;
        std::memcpy(r->txFrame[base] + 1, data, len);
        r->txFrameLen[base] = 1 + len;
        r->txType[base]     = pktType;
        r->txFrameCount     = (uint8_t)(base + 1);
    } else {
        size_t first  = RNODE_MAX_PAYLOAD;
        size_t second = len - first;
        r->txFrame[0][0] = r->txSeq | RNODE_FLAG_SPLIT;
        std::memcpy(r->txFrame[0] + 1, data, first);
        r->txFrameLen[0] = 1 + first;
        r->txType[0]     = pktType;
        r->txFrame[1][0] = r->txSeq | RNODE_FLAG_SPLIT;
        std::memcpy(r->txFrame[1] + 1, data + first, second);
        r->txFrameLen[1] = 1 + second;
        r->txType[1]     = pktType;
        r->txFrameCount  = 2;
    }
    r->txFromRnode = (origin == LORA_ORIG_RNODE);

    /* Segment fan-out: whichever of the other two endpoints exists gets this
     * packet, because it is about to be on air and all three share the
     * channel. Our own transmissions carry no measured signal, so the client's
     * copy goes without stat frames. */
    if (origin == LORA_ORIG_RNODE) rnsdInject(r, data, len, RNODE_INJ_RSSI, RNODE_INJ_SNR10);
    else                           rnodeForwardData(r, data, len, /*withStats=*/false);

    /* Per-frame LoRaMon records + the `log lora debug` line are emitted at each
     * frame's TxDone (serviceRadio); the RNS-header trace (loraTracePacket) is
     * kept but no longer called. The request and the packet must go out at the
     * same power — the peer measures the pair as one path sample. */
    if (forcePwr) apApplyPower(r, *forcePwr);
    else          apApplyPower(r, apTxPower(r, data, len));
    startTxFrame(r, 0);
}

/* ─────────────── the train's staging pair ───────────────
 *
 * A train must be back-to-back: when TxDone lands, the next packet may cost
 * nothing but a buffer copy and a startTransmit. So everything expensive —
 * the split into frames, the observer tap, the segment fan-out — happens at
 * STAGE time, during the previous packet's airtime, and FIRE is microseconds.
 * The one asymmetry against beginTx: a staged packet is fanned out to the
 * other endpoints when staged, so a transaction that dies between stage and
 * fire has bridged a packet that never aired — the same loss profile as air,
 * and the layers above retry. */
bool stageTx(LoraRadio* r, const uint8_t* data, size_t len, uint8_t origin,
             int8_t pwr) {
    if (!r->running || len == 0 || len > RNS_MTU) return false;

    peersObserve(r, data, len, true, 0, 0, origin, LORAQ_PEER_NONE);   /* passive tap (tx side) */

    uint8_t seq = (uint8_t)((esp_random() & 0x0F) << 4);
    uint8_t pktType = (origin == LORA_ORIG_RNODE) ? LORA_PKT_RNODE : LORA_PKT_RNS;
    if (len <= RNODE_MAX_PAYLOAD) {
        r->txStage[0][0] = seq;
        std::memcpy(r->txStage[0] + 1, data, len);
        r->txStageLen[0]  = 1 + len;
        r->txStageType[0] = pktType;
        r->txStageCount   = 1;
    } else {
        size_t first  = RNODE_MAX_PAYLOAD;
        size_t second = len - first;
        r->txStage[0][0] = seq | RNODE_FLAG_SPLIT;
        std::memcpy(r->txStage[0] + 1, data, first);
        r->txStageLen[0]  = 1 + first;
        r->txStageType[0] = pktType;
        r->txStage[1][0] = seq | RNODE_FLAG_SPLIT;
        std::memcpy(r->txStage[1] + 1, data + first, second);
        r->txStageLen[1]  = 1 + second;
        r->txStageType[1] = pktType;
        r->txStageCount   = 2;
    }
    r->txStageBytes     = len;
    r->txStagePwr       = pwr;
    r->txStageFromRnode = (origin == LORA_ORIG_RNODE);

    /* Segment fan-out at stage time — the airtime this rides in. */
    if (origin == LORA_ORIG_RNODE) rnsdInject(r, data, len, RNODE_INJ_RSSI, RNODE_INJ_SNR10);
    else                           rnodeForwardData(r, data, len, /*withStats=*/false);
    return true;
}

/* Fire the staged packet: a buffer copy and a startTransmit, nothing else. */
bool fireStagedTx(LoraRadio* r) {
    if (!r->txStageCount || !r->running) return false;
    for (uint8_t i = 0; i < r->txStageCount; i++) {
        std::memcpy(r->txFrame[i], r->txStage[i], r->txStageLen[i]);
        r->txFrameLen[i] = r->txStageLen[i];
        r->txType[i]     = r->txStageType[i];
    }
    r->txFrameCount   = r->txStageCount;
    r->txFrameSent    = 0;
    r->txPayloadBytes = r->txStageBytes;
    r->txFromRnode    = r->txStageFromRnode;
    r->txWaitMs       = 0;
    r->txOwnMs        = 0;
    r->txWaitPend     = false;
    r->txStageCount   = 0;
    apApplyPower(r, r->txStagePwr);
    startTxFrame(r, 0);
    return r->txActive;
}

/* ─────────────── announce replay ───────────────
 *
 * There is no power sweep and no check behind it: SUPE
 * measures the same quantity continuously and for free, since every frame a
 * detour sends states the power it went out at (plans/SUPE.md §7). What is
 * left is the on-demand replay `lora [<n>] a[nnounce]` asks for — every
 * announce this node has originated, one at a time through the queue and
 * ordinary channel access, followed by SUPE's own ANNOUNCE2.
 *
 * Announces are no longer bunched and no longer swallowed. They go out when
 * rnsd hands them over, which is what SUPE.md §9 requires and what the buffer
 * used to break; the copy kept here exists only so this command has something
 * to repeat. */

/* Kick a replay off (`lora a`). Returns the number of buffered announces it
 * will feed into the queue, 0 if there is nothing to repeat. */
int annReplayStart(LoraRadio* r) {
    int n = annCount(r);
    if (n == 0) return 0;
    r->annReplay = true;
    r->annIdx    = 0;
    /* A beat already waiting for the channel would otherwise duplicate the
     * announcement the replay ends with. The replay re-arms it at the end. */
    supeAnnCancel(r);
    return n;
}

/* Top the queue up with the next buffered announce. One per pass, so a replay
 * cannot crowd live traffic out of the queue; each rides the queue and pays
 * ordinary channel access like any other packet. */
static void annReplayFill(LoraRadio* r) {
    if (!r->annReplay || !r->ann) return;
    if (!loraqAccepting(&r->q)) return;
    annExpire(r, millis());
    while (r->annIdx < ANN_MAX_ENTRIES && !r->ann->e[r->annIdx].used) r->annIdx++;
    if (r->annIdx >= ANN_MAX_ENTRIES) {
        /* Our own announcement closes the run: the identities just replayed and
         * the radio that carries them are one statement about this node. It
         * leaves the radio from supePoll rather than the queue, so it is armed
         * once the last replayed announce has actually left — arming it while
         * any are still queued would put it on the air first. */
        for (uint8_t i = 0; i < loraqDepth(&r->q); i++) {
            LoraPkt* p = loraqAt(&r->q, i);
            if (p && (p->flags & LORAQ_F_REPLAY)) return;
        }
        r->annReplay = false;
        supeAnnArm(r);
        return;
    }
    AnnRec* e = &r->ann->e[r->annIdx++];
    /* Replay the stored bytes verbatim — the announce is signed over its own
     * contents, so it can be neither regenerated nor edited here. The REPLAY
     * flag keeps beginTx from re-recording it (the buffer would otherwise
     * refresh its own timestamps and never age out). */
    uint8_t* b = (uint8_t*)malloc(e->len);
    if (!b) { r->annReplay = false; return; }
    memcpy(b, e->data, e->len);
    if (!loraqPush(&r->q, b, e->len, millis(), LORAQ_PEER_NONE, nullptr, 1,
                   LORAQ_ORIG_RNSD | LORAQ_F_REPLAY)) {
        free(b);
        r->annIdx--;              /* no room this pass; try again next */
    }
}

/* ─────────────── ingress: everything inbound is enqueued ───────────────
 *
 * The queue is the only seam between the bridge and the radio: what arrives
 * here is enqueued, and the engine (or the plain drain) dequeues. Two sources
 * feed it — rnsd's packet link, zero-copy via itsRecvRef, and the RNode
 * client's parked packet, copied because its leg is a byte stream. Both
 * alternate, so neither endpoint starves the other.
 *
 * The routing rule, stated once and applied here: a packet crosses only
 * within its radio's group — never between rnsd interfaces, never between
 * hailing channels. That is rnsd's job. With one radio per group, refs is 1.
 *
 * Global backpressure is loraqAccepting: once the queue is full we simply
 * stop consuming, the packet link backs up, and rnsd's send toward us blocks
 * briefly and then drops with a warning — which Reticulum tolerates. */
static_assert(LORAQ_ORIG_RNSD == LORA_ORIG_RNSD &&
              LORAQ_ORIG_RNODE == LORA_ORIG_RNODE,
              "queue origin bits are handed to beginTx as LORA_ORIG_*");

void queueFill(LoraRadio* r) {
    while (loraqAccepting(&r->q)) {
        size_t rnsdAvail = (r->running && r->rnsdHandle >= 0)
                               ? itsBytesAvailable(r->rnsdHandle) : 0;
        bool rnodeAvail = r->running && s_rnode.handle >= 0 &&
                          s_rnode.radio == r->idx && s_rnode.txLen > 0;
        if (!rnsdAvail && !rnodeAvail) return;
        bool takeRnode = rnodeAvail && (rnsdAvail == 0 || s_rnode.txAlternate);
        if (rnodeAvail && rnsdAvail) s_rnode.txAlternate = !s_rnode.txAlternate;

        uint8_t* b;
        uint16_t len;
        uint8_t  flags;
        if (takeRnode) {
            b = (uint8_t*)malloc(s_rnode.txLen);
            if (!b) return;
            len   = (uint16_t)s_rnode.txLen;
            flags = LORAQ_ORIG_RNODE;
            memcpy(b, s_rnode.txPkt, len);
            /* Cleared now, so the pump can decode the client's next packet
             * while this one waits its turn in the queue. */
            s_rnode.txLen = 0;
        } else {
            void*  blk = nullptr;
            size_t n   = 0;
            if (!itsRecvRef(r->rnsdHandle, &blk, &n, 0)) return;
            if (n == 0 || n > RNS_MTU) { free(blk); continue; }
            b     = (uint8_t*)blk;
            len   = (uint16_t)n;
            flags = LORAQ_ORIG_RNSD;
        }
        /* The peer the first RF hop goes to, for the per-peer cap, and the
         * packet's tag — the first three bytes of its first address field —
         * for the engine's classifier. Both are the observer's reading; the
         * engine itself never parses Reticulum. */
        uint16_t peer = LORAQ_PEER_NONE;
        const uint8_t* nh = apNextHop4(r, b, len);
        if (nh && r->nei) {
            Neighbor* e = peersFindBy4(r->nei, nh);
            if (e && !peersIsLocal(e)) peer = peersIdOf(r->nei, e);
        }
        const uint8_t* tag3 = nullptr;
        RnsHdr h;
        if (rnsParse(b, len, &h)) {
            if (h.ptype != NEI_PT_ANNOUNCE &&
                (h.dtype == NEI_DT_SINGLE || h.dtype == NEI_DT_LINK))
                tag3 = h.hdr2 ? h.transportId : h.dest;
        }
        if (!loraqPush(&r->q, b, len, millis(), peer, tag3, /*refs=*/1, flags)) {
            free(b);              /* full between the check and the push */
            return;
        }
    }
}

/* Transmit (or discard) the head of the queue. */
static void queueSendHead(LoraRadio* r) {
    LoraPkt* p = loraqAt(&r->q, 0);
    if (!p) return;
    beginTx(r, p->bytes, p->len, p->flags & LORAQ_ORIG_MASK,
            /*fromBuffer=*/(p->flags & LORAQ_F_REPLAY) != 0);
    loraqConsume(&r->q, 0);
}

/* Drop the head without transmitting it. Distinct from the send path because
 * a packet the RNode client handed us releases its queue at transmit-done,
 * and one that never airs would otherwise wedge a client running flow
 * control — the same release the LBT valve does when it sheds a frame. */
void queueDiscardHead(LoraRadio* r) {
    LoraPkt* p = loraqAt(&r->q, 0);
    if (!p) return;
    bool wasRnode = (p->flags & LORAQ_ORIG_MASK) == LORAQ_ORIG_RNODE;
    loraqConsume(&r->q, 0);
    if (wasRnode && s_rnode.handle >= 0 && s_rnode.radio == r->idx)
        rnodeSendReady();
}

/* Drain one pending outbound packet for this radio if it's free.
 * Half-duplex: while a split RX is being reassembled OR a transmit is already
 * on-air (txActive) the queue and the ITS buffers simply hold what they hold,
 * and we revisit once the radio is idle. */
void drainOneOutbound(LoraRadio* r) {
    /* Run the wait clock before any of the blocking returns below, so it
     * counts time lost to anything owning the radio and not just to channel
     * contention. */
    bool srcAvail = (r->running && r->rnsdHandle >= 0 &&
                     itsBytesAvailable(r->rnsdHandle) > 0) ||
                    (r->running && s_rnode.handle >= 0 &&
                     s_rnode.radio == r->idx && s_rnode.txLen > 0);
    bool any = loraqDepth(&r->q) > 0 || srcAvail;
    if (!any)                  r->txWaitPend = false;
    else if (!r->txWaitPend) { r->txWaitPend = true; r->txWaitStartMs = millis(); }

    if (r->mtxPhase != MTXP_OFF) return;
    /* A SUPE transaction owns the radio — its frames sit inside a schedule the
     * other end is timing against — and it owns the queue's head with it. This
     * is the one gate left on the drain, and it is radio ownership rather than
     * one module consulting another's bookkeeping.
     *
     * Ingress is the exception, and the reason is the protocol rather than
     * tidiness. What a transaction carries is declared before it runs — the
     * requester's load in the START, the whole duration everyone else holds for
     * in the GRANT — so a packet arriving after that cannot join it and must not
     * disturb the queue the engine is walking. Until then it can: a packet
     * pulled in during the polite wait is still in the queue when `headRun`
     * measures the load, and rides the very detour that is being waited for.
     * That wait is hundreds of milliseconds, so it is where most of the chances
     * to coalesce live. */
    if (!r->running || r->splitPending) return;
    if (supeXactLive(r)) return;
    queueFill(r);
    annReplayFill(r);
    if (supeHoldsRadio(r) || r->txActive) return;

    if (loraqDepth(&r->q) == 0) {
        csmaResetAccess(r);         /* nothing queued → reset channel-access state */
        return;
    }

    /* SUPE's classifier runs on the head of the queue, before channel access:
     * hold and absence are decided before the channel is asked for, because a
     * packet that must not go out this pass has no business contending for
     * the medium. With SUPE off it answers PLAIN and the head simply flies. */
    uint8_t sv = supeHeadVerdict(r);
    if (sv == SUPE_V_HOLD) return;      /* the line behind it waits too */
    /* Our own wait, not a reservation: nobody has claimed the medium, so serve
     * the channel access underneath it and have it ready when the wait ends. */
    if (sv == SUPE_V_WAIT) { csmaPrime(r); return; }
    if (sv == SUPE_V_DROP) { queueDiscardHead(r); return; }
    /* An offer takes the channel on its own terms — after the pre-offer
     * delay, from supePoll — so it stands down here rather than winning the
     * medium now and holding it through a delay. If the transaction will
     * not set up, the packet simply flies. */
    if (sv == SUPE_V_OFFER) return;   /* the glue launches it after the jitter */

    if (!csmaClear(r)) {            /* listen-before-talk not yet satisfied */
        TickType_t waited = xTaskGetTickCount() - r->csmaStart;
        /* Radio contention is otherwise invisible until the drop valve fires —
         * name it explicitly once per frame so a "nothing went out" hunt can
         * rule the channel in or out at a glance. */
        if (!r->csmaStalled && r->csmaPhase != CSMA_IDLE &&
            waited >= pdMS_TO_TICKS(1000)) {
            r->csmaStalled = true;
            if (r->appc)
                warn("lora/%d LBT: tx stalled %u ms by channel contention "
                     "(phase=%s cw=%d/%d slots band=%u airtime=%d%% noise=%.0f dBm)",
                     r->idx, (unsigned)(waited * portTICK_PERIOD_MS),
                     r->csmaPhase == CSMA_DIFS ? "difs" : "backoff",
                     (int)(r->appcCwPassed / (r->appcSlotTicks ? r->appcSlotTicks : 1)),
                     r->appcCw, (unsigned)r->appcBand,
                     (int)(appcAirtime(r) * 100.0f), (double)r->noiseFloor);
            else
                warn("lora/%d LBT: tx stalled %u ms by channel contention "
                     "(phase=%s cw=%d noise=%.0f dBm)",
                     r->idx, (unsigned)(waited * portTICK_PERIOD_MS),
                     r->csmaPhase == CSMA_DIFS ? "difs" : "backoff",
                     r->csmaCw, (double)r->noiseFloor);
        }
        /* Channel never cleared within lbt_timeout → drop the head frame instead
         * of blocking the outbound queue behind a wedged-busy channel. */
        if (r->lbtTimeoutTicks && waited >= r->lbtTimeoutTicks) {
            LoraPkt* p = loraqAt(&r->q, 0);
            size_t   n = p ? p->len : 0;
            queueDiscardHead(r);
            r->txDropped++;
            /* Consumers gate send-failure attribution on this counter at
             * settle time, so it is mirrored the moment a frame is shed —
             * the coalesced stats flush alone lags up to a second. */
            {
                char b[48];
                storageSet(rk(b, sizeof b, r->idx, "stats.tx_dropped"),
                           (int)(r->txDropped & 0x7fffffff));
            }
            err("lora/%d LBT: channel busy > %u ms, dropped %u B frame",
                r->idx, (unsigned)r->lbtTimeoutMs, (unsigned)n);
            csmaResetAccess(r);         /* re-arm access state for the next frame */
            r->csmaStalled = false;
        }
        return;
    }
    r->csmaStalled = false;
    {
        /* The total is everything since the frame first could not go out; the
         * contention part is what channel access just spent. What is left is
         * ours — the radio busy with our own traffic, a split still landing. */
        uint32_t total = r->txWaitPend ? millis() - r->txWaitStartMs : 0;
        uint32_t cont  = csmaGrantWaitMs(r);
        if (cont > total) cont = total;
        r->txWaitMs   = (uint16_t)(cont > 0xFFFF ? 0xFFFF : cont);
        r->txOwnMs    = (uint16_t)((total - cont) > 0xFFFF ? 0xFFFF : (total - cont));
        r->txWaitPend = false;
    }
    queueSendHead(r);
}

/* Ask the chip what just completed and act on it — the IRQ flags are ground
 * truth, so we never guess TX-vs-RX from software state. Half-duplex, so at most
 * one of TX_DONE / RX_DONE is set. txActive is consulted only to run the TxDone
 * watchdog when the chip reports nothing (a wedged transmit). */
static void serviceRadioLocked(LoraRadio* r);

/* **Serialised against SUPE's timer steps.** A transaction advances from the
 * esp_timer task — retune, sense, transmit — while this runs on the radio task,
 * and both drive the same chip. Without the lock a radio task delayed (a
 * storage stall is enough) can resume inside `handleRxDone` and re-arm receive
 * across a `startTransmit` the timer has just issued: the frame never leaves,
 * no TxDone ever arrives, and the watchdog abandons it a second later.
 *
 * Holding it here is cheap now that no transmit blocks — the timer's steps are
 * register writes and a return — which is exactly why this was not affordable
 * before. */
void serviceRadio(LoraRadio* r) {
    supeLock(r);
    serviceRadioLocked(r);
    supeUnlock(r);
}

static void serviceRadioLocked(LoraRadio* r) {
    uint32_t flags = r->radio->getIrqFlags();

    if (flags & r->irqTxDone) {
        r->radio->finishTransmit();          /* clear IRQ, chip → standby */
        r->txFrames++;
        /* Record the frame that just went out (one per split half). */
        uint8_t  doneIdx  = r->txFrameSent;
        uint8_t  doneType = r->txType[doneIdx];
        uint32_t dur = (uint32_t)lround(1000.0 * loraAirtimeSeconds(
                           r->airSf, r->airBwHz, r->cfgCr, r->airPreamble,
                           (int)r->txFrameLen[doneIdx], r->airImplicit));
        /* The chip's own account of the frame it just flew, against the
         * formula's: start-of-air to this TxDone, minus the computed time on
         * air, should be IRQ+task latency and nothing else. A steady excess
         * means the modem is flying settings the formula does not know about
         * (preamble length is the classic), and every deadline and record
         * derived from the formula is off by that much. */
        if (logIsVerbose(TAG)) {
            uint32_t measured = millis() - r->txFrameStartMs;
            if (measured > dur + 5 || measured + 5 < dur)
                verb("lora/%d tx toa measured %lums computed %lums (%uB)",
                     r->idx, (unsigned long)measured, (unsigned long)dur,
                     (unsigned)r->txFrameLen[doneIdx]);
        }
        /* Everything but our own air protocol carries the 1-byte seq/split
         * header on air; the record reports payload bytes, so strip it.
         * RNode-origin packets go out through that same framing as rnsd's. */
        loraMonPush(r, 1 /*tx*/, r->txFrameStartMs, (uint16_t)dur,
                    (uint16_t)(r->txFrameLen[doneIdx] -
                               (doneType == LORA_PKT_OURS ? 0 : 1)),
                    0, 0, r->txPwrNow, doneType,
                    doneIdx == 0 ? r->txWaitMs : 0,
                    doneIdx == 0 ? r->txOwnMs  : 0);
        /* Every frame we put on air is charged somewhere, and *which* somewhere
         * is the whole point. Hailing-channel frames feed the APPC band, which
         * is chosen from this radio's own recent airtime. A detour's frames —
         * the train, which is the bulk of a detour — feed the regime's
         * per-channel enforcement ring instead. Crediting them to the band
         * would make the node contend as though it had spent the shared channel
         * it deliberately did not, and would leave the ring defending a budget
         * it never saw most of. */
        if (r->chNow == LORA_CH_HAIL) appcAddAirtime(r, dur);
        else                          airtimeRecord(r, r->chNow, dur);
        if (++r->txFrameSent < r->txFrameCount) {   /* split: send the second half */
            startTxFrame(r, r->txFrameSent);
            return;
        }
        r->txBytes += r->txPayloadBytes;
        txRearmRx(r);                        /* whole packet sent → back to listening */
        return;
    }

    if (flags & r->irqRxDone) {
        handleRxDone(r);
        return;
    }

    /* Nothing completed. If a transmit is outstanding and overdue, the chip is
     * wedged — recover rather than block outbound forever. */
    if (r->txActive && (int32_t)(xTaskGetTickCount() - r->txDeadline) >= 0) {
        warn("lora/%d TxDone timeout — aborting frame, re-arming RX", r->idx);
        r->radio->finishTransmit();
        /* The regulation counts emissions, not successes (SUPE.md §14.4): the
         * frame may have been modulating the whole watchdog window, so its
         * airtime is credited exactly as a completed one's would be. */
        {
            uint32_t dur = (uint32_t)lround(1000.0 * loraAirtimeSeconds(
                               r->airSf, r->airBwHz, r->cfgCr, r->airPreamble,
                               (int)r->txFrameLen[r->txFrameSent], r->airImplicit));
            if (r->chNow == LORA_CH_HAIL) appcAddAirtime(r, dur);
            else                          airtimeRecord(r, r->chNow, dur);
        }
        /* The frame did not air, so nothing that was waiting on it may proceed
         * as though it had. */
        r->txAborted = true;
        txRearmRx(r);
        r->txAborted = false;
        return;
    }

    /* Nothing completed and no transmit outstanding, yet the IRQ line is still
     * asserted: the chip is holding an interrupt this loop has no handler for.
     * Left alone it is a permanent wake source — the task would be recalled to
     * this same pass forever — so clear the chip's flags and go back to
     * listening. Re-enabling the GPIO interrupt is part of the repair: the
     * trampoline disables it on every fire, and the path that failed to re-enable
     * it is exactly the one that gets us here. */
    if (!r->txActive && radioIrqLinePending(r)) {
        static uint32_t lastWarnMs = 0;
        uint32_t nowMs = millis();
        if (nowMs - lastWarnMs > 10000) {
            lastWarnMs = nowMs;
            warn("lora/%d unhandled IRQ (flags 0x%08lx) — clearing and re-arming RX",
                 r->idx, (unsigned long)flags);
        }
        radioIrqClearAll(r);
        rearmRx(r);
    }
}

static void onRnsdRecv(int handle, size_t /*bytesAvail*/) {
    LoraRadio* r = radioByHandle(handle);
    if (r) drainOneOutbound(r);
}

/* Announce buffer: allocated once, kept across config cycles like the peer
 * table. A failed alloc leaves the replay off (every path guards on ann). */
void annInit(LoraRadio* r) {
    if (r->ann) return;
    r->ann = (AnnBuf*)gp_alloc(sizeof(AnnBuf));
    if (r->ann) std::memset(r->ann, 0, sizeof(AnnBuf));
}

#endif  /* CONFIG_LORA0_CS_PIN */
