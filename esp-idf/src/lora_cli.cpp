/**
 * lora_cli — the `lora` command tree: slot status, the neighbour printer,
 * `lora supe`, the announce replay trigger, config setters, and the manual
 * transmit machinery (tx / tx_psa / tx_prot) it drives on the radio task.
 */
#include "lora_priv.h"

#if defined(CONFIG_LORA0_CS_PIN)

static void manualTxFinish(LoraRadio* r, bool ok, const char* msg) {
    r->mtxPhase = MTXP_OFF;
    r->mtxResOk = ok;
    safeStrncpy(r->mtxResMsg, msg ? msg : "", sizeof r->mtxResMsg);
    r->mtxResGen = r->mtxResGen + 1;   /* release the CLI's poll loop */
}

/* tx_prot: put a header on the air that announces a long 4/8 packet, then cut
 * the carrier before the body — every explicit-header receiver on the channel
 * commits its RX window for the announced duration while we occupy the channel
 * only for the preamble and the 8 header symbols.
 *
 * The header is built by the chip in normal explicit mode, so its length/CR/CRC
 * fields and the header CRC are spec-correct by construction; the "fake" is that
 * the body it promises is never sent. We size the announced payload length so the
 * post-header airtime a receiver computes for it is closest to the requested ms.
 *
 * Synchronous: this is a deliberate one-shot command, so a ~header-length task
 * stall (tens of ms) is acceptable here, unlike on the RX hot path. */
static void manualTxProt(LoraRadio* r) {
    int    sf  = r->cfgSf, bw = r->cfgBwHz, pre = r->cfgPreamble;
    double tSym = (double)((uint32_t)1 << sf) / (double)bw;   /* seconds */
    int    de   = (tSym > 0.016) ? 1 : 0;                     /* LDRO, as in loraAirtimeSeconds */

    /* Announced length whose post-header 4/8 airtime is closest to the target.
     * The receiver's commit after the header is blocks·(CR+4)·Tsym; for 4/8 that
     * is blocks·8·Tsym. Scan against the same model the device reports airtime
     * with so the figure matches what a receiver derives from the header. */
    double target = (double)r->mtxProtMs / 1000.0;
    int    bestL = 0;
    double bestErr = 1e30, bestRem = 0.0;
    for (int L = 0; L <= 255; L++) {
        double num    = 8.0 * L - 4.0 * sf + 28.0 + 16.0 /*CRC on*/;
        double den    = 4.0 * (sf - 2 * de);
        double blocks = fmax(ceil(num / den), 0.0);
        double rem    = blocks * 8.0 * tSym;                  /* CR+4 = 8 for 4/8 */
        double err    = fabs(rem - target);
        if (err < bestErr) { bestErr = err; bestL = L; bestRem = rem; }
    }

    /* Air only the preamble + the 8-symbol header block, plus a few symbols of
     * margin so the whole header is certainly modulated before we abort. */
    double hdrSec = (pre + 4.25 + 8.0 + 4.0) * tSym;
    uint16_t hdrMs = (uint16_t)ceil(hdrSec * 1000.0);

    /* Explicit header (announces a length) at 4/8, so both the header's own CR
     * field and the length→airtime mapping are those of a 4/8 packet. */
    int16_t st = radioHeaderMode(r, false, 0);
    if (st == RADIOLIB_ERR_NONE) st = radioSetCodingRate(r, 8);
    if (st == RADIOLIB_ERR_NONE) st = r->radio->standby();
    if (st != RADIOLIB_ERR_NONE) {
        radioSetCodingRate(r, (uint8_t)r->cfgCr);
        rearmRx(r);
        manualTxFinish(r, false, "modem cfg failed");
        return;
    }
    r->airImplicit = false;
    apApplyPower(r, r->cfgTxp);

    /* The body is never heard, so its contents do not matter. */
    static uint8_t dummy[256] = {0};
    r->txFrameStartMs = millis();
    st = r->radio->startTransmit(dummy, (size_t)bestL);
    if (st != RADIOLIB_ERR_NONE) {
        radioSetCodingRate(r, (uint8_t)r->cfgCr);
        rearmRx(r);
        manualTxFinish(r, false, "startTransmit failed");
        return;
    }
    delay(hdrMs);                     /* let the preamble + header go out … */
    /* … then drop the carrier before the body. finishTransmit both halts
     * modulation (chip → standby) and clears any TX-done IRQ the chip may have
     * latched if the short body actually completed within the delay — so the
     * next serviceRadio pass can't mistake it for a queued frame's completion. */
    r->radio->finishTransmit();

    loraMonPush(r, 1 /*tx*/, r->txFrameStartMs, hdrMs,
                (uint16_t)bestL, 0, 0, r->cfgTxp, LORA_PKT_OURS, 0, 0);
    appcAddAirtime(r, hdrMs);
    r->txFrames++;

    radioSetCodingRate(r, (uint8_t)r->cfgCr);   /* restore the configured payload CR */
    r->txActive = false;
    rearmRx(r);

    char msg[72];
    snprintf(msg, sizeof msg, "committed ~%d ms (announced %d B / 4/8, header ~%d ms on air)",
             (int)lround(bestRem * 1000.0), bestL, (int)hdrMs);
    manualTxFinish(r, true, msg);
}

/* Service a pending manual-TX request and drive the PSA carrier-sense.
 * Runs after serviceRadio in the task loop, so a completed TxDone (which clears
 * txActive and re-arms RX) is already reflected when we check it here. */
void manualTxPoll(LoraRadio* r) {
    if (r->mtxPhase == MTXP_OFF) {
        if (!r->mtxReq) return;
        r->mtxReq = false;
        if (!r->running) { manualTxFinish(r, false, "radio not up"); return; }
        /* A SUPE transaction owns the radio: its frames sit inside a schedule
         * the other end is timing against, and it may be tuned off the hailing
         * channel entirely. Refuse rather than transmit into the middle of one;
         * the standoff has to run both ways or it is not one. */
        if (r->splitPending || r->txActive || supeHoldsRadio(r))
            { manualTxFinish(r, false, "radio busy"); return; }

        if (r->mtxKind == MTX_PROT) { manualTxProt(r); return; }

        /* RAW / PSA: the payload goes on air verbatim as one explicit-header
         * frame at the configured params — no RNS seq/split byte, so what the
         * user passed is exactly what airs. */
        memcpy(r->txFrame[0], r->mtxData, r->mtxLen);
        r->txFrameLen[0]  = r->mtxLen;
        r->txFrameCount   = 1;
        r->txFrameSent    = 0;
        r->txPayloadBytes = r->mtxLen;
        r->txType[0]      = LORA_PKT_OURS;
        r->txWaitMs       = 0;
        apApplyPower(r, r->cfgTxp);

        if (r->mtxKind == MTX_PSA && r->lbt) {   /* carrier-sense before firing */
            r->mtxPhase    = MTXP_LBT;
            csmaResetAccess(r);
            r->csmaStart   = xTaskGetTickCount();
            r->mtxDeadline = r->csmaStart +
                (r->lbtTimeoutTicks ? r->lbtTimeoutTicks : pdMS_TO_TICKS(10000));
            return;
        }
        r->mtxPhase = MTXP_TX;
        startTxFrame(r, 0);
        return;
    }

    if (r->mtxPhase == MTXP_LBT) {
        if (csmaClear(r)) { r->mtxPhase = MTXP_TX; startTxFrame(r, 0); return; }
        if ((int32_t)(xTaskGetTickCount() - r->mtxDeadline) >= 0) {
            csmaResetAccess(r);
            manualTxFinish(r, false, "channel busy (LBT timeout)");
        }
        return;
    }

    /* MTXP_TX: serviceRadio clears txActive at TxDone and re-arms RX. */
    if (!r->txActive) manualTxFinish(r, true, "sent");
}

/* ─────────────── CLI ─────────────── */

static const char* foundStr(const LoraRadio* r) {
    return r->found == 1 ? "found" : r->found == 0 ? "NOT FOUND" : "unprobed";
}

static void cliPrintSlot(int i) {
    LoraRadio* r = &s_radios[i];
    const LoraSlot* s = r->slot;
    cliPrintf("lora/%d  radio=%-6s [%s]  state=%s\n", i, chipName(s->chip), foundStr(r),
              r->running ? "up" : (r->enabled ? "starting" : "down"));
    cliPrintf("        pins cs=%d irq=%d busy=%d rst=%d  tcxo=%dmV  dio2_rf=%d  rfsw=%d/%d\n",
              s->cs, s->dio1, s->busy, s->rst, s->tcxo_mv, s->dio2_rf_switch ? 1 : 0,
              s->rfsw_rx, s->rfsw_tx);

    char kb[48];
    int  freq_hz = storageGetInt(sk(kb, sizeof kb, i, "frequency"), 0);
    int  bw_hz   = storageGetInt(sk(kb, sizeof kb, i, "bandwidth"), 0);
    int  sf      = storageGetInt(sk(kb, sizeof kb, i, "spreading_factor"), 0);
    int  cr      = storageGetInt(sk(kb, sizeof kb, i, "coding_rate"), 0);
    int  txp     = storageGetInt(sk(kb, sizeof kb, i, "tx_power"), 0);
    int  pre     = storageGetInt(sk(kb, sizeof kb, i, "preamble"), 12);
    char mode[24]; storageGetStr(sk(kb, sizeof kb, i, "mode"), mode, sizeof mode, "access_point");
    char sync[16]; storageGetStr(sk(kb, sizeof kb, i, "sync_word"), sync, sizeof sync, "0x42");
    cliPrintf("        freq=%.3f MHz  bw=%.0f kHz  sf=%d  cr=4/%d  txp=%d dBm  preamble=%d\n",
              freq_hz / 1.0e6, bw_hz / 1.0e3, sf, cr, txp, pre);
    cliPrintf("        sync=%s  mode=%s  bitrate=%u bit/s\n", sync, mode, (unsigned)r->curBitrate);
    if (chipFamily(s->chip) == FAM_SX126X)
        cliPrintf("        rx_boosted_gain=%d\n",
                  storageGetInt(sk(kb, sizeof kb, i, "rx_boosted_gain"), 1) != 0);
    if (!r->lbt) {
        cliPrintf("        lbt=off (blind tx)\n");
    } else if (!r->appc) {
        cliPrintf("        lbt=on  appc=off  slot=%u ms  difs=%u ms  cw=2^%d slots\n",
                  (unsigned)(r->slotTicks * portTICK_PERIOD_MS),
                  (unsigned)(r->difsTicks * portTICK_PERIOD_MS), r->csmaCw);
    } else {
        uint8_t band = appcLiveBand(r);
        cliPrintf("        lbt=on  appc=on  slot=%u ms  difs=%u ms  "
                  "airtime=%d%%  band=%u/%d (cw %d-%d slots)\n",
                  (unsigned)(r->appcSlotTicks * portTICK_PERIOD_MS),
                  (unsigned)(r->appcDifsTicks * portTICK_PERIOD_MS),
                  (int)(appcAirtime(r) * 100.0f), (unsigned)band, APPC_CW_BANDS,
                  (band - 1) * APPC_CW_PER_BAND_WINDOWS,
                  band * APPC_CW_PER_BAND_WINDOWS - 2);
    }
    cliPrintf("        rx %u/%uB  tx %u/%uB  rssi %d dBm  snr %d dB  crc_err %u  split_to %u\n",
              (unsigned)r->rxFrames, (unsigned)r->rxBytes,
              (unsigned)r->txFrames, (unsigned)r->txBytes,
              (int)r->rssiLast, (int)r->snrLast,
              (unsigned)r->crcErr, (unsigned)r->splitTimeouts);
    {
        cliPrintf("        announces %d buffered for `lora %d a`\n",
                  annCount(r), r->idx);
        /* Dropped telemetry is invisible on the graph and reads as frames that
         * were never transmitted, so say it out loud rather than let someone
         * chase a radio bug that isn't one. */
        if (r->mon.monDropped || r->mon.rssiDropped)
            cliPrintf("        telemetry dropped: %u frame records, %u rssi samples"
                      " (interface queue full)\n",
                      (unsigned)r->mon.monDropped, (unsigned)r->mon.rssiDropped);
    }
}

/* ── `lora [<n>] neighbors` — the passive radio-neighbourhood picture ── */

static void cliAgo(char* b, size_t n, uint32_t now, uint32_t then) {
    uint32_t s = (now - then) / 1000;
    if (s < 120)        snprintf(b, n, "%us", (unsigned)s);
    else if (s < 7200)  snprintf(b, n, "%um", (unsigned)(s / 60));
    else                snprintf(b, n, "%uh", (unsigned)(s / 3600));
}

struct CliPrintCtx { LoraRadio* r; uint32_t now; bool verbose; };

static void cliPrintNode(Neighbor* e, int num, void* ud) {
    CliPrintCtx* c = (CliPrintCtx*)ud;
    char hex[33], ago[16], lbl[8];
    if (e->isRnode)   safeStrncpy(lbl, "rnode", sizeof lbl);
    else if (e->isUs) safeStrncpy(lbl, "us", sizeof lbl);
    else              snprintf(lbl, sizeof lbl, "%d", num);

    /* One line per hash: full hash, aspect, and the announced name if any.
     * The transport aspect leads — it is the hash every node has. */
    bool first = true;
    for (int pass = 0; pass < 2; pass++) {
        for (int d = 0; d < e->nDests; d++) {
            NeiDest* nd = &e->dests[d];
            const char* asp = nd->haveName ? rnsNameLabel(nd->nameHash) : nullptr;
            bool isTransport = asp && strcmp(asp, "rnstransport.probe") == 0;
            if ((pass == 0) != isTransport) continue;
            loraHex(hex, nd->hash, 16);
            char nh[21] = "";
            if (nd->haveName && !asp) loraHex(nh, nd->nameHash, 10);
            cliPrintf("  %-5s%s %s", first ? lbl : "", hex,
                      asp ? asp : (nd->haveName ? nh : "-"));
            if (nd->name[0]) cliPrintf("  \"%s\"", nd->name);
            if (c->verbose) {
                cliAgo(ago, sizeof ago, c->now, nd->lastMs);
                if (nd->announces) cliPrintf("  ann %u", (unsigned)nd->announces);
                cliPrintf("  %s ago", ago);
            }
            cliPrintf("\n");
            first = false;
        }
    }
    /* Hashes a peer linked to this node that we have never heard directly. */
    for (int l = 0; l < e->nLink4; l++) {
        cliPrintf("  %-5s%02x%02x%02x%02x........................ (not seen yet)\n",
                  first ? lbl : "",
                  e->link4[l][0], e->link4[l][1], e->link4[l][2], e->link4[l][3]);
        first = false;
    }
    if (first) {   /* nothing but a bare node key */
        if (e->haveNode4)
            cliPrintf("  %-5s%02x%02x%02x%02x........................ (not seen yet)\n",
                      lbl, e->node4[0], e->node4[1], e->node4[2], e->node4[3]);
        else
            cliPrintf("  %-5s(no hash seen)\n", lbl);
    }

    if (!peersIsLocal(e)) {
        /* Capability line. TRANSPORT means it forwards for others; ROAMING is
         * its node-flags bit; the mesh tag that it speaks our air protocol; TX
         * the power a probe settled on for it; EST the reciprocity estimate;
         * USE the power we transmit to it at under SUPE.adaptive_txpower. */
        char f[96];
        int o = 0;
        auto add = [&](const char* t) {
            o += snprintf(f + o, sizeof f - (size_t)o, "%s%s", o ? ", " : "", t);
        };
        if (e->transit)  add("TRANSPORT");
        if (e->roaming)  add("ROAMING");
        /* A node becomes a SUPE peer by its announcement being heard, and that
         * announcement is what carries the budget this pair could reach on the
         * widest channel — so the tag names the budget rather than merely the
         * protocol once one is known. BUDGET 0 is a real answer: the pair has
         * no rung above hailing. */
        if (e->supeSeen) {
            char t[24];
            uint32_t maxBw = (uint32_t)c->r->cfgBwHz;
            int nch = 0;
            const SupeChan* ch = supeRegimeChans(c->r->afa, &nch);
            for (int k = 0; k < nch; k++) if (ch[k].bwHz > maxBw) maxBw = ch[k].bwHz;
            SupeLadderEntry lad[SUPE_LADDER_MAX_ENTRIES];
            int ln = supeLadder(c->r->afa, SUPE_VERSION, (uint8_t)c->r->cfgSf,
                                (uint32_t)c->r->cfgBwHz, maxBw,
                                supeOwnFamily(c->r), e->supeCaps.fam,
                                lad, SUPE_LADDER_MAX_ENTRIES);
            uint8_t top = (uint8_t)(ln > 0 ? ln - 1 : 0);
            SupeCaps own = supeOwnCaps(c->r);
            if (top > own.topStep)           top = own.topStep;
            if (top > e->supeCaps.topStep)   top = e->supeCaps.topStep;
            snprintf(t, sizeof t, RF_PROTO_NAME " BUDGET %u", (unsigned)top);
            add(t);
        } else if (e->ourProto) add(RF_PROTO_NAME);
        int est10;
        if (peersEstimateCliff10(c->r, e, c->now, &est10, nullptr)) {
            char t[24];
            snprintf(t, sizeof t, "EST %.0f", (double)est10 / 10.0);
            add(t);
        }
        /* USE is the power frames to this node actually go out at; the `~`
         * marks one derived from EST plus a margin rather than measured. */
        if (e->haveApPwr) {
            char t[16];
            snprintf(t, sizeof t, "USE %s%d", e->apFromEst ? "~" : "", (int)e->apPwr);
            add(t);
        }
        if (o) cliPrintf("       ( %s )\n", f);
    }

    if (c->verbose) {
        for (int n = 0; n < e->nIds; n++) {
            loraHex(hex, e->ids[n], 16);
            cliPrintf("       id:%s\n", hex);
        }
        if (e->haveSig) {
            cliAgo(ago, sizeof ago, c->now, e->lastHeardMs);
            cliPrintf("       rssi %d..%d dBm  snr %.1f..%.1f dB  heard %s ago\n",
                      (int)e->rssiMin, (int)e->rssiMax,
                      (double)e->snrMin10 / 10.0, (double)e->snrMax10 / 10.0, ago);
        }
        if (e->haveQuality)
            cliPrintf("       q %u/255 (%u/%u proofs)%s\n",
                      (unsigned)e->quality, (unsigned)e->qProved, (unsigned)e->qSent,
                      e->provesData ? "  proves-data" : "");
        if (e->haveAdv)
            cliPrintf("       hashes %d/%u\n", peersKnownHashes(e), (unsigned)e->advHashes);
        uint32_t absNow = c->now / NEI_BUCKET_MS;
        uint32_t cnt = 0; int64_t rs = 0, ss = 0;
        for (int b = 0; b < NEI_BUCKETS; b++) {
            const NeiBucket* bk = &e->buck[b];
            if (bk->cnt && absNow - bk->absIdx < NEI_BUCKETS) {
                cnt += bk->cnt; rs += bk->rssiSum; ss += bk->snrSum10;
            }
        }
        if (cnt)
            cliPrintf("       1h: %u pkt  avg %d dBm %.1f dB\n",
                      (unsigned)cnt, (int)(rs / (int64_t)cnt),
                      (double)ss / (double)cnt / 10.0);
    }
    cliPrintf("\n");
}

static void cliPrintNeighbors(int i, bool verbose) {
    LoraRadio* r = &s_radios[i];
    NeiState*  st = r->nei;
    if (!st) {
        cliPrintf("lora/%d neighbors: no observations (radio has never been up)\n", i);
        return;
    }
    uint32_t now = millis();
    int nUs = 0, nRnode = 0, nNodes = 0, nLinks = 0;
    for (int k = 0; k < NEI_MAX; k++) {
        Neighbor* e = &st->nei[k];
        if (!e->used) continue;
        if (e->isRnode)   nRnode++;
        else if (e->isUs) nUs++;
        else              nNodes++;
    }
    for (int k = 0; k < NEI_LINKS_MAX; k++)
        if (st->links[k].used) nLinks++;

    /* The local rows are named rather than counted: there is at most one of
     * each, and which of them exist is the interesting part. */
    const char* local = nUs && nRnode ? " and us + rnode"
                      : nUs           ? " and us"
                      : nRnode        ? " and rnode" : "";
    char ago[16];
    cliAgo(ago, sizeof ago, now, st->sinceMs);
    cliPrintf("lora/%d neighbors: %d other%s%s, %d open link%s (observing %s)\n\n",
              i, nNodes, nNodes == 1 ? "" : "s", local,
              nLinks, nLinks == 1 ? "" : "s", ago);
    if (r->curIfacSize)
        cliPrintf("  note: ifac enabled — frames are masked, passive parse sees nothing\n\n");

    CliPrintCtx ctx = { r, now, verbose };
    peersWalk(st, -1, cliPrintNode, &ctx);

    if (!verbose) return;
    for (int k = 0; k < NEI_LINKS_MAX; k++) {
        NeiLink* L = &st->links[k];
        if (!L->used) continue;
        char lid[33], dst[36];
        loraHex(lid, L->linkId, 16);
        if (L->haveDest) loraHex(dst, L->dest, 16);
        else             snprintf(dst, sizeof dst, "?");
        cliAgo(ago, sizeof ago, now, L->lastMs);
        cliPrintf("  link %s -> %s  %s %s", lid, dst,
                  L->ours ? "ours" : "seen",
                  L->unresolved ? "unresolved" : (L->established ? "established" : "pending"));
        if (L->haveSig)
            cliPrintf("  %d dBm %.1f dB", (int)L->lastRssi, (double)L->lastSnr10 / 10.0);
        cliPrintf("  %u pkt  %s ago\n", (unsigned)L->frames, ago);
    }
}







/* `tok` abbreviates `full` when it is a prefix of it and at least `minLen`
 * long — so `lora n` / `lora neigh` / `lora neighbours` and `lora a` /
 * `lora announce` all reach the same place. */
static bool cliVerb(const char* tok, const char* full, size_t minLen) {
    size_t n = strlen(tok);
    return n >= minLen && n <= strlen(full) && strncmp(tok, full, n) == 0;
}
/* `lora [<n>] a[nnounce]` — replay every buffered announce, then the radio
 * check, now rather than at the next beat. The beat restarts from the run's
 * end, so this also reschedules. */
static void cliAnnounce(int idx) {
    if (idx < 0 || idx >= kNumRadios) { cliPrintf("no radio %d\n", idx); return; }
    LoraRadio* r = &s_radios[idx];
    if (!r->running) { cliPrintf("lora/%d not running\n", idx); return; }
    int n = annReplayStart(r);
    if (n == 0) {
        cliPrintf("lora/%d nothing buffered — no announce has been originated here yet\n", idx);
        return;
    }
    if (s_task) xTaskNotifyGive(s_task);
    cliPrintf("lora/%d repeating %d announce%s, then this node's own SUPE announcement\n",
              idx, n, n == 1 ? "" : "s");
}

/* `lora <n> supe` — everything a field report asks first, in one screen: which
 * dialect this build speaks and until when, what the interface resolved the
 * regime to, what the tag set has learned, what is being held, and the counters
 * that say whether anyone is answering.
 *
 * `lora <n> supe rx <hex>` injects a frame into the receive path exactly as if
 * the radio had decoded it. The hex is meant to come from the golden vectors
 * the host tests emit (esp-idf/test/golden.txt), never hand-written: that is
 * what keeps a one-device test and the codec from disagreeing. */
static int cliParseBytes(const char* s, uint8_t* out, size_t cap);

static void cliSupe(int idx, const char* sub, const char* arg) {
    LoraRadio* r = &s_radios[idx];
    SupeState* st = r->supe;

    if (sub && strcmp(sub, "rx") == 0) {
        if (!arg || !*arg) { cliPrintf("usage: lora %d supe rx <hex>\n", idx); return; }
        uint8_t f[SUPE_MAX_FRAME];
        int n = cliParseBytes(arg, f, sizeof f);
        if (n <= 0) { cliPrintf("bad hex\n"); return; }
        if (!supeReady(r)) { cliPrintf("SUPE is not enabled on lora/%d\n", idx); return; }
        /* A plausible level and signal-to-noise, so anything the frame feeds —
         * a path-loss pair, a capability row — lands with a believable pair
         * behind it rather than with zero. */
        supeOnFrame(r, f, (size_t)n, -80, 80);
        cliPrintf("injected %d B into lora/%d's SUPE receive path\n", n, idx);
        return;
    }

    uint32_t nowUnix = (uint32_t)time(nullptr);
    uint32_t exp = supeExpiryUnix();
    char kb[48];
    cliPrintf("lora/%d SUPE: %s\n", idx,
              r->supeOn ? "on"
                        : (storageGetInt(sk(kb, sizeof kb, idx, "SUPE.enable"), 0)
                               ? "off (an access code is configured)" : "off"));
    const SupeRegime* g = supeRegime(r->afa);
    cliPrintf("  regime      %u (%s), version %u\n", (unsigned)r->afa,
              g ? g->name : "unrecognised — no agile channels", SUPE_VERSION);
    {
        char when[40] = "";
        time_t t = (time_t)exp;
        struct tm tmv;
        if (gmtime_r(&t, &tmv)) strftime(when, sizeof when, "%Y-%m-%d %H:%M UTC", &tmv);
        int32_t left = (int32_t)(exp - nowUnix);
        cliPrintf("  expires     %s (%s)\n", when,
                  supeExpired(nowUnix) ? "EXPIRED — not speaking it"
                                       : "in progress");
        if (!supeExpired(nowUnix) && nowUnix)
            cliPrintf("              %d h %d min left\n",
                      (int)(left / 3600), (int)((left % 3600) / 60));
    }
    {
        SupeCaps c = supeOwnCaps(r);
        cliPrintf("  we are      family %u, ceiling %u, max %d dBm%s\n",
                  (unsigned)c.fam, (unsigned)c.topStep, (int)c.maxPwrDbm,
                  c.adaptive ? ", adaptive power" : "");
        /* The ladder as the widest channel would resolve it, family-bounded on
         * our side alone — what a symmetrical peer could be granted. */
        uint32_t maxBw = (uint32_t)r->cfgBwHz;
        int nch = 0;
        const SupeChan* ch = supeRegimeChans(r->afa, &nch);
        for (int i = 0; i < nch; i++) if (ch[i].bwHz > maxBw) maxBw = ch[i].bwHz;
        SupeLadderEntry lad[SUPE_LADDER_MAX_ENTRIES];
        int ln = supeLadder(r->afa, SUPE_VERSION, (uint8_t)r->cfgSf,
                            (uint32_t)r->cfgBwHz, maxBw, c.fam, c.fam,
                            lad, SUPE_LADDER_MAX_ENTRIES);
        for (int i = 1; i < ln; i++)
            cliPrintf("  budget %-3d SF%u / %u kHz  (%+.1f dB of margin)\n",
                      i, (unsigned)lad[i].sf, (unsigned)(lad[i].bwHz / 1000),
                      -(double)lad[i].marginDeci / 10.0);
        if (ln <= 1)
            cliPrintf("  budgets     none from this hailing configuration\n");
    }
    if (!st) { cliPrintf("  (no state allocated)\n"); return; }
    SupeEngine* e = &st->eng;

    uint32_t now = millis();
    int perm = 0, transient = 0;
    for (int i = 0; i < SUPE_TAGS_MAX; i++) {
        if (!e->tags[i].used) continue;
        if (e->tags[i].perm) perm++; else transient++;
    }
    cliPrintf("  tag set     %d ours, %d transient (of %d)\n",
              perm, transient, SUPE_TAGS_MAX);
    for (int i = 0, shown = 0; i < SUPE_TAGS_MAX && shown < 12; i++) {
        SupeTag* t = &e->tags[i];
        if (!t->used) continue;
        shown++;
        if (t->perm)
            cliPrintf("    %02x%02x%02x    ours\n", t->tag[0], t->tag[1], t->tag[2]);
        else
            cliPrintf("    %02x%02x%02x    %u ref%s, %d s left\n",
                      t->tag[0], t->tag[1], t->tag[2], (unsigned)t->refs,
                      t->refs == 1 ? "" : "s", (int)((int32_t)(t->expiryMs - now) / 1000));
    }
    for (int i = 0; i < SUPE_HOLD_MAX; i++) {
        SupeHold* h = &e->hold[i];
        if (!h->used || (int32_t)(now - h->untilMs) >= 0) continue;
        cliPrintf("  holding     %02x%02x%02x for %d ms\n",
                  h->tag[0], h->tag[1], h->tag[2], (int)(int32_t)(h->untilMs - now));
    }
    cliPrintf("  rx          %u frames, %u discarded, %u for other exchanges\n",
              (unsigned)e->rxFrames, (unsigned)e->rxDiscard, (unsigned)e->rxForeign);
    {
        int peers = 0;
        if (r->nei)
            for (int i = 0; i < NEI_MAX; i++) {
                Neighbor* en = &r->nei->nei[i];
                if (en->used && en->supeSeen && !peersIsLocal(en)) peers++;
            }
        cliPrintf("  peers       %d speak SUPE\n", peers);
        if (r->nei)
            for (int i = 0; i < NEI_MAX; i++) {
                Neighbor* en = &r->nei->nei[i];
                if (!en->used || !en->supeSeen || peersIsLocal(en)) continue;
                uint8_t h4[4] = {};
                peersNodeFirst4(en, h4);
                cliPrintf("    %02x%02x%02x%02x  family %u, ceiling %u, "
                          "heard %us ago%s%s\n",
                          h4[0], h4[1], h4[2], h4[3], (unsigned)en->supeCaps.fam,
                          (unsigned)en->supeCaps.topStep,
                          (unsigned)((now - en->supeHeardMs) / 1000),
                          en->havePair ? "" : ", no path-loss pair yet",
                          (en->absentUntilMs &&
                           (int32_t)(en->absentUntilMs - now) > 0) ? ", ABSENT" : "");
            }
    }
    cliPrintf("  requests    %u out, %u granted, %u refused; %u strikes\n",
              (unsigned)e->startsOut, (unsigned)e->grantsIn,
              (unsigned)e->refusalsIn, (unsigned)e->strikes);
    cliPrintf("  answers     %u granted, %u refused\n",
              (unsigned)e->grantsOut, (unsigned)e->refusalsOut);
    cliPrintf("  detours     %u completed; %u packets out, %u in\n",
              (unsigned)e->detoursDone, (unsigned)e->trainPktsOut,
              (unsigned)e->trainPktsIn);
    cliPrintf("  verdicts    %u holds taken, %u packets dropped as absent\n",
              (unsigned)e->holdsTaken, (unsigned)e->dropsAbsent);
    {
        /* The last transaction ends, oldest first — the engine's own record,
         * good even when the debug log dropped the lines. */
        uint32_t nowMs = millis();
        int nEnds = (int)(sizeof e->ends / sizeof e->ends[0]);
        bool any = false;
        for (int i = 0; i < nEnds; i++) {
            const auto* er = &e->ends[(e->endsAt + i) % nEnds];
            if (!er->why) continue;
            if (!any) { cliPrintf("  last ends   (age role ch sent/plan got/exp outcome)\n"); any = true; }
            cliPrintf("    %6lus %s ch%u %u/%u %u/%u %s (%s)\n",
                      (unsigned long)((nowMs - er->endedMs) / 1000u),
                      er->role_b ? "B" : "A", (unsigned)er->chan,
                      (unsigned)er->sent, (unsigned)er->plan,
                      (unsigned)er->got, (unsigned)er->expect,
                      er->ok ? "ok" : "FAIL", er->why);
        }
    }
    if (g && g->airtimeMaxMs) {
        cliPrintf("  budget      channels");
        int nch = 0;
        supeRegimeChans(r->afa, &nch);
        for (int c = 1; c <= nch && c < SUPE_CH_MAX; c++)
            cliPrintf(" %d:%s", c, r->chans && r->chans->chanOk[c] ? "ok" : "FULL");
        cliPrintf("\n");
    }
    if (e->x.phase != SUPE_X_IDLE)
        cliPrintf("  in flight   phase %u, tag %02x%02x%02x, ch%u budget %u (%s)\n",
                  (unsigned)e->x.phase, e->x.tag[0], e->x.tag[1], e->x.tag[2],
                  (unsigned)e->x.chan, (unsigned)e->x.budget,
                  e->x.role_b ? "answering" : "requesting");
    if (r->q.n) cliPrintf("  queued      %u packet%s\n",
                          (unsigned)r->q.n, r->q.n == 1 ? "" : "s");
}

static bool cliIsNeighbors(const char* t) {
    return cliVerb(t, "neighbors", 1) || cliVerb(t, "neighbours", 1);
}

/* Pointer into `orig` just past the first `skip` whitespace-separated tokens,
 * with the remaining text kept verbatim (embedded spaces included). Returns null
 * if there are fewer than `skip` tokens; may point at the terminating NUL (empty
 * remainder). Used to recover a tx payload from the untruncated args, since the
 * tokeniser above copies only the first bytes for dispatch. */
static const char* cliRest(const char* orig, int skip) {
    const char* p = orig ? orig : "";
    for (int i = 0; i < skip; i++) {
        while (*p == ' ') p++;
        if (!*p) return nullptr;
        while (*p && *p != ' ') p++;
    }
    while (*p == ' ') p++;
    return p;
}

/* Parse a tx payload string into raw bytes. A literal `0x` followed by an even
 * run of hex digits inserts those bytes (`0x0a`, `0x48656c6c6f`); every other
 * character is taken as its own ASCII byte, spaces included. Returns the byte
 * count, or -1 on overflow or an odd-length 0x run. */
static int cliParseBytes(const char* s, uint8_t* out, size_t cap) {
    auto hx = [](char c) -> int {
        c = (char)tolower((unsigned char)c);
        return c <= '9' ? c - '0' : c - 'a' + 10;
    };
    size_t n = 0;
    while (*s) {
        if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X') &&
            isxdigit((unsigned char)s[2]) && isxdigit((unsigned char)s[3])) {
            const char* h = s + 2;
            size_t hd = 0;
            while (isxdigit((unsigned char)h[hd])) hd++;
            if (hd & 1) return -1;                 /* half a byte → malformed */
            for (size_t i = 0; i < hd; i += 2) {
                if (n >= cap) return -1;
                out[n++] = (uint8_t)((hx(h[i]) << 4) | hx(h[i + 1]));
            }
            s = h + hd;
        } else {
            if (n >= cap) return -1;
            out[n++] = (uint8_t)*s++;
        }
    }
    return (int)n;
}

/* tx / tx_psa / tx_prot: hand a manual-transmit request to the lora task and
 * block on its result. `cmd` is the verb, `rest` the verbatim argument tail. */
static void cliManualTx(long idx, const char* cmd, const char* rest) {
    LoraRadio* r = &s_radios[idx];
    bool isProt = strcmp(cmd, "tx_prot") == 0;
    bool isPsa  = strcmp(cmd, "tx_psa")  == 0;
    if (!r->running)                    { cliPrintf("lora/%ld is not up\n", idx); return; }
    if (r->mtxPhase != MTXP_OFF || r->mtxReq) {
        cliPrintf("lora/%ld tx already in progress\n", idx); return;
    }
    if (isProt) {
        if (!rest || !*rest) { cliPrintf("usage: lora %ld tx_prot <ms>\n", idx); return; }
        long ms = strtol(rest, nullptr, 10);
        if (ms <= 0)      { cliPrintf("tx_prot: <ms> must be > 0\n"); return; }
        if (ms > 0xFFFF)  ms = 0xFFFF;
        r->mtxProtMs = (uint16_t)ms;
        r->mtxKind   = MTX_PROT;
    } else {
        if (!rest || !*rest) {
            cliPrintf("usage: lora %ld %s <string>   (0x<hex> inserts raw bytes)\n", idx, cmd);
            return;
        }
        int n = cliParseBytes(rest, r->mtxData, 255);   /* one explicit frame, ≤255 B */
        if (n < 0)  { cliPrintf("%s: payload > 255 B or bad 0x<hex> run\n", cmd); return; }
        if (n == 0) { cliPrintf("%s: empty payload\n", cmd); return; }
        r->mtxLen  = (uint16_t)n;
        r->mtxKind = isPsa ? MTX_PSA : MTX_RAW;
    }

    uint32_t gen = r->mtxResGen;
    r->mtxReq = true;
    if (s_task) xTaskNotifyGive(s_task);
    /* RAW/PROT complete in a few ms; PSA can back off up to lbt_timeout. Cap the
     * wait well past the worst case (SF12 APPC + a busy channel). */
    for (int i = 0; i < 400 && r->mtxResGen == gen; i++) delay(50);   /* ≤ 20 s */
    if (r->mtxResGen == gen) { cliPrintf("lora/%ld %s: no result (timeout)\n", idx, cmd); return; }
    cliPrintf("lora/%ld %s: %s%s\n", idx, cmd,
              r->mtxResOk ? "" : "failed: ", r->mtxResMsg);
}

void cliLora(const char* args) {
    char buf[80];
    safeStrncpy(buf, args ? args : "", sizeof buf);
    char* tok[4] = {};
    int   nt = 0;
    char* save = nullptr;
    for (char* t = strtok_r(buf, " ", &save); t && nt < 4; t = strtok_r(nullptr, " ", &save))
        tok[nt++] = t;

    if (nt == 0) {                                  /* `lora` → all slots */
        for (int i = 0; i < kNumRadios; i++) cliPrintSlot(i);
        return;
    }
    if (strcmp(tok[0], "help") == 0 || strcmp(tok[0], "-h") == 0) {
        cliPrintf("%-*s LoRa status for all radios\n",      CLI_HELP_COL, "lora");
        cliPrintf("%-*s status for one radio\n",            CLI_HELP_COL, "lora <n>");
        cliPrintf("%-*s enable/disable (no <n> = all)\n",   CLI_HELP_COL, "lora [<n>] up|down");
        cliPrintf("%-*s observed direct neighbours (-v for detail)\n", CLI_HELP_COL, "lora [<n>] n[eighbors]");
        cliPrintf("%-*s repeat every announce, then our SUPE announcement\n", CLI_HELP_COL, "lora [<n>] a[nnounce]");
        cliPrintf("%-*s freq MHz / bw kHz / sf / cr /\n",   CLI_HELP_COL, "lora <n> <param> <val>");
        cliPrintf("%-*s   txp dBm / preamble / sync / mode / lbt 0|1 / appc 0|1 /\n", CLI_HELP_COL, "");
        cliPrintf("%-*s   rx_boosted_gain 0|1\n", CLI_HELP_COL, "");
        cliPrintf("%-*s blind-transmit a payload (0x<hex> = raw bytes)\n", CLI_HELP_COL, "lora <n> tx <string>");
        cliPrintf("%-*s carrier-sense (as normal tx), then transmit\n", CLI_HELP_COL, "lora <n> tx_psa <string>");
        cliPrintf("%-*s emit a header committing receivers for <ms> (4/8)\n", CLI_HELP_COL, "lora <n> tx_prot <ms>");
        cliPrintf("%-*s SUPE state: regime, expiry, tag set, holds, counters\n", CLI_HELP_COL, "lora <n> supe");
        cliPrintf("%-*s inject a golden-vector frame into the receive path\n", CLI_HELP_COL, "lora <n> supe rx 0x<hex>");
        return;
    }
    if (cliIsNeighbors(tok[0])) {                           /* all radios */
        bool v = nt > 1 && strcmp(tok[1], "-v") == 0;
        for (int i = 0; i < kNumRadios; i++) cliPrintNeighbors(i, v);
        return;
    }
    if (cliVerb(tok[0], "announce", 1)) { cliAnnounce(0); return; }   /* no index → radio 0 */

    char kb[48];
    /* `lora up|down` → all radios. */
    if (nt == 1 && (strcmp(tok[0], "up") == 0 || strcmp(tok[0], "down") == 0)) {
        int v = strcmp(tok[0], "up") == 0 ? 1 : 0;
        storageBegin();
        for (int i = 0; i < kNumRadios; i++) storageSet(sk(kb, sizeof kb, i, "enable"), v);
        storageEnd();
        cliPrintf("%s %d radio(s)\n", v ? "enabled" : "disabled", kNumRadios);
        return;
    }

    /* `lora <n> ...` */
    char* end = nullptr;
    long  idx = strtol(tok[0], &end, 10);
    if (end == tok[0] || *end || idx < 0 || idx >= kNumRadios) {
        cliPrintf("no such radio '%s' (have 0..%d)\n", tok[0], kNumRadios - 1);
        return;
    }
    if (nt == 1) { cliPrintSlot((int)idx); return; }

    const char* cmd = tok[1];
    if (strcmp(cmd, "up") == 0)   { storageSet(sk(kb, sizeof kb, idx, "enable"), 1); cliPrintf("lora/%ld enabled\n", idx);  return; }
    if (strcmp(cmd, "down") == 0) { storageSet(sk(kb, sizeof kb, idx, "enable"), 0); cliPrintf("lora/%ld disabled\n", idx); return; }
    if (cliIsNeighbors(cmd)) {
        cliPrintNeighbors((int)idx, nt > 2 && strcmp(tok[2], "-v") == 0);
        return;
    }
    /* `lora [<n>] a[nnounce]` — replay every buffered announce, then the radio
     * check, now rather than at the next beat. The beat restarts from here. */
    if (cliVerb(cmd, "announce", 1)) { cliAnnounce((int)idx); return; }

    if (strcmp(cmd, "tx") == 0 || strcmp(cmd, "tx_psa") == 0 || strcmp(cmd, "tx_prot") == 0) {
        cliManualTx(idx, cmd, cliRest(args, 2));
        return;
    }

    /* The hex comes off the original line rather than the token array: the
     * tokeniser holds four and truncates at 80 characters, and a bundled
     * ANNOUNCE2 vector is longer than that. */
    if (cliVerb(cmd, "supe", 4)) {
        cliSupe((int)idx, nt > 2 ? tok[2] : nullptr, cliRest(args, 3));
        return;
    }

    if (nt < 3) { cliPrintf("usage: lora %ld <freq|bw|sf|cr|txp|preamble|sync|mode|lbt|appc|rx_boosted_gain> <value>\n", idx); return; }
    const char* val = tok[2];

    /* Human units in: frequency MHz, bandwidth kHz. Storage stays in Hz. */
    if (strcmp(cmd, "freq") == 0) {
        double mhz = atof(val);
        storageSet(sk(kb, sizeof kb, idx, "frequency"), (int)(mhz * 1.0e6));
        cliPrintf("lora/%ld freq = %.3f MHz\n", idx, mhz);
    } else if (strcmp(cmd, "bw") == 0) {
        double khz = atof(val);
        storageSet(sk(kb, sizeof kb, idx, "bandwidth"), (int)(khz * 1.0e3));
        cliPrintf("lora/%ld bw = %.0f kHz\n", idx, khz);
    } else if (strcmp(cmd, "sf") == 0) {
        storageSet(sk(kb, sizeof kb, idx, "spreading_factor"), atoi(val));
        cliPrintf("lora/%ld sf = %d\n", idx, atoi(val));
    } else if (strcmp(cmd, "cr") == 0) {
        storageSet(sk(kb, sizeof kb, idx, "coding_rate"), atoi(val));
        cliPrintf("lora/%ld cr = 4/%d\n", idx, atoi(val));
    } else if (strcmp(cmd, "txp") == 0) {
        storageSet(sk(kb, sizeof kb, idx, "tx_power"), atoi(val));
        cliPrintf("lora/%ld txp = %d dBm\n", idx, atoi(val));
    } else if (strcmp(cmd, "preamble") == 0) {
        storageSet(sk(kb, sizeof kb, idx, "preamble"), atoi(val));
        cliPrintf("lora/%ld preamble = %d\n", idx, atoi(val));
    } else if (strcmp(cmd, "sync") == 0) {
        storageSet(sk(kb, sizeof kb, idx, "sync_word"), val);
        cliPrintf("lora/%ld sync = %s\n", idx, val);
    } else if (strcmp(cmd, "mode") == 0) {
        storageSet(sk(kb, sizeof kb, idx, "mode"), val);
        cliPrintf("lora/%ld mode = %s\n", idx, val);
    } else if (strcmp(cmd, "lbt") == 0) {
        int on = atoi(val) != 0;
        storageSet(sk(kb, sizeof kb, idx, "lbt"), on);
        cliPrintf("lora/%ld lbt = %s\n", idx, on ? "on (carrier-sense before tx)" : "off (blind tx)");
    } else if (strcmp(cmd, "appc") == 0) {
        int on = atoi(val) != 0;
        storageSet(sk(kb, sizeof kb, idx, "appc"), on);
        cliPrintf("lora/%ld appc = %s\n", idx,
                  on ? "on (contention window banded by own airtime)"
                     : "off (exponential backoff on collisions)");
        if (on && !storageGetInt(sk(kb, sizeof kb, idx, "lbt"), 1))
            cliPrintf("        note: inert while lbt = 0\n");
    } else if (strcmp(cmd, "rx_boosted_gain") == 0) {
        int on = atoi(val) != 0;
        storageSet(sk(kb, sizeof kb, idx, "rx_boosted_gain"), on);
        cliPrintf("lora/%ld rx_boosted_gain = %s (SX126x only)\n", idx,
                  on ? "on (boosted, +~0.4 mA RX)" : "off (power saving)");
    } else {
        cliPrintf("unknown: lora %ld %s (try freq|bw|sf|cr|txp|preamble|sync|mode|lbt|appc|rx_boosted_gain)\n", idx, cmd);
    }
}

#endif  /* CONFIG_LORA0_CS_PIN */
