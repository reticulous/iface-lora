/**
 * lora_cli — the `lora` command tree: slot status, the neighbour printer,
 * `lora supe`, the announce replay trigger, config setters, and the manual
 * transmit machinery (tx / tx_psa / tx_prot) it drives on the radio task.
 */
#include "lora_priv.h"
#include "lora_fem.h"   /* femName / rfCalName, for the front-end line in the slot dump */

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

    /* Addressed at nobody — a bare header with a body that is never heard — so
     * it records as a broadcast, which is what "no addressee" means on the air. */
    loraMonPush(r, 1 /*tx*/, r->txFrameStartMs, hdrMs,
                (uint16_t)bestL, 0, 0, r->cfgTxp, LORA_PKT_OURS, 0, 0,
                LMD_NONE, nullptr, LMC_BCAST);
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
        if (r->splitPending || r->txActive
#if !defined(CONFIG_LORA_NO_SUPE)
            || supeHoldsRadio(r)
#endif
           ) { manualTxFinish(r, false, "radio busy"); return; }
        /* Settings are still moving. Refuse rather than hold: this is a typed
         * one-shot, and a transmit that happens ten seconds later on different
         * parameters is not the one that was asked for. */
        if (loraCfgQuiet()) {
            manualTxFinish(r, false, "settings still settling — try again in a moment");
            return;
        }

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

/* Service a pending `lora forget`, on the task that owns the peer table. It is
 * a table edit and nothing else — no radio state, no air — so unlike a manual
 * transmit it refuses nothing and waits for nothing: a radio that is down still
 * holds a table, and forgetting is exactly what someone asks of a radio they
 * have just reconfigured. */
void peersForgetPoll(LoraRadio* r) {
    if (!r->fgtReq) return;
    r->fgtReq = false;
    r->fgtResN = peersForget(r, r->fgtNum);
    r->fgtResGen = r->fgtResGen + 1;      /* release the CLI's poll loop */
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
    /* The front end, when there is one: which part answered (or that the board
     * simply declared one), the antenna ceiling it sets, and — on a chip that
     * switches its own front end — the DIO map that does it. On a board with no
     * FEM there is nothing here to read, so the line is skipped. */
    if (r->femType != FEM_NONE)
        cliPrintf("        fem=%s  band=%s  rx +%d dB\n",
                  femName((LoraFemType)r->femType),
                  r->highBand ? "2.4GHz" : "sub-GHz", r->cal.rxGainDb);
    /* The range this board reaches at the antenna connector, and how well that
     * is known. An uncalibrated radio says so rather than presenting an
     * identity conversion as a characterisation. */
    cliPrintf("        antenna %d..%d dBm  cal=%s\n",
              r->minTxDbm, r->maxTxDbm, rfCalName(r->cal.grade));
    if (chipFamily(s->chip) == FAM_LR2021)
        cliPrintf("        lr2021 irq=DIO%d  rfsw idle/rx/tx/rx_hf/tx_hf="
                  "%02x/%02x/%02x/%02x/%02x (bit 0 = DIO5)\n",
                  s->lr_irq_dio, s->lr_rfsw[0], s->lr_rfsw[1], s->lr_rfsw[2],
                  s->lr_rfsw[3], s->lr_rfsw[4]);

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
    /* The chip is not asked: a register read from the CLI task can land in the
     * middle of a RadioLib transaction the radio task is holding CS across.
     * These are the values that were applied to it — and only the ones this
     * part answers to, so the line never implies a control the chip ignores. */
    LoraFamily fam = chipFamily(s->chip);
    if (radioHasRxBoost(fam)) {
        bool on = storageGetInt(sk(kb, sizeof kb, i, "rx_boosted_gain"), 1) != 0;
        cliPrintf("        rx_boosted_gain=%d%s", on,
                  r->cal.rxGainDb > 0 ? " (ignored: external amplifier in the RX path, chip's off)"
                  : fam == FAM_LR2021 ? on ? " (gain level 7 of 7)" : " (gain level 0 of 7)"
                                      : "");
        if (fam == FAM_SX126X)
            cliPrintf("  ocp=%.0f mA", (double)radioOcpMilliamps(s->chip));
        if (radioHasAgcReset(fam)) {
            int agc = storageGetInt(sk(kb, sizeof kb, i, "agc_reset"), LORA_AGC_RESET_DEF_S);
            if (agc > 0) cliPrintf("  agc_reset=%ds", agc);
            else         cliPrintf("  agc_reset=off");
        }
        cliPrintf("\n");
    }
    if (r->femType == FEM_KCT8103L) {
        bool lna = storageGetInt(sk(kb, sizeof kb, i, "fem_rx_lna"), 1) != 0;
        cliPrintf("        fem_rx_lna=%d (%s)\n", lna,
                  lna ? "front-end LNA in the RX path, ~8 mA" : "bypassed, power saving");
    }
    /* Not a setting — where the front end happens to be right now, which the
     * last power asked for decided. Worth showing because it is the difference
     * between the two halves of the published range. */
    if (femCanBypassPa(r))
        cliPrintf("        tx path: %s\n",
                  r->femTxPa ? "through the front-end PA"
                             : "round it — the chip drives the antenna");
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
    if (r->lbt) {
        /* Every wait this radio serves is decided against these two numbers, and
         * a floor sitting below where the channel actually rests reads as a busy
         * medium for as long as it takes to creep back — a wait nothing on the
         * air explains. Printed per channel, since each is contended separately
         * and only the one in force answers for the wait happening now. */
        cliPrintf("        noise floor %+.0f dBm on ch%u (busy above %+.0f)",
                  (double)r->noiseFloor, (unsigned)r->chNow,
                  (double)(r->noiseFloor + CSMA_RSSI_MARGIN_DB));
#if !defined(CONFIG_LORA_NO_SUPE)
        int n = 0;
        regimeChans(r->afa, &n);
        for (int c = 1; c <= n && c < LORA_CH_MAX; c++)
            cliPrintf("  ch%d %+.0f", c, (double)r->chFloor[c]);
#endif
        cliPrintf("\n");
    }
    cliPrintf("        rx %u/%uB  tx %u/%uB  rssi %d dBm  snr %d dB  crc_err %u  split_to %u\n",
              (unsigned)r->rxFrames, (unsigned)r->rxBytes,
              (unsigned)r->txFrames, (unsigned)r->txBytes,
              (int)r->rssiLast, (int)r->snrLast,
              (unsigned)r->crcErr, (unsigned)r->splitTimeouts);
    /* The transmit-to-receive turn as this board actually makes it: end of
     * our frame on the air to the receiver armed again. The far end's answer
     * must not start before it (SUPE_FLIP_MS). */
    if (r->flipN)
        cliPrintf("        flip tx->rx %u ms max, %u ms avg over %u frames\n",
                  (unsigned)r->flipMaxMs, (unsigned)(r->flipSumMs / r->flipN),
                  (unsigned)r->flipN);
    /* And the other direction: end of a frame received to the start of the
     * engine's answer to it. The far end's deadlines cover SUPE_TURNAROUND_MS
     * of this. */
    if (r->ansN)
        cliPrintf("        answer rx->tx %u ms max, %u ms avg over %u answers\n",
                  (unsigned)r->ansMaxMs, (unsigned)(r->ansSumMs / r->ansN),
                  (unsigned)r->ansN);
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

/* A link still worth calling open. Nothing announces a teardown, so silence
 * past NEI_LINK_QUIET_MS is the only evidence there is that a session is over
 * — and a listing that counts one as open is describing a neighbourhood that
 * has moved on. The row stays in the table either way: it is what a frame
 * recorded an hour ago resolves through. */
static bool linkIsOpen(const NeiLink* L, uint32_t now) {
    return (uint32_t)(now - L->lastMs) <= NEI_LINK_QUIET_MS;
}

/* The link as this radio has measured it: one line per direction that has a
 * path loss — the loss, the signal-to-noise of the frame it was read from, and
 * the power that frame went out at in both units (§15.1). The loss leads
 * because it is the link's own property whatever either end transmits at; the
 * SNR says whether the link is weak or merely quiet; the power is what the loss
 * was measured against. `us->them` is the peer's own report of how our frame
 * landed, `them->us` a frame heard here against the power the peer stated for
 * it, and a direction nobody has measured has no line — only a SUPE peer states
 * a power, and without one a level is not a loss.
 *
 * With no loss at all, the level this radio read is the whole of what is known
 * and is printed instead; `always` adds it beside a loss too (that is `-v`).
 *
 * `pad` is what each line is indented with: the listing tucks these under a
 * node's block, `lora probe` prints them at the margin as the whole of its
 * answer. One rendering either way, so the two surfaces cannot come to describe
 * the same measurement differently.
 *
 * `since` (0 = no filter) drops any reading taken before that millis(): a
 * caller reporting on something it just did shows what that produced and
 * nothing else, since a loss from ten minutes ago printed under a probe reads
 * as the probe's own answer. Such a caller owns the raw-level line too — it
 * has a better one — so the fallback belongs to the unfiltered form.
 *
 * Returns whether anything was printed, which is what a caller with a fallback
 * of its own needs to know. */
static bool cliPrintLink(const Neighbor* e, uint32_t now, const char* pad, bool always,
                         uint32_t since = 0) {
    bool printed = false;
    char ago[16], pw[16];
    int      lossFrom = 0;
    uint32_t fromMs   = 0;
    int16_t  snrFrom  = 0;
    int8_t   peerTxp  = 0;
    bool     haveFrom = peersLossFrom(e, &lossFrom, &fromMs, &snrFrom, &peerTxp);
    if (since) {
        if (haveFrom && (int32_t)(fromMs - since) < 0) haveFrom = false;
    }
    if (e->haveApRpt && !(since && (int32_t)(e->apRptMs - since) < 0)) {
        cliAgo(ago, sizeof ago, now, e->apRptMs);
        cliPrintf("%sus->them %d dB path loss, SNR %.0f dB @ tx %+d dBm (%s), %s ago\n",
                  pad, (int)e->apRptTxp - (int)e->apRptRssi,
                  (double)e->apRptSnr10 / 10.0, (int)e->apRptTxp,
                  fmtPower(e->apRptTxp, pw, sizeof pw), ago);
        printed = true;
    }
    if (haveFrom) {
        cliAgo(ago, sizeof ago, now, fromMs);
        cliPrintf("%sthem->us %d dB path loss, SNR %.0f dB @ tx %+d dBm (%s), %s ago\n",
                  pad, lossFrom, (double)snrFrom / 10.0, (int)peerTxp,
                  fmtPower(peerTxp, pw, sizeof pw), ago);
        printed = true;
    }
    if (!since && (always || (!haveFrom && !e->haveApRpt)) && e->haveSig) {
        cliAgo(ago, sizeof ago, now, e->lastHeardMs);
        cliPrintf("%slast heard @ %d dBm / SNR %.1f dB, %s ago\n",
                  pad, (int)e->rssiLast, (double)e->snrLast10 / 10.0, ago);
        printed = true;
    }
    return printed;
}

/* `local` selects which half of the table this pass prints: the neighbourhood,
 * or this device's own rows. They are two different subjects and the walk hands
 * them out in one sequence, so the filter is here. */
struct CliPrintCtx { LoraRadio* r; uint32_t now; bool verbose; bool local; };

static void cliPrintNode(Neighbor* e, int num, void* ud) {
    CliPrintCtx* c = (CliPrintCtx*)ud;
    if (peersIsLocal(e) != c->local) return;
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
            /* The local rows are built from announces we were heard sending and
             * are never retired, so this table remembers an address long after
             * it stops being ours — an LXMF account handed to a proxy server is
             * the case in point. The heading says "us", so ask what is actually
             * hosted rather than what was once transmitted. */
            if (c->local && e->isUs && !rnsdHostsDest(nd->hash)) continue;
            const char* asp = nd->haveName ? rnsNameLabel(nd->nameHash) : nullptr;
            bool isTransport = asp && strcmp(asp, "rnstransport.probe") == 0;
            if ((pass == 0) != isTransport) continue;
            loraHex(hex, nd->hash, 16);
            char nh[21] = "";
            if (nd->haveName && !asp) loraHex(nh, nd->nameHash, 10);
            cliPrintf(RNSD_PEER_ROW_FMT "%s %s", first ? lbl : "", hex,
                      asp ? asp : (nd->haveName ? nh : "-"));
            if (nd->name[0]) cliPrintf("  \"%s\"", nd->name);
            /* When it was last heard is not detail — it is what says whether a
             * row is a neighbour or a memory, so it rides every row here as it
             * does on every other medium's. -v adds the count behind it. */
            cliAgo(ago, sizeof ago, c->now, nd->lastMs);
            if (c->verbose && nd->announces) cliPrintf("  ann %u", (unsigned)nd->announces);
            cliPrintf("  %s ago\n", ago);
            first = false;
        }
    }
    /* Hashes a linkage frame said mean this node — a link identifier above all
     * — which we have never heard announced. Only the first four bytes were
     * ever on the air, hence the ellipsis. A timed-out one is left out: this is
     * a picture of the neighbourhood as it is, and a link that has been silent
     * past NEI_LINK_QUIET_MS is over. The row itself stays in the store, where
     * a frame recorded an hour ago still resolves through it. */
    for (int l = 0; ; l++) {
        uint8_t h4[4];
        if (!peersHashAt(c->r->nei, e, l, h4)) break;
        NeiHash* h = peersHashFind(c->r->nei, h4, 4);
        if (h && h->timedOut) continue;
        cliPrintf(RNSD_PEER_ROW_FMT "%02x%02x%02x%02x........................ (link)\n",
                  first ? lbl : "", h4[0], h4[1], h4[2], h4[3]);
        first = false;
    }
    if (first) {   /* nothing but a bare node key */
        if (e->haveNode4)
            cliPrintf(RNSD_PEER_ROW_FMT "%02x%02x%02x%02x........................ (not seen yet)\n",
                      lbl, e->node4[0], e->node4[1], e->node4[2], e->node4[3]);
        else
            cliPrintf(RNSD_PEER_ROW_FMT "(no hash seen)\n", lbl);
    }

    /* The identity prefixes this node answers to, for a SUPE node only — what
     * turns a SUPE log line back into a node with a name. Only for a SUPE node:
     * for anyone else these name a protocol they do not speak, and printing
     * them would be noise on every row. */
#if !defined(CONFIG_LORA_NO_SUPE)
    if (!peersIsLocal(e) && e->supeSeen) {
        /* Only what cannot already be read off the block above: a destination's
         * tag IS the first six characters of the hash printed there, and a link
         * gets a line of its own. What is left is the identities, which appear
         * nowhere else and are what a HAIL names its SENDER by.
         *
         * Labelled `ident` rather than `tags` because it is not one. A TAG is an
         * address prefix — a destination hash or a link id — since that is what
         * a packet carries and what the protocol classifies on. An IDENT names
         * the node itself, and a caller uses it because its own addresses are
         * plural and it cannot know which of them the far end has heard. */
        uint8_t tg[NEI_IDS_MAX + 1][3];
        int ni = 0;
        auto known = [&](const uint8_t* b) {
            for (int i2 = 0; i2 < ni; i2++)
                if (memcmp(tg[i2], b, 3) == 0) return true;
            for (int d = 0; d < e->nDests; d++)
                if (memcmp(e->dests[d].hash, b, 3) == 0) return true;
            return false;
        };
        if (e->haveNode4 && !known(e->node4)) memcpy(tg[ni++], e->node4, 3);
        for (int k = 0; k < e->nIds && ni < (int)(sizeof tg / sizeof tg[0]); k++)
            if (!known(e->ids[k])) memcpy(tg[ni++], e->ids[k], 3);
        if (ni) {
            cliPrintf(RNSD_PEER_ROW_FMT "ident", "");
            for (int i2 = 0; i2 < ni; i2++)
                cliPrintf(" %02x%02x%02x", tg[i2][0], tg[i2][1], tg[i2][2]);
            cliPrintf("\n");
        }
    }
#endif

    if (!peersIsLocal(e)) {
        /* Capability line. TRANSPORT means it forwards for others; ROAMING is
         * its node-flags bit; the mesh tag that it speaks our air protocol; TX
         * the power a probe settled on for it; EST what we would ask IT to
         * transmit at; USE the power we last transmitted to it at. */
        char f[128];
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
#if !defined(CONFIG_LORA_NO_SUPE)
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
#endif
        /* What we would ask this node to transmit at — the power request's own
         * number. Only where the request is a thing this radio sends: it rides
         * our air protocol, so a radio not speaking it asks nobody anything,
         * and a node outside the protocol would not parse it. An estimate
         * driving nothing is not worth a tag. */
        int est10;
        if (apEnabled(c->r) && e->ourProto &&
            peersEstimateCliff10(c->r, e, c->now, &est10, nullptr, nullptr)) {
            char t[24];
            snprintf(t, sizeof t, "EST %.0f", (double)est10 / 10.0);
            add(t);
        }
        /* USE is the power a frame to this node goes out at. With the
         * controller running it is what the last one was derived at, and the
         * tilde says how much of that was guessed: none for the peer's own
         * report of what it heard from us, one for a path loss measured the
         * other way round and assumed reciprocal; absent means the derivation
         * had nothing fresh to work from and the configured power stands.
         *
         * With the controller off — this radio not speaking the protocol —
         * there is no guessing to report and nothing stale to print: every
         * frame goes out at exactly the configured `tx_power`, so that is what
         * the tag says. The derived numbers a row still holds describe a
         * transmission this radio would no longer make. */
        char t[16];
        if (!apEnabled(c->r)) {
            snprintf(t, sizeof t, "USE %d", (int)c->r->cfgTxp);
            add(t);
        } else if (e->apSrc != AP_SRC_NONE) {
            snprintf(t, sizeof t, "USE %s%d",
                     e->apSrc == AP_SRC_REPORT ? "" : "~", (int)e->apPwr);
            add(t);
        }
        if (o) cliPrintf(RNSD_PEER_ROW_PAD "( %s )\n", f);

        cliPrintLink(e, c->now, RNSD_PEER_ROW_PAD, c->verbose);
    }

    if (c->verbose) {
        for (int n = 0; n < e->nIds; n++) {
            loraHex(hex, e->ids[n], 16);
            cliPrintf(RNSD_PEER_ROW_PAD "id:%s\n", hex);
        }
        if (e->haveSig) {
            /* The whole row's span, against the last-heard line's newest
             * reading above it — the age belongs to that one, so it is not
             * repeated here. */
            cliPrintf(RNSD_PEER_ROW_PAD "envelope  rssi %d..%d dBm  snr %.1f..%.1f dB\n",
                      (int)e->rssiMin, (int)e->rssiMax,
                      (double)e->snrMin10 / 10.0, (double)e->snrMax10 / 10.0);
        }
        if (e->haveQuality)
            cliPrintf(RNSD_PEER_ROW_PAD "q %u/255 (%u/%u proofs)%s\n",
                      (unsigned)e->quality, (unsigned)e->qProved, (unsigned)e->qSent,
                      e->provesData ? "  proves-data" : "");
        if (e->haveAdv)
            cliPrintf(RNSD_PEER_ROW_PAD "hashes %d/%u\n", peersKnownHashes(c->r->nei, e),
                      (unsigned)e->advHashes);
        uint32_t absNow = c->now / NEI_BUCKET_MS;
        uint32_t cnt = 0; int64_t rs = 0, ss = 0;
        for (int b = 0; b < NEI_BUCKETS; b++) {
            const NeiBucket* bk = &e->buck[b];
            if (bk->cnt && absNow - bk->absIdx < NEI_BUCKETS) {
                cnt += bk->cnt; rs += bk->rssiSum; ss += bk->snrSum10;
            }
        }
        if (cnt)
            cliPrintf(RNSD_PEER_ROW_PAD "1h: %u pkt  avg %d dBm %.1f dB\n",
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
    int nUs = 0, nRnode = 0, nLinks = 0;
    int nNodes = peersOtherCount(st);
    for (int k = 0; k < NEI_MAX; k++) {
        Neighbor* e = &st->nei[k];
        if (!e->used) continue;
        if (e->isRnode)   nRnode++;
        else if (e->isUs) nUs++;
    }
    for (int k = 0; k < NEI_LINKS_MAX; k++)
        if (st->links[k].used && linkIsOpen(&st->links[k], now)) nLinks++;

    char ago[16];
    cliAgo(ago, sizeof ago, now, st->sinceMs);
    cliPrintf("lora/%d neighbors: %d other%s, %d open link%s (observing %s)\n\n",
              i, nNodes, nNodes == 1 ? "" : "s",
              nLinks, nLinks == 1 ? "" : "s", ago);
    if (r->curIfacSize)
        cliPrintf("  note: ifac enabled — frames are masked, passive parse sees nothing\n\n");

    CliPrintCtx ctx = { r, now, verbose, /*local=*/false };
    peersWalk(st, -1, cliPrintNode, &ctx);
    if (!nNodes) cliPrintf("  (none heard yet)\n\n");

    /* This device's own rows last, under their own heading. They are what the
     * radio hears itself saying, not who is out there, and mixing them into the
     * numbered list invites reading `us` as a neighbour. There is at most one of
     * each, so neither is numbered — a number is something to aim the radio at,
     * and neither of these is addressable. */
    if (nUs || nRnode) {
        cliPrintf("this device%s:\n\n", nRnode ? " (and the attached RNode client)" : "");
        ctx.local = true;
        peersWalk(st, -1, cliPrintNode, &ctx);
    }

    if (!verbose) return;
    for (int k = 0; k < NEI_LINKS_MAX; k++) {
        NeiLink* L = &st->links[k];
        if (!L->used || !linkIsOpen(L, now)) continue;
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
#if !defined(CONFIG_LORA_NO_SUPE)
    cliPrintf("lora/%d repeating %d announce%s%s\n", idx, n, n == 1 ? "" : "s",
              supeReady(r) ? ", then this node's own SUPE announcement" : "");
#else
    cliPrintf("lora/%d repeating %d announce%s\n", idx, n, n == 1 ? "" : "s");
#endif
}

#if !defined(CONFIG_LORA_NO_SUPE)
/* `lora [<n>] supe` — everything a field report asks first, in one screen: which
 * dialect this build speaks and until when, what the interface resolved the
 * regime to, what the tag set has learned, what is being held, and the counters
 * that say whether anyone is answering.
 *
 * `lora [<n>] supe rx <hex>` injects a frame into the receive path exactly as if
 * the radio had decoded it. The hex is meant to come from the golden vectors
 * the host tests emit (esp-idf/test/golden.txt), never hand-written: that is
 * what keeps a one-device test and the codec from disagreeing. */
static int cliParseBytes(const char* s, uint8_t* out, size_t cap);

static void cliSupe(int idx, const char* sub, const char* arg) {
    LoraRadio* r = &s_radios[idx];
    SupeState* st = r->supe;

    /* `lora [<n>] supe enable|disable` — the setting, not a runtime toggle, so
     * it survives a reboot and reads the same in the settings pane. The radio
     * watches the key: flipping it re-announces at once, which is what tells
     * the neighbourhood to start or stop meeting this node rather than leaving
     * them to discover it by silence. */
    if (sub && (cliVerbIs(sub, "enable", 1) || cliVerbIs(sub, "disable", 1))) {
        bool on = cliVerbIs(sub, "enable", 1);
        char kb2[48];
        storageSet(sk(kb2, sizeof kb2, idx, "SUPE.enable"), on ? 1 : 0);
        cliPrintf("lora/%d SUPE.enable = %d — %s\n", idx, on ? 1 : 0,
                  on ? "this node speaks SUPE"
                     : "this node does NOT speak SUPE");
        /* Unconditionally, and everything — not only the SUPE frame. A neighbour
         * that has this node wrong is wrong about more than one bit: it may hold
         * no destination for us at all, and a SUPE announcement alone names
         * identities without saying what is reachable behind them. Running the
         * whole replay is also why repeating the command is useful rather than
         * idempotent — the second one is how you answer "did it hear me?". */
        supeAnnArm(r);      /* certain, even if nothing else is buffered to replay */
        cliAnnounce(idx);
        return;
    }

    if (sub && strcmp(sub, "rx") == 0) {
        if (!arg || !*arg) { cliPrintf("usage: lora %d supe rx <hex>\n", idx); return; }
        uint8_t f[SUPE_MAX_FRAME];
        int n = cliParseBytes(arg, f, sizeof f);
        if (n <= 0) { cliPrintf("bad hex\n"); return; }
        if (!supeMounted(r)) { cliPrintf("SUPE is not running on lora/%d\n", idx); return; }
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
    /* The switch is the wish; `supeOn` is the verdict. Naming the reason they
     * differ is the whole value of this line — a node whose pane says "enabled"
     * and whose monitor draws one graph is asking exactly this question — so
     * each gate is tested for rather than the first one being assumed. */
    const char* supeState = "off";
    if (r->supeOn)
        supeState = "on";
    else if (storageGetInt(sk(kb, sizeof kb, idx, "SUPE.enable"), 0)) {
        if (storageGetInt(sk(kb, sizeof kb, idx, "ifac_size"), 0))
            supeState = "off (an access code is configured)";
        else if (supeExpired(nowUnix))
            supeState = "off (the dialect this build speaks has expired)";
        else
            supeState = "off (enabled, but it did not come up — see the boot log)";
    }
    cliPrintf("lora/%d SUPE: %s\n", idx, supeState);
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
        cliPrintf("  we are      family %u, ceiling %u, max %d dBm\n",
                  (unsigned)c.fam, (unsigned)c.topStep, (int)c.maxPwrDbm);
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
    for (int i = 0; i < SUPE_SCHED_MAX; i++) {
        SupeSched* s = &e->sched[i];
        if (!s->used) continue;
        cliPrintf("  schedule    %02x%02x%02x %s, %u slots, next %u, we %s\n",
                  s->d.hash3[0], s->d.hash3[1], s->d.hash3[2],
                  s->wide ? "wide" : "narrow", (unsigned)s->d.nSlots,
                  (unsigned)s->nextSlot,
                  s->wide ? (s->weTx0 ? "tx even" : "tx odd")
                          : (s->weHailed ? "listen (our hail)" : "speak"));
    }
    cliPrintf("  rx          %u frames, %u discarded, %u for other meetings\n",
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
                           (int32_t)(en->absentUntilMs - now) > 0) ? ", HELD" : "");
            }
    }
    cliPrintf("  hails       %u out, %u hail-backs; %u unanswered; %u schedules taken\n",
              (unsigned)e->hailsOut, (unsigned)e->hailBacksOut,
              (unsigned)e->unanswered, (unsigned)e->schedsIn);
    cliPrintf("  slots       %u listened, %u spoken, %u skipped busy\n",
              (unsigned)e->slotsListened, (unsigned)e->slotsSpoken,
              (unsigned)e->slotsSkipped);
    cliPrintf("  meetings    %u completed; %u frames out, %u in; repairs %u out, %u in\n",
              (unsigned)e->meetingsDone, (unsigned)e->framesOut,
              (unsigned)e->framesIn, (unsigned)e->repairsOut,
              (unsigned)e->repairsIn);
    cliPrintf("  verdicts    %u waits on a held peer; %u packets dropped at patience\n",
              (unsigned)e->dropsHold, (unsigned)e->dropsPatience);
    {
        /* The last meeting ends, oldest first — the engine's own record, good
         * even when the debug log dropped the lines. */
        uint32_t nowMs = millis();
        int nEnds = (int)(sizeof e->ends / sizeof e->ends[0]);
        bool any = false;
        for (int i = 0; i < nEnds; i++) {
            const auto* er = &e->ends[(e->endsAt + i) % nEnds];
            if (!er->why) continue;
            if (!any) { cliPrintf("  last ends   (age role ch sent got/exp outcome)\n"); any = true; }
            cliPrintf("    %6lus %s ch%u %u %u/%u %s (%s)\n",
                      (unsigned long)((nowMs - er->endedMs) / 1000u),
                      er->listener ? "L" : "O", (unsigned)er->chan,
                      (unsigned)er->sent,
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
    if (e->m.phase != SUPE_M_IDLE)
        cliPrintf("  in flight   phase %u, tag %02x%02x%02x, ch%u budget %u (%s)\n",
                  (unsigned)e->m.phase, e->m.tag[0], e->m.tag[1], e->m.tag[2],
                  (unsigned)e->m.chan, (unsigned)e->m.budget,
                  e->m.listener ? "listening" : "opening");
    if (r->q.n) cliPrintf("  queued      %u packet%s\n",
                          (unsigned)r->q.n, r->q.n == 1 ? "" : "s");
}
#endif  /* CONFIG_LORA_NO_SUPE */


/* The verb's spelling and its abbreviations live in rnsd, so every interface
 * answers to the same word. What that word PRINTS here is this radio's own
 * table, which knows more per peer than the shared one can (node identities,
 * observed links, negotiated power). */
static bool cliIsNeighbors(const char* t) { return rnsdIsNeighborsVerb(t); }

/* `lora [<n>] p[robe] <num>` — one round trip to a neighbour, then what this
 * radio knows about it. The probe is `rnprobe`'s, to the node's
 * `rnstransport.probe` destination: a Reticulum packet with a delivery proof
 * behind it, which is the only way to measure a round trip at all.
 *
 * The two halves answer different questions and that is the point of putting
 * them together. The round trip says the node is reachable and how long the
 * whole path takes; the block under it says what the link itself is doing —
 * path loss each way, the power it took, how the last frame landed. And the
 * block is read AFTER the probe, so what it shows includes the probe's own
 * traffic: this is the one command that makes a measurement rather than
 * reporting the last one that happened by.
 *
 * The path is Reticulum's to choose, not this radio's. A direct neighbour
 * answers at one hop over the radio it was heard on — the narration says how
 * many — but a node this radio can hear whose path currently runs somewhere
 * else is probed over that path, and the round trip is that path's. */
static void cliProbe(int idx, const char* arg) {
    LoraRadio* r  = &s_radios[idx];
    NeiState*  st = r->nei;
    if (!arg || !*arg) { cliPrintf("usage: lora %d probe <num>\n", idx); return; }
    char* end = nullptr;
    long  v = strtol(arg, &end, 10);
    if (end == arg || *end || v <= 0) {
        cliPrintf("probe what? a node number from `lora %d n`\n", idx);
        return;
    }
    if (!st) { cliPrintf("lora/%d: no neighbours (the radio has never been up)\n", idx); return; }
    int num = (int)v;
    Neighbor* e = peersWalk(st, num, nullptr, nullptr);
    if (!e) {
        cliPrintf("lora/%d: no node %d — the numbers are `lora %d n`'s\n", idx, num, idx);
        return;
    }

    /* The transport probe destination and nothing else. It is the hash every
     * node has, it is the one that answers a probe by itself (PROVE_ALL), and
     * it is the line this listing puts first. Probing some other aspect of the
     * node would send a meaningless payload to an application and wait on a
     * proof that application decides whether to give. */
    uint8_t dest[16];
    bool have = false;
    for (int d = 0; d < e->nDests && !have; d++) {
        const NeiDest* nd = &e->dests[d];
        const char* asp = nd->haveName ? rnsNameLabel(nd->nameHash) : nullptr;
        if (asp && strcmp(asp, "rnstransport.probe") == 0) {
            memcpy(dest, nd->hash, 16);
            have = true;
        }
    }
    if (!have) {
        /* Name the row, because the likeliest reason to be reading this is that
         * the number now means a different node than it did a moment ago. The
         * numbers are positions in the listing: a row arriving or being folded
         * into another renumbers everything after it, and a node whose
         * identities have not been joined yet is exactly the row with
         * destinations and no transport probe among them. */
        char first[33] = "(no hash of its own)";
        if (e->nDests)          loraHex(first, e->dests[0].hash, 16);
        else if (e->haveNode4)  snprintf(first, sizeof first, "%02x%02x%02x%02x…",
                                         e->node4[0], e->node4[1], e->node4[2], e->node4[3]);
        cliPrintf("lora/%d: node %d is %s and has no rnstransport.probe address here.\n"
                  "        Either it has not announced one (a node answers probes only\n"
                  "        with s.rnsd.respond_to_probes), this radio has not heard that\n"
                  "        announce yet, or the numbers have moved since you read them —\n"
                  "        they are positions in `lora %d n`, not names.\n",
                  idx, num, first, idx);
        return;
    }

    char names[NEI_NAME_MAX * 3];
    peersNodeNames(e, names, sizeof names);
    char hex[33];
    loraHex(hex, dest, 16);
    cliPrintf("lora/%d probe node %d%s%s  %s\n", idx, num,
              names[0] ? " " : "", names, hex);

    /* The mark everything below is measured against: nothing older than the
     * moment this verb started is this verb's answer. */
    uint32_t t0  = millis();
    uint64_t tx0 = r->txFrames;

    /* rnsd narrates the probe as it goes and blocks this task until it
     * settles; the row may have been merged or retired underneath by then, so
     * the listing is resolved again rather than through the old pointer. */
    rnsd_probe_t res = {};
    rnsdProbe("rnstransport.probe", dest, 32, 15, &res);

    /* Nothing came back: the narration has already said so, and that IS the
     * answer. Printing what the radio knew beforehand under a probe that failed
     * invites it to be read as the probe's own result. */
    if (res.status != RNSD_DEST_STATUS_DELIVERED) return;

    /* Where the probe actually went. Reticulum sends on the path it holds, and
     * a neighbour this radio hears is not automatically the path to it: with
     * another interface between the two nodes — or a transport node relaying —
     * the packet takes that instead, and the round trip above is that path's
     * rather than this radio's. One hop is the direct case and needs no note.
     * `rnpath <hash>` says which way it went. */
    if (res.hops != 1)
        cliPrintf("note: %u hops — the probe did NOT take this radio's direct link.\n"
                  "      That round trip is another path's. `rnpath %s` says which\n"
                  "      way it went; anything below is this radio's own air.\n",
                  (unsigned)res.hops, hex);

    Neighbor* now = peersFindByDest(st, dest);
    if (!now) {
        cliPrintf("lora/%d: node %d is gone from the table\n", idx, num);
        return;
    }

    /* The path losses, where this probe is what produced them: a loss needs a
     * power the far end stated, so these appear for a peer inside the protocol
     * and are then milliseconds old. An older reading is left to `lora n`,
     * which dates what it prints. Each line carries the power its reading was
     * measured against, so the level at either end follows by subtraction and
     * needs no line of its own. */
    if (cliPrintLink(now, millis(), "", /*always=*/false, /*since=*/t0)) return;

    /* Nothing stated a power, so there is no loss to have — and this radio's
     * own two halves are the measurement: what the probe's frames radiated, and
     * how the answer came back. Neither needs the protocol, and both are this
     * probe's by construction. A probe that took another path transmitted
     * nothing here and was answered elsewhere, so neither half appears rather
     * than a stale one standing in. */
    bool ourTx = r->txFrames > tx0;
    bool heard = now->haveSig && (int32_t)(now->lastHeardMs - t0) >= 0;
    if (ourTx || heard) {
        char pw[16];
        if (ourTx) cliPrintf("sent at %+d dBm (%s)", (int)r->txPwrNow,
                             fmtPower(r->txPwrNow, pw, sizeof pw));
        if (heard) cliPrintf("%sheard back at %d dBm / SNR %.1f dB",
                             ourTx ? ", " : "",
                             (int)now->rssiLast, (double)now->snrLast10 / 10.0);
        cliPrintf("\n");
    }
}

/* `lora [<n>] f[orget] <num>|all` — drop a node from the table, by the number
 * `lora n` printed beside it. Everything about it goes: its addresses, its
 * links, its measurements, and whether it speaks our air protocol. Nothing is
 * kept "just in case" — a belief that survives a forget is exactly the belief
 * somebody is trying to correct, and the table is a claim about the
 * neighbourhood NOW, rebuilt from the next frame the node sends.
 *
 * The work happens on the radio task (peersForgetPoll), which owns the table.
 * The exception is a parked task — `rns stop` — where nothing else can be
 * walking it and waiting would only time out. */
static void cliForget(int idx, const char* arg) {
    LoraRadio* r = &s_radios[idx];
    if (!arg || !*arg) { cliPrintf("usage: lora %d forget <num>|all\n", idx); return; }
    int num = -1;                                   /* -1 = every node out there */
    if (!cliVerbIs(arg, "all", 1)) {
        char* end = nullptr;
        long  v = strtol(arg, &end, 10);
        if (end == arg || *end || v <= 0) {
            cliPrintf("forget what? a node number from `lora %d n`, or all\n", idx);
            return;
        }
        num = (int)v;
    }
    if (!r->nei) {
        cliPrintf("lora/%d: nothing to forget (the radio has never been up)\n", idx);
        return;
    }
    int n;
    if (!s_task || s_stop) {
        n = peersForget(r, num);
    } else {
        uint32_t gen = r->fgtResGen;
        r->fgtNum = num;
        r->fgtReq = true;
        xTaskNotifyGive(s_task);
        for (int i = 0; i < 100 && r->fgtResGen == gen; i++) delay(20);   /* ≤ 2 s */
        if (r->fgtResGen == gen) {
            cliPrintf("lora/%d forget: no result (timeout)\n", idx);
            return;
        }
        n = r->fgtResN;
    }
    if (n < 0) {
        cliPrintf("lora/%d: no node %d — the numbers are `lora %d n`'s\n", idx, num, idx);
        return;
    }
    cliPrintf("lora/%d forgot %d node%s\n", idx, n, n == 1 ? "" : "s");
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
        cliPrintf("%-*s round trip to a neighbour, then what this radio knows\n", CLI_HELP_COL, "lora [<n>] p[robe] <num>");
        cliPrintf("%-*s forget a neighbour, or every one — by its `n` number\n", CLI_HELP_COL, "lora [<n>] f[orget] <num>|all");
#if !defined(CONFIG_LORA_NO_SUPE)
        cliPrintf("%-*s repeat every announce, then our SUPE announcement\n", CLI_HELP_COL, "lora [<n>] a[nnounce]");
#else
        cliPrintf("%-*s repeat every announce this node has originated\n", CLI_HELP_COL, "lora [<n>] a[nnounce]");
#endif
        cliPrintf("%-*s freq MHz / bw kHz / sf / cr /\n",   CLI_HELP_COL, "lora <n> <param> <val>");
        cliPrintf("%-*s   txp dBm / preamble / sync / mode / lbt 0|1 / appc 0|1 /\n", CLI_HELP_COL, "");
        cliPrintf("%-*s   rx_boosted_gain 0|1 / fem_rx_lna 0|1\n", CLI_HELP_COL, "");
        cliPrintf("%-*s blind-transmit a payload (0x<hex> = raw bytes)\n", CLI_HELP_COL, "lora <n> tx <string>");
        cliPrintf("%-*s carrier-sense (as normal tx), then transmit\n", CLI_HELP_COL, "lora <n> tx_psa <string>");
        cliPrintf("%-*s emit a header committing receivers for <ms> (4/8)\n", CLI_HELP_COL, "lora <n> tx_prot <ms>");
#if !defined(CONFIG_LORA_NO_SUPE)
        cliPrintf("%-*s SUPE state: regime, expiry, tag set, schedules, counters\n", CLI_HELP_COL, "lora [<n>] supe");
        cliPrintf("%-*s speak SUPE here, or stop — announces the change\n", CLI_HELP_COL, "lora [<n>] s[upe] e[nable]|d[isable]");
        cliPrintf("%-*s inject a golden-vector frame into the receive path\n", CLI_HELP_COL, "lora [<n>] supe rx 0x<hex>");
#endif
        return;
    }
    if (cliIsNeighbors(tok[0])) {                           /* all radios */
        bool v = nt > 1 && strcmp(tok[1], "-v") == 0;
        for (int i = 0; i < kNumRadios; i++) cliPrintNeighbors(i, v);
        return;
    }
    /* `lora f[orget] all` with no index means every radio — there is no number
     * to be ambiguous about, and "all" said of a neighbourhood means all of it.
     * A number belongs to one radio's listing, so that form takes radio 0 like
     * every other index-less verb here. */
    if (cliVerbIs(tok[0], "forget", 1)) {
        const char* a = nt > 1 ? tok[1] : nullptr;
        if (a && cliVerbIs(a, "all", 1))
            for (int i = 0; i < kNumRadios; i++) cliForget(i, a);
        else
            cliForget(0, a);
        return;
    }
    if (cliVerbIs(tok[0], "probe", 1)) { cliProbe(0, nt > 1 ? tok[1] : nullptr); return; }
    if (cliVerbIs(tok[0], "announce", 1)) { cliAnnounce(0); return; }   /* no index → radio 0 */
#if !defined(CONFIG_LORA_NO_SUPE)
    if (cliVerbIs(tok[0], "supe", 1)) {                       /* likewise */
        cliSupe(0, nt > 1 ? tok[1] : nullptr, cliRest(args, 2));
        return;
    }
#endif  /* CONFIG_LORA_NO_SUPE */

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
    if (cliVerbIs(cmd, "probe", 1))  { cliProbe((int)idx, nt > 2 ? tok[2] : nullptr); return; }
    if (cliVerbIs(cmd, "forget", 1)) { cliForget((int)idx, nt > 2 ? tok[2] : nullptr); return; }
    /* `lora [<n>] a[nnounce]` — replay every buffered announce, then the radio
     * check, now rather than at the next beat. The beat restarts from here. */
    if (cliVerbIs(cmd, "announce", 1)) { cliAnnounce((int)idx); return; }

    if (strcmp(cmd, "tx") == 0 || strcmp(cmd, "tx_psa") == 0 || strcmp(cmd, "tx_prot") == 0) {
        cliManualTx(idx, cmd, cliRest(args, 2));
        return;
    }

    /* The hex comes off the original line rather than the token array: the
     * tokeniser holds four and truncates at 80 characters, and a bundled
     * ANNOUNCE vector is longer than that. */
#if !defined(CONFIG_LORA_NO_SUPE)
    if (cliVerbIs(cmd, "supe", 1)) {
        cliSupe((int)idx, nt > 2 ? tok[2] : nullptr, cliRest(args, 3));
        return;
    }
#endif  /* CONFIG_LORA_NO_SUPE */

    if (nt < 3) { cliPrintf("usage: lora %ld <freq|bw|sf|cr|txp|preamble|sync|mode|lbt|appc|rx_boosted_gain|fem_rx_lna|agc_reset> <value>\n", idx); return; }
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
        LoraFamily fam = chipFamily(s_radios[idx].slot->chip);
        cliPrintf("lora/%ld rx_boosted_gain = %s\n", idx,
                  on ? "on (boosted, +~0.4 mA RX)" : "off (power saving)");
        if (!radioHasRxBoost(fam))
            cliPrintf("        note: %s has no such control — the setting is inert here\n",
                      chipName(s_radios[idx].slot->chip));
        else if (s_radios[idx].cal.rxGainDb > 0)
            cliPrintf("        note: ignored while the board's external amplifier is in the "
                      "RX path — the chip's stays off; applies once that LNA is bypassed\n");
        else if (fam == FAM_LR2021)
            cliPrintf("        note: on this part it is a gain LEVEL — %s\n",
                      on ? "7 of 7, the most sensitive" : "0, power-saving");
    } else if (strcmp(cmd, "fem_rx_lna") == 0) {
        int on = atoi(val) != 0;
        storageSet(sk(kb, sizeof kb, idx, "fem_rx_lna"), on);
        cliPrintf("lora/%ld fem_rx_lna = %s\n", idx,
                  on ? "on (front-end LNA in the RX path, ~8 mA while listening)"
                     : "off (bypassed — ~8 mA less, ~20 dB less in front of the chip)");
        if (s_radios[idx].femType != FEM_KCT8103L)
            cliPrintf("        note: only a KCT8103L front end can switch its LNA out — "
                      "the setting is inert here\n");
    } else if (strcmp(cmd, "agc_reset") == 0) {
        int secs = atoi(val);
        if (secs < 0) secs = 0;
        storageSet(sk(kb, sizeof kb, idx, "agc_reset"), secs);
        if (secs) cliPrintf("lora/%ld agc_reset = %d s\n", idx, secs);
        else      cliPrintf("lora/%ld agc_reset = off\n", idx);
        if (secs && !radioHasAgcReset(chipFamily(s_radios[idx].slot->chip)))
            cliPrintf("        note: the latching front end this recalibrates is the "
                      "SX126x's — %s ignores the beat\n",
                      chipName(s_radios[idx].slot->chip));
    } else {
        cliPrintf("unknown: lora %ld %s (try freq|bw|sf|cr|txp|preamble|sync|mode|lbt|appc|rx_boosted_gain|fem_rx_lna|agc_reset)\n", idx, cmd);
    }
}

#endif  /* CONFIG_LORA0_CS_PIN */
