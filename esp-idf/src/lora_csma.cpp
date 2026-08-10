/**
 * lora_csma — medium access on the hailing channel: carrier sense against a
 * tracked noise floor, the DIFS + contention-window machine, and the APPC
 * (adaptive contention window) regime with its own-airtime accounting.
 */
#include "lora_priv.h"

#if defined(CONFIG_LORA0_CS_PIN)

/* Carrier sense: sample the channel and decide busy/free, tracking the noise
 * floor as the low envelope of RSSI (snap down fast, creep up slowly) so an
 * active channel can't inflate the reference it's compared against. Also busy
 * while a multi-frame reception is being reassembled (half-duplex). */
static bool channelBusy(LoraRadio* r) {
    if (r->splitPending) return true;
    /* Ask the demodulator before measuring power. It is the only party that can
     * see a frame arriving below the noise floor, which LoRa routinely does, and
     * the answer is one register read — the same order of cost as the sense
     * itself. This is the prospective half of what csmaMediumHeld corrects after
     * the fact: together they mean a neighbour's frame is neither transmitted
     * over nor credited to us as free medium. */
    if (radioRxInProgress(r)) return true;
    float rssi = channelRssi(r);
    /* `GetRssiInst` asked before the receiver is actually running answers
     * 0xFF, which decodes to −127.5 dBm — below the thermal noise of any
     * bandwidth this part receives, so it is the receiver answering before it
     * is listening, not a measurement (see LORA_RSSI_INVALID_DBM). It must not
     * touch the floor: one such reading pins the floor at −128 and every real
     * reading then compares as "busy" for the minutes the floor takes to creep
     * back — which is exactly a sense taken right after a transmit re-armed
     * receive. Read it as the quiet channel it superficially resembles and
     * learn nothing from it; the settle is microseconds and the next sample is
     * real. */
    if (rssi <= LORA_RSSI_INVALID_DBM) return false;
    if (rssi < r->noiseFloor) r->noiseFloor = rssi;
    else                      r->noiseFloor += 0.02f * (rssi - r->noiseFloor);
    /* A reception completed since the last sense (csmaMediumHeld). The medium
     * was held for that frame's whole time on air whether or not the threshold
     * below ever noticed, so report it busy once — that is what puts the
     * inter-frame space after the frame instead of over the top of it.
     *
     * One-shot, and only if the sense caught it fresh: past one DIFS the
     * required idle period has already elapsed as real idle time, and the
     * sample stands on its own. */
    if (r->rxHeldTick) {
        TickType_t difs = r->appc ? r->appcDifsTicks : r->difsTicks;
        bool fresh = (TickType_t)(xTaskGetTickCount() - r->rxHeldTick) < difs;
        r->rxHeldTick = 0;
        if (fresh) return true;
    }
    return rssi > r->noiseFloor + CSMA_RSSI_MARGIN_DB;
}

/* Forget the tracked floor and start again from the seed.
 *
 * The floor describes one channel measured through one receiver, so it survives
 * neither a retune nor a recalibration of the front end that measured it. It is
 * cheap to re-learn: the tracker snaps down to the first sample below it, so a
 * seed above the real floor is corrected on the very next sense, while the slow
 * creep upward is what stops an occupied channel from inflating its own
 * reference. Carrying a floor across a channel change is the expensive mistake —
 * a quiet band's floor makes a busy one read as permanently free, and a busy
 * band's makes a quiet one read as permanently busy. */
void csmaNoiseFloorReset(LoraRadio* r) {
    r->noiseFloor = CSMA_NOISE_FLOOR_DBM;
    for (int i = 0; i < LORA_CH_MAX; i++) r->chFloor[i] = CSMA_NOISE_FLOOR_DBM;
}

/* Change channel: park the floor the radio just learned and take up the one it
 * left on the channel it is moving to.
 *
 * The reasoning above says a floor must not be carried ACROSS channels, and it
 * still holds — this carries each channel's own floor forward instead, which is
 * the opposite operation. What made a plain reseed expensive is the asymmetry
 * the tracker is built on: a seed above the real floor is corrected by the next
 * sample, a seed below it is only walked off at 2% of the gap per sense. A node
 * that detours every second reseeds the hailing channel twice a second, and if
 * the channel rests above the seed — mid −90s against a −105 dBm seed is an
 * ordinary bench — every sense in between reads busy, the contention window
 * accrues nothing, and a transmit that was ready waits a few hundred ms for a
 * medium that was free the whole time.
 *
 * A channel never visited starts from the seed, which is what the reset leaves
 * behind for it. */
void csmaFloorSwitch(LoraRadio* r, uint8_t from, uint8_t to) {
    if (from < LORA_CH_MAX) r->chFloor[from] = r->noiseFloor;
    if (from == to) return;
    r->noiseFloor = to < LORA_CH_MAX ? r->chFloor[to] : CSMA_NOISE_FLOOR_DBM;
}

/* End of a reception — the channel-state half of receiving a frame.
 *
 * Carrier sense cannot be relied on to have noticed the transmission we just
 * decoded. LoRa demodulates below the noise floor (SF7 at −7.5 dB SNR), so a
 * frame received perfectly may never have risen above `noiseFloor +
 * CSMA_RSSI_MARGIN_DB`; and the sense is a point sample once per slot, not a
 * continuous watch, so even a strong frame can fall between two of them. The
 * receiver holds the one piece of evidence the sense lacks: it decoded
 * something, therefore the medium was occupied.
 *
 * Two corrections follow, and the second is the one that bites:
 *
 *  - The next sense reads busy, so the inter-frame space restarts from the end
 *    of the frame rather than from wherever the sampling happened to land.
 *
 *  - Under APPC the contention window is a wall-clock target accumulated
 *    *while the medium reads free*. A reception the sense missed is therefore
 *    not merely ignored — its whole duration is banked as credit toward our own
 *    transmission, so hearing a neighbour out makes us more eager rather than
 *    less. That credit is given back here. `appcCwStart` discriminates: non-zero
 *    means the accumulator was still running when the frame ended, so the sense
 *    never saw it and the credit is bogus; zero means a busy sense had already
 *    stopped the clock and there is nothing to return.
 *
 * Called for every frame off the air, CRC failures and our own air protocol
 * included: a frame that failed its CRC still occupied the channel, and a frame
 * we consumed ourselves was still somebody else transmitting. */
void csmaMediumHeld(LoraRadio* r, uint32_t durMs) {
    r->rxHeldTick = xTaskGetTickCount();
    if (r->rxHeldTick == 0) r->rxHeldTick = 1;      /* 0 is the "nothing to apply" sentinel */
    if (!r->appc || r->appcCwStart == 0) return;
    TickType_t held = pdMS_TO_TICKS(durMs);
    r->appcCwPassed = (r->appcCwPassed > held) ? (TickType_t)(r->appcCwPassed - held) : 0;
}

/* ── APPC: own-airtime accounting ──
 *
 * RNode buckets its own time-on-air into 7.5 s bins across a ring covering the
 * uptime hour, and reads the current plus previous bin as `airtime`, the figure
 * that picks the contention band. We keep only those two live bins: the rest of
 * the ring exists upstream to feed a long-term duty-cycle lock, which this
 * straddle does not implement. Bins are aligned to uptime, not to a rolling
 * window, so `airtime` covers between one and two bins of history — that is
 * upstream's behaviour and the band edges are calibrated against it. */
static void appcRollBins(LoraRadio* r, uint32_t nowMs) {
    uint32_t bin = (nowMs % APPC_HOUR_MS) / APPC_BIN_MS;
    if (bin == r->appcBinIdx) return;
    /* Adjacent bin → the old current ages into previous. Any longer gap means
     * both windows elapsed without a transmit of ours, so both are empty. */
    r->appcBinPrev = (bin == (r->appcBinIdx + 1) % APPC_BINS) ? r->appcBinCur : 0;
    r->appcBinCur  = 0;
    r->appcBinIdx  = bin;
}

/* Credit one transmitted frame's time-on-air. Called per frame at TxDone, where
 * the duration is already computed for the LoRaMon record. Sole writer of the
 * bins, and it runs on the radio task. */
void appcAddAirtime(LoraRadio* r, uint32_t durMs) {
    if (!r->appc) return;
    appcRollBins(r, millis());
    r->appcBinCur += durMs;
}

/* Fraction of the last two bins this radio spent transmitting. Ages the stored
 * bins into the present without writing them, so the CLI printer — which reads
 * live radio state from its own task — cannot race the radio task's accounting.
 * Bins older than the previous window read as empty, which is what they would
 * be rolled to on the next transmit. */
float appcAirtime(const LoraRadio* r) {
    uint32_t bin = (millis() % APPC_HOUR_MS) / APPC_BIN_MS;
    uint32_t cur = 0, prev = 0;
    if (bin == r->appcBinIdx)                        { cur = r->appcBinCur; prev = r->appcBinPrev; }
    else if (bin == (r->appcBinIdx + 1) % APPC_BINS) { prev = r->appcBinCur; }
    return (float)(cur + prev) / (2.0f * (float)APPC_BIN_MS);
}


/* Arduino map(), reproduced so the band edges land exactly where upstream's do:
 * integer arithmetic throughout, division truncating toward zero. */
static int appcMap(int x, int inMin, int inMax, int outMin, int outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/* Band for an own-airtime percentage. Upstream feeds map() the percentage with
 * APPC_BAND_1_MAX_AIRTIME added *and* uses the same value as the input floor,
 * which cancels out to a plain 0-based scale — reproduced verbatim rather than
 * simplified, so a future upstream change to either constant stays a one-line
 * diff. With today's numbers the edges are: ≤7 % band 1, 8–38 % band 2,
 * 39–77 % band 3, ≥78 % band 4. */
static uint8_t appcBandFor(int airtimePct) {
    int band;
    if (airtimePct <= APPC_BAND_1_MAX_AIRTIME) band = 1;
    else band = appcMap(airtimePct + APPC_BAND_1_MAX_AIRTIME,
                        APPC_BAND_1_MAX_AIRTIME, APPC_BAND_N_MIN_AIRTIME,
                        2, APPC_CW_BANDS);
    if (band > APPC_CW_BANDS) band = APPC_CW_BANDS;
    if (band < 1) band = 1;
    return (uint8_t)band;
}

/* The band the airtime right now selects — what a window drawn now would use.
 * appcBand holds the band of the window currently *in flight*, which is what
 * the contention stall warning wants; telemetry and the CLI want this. */
uint8_t appcLiveBand(const LoraRadio* r) {
    return appcBandFor((int)(appcAirtime(r) * 100.0f));
}

/* Draw this frame's contention window from the band our airtime puts us in.
 * Bands partition a single 0..(bands×windows−1) ladder, so band 1 draws 0–13,
 * band 2 15–28, band 3 30–43, band 4 45–58 — upstream's random(min,max) excludes
 * the top of each band, and so does this. */
static void appcDrawWindow(LoraRadio* r) {
    int pct  = (int)(appcAirtime(r) * 100.0f);
    r->appcBand = appcBandFor(pct);
    int cwMin = (r->appcBand - 1) * APPC_CW_PER_BAND_WINDOWS;
    int cwMax = r->appcBand * APPC_CW_PER_BAND_WINDOWS - 1;
    r->appcCw = cwMin + (int)(esp_random() % (uint32_t)(cwMax - cwMin));
    r->appcCwTarget = (TickType_t)r->appcCw * r->appcSlotTicks;
}

/* Advance the APPC channel-access machine one sense. Same contract as
 * csmaClear(): true only on the pass the medium is granted.
 *
 * The window is a wall-time target, accumulated only while the medium reads
 * free — a busy sense restarts DIFS from scratch but *freezes* rather than
 * discards the accumulated backoff, so a frame contending on a loaded channel
 * keeps its progress and cannot be starved indefinitely by neighbours. The
 * window is redrawn only after a grant, never widened on a busy encounter: all
 * adaptation lives in the band. */
static bool csmaClearAppc(LoraRadio* r, bool prime) {
    TickType_t now = xTaskGetTickCount();
    if (now == 0) now = 1;                          /* 0 is the "not started" sentinel */
    bool free_ = !channelBusy(r);

    if (r->appcCw < 0) appcDrawWindow(r);

    if (r->appcDifsStart == 0) {                    /* DIFS not begun */
        r->csmaPhase = CSMA_DIFS;
        if (free_) r->appcDifsStart = now;
        return false;
    }
    if (!free_) {                                   /* medium taken → restart DIFS */
        r->csmaPhase = CSMA_DIFS;
        r->appcDifsStart = 0;
        r->appcCwStart   = 0;                       /* stop counting; keep what passed */
        return false;
    }
    if ((TickType_t)(now - r->appcDifsStart) < r->appcDifsTicks) return false;

    r->csmaPhase = CSMA_BACKOFF;
    if (r->appcCwStart == 0) { r->appcCwStart = now; return false; }
    r->appcCwPassed += (TickType_t)(now - r->appcCwStart);
    r->appcCwStart   = now;
    if (r->appcCwPassed < r->appcCwTarget) return false;

    /* Ready, but the caller is priming rather than transmitting: leave the
     * window drained and the phase where it is, so the sense goes on running
     * (a busy medium still restarts the DIFS below) and the first real call
     * after the wait lifts grants on its own first pass. */
    if (prime) return false;

    r->appcCwPassed  = 0;                           /* granted → next frame redraws */
    r->appcCw        = -1;
    r->appcDifsStart = 0;
    r->appcCwStart   = 0;
    r->csmaPhase     = CSMA_IDLE;
    return true;
}

/* Advance the channel-access state machine. Returns true only on the tick the
 * medium is granted (DIFS observed idle, then a random backoff drained without
 * the channel going busy). Otherwise updates state and returns false; the
 * caller leaves the frame queued and nextDeadline() re-wakes at the next slot.
 * cw grows on every busy encounter (exponential backoff) and resets after a
 * grant; with appc set, the window comes from the airtime band instead. */
static bool csmaAdvance(LoraRadio* r, bool prime) {
    if (!r->lbt) return true;                       /* LBT off → blind transmit */

    if (r->appc) {
        /* csmaStart drives the shared stall warning and lbt_timeout valve, so
         * it is stamped here for both regimes. */
        if (r->csmaPhase == CSMA_IDLE) r->csmaStart = xTaskGetTickCount();
        return csmaClearAppc(r, prime);
    }

    TickType_t now = xTaskGetTickCount();
    bool busy = channelBusy(r);

    switch (r->csmaPhase) {
        case CSMA_IDLE:
            /* New frame: begin an inter-frame (DIFS) listen. */
            r->csmaCw = CSMA_CW_MIN;
            r->csmaPhase = CSMA_DIFS;
            r->csmaStart = now;                     /* start the lbt_timeout clock */
            r->csmaDifsStart = busy ? 0 : now;      /* 0 = free window not begun */
            return false;

        case CSMA_DIFS:
            if (busy) { r->csmaDifsStart = 0; return false; }   /* restart on activity */
            if (r->csmaDifsStart == 0) r->csmaDifsStart = now;
            if ((TickType_t)(now - r->csmaDifsStart) < r->difsTicks) return false;
            /* DIFS observed idle → draw a backoff in [0, 2^cw) slots. */
            r->csmaBackoff = (int)(esp_random() & ((1u << r->csmaCw) - 1));
            r->csmaPhase = CSMA_BACKOFF;
            if (r->csmaBackoff == 0 && !prime) { r->csmaPhase = CSMA_IDLE; return true; }
            r->csmaSlotDeadline = now + r->slotTicks;
            return false;

        case CSMA_BACKOFF:
            if (busy) {                             /* lost the medium → widen, re-listen */
                if (r->csmaCw < CSMA_CW_MAX) r->csmaCw++;
                r->csmaPhase = CSMA_DIFS;
                r->csmaDifsStart = 0;
                return false;
            }
            if (r->csmaBackoff <= 0) {             /* drained while priming */
                if (prime) return false;
                r->csmaPhase = CSMA_IDLE;
                return true;
            }
            if ((int32_t)(now - r->csmaSlotDeadline) < 0) return false;  /* slot not up */
            r->csmaSlotDeadline = now + r->slotTicks;
            if (--r->csmaBackoff <= 0 && !prime) { r->csmaPhase = CSMA_IDLE; return true; }
            return false;
    }
    return true;
}

/* Advance the machine on behalf of a frame that is queued but not yet allowed
 * to fly — held by our own timing rather than by anyone's reservation. Sensing
 * is a read, and the radio is listening anyway, so the inter-frame space and
 * the contention window are served during the wait instead of after it. The
 * grant itself is withheld: the state stops one step short, keeps sensing (a
 * medium that goes busy still restarts the DIFS, exactly as it would for a
 * frame about to fly), and the first csmaClear after the wait lifts takes it.
 *
 * Never call this for a SUPE_V_HOLD: that medium is reserved by somebody else's
 * GRANT, and contending underneath a reservation is the one thing the
 * reservation exists to stop. */
void csmaPrime(LoraRadio* r) {
    if (!r->lbt) return;
    (void)csmaAdvance(r, /*prime=*/true);
}

bool csmaClear(LoraRadio* r) {
    return csmaAdvance(r, /*prime=*/false);
}

/* Abandon any channel-access attempt in progress: the next frame contends from
 * scratch. Used when the queue drains, when a frame is shed by the lbt_timeout
 * valve, and when the probe takes or releases the radio. Under appc this also
 * discards the frozen backoff — upstream would carry it into the next frame,
 * but here the machine is shared by three producers and stale progress must not
 * leak from one to another. */
void csmaResetAccess(LoraRadio* r) {
    r->csmaPhase     = CSMA_IDLE;
    r->appcCw        = -1;
    r->appcCwPassed  = 0;
    r->appcCwStart   = 0;
    r->appcDifsStart = 0;
}

/* How long the channel access that just granted the medium took, for the
 * LoRaMon wait mark. Valid only on the pass csmaClear() returned true — both
 * regimes stamp csmaStart on an attempt's first sense and leave it alone until
 * the next. With LBT off nothing was sensed.
 *
 * Every frame that wins the medium for itself reports through this, rather
 * than reading `txWaitMs`: that field belongs to the outbound drain, and a
 * frame that took the channel on its own terms would otherwise report whatever
 * the last ordinary packet left behind — a stale bar on the first such frame
 * and none at all on the rest. */
uint16_t csmaGrantWaitMs(const LoraRadio* r) {
    if (!r->lbt) return 0;
    uint32_t ms = (uint32_t)(xTaskGetTickCount() - r->csmaStart) * portTICK_PERIOD_MS;
    return (uint16_t)(ms > 0xFFFF ? 0xFFFF : ms);
}

#endif  /* CONFIG_LORA0_CS_PIN */
