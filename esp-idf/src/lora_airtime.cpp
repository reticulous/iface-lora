/**
 * lora_airtime — regime airtime enforcement (SUPE.md §14.4): the per-channel
 * ring credited at transmit-done, the precomputed transmit verdict, and the
 * frequency-reuse gap. The transmit path reads the verdict and nothing else.
 * The channel state lives in the ChanLedger this module owns; the engine asks
 * through airtimeMayI and never reads the ring.
 */
#include "lora_priv.h"

#if defined(CONFIG_LORA0_CS_PIN)

/* The ledger: allocated once (PSRAM, beside the peer table) and kept across
 * config cycles, so a toggle does not amnesty a spent budget. */
bool airtimeInit(LoraRadio* r) {
    if (r->chans) return true;
    ChanLedger* led = (ChanLedger*)gp_alloc(sizeof(ChanLedger));
    if (!led) {
        warn("lora/%d airtime: no memory for the channel ledger", r->idx);
        return false;
    }
    memset(led, 0, sizeof *led);
    led->ringBucket    = millis() / SUPE_RING_BUCKET_MS;
    led->verdictNextMs = millis();
    led->beatOn        = true;    /* the first recompute parks it if there is nothing to do */
    for (int c = 0; c < SUPE_CH_MAX; c++) led->chanOk[c] = true;
    r->chans = led;
    return true;
}

/* Age the ring to now, dropping whole buckets rather than sliding: each bucket
 * holds one 10-second slice and the window is the last 360 of them. */
static void airtimeRingRoll(ChanLedger* led, uint32_t now) {
    uint32_t b = now / SUPE_RING_BUCKET_MS;
    if (b == led->ringBucket) return;
    uint32_t gap = b - led->ringBucket;
    if (gap >= SUPE_RING_BUCKETS) {
        memset(led->ring, 0, sizeof led->ring);
    } else {
        for (uint32_t k = 1; k <= gap; k++) {
            uint32_t idx = (led->ringBucket + k) % SUPE_RING_BUCKETS;
            for (int ch = 0; ch < SUPE_CH_MAX; ch++) led->ring[ch][idx] = 0;
        }
    }
    led->ringBucket = b;
}

void airtimeRecord(LoraRadio* r, uint8_t chan, uint32_t ms) {
    ChanLedger* led = r->chans;
    if (!led || chan >= SUPE_CH_MAX) return;
    uint32_t now = millis();
    airtimeRingRoll(led, now);
    uint32_t idx = led->ringBucket % SUPE_RING_BUCKETS;
    uint32_t v = led->ring[chan][idx] + ms;
    led->ring[chan][idx] = (uint16_t)(v > 0xFFFF ? 0xFFFF : v);
    led->chanLastTxMs[chan] = now;
    /* Agile airtime into an empty window re-arms the parked verdict beat.
     * Hailing airtime never does: channel 0 carries no SUPE budget, so no
     * amount of it gives the beat a verdict to move. */
    if (!led->beatOn && chan != SUPE_CH_HAIL) {
        led->beatOn        = true;
        led->verdictNextMs = now + SUPE_RING_BUCKET_MS;
    }
}

/* Recompute every channel's verdict. Off the transmit path by construction —
 * the transmit path reads chanOk[] and nothing else.
 *
 * **The hailing channel carries no SUPE budget.** Channel 0 is the Reticulum
 * network's own frequency, chosen by the operator and governed by whatever that
 * network operates under; SUPE does not own it and may not impose a cap on it
 * (§3, §14). What governs airtime there is the interface's existing
 * listen-before-talk and contention band. So the ring defends the regime's own
 * channels and hailing airtime is accounted separately — two budgets, not one. */
static void airtimeRecompute(LoraRadio* r) {
    ChanLedger* led = r->chans;
    const SupeRegime* g = supeRegime(r->afa);
    uint32_t now = millis();
    airtimeRingRoll(led, now);
    led->verdictNextMs = now + SUPE_RING_BUCKET_MS;

    led->chanOk[SUPE_CH_HAIL] = true;
    if (!g || g->airtimeMaxMs == 0) {
        for (int ch = 1; ch < SUPE_CH_MAX; ch++) led->chanOk[ch] = true;
        /* No cap in force means no verdict can ever change: nothing for a
         * beat to do, so it parks outright. */
        led->beatOn = false;
        return;
    }
    /* The effective cap sits one bucket below the legal one: the verdict may be
     * a whole bucket stale, and that margin is exactly what the staleness can
     * spend. Set this way, coarse bins cannot cross the real cap. */
    uint32_t buckets = g->airtimeWinMs / SUPE_RING_BUCKET_MS;
    if (buckets > SUPE_RING_BUCKETS) buckets = SUPE_RING_BUCKETS;
    uint32_t cap = g->airtimeMaxMs > SUPE_RING_BUCKET_MS
                       ? g->airtimeMaxMs - SUPE_RING_BUCKET_MS : 0;
    uint32_t busyMs = 0;
    for (int ch = 1; ch < SUPE_CH_MAX; ch++) {
        uint32_t sum = 0;
        for (uint32_t k = 0; k < buckets; k++) {
            uint32_t idx = (led->ringBucket + SUPE_RING_BUCKETS - k) % SUPE_RING_BUCKETS;
            sum += led->ring[ch][idx];
        }
        busyMs += sum;
        bool was = led->chanOk[ch];
        led->chanOk[ch] = sum < cap;
        if (was != led->chanOk[ch] && logIsDebug(TAG))
            dbg("lora/%d supe: channel %d %s (%u ms of %u in window)", r->idx, ch,
                led->chanOk[ch] ? "back in budget" : "out of budget",
                (unsigned)sum, (unsigned)cap);
    }
    /* An empty window can only stay empty until a transmit, and every verdict
     * over it is already "in budget": park the beat, airtimeRecord re-arms it.
     * This is what lets an idle SUPE node stop ticking every bucket. */
    if (busyMs == 0) led->beatOn = false;
}

/* The verdict beat: recompute when due. True on the pass it fired, so the
 * engine can hang its own once-a-beat maintenance off it. */
bool airtimePoll(LoraRadio* r) {
    ChanLedger* led = r->chans;
    if (!led || !led->beatOn) return false;
    if ((int32_t)(millis() - led->verdictNextMs) < 0) return false;
    airtimeRecompute(r);
    return true;
}

/* Ms until the verdict beat is next due, for the task's wake computation.
 * A parked beat holds no deadline at all. */
uint32_t airtimeNextDeadlineMs(const LoraRadio* r, uint32_t now) {
    if (!r->chans || !r->chans->beatOn) return UINT32_MAX;
    int32_t rem = (int32_t)(r->chans->verdictNextMs - now);
    return rem > 0 ? (uint32_t)rem : 0;
}

/* May a detour use this channel right now? Budget, and the minimum gap before
 * returning to a frequency — a second, independent reason an escalation never
 * reuses a channel within a transaction. A budget refusal is counted into the
 * caller's counter, so the engine's stats stay the engine's. */
bool airtimeMayI(LoraRadio* r, uint8_t chan, uint32_t* budgetRefusals) {
    ChanLedger* led = r->chans;
    const SupeRegime* g = supeRegime(r->afa);
    if (!led || chan >= SUPE_CH_MAX) return false;
    if (!led->chanOk[chan]) {
        if (budgetRefusals) (*budgetRefusals)++;
        return false;
    }
    if (g && g->reuseGapMs && led->chanLastTxMs[chan]) {
        if (millis() - led->chanLastTxMs[chan] < g->reuseGapMs) return false;
    }
    return true;
}

/* Is any channel still inside its airtime budget? (The reuse gap is a
 * moment's wait; the budget is the hour — a refusal wants to know which.) */
bool airtimeAnyBudget(const LoraRadio* r) {
    if (!r->chans) return true;
    for (int c = 1; c < SUPE_CH_MAX; c++)
        if (r->chans->chanOk[c]) return true;
    return false;
}

#endif  /* CONFIG_LORA0_CS_PIN */
