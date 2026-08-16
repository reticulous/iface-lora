/**
 * lora_mon — telemetry: the radio→interface record queue, the LoRaMon
 * per-frame ring and its storage nodes, the stats flush, the channel-RSSI
 * series, and the interface task that owns every storage write.
 */
#include "lora_priv.h"

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
        r->mon.pktHead = (uint16_t)((r->mon.pktHead + 1) % r->mon.pktCap);
        r->mon.pktCount--;
    }
}

/* Publish one packet node `lora.<n>.packets.<ms>` holding a packed string:
 * "r|rssi|snr|dur|bytes|type|ch" (rx) or "t|txp|dur|bytes|type|wait|ch" (tx).
 * The leading token is the direction; snr is deci-dB; ch is the channel the
 * frame flew on, 0 being the reticulum hailing channel. Then age old nodes out.
 *
 * INTERFACE TASK ONLY — this is the storage half of a record. */
static void loraMonRecordOne(LoraRadio* r, const IfMsg* m) {
    if (!r->mon.pktMs || !r->mon.pktCap) return;
    char key[40], val[56];
    snprintf(key, sizeof key, "lora.%d.packets.%u", r->idx, (unsigned)m->t_ms);
    if (m->dir) snprintf(val, sizeof val, "t|%d|%u|%u|%u|%u|%u|%u",
                         (int)m->txp, (unsigned)m->dur_ms, (unsigned)m->bytes,
                         (unsigned)m->type, (unsigned)m->wait_ms, (unsigned)m->ch,
                         (unsigned)m->own_ms);
    else        snprintf(val, sizeof val, "r|%d|%d|%u|%u|%u|%u",
                         (int)m->rssi, (int)m->snr10, (unsigned)m->dur_ms,
                         (unsigned)m->bytes, (unsigned)m->type, (unsigned)m->ch);
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
void loraMonPush(LoraRadio* r, uint8_t dir, uint32_t t_ms, uint16_t dur_ms,
                        uint16_t bytes, int16_t rssi, int16_t snr10, int8_t txp,
                        uint8_t type, uint16_t wait_ms, uint16_t own_ms) {
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
}

/* Publish the channel list the regime puts in force:
 * `lora.<n>.chans` = "<freqHz>,<bwHz>|…", index = channel, 0 = hailing.
 *
 * One key rather than a subtree: it is a handful of numbers that only change on
 * a config apply, and the viewers want all of it at once to label their graphs.
 * With no agility in force it is just the hailing channel, so a viewer can tell
 * the two cases apart by the entry count alone and needs no separate flag. */
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
/* Measure the agile channels of the regime in force, as one excursion off the
 * hailing channel and back.
 *
 * Retune, read, retune home: standby → setFrequency → startReceive → getRSSI
 * per channel, then the same to come home. Nine channels is roughly 2–3 ms away
 * from the hailing channel — inside the ~4 ms an 8-symbol SF7/BW125 preamble
 * allows before a frame could be missed (plans/psa.md §3.5). The caller has
 * already established that nothing else wants the radio.
 *
 * **Bandwidth is deliberately not retuned.** Every channel is measured with the
 * receiver the hailing channel is configured for, so the readings share one
 * noise reference and are directly comparable — which is what a graph of nine
 * channels needs. Measuring each at its own width would make a 500 kHz channel
 * read ~6 dB hotter than a 125 kHz one from thermal noise alone, for no gain. A
 * regulatory Clear Channel Assessment is the opposite case and would have to
 * match the channel's occupied bandwidth; that is a different measurement for a
 * different purpose, and not this one. */
#if !defined(CONFIG_LORA_NO_SUPE)
static void rssiSweepAgile(LoraRadio* r, IfMsg* m, const RegimeChan* ch, int n) {
    for (int i = 0; i < n && i + 1 < LORA_CH_MAX; i++) {
        r->radio->standby();
        if (r->radio->setFrequency((float)ch[i].freqHz / 1.0e6f) != RADIOLIB_ERR_NONE)
            continue;                                  /* out of the part's range */
        if (radioStartRx(r) != RADIOLIB_ERR_NONE) continue;
        r->hal->delayMicroseconds(LORA_RSSI_SETTLE_US);
        float dbm = channelRssi(r);
        /* Below the thermal noise of any bandwidth this part can receive, so
         * not a measurement — it is the receiver answering before it is
         * listening. Leave the channel unreported rather than draw a floor
         * that isn't one. */
        if (dbm <= LORA_RSSI_INVALID_DBM) continue;
        m->chRssi[i + 1] = (int16_t)lround(dbm);
    }
    /* Home. Unconditional and unchecked: a failed retune above must not strand
     * the radio off the hailing channel, which is the one thing this must never
     * do. */
    r->radio->standby();
    r->radio->setFrequency((float)r->cfgFreqHz / 1.0e6f);
    /* The tracked noise floor is left alone: the sweep reads each channel
     * directly and never feeds the floor, and the radio returns to the channel
     * whose floor it already holds. */
    radioStartRx(r);
}
#endif  /* CONFIG_LORA_NO_SUPE */

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

#if !defined(CONFIG_LORA_NO_SUPE)
    int n = 0;
    const RegimeChan* ch = regimeChans(r->afa, &n);
    /* The field count is the regime's channel count whether or not every one of
     * them answered, so a viewer reads a stable set of columns and a channel
     * that failed to measure is an empty field rather than a shifted one. */
    m.nch = (uint8_t)((1 + n > LORA_CH_MAX) ? LORA_CH_MAX : 1 + n);
#else
    m.nch = 1;                       /* the hailing channel, and nothing else */
#endif

    /* The hailing channel first and in place — the radio is already on it and
     * settled, so this reading costs one transaction and no retune. */
    float hail = channelRssi(r);
    m.chRssi[LORA_CH_HAIL] = (int16_t)lround(hail);
    m.rssi = m.chRssi[LORA_CH_HAIL];

    /* Leaving the hailing channel mid-reception destroys the frame, and unlike
     * a preamble there is no partial-recovery argument. The reading just taken
     * is the cheapest available evidence that something is on air, so energy
     * above the tracked floor cancels this beat's excursion — the agile
     * channels simply go unreported and the viewers draw the gap.
     *
     * It is the same evidence carrier sense uses and carries the same blind
     * spot: a frame below the floor is invisible to it (§4.2 of plans/psa.md).
     * Closing that needs the preamble-detect and header-valid interrupts armed
     * during receive, which the receive path does not currently ask for. */
    bool quiet = hail <= r->noiseFloor + CSMA_RSSI_MARGIN_DB;
#if !defined(CONFIG_LORA_NO_SUPE)
    if (ch && n > 0 && quiet) rssiSweepAgile(r, &m, ch, n);
#else
    (void)quiet;
#endif

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
