#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

/* Regime 1's airtime enforcement (SUPE.md §14.4). The transmit path reads a
 * precomputed verdict per channel; the ring behind it is credited at
 * transmit-done and recomputed on its own beat, never on the transmit path.
 *
 * This is a second structure beside `txAir`, and deliberately: Rolling1h is six
 * ten-minute buckets, so a node could spend the budget late in one bucket, have
 * it age out, and spend it again — approaching twice the cap inside a true hour.
 * `txAir` measures and this enforces, and neither is asked to do the other's
 * job. */
#define SUPE_RING_BUCKET_MS  10000
#define SUPE_RING_BUCKETS      360           /* an hour of them */

/* The channel state, one per radio: ms on air per bucket per channel, the
 * precomputed transmit verdict, and when each frequency was last used (the
 * reuse gap). Owned by lora_airtime; the engine asks, transmit-done records. */
struct ChanLedger {
    uint16_t ring[SUPE_CH_MAX][SUPE_RING_BUCKETS];   /* ms on air per bucket */
    uint32_t ringBucket;             /* absolute bucket the ring is aligned to */
    bool     chanOk[SUPE_CH_MAX];    /* the precomputed verdict */
    uint32_t chanLastTxMs[SUPE_CH_MAX];  /* the frequency-reuse gap */
    uint32_t verdictNextMs;
    /* The verdict beat runs only while it can change a verdict: with agile
     * airtime in the window it ticks per bucket; once the window drains — or
     * with no cap to enforce — it parks, and the first detour transmit re-arms
     * it. An idle SUPE node holds no standing wake for this. */
    bool     beatOn;
};

bool     airtimeInit(LoraRadio* r);
void     airtimeRecord(LoraRadio* r, uint8_t chan, uint32_t ms);
bool     airtimeMayI(LoraRadio* r, uint8_t chan, uint32_t* budgetRefusals);
bool     airtimePoll(LoraRadio* r);
uint32_t airtimeNextDeadlineMs(const LoraRadio* r, uint32_t now);
bool     airtimeAnyBudget(const LoraRadio* r);
