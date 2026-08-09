#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

/* ─────────────── LoRaMon: per-on-air-frame record ring ───────────────
 * One record per LoRa frame (so two per split RNS packet). Always recording
 * while a radio is up; the ring holds ~a busy hour. A LoRaMon viewer (web/LCD)
 * pulls the whole ring once over ITS (port LORAMON_PORT) and then follows live
 * frames via the ephemeral `lora.<n>.mon.*` storage keys. `dur_ms` is the
 * frame's computed time-on-air; `t_ms` its start on the monotonic ms clock. */
#define LORA_MON_CAP  4096          /* max published packet nodes per radio (FIFO backstop) */

/* Rolling one-hour airtime, 12 × 5-minute buckets per radio. The apps derive
 * every shorter window from the frame records themselves; only the hour — which
 * needs more history than a viewer may have been open for — is published. */
#define AIR_BUCKETS     12
#define AIR_BUCKET_MS   (5u * 60u * 1000u)

struct AirBucket { uint32_t absIdx; uint32_t rxMs, txMs; };

/* Telemetry state, one per radio, embedded in LoraRadio as `mon`. The
 * airtime rollups and drop counters are written by the radio task's recorder
 * (loraMonPush); the FIFO belongs to the interface task. Nothing outside
 * lora_mon touches any of it. */
struct LoraMonState {
    AirBucket  air[AIR_BUCKETS];     /* rolling one-hour airtime */
    Rolling1h  txAir[LORA_CH_MAX];   /* transmit seconds per channel, rolling hour */
    uint32_t*  pktMs;                /* FIFO of published packet start-ms (gp_alloc'd once) */
    uint16_t   pktCap, pktHead, pktCount;
    TickType_t rssiNext;             /* tick the next channel-RSSI sample is due */
    uint32_t   rssiDropped;          /* samples lost to a full interface queue */
    uint32_t   monDropped;           /* frame records lost to a full interface queue */
};

/* ─────────────── lora_mon: telemetry ─────────────── */
void loraMonPush(LoraRadio* r, uint8_t dir, uint32_t t_ms, uint16_t dur_ms,
                 uint16_t bytes, int16_t rssi, int16_t snr10, int8_t txp,
                 uint8_t type, uint16_t wait_ms, uint16_t own_ms);
void publishStats(LoraRadio* r);
void publishChannels(LoraRadio* r);
void publishState(LoraRadio* r, const char* state);
void rssiSamplePoll(LoraRadio* r);
void loraMonInit(LoraRadio* r);
void loraMonStart(void);
bool loraMonParked(void);
