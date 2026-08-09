#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

/* ── CSMA / listen-before-talk ──
 * Non-blocking DIFS + contention-window backoff, run from the task loop.
 * Carrier sense is instantaneous channel RSSI vs a tracked noise floor.
 *
 * Two backoff regimes share the sense and the state machine, selected per radio
 * by s.lora.<i>.appc (both require s.lora.<i>.lbt):
 *   appc=0  the constants below — a contention window that grows exponentially
 *           on each busy encounter and resets after every grant.
 *   appc=1  the APPC_* constants further down — RNode's regime, where the
 *           window is drawn from a band chosen by our own recent airtime. */
#define CSMA_CW_MIN          2       /* initial CW exponent → up to 2^2 = 4 slots */
#define CSMA_CW_MAX          6       /* CW ceiling → up to 2^6 = 64 slots */
#define CSMA_RSSI_MARGIN_DB  6.0f    /* dB above noise floor that reads as busy */
#define CSMA_NOISE_FLOOR_DBM (-105.0f)  /* initial noise-floor estimate */
#define CSMA_SLOT_MS_MIN     2       /* slot-time clamp (derived from symbol time) */
#define CSMA_SLOT_MS_MAX     25

/* ── APPC: adaptive p-persistent CSMA ──
 *
 * The acronym is ours, coined for this straddle — no upstream project uses it,
 * and it is imprecise: the mechanism is not p-persistent CSMA in the textbook
 * sense (Kleinrock & Tobagi 1975), which gates each transmit opportunity behind
 * a probability p. There is no coin flip here. What this implements — copied
 * from RNode firmware, which is where the numbers below come from — is an
 * *adaptive contention window*: the random backoff is drawn from one of four
 * bands, and the band is selected by how much of the recent past this radio
 * spent transmitting. Busier radio, higher band, longer expected backoff. The
 * effect is the load-responsive politeness p-persistence aims at, reached by
 * sizing the window instead of by rolling dice, so the name is a label for the
 * feature and not a description of the algorithm.
 *
 * The load input is our OWN transmit duty cycle, not observed channel
 * occupancy. That is RNode's choice too: its update_csma_parameters() reads
 * `airtime` (own time-on-air over the last two bins), not `total_channel_util`
 * (which adds carrier-detect samples and is only reported to the host). Every
 * radio on a congested channel is transmitting more — including its retries —
 * so own-airtime tracks aggregate load closely enough to act on, and it costs
 * no extra sensing: we already know each frame's time-on-air.
 *
 * Upstream reference (RNode_Firmware, Config.h CSMA Parameters,
 * update_csma_parameters(), tx_queue_handler(), add_airtime(), updateBitrate()).
 * Divergences from upstream are listed in INTERNALS.md §6a. */
#define APPC_SLOT_SYMBOLS        12      /* slot = 12 symbol times … */
#define APPC_SLOT_MAX_MS        100      /* … clamped to this ceiling … */
#define APPC_SLOT_MIN_MS         24      /* … and this floor … */
#define APPC_SLOT_MIN_FAST_DELTA 18      /* … which drops to 6 ms above the fast rate */
#define APPC_FAST_THRESHOLD_BPS  30000   /* nominal bitrate that counts as "fast" */
#define APPC_SIFS_MS              0      /* DIFS = SIFS + 2 slots */
#define APPC_CW_BANDS             4      /* number of contention bands */
#define APPC_CW_PER_BAND_WINDOWS 15      /* window count each band spans */
#define APPC_BAND_1_MAX_AIRTIME   7      /* ≤ this own-airtime % stays in band 1 */
#define APPC_BAND_N_MIN_AIRTIME  85      /* airtime % that maps to the top band */
#define APPC_BIN_MS            7500      /* airtime accounting bin (RNode: 3 ms × 2500) */
#define APPC_BINS               480      /* bins in the uptime hour (3600 s / 7.5 s) */
#define APPC_HOUR_MS        3600000

enum CsmaPhase : uint8_t { CSMA_IDLE, CSMA_DIFS, CSMA_BACKOFF };

/* ─────────────── lora_csma: medium access on the hailing channel ────────── */
void     csmaMediumHeld(LoraRadio* r, uint32_t durMs);
bool     csmaClear(LoraRadio* r);
void     csmaResetAccess(LoraRadio* r);
uint16_t csmaGrantWaitMs(const LoraRadio* r);
void     appcAddAirtime(LoraRadio* r, uint32_t durMs);
float    appcAirtime(const LoraRadio* r);
uint8_t  appcLiveBand(const LoraRadio* r);
