#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

#include "supe_engine.h"

/* lora_supe — the ESP-IDF boundary around the pure engine: the one recursive
 * mutex every entry point takes (the engine itself is single-threaded by
 * contract and lock-free), the esp_timer that carries host->schedule, the
 * SupeHost implementation over the radio/queue/peers/airtime modules, the
 * train buffers, and the announce beat. The contexts the boundary covers: the
 * radio task (RX/TX-done, the drain, polling), the esp_timer task (the
 * engine's step timer), the console task (CLI), and config callbacks. */
struct SupeState {
    SemaphoreHandle_t  lock;
    SupeEngine         eng;
    esp_timer_handle_t timer;
    bool               engineTx;     /* the frame on the air is the engine's */
    /* The meeting's trains (SUPE.md §8). Outgoing frames are held whole to the
     * close, because the repair round resends them byte for byte — a changed
     * byte is a changed checksum. Inbound frames are held for whole,
     * in-sequence delivery at the close: split halves must reach rnsd
     * adjacent, and a repaired frame takes its place, not the end. This is the
     * RAM commitment SUPE_TRAIN_MAX bounds. */
    uint8_t  txT[SUPE_TRAIN_MAX][1 + RNODE_MAX_PAYLOAD];
    uint16_t txTLen[SUPE_TRAIN_MAX];
    uint8_t* txTPkt[SUPE_TRAIN_MAX]; /* the queue heap block each frame was cut
                                      * from — how a delivered close consumes */
    uint8_t  txTCount;
    uint8_t  rxT[SUPE_TRAIN_MAX][1 + RNODE_MAX_PAYLOAD];
    uint16_t rxTLen[SUPE_TRAIN_MAX];
    int16_t  rxTRssi[SUPE_TRAIN_MAX];
    int16_t  rxTSnr10[SUPE_TRAIN_MAX];
    uint8_t  rxTCount;
    /* the announce beat (SUPE.md §9) — platform-paced, engine-built */
    uint32_t annNextMs;
    bool     annPending;
    uint32_t annTryMs;
    /* The coalescing window opened by a first-seen announce going on the air
     * (supeAnnSoon). Held apart from annPending because it is a deadline and
     * not a request: annPending means "send as soon as the channel is clear",
     * and arming that directly would put an ANNOUNCE2 immediately behind every
     * announce instead of one behind a burst of them. */
    bool     annSoonPend;
    uint32_t annSoonMs;
    /* next dialect-expiry re-check; rides passes other work causes, holds no
     * wake of its own */
    uint32_t expiryNextMs;
};

bool     supeInit(LoraRadio* r);            /* alloc + configure; false = no memory */
void     supeOnRadioStop(LoraRadio* r);
bool     supeMounted(const LoraRadio* r);
bool     supeReady(const LoraRadio* r);
bool     supeBusy(const LoraRadio* r);      /* the engine owns the radio */
/* Narrower: a meeting is actually under way, so the queue is the engine's to
 * walk. A seed merely armed is not this — nothing has been met yet. */
bool     supeXactLive(const LoraRadio* r);
uint16_t supeCargoPeer(const LoraRadio* r); /* whose cargo is arriving, if any */
bool     supeHoldsRadio(const LoraRadio* r);

/* The peer a meeting in progress is with, if there is one. Every frame a
 * meeting puts on air concerns that peer whatever its own bytes say — a THATSIT
 * names a schedule and a train frame names whichever destination the packet
 * inside it was for, so without this a single exchange with one node reads as a
 * scatter of unrelated addresses and unnamed control frames. */
bool     supeMeetingTag(const LoraRadio* r, uint8_t out[SUPE_TAG_LEN]);
void     supeLock(LoraRadio* r);
void     supeUnlock(LoraRadio* r);
void     supeOnFrame(LoraRadio* r, const uint8_t* f, size_t len,
                     int16_t rssi, int16_t snr10);
/* A non-SUPE frame arrived while the engine held the radio. True: it belongs
 * to the meeting's inbound train and was buffered — the receive path stops
 * here, and the frame reaches the ordinary delivery path at the meeting's
 * close, in sequence. False: ordinary traffic; the caller proceeds. */
bool     supeTrainCapture(LoraRadio* r, const uint8_t* frame, size_t len,
                          int16_t rssi, int16_t snr10);
bool     supeAfterTx(LoraRadio* r);         /* true = the engine dealt with the radio */
uint8_t  supeHeadVerdict(LoraRadio* r);
void     supePoll(LoraRadio* r);
uint32_t supeNextDeadlineMs(LoraRadio* r);
void     supeTagAdd(LoraRadio* r, const uint8_t* addr, bool perm, uint32_t ttlMs);
void     supeTagRelease(LoraRadio* r, const uint8_t* addr);
void     supeProofRetFile(LoraRadio* r, const uint8_t phash[16], const uint8_t node4[4]);
void     supeAnnArm(LoraRadio* r);
void     supeAnnCancel(LoraRadio* r);
/* An announce this interface had never put on air just went out — ours for a
 * destination we had not announced before, or somebody else's that we relayed.
 * Either way the picture the neighbourhood holds of the air has just changed,
 * and the frame that says which identities and capabilities are ours is worth
 * more now than at the next beat. Coalesced: the first such announce sets the
 * deadline and everything inside the window folds into the one frame — a boot
 * that announces six destinations, or a burst of relayed announces, costs one
 * ANNOUNCE2 and not six. */
void     supeAnnSoon(LoraRadio* r);
uint8_t  supeOwnFamily(const LoraRadio* r);
SupeCaps supeOwnCaps(const LoraRadio* r);
