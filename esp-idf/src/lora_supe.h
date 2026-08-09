#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

#include "supe_engine.h"

/* lora_supe — the ESP-IDF boundary around the pure engine: the one recursive
 * mutex every entry point takes (the engine itself is single-threaded by
 * contract and lock-free), the esp_timer that carries host->schedule, the
 * SupeHost implementation over the radio/queue/peers/airtime modules, and the
 * announce beat. The contexts the boundary covers: the radio task (RX/TX-done,
 * the drain, polling), the esp_timer task (the engine's step timer), the
 * console task (CLI), and config callbacks. */
struct SupeState {
    SemaphoreHandle_t  lock;
    SupeEngine         eng;
    esp_timer_handle_t timer;
    bool               engineTx;     /* the frame on the air is the engine's */
    /* the announce beat (SUPE.md §7) — platform-paced, engine-built */
    uint32_t annNextMs;
    bool     annPending;
    uint32_t annTryMs;
};

bool     supeInit(LoraRadio* r);            /* alloc + configure; false = no memory */
void     supeOnRadioStop(LoraRadio* r);
bool     supeReady(const LoraRadio* r);
bool     supeBusy(const LoraRadio* r);      /* a transaction owns the radio */
bool     supeHoldsRadio(const LoraRadio* r);
void     supeLock(LoraRadio* r);
void     supeUnlock(LoraRadio* r);
void     supeOnFrame(LoraRadio* r, const uint8_t* f, size_t len,
                     int16_t rssi, int16_t snr10);
void     supeOnPacketRx(LoraRadio* r, int16_t rssi, int16_t snr10);
bool     supeAfterTx(LoraRadio* r);         /* true = the engine dealt with the radio */
uint8_t  supeHeadVerdict(LoraRadio* r);
void     supePoll(LoraRadio* r);
uint32_t supeNextDeadlineMs(LoraRadio* r);
void     supeTagAdd(LoraRadio* r, const uint8_t* addr, bool perm, uint32_t ttlMs);
void     supeTagRelease(LoraRadio* r, const uint8_t* addr);
void     supeProofRetFile(LoraRadio* r, const uint8_t phash[16], const uint8_t node4[4]);
void     supeAnnArm(LoraRadio* r);
void     supeAnnCancel(LoraRadio* r);
uint8_t  supeOwnFamily(const LoraRadio* r);
SupeCaps supeOwnCaps(const LoraRadio* r);
