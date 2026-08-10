/**
 * lora_queue — the one seam between the bridge and the radio. What arrives is
 * enqueued; what transmits is dequeued; nothing else passes. Holds packets as
 * heap blocks it owns, refcounted, with a per-peer cap and a global cap
 * (plans/structuring-lora-code.md §4).
 *
 * Pure: no ESP-IDF, RadioLib or FreeRTOS — host-compilable by test/.
 * Single-threaded by contract; the caller serialises access.
 *
 * Backpressure has two levels, because the congestion is per-peer and the
 * signal to rnsd is one bit:
 *   1. per-peer cap → drop that peer's oldest. Our decision, invisible to
 *      rnsd. Reticulum tolerates loss.
 *   2. global cap → stop accepting (loraqAccepting false). The one bit rnsd
 *      sees: its send toward us blocks briefly, then it warns and drops
 *      (rns/esp-idf/src/rnsd.cpp, iface out).
 *
 * Deliberately absent from LoraPkt, each for a reason (§4): no modulation,
 * channel or power — not known at enqueue, decided at negotiation for a whole
 * train; no expiry — first_seen_ms is the fact, age limits are read by
 * whoever looks; no type — SUPE control frames never enter the queue.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Global cap. rnsd drops after a bounded stall once we stop accepting, so the
 * cap is sized to make that rare: a train's worth of packets plus headroom for
 * a burst arriving while one is out. */
#define LORAQ_CAP        16
/* Per-peer cap: one congested peer may not displace everyone else's traffic. */
#define LORAQ_PEER_CAP    8
#define LORAQ_PEER_NONE  0xFFFF

/* flags: origin in the low bits (values match LORA_ORIG_*), plus markers. */
#define LORAQ_ORIG_RNSD   0x00
#define LORAQ_ORIG_RNODE  0x01
#define LORAQ_ORIG_MASK   0x03
#define LORAQ_F_REPLAY    0x04   /* a buffered announce being replayed: goes out
                                  * verbatim and is never re-recorded */
#define LORAQ_F_HAVE_TAG  0x08   /* `tag` holds the packet's first-address prefix */

struct LoraPkt {
    uint8_t*  bytes;           /* heap block we own; free() when refs hits 0 */
    uint16_t  len;
    uint32_t  first_seen_ms;   /* the only timestamp; everything else is policy */
    uint16_t  peer_id;         /* index into the peer table; LORAQ_PEER_NONE if unknown */
    uint8_t   tag[3];          /* first three bytes of the first address field,
                                * computed at ingress by the observer — what the
                                * engine classifies on without parsing Reticulum */
    uint8_t   refs;            /* destinations counted at ingress */
    uint8_t   flags;           /* LORAQ_* */
};

struct LoraQueue {
    LoraPkt  e[LORAQ_CAP];     /* FIFO — e[0] is the head */
    uint8_t  n;
    uint32_t dropsPeerCap;     /* packets shed by the per-peer cap */
};

void loraqInit(LoraQueue* q);

/** The one bit of global backpressure: false once the queue is full. */
bool loraqAccepting(const LoraQueue* q);

/** Enqueue a heap block. Ownership transfers on true; on false the caller
 *  still owns `bytes`. The per-peer cap is applied here: at the cap, the
 *  peer's oldest queued packet is dropped to make room. */
bool loraqPush(LoraQueue* q, uint8_t* bytes, uint16_t len, uint32_t now_ms,
               uint16_t peer_id, const uint8_t* tag3, uint8_t refs, uint8_t flags);

/** Peek entry i (0 = head); null past the tail. The queue keeps ownership. */
LoraPkt* loraqAt(LoraQueue* q, uint8_t i);

/** One destination is done with entry i: refs--, removed and freed at zero. */
void loraqConsume(LoraQueue* q, uint8_t i);

/** Drop everything (the radio went down under the queue). */
void loraqFlush(LoraQueue* q);

static inline uint8_t loraqDepth(const LoraQueue* q) { return q->n; }
