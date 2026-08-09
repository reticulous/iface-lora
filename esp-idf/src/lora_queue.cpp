/**
 * lora_queue — implementation. Plain arrays and plain free(): the blocks come
 * from ITS's packet links or from the bridge's own mallocs, and whoever ends
 * up with one calls free() (see itsRecvRef/itsSendOwned in its.h).
 */
#include "lora_queue.h"

#include <stdlib.h>
#include <string.h>

void loraqInit(LoraQueue* q) {
    memset(q, 0, sizeof *q);
}

bool loraqAccepting(const LoraQueue* q) {
    return q->n < LORAQ_CAP;
}

static void loraqRemove(LoraQueue* q, uint8_t i) {
    free(q->e[i].bytes);
    for (uint8_t k = (uint8_t)(i + 1); k < q->n; k++) q->e[k - 1] = q->e[k];
    q->n--;
}

bool loraqPush(LoraQueue* q, uint8_t* bytes, uint16_t len, uint32_t now_ms,
               uint16_t peer_id, const uint8_t* tag3, uint8_t refs, uint8_t flags) {
    if (!bytes || len == 0 || q->n >= LORAQ_CAP) return false;
    /* The per-peer cap: FIFO order is preserved for everyone else; the
     * congested peer loses its oldest, which is the packet Reticulum's
     * layers above have been waiting on longest and will retry first. */
    if (peer_id != LORAQ_PEER_NONE) {
        uint8_t held = 0, oldest = 0;
        bool haveOldest = false;
        for (uint8_t i = 0; i < q->n; i++) {
            if (q->e[i].peer_id != peer_id) continue;
            held++;
            if (!haveOldest) { oldest = i; haveOldest = true; }
        }
        if (held >= LORAQ_PEER_CAP && haveOldest) {
            loraqRemove(q, oldest);
            q->dropsPeerCap++;
        }
    }
    LoraPkt* p = &q->e[q->n++];
    p->bytes         = bytes;
    p->len           = len;
    p->first_seen_ms = now_ms;
    p->peer_id       = peer_id;
    if (tag3) {
        p->tag[0] = tag3[0]; p->tag[1] = tag3[1]; p->tag[2] = tag3[2];
        flags = (uint8_t)(flags | LORAQ_F_HAVE_TAG);
    } else {
        p->tag[0] = p->tag[1] = p->tag[2] = 0;
    }
    p->refs          = refs ? refs : 1;
    p->flags         = flags;
    return true;
}

LoraPkt* loraqAt(LoraQueue* q, uint8_t i) {
    return i < q->n ? &q->e[i] : (LoraPkt*)0;
}

void loraqConsume(LoraQueue* q, uint8_t i) {
    if (i >= q->n) return;
    if (q->e[i].refs > 1) { q->e[i].refs--; return; }
    loraqRemove(q, i);
}

void loraqFlush(LoraQueue* q) {
    for (uint8_t i = 0; i < q->n; i++) free(q->e[i].bytes);
    q->n = 0;
}
