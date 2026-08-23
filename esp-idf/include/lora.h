/**
 * lora — RadioLib LoRa interface task.
 *
 * Drives any RadioLib LoRa chip (SX126x, SX127x/RFM9x, SX128x, LR11x0, LR2021;
 * per-radio Kconfig) via RadioLib + a custom ESP-IDF HAL. The chip's IRQ line
 * notifies this task; task-side reads IRQ status, drains FIFO, reassembles
 * split-framed packets (1-byte header, seq nibble + SPLIT flag, ≤254 B payload
 * per frame, ≤2 frames per RNS packet), forwards to rnsd. Each radio
 * self-registers with rnsd as its own interface lora/<slot>.
 */
#pragma once

#include <stddef.h>

#include "service.h"

class LoraService : public Service {
public:
    void onInit() override;
};

/* LoRaMon publishes each on-air frame as a storage node
 * `lora.<n>.packets.<ms>` = a packed string ("r|rssi|snr|dur|bytes" for rx,
 * "t|txp|dur|bytes" for tx; snr is deci-dB). Viewers (browser + LCD) read that
 * subtree directly — no accessor surface here. */

/** Point-in-time neighbour summary for a small status surface (e.g. a tinylcd
 *  page). Counts the in-memory neighbour table the way the `lora` CLI does —
 *  the same unsynchronised cross-radio-task read; values are advisory display
 *  data, not state to act on. rssi/snr10 are the last received frame's. */
struct lora_peer_summary {
    int peers;      /* other nodes heard (not us, not our rnode) */
    int links;      /* link rows observed open */
    int rssi;       /* dBm, last rx frame (0 when nothing received yet) */
    int snr10;      /* deci-dB, last rx frame */
};

/** Fill `out` for radio slot `radio`. False when the slot is invalid, no
 *  radios are configured, or the radio has never been up (no observations). */
bool loraPeerSummary(int radio, lora_peer_summary* out);

/** What to call the node a three-byte tag resolves to: its announced name(s),
 *  comma-joined, or "#N" — the number `lora n` prints — where it has none. Empty
 *  only where the tag names nobody known. `tag` is six lowercase hex characters,
 *  the form every SUPE log line and every LoRaMon record uses.
 *
 *  For an ON-DEVICE surface. A browser cannot call this and reads
 *  `lora.<n>.peers.*` instead, which carries the same mapping and more; anything
 *  inside this firmware should come here rather than parse that back, since the
 *  peer table is right there and the published copy is written only while a web
 *  reader says it is looking. Same unsynchronised cross-task read as
 *  loraPeerSummary: advisory display data, not state to act on. */
void loraNameForTag(int radio, const char* tag, char* out, size_t outLen);
