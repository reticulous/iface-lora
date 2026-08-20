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

#include <stdint.h>

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

/** Point-in-time traffic snapshot for a small status surface, read straight
 *  off the radio's in-memory counters — deliberately NOT the storage stats
 *  keys, which publish only while a monitor UI holds them open
 *  (`uiTelemetryWanted`). Same advisory-display contract as
 *  lora_peer_summary: an unsynchronised cross-task read, not state to act
 *  on. A caller charting rates keeps its own previous sample and divides by
 *  its own wall-clock delta. */
struct lora_traffic_summary {
    bool     up;            /* radio on-air right now */
    uint64_t tx_bytes, rx_bytes;
    uint64_t tx_frames, rx_frames;
    int      airtime_pct;   /* own TX airtime over the appc window, percent */
    int      noise_dbm;     /* tracked channel noise floor, dBm */
    int      txp_dbm;       /* configured TX power, antenna dBm */
};

/** Fill `out` for radio slot `radio`. False when the slot is invalid or no
 *  radios are configured (counters read zero before the radio first runs). */
bool loraTrafficSummary(int radio, lora_traffic_summary* out);
