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

#include "service.h"

class LoraService : public Service {
public:
    void onInit() override;
};
