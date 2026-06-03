/**
 * lora — RadioLib LoRa transport task.
 *
 * Drives any RadioLib LoRa chip (SX126x, SX127x/RFM9x, SX128x, LR11x0, LR2021;
 * per-radio Kconfig) via RadioLib + a custom ESP-IDF HAL. The chip's IRQ line
 * notifies this task; task-side reads IRQ status, drains FIFO, reassembles
 * RNode-framed packets (1-byte header, seq nibble + SPLIT flag, ≤254 B payload
 * per frame, ≤2 frames per RNS packet), forwards to rnsd. Self-registers with
 * rnsd as the lora interface.
 *
 * See docs/component-plan.md §4 / §5.2 / §7.
 */
#pragma once

void loraInit(void);
