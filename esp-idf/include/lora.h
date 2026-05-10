/**
 * lora — LoRa (SX1262) transport task.
 *
 * Owns the SX1262 driver via RadioLib + custom ESP-IDF HAL. DIO1 ISR
 * notifies this task; task-side reads IRQ status, drains FIFO,
 * reassembles RNode-framed packets (1-byte header, seq nibble + SPLIT
 * flag, ≤254 B payload per frame, ≤2 frames per RNS packet), forwards
 * to rnsd. Self-registers with rnsd as the lora interface.
 *
 * See docs/component-plan.md §4 / §5.2 / §7.
 */
#pragma once

void loraInit(void);
