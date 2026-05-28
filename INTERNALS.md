# reticulous-lora — internals

## Task

One transport task. Single `itsPoll(deadline)` wait point. Wake
sources: ITS messages, DIO1 task notification from the ISR, computed
deadlines (RX-timeout, listen-before-talk window).

## DIO1 ISR

The SX1262's `DIO1` line signals interrupt events (TX-done,
RX-done, CRC error, header valid, …). Our HAL installs an ISR on the
pin that does **only**:

```c
BaseType_t hpw = pdFALSE;
vTaskNotifyGiveFromISR(loraTaskHandle, &hpw);
portYIELD_FROM_ISR(hpw);
```

All event decoding happens in the task on wakeup. **Never** call
`itsSend*` from the ISR — ITS queues are PSRAM-backed and not
ISR-safe.

## RadioLib + EspIdfHal

We use RadioLib for the SX1262 driver. `esp_idf_hal.cpp` is our own
RadioLib HAL implementation against ESP-IDF SPI, GPIO, and the FreeRTOS
notify API. The HAL is fully event-driven — no polling loops, no
`millis()`-busy-waits.

## RNode framing

On-air format matches Mark Qvist's RNode firmware: HDLC byte-stuffed,
KISS-shaped frames, with split-and-reassemble for packets above the
LoRa frame limit. Without this we wouldn't be wire-compatible with the
wider Reticulum LoRa ecosystem.

## Half-duplex coordination

LoRa is half-duplex: you cannot transmit while receiving and vice
versa. The task carries an explicit state machine:

- `IDLE` (continuous RX) → operator wants to TX → `TX_PENDING` (wait
  for current frame to finish, if any) → `TX` → DIO1 TX-done →
  back to `IDLE`.
- Optional listen-before-talk hold-off in the `TX_PENDING` state if a
  carrier is detected.

## SPI bus sharing

The T-Deck Plus shares its FSPI bus between display, SX1262, and the
SD card. The board HAL parks LCD + SX1262 CS HIGH before any other
peripheral drives MISO; `loraInit()` re-asserts the LoRa power pin
just in case (idempotent). The `spi_helper` from spangap-core arbitrates
in-flight transfers.

## Iface registration

Registers as `lora.<freq>.<sf>.<bw>` with rnsd. One iface per radio
(typical board has one SX1262).

## IFAC

Per-iface PSK in `s.lora.ifac_psk`, secret-mirrored to
`secrets.lora.ifac_psk`. Frames carry a per-packet HMAC; non-matching
frames are silently dropped by rnsd.
