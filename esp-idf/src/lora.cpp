/**
 * lora — LoRa (SX1262) transport task. Stub.
 *
 * Phase 3 work. RadioLib + custom HAL not yet wired in; this scaffold
 * lets main.cpp compile and the task start so we can verify lifecycle.
 */
#include "lora.h"
#include "diptych.h"
#include "ports.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "lora";

static TaskHandle_t s_task = nullptr;

static TickType_t nextDeadline(void)
{
    /* TODO: split-RX timeout, duty-cycle window, backoff timers. */
    return portMAX_DELAY;
}

static void loraTaskMain(void*)
{
    info("[%s] task up", TAG);

    /* TODO: watch s.lora.enable; on enable:
     *   - bring up SX1262 via RadioLib + EspIdfHal
     *   - register IRAM_ATTR ISR via radio.setPacketReceivedAction(...)
     *     ISR: vTaskNotifyGiveFromISR(loraTask)
     *   - configure freq / BW / SF / CR / TXP from s.lora.*
     *   - itsConnect("rnsd", RNSD_PORT_REGISTER, payload)
     *     name=lora, MTU=500, bitrate=computed-from-LoRa-params
     *   - main loop: on wake, getIrqStatus / clearIrqStatus / drain FIFO,
     *     RNode-frame assembly, forward to rnsd; outbound from rnsd:
     *     RNode-frame split, schedule TX, defer during split-RX. */

    for (;;) {
        itsPoll(nextDeadline());
    }
}

void loraInit(void)
{
    /* Slightly larger stack than other transports because of the LoRa
     * frame buffers (DRAM, ≤256 B per direction). */
    s_task = spawnTask(loraTaskMain, TAG, 6144, nullptr, 2, 0, STACK_PSRAM);
}
