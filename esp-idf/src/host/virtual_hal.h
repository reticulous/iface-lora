/**
 * VirtualHal — RadioLib's HAL over the GPIO shim and SIMesh's chip model.
 *
 * It stands where EspIdfHal stands on a board and answers the same questions:
 * pins through the shim, time through esp_timer, and an SPI transfer handed to
 * the slot's chip in the model (simradio.h) instead of to a bus. The model
 * drives DIO1 through the shim, so the driver's interrupt path is the one it
 * has on a board. The pin-mode, level and interrupt-trigger constants are the
 * same values EspIdfHal uses, because RadioLib stores them opaquely and hands
 * them back.
 */
#pragma once

#include <RadioLib.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#include "../lora_radio.h"
#include "simradio.h"

class VirtualHal : public RadioLibHal {
public:
    static constexpr uint32_t MODE_INPUT  = (uint32_t)GPIO_MODE_INPUT;
    static constexpr uint32_t MODE_OUTPUT = (uint32_t)GPIO_MODE_OUTPUT;

    static constexpr uint32_t LEVEL_LOW   = 0;
    static constexpr uint32_t LEVEL_HIGH  = 1;

    static constexpr uint32_t EDGE_RISING  = (uint32_t)GPIO_INTR_POSEDGE;
    static constexpr uint32_t EDGE_FALLING = (uint32_t)GPIO_INTR_NEGEDGE;

    VirtualHal(int slot, const LoraSlot* pins);

    void init() override {}
    void term() override {}

    /** There is no bus to fail to claim. */
    bool ready() const { return true; }

    void     pinMode(uint32_t pin, uint32_t mode) override;
    void     digitalWrite(uint32_t pin, uint32_t value) override;
    uint32_t digitalRead(uint32_t pin) override;
    void     attachInterrupt(uint32_t interruptNum, void (*cb)(void), uint32_t mode) override;
    void     detachInterrupt(uint32_t interruptNum) override;

    void          delay(unsigned long ms) override;
    void          delayMicroseconds(unsigned long us) override;
    unsigned long millis() override;
    unsigned long micros() override;
    long          pulseIn(uint32_t, uint32_t, unsigned long) override { return 0; }

    void spiBegin() override {}
    void spiBeginTransaction() override {}
    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override;
    void spiEndTransaction() override {}
    void spiEnd() override {}

    void yield() override { taskYIELD(); }

private:
    static void onPin(void* ctx, int pin, int level);

    int             _slot;
    const LoraSlot* _pins;
    simradio_t*     _chip;
    uint32_t        _rstLevel = 1;
};
