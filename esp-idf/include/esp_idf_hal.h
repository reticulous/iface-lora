/**
 * EspIdfHal — RadioLib HAL implementation on top of ESP-IDF.
 *
 * Owns one SPI device handle on the shared bus per radio (the bus is brought
 * up idempotently via spi_helper). GPIO pin numbers are passed through
 * verbatim — the radio
 * module hands us back whatever we set as `inputPin`/`outputPin`/etc. in
 * the ctor, so we map our HAL constants 1:1 onto ESP-IDF enums.
 *
 * Pin mode / level / IRQ trigger constants are *our* constants, not
 * Arduino's. RadioLib stores them as member fields and feeds them back to
 * us when it wants to drive a pin — they are opaque to RadioLib.
 *
 * ISR: attachInterrupt registers a `void (*)(void)` callback against a
 * GPIO; the GPIO ISR service trampolines into it. lora.cpp installs a
 * single IRAM_ATTR ISR for DIO1 that does vTaskNotifyGiveFromISR.
 */
#pragma once

#include <RadioLib.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"

class EspIdfHal : public RadioLibHal {
public:
    /* Pin-mode encoding the radio module will hand back to pinMode().
     * Values chosen so we can pass them straight to gpio_set_direction(). */
    static constexpr uint32_t MODE_INPUT  = (uint32_t)GPIO_MODE_INPUT;
    static constexpr uint32_t MODE_OUTPUT = (uint32_t)GPIO_MODE_OUTPUT;

    static constexpr uint32_t LEVEL_LOW   = 0;
    static constexpr uint32_t LEVEL_HIGH  = 1;

    static constexpr uint32_t EDGE_RISING  = (uint32_t)GPIO_INTR_POSEDGE;
    static constexpr uint32_t EDGE_FALLING = (uint32_t)GPIO_INTR_NEGEDGE;

    EspIdfHal(spi_host_device_t spiHost,
              int sckPin, int mosiPin, int misoPin,
              int csPin, int sxClockHz = 8 * 1000 * 1000);

    /* RadioLibHal — lifecycle */
    void init() override;
    void term() override;

    /* RadioLibHal — GPIO */
    void     pinMode(uint32_t pin, uint32_t mode) override;
    void     digitalWrite(uint32_t pin, uint32_t value) override;
    uint32_t digitalRead(uint32_t pin) override;
    void     attachInterrupt(uint32_t interruptNum, void (*cb)(void), uint32_t mode) override;
    void     detachInterrupt(uint32_t interruptNum) override;

    /* RadioLibHal — time */
    void     delay(unsigned long ms) override;
    void     delayMicroseconds(unsigned long us) override;
    unsigned long millis() override;
    unsigned long micros() override;
    long     pulseIn(uint32_t /*pin*/, uint32_t /*state*/, unsigned long /*timeout*/) override { return 0; }

    /* RadioLibHal — SPI */
    void spiBegin() override;
    void spiBeginTransaction() override;
    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override;
    void spiEndTransaction() override;
    void spiEnd() override;

    /* Optional virtuals defined with empty defaults in some versions. */
    void yield() override { taskYIELD(); }

    /* Direct access for lora.cpp to drive a few non-RadioLib things
     * (e.g. reading BUSY for diagnostics). Not used by the radio class. */
    spi_device_handle_t spiDev() const { return _spiDev; }

private:
    spi_host_device_t   _host;
    int                 _sckPin, _mosiPin, _misoPin, _csPin;
    int                 _sxClockHz;
    spi_device_handle_t _spiDev = nullptr;
    bool                _inited = false;
};
