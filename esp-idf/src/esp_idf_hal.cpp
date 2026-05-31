/**
 * EspIdfHal — see header for docs.
 *
 * SPI driver: each transfer uses `spi_device_polling_transmit` for low
 * latency on small SX126x command words. RadioLib brackets all transfers
 * with spiBeginTransaction / spiEndTransaction; we hold the bus across
 * the pair so multi-byte command-then-data exchanges stay atomic on the
 * shared FSPI bus.
 *
 * CS pin: the SPI driver drives CS for us (`spics_io_num`), so RadioLib's
 * pinMode/digitalWrite on the CS pin is a no-op from our perspective —
 * but RadioLib still sends those calls, so we treat them like any other
 * GPIO. We do NOT pass CS to the SPI driver as the device's CS; instead
 * we let RadioLib pulse it via digitalWrite. This avoids surprises when
 * RadioLib (legitimately) wants to hold CS low across what it considers
 * two "transactions".
 */
#include "esp_idf_hal.h"

#include "spangap.h"     /* info()/warn()/err() macros, safeStrncpy, etc. */
#include "spi_helper.h"  /* spiHelperInitBus — spangap-core shared SPI bus owner */

#include "esp_timer.h"
#include "freertos/task.h"

#include <cstring>

EspIdfHal::EspIdfHal(spi_host_device_t spiHost,
                     int sckPin, int mosiPin, int misoPin,
                     int csPin, int sxClockHz)
    : RadioLibHal(MODE_INPUT, MODE_OUTPUT, LEVEL_LOW, LEVEL_HIGH,
                  EDGE_RISING, EDGE_FALLING),
      _host(spiHost), _sckPin(sckPin), _mosiPin(mosiPin), _misoPin(misoPin),
      _csPin(csPin), _sxClockHz(sxClockHz)
{
}

void EspIdfHal::init()
{
    /* RadioLib's Module::init() also calls hal->init(), so we may be
     * called more than once across radio start/stop cycles. Bus + ISR
     * service + device handle are one-shot. */
    if (_inited) return;

    /* SPI bus init via spangap-core's spi_helper — idempotent, safe to
     * call alongside future LCD/SD drivers that share this bus. */
    spi_bus_config_t bus = {};
    bus.mosi_io_num   = _mosiPin;
    bus.miso_io_num   = _misoPin;
    bus.sclk_io_num   = _sckPin;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 0;
    esp_err_t r = spiHelperInitBus(_host, &bus);
    if (r != ESP_OK) return;

    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = _sxClockHz;
    dev.mode           = 0;                    /* SX126x is SPI mode 0 */
    dev.spics_io_num   = -1;                   /* RadioLib drives CS itself */
    dev.queue_size     = 1;
    dev.flags          = 0;
    r = spi_bus_add_device(_host, &dev, &_spiDev);
    if (r != ESP_OK) {
        err("spi_bus_add_device: %s", esp_err_to_name(r));
        return;
    }

    /* GPIO ISR service for attachInterrupt — install once, lazily. */
    if (!_isrServiceInstalled) {
        r = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (r == ESP_OK || r == ESP_ERR_INVALID_STATE /* someone beat us */) {
            _isrServiceInstalled = true;
        } else {
            warn("gpio_install_isr_service: %s", esp_err_to_name(r));
        }
    }
    _inited = true;
}

void EspIdfHal::term()
{
    if (_spiDev) {
        spi_bus_remove_device(_spiDev);
        _spiDev = nullptr;
    }
    /* Don't tear down the SPI bus — it may be shared with other peripherals
     * (display/SD on T-Deck). It's cheap to leave initialized. */
    _inited = false;
}

void EspIdfHal::pinMode(uint32_t pin, uint32_t mode)
{
    gpio_config_t io = {};
    io.pin_bit_mask  = 1ULL << pin;
    io.mode          = (gpio_mode_t)mode;
    io.pull_up_en    = GPIO_PULLUP_DISABLE;
    io.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    io.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&io);
}

void EspIdfHal::digitalWrite(uint32_t pin, uint32_t value)
{
    gpio_set_level((gpio_num_t)pin, value ? 1 : 0);
}

uint32_t EspIdfHal::digitalRead(uint32_t pin)
{
    return (uint32_t)gpio_get_level((gpio_num_t)pin);
}

/* GPIO ISR trampoline — RadioLib hands us a `void (*)(void)`; ESP-IDF
 * wants `void (*)(void*)`. Stash the callback per-pin so the trampoline
 * can find it. One slot per pin is plenty for our usage (one DIO1). */
namespace {
constexpr int kMaxPins = GPIO_NUM_MAX;
void (*g_isrCb[kMaxPins])(void) = {};

void IRAM_ATTR isrTrampoline(void* arg) {
    uintptr_t pin = (uintptr_t)arg;
    /* Disable the GPIO interrupt before invoking the callback. For edge-
     * triggered users this is harmless; for level-triggered users (e.g.
     * pins registered with pm as light-sleep wake sources) it prevents
     * the ISR from re-firing continuously while the line is asserted.
     * The consumer task re-enables via gpio_intr_enable after servicing
     * the peripheral that pulled the line. */
    gpio_intr_disable((gpio_num_t)pin);
    if (pin < kMaxPins && g_isrCb[pin]) g_isrCb[pin]();
}
}

void EspIdfHal::attachInterrupt(uint32_t interruptNum, void (*cb)(void), uint32_t mode)
{
    if (interruptNum >= (uint32_t)kMaxPins) return;
    g_isrCb[interruptNum] = cb;

    gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)mode);
    gpio_isr_handler_add((gpio_num_t)interruptNum, isrTrampoline, (void*)(uintptr_t)interruptNum);
    gpio_intr_enable((gpio_num_t)interruptNum);
}

void EspIdfHal::detachInterrupt(uint32_t interruptNum)
{
    if (interruptNum >= (uint32_t)kMaxPins) return;
    gpio_intr_disable((gpio_num_t)interruptNum);
    gpio_isr_handler_remove((gpio_num_t)interruptNum);
    g_isrCb[interruptNum] = nullptr;
}

void EspIdfHal::delay(unsigned long ms)
{
    if (ms == 0) { taskYIELD(); return; }
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void EspIdfHal::delayMicroseconds(unsigned long us)
{
    if (us == 0) return;
    /* Up to ~1 ms busy-wait; longer goes through vTaskDelay (granularity
     * is 1 tick, but RadioLib only calls us with us<1000 in practice). */
    int64_t end = esp_timer_get_time() + (int64_t)us;
    while (esp_timer_get_time() < end) { /* spin */ }
}

unsigned long EspIdfHal::millis()
{
    return (unsigned long)(esp_timer_get_time() / 1000);
}

unsigned long EspIdfHal::micros()
{
    return (unsigned long)esp_timer_get_time();
}

void EspIdfHal::spiBegin()
{
    /* Bus + device set up in init(); nothing to do per-driver-begin. */
}

void EspIdfHal::spiBeginTransaction()
{
    /* Serialize with the LCD's async-DMA flush on the shared SPI2 bus (the
     * SPI driver's own bus lock isn't enough — esp_lcd drops it before its DMA
     * finishes). Outer lock first, then the driver bus, released in reverse. */
    spiHelperBusLock();
    if (_spiDev) spi_device_acquire_bus(_spiDev, portMAX_DELAY);
}

void EspIdfHal::spiTransfer(uint8_t* out, size_t len, uint8_t* in)
{
    if (!_spiDev || len == 0) {
        if (in && len) std::memset(in, 0, len);
        return;
    }
    spi_transaction_t t = {};
    t.length    = len * 8;
    t.tx_buffer = out;
    t.rx_buffer = in;
    esp_err_t r = spi_device_polling_transmit(_spiDev, &t);
    if (r != ESP_OK) {
        warn("spi xfer (%u B) failed: %s", (unsigned)len, esp_err_to_name(r));
        if (in) std::memset(in, 0, len);
    }
}

void EspIdfHal::spiEndTransaction()
{
    if (_spiDev) spi_device_release_bus(_spiDev);
    spiHelperBusUnlock();
}

void EspIdfHal::spiEnd()
{
    /* Mirror spiBegin — no-op. Bus stays up until term(). */
}
