/**
 * VirtualHal — see the header.
 */
#include "virtual_hal.h"

#include "esp_timer.h"
#include "freertos/task.h"

#include <cstring>

namespace {
const LoraSlot* s_pins[8] = {};

/* RadioLib hands out a plain `void(*)(void)`; the shim calls a handler with an
 * argument. One trampoline per pin bridges the two without casting a function
 * pointer to a different signature. */
typedef void (*rl_isr_t)(void);
rl_isr_t s_isr[GPIO_NUM_MAX] = {};

void isrTrampoline(void* arg)
{
    int pin = (int)(intptr_t)arg;
    if (pin >= 0 && pin < GPIO_NUM_MAX && s_isr[pin]) s_isr[pin]();
}
}  // namespace

/* The chip model raises its own interrupt line, and this is where it learns
 * which pin the board called it. */
int virtualHalDio1Pin(int slot)
{
    if (slot < 0 || slot >= (int)(sizeof(s_pins) / sizeof(s_pins[0]))) return -1;
    return s_pins[slot] ? s_pins[slot]->dio1 : -1;
}

VirtualHal::VirtualHal(int slot, const LoraSlot* pins)
    : RadioLibHal(MODE_INPUT, MODE_OUTPUT, LEVEL_LOW, LEVEL_HIGH, EDGE_RISING, EDGE_FALLING),
      _slot(slot), _pins(pins), _chip(virtualChip(slot))
{
    if (slot >= 0 && slot < (int)(sizeof(s_pins) / sizeof(s_pins[0]))) s_pins[slot] = pins;
}

void VirtualHal::pinMode(uint32_t pin, uint32_t mode)
{
    (void)pin; (void)mode;
}

void VirtualHal::digitalWrite(uint32_t pin, uint32_t value)
{
    /* The reset line is the one write with a meaning of its own: the model
     * comes back to its power-on state on the rising edge. */
    if (_pins && (int)pin == _pins->rst) {
        uint32_t was = _rstLevel;
        _rstLevel = value;
        gpio_shim_set_level((int)pin, value);
        if (!was && value && _chip) _chip->reset();
        return;
    }
    gpio_shim_set_level((int)pin, value);
}

uint32_t VirtualHal::digitalRead(uint32_t pin)
{
    /* BUSY is never busy: the model answers a command the moment it is
     * handed one, so the line the driver polls is always clear. */
    if (_pins && (int)pin == _pins->busy) return 0;
    return (uint32_t)gpio_get_level((gpio_num_t)pin);
}

void VirtualHal::attachInterrupt(uint32_t interruptNum, void (*cb)(void), uint32_t mode)
{
    int pin = (int)interruptNum;
    if (pin < 0 || pin >= GPIO_NUM_MAX) return;
    s_isr[pin] = cb;
    gpio_set_intr_type((gpio_num_t)pin, (gpio_int_type_t)mode);
    gpio_isr_handler_add((gpio_num_t)pin, isrTrampoline, (void*)(intptr_t)pin);
    gpio_intr_enable((gpio_num_t)pin);
}

void VirtualHal::detachInterrupt(uint32_t interruptNum)
{
    int pin = (int)interruptNum;
    if (pin < 0 || pin >= GPIO_NUM_MAX) return;
    gpio_intr_disable((gpio_num_t)pin);
    gpio_isr_handler_remove((gpio_num_t)pin);
    s_isr[pin] = nullptr;
}

void VirtualHal::delay(unsigned long ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms) + 1);
}

void VirtualHal::delayMicroseconds(unsigned long us)
{
    if (us >= (unsigned long)portTICK_PERIOD_MS * 1000) {
        vTaskDelay(pdMS_TO_TICKS(us / 1000) + 1);
        return;
    }
    int64_t until = esp_timer_get_time() + (int64_t)us;
    while (esp_timer_get_time() < until) taskYIELD();
}

unsigned long VirtualHal::millis()
{
    return (unsigned long)(esp_timer_get_time() / 1000);
}

unsigned long VirtualHal::micros()
{
    return (unsigned long)esp_timer_get_time();
}

void VirtualHal::spiTransfer(uint8_t* out, size_t len, uint8_t* in)
{
    if (_chip) _chip->transfer(out, len, in);
    else if (in) memset(in, 0, len);
}
