/**
 * lora_tiny.h — LoRa peer/link status page on the tiny OLED.
 *
 * The when:-gated tinylcd slice of iface-lora (conditional/tinylcd/): only
 * compiled when spangap/tinylcd is staged. Radio state, neighbours heard,
 * open links, last-frame RSSI/SNR. See lora_tiny.cpp.
 */
#pragma once

#include "service.h"

class LoraTinyPage : public Service {
public:
    void onInit() override;
};
