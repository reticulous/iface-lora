/**
 * loramon_app.h — "LoRaMon": the on-device LoRa airtime/signal monitor as a
 * boot-registered Service (an LcdApp).
 *
 * Per-radio tabs; three stacked per-on-air-frame graphs (1 min / 10 min / 1 hour),
 * blue for rx, yellow for tx, bar height a signal/quality score. Reads the ring
 * lora.cpp keeps (loraMonSnapshot) in-process — the browser reads the same ring
 * over the `loramon` ITS port. Compiled only under conditional/spangap-lcd/, so
 * it exists only when the lcd straddle is staged (the `when:` gate on the
 * straddle.yaml services entry).
 */
#pragma once

#include "lcd_app.h"   /* LcdApp (a Service) */
#include "lvgl.h"      /* lv_obj_t */

class LoraMonApp : public LcdApp {
public:
    LoraMonApp();
    void onCreate(lv_obj_t* root) override;
    void onShow() override;
    void onHide() override;
    void onClose() override;
};
