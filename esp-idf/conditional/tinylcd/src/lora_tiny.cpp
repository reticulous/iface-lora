/**
 * lora_tiny.cpp — LoRa peer/link status page on the tiny OLED.
 *
 * The neighbour table is in-memory and publishes no change events (and the
 * lora.<n>.stats.* keys are gated on uiTelemetryWanted(), so they can be
 * stale on a headless node) — this page reads loraPeerSummary() directly and
 * uses tinylcd's periodic refresh instead of a subscription. The 2 s repaint
 * only runs while the page is on screen. Radio 0 only: the boards carrying a
 * tiny OLED are single-radio.
 */
#include "lora_tiny.h"

#include <stdio.h>
#include <stdlib.h>

#include "lora.h"
#include "storage.h"
#include "tinylcd.h"

static bool drawLoraPage(tinylcd_page_t, u8g2_t* g, tinylcd_ev_t)
{
    char line[48];

    u8g2_SetFont(g, u8g2_font_7x13B_tr);
    std::string freq = storageGetStr("lora.0.freq_mhz", "");
    snprintf(line, sizeof line, "LoRa %s", freq.c_str());
    u8g2_DrawStr(g, 0, TINYLCD_TITLE_Y, line);

    u8g2_SetFont(g, u8g2_font_6x10_tf);
    std::string state = storageGetStr("lora.0.state", "off");
    lora_peer_summary sum;
    if (!loraPeerSummary(0, &sum)) {
        snprintf(line, sizeof line, "%s, no observations", state.c_str());
        u8g2_DrawStr(g, 0, 31, line);
        return false;
    }
    snprintf(line, sizeof line, "%s  peers %d  links %d", state.c_str(),
             sum.peers, sum.links);
    u8g2_DrawStr(g, 0, 31, line);
    snprintf(line, sizeof line, "rx %d dBm  %d.%d dB", sum.rssi,
             sum.snr10 / 10, abs(sum.snr10) % 10);
    u8g2_DrawStr(g, 0, 43, line);
    return false;   /* draw-only page: no events handled */
}

void LoraTinyPage::onInit()
{
    tinylcdAddPage("lora", drawLoraPage, 2000);
}
