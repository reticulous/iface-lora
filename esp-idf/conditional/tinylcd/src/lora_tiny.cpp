/**
 * lora_tiny.cpp — LoRa pages on the tiny OLED: peers and links (`lora`), and
 * traffic (`lora-air`, further down).
 *
 * The neighbour table is in-memory and publishes no change events (and the
 * lora.<n>.stats.* keys are gated on uiTelemetryWanted(), so they can be
 * stale on a headless node) — this page reads loraPeerSummary() directly and
 * uses tinylcd's periodic refresh instead of a subscription. The 2 s repaint
 * only runs while the page is on screen. Radio 0 only: the boards carrying a
 * tiny OLED are single-radio.
 */
#include "lora_tiny.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "compat.h"   /* millis — the traffic page's own sample clock */
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

/* ── the traffic page: what the channel is doing right now ────────────────
 *
 * Byte rates in the title, the radio's own airtime, hailing-channel hourly
 * duty, noise floor and power under it, and a chart of one bar per second,
 * newest at the right. A bar is rx + tx; a second with any tx carries a dot
 * above its bar. Square-root heights: LoRa moves tens to hundreds of bytes a
 * second, and a linear scale flattens all of it the moment one busy second
 * reaches the top.
 *
 * Sampling rides the page's 1 s repaint, which tinylcd runs only while the
 * page is on screen, so the chart is what the page saw: a rate divides by the
 * real interval since the last repaint, and a gap fills one bar, not many. */

#define AIR_SLOTS    60                 /* one minute, a bar a second */
#define AIR_FULL_BPS 2000.0f            /* bytes/s at full height */
#define AIR_BASE     55                 /* chart baseline: clear of a bottom dot strip */
#define AIR_TOP      38                 /* highest a bar or its dot reaches */

static uint16_t s_txRate[AIR_SLOTS], s_rxRate[AIR_SLOTS];   /* bytes/s */
static int      s_slot = -1;                                /* -1: nothing yet */
static uint64_t s_prevTx, s_prevRx;
static uint32_t s_prevMs;
static bool     s_haveBase;                                 /* first repaint is a baseline */

/* A line that must stay on the panel: cut a character at a time until it fits. */
static void drawFit(u8g2_t* g, int y, char* s)
{
    size_t len = strlen(s);
    while (len && u8g2_GetStrWidth(g, s) > 128) s[--len] = 0;
    u8g2_DrawStr(g, 0, y, s);
}

static uint16_t rateOf(uint64_t now, uint64_t prev, uint32_t dtMs)
{
    if (now <= prev || dtMs == 0) return 0;
    uint64_t bps = (now - prev) * 1000u / dtMs;
    return bps > 0xFFFF ? 0xFFFF : (uint16_t)bps;
}

static bool drawAirPage(tinylcd_page_t, u8g2_t* g, tinylcd_ev_t)
{
    char line[48];
    lora_traffic_summary t;

    u8g2_SetFont(g, u8g2_font_7x13B_tr);
    if (!loraTrafficSummary(0, &t)) {
        u8g2_DrawStr(g, 0, TINYLCD_TITLE_Y, "LoRa air");
        u8g2_SetFont(g, u8g2_font_6x10_tf);
        u8g2_DrawStr(g, 0, 31, "no radio");
        return false;
    }

    uint32_t now = millis();
    uint16_t txr = 0, rxr = 0;
    if (s_haveBase) {
        uint32_t dt = now - s_prevMs;
        txr = rateOf(t.tx_bytes, s_prevTx, dt);
        rxr = rateOf(t.rx_bytes, s_prevRx, dt);
        s_slot = (s_slot + 1) % AIR_SLOTS;
        s_txRate[s_slot] = txr;
        s_rxRate[s_slot] = rxr;
    }
    s_prevTx = t.tx_bytes; s_prevRx = t.rx_bytes;
    s_prevMs = now; s_haveBase = true;

    snprintf(line, sizeof line, "tx %u rx %u B/s", txr, rxr);
    drawFit(g, TINYLCD_TITLE_Y, line);

    u8g2_SetFont(g, u8g2_font_6x10_tf);
    snprintf(line, sizeof line, "air %d%%  duty %d.%d%%", t.airtime_pct,
             t.duty_pct10 / 10, t.duty_pct10 % 10);
    drawFit(g, TINYLCD_BODY_Y, line);
    snprintf(line, sizeof line, "nf %d dBm  tx %d dBm", t.noise_dbm, t.txp_dbm);
    drawFit(g, TINYLCD_BODY_Y + 10, line);

    /* 60 bars of 2 px from x = 2; a bar leaves 3 px above it for the tx dot. */
    const int hMax = AIR_BASE - AIR_TOP - 3;
    u8g2_DrawHLine(g, 0, AIR_BASE, 122);
    for (int i = 0; s_slot >= 0 && i < AIR_SLOTS; i++) {
        int idx = (s_slot + 1 + i) % AIR_SLOTS;              /* oldest first */
        uint32_t total = (uint32_t)s_txRate[idx] + s_rxRate[idx];
        if (!total) continue;
        float fr = (float)total / AIR_FULL_BPS;
        int h = (int)(sqrtf(fr > 1.0f ? 1.0f : fr) * hMax);
        if (h < 1) h = 1;
        int x = 2 + i * 2;
        u8g2_DrawVLine(g, x, AIR_BASE - h, h);
        u8g2_DrawVLine(g, x + 1, AIR_BASE - h, h);
        if (s_txRate[idx]) u8g2_DrawVLine(g, x, AIR_BASE - h - 3, 2);
    }
    return false;   /* draw-only page: no events handled */
}

void LoraTinyPage::onInit()
{
    tinylcdAddPage("lora", drawLoraPage, 2000);
    tinylcdAddPage("lora-air", drawAirPage, 1000);
}
