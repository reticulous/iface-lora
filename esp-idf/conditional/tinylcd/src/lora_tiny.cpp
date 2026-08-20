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
    u8g2_DrawStr(g, 0, 18, line);

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

/* ── the traffic page — counters plus a scrolling per-second chart ────────
 *
 * The RNode-display idea, on tinylcd terms: what is the channel doing RIGHT
 * NOW. Top rows are instantaneous tx/rx byte rates and the radio's own
 * airtime / noise-floor / power figures; the rest is a chart, one bar per
 * second, scrolling left. RX is the solid bar from the baseline; a TX second
 * carries a 2-px cap dot above its bar, so transmit activity reads at a
 * glance without a second axis. Square-root height scale — LoRa rates live
 *  in the tens-to-hundreds of bytes/s and a linear scale would flatline them
 * the moment one good second hits the cap.
 *
 * Sampling rides the page's own 1 s refresh, so the ring only advances while
 * the page is on screen (tinylcd only repaints the visible page). Rates stay
 * honest across a gap: the delta divides by the real wall-clock interval,
 * and a gap longer than one slot just fills one slot — history is what the
 * page SAW, not a pretense of background bookkeeping. */

#define TRAFFIC_SLOTS   60                     /* one minute at 1 s a bar */
#define TRAFFIC_FULL_BPS 2000.0f               /* bar full scale, bytes/s */

static uint16_t s_txRate[TRAFFIC_SLOTS];       /* bytes/s, capped to 16 bit */
static uint16_t s_rxRate[TRAFFIC_SLOTS];
static int      s_slot = -1;                   /* -1: ring empty */
static uint64_t s_prevTx, s_prevRx;
static uint32_t s_prevMs;
static bool     s_haveBase;                    /* first sample = baseline only */

static uint16_t rateOf(uint64_t now, uint64_t prev, uint32_t dtMs)
{
    if (now <= prev || dtMs == 0) return 0;
    uint64_t bps = (now - prev) * 1000u / dtMs;
    return (bps > 0xFFFF) ? 0xFFFF : (uint16_t)bps;
}

static bool drawTrafficPage(tinylcd_page_t, u8g2_t* g, tinylcd_ev_t)
{
    char line[48];
    lora_traffic_summary t;
    if (!loraTrafficSummary(0, &t)) {
        u8g2_SetFont(g, u8g2_font_6x10_tf);
        u8g2_DrawStr(g, 0, 31, "no radio");
        return false;
    }

    /* Sample on every repaint (1 s refresh while visible). */
    uint32_t now = millis();
    uint16_t txr = 0, rxr = 0;
    if (s_haveBase) {
        uint32_t dt = now - s_prevMs;
        txr = rateOf(t.tx_bytes, s_prevTx, dt);
        rxr = rateOf(t.rx_bytes, s_prevRx, dt);
        s_slot = (s_slot + 1) % TRAFFIC_SLOTS;
        s_txRate[s_slot] = txr;
        s_rxRate[s_slot] = rxr;
    }
    s_prevTx = t.tx_bytes; s_prevRx = t.rx_bytes;
    s_prevMs = now; s_haveBase = true;

    /* Numbers: instantaneous rates, then the radio's own channel figures. */
    u8g2_SetFont(g, u8g2_font_6x10_tf);
    snprintf(line, sizeof line, "tx %uB/s rx %uB/s", txr, rxr);
    u8g2_DrawStr(g, 0, 8, line);
    snprintf(line, sizeof line, "air %d%%  nf %d  txp %d",
             t.airtime_pct, t.noise_dbm, t.txp_dbm);
    u8g2_DrawStr(g, 0, 18, line);

    /* Chart: 60 bars x 2 px, newest at the right edge, baseline at y=63.
     * Height = sqrt scale to TRAFFIC_FULL_BPS over 40 px. The chart region
     * starts at y=22, leaving 2 px air under the text. */
    const int base = 63, hMax = 40;
    u8g2_DrawHLine(g, 0, base, 122);
    if (s_slot >= 0) {
        for (int i = 0; i < TRAFFIC_SLOTS; i++) {
            /* i=0 oldest … newest at the right */
            int idx = (s_slot + 1 + i) % TRAFFIC_SLOTS;
            int x = 2 + i * 2;
            uint32_t total = (uint32_t)s_txRate[idx] + s_rxRate[idx];
            if (!total) continue;
            float fr = (float)total / TRAFFIC_FULL_BPS;
            if (fr > 1.0f) fr = 1.0f;
            int h = (int)(sqrtf(fr) * hMax);
            if (h < 1) h = 1;
            if (h > hMax) h = hMax;
            u8g2_DrawVLine(g, x, base - h, h);
            u8g2_DrawVLine(g, x + 1, base - h, h);
            if (s_txRate[idx])                     /* TX cap dot */
                u8g2_DrawVLine(g, x, base - h - 3, 2);
        }
    }
    return false;   /* draw-only page: no events handled */
}

void LoraTinyPage::onInit()
{
    tinylcdAddPage("lora", drawLoraPage, 2000);
    tinylcdAddPage("lora-air", drawTrafficPage, 1000);
}
