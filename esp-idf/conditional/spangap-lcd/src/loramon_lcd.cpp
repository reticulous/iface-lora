/**
 * loramon_lcd.cpp — "LoRaMon": per-on-air-frame airtime/signal monitor painted
 * by hand into one RGB565 canvas. The on-device sibling of the browser LoRaMon
 * window; the LCD counterpart to actmon_app.cpp.
 *
 *   Per-radio tabs (lora/0, lora/1, …). Three stacked graphs, newest at the
 *   right edge: 1 min (top), 10 min, 1 hour. Each frame is a bar spanning its
 *   time-on-air, coloured #4088E8 for rx and #E8D040 for tx; the height is a
 *   signal score — SNR-weighted link quality for rx, transmit power for tx.
 *   Each caption shows the window's rx/tx airtime %.
 *
 * Source of truth is storage: the firmware publishes one node per frame at
 * `lora.<n>.packets.<ms>` = "r|rssi|snr|dur|bytes" (rx) / "t|txp|dur|bytes"
 * (tx), and deletes them past 1 h. We rebuild our view by iterating that subtree
 * each redraw (so expiry — which doesn't fire subscribe callbacks — is handled),
 * and setting `sys.stats.lcd_loramon` tells the firmware to record while we're up.
 */
#include "lcd.h"        /* lcdFont / LcdFace */
#include "lcd_app.h"
#include "loramon_app.h"

#include "storage.h"
#include "compat.h"     /* millis() */

#include <esp_heap_caps.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>

namespace {

constexpr int TABH = 18;          /* tab bar height (0 when a single radio) */
constexpr int CAPH = 11;          /* caption strip, overlaid at each band top */
constexpr int MON_MAX = 4096;     /* max packets held for a redraw (matches fw cap) */

struct Win { const char* label; uint32_t ms; };
constexpr Win WINS[3] = {
    { "1 min",  60u * 1000 },
    { "10 min", 600u * 1000 },
    { "1 hour", 3600u * 1000 },
};

struct Rec { uint32_t t; uint8_t dir; uint32_t dur; uint32_t bytes; int rssi; int snr10; int txp; };

uint16_t C_RX, C_TX, C_BLACK;
bool s_colorsReady = false;
void initColors() {
    if (s_colorsReady) return;
    C_BLACK = lv_color_to_u16(lv_color_black());
    C_RX    = lv_color_to_u16(lv_color_hex(0x4088E8));   /* received frames (blue) */
    C_TX    = lv_color_to_u16(lv_color_hex(0xE8D040));   /* transmitted frames (yellow) */
    s_colorsReady = true;
}

struct State {
    lv_obj_t* canvas = nullptr;
    lv_obj_t* tabs[8] = {};
    int       radioOf[8] = {};
    int       nTabs = 0;
    lv_obj_t* cap[3] = {};
    uint16_t* buf = nullptr;
    Rec*      recs = nullptr;
    int       n = 0;                /* records in `recs` this redraw */
    int       W = 0, H = 0, stridePx = 0;
    int       bandY[3] = {}, bandH[3] = {};
    int       radio = 0;
    bool      visible = false;
};
State s;

inline void px(int x, int y, uint16_t c) {
    if ((unsigned)x < (unsigned)s.W && (unsigned)y < (unsigned)s.H)
        s.buf[y * s.stridePx + x] = c;
}
inline void vseg(int x, int yTop, int yBot, uint16_t c) {
    for (int y = yTop; y <= yBot; y++) px(x, y, c);
}

void drawBands(int y0, int h) {
    int qh = h / 4; if (qh < 1) qh = 1;
    for (int r = 0; r < h; r++) {
        int inq = (r % qh);
        int lvl = 0x31 - (inq * (0x31 - 0x24)) / qh;
        uint16_t g = lv_color_to_u16(lv_color_make(lvl, lvl, lvl));
        int y = y0 + r;
        if ((unsigned)y >= (unsigned)s.H) break;
        uint16_t* row = &s.buf[y * s.stridePx];
        for (int x = 0; x < s.W; x++) row[x] = g;
    }
}

inline double clamp01(double x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

int barHpx(const Rec& r, int h) {
    double norm;
    if (r.dir == 1) {
        norm = clamp01((r.txp + 9) / 31.0);
    } else {
        double snrN  = clamp01((r.snr10 / 10.0 + 20) / 30.0);   /* −20 … +10 dB */
        double rssiN = clamp01((r.rssi + 130) / 90.0);          /* −130 … −40 dBm */
        norm = 0.6 * snrN + 0.4 * rssiN;
    }
    int hp = (int)(norm * (h - 1) + 0.5);
    if (hp < 1) hp = 1;
    if (hp > h - 1) hp = h - 1;
    return hp;
}

int airPermille(uint32_t now, uint32_t win, int dir) {
    if (!win) return 0;
    uint32_t lo = now > win ? now - win : 0;
    uint64_t busy = 0;
    for (int i = 0; i < s.n; i++) {
        if (s.recs[i].dir != (uint8_t)dir) continue;
        uint32_t st = s.recs[i].t, en = s.recs[i].t + s.recs[i].dur;
        if (en <= lo || st >= now) continue;
        if (st < lo) st = lo;
        if (en > now) en = now;
        if (en > st) busy += (en - st);
    }
    return (int)(busy * 1000 / win);
}

/* storageForEach has no userdata — accumulate into the file-static `s.recs`. */
void rebuildCb(const char* key, const char* val) {
    if (s.n >= MON_MAX || !val) return;
    const char* dot = strrchr(key, '.');
    if (!dot) return;
    Rec& r = s.recs[s.n];
    r.t = (uint32_t)strtoul(dot + 1, nullptr, 10);
    if (val[0] == 'r') {
        int rssi, snr, dur, bytes;
        if (sscanf(val + 2, "%d|%d|%d|%d", &rssi, &snr, &dur, &bytes) != 4) return;
        r.dir = 0; r.rssi = rssi; r.snr10 = snr; r.dur = (uint32_t)dur; r.bytes = (uint32_t)bytes; r.txp = 0;
    } else if (val[0] == 't') {
        int txp, dur, bytes;
        if (sscanf(val + 2, "%d|%d|%d", &txp, &dur, &bytes) != 3) return;
        r.dir = 1; r.txp = txp; r.dur = (uint32_t)dur; r.bytes = (uint32_t)bytes; r.rssi = 0; r.snr10 = 0;
    } else return;
    s.n++;
}

void rebuild() {
    s.n = 0;
    if (!s.recs) return;
    char pfx[32];
    snprintf(pfx, sizeof pfx, "lora.%d.packets.", s.radio);
    storageForEach(pfx, rebuildCb);
}

void drawGraph(int gi, uint32_t now) {
    int y0 = s.bandY[gi], h = s.bandH[gi];
    drawBands(y0, h);
    if (!now || h < 2) return;
    uint32_t win = WINS[gi].ms;
    int bottom = y0 + h - 1;
    uint32_t lo = now > win ? now - win : 0;
    for (int i = 0; i < s.n; i++) {
        const Rec& r = s.recs[i];
        uint32_t st = r.t, en = r.t + r.dur;
        if (en < lo || st > now) continue;
        uint32_t cs = st > lo ? st - lo : 0;
        uint32_t ce = (en < now ? en : now) - lo;
        int xs = (int)((uint64_t)cs * s.W / win);
        int xe = (int)((uint64_t)ce * s.W / win);
        if (xs < 0) xs = 0;
        if (xe > s.W - 1) xe = s.W - 1;
        if (xe < xs) xe = xs;
        int hp = barHpx(r, h);
        uint16_t col = r.dir ? C_TX : C_RX;
        int th = h / 20; if (th < 1) th = 1;  /* line thickness = 5% of band height */
        int yb = bottom - hp;                 /* horizontal line at the power level */
        int yt = yb - (th - 1); if (yt < y0) yt = y0;
        if (yb > bottom) yb = bottom;
        for (int x = xs; x <= xe; x++) vseg(x, yt, yb, col);
    }
}

void clearAll() {
    int total = s.stridePx * s.H;
    for (int i = 0; i < total; i++) s.buf[i] = C_BLACK;
}

void drawAll() {
    if (!s.canvas || !s.buf || !s.recs) return;
    clearAll();
    rebuild();
    uint32_t now = millis();
    for (int gi = 0; gi < 3; gi++) {
        drawGraph(gi, now);
        if (s.cap[gi]) {
            int rx = airPermille(now, WINS[gi].ms, 0);
            int tx = airPermille(now, WINS[gi].ms, 1);
            char b[80];
            snprintf(b, sizeof b, "%s  airtime #4088E8 %d.%d%% rx# / #E8D040 %d.%d%% tx#",
                     WINS[gi].label, rx / 10, rx % 10, tx / 10, tx % 10);
            lv_label_set_text(s.cap[gi], b);
        }
    }
    lv_obj_invalidate(s.canvas);
}

void tickCb(lv_timer_t*) { if (s.visible) drawAll(); }

lv_obj_t* mkCaption(lv_obj_t* root, int y, const char* text) {
    lv_obj_t* l = lv_label_create(root);
    lv_label_set_recolor(l, true);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, lcdFont(LcdFace::MONO, 8), 0);
    lv_obj_set_style_text_color(l, lv_color_hex(0xC8C8C8), 0);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 2, y);
    return l;
}

void setTab(int t);
void tabEventCb(lv_event_t* e) { setTab((int)(intptr_t)lv_event_get_user_data(e)); }

void styleTabs(int active) {
    for (int i = 0; i < s.nTabs; i++)
        if (s.tabs[i])
            lv_obj_set_style_bg_color(s.tabs[i],
                lv_color_hex(i == active ? 0x383838 : 0x202020), 0);
}

lv_obj_t* mkTab(lv_obj_t* root, const char* label, int idx, int x, int w) {
    lv_obj_t* b = lv_obj_create(root);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, w, TABH);
    lv_obj_set_pos(b, x, 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(b, tabEventCb, LV_EVENT_CLICKED, (void*)(intptr_t)idx);
    lv_obj_t* l = lv_label_create(b);
    lv_label_set_text(l, label);
    lv_obj_set_style_text_font(l, lcdFont(LcdFace::MONO, 8), 0);
    lv_obj_center(l);
    return b;
}

void setTab(int t) {
    if (t < 0 || t >= s.nTabs) return;
    s.radio = s.radioOf[t];
    styleTabs(t);
    drawAll();
}

/* A radio slot is present once the firmware has published its state key. */
bool radioPresent(int n) {
    char k[32]; snprintf(k, sizeof k, "lora.%d.state", n);
    char st[16]; storageGetStr(k, st, sizeof st, "");
    return st[0] != '\0';
}

}  // namespace

LoraMonApp::LoraMonApp() : LcdApp({ .name = "LoRaMon", .iconBasename = "loramon" }) {}

void LoraMonApp::onCreate(lv_obj_t* root) {
    initColors();

    int W = lv_obj_get_content_width(root);
    int H = lv_obj_get_content_height(root);
    if (W <= 0) W = 320;
    if (H <= 0) H = 200;

    uint32_t stride = lv_draw_buf_width_to_stride((uint32_t)W, LV_COLOR_FORMAT_RGB565);
    s.buf  = (uint16_t*)heap_caps_malloc((size_t)stride * H, MALLOC_CAP_SPIRAM);
    s.recs = (Rec*)heap_caps_malloc((size_t)MON_MAX * sizeof(Rec), MALLOC_CAP_SPIRAM);
    if (!s.buf || !s.recs) { free(s.buf); free(s.recs); s.buf = nullptr; s.recs = nullptr; return; }
    s.W = W; s.H = H; s.stridePx = (int)(stride / 2);

    s.canvas = lv_canvas_create(root);
    lv_canvas_set_buffer(s.canvas, s.buf, W, H, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(s.canvas, lv_color_black(), LV_OPA_COVER);
    lv_obj_align(s.canvas, LV_ALIGN_TOP_LEFT, 0, 0);

    s.nTabs = 0;
    for (int r = 0; r < 4 && s.nTabs < 8; r++)
        if (radioPresent(r)) s.radioOf[s.nTabs++] = r;
    if (s.nTabs == 0) { s.radioOf[0] = 0; s.nTabs = 1; }
    s.radio = s.radioOf[0];

    int top = 0;
    if (s.nTabs > 1) {
        int tw = W / s.nTabs;
        for (int i = 0; i < s.nTabs; i++) {
            char label[12];
            snprintf(label, sizeof label, "lora/%d", s.radioOf[i]);
            s.tabs[i] = mkTab(root, label, i, i * tw, (i == s.nTabs - 1) ? W - i * tw : tw);
        }
        styleTabs(0);
        top = TABH;
    }

    int avail = H - top;
    for (int gi = 0; gi < 3; gi++) {
        s.bandY[gi] = top + (avail * gi) / 3;
        int next    = top + (avail * (gi + 1)) / 3;
        s.bandH[gi] = next - s.bandY[gi];
        s.cap[gi]   = mkCaption(root, s.bandY[gi] + 1, WINS[gi].label);
    }
    (void)CAPH;

    drawAll();
    timer(tickCb, 1000, this);
}

void LoraMonApp::onShow() { s.visible = true; storageSet("sys.stats.lcd_loramon", 1); drawAll(); }
void LoraMonApp::onHide() { s.visible = false; storageSet("sys.stats.lcd_loramon", 0); }
void LoraMonApp::onClose() {
    storageSet("sys.stats.lcd_loramon", 0);
    free(s.buf);
    free(s.recs);
    s = State{};
}
