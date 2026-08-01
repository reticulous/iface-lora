/**
 * loramon_lcd.cpp — "LoRaMon": per-on-air-frame airtime/signal monitor painted
 * by hand into one RGB565 canvas. The on-device sibling of the browser LoRaMon
 * window; the LCD counterpart to actmon_app.cpp.
 *
 *   Per-radio tabs (lora/0, lora/1, …), then a row of window pills (10s, 1m,
 *   5m, 10m, 30m, 1hr) selecting the time span shown. One plot, newest at the
 *   right edge, carrying both directions: each frame is a bar spanning its
 *   time-on-air, placed at its power on one of two dBm axes over the same four
 *   bands — transmit −10…+30 dBm down the left gutter in 10 dB steps, receive
 *   −130…−30 dBm down the right in 25 dB. Behind a transmit the bands are cast
 *   red, over its time-on-air only, so which direction a bar belongs to is
 *   legible without asking the colour: colour is the frame's PROTOCOL —
 *   Reticulum traffic yellow, the RNode client's orange, this straddle's own
 *   air protocol (rfprobe, hash linkage) red.
 *
 *   Touching the graph starts a highlighted span at that instant. Because the
 *   anchor is a *time*, not a pixel, holding still on a live graph widens the
 *   highlight — the anchor drifts left as "now" advances. Releasing zooms to
 *   the span and pushes it on a zoom stack; the pills give way to a single back
 *   pill. A zoomed view stands still, so it can carry a timescale: division
 *   lines in the gradient's own darkest tone, with the span and what one
 *   division is worth printed beside the back pill.
 *   Selecting inside it zooms further, pushing another level. Back pops one and
 *   is blue while a single level is left — while it leads back to the live
 *   window — and emptying the stack returns to whichever moving window was
 *   active.
 *
 * Source of truth is storage: the firmware publishes one node per frame at
 * `lora.<n>.packets.<ms>` = "r|rssi|snr|dur|bytes|type" (rx) /
 * "t|txp|dur|bytes|type|wait" (tx), and deletes them past 1 h. We rebuild our view
 * by iterating that subtree each redraw (so expiry — which doesn't fire
 * subscribe callbacks — is handled), and setting `sys.stats.lcd_loramon` tells
 * the firmware to record while we're up. The caption's two figures — what we
 * transmitted, and how much of the channel was in use at all — are computed
 * here from those records for the selected window; the 1-hour pair is the one
 * the firmware publishes itself (`lora.<n>.air1h.{rx,tx}`, per mille), because
 * it covers more history than this app is typically open for.
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
constexpr int PILLH = 16;         /* window-selector strip height */
constexpr int PAD_PILLS = 6;      /* breathing room under the pill row */
constexpr int GUT_L = 26;         /* left scale gutter (tx dBm), px */
constexpr int GUT_R = 30;         /* right scale gutter (rx dBm) — four digits */
constexpr int MON_MAX = 4096;     /* max packets held for a redraw (matches fw cap) */
constexpr int ZOOM_MAX = 8;       /* zoom-stack depth */
/* Narrowest timescale division, px. Divisions come off the 1-2-5-10 ladder,
 * whose widest gap is ×2.5 (2 → 5), so a band of allowed pixels-per-division at
 * least 2.5× wide always contains a step, whatever span is frozen. 40 px up
 * puts 3 to 7 lines across this plot; the 100 px it implies is the other edge. */
constexpr int DIV_PX_MIN = 40;

struct Win { const char* label; uint32_t ms; };
constexpr Win WINS[6] = {
    { "10s", 10u * 1000 },
    { "1m",  60u * 1000 },
    { "5m",  300u * 1000 },
    { "10m", 600u * 1000 },
    { "30m", 1800u * 1000 },
    { "1hr", 3600u * 1000 },
};
constexpr int NWINS = (int)(sizeof(WINS) / sizeof(WINS[0]));

/* One plot, two dBm axes reading the same four bands: transmit power down the
 * left gutter in 10 dB steps, received strength down the right in 25 dB. RX
 * needs the wider step because its range is 100 dB against TX's 40, and both
 * have to land on the same band edges for one grid to serve them. Which axis a
 * bar is on is its direction — a transmit also tints the air behind it. */
constexpr int NBANDS = 4;
struct Axis { int lo, hi; };
constexpr Axis AX_TX = { -10, 30 };
constexpr Axis AX_RX = { -130, -30 };

struct Rec { uint32_t t; uint8_t dir; uint32_t dur; uint32_t bytes; int rssi; int txp; uint8_t type; uint32_t wait; };

/* A record's protocol class, as the firmware writes it into the packed string
 * (lora.cpp's LORA_PKT_*). Colour is the protocol, not the direction —
 * direction is already the graph you are looking at. */
constexpr uint8_t PKT_RNS   = 0;   /* Reticulum traffic (yellow) */
constexpr uint8_t PKT_OURS  = 1;   /* our own air protocol (red) */
constexpr uint8_t PKT_RNODE = 2;   /* the attached RNode client (orange) */

uint16_t C_RNS, C_OURS, C_RNODE, C_BLACK, C_SEL, C_SELEDGE, C_GRID;
bool s_colorsReady = false;
void initColors() {
    if (s_colorsReady) return;
    C_BLACK   = lv_color_to_u16(lv_color_black());
    C_RNS     = lv_color_to_u16(lv_color_hex(0xE8D040));   /* Reticulum traffic (yellow) */
    C_OURS    = lv_color_to_u16(lv_color_hex(0xE84040));   /* our air protocol (red) */
    C_RNODE   = lv_color_to_u16(lv_color_hex(0xE89040));   /* the RNode client (orange) */
    C_SEL     = lv_color_to_u16(lv_color_hex(0x4A4A4A));   /* selection wash */
    C_SELEDGE = lv_color_to_u16(lv_color_hex(0xC8C8C8));
    /* The darkest tone of the band gradient, so the timescale reads as part of
     * the background rather than as something drawn over it. */
    C_GRID    = lv_color_to_u16(lv_color_hex(0x242424));
    s_colorsReady = true;
}

struct Zoom { uint32_t t0, t1; };

struct State {
    lv_obj_t* canvas = nullptr;
    lv_obj_t* tabs[8] = {};
    int       radioOf[8] = {};
    int       nTabs = 0;
    lv_obj_t* pills[NWINS] = {};
    lv_obj_t* back = nullptr;
    lv_obj_t* zoomLbl = nullptr;    /* span + timescale, beside the back pill */
    lv_obj_t* cap = nullptr;
    uint16_t* buf = nullptr;
    Rec*      recs = nullptr;
    int       n = 0;                /* records in `recs` this redraw */
    int       W = 0, H = 0, stridePx = 0;
    int       plotY = 0, plotH = 0; /* the single plot's band area */
    int       graphTop = 0;         /* first pixel row belonging to the plot */
    int       radio = 0;
    int       win = 1;              /* index into WINS */
    /* Zoom stack — each level an absolute [t0,t1] device-time span. */
    Zoom      zoom[ZOOM_MAX];
    int       nZoom = 0;
    /* Selection in progress, held as times so a still finger still widens it. */
    bool      selActive = false;
    uint32_t  selAnchor = 0, selCur = 0;
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

/* One row's tone in the band sawtooth: darkest at the band's bottom edge,
 * lightest at its top. `tx` casts the same ramp red, which is how air this
 * radio was holding is marked. */
inline uint16_t bandTone(int r, int bh, bool tx) {
    int lvl = 0x31 - ((r % bh) * (0x31 - 0x24)) / bh;
    return tx ? lv_color_to_u16(lv_color_make(lvl + 0x17, lvl - 1, lvl - 1))
              : lv_color_to_u16(lv_color_make(lvl, lvl, lvl));
}

/* Gradient bands filling the plot area between the two gutters. */
void drawBands(int y0, int h) {
    int bh = h / NBANDS; if (bh < 1) bh = 1;
    for (int r = 0; r < h; r++) {
        uint16_t gc = bandTone(r, bh, false);
        int y = y0 + r;
        if ((unsigned)y >= (unsigned)s.H) break;
        uint16_t* row = &s.buf[y * s.stridePx];
        for (int x = GUT_L; x < s.W - GUT_R; x++) row[x] = gc;
    }
}

/* Repaint one column range in the reddish cast of the same gradient: the frame's
 * time-on-air. The wait before it is channel access, not transmission, so
 * tinting that would claim airtime the radio never spent. */
void tintTx(int y0, int h, int x0, int x1) {
    int bh = h / NBANDS; if (bh < 1) bh = 1;
    if (x0 < GUT_L) x0 = GUT_L;
    if (x1 > s.W - GUT_R - 1) x1 = s.W - GUT_R - 1;
    for (int r = 0; r < h; r++) {
        uint16_t gc = bandTone(r, bh, true);
        int y = y0 + r;
        if ((unsigned)y >= (unsigned)s.H) break;
        uint16_t* row = &s.buf[y * s.stridePx];
        for (int x = x0; x <= x1; x++) row[x] = gc;
    }
}

inline double clamp01(double x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

/* Where this frame's power sits on its own dBm axis — transmit power for a tx,
 * received strength for an rx — in pixels above the bottom edge. */
int barHpx(const Rec& r, int h) {
    const Axis& a = (r.dir == 1) ? AX_TX : AX_RX;
    double dbm  = (r.dir == 1) ? (double)r.txp : (double)r.rssi;
    double norm = clamp01((dbm - a.lo) / (double)(a.hi - a.lo));
    int hp = (int)(norm * (h - 1) + 0.5);
    if (hp < 1) hp = 1;
    if (hp > h - 1) hp = h - 1;
    return hp;
}

/* The span on screen: the top of the zoom stack, else the live window. */
void view(uint32_t now, uint32_t* lo, uint32_t* hi) {
    if (s.nZoom > 0) { *lo = s.zoom[s.nZoom - 1].t0; *hi = s.zoom[s.nZoom - 1].t1; return; }
    uint32_t win = WINS[s.win].ms;
    *hi = now;
    *lo = now > win ? now - win : 0;
}

/* The plot proper: the canvas minus both scale gutters. */
inline int plotWidth() { return s.W - GUT_L - GUT_R; }

uint32_t timeAtX(uint32_t now, int x) {
    uint32_t lo, hi;
    view(now, &lo, &hi);
    int plotW = plotWidth();
    if (plotW < 1) return lo;
    if (x < GUT_L) x = GUT_L;
    if (x > GUT_L + plotW - 1) x = GUT_L + plotW - 1;
    return lo + (uint32_t)((uint64_t)(x - GUT_L) * (hi - lo) / plotW);
}

int xAtTime(uint32_t lo, uint32_t hi, uint32_t t) {
    if (hi <= lo) return GUT_L;
    if (t < lo) t = lo;
    if (t > hi) t = hi;
    return GUT_L + (int)((uint64_t)(t - lo) * plotWidth() / (hi - lo));
}

/* Timescale division for a frozen span: the smallest 1-2-5 step at least
 * DIV_PX_MIN wide. Never below 1 ms — records are ms-stamped, so a finer
 * division would draw precision the data doesn't have. */
uint32_t divStepMs(uint32_t win, int plotW) {
    if (plotW < 1) return 1;
    uint64_t want = (uint64_t)win * DIV_PX_MIN / (uint32_t)plotW;
    uint32_t dec = 1;
    for (int k = 0; k < 8; k++) {            /* 1 ms … 10 s decades cover the 1 h span */
        const uint32_t mant[3] = { 1, 2, 5 };
        for (int i = 0; i < 3; i++)
            if ((uint64_t)dec * mant[i] >= want) return dec * mant[i];
        dec *= 10;
    }
    return 3600000u;
}

/* Airtime per mille over an explicit span, computed from the records. */
int airPermille(uint32_t lo, uint32_t hi, int dir) {
    if (hi <= lo) return 0;
    uint32_t win = hi - lo;
    uint64_t busy = 0;
    for (int i = 0; i < s.n; i++) {
        if (s.recs[i].dir != (uint8_t)dir) continue;
        uint32_t st = s.recs[i].t, en = s.recs[i].t + s.recs[i].dur;
        if (en <= lo || st >= hi) continue;
        if (st < lo) st = lo;
        if (en > hi) en = hi;
        if (en > st) busy += (en - st);
    }
    return (int)(busy * 1000 / win);
}

/* storageForEach has no userdata — accumulate into the file-static `s.recs`. */
void rebuildCb(const char* key, const char* val) {
    if (s.n >= MON_MAX || !val) return;
    const char* dot = strrchr(key, '.');
    if (!dot) return;
    /* Cleared, not just filled: the buffer is reused across redraws, so a field
     * this record has no value for (an rx has no wait) would otherwise inherit
     * the last occupant's — an rx drawing some earlier transmit's channel-access
     * line. */
    Rec& r = s.recs[s.n];
    r = Rec{};
    r.t = (uint32_t)strtoul(dot + 1, nullptr, 10);
    if (val[0] == 'r') {
        int rssi, snr, dur, bytes, type = 0;
        if (sscanf(val + 2, "%d|%d|%d|%d|%d", &rssi, &snr, &dur, &bytes, &type) < 4) return;
        r.dir = 0; r.rssi = rssi; r.dur = (uint32_t)dur; r.bytes = (uint32_t)bytes;
        r.txp = 0; r.type = (uint8_t)type;
    } else if (val[0] == 't') {
        int txp, dur, bytes, type = 0, wait = 0;
        if (sscanf(val + 2, "%d|%d|%d|%d|%d", &txp, &dur, &bytes, &type, &wait) < 3) return;
        r.dir = 1; r.txp = txp; r.dur = (uint32_t)dur; r.bytes = (uint32_t)bytes;
        r.wait = (uint32_t)wait;
        r.rssi = 0; r.type = (uint8_t)type;
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

void drawGraph(uint32_t now) {
    int y0 = s.plotY, h = s.plotH;
    drawBands(y0, h);
    if (!now || h < 2) return;
    uint32_t lo, hi;
    view(now, &lo, &hi);
    if (hi <= lo) return;
    uint32_t win = hi - lo;
    int bottom = y0 + h - 1;
    int plotW = plotWidth();
    if (plotW < 2) return;
    int xmax = GUT_L + plotW - 1;
    /* Our own air first, so everything else lands on top of it. */
    for (int i = 0; i < s.n; i++) {
        const Rec& r = s.recs[i];
        if (r.dir != 1) continue;
        uint32_t st = r.t, en = r.t + r.dur;
        if (en < lo || st > hi) continue;
        uint32_t cs = st > lo ? st - lo : 0;
        uint32_t ce = (en < hi ? en : hi) - lo;
        tintTx(y0, h, GUT_L + (int)((uint64_t)cs * plotW / win),
                      GUT_L + (int)((uint64_t)ce * plotW / win));
    }
    /* Frozen view only: a live one slides, and a grid on absolute time would
     * crawl across it. Lines land on round multiples of the step and go down
     * first, so a bar always wins the pixels it shares with one. */
    if (s.nZoom > 0) {
        uint32_t step = divStepMs(win, plotW);
        uint32_t rem  = lo % step;
        /* Walked as offsets from `lo`, so a millis wrap inside the span can't
         * turn the loop bound into four billion iterations. */
        for (uint32_t d = rem ? step - rem : 0; d <= win; d += step)
            vseg(GUT_L + (int)((uint64_t)d * plotW / win), y0, bottom, C_GRID);
    }
    for (int i = 0; i < s.n; i++) {
        const Rec& r = s.recs[i];
        uint32_t st = r.t, en = r.t + r.dur;
        if (en < lo || st > hi) continue;
        uint32_t cs = st > lo ? st - lo : 0;
        uint32_t ce = (en < hi ? en : hi) - lo;
        int xs = GUT_L + (int)((uint64_t)cs * plotW / win);
        int xe = GUT_L + (int)((uint64_t)ce * plotW / win);
        if (xs < GUT_L) xs = GUT_L;
        if (xe > xmax) xe = xmax;
        if (xe < xs) xe = xs;
        int hp = barHpx(r, h);
        uint16_t col = r.type == PKT_RNODE ? C_RNODE
                     : r.type == PKT_OURS  ? C_OURS : C_RNS;
        int th = h / 20; if (th < 1) th = 1;  /* line thickness = 5% of band height */
        int yb = bottom - hp;                 /* horizontal line at the power level */
        int yt = yb - (th - 1); if (yt < y0) yt = y0;
        if (yb > bottom) yb = bottom;
        /* Time the frame sat queued before its first bit went on air: a tick at
         * the moment it joined the queue, then a hairline at mid-height running
         * up to the bar. Light enough that channel occupancy still reads as the
         * filled area alone, so a long wait can't be mistaken for airtime. */
        if (r.wait) {
            uint32_t qs = st > r.wait ? st - r.wait : 0;
            if (qs < hi && st > lo) {
                uint32_t cq = qs > lo ? qs - lo : 0;
                int xq = GUT_L + (int)((uint64_t)cq * plotW / win);
                if (xq < GUT_L) xq = GUT_L;
                if (xs - xq >= 1) {
                    int ym = yt + (yb - yt) / 2;
                    vseg(xq, yt, yb, col);                          /* when it queued */
                    for (int x = xq; x < xs; x++) px(x, ym, col);   /* how long it sat */
                }
            }
        }
        for (int x = xs; x <= xe; x++) vseg(x, yt, yb, col);
    }

    /* The live selection — the axis it picks is time. */
    if (s.selActive) {
        uint32_t a = s.selAnchor < s.selCur ? s.selAnchor : s.selCur;
        uint32_t b = s.selAnchor < s.selCur ? s.selCur : s.selAnchor;
        int xa = xAtTime(lo, hi, a), xb = xAtTime(lo, hi, b);
        for (int x = xa; x <= xb; x++)
            for (int y = y0; y <= bottom; y++)
                if (((x + y) & 1) == 0) px(x, y, C_SEL);   /* 50% wash, no blending */
        vseg(xa, y0, bottom, C_SELEDGE);
        vseg(xb, y0, bottom, C_SELEDGE);
    }
}

void clearAll() {
    int total = s.stridePx * s.H;
    for (int i = 0; i < total; i++) s.buf[i] = C_BLACK;
}

/* What we transmitted and how much of the channel was in use at all (ours +
 * theirs — the radio is half duplex, so the two never overlap), per mille.
 * The live 1-hour figures are the firmware's own rollup: this app is rarely
 * open that long, so it cannot build the hour from what it has seen. A zoomed
 * span is always computed locally. */
void airFor(uint32_t now, int* txPm, int* busyPm) {
    if (s.nZoom == 0 && strcmp(WINS[s.win].label, "1hr") == 0) {
        char k[40];
        snprintf(k, sizeof k, "lora.%d.air1h.tx", s.radio);
        int tx = storageGetInt(k, 0);
        snprintf(k, sizeof k, "lora.%d.air1h.rx", s.radio);
        *txPm = tx; *busyPm = tx + storageGetInt(k, 0);
        return;
    }
    uint32_t lo, hi;
    view(now, &lo, &hi);
    int tx = airPermille(lo, hi, 1);
    *txPm = tx; *busyPm = tx + airPermille(lo, hi, 0);
}

void spanLabel(char* b, size_t n, uint32_t ms) {
    if (ms < 1000)          snprintf(b, n, "%ums", (unsigned)ms);
    else if (ms < 60000)    snprintf(b, n, "%u.%us", (unsigned)(ms / 1000), (unsigned)(ms % 1000) / 100);
    else if (ms < 3600000)  snprintf(b, n, "%umin", (unsigned)(ms / 60000));
    else                    snprintf(b, n, "%uh", (unsigned)(ms / 3600000));
}

void drawAll() {
    if (!s.canvas || !s.buf || !s.recs) return;
    clearAll();
    rebuild();
    uint32_t now = millis();
    uint32_t lo, hi;
    view(now, &lo, &hi);
    drawGraph(now);
    /* The frozen view's span and what one division of its grid is worth. It
     * rides the pill strip beside the back pill, where the window pills leave
     * the room — the caption below has none to spare. */
    if (s.zoomLbl) {
        if (s.nZoom > 0) {
            char span[16], st[16], b[40];
            spanLabel(span, sizeof span, hi - lo);
            spanLabel(st, sizeof st, divStepMs(hi - lo, plotWidth()));
            snprintf(b, sizeof b, "%s  %s/div", span, st);
            lv_label_set_text(s.zoomLbl, b);
            lv_obj_remove_flag(s.zoomLbl, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s.zoomLbl, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (s.cap) {
        int tx = 0, busy = 0;
        airFor(now, &tx, &busy);
        /* Sized for the whole caption including all three legend entries:
         * a truncated recolor string shows as a silently missing legend, not
         * as an error. */
        char b[128];
        snprintf(b, sizeof b,
                 "tx %d.%d%%  busy %d.%d%%  #E8D040 rnsd# #E89040 rnode# #E84040 rfprobe#",
                 tx / 10, tx % 10, busy / 10, busy % 10);
        lv_label_set_text(s.cap, b);
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
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, GUT_L + 2, y);
    return l;
}

/* Band-edge labels down one gutter, so a bar's height reads as dBm. Transmit
 * power on the left, received strength on the right; four bands either way, so
 * the two scales share every edge. */
void mkScale(lv_obj_t* root, const Axis& a, bool left) {
    int h = s.plotH;
    for (int i = 0; i <= NBANDS; i++) {
        int y = s.plotY + h - (i * h) / NBANDS - 4;
        if (i == 0)      y -= 3;                /* keep the end labels on-screen */
        if (i == NBANDS) y += 3;
        lv_obj_t* l = lv_label_create(root);
        char b[8];
        snprintf(b, sizeof b, "%d", a.lo + i * (a.hi - a.lo) / NBANDS);
        lv_label_set_text(l, b);
        lv_obj_set_style_text_font(l, lcdFont(LcdFace::MONO, 8), 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0x8A8A8A), 0);
        lv_obj_set_width(l, (left ? GUT_L : GUT_R) - 3);
        lv_obj_set_style_text_align(l, left ? LV_TEXT_ALIGN_RIGHT : LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_align(l, LV_ALIGN_TOP_LEFT, left ? 0 : s.W - GUT_R + 3, y);
    }
}

/* The axis name, standing over the gutter it labels in the pill strip. */
void mkAxisName(lv_obj_t* root, const char* text, int y, bool left) {
    lv_obj_t* l = lv_label_create(root);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, lcdFont(LcdFace::MONO, 8), 0);
    lv_obj_set_style_text_color(l, lv_color_hex(0xC8C8C8), 0);
    lv_obj_set_width(l, (left ? GUT_L : GUT_R) - 3);
    lv_obj_set_style_text_align(l, left ? LV_TEXT_ALIGN_RIGHT : LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, left ? 0 : s.W - GUT_R + 3, y);
}

void setTab(int t);
void tabEventCb(lv_event_t* e) { setTab((int)(intptr_t)lv_event_get_user_data(e)); }

void setWin(int w);
void pillEventCb(lv_event_t* e) { setWin((int)(intptr_t)lv_event_get_user_data(e)); }

void showPills();

void zoomOut() {
    if (s.nZoom > 0) s.nZoom--;
    showPills();
    drawAll();
}
void backEventCb(lv_event_t*) { zoomOut(); }

/* Touch on the graph: press anchors a time, dragging (or simply holding, on a
 * live graph) widens it, release zooms. */
void canvasEventCb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t* indev = lv_indev_active();
    if (!indev) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);
    uint32_t now = millis();

    if (code == LV_EVENT_PRESSED) {
        if (p.y < s.graphTop) return;           /* tabs / pills strip */
        s.selActive = true;
        s.selAnchor = s.selCur = timeAtX(now, p.x);
        drawAll();
    } else if (code == LV_EVENT_PRESSING) {
        if (!s.selActive) return;
        s.selCur = timeAtX(now, p.x);
        drawAll();
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if (!s.selActive) return;
        s.selActive = false;
        uint32_t a = s.selAnchor < s.selCur ? s.selAnchor : s.selCur;
        uint32_t b = s.selAnchor < s.selCur ? s.selCur : s.selAnchor;
        uint32_t lo, hi;
        view(now, &lo, &hi);
        /* A tap that never widened (a static, already-zoomed view) is not a
         * zoom — 1% of the span is the floor. */
        if (b - a >= (hi - lo) / 100 && b > a && s.nZoom < ZOOM_MAX) {
            s.zoom[s.nZoom].t0 = a;
            s.zoom[s.nZoom].t1 = b;
            s.nZoom++;
            showPills();
        }
        drawAll();
    }
}

void styleTabs(int active) {
    for (int i = 0; i < s.nTabs; i++)
        if (s.tabs[i])
            lv_obj_set_style_bg_color(s.tabs[i],
                lv_color_hex(i == active ? 0x383838 : 0x202020), 0);
}

void stylePills(int active) {
    for (int i = 0; i < NWINS; i++)
        if (s.pills[i])
            lv_obj_set_style_bg_color(s.pills[i],
                lv_color_hex(i == active ? 0x3A3A3A : 0x181818), 0);
}

/* Zoomed, the moving-window pills are meaningless — only the way back out. */
void showPills() {
    bool z = s.nZoom > 0;
    for (int i = 0; i < NWINS; i++)
        if (s.pills[i]) {
            if (z) lv_obj_add_flag(s.pills[i], LV_OBJ_FLAG_HIDDEN);
            else   lv_obj_remove_flag(s.pills[i], LV_OBJ_FLAG_HIDDEN);
        }
    if (s.back) {
        if (z) lv_obj_remove_flag(s.back, LV_OBJ_FLAG_HIDDEN);
        else   lv_obj_add_flag(s.back, LV_OBJ_FLAG_HIDDEN);
        /* Blue = one level of zoom left, so pressing it returns to the live
         * window. Grey means another frozen view stands behind this one. */
        lv_obj_set_style_bg_color(s.back,
            lv_color_hex(s.nZoom == 1 ? 0x2B5CC4 : 0x181818), 0);
    }
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

lv_obj_t* mkPill(lv_obj_t* root, const char* label, int idx, int x, int y, int w,
                 lv_event_cb_t cb) {
    lv_obj_t* b = lv_obj_create(root);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, w - 3, PILLH - 2);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x181818), 0);
    lv_obj_set_style_radius(b, (PILLH - 2) / 2, 0);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, (void*)(intptr_t)idx);
    lv_obj_t* l = lv_label_create(b);
    lv_label_set_text(l, label);
    lv_obj_set_style_text_font(l, lcdFont(LcdFace::MONO, 8), 0);
    lv_obj_set_style_text_color(l, lv_color_hex(0xC8C8C8), 0);
    lv_obj_center(l);
    return b;
}

void setTab(int t) {
    if (t < 0 || t >= s.nTabs) return;
    s.radio = s.radioOf[t];
    s.nZoom = 0;
    s.selActive = false;
    styleTabs(t);
    showPills();
    drawAll();
}

void setWin(int w) {
    if (w < 0 || w >= NWINS) return;
    s.win = w;
    stylePills(w);
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
    lv_obj_add_flag(s.canvas, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s.canvas, canvasEventCb, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(s.canvas, canvasEventCb, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(s.canvas, canvasEventCb, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(s.canvas, canvasEventCb, LV_EVENT_PRESS_LOST, nullptr);

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

    /* The pills live between the gutters: each gutter's own name stands above
     * it, so the scale below reads as tx on the left and rx on the right. */
    int pw = (W - GUT_L - GUT_R) / NWINS;
    for (int i = 0; i < NWINS; i++)
        s.pills[i] = mkPill(root, WINS[i].label, i, GUT_L + i * pw, top + 1, pw, pillEventCb);
    /* ASCII, not LV_SYMBOL_LEFT: the pills are set in the 8 px mono face, which
     * carries no symbol glyphs — the arrow would render as a blank pill. */
    s.back = mkPill(root, "<", 0, GUT_L, top + 1, pw, backEventCb);
    s.zoomLbl = lv_label_create(root);
    lv_label_set_text(s.zoomLbl, "");
    lv_obj_set_style_text_font(s.zoomLbl, lcdFont(LcdFace::MONO, 8), 0);
    lv_obj_set_style_text_color(s.zoomLbl, lv_color_hex(0x7A7A7A), 0);
    lv_obj_align(s.zoomLbl, LV_ALIGN_TOP_LEFT, GUT_L + pw + 4, top + 5);
    lv_obj_add_flag(s.zoomLbl, LV_OBJ_FLAG_HIDDEN);
    mkAxisName(root, "tx", top + 5, true);
    mkAxisName(root, "rx", top + 5, false);
    stylePills(s.win);
    showPills();
    top += PILLH + PAD_PILLS;
    s.graphTop = top;

    s.plotY = top;
    s.plotH = H - top;               /* one plot now: it takes what is left */
    s.cap   = mkCaption(root, s.plotY + 1, "");
    mkScale(root, AX_TX, true);
    mkScale(root, AX_RX, false);

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
