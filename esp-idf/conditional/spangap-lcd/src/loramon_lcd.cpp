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
 *   −128…0 dBm down the right in 32 dB. Colour is direction and audience: red
 *   is what this radio put on the air, blue what arrived, light for a broadcast
 *   anyone may read and dark for a frame with one addressee. Grey is somebody
 *   else's unicast — on the air, but not our conversation — and purple a frame
 *   whose CRC failed, which is neither, since nothing in it was readable.
 *
 *   Touching a frame prints what it is and who it was with along the top of the
 *   plot; the hit test runs across every channel lane at that instant, so a
 *   train is inspectable wherever it is on screen.
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
 * `lora.<n>.packets.<ms>` = "r|rssi|snr|dur|bytes|type|ch|desc|cast[|tag]" (rx)
 * / "t|txp|dur|bytes|type|wait|ch|own|desc|cast[|tag]" (tx) / "a|ch|dur" (a
 * dwell — where the radio was), and deletes them past 1 h. We rebuild our view
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
#include "lora.h"     /* loraNameForTag — the peer table, not a published copy */
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

/* The agile-channel strips under the hailing plot. Small by design: on this
 * screen a lane's job is to say *when* the radio was there and whether anything
 * landed, not to be read for dBm — the main plot carries the scale. Below
 * LORAMON_LANE_MIN_H a strip stops saying anything, so strips are dropped from
 * the bottom until the rest fit rather than squeezing everything. */
constexpr int LORAMON_LANE_H     = 7;    /* one agile strip, px */
constexpr int LORAMON_LANE_MIN_H  = 6;   /* below this a strip says nothing legible */
constexpr int LORAMON_MAX_LANES  = 9;    /* the largest regime's agile set */
constexpr int LORAMON_LANE_PX    = 400;  /* widest plot the attendance mask covers */
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
 * left gutter in 10 dB steps, received strength down the right in 32 dB. RX
 * needs the wider step because its range is 128 dB against TX's 40, and both
 * have to land on the same band edges for one grid to serve them. Which axis a
 * bar is on is its direction — a transmit also tints the air behind it.
 *
 * The RX axis is the reportable range itself, not a guess at a comfortable one:
 * the packet-strength register is a byte read as −value/2 dBm, so 0 is the
 * strongest level a receiver can state and −127.5 the weakest. A bench pair a
 * hand apart reads well above −30, and an axis that stopped there put every one
 * of those in the top band with nothing to tell them apart. */
constexpr int NBANDS = 4;
struct Axis { int lo, hi; };
constexpr Axis AX_TX = { -10, 30 };
constexpr Axis AX_RX = { -128, 0 };

/* dir: 0 rx, 1 tx, 2 a DWELL — the radio was tuned to `ch` and listening for
 * `dur`. A dwell carries no level and is never drawn as a bar; it is what tells
 * a lane the radio was watching from a lane it had left. */
struct Rec { uint32_t t; uint8_t dir; uint32_t dur; uint32_t bytes; int rssi; int txp; uint8_t type; uint32_t wait; uint8_t ch; uint8_t desc; uint8_t cast; char tag[7]; };

/* What a frame is, by the code the device writes into the record — the same
 * table the browser keeps, kept short because this one is read on a strip of
 * screen a few characters wide. Codes are appended to, never renumbered. */
static const char* const kDesc[] = {
    "", "PRIVSYNC", "ANNOUNCE2", "HAVEDATA", "GIMME", "THATSIT", "BYE",
    "RESEND", "data", "announce", "link req", "proof", "split", "RNode",
};
static const char* descOf(uint8_t d) {
    return d < (uint8_t)(sizeof kDesc / sizeof kDesc[0]) ? kDesc[d] : "";
}

/* A record's class as the firmware writes it into the packed string: only a
 * CRC failure is read from it, since colour says direction and audience and a
 * frame that failed its CRC has neither — nothing in it was readable. */
constexpr uint8_t PKT_BAD = 3;

/* Who a frame was aimed at, as the device decided it (lora_mon.h's LMC_*). The
 * screen cannot work this out for itself: it turns on which addresses mean US,
 * and that lives in the peer table. */
constexpr uint8_t CAST_BCAST   = 0;
constexpr uint8_t CAST_US      = 1;
/* Ours by a link identifier. Same colour as any other frame of ours — the
 * distinction it draws is about naming the far end, which this screen does not
 * do — but it has to be spelt out, or our own link traffic reads as overheard. */
constexpr uint8_t CAST_US_LINK = 3;

uint16_t C_TX_BCAST, C_TX_UNI, C_RX_BCAST, C_RX_US, C_RX_OTHER, C_BAD;
uint16_t C_BLACK, C_SEL, C_SELEDGE, C_GRID, C_FLOOR, C_BEZEL;
bool s_colorsReady = false;
void initColors() {
    if (s_colorsReady) return;
    C_BLACK   = lv_color_to_u16(lv_color_black());
    /* Red is what we put on the air, blue is what arrived; light is a broadcast
     * anyone may read, dark a frame with one addressee. Grey is somebody else's
     * unicast — on the air, but not our conversation. */
    C_TX_BCAST = lv_color_to_u16(lv_color_hex(0xF08080));
    C_TX_UNI   = lv_color_to_u16(lv_color_hex(0xB02020));
    C_RX_BCAST = lv_color_to_u16(lv_color_hex(0x80B8F0));
    C_RX_US    = lv_color_to_u16(lv_color_hex(0x2060C0));
    C_RX_OTHER = lv_color_to_u16(lv_color_hex(0x8A8A8A));
    C_BAD      = lv_color_to_u16(lv_color_hex(0x8050C8));  /* CRC failure */
    C_SEL     = lv_color_to_u16(lv_color_hex(0x4A4A4A));   /* selection wash */
    C_SELEDGE = lv_color_to_u16(lv_color_hex(0xC8C8C8));
    /* The darkest tone of the band gradient, so the timescale reads as part of
     * the background rather than as something drawn over it. */
    C_GRID    = lv_color_to_u16(lv_color_hex(0x242424));
    C_FLOOR   = lv_color_to_u16(lv_color_hex(0x4A4A4A));   /* the noise-floor line */
    /* The frame around each lane's plot area. Light enough to stay background,
     * solid enough to survive a lane with nothing in it. */
    C_BEZEL   = lv_color_to_u16(lv_color_hex(0x5A5A5A));
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
    /* Floating labels over the canvas: the noise floor's figure sitting on its
     * line, the description of whatever frame was last tapped, and a small pool
     * naming the peer above a run of frames. Labels rather than pixels because
     * this canvas has no glyph blitter, and adding one to print a few short
     * strings would be the wrong trade.
     *
     * The pool is fixed and small on purpose. A run has to be wide enough to
     * hold its own name before it gets one, so a view showing more than a
     * handful of nameable runs is one where they would be shoulder to shoulder
     * and unreadable anyway; the widest get the labels. */
    lv_obj_t* nfLbl = nullptr;
    lv_obj_t* insLbl = nullptr;
    static constexpr int TRAIN_LBL_MAX = 4;
    lv_obj_t* trainLbl[TRAIN_LBL_MAX] = {};
    uint16_t* buf = nullptr;
    Rec*      recs = nullptr;
    int       n = 0;                /* records in `recs` this redraw */
    int       W = 0, H = 0, stridePx = 0;
    int       plotY = 0, plotH = 0; /* the single plot's band area */
    int       graphTop = 0;         /* first pixel row belonging to the plot */
    int       radio = 0;
    int       nChans = 1;           /* hailing + the regime's agile channels */
    int       win = 1;              /* index into WINS */
    /* Zoom stack — each level an absolute [t0,t1] device-time span. */
    Zoom      zoom[ZOOM_MAX];
    int       nZoom = 0;
    /* Selection in progress, held as times so a still finger still widens it. */
    bool      selActive = false;
    uint32_t  selAnchor = 0, selCur = 0;
    /* The hailing channel's recent readings, for the noise floor. A minute at
     * the publish beat is a handful of points, so a small ring is the whole
     * store — this is a statement about conditions now, not history. */
    static constexpr int NF_MAX = 96;
    uint32_t  nfT[NF_MAX] = {};
    int16_t   nfDbm[NF_MAX] = {};
    uint8_t   nfN = 0, nfHead = 0;
    uint32_t  nfLastMs = 0;
    /* A tap that did not become a zoom leaves this: the instant to describe. */
    bool      inspect = false;
    uint32_t  inspectT = 0;
    /* The name behind the inspected frame's tag, resolved once per tag rather
     * than per redraw: the answer cannot change while a tap stands, and the
     * lookup walks a storage subtree. */
    char      insTag[7] = "";
    char      insName[40] = "";
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
 * lightest at its top. */
inline uint16_t bandTone(int r, int bh) {
    int lvl = 0x31 - ((r % bh) * (0x31 - 0x24)) / bh;
    return lv_color_to_u16(lv_color_make(lvl, lvl, lvl));
}

/* Gradient bands filling the plot area between the two gutters. Nothing tints
 * the field behind a frame: direction lives in the bar's own colour, and
 * repainting the bands under every transmit to repeat it costs the whole
 * strip's contrast. The background is one thing. */
void drawBands(int y0, int h) {
    int bh = h / NBANDS; if (bh < 1) bh = 1;
    for (int r = 0; r < h; r++) {
        uint16_t gc = bandTone(r, bh);
        int y = y0 + r;
        if ((unsigned)y >= (unsigned)s.H) break;
        uint16_t* row = &s.buf[y * s.stridePx];
        for (int x = GUT_L; x < s.W - GUT_R; x++) row[x] = gc;
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

/* How far behind "now" the live edge sits. Past the 1 Hz publish beat with room
 * for the batch a burst is flushed in, so what reaches the right edge is settled
 * and the graph never corrects itself in front of you: a stay on one channel is
 * one record that GROWS, so the background would otherwise arrive a step at a
 * time, and frame records are batched, so a packet would land in a column
 * already drawn as empty. A second and a half of latency on a plot whose
 * shortest window is ten seconds is not a cost anybody is watching for. */
constexpr uint32_t LAG_MS = 1500;

/* The span on screen: the top of the zoom stack, else the live window — which
 * ends LAG_MS ago, not at `now`. */
void view(uint32_t now, uint32_t* lo, uint32_t* hi) {
    if (s.nZoom > 0) { *lo = s.zoom[s.nZoom - 1].t0; *hi = s.zoom[s.nZoom - 1].t1; return; }
    uint32_t win = WINS[s.win].ms;
    now = now > LAG_MS ? now - LAG_MS : 0;
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
        int rssi, snr, dur, bytes, type = 0, ch = 0, desc = 0, cast = 0;
        char tg[8] = "";
        if (sscanf(val + 2, "%d|%d|%d|%d|%d|%d|%d|%d|%7s",
                   &rssi, &snr, &dur, &bytes, &type, &ch, &desc, &cast, tg) < 4) return;
        safeStrncpy(r.tag, tg, sizeof r.tag);
        r.dir = 0; r.rssi = rssi; r.dur = (uint32_t)dur; r.bytes = (uint32_t)bytes;
        r.txp = 0; r.type = (uint8_t)type; r.ch = (uint8_t)ch; r.desc = (uint8_t)desc;
        r.cast = (uint8_t)cast;
    } else if (val[0] == 't') {
        int txp, dur, bytes, type = 0, wait = 0, ch = 0, own = 0, desc = 0, cast = 0;
        char tg[8] = "";
        if (sscanf(val + 2, "%d|%d|%d|%d|%d|%d|%d|%d|%d|%7s",
                   &txp, &dur, &bytes, &type, &wait, &ch, &own, &desc, &cast, tg) < 3) return;
        safeStrncpy(r.tag, tg, sizeof r.tag);
        r.dir = 1; r.txp = txp; r.dur = (uint32_t)dur; r.bytes = (uint32_t)bytes;
        r.wait = (uint32_t)wait;
        r.rssi = 0; r.type = (uint8_t)type; r.ch = (uint8_t)ch; r.desc = (uint8_t)desc;
        r.cast = (uint8_t)cast;
    } else if (val[0] == 'a') {
        int ch = 0, dur = 0;
        if (sscanf(val + 2, "%d|%d", &ch, &dur) < 2) return;
        r.dir = 2; r.ch = (uint8_t)ch; r.dur = (uint32_t)dur;
    } else return;
    s.n++;
}

/* The newest channel-RSSI sweep for the hailing channel, appended if it is one
 * we have not seen. Keyed on the device timestamp leading the value, so a
 * repeat publish of an unchanged key adds nothing. */
void pollFloor() {
    char k[32]; snprintf(k, sizeof k, "lora.%d.rssi", s.radio);
    char v[24 * 12];
    storageGetStr(k, v, sizeof v, "");
    if (!v[0]) return;
    char* end = nullptr;
    uint32_t t = (uint32_t)strtoul(v, &end, 10);
    if (!end || *end != '|' || t == s.nfLastMs) return;
    s.nfLastMs = t;
    int dbm = (int)strtol(end + 1, nullptr, 10);      /* field 1 = channel 0 */
    if (!dbm) return;                                 /* no reading this beat */
    int slot = (s.nfHead + s.nfN) % State::NF_MAX;
    if (s.nfN < State::NF_MAX) s.nfN++;
    else s.nfHead = (uint8_t)((s.nfHead + 1) % State::NF_MAX);
    s.nfT[slot] = t;
    s.nfDbm[slot] = (int16_t)dbm;
}

/* The quietest reading the hailing channel has given in the last minute. A
 * minute rather than the window on screen, because this describes conditions
 * now — zooming out to an hour must not turn it into the quietest moment of
 * that hour — and the quietest rather than a mean, because a mean over a
 * channel carrying traffic measures the traffic.
 *
 * Two kinds of non-reading are dropped, and dropping them is the whole reason
 * this returns false rather than a number: a floor nobody can vouch for is worse
 * than none.
 *   - a zero is a beat that produced no reading at all;
 *   - anything at the register's weakest rail is the RAIL, not the channel. The
 *     level is a byte read as −value/2 dBm, so −127.5 is as quiet as it can say,
 *     and it says that whenever the front end has nothing yet — which is exactly
 *     the first minutes after a radio comes up. It cannot be a measurement
 *     either: thermal noise alone is about −123 dBm in 125 kHz before any
 *     receiver's own noise figure, so no working front end reads below it. */
constexpr int NOISE_RAIL_DBM = -127;
bool noiseFloor(int* out) {
    if (!s.nfN) return false;
    uint32_t newest = s.nfT[(s.nfHead + s.nfN - 1) % State::NF_MAX];
    int lo = 0; bool any = false;
    for (int i = 0; i < s.nfN; i++) {
        int j = (s.nfHead + i) % State::NF_MAX;
        if (newest - s.nfT[j] > 60u * 1000) continue;
        if (!s.nfDbm[j] || s.nfDbm[j] <= NOISE_RAIL_DBM) continue;
        if (!any || s.nfDbm[j] < lo) { lo = s.nfDbm[j]; any = true; }
    }
    if (any) *out = lo;
    return any;
}

/* How many channels the regime in force puts up, from `lora.<n>.chans` —
 * "<hail freq>,<bw>|<agile freq>,<bw>|…". Only the count is wanted here; the
 * frequencies belong to the settings panel. */
void readChans() {
    char k[32]; snprintf(k, sizeof k, "lora.%d.chans", s.radio);
    char v[24 * 12];
    storageGetStr(k, v, sizeof v, "");
    int n = v[0] ? 1 : 1;
    for (const char* p2 = v; *p2; p2++) if (*p2 == '|') n++;
    s.nChans = n;
}

void rebuild() {
    s.n = 0;
    readChans();
    if (!s.recs) return;
    char pfx[32];
    snprintf(pfx, sizeof pfx, "lora.%d.packets.", s.radio);
    storageForEach(pfx, rebuildCb);
}

/* One lane: the hailing plot at full height, or an agile channel's strip under
 * it. Every lane spans the same x and the same window, so a column is the same
 * moment in all of them — which is what makes a stack of them readable at this
 * size, and why the strips carry no scale of their own. */
void drawLane(uint32_t now, int y0, int h, uint8_t ch, bool main) {
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

    /* Where the radio was not on this channel, the lane goes dark. A lane with
     * nothing in it otherwise answers two questions the same way — nobody
     * spoke, or we were listening elsewhere — and on a hopping radio the second
     * is the usual one. Painted first, so traffic and the grid land on top. */
    {
        static uint8_t held[LORAMON_LANE_PX];
        int npx = plotW < LORAMON_LANE_PX ? plotW : LORAMON_LANE_PX;
        memset(held, 0, (size_t)npx);
        for (int i = 0; i < s.n; i++) {
            const Rec& r = s.recs[i];
            if (r.dir != 2 || r.ch != ch) continue;
            uint32_t st = r.t, en = r.t + r.dur;
            if (en < lo || st > hi) continue;
            uint32_t cs = st > lo ? st - lo : 0;
            uint32_t ce = (en < hi ? en : hi) - lo;
            int xa = (int)((uint64_t)cs * plotW / win);
            int xb = (int)((uint64_t)ce * plotW / win);
            if (xa < 0) xa = 0;
            if (xb >= npx) xb = npx - 1;
            for (int x = xa; x <= xb; x++) held[x] = 1;
        }
        for (int x = 0; x < npx; x++)
            if (!held[x]) vseg(GUT_L + x, y0, bottom, C_BLACK);
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
        if (r.dir == 2 || r.ch != ch) continue;   /* dwells are background */
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
        uint16_t col = r.type == PKT_BAD          ? C_BAD
                     : r.dir == 1                 ? (r.cast == CAST_BCAST ? C_TX_BCAST : C_TX_UNI)
                     : r.cast == CAST_BCAST       ? C_RX_BCAST
                     : (r.cast == CAST_US ||
                        r.cast == CAST_US_LINK)   ? C_RX_US : C_RX_OTHER;
        /* Never thinner than two pixels: on an agile strip the proportional
         * term falls below one, and a frame that rounds away is a frame the
         * graph is lying about. */
        int th = h / 20; if (th < 2) th = 2;
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

    /* The live selection — the axis it picks is time. Drawn on every lane: the
     * stack shares one axis, so the band is the same column throughout. */
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

    (void)main;
}

/* The lane's own outline, drawn over everything it encloses so nothing can eat
 * an edge. A lane the radio never visited is veiled end to end and has no
 * traffic to give it shape, so without this it is a rectangle of dark against a
 * dark screen — indistinguishable from the gap between two lanes, and from no
 * lane at all. The bezel is what says a graph is there and empty rather than
 * absent, which is exactly the case drawLane returns early on: the caller draws
 * it, so an empty lane gets its frame whether or not there was anything to
 * plot. */
void drawBezel(int y0, int h) {
    if (h < 2) return;
    int bottom = y0 + h - 1, xl = GUT_L, xr = s.W - GUT_R - 1;
    if (xr <= xl) return;
    for (int x = xl; x <= xr; x++) { px(x, y0, C_BEZEL); px(x, bottom, C_BEZEL); }
    vseg(xl, y0, bottom, C_BEZEL);
    vseg(xr, y0, bottom, C_BEZEL);
}

/* Who a run of frames was with, named once above the run.
 *
 * Every frame of a SUPE train carries the same tag, and a single frame on the
 * hailing channel is that run at length one — so one rule covers both. Frames
 * on a lane sharing a tag are one run; a gap longer than the widest schedule
 * spacing ends it, because past that they are two conversations that happened
 * to be with the same node.
 *
 * The pill sits BESIDE the run — level with its top, a few pixels off its right
 * end — and never over it. Centred, its offset from the frames it names changes
 * with the run's width, so the same train's label lands somewhere different
 * every time the view moves; anchored to one end it stays put.
 *
 * A run is named only where it is at least as wide as the name and the pill has
 * room before the right gutter, so the labels come and go as the view zooms: at
 * a width where the text would dwarf what it labels, it is pointing at the wrong
 * traffic. With more nameable runs than labels the widest win — the rest are
 * shoulder to shoulder at that zoom and would be unreadable regardless.
 *
 * Just past the widest slot spacing a schedule ever asks for, so a train stays
 * whole across its longest legitimate gap. */
constexpr uint32_t TRAIN_GAP_MS = 400;
/* Pill geometry, shared with the noise floor's figure: one style for every note
 * stuck on the graph. The character width is the 8 px mono face's, measured
 * here rather than through LVGL, which would mean creating a label per
 * candidate before knowing which candidates win. */
constexpr int PILL_CH_W  = 5;
constexpr int PILL_PAD   = 6;   /* the pill's own horizontal padding, both sides */
constexpr int PILL_GAP   = 4;   /* between the pill and whatever it sits beside */

struct TrainPill { int x0, x1, y; char name[24]; };

void drawTrainPills(uint32_t now, int nAgile, int mainH, int laneH) {
    uint32_t lo, hi; view(now, &lo, &hi);
    TrainPill best[State::TRAIN_LBL_MAX];
    int nBest = 0;
    /* One lane at a time, and inside a lane in time order: the records arrive in
     * whatever order the storage subtree walks, and a run is an ordering fact.
     * Sorting an index rather than the records keeps this to a few dozen
     * two-byte swaps instead of moving the record array around in PSRAM. */
    static constexpr int IDX_MAX = 256;
    int16_t idx[IDX_MAX];
    for (int ch = 0; ch <= nAgile; ch++) {
        int y = ch == 0 ? s.plotY : s.plotY + mainH + (ch - 1) * laneH;
        int ni = 0;
        for (int i = 0; i < s.n && ni < IDX_MAX; i++) {
            const Rec& r = s.recs[i];
            if (r.dir == 2 || r.ch != ch || !r.tag[0]) continue;
            if (r.t + r.dur < lo || r.t > hi) continue;
            int j = ni++;
            while (j > 0 && s.recs[idx[j - 1]].t > r.t) { idx[j] = idx[j - 1]; j--; }
            idx[j] = (int16_t)i;
        }
        const char* runTag = nullptr;
        int x0 = 0, x1 = 0;
        uint32_t endMs = 0;
        auto flush = [&]() {
            if (!runTag) return;
            const char* tag = runTag;
            runTag = nullptr;
            /* The name, not the tag, decides the fit: it is what gets drawn, and
             * a six-character measurement would let a long name spill past the
             * frames it belongs to. */
            TrainPill p{ x0, x1, y, "" };
            loraNameForTag(s.radio, tag, p.name, sizeof p.name);
            if (!p.name[0]) safeStrncpy(p.name, tag, sizeof p.name);
            int wpx = (int)strlen(p.name) * PILL_CH_W + PILL_PAD * 2;
            if (x1 - x0 < wpx) return;
            if (x1 + PILL_GAP + wpx > s.W - GUT_R) return;   /* no room beside it */
            /* Widest wins: keep the pool sorted by width, drop the narrowest
             * once it is full. */
            int at = nBest;
            while (at > 0 && (best[at - 1].x1 - best[at - 1].x0) < (x1 - x0)) at--;
            if (at >= State::TRAIN_LBL_MAX) return;
            int last = nBest < State::TRAIN_LBL_MAX ? nBest : State::TRAIN_LBL_MAX - 1;
            for (int k = last; k > at; k--) best[k] = best[k - 1];
            best[at] = p;
            if (nBest < State::TRAIN_LBL_MAX) nBest++;
        };
        for (int k = 0; k < ni; k++) {
            const Rec& r = s.recs[idx[k]];
            if (!runTag || strcmp(r.tag, runTag) != 0 ||
                (r.t > endMs && r.t - endMs > TRAIN_GAP_MS)) {
                flush();
                runTag = r.tag; x0 = xAtTime(lo, hi, r.t);
            }
            x1 = xAtTime(lo, hi, r.t + r.dur); endMs = r.t + r.dur;
        }
        flush();
    }
    for (int i = 0; i < State::TRAIN_LBL_MAX; i++) {
        if (i >= nBest) { lv_obj_add_flag(s.trainLbl[i], LV_OBJ_FLAG_HIDDEN); continue; }
        lv_label_set_text(s.trainLbl[i], best[i].name);
        lv_obj_align(s.trainLbl[i], LV_ALIGN_TOP_LEFT,
                     best[i].x1 + PILL_GAP, best[i].y + 1);
        lv_obj_clear_flag(s.trainLbl[i], LV_OBJ_FLAG_HIDDEN);
    }
}

/* The hailing plot, then one strip per agile channel of the regime in force. */
void drawGraph(uint32_t now) {
    int nAgile = s.nChans > 1 ? s.nChans - 1 : 0;
    if (nAgile > LORAMON_MAX_LANES) nAgile = LORAMON_MAX_LANES;
    /* The strips take a fixed slice each and the hailing plot keeps the rest,
     * with a floor under it: below that the bands stop being readable and a
     * stack of unreadable lanes is worse than no lanes. */
    /* The hailing plot is TWICE a strip — no more. It carries the same kind of
     * information as its neighbours, and a lane many times their height says it
     * matters many times as much, which is not what a monitor is for. So the
     * plot divides into (agile + 2) equal parts and hailing takes two of them;
     * lanes are dropped from the bottom only when a strip would fall below
     * legibility, not to keep hailing large. */
    int nLanes = nAgile + 2;
    while (nAgile && s.plotH / (nAgile + 2) < LORAMON_LANE_MIN_H) { nAgile--; nLanes = nAgile + 2; }
    int laneH = nAgile ? s.plotH / nLanes : 0;
    int mainH = s.plotH - nAgile * laneH;      /* the rounding remainder rides here */
    drawLane(now, s.plotY, mainH, 0, true);
    drawBezel(s.plotY, mainH);
    /* The noise floor: one dotted line at the quietest the channel has been
     * lately, with its figure sitting ON the line, so the number and the level
     * it describes are one object — at the right end of it, where the rx axis it
     * is quoting runs, held clear of the scale itself. Same pill as a train's
     * name: both are notes stuck on the graph, and two label styles for one job
     * is one too many. */
    int nf = 0;
    if (noiseFloor(&nf)) {
        int y = s.plotY + mainH - 1
              - (int)((long)(nf - AX_RX.lo) * (mainH - 1) / (AX_RX.hi - AX_RX.lo));
        if (y < s.plotY) y = s.plotY;
        if (y >= s.plotY + mainH - 1) y = s.plotY + mainH - 2;
        /* Two rows, not one: at this size a single-pixel dotted line reads as
         * dirt on the glass rather than as a level. Between the gutters and
         * inside the bezel — the line is a level ON the plot, and running it
         * through the scales strikes out the very numbers it is read against. */
        for (int x = GUT_L + 1; x < s.W - GUT_R - 1; x += 3) {
            px(x, y, C_FLOOR);
            px(x, y + 1, C_FLOOR);
        }
        char b[16];
        int bn = snprintf(b, sizeof b, "%d dBm", nf);
        int wpx = bn * PILL_CH_W + PILL_PAD * 2;
        int x = s.W - GUT_R - PILL_GAP - wpx;
        if (x < GUT_L) x = GUT_L;
        /* Kept whole inside the lane even where the line runs along an edge:
         * half a pill is unreadable, and the line already says which level it
         * belongs to. */
        int py = y - 5;
        if (py < s.plotY) py = s.plotY;
        if (py + 11 > s.plotY + mainH) py = s.plotY + mainH - 11;
        lv_label_set_text(s.nfLbl, b);
        lv_obj_align(s.nfLbl, LV_ALIGN_TOP_LEFT, x, py);
        lv_obj_clear_flag(s.nfLbl, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s.nfLbl, LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < nAgile; i++) {
        drawLane(now, s.plotY + mainH + i * laneH, laneH, (uint8_t)(i + 1), false);
        drawBezel(s.plotY + mainH + i * laneH, laneH);
    }

    drawTrainPills(now, nAgile, mainH, laneH);

    /* What the tapped instant holds, if anything. Searched across every lane
     * that is on screen, since the tap named a moment rather than a lane. */
    const Rec* hit = nullptr;
    if (s.inspect) {
        for (int i = 0; i < s.n; i++) {
            const Rec& r = s.recs[i];
            if (r.dir == 2) continue;
            if (r.ch != 0 && (r.ch - 1) >= nAgile) continue;
            if (s.inspectT >= r.t && s.inspectT <= r.t + r.dur) { hit = &r; break; }
        }
    }
    if (hit) {
        const char* name = descOf(hit->desc);
        /* What it is, how big, and who it was with — by the best name there is
         * for them: what they announced, the number `lora n` gives them, or the
         * six hex characters only where the tag resolves to nobody at all. The
         * tag is what a log line quotes, so it is the fallback rather than a
         * prefix on every line: once there is a name, the hex is the part
         * nobody reads, and this strip is a few characters wide. Resolved only
         * when the tag changes — this runs every redraw, and the peer table is
         * a linear scan. */
        if (strcmp(s.insTag, hit->tag) != 0) {
            safeStrncpy(s.insTag, hit->tag, sizeof s.insTag);
            loraNameForTag(s.radio, hit->tag, s.insName, sizeof s.insName);
        }
        char b[96];
        int o = 0;
        if (name[0]) o += snprintf(b + o, sizeof b - (size_t)o, "%s ", name);
        o += snprintf(b + o, sizeof b - (size_t)o, "%uB", (unsigned)hit->bytes);
        const char* who = s.insName[0] ? s.insName : hit->tag;
        if (who[0]) snprintf(b + o, sizeof b - (size_t)o, " %s", who);
        int laneY = hit->ch == 0 ? s.plotY
                                 : s.plotY + mainH + (hit->ch - 1) * laneH;
        int y = laneY - 10;
        if (y < s.plotY) y = laneY + 2;
        uint32_t vlo, vhi; view(now, &vlo, &vhi);
        int x = xAtTime(vlo, vhi, hit->t);
        if (x > s.W - GUT_R - 150) x = s.W - GUT_R - 150;
        if (x < GUT_L) x = GUT_L;
        lv_label_set_text(s.insLbl, b);
        lv_obj_align(s.insLbl, LV_ALIGN_TOP_LEFT, x, y);
        lv_obj_clear_flag(s.insLbl, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s.insLbl, LV_OBJ_FLAG_HIDDEN);
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
#if !defined(CONFIG_LORA_NO_SUPE)
        snprintf(b, sizeof b,
                 "tx %d.%d%%  busy %d.%d%%  #E8D040 rnsd# #E89040 rnode# #E84040 SUPE#",
                 tx / 10, tx % 10, busy / 10, busy % 10);
#else
        /* No third protocol to colour: nothing here transmits one. */
        snprintf(b, sizeof b,
                 "tx %d.%d%%  busy %d.%d%%  #E8D040 rnsd# #E89040 rnode#",
                 tx / 10, tx % 10, busy / 10, busy % 10);
#endif
        lv_label_set_text(s.cap, b);
    }
    lv_obj_invalidate(s.canvas);
}

void tickCb(lv_timer_t*) { if (s.visible) drawAll(); }

/* A small label floating over the canvas, hidden until it has something to say. */
lv_obj_t* mkFloat(lv_obj_t* root, uint32_t colour) {
    lv_obj_t* l = lv_label_create(root);
    lv_label_set_text(l, "");
    lv_obj_set_style_text_font(l, lcdFont(LcdFace::MONO, 8), 0);
    lv_obj_set_style_text_color(l, lv_color_hex(colour), 0);
    lv_obj_set_style_bg_color(l, lv_color_hex(0x101010), 0);
    lv_obj_set_style_bg_opa(l, LV_OPA_80, 0);
    lv_obj_set_style_pad_all(l, 2, 0);
    lv_obj_add_flag(l, LV_OBJ_FLAG_HIDDEN);
    return l;
}

/* A pill: a note stuck on the graph rather than part of it, so it takes a paper
 * colour and black text instead of joining the plot's palette, and rounds its
 * corners to say the same thing. One style for both the peer names beside trains
 * and the noise floor's figure — two label styles for one job is one too many. */
lv_obj_t* mkNotePill(lv_obj_t* root) {
    lv_obj_t* l = mkFloat(root, 0x000000);
    lv_obj_set_style_bg_color(l, lv_color_hex(0xFFFFCC), 0);
    lv_obj_set_style_bg_opa(l, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(l, 4, 0);
    lv_obj_set_style_pad_hor(l, 3, 0);
    return l;
}

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
            s.inspect = false;
            showPills();
        } else {
            /* A tap that never widened is not a zoom — it is a question about
             * the instant under the finger. The answer is the same wherever in
             * the stack the finger landed: the lanes share a time axis, so a tap
             * names a COLUMN, and whichever lane holds a frame there answers. */
            s.inspect = !(s.inspect && s.inspectT == a);   /* tap again to dismiss */
            s.inspectT = a;
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
    s.nfLbl  = mkNotePill(root);
    s.insLbl = mkFloat(root, 0xE8E8E8);
    for (int i = 0; i < State::TRAIN_LBL_MAX; i++) s.trainLbl[i] = mkNotePill(root);
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
