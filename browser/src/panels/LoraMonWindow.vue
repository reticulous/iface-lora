<template>
  <FloatingWindow
    id="loramon"
    :title="title"
    :visible="visible"
    :focus-token="focusToken"
    :default-geom="defaultGeom"
    :min-size="{ w: 24, h: 16 }"
    flush
    @update:visible="v => emit('update:visible', v)"
  >
    <template #default>
      <div class="lm-body">
        <div v-if="radios.length > 1" class="lm-tabs">
          <button v-for="r in radios" :key="r" class="lm-tab" :class="{ active: r === activeRadio }"
                  @click="activeRadio = r">lora/{{ r }}</button>
        </div>
        <!-- The axis names sit in the strip above their own gutters, which is
             what pushes the pills inward. Zoomed in, the window is a fixed span
             on the stack, so the moving-window pills are meaningless — only the
             way back out remains. -->
        <div class="lm-pills">
          <span class="lm-axis lm-axis-tx">tx</span>
          <template v-if="zoomed">
            <button class="lm-pill lm-back" :class="{ 'lm-last': zoomStack.length === 1 }"
                    @click="zoomOut">←</button>
            <span class="lm-zoomlabel">{{ zoomLabel }}</span>
          </template>
          <button v-else v-for="w in WINDOWS" :key="w.key" class="lm-pill"
                  :class="{ active: w.key === winKey }"
                  @click="winKey = w.key">{{ w.label }}</button>
          <span class="lm-legend">
            <span class="c-rns">rnsd</span> / <span class="c-rnode">rnode</span> / <span class="c-ours">SUPE</span> / <span class="c-bad">CRC</span>
          </span>
          <span class="lm-axis lm-axis-rx">rx</span>
        </div>
        <div class="lm-graphs">
          <div class="lm-graph lm-graph-main">
            <canvas ref="canvasRef" class="lm-canvas"
                    @pointerdown="onDown"
                    @pointermove="onMove"
                    @pointerup="onUp"
                    @pointercancel="onUp" />
            <div class="lm-caption">
              <span class="lm-chan">{{ chanLabel(0) }}</span>
              <span class="lm-air">tx airtime {{ air.tx }}</span>
              <span class="lm-air">channel busy {{ air.busy }}</span>
            </div>
          </div>
          <!-- One graph per agile channel of the regime in force, stacked under
               the hailing channel's at a quarter its height. Same width, so the
               same time axis: a moment is the same column in every one of them.
               Same bands, same dBm scale, same window — only the gutter labels
               are left off, since repeating one scale ten times is noise. -->
          <div v-for="c in agileChans" :key="c" class="lm-graph lm-graph-chan">
            <canvas :ref="el => setChanCanvas(c, el)" class="lm-canvas" />
            <div class="lm-caption lm-caption-chan">
              <span class="lm-chan">{{ chanLabel(c) }}</span>
              <span class="lm-air">tx {{ chanTx(c) }}</span>
            </div>
          </div>
        </div>
      </div>
    </template>
  </FloatingWindow>
</template>

<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted, nextTick } from 'vue'
import FloatingWindow from 'spangap-browser/components/FloatingWindow.vue'
import { useDeviceStore } from 'spangap-browser/stores/device'
import { getSession } from 'spangap-browser/lib/webrtc-session'
import { focusedWindowId } from 'spangap-browser/lib/windows'

const WIN_ID = 'loramon'          // must match the id given to FloatingWindow

const props = defineProps<{ visible: boolean; title: string; focusToken?: number }>()
const emit = defineEmits<{ 'update:visible': [value: boolean] }>()

const device = useDeviceStore()

const isPhoneInit = window.matchMedia?.('(max-width: 599px)').matches ?? false
const defaultGeom = isPhoneInit
  ? { x: 0, y: 0, w: 100, h: 82 }
  : { x: 20, y: 6, w: 55, h: 84 }

const HOUR_MS = 3600 * 1000
const MAX_RADIOS = 4
const GUT_L_CSS = 30          // left scale gutter (tx dBm), CSS px
const GUT_R_CSS = 34          // right scale gutter (rx dBm) — four-digit labels

/* Colour is the frame's protocol, not its direction — direction is the axis a
 * bar is read against, and the tinted background under a transmit. Types match
 * the firmware's LORA_PKT_*. */
const C_RNS = '#E8D040'       // Reticulum traffic (yellow)
const C_OURS = '#40A0FF'      // our own air protocol, SUPE (blue)
const C_RNODE = '#E89040'     // the attached RNode client's traffic (orange)
const C_BAD = '#E04048'       // rx frame that failed CRC: air held, nothing decoded
/* SUPE is blue rather than red because the transmit background IS red: a red
 * bar on a red field is the one pair a viewer cannot separate at a glance, and
 * the background is the more important of the two — direction is read off it
 * for every frame, where the protocol tag matters for a few. */

/* The politeness marks — what the frame waited before it went out. Near-white
 * and thin: they are annotation on a bar, not a quantity to compare against
 * one, and over short bursts anything stronger dominates the airtime it is
 * describing. */
const C_WAIT = '#E8E8E8'

/* Band gradient, bottom → top of each band, and the reddish cast of the same
 * gradient that marks the air being ours. The darkest tone doubles as the
 * timescale grid, so the grid reads as part of the background. */
const BG_LO = '#242424', BG_HI = '#313131'
/* The transmit cast is a real red rather than a hint of one: it is how
 * direction is read, and at these bar widths a subtle tint is no signal at all. */
const TX_LO = '#5e1c1c', TX_HI = '#8a2a2a'
const C_GRID = BG_LO

const WINDOWS = [
  { key: '10s', ms: 10 * 1000,  label: '10s' },
  { key: '1m',  ms: 60 * 1000,  label: '1m'  },
  { key: '5m',  ms: 300 * 1000, label: '5m'  },
  { key: '10m', ms: 600 * 1000, label: '10m' },
  { key: '30m', ms: 1800 * 1000, label: '30m' },
  { key: '1h',  ms: HOUR_MS,    label: '1hr' },
] as const

/* One plot, two dBm axes reading the same four bands: transmit power down the
 * left gutter in 10 dB steps, received strength down the right in 25 dB. RX
 * needs the wider step because its range is 100 dB against TX's 40, and both
 * have to land on the same band edges for a single grid to serve them. */
const NBANDS = 4
const AX_TX = { lo: -10, hi: 30 }     // 10 dB per band
const AX_RX = { lo: -130, hi: -30 }   // 25 dB per band

/* The channel-noise floor the traffic sits on: very light grey, so a bar always
 * wins the pixels it lands on and the floor reads as background texture. */
const C_FLOOR = 'rgba(255,255,255,0.09)'

interface Rec { t: number; dir: number; dur: number; bytes: number; rssi: number; snr10: number; txp: number; type: number; wait: number; own: number; ch: number }

/* recs = the active radio's packets, rebuilt each tick from the mirrored
 * `lora.<n>.packets` subtree (the firmware adds/deletes those nodes). */
let recs: Rec[] = []

/* Channel RSSI, accumulated live rather than mirrored as history: the firmware
 * publishes only the newest sweep (`lora.<n>.rssi` = "<ms>|<ch0>|<ch1>|…"), so
 * the series starts when the window opens — the same rule the packet nodes
 * follow. A beat the radio skipped (carrier sense had it) republishes nothing,
 * so the key is unchanged, no point is appended, and the gap draws as a gap. */
const CH_MAX = 10
interface Floor { t: number; dbm: number }
let floorSeries: Floor[][] = Array.from({ length: CH_MAX }, () => [])
let floorLastMs = 0

/* The channels the regime in force puts up, from `lora.<n>.chans`. Index 0 is
 * always the hailing channel at the radio's configured frequency; a list of one
 * means no agility, which is how the viewer knows not to draw the extra graphs
 * without needing a separate flag. */
interface Chan { freq: number; bw: number }
const chanList = ref<Chan[]>([])
const agileChans = computed<number[]>(() =>
  chanList.value.slice(1).map((_, i) => i + 1))

const fmtMHz = (hz: number) => (hz / 1e6).toFixed(hz % 100000 === 0 ? 2 : 3)
const fmtBw = (hz: number) => hz >= 1e6 ? `${hz / 1e6}M` : `${Math.round(hz / 1e3)}k`

function chanLabel(c: number): string {
  const k = chanList.value[c]
  return k ? `${fmtMHz(k.freq)} ${fmtBw(k.bw)}` : '—'
}
let devClock = 0              // newest packet ms seen = device clock reference
let devClockAt = 0           // Date.now() when devClock was captured

const activeRadio = ref(0)
const winKey = ref<string>('1m')
const air = ref<{ tx: string; busy: string }>({ tx: '0%', busy: '0%' })

/* Zoom stack: each entry is an absolute [t0,t1] device-time span. Empty = the
 * live moving window chosen by the pills. Selecting inside a zoomed view pushes
 * a further span, so the stack is the zoom history and back pops one level. */
const zoomStack = ref<{ t0: number; t1: number }[]>([])
const zoomed = computed(() => zoomStack.value.length > 0)

/* Selection in progress, held as DEVICE TIMES, not pixels: that is what makes a
 * still finger widen the highlight on a moving graph — the anchor time stays
 * put while "now" advances, so the anchor drifts left under the pointer. */
const sel = ref<{ anchor: number; cur: number } | null>(null)

const winMs = computed(() => WINDOWS.find(w => w.key === winKey.value)?.ms ?? 60000)

function fmtSpan(ms: number): string {
  if (ms < 1000) return `${Math.round(ms)} ms`
  if (ms < 60000) return `${(ms / 1000).toFixed(ms < 10000 ? 1 : 0)} s`
  if (ms < HOUR_MS) return `${(ms / 60000).toFixed(ms < 600000 ? 1 : 0)} min`
  return `${(ms / HOUR_MS).toFixed(1)} h`
}

/* Timescale of the frozen view: division lines and the ms/div they are worth.
 * Divisions come off the 1-2-5-10 ladder, whose widest gap is ×2.5 (2 → 5), so
 * a band of allowed pixels-per-division at least 2.5× wide always contains a
 * step, whatever the span and however wide the canvas is. 70 CSS px up gives 5
 * divisions on a phone-width window and 12 docked; the minimum is what is
 * chosen from, the 175 px it implies is only the other edge. */
const DIV_PX_MIN = 70

/* Finest grid the band allows: the smallest 1-2-5 step at least DIV_PX_MIN
 * wide. Floored at 1 ms — records are ms-stamped, so a finer division would
 * draw precision the data doesn't have. */
function divStepMs(msPerPx: number): number {
  const want = msPerPx * DIV_PX_MIN
  if (!(want > 1)) return 1
  const dec = Math.pow(10, Math.floor(Math.log10(want)))
  const m = want / dec
  return dec * (m <= 1 ? 1 : m <= 2 ? 2 : m <= 5 ? 5 : 10)
}

/* Set by the draw, read by the caption: the step needs the canvas width. */
const divMs = ref(0)

/* Depth is the back button's colour (blue = one level left), so the label is
 * free to carry what the frozen view can't show any other way: its timescale. */
const zoomLabel = computed(() => {
  const v = zoomStack.value[zoomStack.value.length - 1]
  if (!v) return ''
  const span = `${fmtSpan(v.t1 - v.t0)} window`
  return divMs.value > 0 ? `${span} · ${fmtSpan(divMs.value)} / div` : span
})

/* Radios that exist = those the firmware has published a state key for. */
const radios = computed<number[]>(() => {
  const out: number[] = []
  for (let n = 0; n < MAX_RADIOS; n++)
    if (device.get(`lora.${n}.state`) != null) out.push(n)
  return out.length ? out : [0]
})

/* Estimated device clock (ms) now, extrapolated from the newest packet seen. */
function devNow(): number {
  return devClock ? devClock + (Date.now() - devClockAt) : 0
}

/* The time span currently on screen: the top of the zoom stack if there is
 * one, else the live window ending at "now". */
function view(): { lo: number; hi: number } {
  const top = zoomStack.value[zoomStack.value.length - 1]
  if (top) return { lo: top.t0, hi: top.t1 }
  const dnow = devNow()
  return { lo: dnow - winMs.value, hi: dnow }
}

/* ── canvas ── */
const canvasRef = ref<HTMLCanvasElement | null>(null)

/* Size the canvas backing store to its displayed pixels × DPR (full resolution,
 * tracks resize). */
function fit(cv: HTMLCanvasElement | null): { ctx: CanvasRenderingContext2D; w: number; h: number; dpr: number } | null {
  if (!cv) return null
  const dpr = window.devicePixelRatio || 1
  const w = Math.max(1, Math.round(cv.clientWidth * dpr))
  const h = Math.max(1, Math.round(cv.clientHeight * dpr))
  if (cv.width !== w) cv.width = w
  if (cv.height !== h) cv.height = h
  const ctx = cv.getContext('2d')
  if (!ctx) return null
  return { ctx, w, h, dpr }
}

/* Four gradient bands across the plot, with both axes labelled in their own
 * gutter so a bar's height reads directly as dBm — transmit power on the left,
 * received strength on the right. */
function drawBands(ctx: CanvasRenderingContext2D, w: number, h: number, dpr: number,
                   gl: number, gr: number, labels: boolean) {
  const bh = h / NBANDS
  for (let i = 0; i < NBANDS; i++) {
    const yTop = h - (i + 1) * bh
    const g = ctx.createLinearGradient(0, yTop + bh, 0, yTop)
    g.addColorStop(0, BG_LO); g.addColorStop(1, BG_HI)
    ctx.fillStyle = g
    ctx.fillRect(gl, yTop, w - gl - gr, bh)
  }
  if (!labels) return
  ctx.font = `${Math.round(9 * dpr)}px 'SF Mono','Menlo','Consolas',monospace`
  ctx.fillStyle = '#8a8a8a'
  ctx.textBaseline = 'middle'
  const pad = Math.round(4 * dpr)
  for (let i = 0; i <= NBANDS; i++) {
    let y = h - i * bh
    if (i === 0) y -= Math.round(5 * dpr)          // keep the end labels on-canvas
    if (i === NBANDS) y += Math.round(5 * dpr)
    ctx.textAlign = 'right'
    ctx.fillText(String(AX_TX.lo + i * (AX_TX.hi - AX_TX.lo) / NBANDS), gl - pad, y)
    ctx.textAlign = 'left'
    ctx.fillText(String(AX_RX.lo + i * (AX_RX.hi - AX_RX.lo) / NBANDS), w - gr + pad, y)
  }
}

const clamp01 = (x: number) => Math.max(0, Math.min(1, x))

/* Draw one channel's plot. `ch` selects the records and the RSSI series.
 *
 * The gutters are reserved on every graph, labelled only on the hailing
 * channel's. Reserved because that is what makes the stack readable: the plots
 * begin and end at the same x, so a moment is the same column in all ten and
 * the eye can run down it. Unlabelled because the scale is identical on each
 * and a quarter-height band has no room for the numbers anyway. */
function drawOne(cv: HTMLCanvasElement | null, ch: number, main: boolean) {
  const f = fit(cv)
  if (!f) return
  const { ctx, w, h, dpr } = f
  const gl = Math.round(GUT_L_CSS * dpr)
  const gr = Math.round(GUT_R_CSS * dpr)
  ctx.clearRect(0, 0, w, h)
  drawBands(ctx, w, h, dpr, gl, gr, main)
  if (!devClock) return
  const recsCh = recs.filter(r => r.ch === ch)
  const floorPts = floorSeries[ch] ?? []
  const { lo, hi } = view()
  const ms = hi - lo
  if (ms <= 0) return
  const span = w - gl - gr
  if (span <= 0) return
  const xAt = (t: number) => gl + clamp01((t - lo) / ms) * span
  const bh = h / NBANDS

  /* Air we are holding ourselves: the same gradient cast red, over the frame's
   * time-on-air only. The wait before it is channel access, not transmission —
   * tinting that would claim airtime the radio never spent. */
  const txSpans: { x: number; w: number }[] = []
  for (const rec of recsCh) {
    if (rec.dir !== 1) continue
    const s = rec.t, e = rec.t + rec.dur
    if (e < lo || s > hi) continue
    const xs = xAt(s)
    txSpans.push({ x: xs, w: Math.max(1, xAt(e) - xs) })
  }
  if (txSpans.length) {
    for (let i = 0; i < NBANDS; i++) {
      const yTop = h - (i + 1) * bh
      const g = ctx.createLinearGradient(0, yTop + bh, 0, yTop)
      g.addColorStop(0, TX_LO); g.addColorStop(1, TX_HI)
      ctx.fillStyle = g
      for (const s of txSpans) ctx.fillRect(s.x, yTop, s.w, bh)
    }
  }

  /* The channel noise floor: each sample a bar from the bottom of the plot up
   * to its dBm on the RX axis, held until the next sample so a 1 Hz series
   * reads as a continuous floor rather than a picket fence. Drawn after the
   * transmit tint and before the frames — traffic lies on top of the noise it
   * had to get above.
   *
   * A gap in the series is a beat carrier sense took the radio for. It is left
   * empty on purpose: the bar would otherwise be drawn from a reading that
   * described the transmission we were queued behind. */
  if (floorPts.length) {
    ctx.fillStyle = C_FLOOR
    for (let i = 0; i < floorPts.length; i++) {
      const p = floorPts[i]
      const next = floorPts[i + 1]
      /* Hold for one beat at most: a longer silence is a gap, not a level. */
      const end = Math.min(next ? next.t : p.t + 1000, p.t + 1000)
      if (end < lo || p.t > hi) continue
      const xs = xAt(p.t)
      const bw = Math.max(1, xAt(end) - xs)
      const y = h - clamp01((p.dbm - AX_RX.lo) / (AX_RX.hi - AX_RX.lo)) * h
      ctx.fillRect(xs, y, bw, h - y)
    }
  }

  /* Frozen view only: a live one slides, and a grid on absolute time would
   * crawl across it. Lines are laid on round multiples of the step, under the
   * frames so a bar always wins the pixels it lands on. */
  if (zoomed.value) {
    const step = divStepMs(ms / Math.max(1, span / dpr))
    /* The label quotes the main graph's grid — an agile channel is a quarter
     * the width, so its own step is coarser and would overwrite it. */
    if (main) divMs.value = step
    const lw = Math.max(1, dpr)
    ctx.fillStyle = C_GRID
    for (let t = Math.ceil(lo / step) * step; t <= hi; t += step)
      ctx.fillRect(xAt(t), 0, lw, h)
  }

  for (const rec of recsCh) {
    const s = rec.t, e = rec.t + rec.dur
    if (e < lo || s > hi) continue
    const xs = xAt(s)
    const bw = Math.max(1, xAt(e) - xs)
    const th = Math.max(1, h * 0.05)
    const ax = rec.dir === 1 ? AX_TX : AX_RX
    const dbm = rec.dir === 1 ? rec.txp : rec.rssi
    let y = h - clamp01((dbm - ax.lo) / (ax.hi - ax.lo)) * h
    if (y > h - th) y = h - th
    if (y < 0) y = 0
    const col = rec.type === 3 ? C_BAD
              : rec.type === 2 ? C_RNODE : rec.type === 1 ? C_OURS : C_RNS
    /* What the frame waited before its first bit went on air, drawn as two
     * runs because they are two different facts. Both sit at mid-height in the
     * frame's own colour, light enough that channel occupancy still reads as
     * the filled area alone — a long wait must not be mistaken for airtime.
     *
     *   solid  — CONTENTION: the channel was busy. Somebody else's traffic.
     *   dotted — OURS: the radio was held by our own work, a split was still
     *            landing, or we deliberately delayed (a pre-offer jitter).
     *
     * Ours runs first and contention second, so the pair reads left to right in
     * the order the frame actually experienced them, ending at the bar. */
    const lw = Math.max(1, dpr)
    const drawWait = (fromMs: number, toMs: number, dotted: boolean) => {
      const x0 = xAt(fromMs), x1 = xAt(toMs)
      if (x1 - x0 < 1) return
      ctx.fillStyle = C_WAIT
      const yl = y + (th - lw) / 2
      if (!dotted) { ctx.fillRect(x0, yl, x1 - x0, lw); return }
      /* Dotted by hand rather than via setLineDash: these are fillRects on a
       * device-pixel grid, and a dash pattern on a 1px line renders unevenly
       * once dpr is not 1. */
      const step = Math.max(2, Math.round(3 * dpr))
      for (let x = x0; x < x1; x += step * 2) ctx.fillRect(x, yl, Math.min(step, x1 - x), lw)
    }
    /* No tick at the start. These bursts are a few milliseconds wide, so a
     * full-height mark beside them reads as the loudest thing on the graph
     * while carrying the least — the run's left end already says when the
     * frame first wanted the air. */
    drawWait(s - rec.wait - rec.own, s - rec.wait, true)   /* ours, dotted */
    drawWait(s - rec.wait, s, false)                       /* the channel's, solid */
    ctx.fillStyle = col
    ctx.fillRect(xs, y, bw, th)
  }

  if (main && sel.value) {
    const a = Math.min(sel.value.anchor, sel.value.cur)
    const b = Math.max(sel.value.anchor, sel.value.cur)
    const xa = xAt(a), xb = xAt(b)
    ctx.fillStyle = 'rgba(255,255,255,0.16)'
    ctx.fillRect(xa, 0, Math.max(1, xb - xa), h)
    ctx.strokeStyle = 'rgba(255,255,255,0.55)'
    ctx.lineWidth = Math.max(1, dpr)
    ctx.beginPath()
    ctx.moveTo(xa, 0); ctx.lineTo(xa, h)
    ctx.moveTo(xb, 0); ctx.lineTo(xb, h)
    ctx.stroke()
  }
}

/* Canvases of the agile channels, collected by the v-for's ref callback. */
const chanCanvas = new Map<number, HTMLCanvasElement>()
function setChanCanvas(c: number, el: unknown) {
  if (el instanceof HTMLCanvasElement) chanCanvas.set(c, el)
  else chanCanvas.delete(c)
}

function redraw() {
  drawOne(canvasRef.value, 0, true)
  for (const c of agileChans.value) drawOne(chanCanvas.get(c) ?? null, c, false)
}

/* ── selection → zoom ── */
function xToTime(ev: PointerEvent): number | null {
  const cv = canvasRef.value
  if (!cv) return null
  const rect = cv.getBoundingClientRect()
  const span = rect.width - GUT_L_CSS - GUT_R_CSS
  if (span <= 0) return null
  const frac = clamp01((ev.clientX - rect.left - GUT_L_CSS) / span)
  const { lo, hi } = view()
  return lo + frac * (hi - lo)
}

function onDown(ev: PointerEvent) {
  /* A press that raises this window from behind another only raises it — the
   * same swallow FloatingWindow gives a click, which pointer events bypass.
   * Without it, reaching for an occluded LoRaMon costs you a zoom. The press
   * still travels: FloatingWindow brings the window to the front on mousedown,
   * which arrives after this. */
  if (focusedWindowId.value !== WIN_ID) return
  const t = xToTime(ev)
  if (t == null) return
  ;(ev.currentTarget as HTMLElement).setPointerCapture?.(ev.pointerId)
  sel.value = { anchor: t, cur: t }
  ev.preventDefault()
}

function onMove(ev: PointerEvent) {
  if (!sel.value) return
  const t = xToTime(ev)
  if (t == null) return
  sel.value = { anchor: sel.value.anchor, cur: t }
}

function onUp() {
  const s = sel.value
  sel.value = null
  if (!s) return
  const t0 = Math.min(s.anchor, s.cur)
  const t1 = Math.max(s.anchor, s.cur)
  const { lo, hi } = view()
  /* A tap that never widened (a static, already-zoomed view) is not a zoom. */
  if (t1 - t0 < (hi - lo) * 0.01 || t1 - t0 < 5) return
  zoomStack.value = [...zoomStack.value, { t0, t1 }]
  tick()
  nextTick(redraw)
}

function zoomOut() {
  zoomStack.value = zoomStack.value.slice(0, -1)
  tick()
  nextTick(redraw)
}

/* ── rebuild recs from the mirrored subtree ── */
function parseRec(t: number, s: string): Rec | null {
  const p = s.split('|')
  if (p[0] === 'r') return { t, dir: 0, rssi: +p[1], snr10: +p[2], dur: +p[3], bytes: +p[4], txp: 0, type: +(p[5] ?? 0), wait: 0, own: 0, ch: +(p[6] ?? 0) }
  if (p[0] === 't') return { t, dir: 1, txp: +p[1], dur: +p[2], bytes: +p[3], rssi: 0, snr10: 0, type: +(p[4] ?? 0), wait: +(p[5] ?? 0), ch: +(p[6] ?? 0), own: +(p[7] ?? 0) }
  return null
}

/* Append the newest channel-RSSI sweep if it is one we haven't seen. Keyed on
 * the device timestamp leading the value, so a repeat publish of an unchanged
 * key adds nothing and a skipped beat leaves a hole. */
function pollFloor() {
  const raw = device.get(`lora.${activeRadio.value}.rssi`)
  if (raw == null) return
  const p = String(raw).split('|')
  const t = +p[0]
  if (!Number.isFinite(t) || t === floorLastMs) return
  floorLastMs = t
  const cut = t - HOUR_MS
  for (let c = 0; c < CH_MAX && c + 1 < p.length; c++) {
    /* An empty field is a channel that did not answer this beat — the radio
     * was elsewhere, or the receiver had not settled. No point, so a gap. */
    if (p[c + 1] === '') continue
    const dbm = +p[c + 1]
    if (!Number.isFinite(dbm)) continue
    const s = floorSeries[c]
    s.push({ t, dbm })
    if (s.length > 8 && s[0].t < cut) floorSeries[c] = s.filter(f => f.t >= cut)
  }
  /* Same monotonic anchor as rebuild(): only a timestamp AHEAD of the local
   * extrapolation may re-anchor. Re-anchoring on merely newer-than-devClock
   * pulled "now" back by the beat's transport delay — the timeline yank. */
  if (t > devNow()) { devClock = t; devClockAt = Date.now() }
}

/* The regime's channel list. Cheap to reparse; it only changes on a config
 * apply, so compare the raw string before touching the reactive ref. */
let chansRaw = ''
function pollChans() {
  const raw = String(device.get(`lora.${activeRadio.value}.chans`) ?? '')
  if (raw === chansRaw) return
  chansRaw = raw
  chanList.value = raw ? raw.split('|').map(s => {
    const [f, b] = s.split(',')
    return { freq: +f, bw: +b }
  }).filter(k => Number.isFinite(k.freq) && Number.isFinite(k.bw)) : []
}

function rebuild() {
  const tree = device.get(`lora.${activeRadio.value}.packets`) ?? {}
  const arr: Rec[] = []
  let newest = devClock
  for (const k in tree) {
    const t = Number(k)
    if (!Number.isFinite(t)) continue
    const rec = parseRec(t, String(tree[k]))
    if (rec) { arr.push(rec); if (t > newest) newest = t }
  }
  arr.sort((a, b) => a.t - b.t)
  recs = arr
  /* Anchor device-now monotonically: never pull it backward (that snap caused
   * the new-bar jerk) — advance to the newest packet or the local extrapolation,
   * whichever is later. */
  const cur = devNow()
  const anchor = Math.max(newest, cur)
  if (anchor > 0) { devClock = anchor; devClockAt = Date.now() }
}

const fmtPct = (p: number) => `${p.toFixed(p >= 10 ? 0 : 1)}%`

/* Busy milliseconds in one direction over a span, overlap-counted from the
 * frame records. */
function busyMs(dir: number, lo: number, hi: number, ch: number): number {
  let busy = 0
  for (const r of recs) {
    if (r.dir !== dir || r.ch !== ch) continue
    let s = r.t, e = r.t + r.dur
    if (e <= lo || s >= hi) continue
    if (s < lo) s = lo
    if (e > hi) e = hi
    if (e > s) busy += e - s
  }
  return busy
}

/* Airtime over whatever span is on screen: what we transmitted, and how much of
 * the channel was in use at all (ours + theirs — the radio is half duplex, so
 * the two never overlap). The live hour is the exception: it needs more history
 * than a viewer has usually been open for, so the firmware publishes those two
 * figures (per mille). A zoomed span is always computed locally. */
/* Absolute airtime next to the percent: the percent says how full the window
 * was, the seconds say what it cost. Two decimals under 10 s, then fmtSpan's
 * coarser steps. */
const fmtSecs = (ms: number) => ms < 10000 ? `${(ms / 1000).toFixed(2)} s` : fmtSpan(ms)

function airFor(): { tx: string; busy: string } {
  if (!zoomed.value && winKey.value === '1h') {
    const tx = device.get(`lora.${activeRadio.value}.air1h.tx`)
    const rx = device.get(`lora.${activeRadio.value}.air1h.rx`)
    if (tx == null || rx == null) return { tx: '—', busy: '—' }
    /* Firmware publishes the hour per mille → 1‰ of an hour is 3600 ms. */
    return { tx: `${fmtPct(Number(tx) / 10)} · ${fmtSecs(Number(tx) * 3600)}`,
             busy: `${fmtPct((Number(tx) + Number(rx)) / 10)} · ${fmtSecs((Number(tx) + Number(rx)) * 3600)}` }
  }
  const { lo, hi } = view()
  const ms = hi - lo
  if (ms <= 0) return { tx: '0%', busy: '0%' }
  const tx = busyMs(1, lo, hi, 0)
  const rx = busyMs(0, lo, hi, 0)
  return { tx: `${fmtPct(tx / ms * 100)} · ${fmtSecs(tx)}`,
           busy: `${fmtPct((tx + rx) / ms * 100)} · ${fmtSecs(tx + rx)}` }
}

/* Each agile channel's transmit airtime over the window on screen. Always
 * computed locally — the firmware's published hour is the radio's total, which
 * is the hailing channel's while nothing transmits anywhere else. "Channel
 * busy" is deliberately absent here: what another node is doing on a channel we
 * only visit to measure says little, and the figure would invite reading it as
 * occupancy when it is one instant sampled per second.
 *
 * Held in a ref rather than computed in the template: `recs` is a plain array
 * rebuilt each tick, so a template call would not re-run when it changed. */
const chanAir = ref<string[]>([])
const chanTx = (c: number) => chanAir.value[c] ?? '0%'

function chanAirFor(): string[] {
  const { lo, hi } = view()
  const ms = hi - lo
  const out: string[] = []
  for (const c of agileChans.value) {
    const b = ms > 0 ? busyMs(1, lo, hi, c) : 0
    out[c] = ms > 0 ? `${fmtPct(b / ms * 100)} · ${fmtSecs(b)}` : '0%'
  }
  return out
}

function tick() {
  if (props.visible) device.set('sys.stats.web_loramon', 1)   // heartbeat while open
  pollChans()
  pollFloor()
  rebuild()
  air.value = airFor()
  chanAir.value = chanAirFor()
}

let timer: ReturnType<typeof setInterval> | null = null
let raf = 0
let lastDraw = 0

/* Smooth slide: the "now" edge advances continuously (devNow extrapolates), so
 * redrawing on animation frames glides the bars. Throttled to ~30 fps. */
function frame(ts: number) {
  raf = requestAnimationFrame(frame)
  if (!props.visible || ts - lastDraw < 33) return
  lastDraw = ts
  redraw()
}

onMounted(() => {
  if (radios.value.length && !radios.value.includes(activeRadio.value))
    activeRadio.value = radios.value[0]
  device.set('sys.stats.web_loramon', 1)   // start recording before the first tick
  getSession().connect()                    // ensure the storage mirror is flowing
  tick()
  timer = setInterval(tick, 1000)
  raf = requestAnimationFrame(frame)
})
onUnmounted(() => {
  if (timer) { clearInterval(timer); timer = null }
  if (raf) { cancelAnimationFrame(raf); raf = 0 }
  device.set('sys.stats.web_loramon', 0)
})

watch(() => props.visible, v => {
  device.set('sys.stats.web_loramon', v ? 1 : 0)
  if (v) { tick(); nextTick(redraw) }
})

watch(winKey, () => { tick(); if (props.visible) nextTick(redraw) })

watch(activeRadio, () => {
  devClock = 0; devClockAt = 0; recs = []
  floorSeries = Array.from({ length: CH_MAX }, () => [])
  floorLastMs = 0; chansRaw = ''; chanList.value = []
  chanCanvas.clear()
  zoomStack.value = []; sel.value = null
  tick()
  if (props.visible) nextTick(redraw)
})
</script>

<style scoped>
.lm-body { position: relative; width: 100%; height: 100%; background: #000; display: flex; flex-direction: column; }

.lm-tabs { display: flex; gap: 2px; padding: 3px 4px 0; flex: 0 0 auto; }
.lm-tab {
  padding: 2px 10px;
  border: none; border-radius: 4px 4px 0 0;
  background: #1c1c1c; color: #9a9a9a;
  font: 11px/1.4 'SF Mono', 'Menlo', 'Consolas', monospace;
  cursor: pointer;
}
.lm-tab.active { background: #2c2c2c; color: #fff; }

/* No wrapping: the axis names have to stay on the gutters they label, so a
 * cramped window clips the pill row rather than folding it. */
.lm-pills { display: flex; gap: 4px; padding: 6px 8px 2px; flex: 0 0 auto; flex-wrap: nowrap; align-items: center; overflow: hidden; }
.lm-pill {
  padding: 2px 10px;
  border: 1px solid #3a3a3a; border-radius: 999px;
  background: #181818; color: #9a9a9a;
  font: 11px/1.4 'SF Mono', 'Menlo', 'Consolas', monospace;
  cursor: pointer;
}
.lm-pill.active { background: #3a3a3a; border-color: #6a6a6a; color: #fff; }
/* Blue = one level of zoom left, so pressing it returns to the live window.
 * Grey means the stack is deeper and there is another frozen view behind. */
/* The arrow is the one pill you aim for without reading it, so it gets a glyph
 * a size up and room on both sides rather than the row's usual 4 px gap. */
.lm-back { color: #e8e8e8; padding: 0 18px; margin: 0 6px; font-size: 15px; line-height: 1.3; }
.lm-back.lm-last { background: #2b5cc4; border-color: #5a86e0; color: #fff; }
.lm-zoomlabel { font: 11px/1.4 'SF Mono', 'Menlo', 'Consolas', monospace; color: #7a7a7a; white-space: nowrap; }

/* Each axis name stands over its own gutter, so the widths here are the canvas
 * gutter widths and `tx` reads as the heading of the scale below it. */
.lm-axis {
  flex: 0 0 auto;
  font: 11px/1.4 'SF Mono', 'Menlo', 'Consolas', monospace;
  color: #c8c8c8;
}
.lm-axis-tx { width: 30px; text-align: right; }
.lm-axis-rx { width: 34px; text-align: left; margin-left: auto; }

/* Graphs stack, so every one of them spans the same width and therefore the
 * same time axis — a moment is the same column in all ten. The flex ratios are
 * the heights: the hailing channel takes 4, each agile channel 1. */
.lm-graphs { flex: 1 1 auto; display: flex; flex-direction: column; gap: 8px; padding: 2px 8px 4px; min-height: 0; }
.lm-graph { display: flex; flex-direction: column; min-height: 0; min-width: 0; }
.lm-graph-main { flex: 4 1 0; }
.lm-graph-chan { flex: 1 1 0; }
.lm-canvas { flex: 1 1 auto; display: block; width: 100%; min-height: 0; touch-action: none; cursor: crosshair; }

/* Caption sits below the graph, left-aligned. */
.lm-caption {
  flex: 0 0 auto;
  padding: 3px 0 0 2px;
  display: flex; gap: 12px;
  font: 11px/1.2 'SF Mono', 'Menlo', 'Consolas', monospace;
  color: #c8c8c8;
}
/* A quarter-height row has little to spare, so the caption is smaller and
 * clips rather than wraps — every channel's block has to stay the same height
 * or the plots stop lining up. */
.lm-caption-chan {
  padding-top: 1px;
  font-size: 10px; gap: 10px;
  white-space: nowrap; overflow: hidden;
}
.lm-chan { color: #c8c8c8; }
.lm-air { color: #a8a8a8; }
/* One pill's worth of space off the window row, so the colour key reads as its
 * own thing rather than as another pill. */
.lm-legend { color: #7a7a7a; margin-left: 28px; white-space: nowrap;
             font: 11px/1.4 'SF Mono', 'Menlo', 'Consolas', monospace; }
.lm-legend .c-rns { color: #E8D040; }
.lm-legend .c-ours { color: #40A0FF; }
.lm-legend .c-rnode { color: #E89040; }
.lm-legend .c-bad { color: #E04048; }
</style>
