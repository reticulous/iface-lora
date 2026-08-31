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
            <!-- The span on screen, as storage lines, for pasting somewhere a
                 screenshot of a graph would not survive. -->
            <button class="lm-pill lm-copy" title="copy the frames in view as lora.n.packets lines"
                    @click="copyRecords">{{ copyMark }}</button>
          </template>
          <button v-else v-for="w in WINDOWS" :key="w.key" class="lm-pill"
                  :class="{ active: w.key === winKey }"
                  @click="winKey = w.key">{{ w.label }}</button>
          <span class="lm-legend">
            tx <span class="c-tx-bcast">all</span>/<span class="c-tx-uni">one</span>
            rx <span class="c-rx-bcast">all</span>/<span class="c-rx-us">us</span>/<span class="c-rx-other">other</span>
            / <span class="c-bad">CRC</span>
          </span>
          <span class="lm-axis lm-axis-rx">rx</span>
        </div>
        <div class="lm-graphs">
          <div class="lm-graph lm-graph-main">
            <canvas ref="canvasRef" class="lm-canvas"
                    @pointerdown="onDown"
                    @pointermove="onMove"
                    @pointerup="onUp"
                    @pointerleave="onLeave"
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
            <canvas :ref="el => setChanCanvas(c, el)" class="lm-canvas"
                    @pointerdown="onDown"
                    @pointermove="onMove"
                    @pointerup="onUp"
                    @pointerleave="onLeave"
                    @pointercancel="onUp" />
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

/* Colour is DIRECTION and AUDIENCE, which is what a person wants at a glance:
 * was that us talking, us being talked to, or somebody else's conversation we
 * happened to overhear. Red is ours going out, blue is ours coming in, and the
 * light/dark pair within each is broadcast against unicast.
 *
 * This replaced colour-as-protocol. Protocol was the wrong axis to spend hue
 * on — it is a property of a few frames and the same for whole runs of them —
 * and now that a frame can be inspected, what it IS can be read on demand while
 * hue carries the thing that has to be legible without asking. It also frees
 * the background: direction used to be a red cast behind every transmit, which
 * cost the whole plot's contrast to say what one bar's colour now says. */
const C_TX_BCAST = '#F08080'   // ours, to everyone — an announce
const C_TX_UNI   = '#B02020'   // ours, to one node
const C_RX_BCAST = '#80B8F0'   // theirs, to everyone
const C_RX_US    = '#2060C0'   // theirs, to us
const C_RX_OTHER = '#8A8A8A'   // theirs, to someone else — overheard
const C_BAD      = '#8050C8'   // failed its CRC: air held, nothing decoded

/* The politeness marks — what the frame waited before it went out. Near-white
 * and thin: they are annotation on a bar, not a quantity to compare against
 * one, and over short bursts anything stronger dominates the airtime it is
 * describing. */
const C_WAIT = '#E8E8E8'

/* Band gradient, bottom → top of each band. One background now, with no cast
 * behind a transmit: direction is in the bar's own colour, so tinting the field
 * as well spent the plot's contrast to repeat something already said. The
 * darkest tone doubles as the timescale grid, so the grid reads as part of the
 * background. */
const BG_LO = '#242424', BG_HI = '#313131'
const C_GRID = BG_LO
/* The veil over spans the radio spent on another channel. Nearly opaque black:
 * the point is that an unwatched stretch should not read as a quiet one, so it
 * has to be plainly darker than the darkest band rather than a shade of it. */
const C_UNHELD = 'rgba(0,0,0,0.72)'

const WINDOWS = [
  { key: '10s', ms: 10 * 1000,  label: '10s' },
  { key: '1m',  ms: 60 * 1000,  label: '1m'  },
  { key: '5m',  ms: 300 * 1000, label: '5m'  },
  { key: '10m', ms: 600 * 1000, label: '10m' },
  { key: '30m', ms: 1800 * 1000, label: '30m' },
  { key: '1h',  ms: HOUR_MS,    label: '1hr' },
] as const

/* How far behind "now" the live edge sits. Past the 1 Hz publish beat with room
 * for the batch a burst is flushed in, so what reaches the right edge is settled
 * and the graph never corrects itself in front of you. */
const LAG_MS = 1500

/* One plot, two dBm axes reading the same four bands: transmit power down the
 * left gutter in 10 dB steps, received strength down the right in 32 dB. RX
 * needs the wider step because its range is 128 dB against TX's 40, and both
 * have to land on the same band edges for a single grid to serve them.
 *
 * The RX axis is the reportable range itself, not a guess at a comfortable one:
 * the packet-strength register is a byte read as −value/2 dBm, so 0 is the
 * strongest level a receiver can state and −127.5 the weakest. A bench pair a
 * hand apart reads well above −30, and an axis that stopped there put every one
 * of those in the top band with nothing to tell them apart. */
const NBANDS = 4
const AX_TX = { lo: -10, hi: 30 }     // 10 dB per band
const AX_RX = { lo: -128, hi: 0 }     // 32 dB per band

/* The channel-noise floor the traffic sits on: very light grey, so a bar always
 * wins the pixels it lands on and the floor reads as background texture. */
const C_FLOOR = 'rgba(255,255,255,0.20)'

/* The frame around each lane's plot area. Light enough to stay background, solid
 * enough to survive a lane with nothing in it. */
const C_BEZEL = 'rgba(255,255,255,0.30)'

/* Pills — the train's peer name and the noise floor's figure. Both are notes
 * stuck on the graph rather than part of it, so they take a paper colour and
 * black text instead of joining the plot's palette, and both use this one style:
 * two label styles for one job is one too many. */
const C_PILL_BG = '#ffffcc'
const C_PILL_FG = '#000000'

interface Rec { t: number; dir: number; dur: number; bytes: number; rssi: number; snr10: number; txp: number; type: number; wait: number; own: number; ch: number; desc: number; cast: number; tag: string }

/* How far two dwell records may sit apart and still be one stay on a channel.
 * The device already merges a stay into one record and breaks it on every
 * frame, so the pieces of one slot are adjacent to the millisecond and this is
 * only slack against the beat's quantisation. Anything wider means the radio
 * left and came back, which is a second slot however the first was labelled. */
const SLOT_JOIN_MS = 20

/* Who a frame was aimed at, as the device decided it — the browser cannot, since
 * it turns on which addresses mean US and that lives in the peer table. */
const CAST_BCAST = 0, CAST_US = 1, CAST_OTHER = 2, CAST_US_LINK = 3
/* Ours either way — the split is about whether the far end is nameable at all. */
const isOurs = (c: number) => c === CAST_US || c === CAST_US_LINK

/* Direction and audience, which is what the colour says. A CRC failure is
 * neither: nothing in it was readable, so it gets its own hue rather than a
 * guess. */
function colourOf(rec: Rec): string {
  if (rec.type === 3) return C_BAD
  if (rec.dir === 1) return rec.cast === CAST_BCAST ? C_TX_BCAST : C_TX_UNI
  if (rec.cast === CAST_BCAST) return C_RX_BCAST
  return isOurs(rec.cast) ? C_RX_US : C_RX_OTHER
}

/* What a frame is, indexed by the code the device writes into the record. The
 * names live here rather than on the wire because every frame is a storage node
 * and there may be thousands: the device sends a byte, each viewer holds its
 * own table. Codes are appended to, never renumbered. */
const DESC = [
  '', 'PRIVSYNC', 'ANNOUNCE2', 'HAVEDATA', 'GIMME', 'THATSIT', 'BYE', 'RESEND',
  'data', 'announce', 'link request', 'proof', 'split', 'RNode',
] as const
const descOf = (d: number) => DESC[d] ?? ''

/* recs = the active radio's packets, rebuilt each tick from the mirrored
 * `lora.<n>.packets` subtree (the firmware adds/deletes those nodes). */
let recs: Rec[] = []

/* Channel RSSI, accumulated live rather than mirrored as history: the firmware
 * publishes only the newest reading (`lora.<n>.rssi` = "<ms>|<ch0>|<ch1>|…"),
 * so the series starts when the window opens — the same rule the packet nodes
 * follow. A beat the radio skipped (carrier sense had it) republishes nothing,
 * so the key is unchanged, no point is appended, and the gap draws as a gap.
 *
 * A device measures the channel it is camped on and never retunes away to
 * sample another, so in practice one field arrives and the backdrop appears
 * under the hailing graph alone. The agile lanes are not empty for it: what
 * fills them is traffic, which is recorded per channel regardless. */
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

/* The device stamps its records with millis(), so a reboot restarts that clock
 * near zero while our extrapolation keeps marching forward. Everything the
 * device publishes afterwards then arrives tens of thousands of seconds "in the
 * past", falls left of every window, and nothing draws again for as long as the
 * window stays mounted — which is why the anchor is monotonic ONLY within a
 * boot. A record a whole hour behind "now" is not transport jitter (the device
 * expires its own nodes at an hour, so it can never publish one that old): it
 * is a device counting again from the start, and the anchor follows it back.
 * The floor series and any zoom span are in the old boot's timebase, so they go
 * with it, exactly as they do when the radio tab changes. */
function restarted(t: number): boolean {
  return devClock > 0 && t > 0 && t < devNow() - HOUR_MS
}
function reanchor(t: number) {
  devClock = t
  devClockAt = Date.now()
  floorSeries = Array.from({ length: CH_MAX }, () => [])
  floorLastMs = 0
  zoomStack.value = []
  sel.value = null
}

/* The time span currently on screen: the top of the zoom stack if there is
 * one, else the live window ending at "now". */
function view(): { lo: number; hi: number } {
  const top = zoomStack.value[zoomStack.value.length - 1]
  if (top) return { lo: top.t0, hi: top.t1 }
  /* The live edge stops short of "now" by LAG_MS. The most recent moment is not
   * a finished picture: a stay on one channel is one record that grows, so the
   * background arrives a step at a time, and frame records are batched, so a
   * packet lands in a column the graph has already drawn as empty. Both read as
   * the graph correcting itself in public — blocks filling in at the right edge
   * and bars appearing out of nowhere behind them.
   *
   * Holding the edge back past the slowest of those publishers shows only what
   * has settled. It costs a second and a half of latency, which on a plot whose
   * shortest window is ten seconds is not a cost anybody is watching for. */
  const dnow = devNow() - LAG_MS
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

/* The lane's own outline. A lane the radio never visited is veiled end to end
 * and has no traffic to give it shape, so without this it is a rectangle of dark
 * against a dark window — indistinguishable from the gap between two lanes, and
 * from no lane at all. The bezel is what says a graph is there and empty rather
 * than absent, which is why it is drawn both before the traffic (the lane that
 * has nothing to draw yet still gets its frame) and again after it (nothing may
 * eat an edge). */
function drawBezel(ctx: CanvasRenderingContext2D, w: number, h: number,
                   dpr: number, gl: number, gr: number) {
  ctx.strokeStyle = C_BEZEL
  ctx.lineWidth = Math.max(1, dpr)
  ctx.strokeRect(gl + 0.5, 0.5, (w - gr) - gl - 1, h - 1)
}

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
  drawBezel(ctx, w, h, dpr, gl, gr)
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

  /* Where the radio was NOT here, the lane goes dark.
   *
   * A lane with nothing in it answers two different questions the same way —
   * nobody spoke, or we were somewhere else — and on a hopping radio the second
   * is the usual one. The dwell records say which: the plot is drawn at full
   * strength only across the spans this channel actually held the receiver, and
   * everything else is veiled. Read the lane as: bright means listening, and a
   * gap in the bright means the radio was on another lane at that moment.
   *
   * Drawn over the bands and under everything else, so the axis still reads
   * through it and traffic still lands on top at full contrast. */
  {
    const held: { x0: number; x1: number }[] = []
    for (const rec of recsCh) {
      if (rec.dir !== 2) continue
      const e = rec.t + rec.dur
      if (e < lo || rec.t > hi) continue
      held.push({ x0: xAt(rec.t), x1: xAt(e) })
    }
    held.sort((a, b) => a.x0 - b.x0)
    ctx.fillStyle = C_UNHELD
    let x = gl
    for (const s2 of held) {
      if (s2.x0 > x) ctx.fillRect(x, 0, s2.x0 - x, h)
      if (s2.x1 > x) x = s2.x1
    }
    if (x < w - gr) ctx.fillRect(x, 0, (w - gr) - x, h)
  }

  /* The channel noise floor: each sample a bar from the bottom of the plot up
   * to its dBm on the RX axis, held until the next sample so a 1 Hz series
   * reads as a continuous floor rather than a picket fence. Drawn under the
   * frames — traffic lies on top of the noise it had to get above.
   *
   * A gap in the series is a beat carrier sense took the radio for. It is left
   * empty on purpose: the bar would otherwise be drawn from a reading that
   * described the transmission we were queued behind. */
  /* The line goes down here, under the traffic; its pill is held back and drawn
   * with the other pills, above the division grid (see the pill layer below). */
  let floorY: number | null = null, floorLabel = ''
  if (main) {
    const nf = noiseFloor(ch)
    if (nf != null) {
      const y = Math.round(h - clamp01((nf - AX_RX.lo) / (AX_RX.hi - AX_RX.lo)) * h) + 0.5
      /* Between the gutters and no further: the line is a level ON the plot, and
       * running it through the scales strikes out the very numbers it is read
       * against. */
      ctx.save()
      ctx.strokeStyle = C_FLOOR
      ctx.lineWidth = Math.max(2, 2 * dpr)
      ctx.setLineDash([2 * dpr, 3 * dpr])
      ctx.beginPath(); ctx.moveTo(gl, y); ctx.lineTo(w - gr, y); ctx.stroke()
      ctx.restore()
      floorY = y
      floorLabel = `noise floor ${Math.round(nf)} dBm`
    }
  }

  /* Frozen view only: a live one slides, and a grid on absolute time would
   * crawl across it. Lines are laid on round multiples of the step, and drawn
   * FIRST — below the traffic, below the pills, below everything else the lane
   * carries. It is the ruling on the paper: part of the background, never
   * crossing anything drawn on top of it. */
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
    if (rec.dir === 2) continue                 /* a dwell is background, not a frame */
    const s = rec.t, e = rec.t + rec.dur
    if (e < lo || s > hi) continue
    const xs = xAt(s)
    const bw = Math.max(1, xAt(e) - xs)
    /* Never thinner than two device-independent pixels. On an agile lane at a
     * quarter height the proportional term falls under one pixel, and a frame
     * that rounds away is a frame the graph is lying about. */
    const th = Math.max(2 * dpr, h * 0.05)
    const ax = rec.dir === 1 ? AX_TX : AX_RX
    const dbm = rec.dir === 1 ? rec.txp : rec.rssi
    let y = h - clamp01((dbm - ax.lo) / (ax.hi - ax.lo)) * h
    if (y > h - th) y = h - th
    if (y < 0) y = 0
    const col = colourOf(rec)
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

  /* The noise floor's figure, held back from its line so it lands in the pill
   * layer. It sits ON the line, so the number and the level it describes are
   * one object rather than a legend to cross-reference — and at the right end
   * of it, where the rx axis it is quoting runs, held clear of the scale
   * itself. Same cream note as a peer pill: both are labels stuck on the graph,
   * and two label styles for one job is one too many. */
  if (floorY != null) {
    ctx.font = `${9 * dpr}px 'SF Mono','Menlo','Consolas',monospace`
    const ph = 12 * dpr, pad = 4 * dpr, rad = 4 * dpr
    const pw = ctx.measureText(floorLabel).width + pad * 2
    const px = w - gr - 8 * dpr - pw
    /* Kept whole inside the lane even where the line runs along an edge: half a
     * pill is unreadable, and the line itself already says which level it
     * belongs to. */
    let py = floorY - ph / 2
    if (py < 0) py = 0
    if (py + ph > h) py = h - ph
    ctx.beginPath()
    ctx.roundRect(px, py, pw, ph, rad)
    ctx.fillStyle = C_PILL_BG
    ctx.fill()
    ctx.fillStyle = C_PILL_FG
    ctx.textBaseline = 'middle'
    ctx.fillText(floorLabel, px + pad, py + ph / 2)
  }

  /* Who the lane is with, named where the answer starts and where it changes.
   *
   * A pill is LEFT-ALIGNED on the moment it becomes true and stands until the
   * next one contradicts it — the same reading as a name on a timeline, and the
   * reason it is anchored to a point rather than fitted to a run: a run's width
   * changes with the zoom, so a label fitted to one moves every time the view
   * does, and a label that will not fit disappears entirely from traffic that
   * is plainly there.
   *
   * The two channel kinds fall out of ONE rule — mark wherever the attribution
   * changes — because they carry the answer in different records, and they
   * differ only in which side of the mark the pill sits:
   *
   *   - the hailing channel has no meetings, so only frames are tagged: the
   *     first attributable frame on screen gets a pill (nothing precedes it in
   *     view to have established the name), and after that every change does.
   *     The pill starts AT the frame — the traffic runs on to the right of it,
   *     and the name is the head of that run.
   *   - a detour channel is attended in slots, and a slot belongs to its peer
   *     for its whole width even if nothing arrives in it. The dwell carries
   *     that, so a slot is named on its own beginning rather than on whichever
   *     frame happened to be first inside it, and an empty slot is still named.
   *     The pill sits just BEFORE the slot opens, clear of it: a window is a
   *     bounded thing with its own left edge, and a label laid over that edge
   *     hides where the window starts — which on a lane whose whole point is
   *     "we were listening from here to here" is the one thing not to cover.
   *
   * A dwell additionally starts a new pill when it does not continue the
   * previous stay, so two consecutive slots with the SAME peer read as two
   * slots. A frame never does: an untagged frame between two of a peer's
   * frames — somebody else's broadcast, overheard — is not a change of who
   * this lane is with, and clearing the run on it would re-label the same
   * conversation every time the air was shared.
   *
   * A pill that would collide with the one before it, or run past the right
   * gutter, is dropped rather than moved: a name that has slid off the moment
   * it belongs to is worse than no name. */
  {
    ctx.font = `${9 * dpr}px 'SF Mono','Menlo','Consolas',monospace`
    const ph = 12 * dpr, pad = 4 * dpr, rad = 4 * dpr, gap = 4 * dpr
    /* `before` = the mark opens a listening window, so the pill goes to its
     * left; otherwise the mark is a frame and the pill starts on it. */
    const marks: { t: number; tag: string; before: boolean; cast: number }[] = []
    let cur = '', prevEnd = -Infinity
    for (const rec of recsCh) {
      const e = rec.t + rec.dur
      if (e < lo || rec.t > hi) continue     /* on screen: the first one in view
                                              * is the first that can be named */
      if (rec.tag) {
        const slot = rec.dir === 2
        const newSlot = slot && rec.t - prevEnd > SLOT_JOIN_MS
        if (rec.tag !== cur || newSlot) {
          marks.push({ t: rec.t, tag: rec.tag, before: slot, cast: rec.cast })
          cur = rec.tag
        }
      }
      prevEnd = e
    }
    let lastRight = -Infinity
    for (const mk of marks) {
      const label = peerLabel(mk.tag, mk.cast)
      if (!label) continue
      const pw = ctx.measureText(label).width + pad * 2
      const px = mk.before ? xAt(mk.t) - gap - pw : xAt(mk.t)
      if (px < gl || px < lastRight + gap || px + pw > w - gr) continue
      ctx.beginPath()
      ctx.roundRect(px, 1 * dpr, pw, ph, rad)
      ctx.fillStyle = C_PILL_BG
      ctx.fill()
      ctx.fillStyle = C_PILL_FG
      ctx.textBaseline = 'middle'
      ctx.fillText(label, px + pad, 1 * dpr + ph / 2)
      lastRight = px + pw
    }
  }

  /* On every lane, not just the one under the pointer: the stack shares one
   * time axis, so the band is the same column everywhere and showing it whole
   * is what makes the selection legible. */
  if (sel.value) {
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

  drawBezel(ctx, w, h, dpr, gl, gr)   /* again, over the traffic it encloses */

  /* The column under the pointer, and what the frame there IS. Drawn last, over
   * everything: it is an answer to a question just asked, so it may cover the
   * picture it is describing. Every lane draws its own — the hairline marks the
   * instant in all of them, and whichever lane holds a frame at that instant is
   * the one that gets the pill. */
  if (hoverFrac.value != null) {
    /* The line stands where the pointer is; the instant under it is whatever the
     * view puts there this frame. On a live graph that instant advances while
     * the hand holds still, which is the point — the cursor is not a bookmark. */
    const ht = lo + hoverFrac.value * (hi - lo)
    const hx = Math.round(gl + hoverFrac.value * span) + 0.5
    ctx.save()
    ctx.strokeStyle = 'rgba(255,255,255,0.35)'
    ctx.lineWidth = Math.max(1, dpr)
    ctx.beginPath(); ctx.moveTo(hx, 0); ctx.lineTo(hx, h); ctx.stroke()
    ctx.restore()
    let hit: Rec | null = null
    for (const rec of recsCh) {
      if (rec.dir === 2) continue
      if (ht >= rec.t && ht <= rec.t + rec.dur) { hit = rec; break }
    }
    if (hit) {
      const name = descOf(hit.desc)
      /* Who it was with, by the best name there is for them: what they announced,
       * the number `lora n` gives them, or — only when the tag resolves to
       * nobody at all — the six hex characters themselves. The tag is what a log
       * line quotes, so it is the fallback rather than a prefix on every pill:
       * once there is a name, the hex is the part nobody reads. The hover has
       * room for the cast as well, which is what turns the one unresolvable
       * address there is a reason for into `inbound link`. */
      const who = peerLabel(hit.tag, hit.cast)
      const label = [name, `${hit.bytes}B`, who].filter(Boolean).join(' · ')
      ctx.font = `${10 * dpr}px 'SF Mono','Menlo','Consolas',monospace`
      const tw = ctx.measureText(label).width
      const pad = 4 * dpr, ph = 14 * dpr
      const ax = hit.dir === 1 ? AX_TX : AX_RX
      const dbm = hit.dir === 1 ? hit.txp : hit.rssi
      let y = h - clamp01((dbm - ax.lo) / (ax.hi - ax.lo)) * h
      /* Above the bar where there is room, below it where there is not. */
      let py = y - ph - 2 * dpr
      if (py < 0) py = Math.min(h - ph, y + 3 * dpr)
      let px = xAt(hit.t)
      if (px + tw + pad * 2 > w) px = Math.max(0, w - tw - pad * 2)
      ctx.fillStyle = 'rgba(12,12,12,0.92)'
      ctx.fillRect(px, py, tw + pad * 2, ph)
      ctx.fillStyle = '#e8e8e8'
      ctx.textBaseline = 'middle'
      ctx.fillText(label, px + pad, py + ph / 2)
    }
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
/* Geometry from the canvas the gesture landed on, not from the main graph:
 * every lane is the same width with the same gutters and the same time axis, so
 * a column means the same moment in all of them — which is exactly why a swipe
 * should work wherever the pointer happens to be. Reaching for the main graph
 * to zoom is a rule with no reason behind it. */
/* Where the pointer is, shared by every lane. Hover is a COLUMN, not a spot in
 * one graph: the lanes share a time axis, so the honest question a pointer asks
 * is "what happened at this instant", and the answer may be in a lane the
 * pointer is nowhere near.
 *
 * Held as a FRACTION of the plot width, not as a device time — which is the
 * opposite of the selection anchor below, and deliberately so. A selection is a
 * span of time and must keep meaning the same span while the graph slides; a
 * cursor is where your hand is, and a live view sliding out from under a
 * time-anchored line walks it off to the left while the mouse has not moved.
 * The frame the line is over changes underneath it, which is exactly right: the
 * pointer is asking about whatever is there now. */
const hoverFrac = ref<number | null>(null)

function xToFrac(ev: PointerEvent): number | null {
  const cv = (ev.currentTarget as HTMLCanvasElement | null) ?? canvasRef.value
  if (!cv) return null
  const rect = cv.getBoundingClientRect()
  const span = rect.width - GUT_L_CSS - GUT_R_CSS
  if (span <= 0) return null
  return clamp01((ev.clientX - rect.left - GUT_L_CSS) / span)
}

function xToTime(ev: PointerEvent): number | null {
  const frac = xToFrac(ev)
  if (frac == null) return null
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
  const frac = xToFrac(ev)
  if (frac == null) return
  if (!sel.value) {                 /* not dragging: this is a hover */
    hoverFrac.value = frac
    redraw()
    return
  }
  hoverFrac.value = null            /* a drag is a zoom gesture, not an inspection */
  const { lo, hi } = view()
  sel.value = { anchor: sel.value.anchor, cur: lo + frac * (hi - lo) }
}

function onLeave() {
  if (hoverFrac.value == null) return
  hoverFrac.value = null
  redraw()
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

/* The neighbourhood the device publishes: `lora.<n>.peers.<slot>` =
 * "<num>|<supe>|<tags…>|<names…>|…". A tag is three bytes and means nothing on
 * its own; this is what turns one back into a node with a name, which is the
 * whole reason a frame's tag is worth carrying. Rebuilt from the mirror each
 * tick — peers come and go, and a stale map is worse than none. */
interface Peer { num: number; supe: boolean; names: string[] }
const peerByTag = ref<Map<string, Peer>>(new Map())

/* What to call a node on screen: its name if anything announced one, and the
 * number `lora n` gives it otherwise — so the graph and the console agree, and
 * an unnamed node is still something you can refer to.
 *
 * Pass `cast` where there is room to say more than a name. A tag that resolves
 * to nobody is usually a gap, but on a link the device says is ours it is not:
 * a link request carries no sender and its identify step is encrypted inside
 * the session, so the far end of a link dialled TO us is anonymous for the
 * link's whole life. Six hex characters with nothing behind them read as a
 * failure; saying which kind of address it is answers the question instead. */
function peerLabel(tag: string, cast?: number): string {
  if (!tag) return ''
  const p = peerByTag.value.get(tag)
  if (p) return p.names.length ? p.names.join(',') : `#${p.num}`
  /* An address of this node's own resolves to nobody here on purpose: the
   * published table is the NEIGHBOURHOOD, and we are not in it. Every frame
   * addressed to us carries one, so without this the commonest tag on the pane
   * is six characters of hex — and the device already said which kind it is. */
  if (cast === CAST_US)      return 'us'
  if (cast === CAST_US_LINK) return `${tag} · inbound link`
  return tag
}

function rebuildPeers() {
  const m = new Map<string, Peer>()
  const sub = device.get(`lora.${activeRadio.value}.peers`)
  if (sub && typeof sub === 'object') {
    for (const v of Object.values(sub as Record<string, unknown>)) {
      const f = String(v).split('|')
      const peer: Peer = {
        num: +(f[0] ?? 0),
        supe: f[1] === '1',
        names: (f[3] ?? '').split(',').filter(Boolean),
      }
      for (const t of (f[2] ?? '').split(',')) if (t) m.set(t, peer)
    }
  }
  peerByTag.value = m
}

/* The channel's noise floor: the quietest reading it has given in the last
 * minute. A minute rather than the window on screen, because this is a statement
 * about conditions now — zooming out to an hour should not turn it into the
 * quietest moment of that hour — and the quietest rather than the mean, because
 * a mean over a channel carrying traffic is a measure of the traffic.
 *
 * Two kinds of non-reading are dropped, and dropping them is the whole reason
 * this can return null: a floor nobody can vouch for is worse than none.
 *   - a zero is a beat that produced no reading at all;
 *   - anything at the register's weakest rail is the RAIL, not the channel. The
 *     level is a byte read as −value/2 dBm, so −127.5 is as quiet as it can say,
 *     and it says that whenever the front end has nothing yet — which is exactly
 *     the first minutes after a radio comes up. It cannot be a measurement
 *     either: thermal noise alone is about −123 dBm in 125 kHz before any
 *     receiver's own noise figure, so no working front end reads below it. */
const NOISE_WINDOW_MS = 60 * 1000
const NOISE_RAIL_DBM = -127
function noiseFloor(ch: number): number | null {
  const s = floorSeries[ch]
  if (!s || !s.length) return null
  const cut = s[s.length - 1].t - NOISE_WINDOW_MS
  let lo: number | null = null
  for (let i = s.length - 1; i >= 0 && s[i].t >= cut; i--) {
    const d = s[i].dbm
    if (!d || d <= NOISE_RAIL_DBM) continue
    if (lo == null || d < lo) lo = d
  }
  return lo
}

/* ── rebuild recs from the mirrored subtree ── */
function parseRec(t: number, s: string): Rec | null {
  const p = s.split('|')
  if (p[0] === 'r') return { t, dir: 0, rssi: +p[1], snr10: +p[2], dur: +p[3], bytes: +p[4], txp: 0, type: +(p[5] ?? 0), wait: 0, own: 0, ch: +(p[6] ?? 0), desc: +(p[7] ?? 0), cast: +(p[8] ?? 0), tag: p[9] ?? '' }
  if (p[0] === 't') return { t, dir: 1, txp: +p[1], dur: +p[2], bytes: +p[3], rssi: 0, snr10: 0, type: +(p[4] ?? 0), wait: +(p[5] ?? 0), ch: +(p[6] ?? 0), own: +(p[7] ?? 0), desc: +(p[8] ?? 0), cast: +(p[9] ?? 0), tag: p[10] ?? '' }
  /* A dwell: the radio was tuned here and listening for this long. Not a frame
   * — it carries no level and is never drawn as one; it is what tells a lane
   * apart from a lane nobody was watching. Its tag, where it has one, is the
   * meeting whose slot the stay is: the slot belongs to that peer for its whole
   * width, and on a detour channel it is the ONLY thing that says so, since a
   * slot may pass with nothing arriving in it. */
  if (p[0] === 'a') return { t, dir: 2, ch: +p[1], dur: +p[2], bytes: 0, rssi: 0, snr10: 0, txp: 0, type: 0, wait: 0, own: 0, desc: 0, cast: 0, tag: p[3] ?? '' }
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
  /* Checked here too, not just in rebuild(): on a quiet channel the sample beat
   * is the only thing publishing, so it has to be able to spot the restart by
   * itself. Before floorLastMs moves — reanchor() clears it. */
  if (restarted(t)) reanchor(t)
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
  /* The newest record actually in the mirror, 0 when there are none — kept
   * apart from devClock so an idle radio (nothing published for over an hour)
   * cannot read as a restart. */
  let seen = 0
  for (const k in tree) {
    const t = Number(k)
    if (!Number.isFinite(t)) continue
    const rec = parseRec(t, String(tree[k]))
    if (rec) { arr.push(rec); if (t > seen) seen = t }
  }
  arr.sort((a, b) => a.t - b.t)
  recs = arr
  if (restarted(seen)) { reanchor(seen); return }
  /* Anchor device-now monotonically within the boot: never pull it backward
   * (that snap caused the new-bar jerk) — advance to the newest packet or the
   * local extrapolation, whichever is later. */
  const anchor = Math.max(seen, devNow())
  if (anchor > 0) { devClock = anchor; devClockAt = Date.now() }
}

/* ── the span on screen, as text ──
 *
 * A frozen view is usually looked at because something in it needs explaining
 * to someone else, and a graph does not paste into a bug report. This lifts the
 * frames under the current span out as the device's own `show lora.<n>.packets`
 * would print them — the same `key = value` lines, in time order — so a span
 * lands beside CLI output in the same format and nothing has to be transcribed.
 *
 * A frame is in view when its time on air OVERLAPS the span, which is the rule
 * that decides whether it is drawn: a bar clipped by the left edge is on screen
 * and belongs in the copy. The raw value comes from the mirror rather than from
 * the parsed record, so what is pasted is exactly what the device published. */
const copyMark = ref('⧉')
let copyTimer: ReturnType<typeof setTimeout> | null = null

function markCopy(mark: string) {
  copyMark.value = mark
  if (copyTimer) clearTimeout(copyTimer)
  copyTimer = setTimeout(() => { copyMark.value = '⧉'; copyTimer = null }, 1200)
}

async function copyRecords() {
  const { lo, hi } = view()
  const tree = device.get(`lora.${activeRadio.value}.packets`) ?? {}
  const lines: string[] = []
  for (const r of recs) {                       // recs is already time-sorted
    if (r.t + r.dur <= lo || r.t >= hi) continue
    const raw = tree[String(r.t)]
    if (raw == null) continue
    lines.push(`lora.${activeRadio.value}.packets.${r.t} = ${String(raw)}`)
  }
  if (!lines.length) { markCopy('∅'); return }
  try {
    await navigator.clipboard.writeText(lines.join('\n') + '\n')
    markCopy('✓')
  } catch {
    /* No clipboard (an insecure context denies it outright), so say so on the
     * button rather than looking like a copy that worked. */
    markCopy('✕')
  }
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
  if (props.visible) {
    device.set('sys.stats.web_loramon', 1)     // heartbeat while open
    /* The neighbourhood is a separate appetite from the frames, and the device
     * only publishes it while something says it is reading — so say so. */
    device.set('sys.stats.web_peers', 1)
  }
  pollChans()
  pollFloor()
  rebuildPeers()
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
  if (copyTimer) { clearTimeout(copyTimer); copyTimer = null }
  device.set('sys.stats.web_loramon', 0)
  device.set('sys.stats.web_peers', 0)
})

watch(() => props.visible, v => {
  device.set('sys.stats.web_loramon', v ? 1 : 0)
  device.set('sys.stats.web_peers', v ? 1 : 0)
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
/* Sits with the zoom label rather than with the pills: it acts on the frozen
 * span, not on the choice of window. Fixed width so the mark it flashes back
 * cannot shift the label beside it. */
.lm-copy { margin-left: 8px; padding: 2px 0; width: 26px; text-align: center; line-height: 1.4; }

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
 * the heights: the hailing channel takes 2, each agile channel 1. */
.lm-graphs { flex: 1 1 auto; display: flex; flex-direction: column; gap: 8px; padding: 2px 8px 4px; min-height: 0; }
.lm-graph { display: flex; flex-direction: column; min-height: 0; min-width: 0; }
.lm-graph-main { flex: 2 1 0; }
.lm-graph-chan { flex: 1 1 0; }
.lm-canvas { flex: 1 1 auto; display: block; width: 100%; min-height: 0; touch-action: none; cursor: crosshair; }

/* Caption sits below the graph and starts where the graph does: the left inset
 * is the canvas's own tx gutter, so the channel it names lines up with the plot
 * rather than with the scale beside it. Kept in step with GUT_L_CSS by hand —
 * a stylesheet cannot read it. */
.lm-caption {
  flex: 0 0 auto;
  padding: 3px 0 0 30px;
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
/* Kept in step with the canvas constants of the same names by hand — a stylesheet
 * cannot read them, and a key in different colours from the plot is worse than
 * no key at all. */
.lm-legend .c-tx-bcast { color: #F08080; }
.lm-legend .c-tx-uni   { color: #B02020; }
.lm-legend .c-rx-bcast { color: #80B8F0; }
.lm-legend .c-rx-us    { color: #2060C0; }
.lm-legend .c-rx-other { color: #8A8A8A; }
.lm-legend .c-bad      { color: #8050C8; }
</style>
