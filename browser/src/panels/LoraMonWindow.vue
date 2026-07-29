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
        <div class="lm-graphs">
          <div v-for="(win, i) in WINDOWS" :key="win.key" class="lm-graph">
            <canvas :ref="el => setCanvas(i, el)" class="lm-canvas" />
            <div class="lm-caption">
              <span class="lm-win">{{ win.label }}</span>
              <span class="lm-air">airtime
                <span class="c-rx">{{ airVals[win.key]?.rx ?? '0%' }} rx</span> /
                <span class="c-tx">{{ airVals[win.key]?.tx ?? '0%' }} tx</span>
              </span>
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

const props = defineProps<{ visible: boolean; title: string; focusToken?: number }>()
const emit = defineEmits<{ 'update:visible': [value: boolean] }>()

const device = useDeviceStore()

const isPhoneInit = window.matchMedia?.('(max-width: 599px)').matches ?? false
const defaultGeom = isPhoneInit
  ? { x: 0, y: 0, w: 100, h: 82 }
  : { x: 20, y: 6, w: 55, h: 84 }

const HOUR_MS = 3600 * 1000
const MAX_RADIOS = 4
const C_RX = '#4088E8'        // received frames (blue)
const C_TX = '#E8D040'        // transmitted frames (yellow)

const WINDOWS = [
  { key: '1m',  ms: 60 * 1000,   label: '1 min' },
  { key: '10m', ms: 600 * 1000,  label: '10 min' },
  { key: '1h',  ms: HOUR_MS,     label: '1 hour' },
] as const

interface Rec { t: number; dir: number; dur: number; bytes: number; rssi: number; snr10: number; txp: number }

/* recs = the active radio's packets, rebuilt each tick from the mirrored
 * `lora.<n>.packets` subtree (the firmware adds/deletes those nodes). */
let recs: Rec[] = []
let devClock = 0              // newest packet ms seen = device clock reference
let devClockAt = 0           // Date.now() when devClock was captured

const activeRadio = ref(0)
const airVals = ref<Record<string, { rx: string; tx: string }>>({})

/* Radios that exist = those the firmware has published a state key for. */
const radios = computed<number[]>(() => {
  const out: number[] = []
  for (let n = 0; n < MAX_RADIOS; n++)
    if (device.get(`lora.${n}.state`) != null) out.push(n)
  return out.length ? out : [0]
})

/* ── canvases (one per window) ── */
const canvases: (HTMLCanvasElement | null)[] = [null, null, null]
function setCanvas(i: number, el: unknown) { canvases[i] = (el as HTMLCanvasElement) ?? null }

/* Size the canvas backing store to its displayed pixels × DPR (full resolution,
 * tracks resize). */
function fit(cv: HTMLCanvasElement | null): { ctx: CanvasRenderingContext2D; w: number; h: number } | null {
  if (!cv) return null
  const dpr = window.devicePixelRatio || 1
  const w = Math.max(1, Math.round(cv.clientWidth * dpr))
  const h = Math.max(1, Math.round(cv.clientHeight * dpr))
  if (cv.width !== w) cv.width = w
  if (cv.height !== h) cv.height = h
  const ctx = cv.getContext('2d')
  if (!ctx) return null
  return { ctx, w, h }
}

function drawBands(ctx: CanvasRenderingContext2D, w: number, h: number) {
  const q = h / 4
  for (let i = 0; i < 4; i++) {
    const yTop = h - (i + 1) * q
    const g = ctx.createLinearGradient(0, yTop + q, 0, yTop)
    g.addColorStop(0, '#242424'); g.addColorStop(1, '#313131')
    ctx.fillStyle = g
    ctx.fillRect(0, yTop, w, q)
  }
}

const clamp01 = (x: number) => Math.max(0, Math.min(1, x))

/* Normalised bar height 0..1. RX: SNR-weighted link quality with RSSI secondary
 * (LoRa decodes below the noise floor, so SNR headroom is the meaningful metric).
 * TX: transmit power over the radio's −9..+22 dBm range. */
function barNorm(r: Rec): number {
  if (r.dir === 1) return clamp01((r.txp + 9) / (22 + 9))
  const snrN = clamp01((r.snr10 / 10 + 20) / 30)   // SNR −20 … +10 dB
  const rssiN = clamp01((r.rssi + 130) / 90)        // RSSI −130 … −40 dBm
  return 0.6 * snrN + 0.4 * rssiN
}

/* Estimated device clock (ms) now, extrapolated from the newest packet seen. */
function devNow(): number {
  return devClock ? devClock + (Date.now() - devClockAt) : 0
}

function drawGraph(i: number, winMs: number) {
  const f = fit(canvases[i])
  if (!f) return
  const { ctx, w, h } = f
  drawBands(ctx, w, h)
  const dnow = devNow()
  if (!dnow) return
  const lo = dnow - winMs
  for (const rec of recs) {
    const s = rec.t, e = rec.t + rec.dur
    if (e < lo || s > dnow) continue
    const xs = clamp01((s - lo) / winMs) * w
    const xe = clamp01((e - lo) / winMs) * w
    const bw = Math.max(1, xe - xs)
    const th = Math.max(1, h * 0.05)          // line thickness = 5% of graph height
    let y = h - barNorm(rec) * h              // line at the power level
    if (y > h - th) y = h - th
    if (y < 0) y = 0
    ctx.fillStyle = rec.dir ? C_TX : C_RX
    ctx.fillRect(xs, y, bw, th)
  }
}

function redraw() {
  for (let i = 0; i < WINDOWS.length; i++) drawGraph(i, WINDOWS[i].ms)
}

/* ── rebuild recs from the mirrored subtree ── */
function parseRec(t: number, s: string): Rec | null {
  const p = s.split('|')
  if (p[0] === 'r') return { t, dir: 0, rssi: +p[1], snr10: +p[2], dur: +p[3], bytes: +p[4], txp: 0 }
  if (p[0] === 't') return { t, dir: 1, txp: +p[1], dur: +p[2], bytes: +p[3], rssi: 0, snr10: 0 }
  return null
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

/* Airtime % over a window for one direction, overlap-counted. */
function pct(winMs: number, dir: number, dnow: number): string {
  if (!dnow) return '0%'
  const lo = dnow - winMs
  let busy = 0
  for (const r of recs) {
    if (r.dir !== dir) continue
    let s = r.t, e = r.t + r.dur
    if (e <= lo || s >= dnow) continue
    if (s < lo) s = lo
    if (e > dnow) e = dnow
    if (e > s) busy += e - s
  }
  const p = busy / winMs * 100
  return `${p.toFixed(p >= 10 ? 0 : 1)}%`
}

function tick() {
  if (props.visible) device.set('sys.stats.web_loramon', 1)   // heartbeat while open
  rebuild()
  const dnow = devNow()
  const v: Record<string, { rx: string; tx: string }> = {}
  for (const w of WINDOWS) v[w.key] = { rx: pct(w.ms, 0, dnow), tx: pct(w.ms, 1, dnow) }
  airVals.value = v
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

watch(activeRadio, () => {
  devClock = 0; devClockAt = 0; recs = []
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

.lm-graphs { flex: 1 1 auto; display: flex; flex-direction: column; gap: 12px; padding: 8px 8px 4px; min-height: 0; }
.lm-graph { flex: 1 1 0; display: flex; flex-direction: column; min-height: 0; }
.lm-canvas { flex: 1 1 auto; display: block; width: 100%; min-height: 0; }

/* Caption sits below its graph, left-aligned. */
.lm-caption {
  flex: 0 0 auto;
  padding: 3px 0 0 2px;
  display: flex; gap: 12px;
  font: 11px/1.2 'SF Mono', 'Menlo', 'Consolas', monospace;
  color: #c8c8c8;
}
.lm-win { color: #e8e8e8; }
.lm-air { color: #a8a8a8; }
.lm-caption .c-rx { color: #4088E8; }
.lm-caption .c-tx { color: #E8D040; }
</style>
