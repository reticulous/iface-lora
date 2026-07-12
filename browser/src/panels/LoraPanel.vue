<template>
  <div class="q-gutter-y-md">
    <PanelHeading title="LoRa" />

    <div class="text-caption" style="opacity:0.7">
      LoRa interface. Set frequency and bandwidth directly, pick a spreading
      factor and coding rate; the radio stays disabled until you enable it.
    </div>

    <SettingToggle label="Enabled" k="s.lora.0.enable" />

    <div class="section-heading">Radio</div>

    <div class="row items-center no-wrap">
      <div class="col-4 text-caption">Frequency</div>
      <q-input class="col" :model-value="freqText" dense outlined debounce="500"
        suffix="MHz" inputmode="decimal" placeholder="e.g. 869.525"
        @update:model-value="setFreq" />
    </div>
    <div class="row items-center no-wrap">
      <div class="col-4 text-caption">Bandwidth</div>
      <q-input class="col" :model-value="bwText" dense outlined debounce="500"
        suffix="kHz" inputmode="decimal" placeholder="e.g. 125"
        @update:model-value="setBw" />
    </div>
    <SettingSelect label="Spreading factor" k="s.lora.0.spreading_factor" :options="sfOptions" />
    <SettingSelect label="Coding rate" k="s.lora.0.coding_rate" :options="crOptions" />
    <SettingSlider label="TX power (dBm)" k="s.lora.0.tx_power" :min="-9" :max="22" />
    <SettingSlider label="Preamble (sym)"  k="s.lora.0.preamble"  :min="6"  :max="32" />
    <SettingText   label="Sync word"       k="s.lora.0.sync_word" />

    <div class="section-heading">RNS interface</div>

    <SettingSelect label="Mode" k="s.lora.0.mode" :options="modeOptions" />

    <q-expansion-item dense dense-toggle label="Advanced" header-class="text-caption" class="q-mt-xs">
      <div class="q-pl-sm q-gutter-y-sm q-pt-sm">
        <div class="text-caption" style="opacity:0.6">
          IFAC (Interface Access Codes): a network name + passphrase that must
          match every peer on this interface, or traffic is dropped. Leave
          both blank for an open interface.
        </div>
        <SettingText label="IFAC network" k="s.lora.0.ifac_netname" />
        <div class="row items-center no-wrap">
          <div class="col-4 text-caption">IFAC passphrase</div>
          <q-input class="col" :model-value="ifacKey" type="password" dense outlined
            debounce="600" placeholder="(write-only — set to change)"
            autocomplete="new-password" autocorrect="off" autocapitalize="off" spellcheck="false"
            @update:model-value="setIfacKey" />
        </div>
      </div>
    </q-expansion-item>

    <q-separator dark class="q-mt-md" />

    <div class="row items-center no-wrap">
      <div class="col-4 text-caption">State</div>
      <div class="col">
        <q-badge v-if="state === 'up'"           color="green">up</q-badge>
        <q-badge v-else-if="state === 'starting'" color="orange">starting</q-badge>
        <q-badge v-else-if="state === 'error'"    color="red">error</q-badge>
        <q-badge v-else-if="state === 'unconfigured'" color="grey">unconfigured</q-badge>
        <q-badge v-else                           color="grey">{{ state || 'down' }}</q-badge>
      </div>
    </div>

    <div v-if="chip" class="row items-center no-wrap">
      <div class="col-4 text-caption">Chip</div>
      <div class="col text-caption">{{ chip }}</div>
    </div>
    <div v-if="bitrate" class="row items-center no-wrap">
      <div class="col-4 text-caption">Bitrate</div>
      <div class="col text-caption">{{ bitrate }} bit/s</div>
    </div>
    <div v-if="rssi || snr" class="row items-center no-wrap">
      <div class="col-4 text-caption">Last RX</div>
      <div class="col text-caption">RSSI {{ rssi }} dBm · SNR {{ snr }} dB</div>
    </div>
    <div class="row items-center no-wrap">
      <div class="col-4 text-caption">Frames</div>
      <div class="col text-caption">rx {{ rxFrames }} · tx {{ txFrames }}</div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed, ref } from 'vue'
import { useDeviceStore } from 'spangap-browser/stores/device'

const device = useDeviceStore()

// IFAC passphrase lives in secrets.* — never synced to the browser, so this
// field is write-only.
const ifacKey = ref('')
function setIfacKey(val: string | number | null) {
  ifacKey.value = String(val ?? '')
  device.set('secrets.lora.0.ifac_netkey', ifacKey.value)
  device.save()
}

const state     = computed(() => String(device.get('lora.0.state') ?? ''))
const chip      = computed(() => String(device.get('lora.0.chip') ?? ''))
const bitrate   = computed(() => Number(device.get('lora.0.bitrate_eff') ?? 0))
const rssi      = computed(() => Number(device.get('lora.0.stats.rssi_last') ?? 0))
const snr       = computed(() => Number(device.get('lora.0.stats.snr_last')  ?? 0))
const rxFrames  = computed(() => Number(device.get('lora.0.stats.rx_frames') ?? 0))
const txFrames  = computed(() => Number(device.get('lora.0.stats.tx_frames') ?? 0))

/* Frequency and bandwidth are stored in Hz (int) but entered in human units —
 * MHz for frequency, kHz for bandwidth — so any value is allowed, not just a
 * fixed set of presets. Frequency has no default (region/antenna — the user
 * must pick); the device rejects an unconfigured/out-of-band radio too.
 *
 * hzToUnit renders the stored Hz as a trimmed decimal in `scale` units;
 * commitHz parses the entry back to Hz and writes it. A non-numeric or
 * out-of-range entry is ignored, so storage keeps its last good value. */
function hzToUnit(raw: unknown, scale: number): string {
  const hz = Number(raw ?? 0)
  if (!(hz > 0)) return ''
  return (hz / scale).toFixed(6).replace(/\.?0+$/, '')
}
function commitHz(k: string, val: string | number | null,
                 scale: number, minHz: number, maxHz: number) {
  const n = parseFloat(String(val ?? ''))
  if (!Number.isFinite(n)) return                 // non-numeric → keep current
  const hz = Math.round(n * scale)
  if (hz < minHz || hz > maxHz) return            // out of range → ignore
  device.set(k, hz)
}

const freqText = computed(() => hzToUnit(device.get('s.lora.0.frequency'), 1e6))
const bwText   = computed(() => hzToUnit(device.get('s.lora.0.bandwidth'), 1e3))
/* Bounds span every chip family this interface drives: ~137 MHz (SX127x low)
 * to 2.5 GHz (SX128x), and 7.8 kHz to 1625 kHz bandwidth. */
const setFreq = (v: string | number | null) => commitHz('s.lora.0.frequency', v, 1e6, 137e6, 2600e6)
const setBw   = (v: string | number | null) => commitHz('s.lora.0.bandwidth', v, 1e3, 5e3, 1700e3)

const sfOptions = [
  { label: 'SF7 (fast, short range)',  value: '7' },
  { label: 'SF8',                       value: '8' },
  { label: 'SF9',                       value: '9' },
  { label: 'SF10',                      value: '10' },
  { label: 'SF11',                      value: '11' },
  { label: 'SF12 (slow, long range)',   value: '12' },
]
const crOptions = [
  { label: '4/5', value: '5' },
  { label: '4/6', value: '6' },
  { label: '4/7', value: '7' },
  { label: '4/8', value: '8' },
]
const modeOptions = [
  { label: 'Full',         value: 'full' },
  { label: 'Gateway',      value: 'gateway' },
  { label: 'Access point', value: 'access_point' },
  { label: 'Roaming',      value: 'roaming' },
  { label: 'Boundary',     value: 'boundary' },
]
</script>

<style scoped>
.section-heading {
  opacity: 0.6;
  font-size: 11px;
  text-transform: uppercase;
  letter-spacing: 0.06em;
  margin-top: 12px;
  margin-bottom: -4px;
}
</style>
