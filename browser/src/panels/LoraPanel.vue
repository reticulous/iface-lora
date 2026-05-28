<template>
  <div class="q-gutter-y-md">
    <PanelHeading title="LoRa" />

    <div class="text-caption" style="opacity:0.7">
      SX1262 LoRa transport. Pick a band, bandwidth, spreading factor, and
      coding rate; the radio stays disabled until you enable it.
    </div>

    <SettingToggle label="Enabled" k="s.lora.enable" />

    <div class="section-heading">Radio</div>

    <SettingSelect label="Frequency" k="s.lora.frequency" :options="freqOptions" />
    <SettingSelect label="Bandwidth" k="s.lora.bandwidth" :options="bwOptions" />
    <SettingSelect label="Spreading factor" k="s.lora.spreading_factor" :options="sfOptions" />
    <SettingSelect label="Coding rate" k="s.lora.coding_rate" :options="crOptions" />
    <SettingSlider label="TX power (dBm)" k="s.lora.tx_power" :min="-9" :max="22" />
    <SettingSlider label="Preamble (sym)"  k="s.lora.preamble"  :min="6"  :max="32" />
    <SettingText   label="Sync word"       k="s.lora.sync_word" />

    <div class="section-heading">RNS interface</div>

    <SettingSelect label="Mode" k="s.lora.mode" :options="modeOptions" />

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
import { computed } from 'vue'
import { useDeviceStore } from 'spangap-browser/stores/device'

const device = useDeviceStore()

const state     = computed(() => String(device.get('lora.state') ?? ''))
const chip      = computed(() => String(device.get('lora.chip') ?? ''))
const bitrate   = computed(() => Number(device.get('lora.bitrate_eff') ?? 0))
const rssi      = computed(() => Number(device.get('lora.stats.rssi_last') ?? 0))
const snr       = computed(() => Number(device.get('lora.stats.snr_last')  ?? 0))
const rxFrames  = computed(() => Number(device.get('lora.stats.rx_frames') ?? 0))
const txFrames  = computed(() => Number(device.get('lora.stats.tx_frames') ?? 0))

/* No preselected defaults (plan §12.4). All values stored as ints
 * (frequency in Hz, bandwidth in Hz) so storage stays type-clean. */

/* Storage values are stored as strings here (matching spangap's
 * SettingSelect type); storageGetInt on the device side atoi's them. */
const freqOptions = [
  { label: '433.000 MHz (ISM EU/AS)', value: '433000000' },
  { label: '868.100 MHz (EU868)',     value: '868100000' },
  { label: '869.525 MHz (EU868 RNS)', value: '869525000' },
  { label: '915.000 MHz (US915)',     value: '915000000' },
  { label: '923.000 MHz (AS923)',     value: '923000000' },
]
const bwOptions = [
  { label: '125 kHz', value: '125000' },
  { label: '250 kHz', value: '250000' },
  { label: '500 kHz', value: '500000' },
]
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
