import { useMenuStore } from 'spangap-browser/stores/menu'
import LoraPanel from '../panels/LoraPanel.vue'

export function registerLora() {
  useMenuStore().register('settings/reticulum/transports/lora', 'LoRa', { type: 'panel', component: LoraPanel })
}
