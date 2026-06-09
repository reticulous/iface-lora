import { useMenuStore } from 'spangap-browser/stores/menu'
import LoraPanel from '../panels/LoraPanel.vue'

export function registerLora() {
  const menu = useMenuStore()
  menu.setMenu('settings/mesh/interfaces', { label: 'RNS Interfaces', placement: 2 })
  menu.register('settings/mesh/interfaces/lora', 'LoRa', { type: 'panel', component: LoraPanel })
}
