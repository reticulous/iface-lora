import { ref } from 'vue'
import { useMenuStore } from 'spangap-browser/stores/menu'
import { registerApp } from 'spangap-browser/lib/apps'
import { registerWindowMount } from 'spangap-browser/lib/windowMounts'
import LoraPanel from '../panels/LoraPanel.vue'
import LoraMonWindow from '../panels/LoraMonWindow.vue'

/* FloatingWindow restores its own saved visibility on mount and emits it back. */
export const loraMonVisible = ref(false)
export const loraMonFocus = ref(0)
export function showLoraMon() {
  loraMonVisible.value = true
  loraMonFocus.value++
}

export function registerLora() {
  const menu = useMenuStore()
  menu.setMenu('settings/mesh/interfaces', { label: 'RNS Interfaces', placement: 2 })
  menu.register('settings/mesh/interfaces/lora', 'LoRa', { type: 'panel', component: LoraPanel })

  /* Dock app: LoRaMon — per-on-air-frame airtime/signal monitor (like actmon,
   * but per-packet). Self-mounts its window so no buildable MainLayout edit. */
  registerApp({ id: 'loramon', label: 'LoRaMon', icon: 'loramon', placement: 6,
                open: showLoraMon, isOpen: () => loraMonVisible.value })
  registerWindowMount({ id: 'loramon', title: 'LoRaMon', component: LoraMonWindow,
                        visible: loraMonVisible, focusToken: loraMonFocus })
}
