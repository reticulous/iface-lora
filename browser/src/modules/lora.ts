import { useMenuStore } from 'spangap-browser/stores/menu'
import LoraPanel from '../panels/LoraPanel.vue'

export function registerLora() {
  useMenuStore().register('settings', 'Settings', [
    {
      id: 'reticulum', label: 'Reticulum', type: 'submenu',
      children: [
        {
          id: 'reticulum.transports', label: 'Transports', type: 'submenu',
          children: [
            { id: 'reticulum.transports.lora', label: 'LoRa', type: 'panel',
              component: LoraPanel },
          ],
        },
      ],
    },
  ])
}
