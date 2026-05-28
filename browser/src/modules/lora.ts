import { useMenuStore } from 'spangap-browser/stores/menu'
import LoraPanel from '../panels/LoraPanel.vue'

export function registerLora() {
  useMenuStore().register('settings', 'Settings', 10, [
    {
      id: 'reticulum', label: 'Reticulum', type: 'submenu', order: 30,
      children: [
        {
          id: 'reticulum.transports', label: 'Transports', type: 'submenu', order: 20,
          children: [
            { id: 'reticulum.transports.lora', label: 'LoRa', type: 'panel', order: 30,
              component: LoraPanel },
          ],
        },
      ],
    },
  ])
}
