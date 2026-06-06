/**
 * lora_lcd.cpp — on-device LoRa Settings pane (LVGL).
 *
 * Settings → Reticulum → Transports → LoRa. Targets radio 0 (the common
 * single-radio case); a per-radio selector is a follow-up. Mirrors the
 * web LoraPanel — selects store the raw wire values.
 *
 * This whole file lives under conditional/spangap-lcd/, compiled only when the
 * lcd straddle is staged, so no #if is needed. It registers via the when:-gated
 * loraLcdRegister init hook (spangap/spangap-lcd), in plain C++ linkage to match
 * the generated dispatcher's forward decl.
 */
#include "lcd.h"

namespace {

static void loraSettingsPane(void* arg) {
    lv_obj_t* p = static_cast<lv_obj_t*>(arg);
    lcdSettingSection (p, "LoRa");
    lcdSettingSwitch  (p, "Enable", "s.lora.0.enable");
    lcdSettingSection (p, "Radio");
    /* Sub-GHz presets first, then the 2.4 GHz set (SX128x) and SX128x
     * bandwidths — pick the ones valid for the slot's chip. */
    lcdSettingDropdown(p, "Frequency", "s.lora.0.frequency",
                       "433000000,868100000,869525000,915000000,923000000,"
                       "2400000000,2450000000,2480000000");
    lcdSettingDropdown(p, "Bandwidth", "s.lora.0.bandwidth",
                       "125000,250000,500000,203125,406250,812500,1625000");
    lcdSettingDropdown(p, "Spreading", "s.lora.0.spreading_factor", "7,8,9,10,11,12");
    lcdSettingDropdown(p, "Coding rate", "s.lora.0.coding_rate", "5,6,7,8");
    lcdSettingSlider  (p, "TX power", "s.lora.0.tx_power", -9, 22);
    lcdSettingSlider  (p, "Preamble", "s.lora.0.preamble", 6, 32);
    lcdSettingText    (p, "Sync word", "s.lora.0.sync_word");
    lcdSettingDropdown(p, "Mode", "s.lora.0.mode",
                       "full,gateway,access_point,roaming,boundary");
    lcdSettingSection (p, "Status");
    lcdSettingValue   (p, "State", "lora.0.state");
    lcdSettingValue   (p, "Chip", "lora.0.chip");
    lcdSettingValue   (p, "Bitrate", "lora.0.bitrate_eff");
    lcdSettingValue   (p, "RSSI", "lora.0.stats.rssi_last");
}

}  // namespace

/* Register the on-device LoRa Settings pane — a when:-gated init: hook
 * (spangap/spangap-lcd). Plain C++ linkage to match the generated dispatcher's
 * forward decl. */
void loraLcdRegister(void) {
    lcdRegisterSettings("Reticulum/Transports/LoRa", "LoRa", loraSettingsPane);
}
