set(LORA_SRCS
    "src/lora.cpp"
    "src/esp_idf_hal.cpp"
)

include(${CMAKE_CURRENT_LIST_DIR}/spangap_requires.cmake)
# Conditional sources: esp-idf/conditional/<name>/ is compiled only when <name>
# is staged. The on-device LoRa Settings pane lives in conditional/spangap-lcd/
# and registers via the when:-gated loraLcdRegister hook — no #if anywhere.
idf_component_register(
    SRCS
        ${LORA_SRCS}
        ${SPANGAP_CONDITIONAL_SRCS}
    INCLUDE_DIRS "include" ${SPANGAP_CONDITIONAL_INCLUDE_DIRS}
    REQUIRES
        ${SPANGAP_REQUIRES}
        driver
        esp_timer
        esp_event
    PRIV_REQUIRES
        radiolib
)

# Pins + per-radio type come from this component's own Kconfig (LORA_* /
# LORAn_*), resolved from the buildable's sdkconfig — no reach into the
# board's main/ header, no function-pointer table.
