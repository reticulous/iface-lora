set(LORA_SRCS
    "src/lora.cpp"
    "src/lora_queue.cpp"
    "src/lora_radio.cpp"
    "src/lora_csma.cpp"
    "src/lora_peers.cpp"
    "src/lora_observe.cpp"
    "src/lora_power.cpp"
    "src/lora_fem.cpp"
    "src/lora_rfcal.cpp"
    "src/lora_mon.cpp"
    "src/lora_bridge.cpp"
    "src/lora_rnode.cpp"
    "src/lora_cli.cpp"
    "src/esp_idf_hal.cpp"
    "src/rolling.cpp"
)

# On the host the bus is a chip model in the same process and the medium is a
# UDP socket, so the ESP-IDF SPI HAL is replaced rather than configured.
if(IDF_TARGET STREQUAL "linux")
    list(REMOVE_ITEM LORA_SRCS "src/esp_idf_hal.cpp")
    list(APPEND LORA_SRCS
        "src/host/virtual_hal.cpp"
        "src/host/virtual_sx126x.cpp"
        "src/host/ether_task.cpp")
endif()

# SUPE, whole. CONFIG_LORA_NO_SUPE leaves it out of the image; every call site
# in the sources above is gated on the same symbol, so nothing here is referenced
# in that build. supe.cpp is SUPE's pure core, deliberately free of ESP-IDF,
# RadioLib and FreeRTOS so that test/ can compile it with a plain g++ — see
# test/Makefile.
#
# lora_chanplan.cpp is the regime channel tables and lora_airtime.cpp the regime
# airtime ledger: both exist to serve the schedule SUPE negotiates, and nothing
# outside it asks them anything.
if(NOT CONFIG_LORA_NO_SUPE)
    list(APPEND LORA_SRCS
        "src/supe.cpp"
        "src/supe_engine.cpp"
        "src/lora_supe.cpp"
        "src/lora_chanplan.cpp"
        "src/lora_airtime.cpp"
    )
endif()

include(${CMAKE_CURRENT_LIST_DIR}/spangap_requires.cmake)
# Conditional sources: esp-idf/conditional/<name>/ is compiled only when <name>
# is staged. The mono-OLED radio page lives in conditional/tinylcd/ and
# registers via the when:-gated LoraTinyPage service entry — no #if needed.
#
# The LoRaMon viewers are their own straddle (reticulous/loramon). What stays
# here is the recorder, gated on CONFIG_STRADDLE_LORAMON inside lora_mon.cpp
# rather than by a slice dir, because the radio task calls into it and a source
# file that simply vanished would leave those calls unresolved.
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
