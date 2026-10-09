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

# On the host the bus is a chip model and the medium is a UDP socket, so the
# ESP-IDF SPI HAL is replaced rather than configured. The model is sim-mesh's
# virtual SX1262, the shared library libsimradio-sx1262.so: linked by its
# soname and never carried in the image, since sim-mesh provides it when it
# starts the station. The copy linked against is compiled here, from sim-mesh's
# own radio directory and with this build's compiler, so a station built for
# the other architecture links one of its own. That directory is
# SIM_MESH_RADIO_DIR, else sim-mesh's beside this straddle in the workspace.
set(LORA_HOST_PRIV_INCLUDE_DIRS "")
if(IDF_TARGET STREQUAL "linux")
    if(DEFINED ENV{SIM_MESH_RADIO_DIR})
        set(SIMRADIO_DIR "$ENV{SIM_MESH_RADIO_DIR}")
    else()
        get_filename_component(_lora_real "${CMAKE_CURRENT_LIST_FILE}" REALPATH)
        get_filename_component(_lora_dir "${_lora_real}" DIRECTORY)
        get_filename_component(SIMRADIO_DIR "${_lora_dir}/../../sim-mesh/radio" ABSOLUTE)
    endif()
    if(NOT EXISTS "${SIMRADIO_DIR}/include/simradio.h" OR NOT EXISTS "${SIMRADIO_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "iface-lora on the linux target links sim-mesh's virtual SX1262, "
                            "and ${SIMRADIO_DIR} is not sim-mesh's radio directory: clone "
                            "sim-mesh beside this straddle, or name its radio directory in "
                            "SIM_MESH_RADIO_DIR")
    endif()
    list(REMOVE_ITEM LORA_SRCS "src/esp_idf_hal.cpp")
    list(APPEND LORA_SRCS "src/host/virtual_hal.cpp" "src/host/simradio_glue.cpp")
    set(LORA_HOST_PRIV_INCLUDE_DIRS "${SIMRADIO_DIR}/include")
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
    PRIV_INCLUDE_DIRS ${LORA_HOST_PRIV_INCLUDE_DIRS}
    REQUIRES
        ${SPANGAP_REQUIRES}
        driver
        esp_timer
        esp_event
    PRIV_REQUIRES
        radiolib
)

# RadioLib ships a HAL for ESP-IDF of its own and compiles it whatever the
# target. This straddle never uses it — it hands RadioLib the HAL above, the
# chip's bus on a board and the chip model's in a station — so the file leaves
# the build on every target. It would not build on either: on the host it
# includes driver/spi_master.h, which no host target has, and on a board it
# refuses a FreeRTOS tick below 1000 Hz, because its delay() truncates to
# ticks. The tick here stays at 100 Hz for light sleep's sake, and
# EspIdfHal::delay rounds up to whole ticks, so no wait RadioLib asks for is
# lost.
if(TARGET __idf_jgromes__radiolib)
    get_target_property(_lora_rl_srcs __idf_jgromes__radiolib SOURCES)
    set(_lora_rl_keep "")
    foreach(_lora_f IN LISTS _lora_rl_srcs)
        get_filename_component(_lora_n "${_lora_f}" NAME)
        if(_lora_n STREQUAL "EspHal.cpp")
            continue()
        endif()
        list(APPEND _lora_rl_keep "${_lora_f}")
    endforeach()
    set_target_properties(__idf_jgromes__radiolib PROPERTIES SOURCES "${_lora_rl_keep}")
endif()

# The virtual radio, by its soname: the station finds it at run time on the
# LD_LIBRARY_PATH sim-mesh gives it. And the glue that hands it this port's
# services, which only weak references reach, named so it is linked.
if(IDF_TARGET STREQUAL "linux")
    include(ExternalProject)
    set(_lora_simradio_bin "${CMAKE_CURRENT_BINARY_DIR}/simradio")
    set(_lora_simradio_lib "${_lora_simradio_bin}/libsimradio-sx1262.so")
    ExternalProject_Add(lora_simradio
        SOURCE_DIR "${SIMRADIO_DIR}"
        BINARY_DIR "${_lora_simradio_bin}"
        CMAKE_ARGS "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}"
                   "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --target simradio-sx1262
        BUILD_ALWAYS ON
        INSTALL_COMMAND ""
        BUILD_BYPRODUCTS "${_lora_simradio_lib}")
    add_dependencies(${COMPONENT_LIB} lora_simradio)
    target_link_libraries(${COMPONENT_LIB} PUBLIC "${_lora_simradio_lib}")
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u simradio_glue_linked")
endif()

# Pins + per-radio type come from this component's own Kconfig (LORA_* /
# LORAn_*), resolved from the buildable's sdkconfig — no reach into the
# board's main/ header, no function-pointer table.
