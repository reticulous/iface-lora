set(LORA_SRCS
    "src/lora.cpp"
    "src/esp_idf_hal.cpp"
)

include(${CMAKE_CURRENT_LIST_DIR}/spangap_requires.cmake)
idf_component_register(
    SRCS ${LORA_SRCS}
    INCLUDE_DIRS "include"
    REQUIRES
        ${SPANGAP_REQUIRES}
        driver
        esp_timer
        esp_event
    PRIV_REQUIRES
        radiolib
)

# Board pin-map bridge: lora.cpp uses BOARD_LORA_* macros (CS, DIO1, RST,
# BUSY, TCXO voltage, etc.) defined in the consumer's main/tdeck.h (or
# the equivalent board header). Adding main/ to the PRIVATE include path
# lets `#include "tdeck.h"` resolve directly, no Kconfig plumbing.
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_SOURCE_DIR}/main")
