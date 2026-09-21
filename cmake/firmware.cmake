###############################################################################
## Firmware shadow: the firmware compiled against the simulation proxies
###############################################################################

set(MICRAS_FIRMWARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/MicrasFirmware")

execute_process(
    COMMAND git -C "${MICRAS_FIRMWARE_DIR}" rev-parse HEAD
    OUTPUT_VARIABLE MICRAS_FIRMWARE_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

if(NOT MICRAS_FIRMWARE_SHA)
    set(MICRAS_FIRMWARE_SHA "unknown")
endif()

# main.cpp is excluded because it pulls in the STM32 HAL; micras_hal and
# micras_proxy are replaced by include/micras/proxy + src/proxy.
file(GLOB FIRMWARE_SOURCES CONFIGURE_DEPENDS
    "${MICRAS_FIRMWARE_DIR}/micras_core/src/*.cpp"
    "${MICRAS_FIRMWARE_DIR}/micras_nav/src/*.cpp"
    "${MICRAS_FIRMWARE_DIR}/micras_comm/src/*.cpp"
)
list(APPEND FIRMWARE_SOURCES
    "${MICRAS_FIRMWARE_DIR}/src/micras.cpp"
    "${MICRAS_FIRMWARE_DIR}/src/interface.cpp"
)

file(GLOB PROXY_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/proxy/*.cpp")

add_library(micras_firmware_shadow STATIC ${FIRMWARE_SOURCES} ${PROXY_SOURCES})

# Order matters: config/ and include/ must shadow the firmware headers.
target_include_directories(micras_firmware_shadow PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/config"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${MICRAS_FIRMWARE_DIR}/include"
    "${MICRAS_FIRMWARE_DIR}/micras_nav/include"
    "${MICRAS_FIRMWARE_DIR}/micras_comm/include"
    "${MICRAS_FIRMWARE_DIR}/micras_core/include"
)

target_link_libraries(micras_firmware_shadow PUBLIC micras_sim_core)

# -Wall/-Wextra only on our own code; the firmware is pinned and out of scope.
set_source_files_properties(${PROXY_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
