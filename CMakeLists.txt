cmake_minimum_required(VERSION 3.25)
project(micras_simulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(MICRAS_VIDEO "Build the offscreen EGL video recorder" ON)
option(MICRAS_VIEWER "Build the live GLFW viewer" ON)
option(MICRAS_BRIDGE "Build the WebSocket bridge to micras-monitor" ON)
option(MICRAS_TESTS "Build the unit tests" ON)

include(cmake/mujoco.cmake)
include(cmake/third_party.cmake)

###############################################################################
## Simulation libraries
###############################################################################

add_library(micras_sim_core STATIC
    src/sim/core/clock.cpp
    src/sim/core/firmware_thread.cpp
    src/sim/core/simulation.cpp
    src/sim/core/mujoco_world.cpp
    src/sim/core/serial_bus.cpp
    src/sim/core/simulation_context.cpp
)
target_include_directories(micras_sim_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(micras_sim_core PUBLIC mujoco::mujoco)

include(cmake/firmware.cmake)

add_library(micras_sim STATIC
    src/sim/core/scenario.cpp
    src/sim/telemetry/packet_framer.cpp
    src/sim/telemetry/telemetry.cpp
    src/sim/recording/csv_writer.cpp
    src/sim/recording/ground_truth.cpp
    src/sim/recording/csv_recorder.cpp
    src/sim/recording/run_metadata.cpp
)
target_link_libraries(micras_sim PUBLIC micras_sim_core micras_firmware_shadow PRIVATE picosha2)

add_library(micras_sim_view STATIC
    src/sim/view/control_panel.cpp
    src/sim/view/mujoco_viewer.cpp
    src/sim/view/video_recorder.cpp
)
target_link_libraries(micras_sim_view PUBLIC micras_sim_core)

if(MICRAS_VIDEO)
    set(OpenGL_GL_PREFERENCE GLVND)
    find_package(OpenGL COMPONENTS OpenGL EGL)

    if(OpenGL_FOUND AND OpenGL_EGL_FOUND)
        target_link_libraries(micras_sim_view PRIVATE OpenGL::OpenGL OpenGL::EGL)
    else()
        target_link_libraries(micras_sim_view PRIVATE EGL GL)
    endif()

    target_compile_definitions(micras_sim_view PRIVATE MICRAS_VIDEO)
endif()

if(MICRAS_VIEWER)
    find_package(glfw3 QUIET)

    if(glfw3_FOUND)
        target_link_libraries(micras_sim_view PRIVATE glfw)
    else()
        find_library(GLFW_LIBRARY NAMES glfw glfw3 REQUIRED)
        target_link_libraries(micras_sim_view PRIVATE "${GLFW_LIBRARY}")
    endif()

    target_link_libraries(micras_sim_view PRIVATE imgui)
    target_compile_definitions(micras_sim_view PRIVATE MICRAS_VIEWER)
endif()

add_library(micras_sim_bridge STATIC
    src/sim/bridge/monitor_bridge.cpp
    src/sim/bridge/web_socket_server.cpp
)
target_link_libraries(micras_sim_bridge PUBLIC micras_sim)

if(MICRAS_BRIDGE)
    target_link_libraries(micras_sim_bridge PRIVATE ixwebsocket)
    target_compile_definitions(micras_sim_bridge PRIVATE MICRAS_BRIDGE)
endif()

###############################################################################
## Executable
###############################################################################

add_executable(micras_simulation src/main.cpp)
target_link_libraries(micras_simulation PRIVATE micras_sim_app)

# The application is the only layer that knows a run can have a window, so it is
# the only one that links the view.
add_library(micras_sim_app STATIC
    src/sim/app/application.cpp
    src/sim/app/cli.cpp
    src/sim/app/crash_reporter.cpp
)
target_link_libraries(micras_sim_app PUBLIC micras_sim micras_sim_view micras_sim_bridge)

target_compile_definitions(micras_sim_app PRIVATE
    MICRAS_FIRMWARE_SHA="${MICRAS_FIRMWARE_SHA}"
    MICRAS_COMPILER_ID="${CMAKE_CXX_COMPILER_ID}"
    MICRAS_COMPILER_VERSION="${CMAKE_CXX_COMPILER_VERSION}"
    MICRAS_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)

set_target_properties(micras_simulation PROPERTIES
    BUILD_RPATH "${MUJOCO_DIR}/lib"
    INSTALL_RPATH "${MUJOCO_DIR}/lib"
)

###############################################################################
## Firmware hardware tests
###############################################################################

include(cmake/hardware_tests.cmake)

micras_add_hardware_tests(
    tests/src/proxy/test_battery.cpp
    tests/src/proxy/test_fan.cpp
    tests/src/proxy/test_imu.cpp
    tests/src/proxy/test_locomotion.cpp
    tests/src/proxy/test_rotary_sensors.cpp
    tests/src/proxy/test_wall_sensors.cpp
    tests/src/nav/test_odometry.cpp
    tests/src/nav/test_calibrate_feed_forward.cpp
)

###############################################################################
## Tests
###############################################################################

if(MICRAS_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

###############################################################################
## Format and lint
###############################################################################

file(GLOB_RECURSE HARNESS_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp"
)
file(GLOB_RECURSE HARNESS_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/config/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.hpp"
)

foreach(TARGET micras_sim_core micras_sim micras_sim_app micras_sim_view micras_sim_bridge micras_simulation)
    target_compile_options(${TARGET} PRIVATE -Wall -Wextra)
endforeach()

include(cmake/lint.cmake)
micras_add_lint_targets(${HARNESS_SOURCES} ${HARNESS_HEADERS})
