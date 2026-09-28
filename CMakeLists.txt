cmake_minimum_required(VERSION 3.25)
project(micras_simulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
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
## Engine: knows no robot and no HAL
###############################################################################

# The engine targets carry only their own include directories, so an engine source
# that includes a firmware or robot target header does not compile.

add_library(micras_sim_engine STATIC
    engine/arenas/maze/src/maze.cpp
    engine/src/core/clock.cpp
    engine/src/core/firmware_thread.cpp
    engine/src/core/mujoco_world.cpp
    engine/src/core/noise.cpp
    engine/src/core/serial_bus.cpp
    engine/src/core/simulation.cpp
    engine/src/devices/dc_motor.cpp
    engine/src/devices/digital_input.cpp
    engine/src/devices/device.cpp
    engine/src/devices/imu.cpp
    engine/src/devices/power.cpp
    engine/src/devices/quadrature_encoder.cpp
    engine/src/devices/serial_link.cpp
    engine/src/devices/wall_sensors.cpp
    engine/src/recording/csv_recorder.cpp
    engine/src/recording/csv_writer.cpp
    engine/src/recording/event_log.cpp
    engine/src/recording/ground_truth.cpp
    engine/src/recording/run_metadata.cpp
    engine/src/robot/robot_description.cpp
    engine/src/robot/robot_model.cpp
    engine/src/scenario/scenario.cpp
)
add_library(micras::sim_engine ALIAS micras_sim_engine)
target_include_directories(micras_sim_engine PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/engine/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/engine/arenas/maze/include"
)
target_link_libraries(micras_sim_engine PUBLIC mujoco::mujoco PRIVATE picosha2 tomlplusplus::tomlplusplus)

add_library(micras_sim_view STATIC
    view/src/control_panel.cpp
    view/src/mujoco_viewer.cpp
    view/src/panel_spec.cpp
    view/src/video_recorder.cpp
)
add_library(micras::sim_view ALIAS micras_sim_view)
target_include_directories(micras_sim_view PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/view/include")
target_link_libraries(micras_sim_view PUBLIC micras_sim_engine)

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
    bridge/src/monitor_bridge.cpp
    bridge/src/web_socket_server.cpp
)
add_library(micras::sim_bridge ALIAS micras_sim_bridge)
target_include_directories(micras_sim_bridge PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/bridge/include")
target_link_libraries(micras_sim_bridge PUBLIC micras_sim_engine)

if(MICRAS_BRIDGE)
    target_link_libraries(micras_sim_bridge PRIVATE ixwebsocket)
    target_compile_definitions(micras_sim_bridge PRIVATE MICRAS_BRIDGE)
endif()

# The application is the only layer that knows a run can have a window, so it is
# the only one that links the view. It has no main: each robot target calls
# micras::sim::run() from its own.
add_library(micras_sim_app STATIC
    app/src/application.cpp
    app/src/cli.cpp
    app/src/crash_reporter.cpp
)
add_library(micras::sim_app ALIAS micras_sim_app)
target_include_directories(micras_sim_app PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/app/include")
target_link_libraries(micras_sim_app PUBLIC micras_sim_engine micras_sim_view micras_sim_bridge)
target_compile_definitions(micras_sim_app PRIVATE
    MICRAS_SIM_MAZES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/engine/arenas/maze/mazes"
    MICRAS_COMPILER_ID="${CMAKE_CXX_COMPILER_ID}"
    MICRAS_COMPILER_VERSION="${CMAKE_CXX_COMPILER_VERSION}"
    MICRAS_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)

foreach(TARGET micras_sim_engine micras_sim_view micras_sim_bridge micras_sim_app)
    target_compile_options(${TARGET} PRIVATE -Wall -Wextra)
endforeach()

###############################################################################
## Host micras_hal: compiled by each robot target against its own HAL headers
###############################################################################

# Not a library of its own: the micras_hal headers it implements, and the fake Cube
# layer they include, belong to the robot being built. It includes nothing from the
# engine, so it can move next to micras_hal unchanged.
set(MICRAS_SIM_HAL_HOST_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/hal_host/include")
file(GLOB MICRAS_SIM_HAL_HOST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/hal_host/src/*.cpp")

###############################################################################
## Tests, chip models and robot targets
###############################################################################

if(MICRAS_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# The SPI chips a robot's bindings attach to the host HAL's buses: micras::hal_host_models.
add_subdirectory(hal_host/models)

# Every robot target is a folder of targets/ with its own CMakeLists.txt.
file(GLOB MICRAS_SIM_TARGET_DIRS LIST_DIRECTORIES true "${CMAKE_CURRENT_SOURCE_DIR}/targets/*")

foreach(target_dir IN LISTS MICRAS_SIM_TARGET_DIRS)
    if(EXISTS "${target_dir}/CMakeLists.txt")
        add_subdirectory("${target_dir}")
    endif()
endforeach()

###############################################################################
## Format and lint
###############################################################################

file(GLOB_RECURSE HARNESS_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/engine/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/hal_host/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/view/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/bridge/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/app/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/targets/*/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/targets/*/proxy/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/targets/*/cube/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/targets/*/tests/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/targets/*/tools/*.cpp"
)
file(GLOB_RECURSE HARNESS_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/engine/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/hal_host/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/view/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/bridge/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/app/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/targets/*/include/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/targets/*/tests/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/hal_host/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/targets/*/cube/*.h"
)

include(cmake/lint.cmake)
micras_add_lint_targets(${HARNESS_SOURCES} ${HARNESS_HEADERS})
