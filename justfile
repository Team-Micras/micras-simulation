# Human entry point for the simulator. Linux only.
#
# The engine recipes live here; each robot target's recipes live in its own
# justfile, loaded as a module: `just micras check`, `just micras run-explore`.

set shell := ["bash", "-euo", "pipefail", "-c"]
set positional-arguments := true

mod micras 'targets/micras'

preset := env("MICRAS_PRESET", "default")
build_dir := "build" / preset

# Identifiers that name a specific robot. None may appear outside targets/.
robot_identifiers := "micras::Micras|MicrasTarget|SimulationContext|ProxyState|proxy_state|robot_v2|\"micras\"|FSM State|Desired Linear|Desired Angular|Odometry Linear|Odometry Angular|Grid Pose|grid_pose|fsm_state|odometry_state|front wheel|MicrasFirmware|MicrasBoard|micras_firmware|targets/micras|wall_adc|dip_switch|button_config|loop_time_us\\{1042|hadc[0-9]|htim[0-9]|hspi[0-9]|huart[0-9]|MX_[A-Z0-9]+_Init"

default:
    @just --list --list-submodules

# Configure the chosen CMake preset (default, debug, release).
configure:
    cmake --preset {{preset}}

[private]
configured:
    @test -f {{build_dir}}/CMakeCache.txt || cmake --preset {{preset}}

# Configure if needed, then build the engine and every robot target.
build: configured
    cmake --build --preset {{preset}}

# Every unit test, the engine's and the targets', through CTest.
test: build
    ctest --preset {{preset}}

# Reformat every source in place.
format: configured
    cmake --build {{build_dir}} --target format

# Fail if any source is not clang-format clean.
format-check: configured
    cmake --build {{build_dir}} --target format-check

# clang-tidy over every source.
lint: build
    cmake --build {{build_dir}} --target lint

# Configure every optional subsystem off, so the options stay buildable.
check-options:
    cmake -S . -B {{build_dir}}-minimal -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER=gcc-15 -DCMAKE_CXX_COMPILER=g++-15 \
        -DMICRAS_VIEWER=OFF -DMICRAS_VIDEO=OFF -DMICRAS_BRIDGE=OFF -DMICRAS_TESTS=OFF > /dev/null
    cmake --build {{build_dir}}-minimal > /dev/null

# Fail if anything outside targets/ names a specific robot.
check-generic:
    if grep -rnE '{{robot_identifiers}}' engine view bridge app tests tools cmake CMakeLists.txt; then \
        echo "robot identifiers outside targets/: move them into the robot's target"; \
        exit 1; \
    fi
    echo "ok   nothing outside targets/ names a robot"

# The gate: the engine's checks, then every robot target's.
check: build test check-options check-generic
    just micras check

# Remove the build directory of the chosen preset.
clean:
    rm -rf {{build_dir}}

# Remove every build directory.
clean-all:
    rm -rf build
