# Human entry point for the Micras simulation harness. Linux only.

set shell := ["bash", "-euo", "pipefail", "-c"]
set positional-arguments := true

preset := env("MICRAS_PRESET", "default")
build_dir := "build" / preset
bin := build_dir / "micras_simulation"
python := "python3"

model := "models/robot_v2.xml"
legacy_model := "models/robot_legacy.xml"
pool_columns := "29"
# The current baseline, compared byte for byte; previous versions keep their columns
# checked as a prefix so every run ever recorded stays comparable.
baseline := "baseline/v2"
previous_baselines := "baseline/v1 baseline/v0"
check_runs := "idle explore_v2 explore_legacy"
hardware_tests := "test_battery test_imu test_locomotion test_odometry test_rotary_sensors test_wall_sensors test_calibrate_feed_forward"

default:
    @just --list

# Configure the chosen CMake preset (default, debug, release).
configure:
    cmake --preset {{preset}}

[private]
configured:
    @test -f {{build_dir}}/CMakeCache.txt || cmake --preset {{preset}}

# Configure if needed, then build.
build: configured
    cmake --build --preset {{preset}}

# Unit tests through CTest.
test: build
    ctest --preset {{preset}}

# Run the main firmware program with arbitrary harness flags.
run *args: build
    {{bin}} "$@"

# Build every firmware hardware test executable.
build-tests: configured
    cmake --build {{build_dir}} --target hardware_tests

# Run one firmware hardware test, e.g. `just run-test test_imu --seconds 4 --out runs/imu`.
run-test name *args: build-tests
    {{build_dir}}/hardware_tests/"$1" --model {{model}} "${@:2}"

# 4 s with no command -> runs/idle
run-idle: build
    {{bin}} --model {{model}} --seconds 4 --out runs/idle

# 8 s explore on robot_v2 -> runs/explore_v2
run-explore: build
    {{bin}} --model {{model}} --command explore --seconds 8 --out runs/explore_v2

# 8 s explore on the pre-v2 chassis -> runs/explore_legacy
run-explore-legacy: build
    {{bin}} --model {{legacy_model}} --command explore --seconds 8 --out runs/explore_legacy

# Plots and report.json for one run directory.
analyze run:
    {{python}} tools/analyze.py "$1"

# Run with the bridge open so micras-monitor can connect on ws://localhost:8080.
serve *args: build
    {{bin}} --model {{model}} --command explore --seconds 120 --out runs/serve --monitor "$@"

# Watch a run in a window. Space pauses, right steps, tab cycles cameras, esc quits.
watch *args: build
    {{bin}} --model {{model}} --command explore --seconds 30 --out runs/watch --viewer "$@"

# 100 s explore recorded offscreen, then analysed.
video: build
    mkdir -p runs/video
    {{bin}} --model {{model}} --command explore --seconds 100 --out runs/explore_v2 \
        --video runs/video/explore_v2.mp4 --video-fps 30 --video-camera "side tracking" --video-size 1280x720
    {{python}} tools/analyze.py runs/explore_v2

# Config drift against the firmware submodule.
drift:
    {{python}} tools/check_config_drift.py

# Reformat the harness sources in place.
format: configured
    cmake --build {{build_dir}} --target format

# Fail if any harness source is not clang-format clean.
format-check: configured
    cmake --build {{build_dir}} --target format-check

# clang-tidy over the harness sources.
lint: build
    cmake --build {{build_dir}} --target lint

# Compare the check scenarios against every recorded baseline.
compare-baseline:
    for run in {{check_runs}}; do \
        {{python}} tools/compare_run.py runs/$run {{baseline}}/$run; \
        for previous in {{previous_baselines}}; do \
            {{python}} tools/compare_run.py runs/$run $previous/$run --columns-subset; \
        done; \
    done

# Record a new baseline version. Refuses to overwrite one, so an older version
# is never lost and the prefix checks it anchors keep working.
record-baseline: test drift run-idle run-explore run-explore-legacy
    for run in {{check_runs}}; do \
        test ! -e {{baseline}}/$run/data.csv.gz \
            || { echo "{{baseline}}/$run already exists; bump the baseline version instead"; exit 1; }; \
    done
    for run in {{check_runs}}; do \
        mkdir -p {{baseline}}/$run; \
        gzip -9 -c runs/$run/data.csv > {{baseline}}/$run/data.csv.gz; \
        cp runs/$run/meta.json {{baseline}}/$run/meta.json; \
    done
    just compare-baseline

# Prove the monitor bridge talks and changes nothing: a client asks for the variable
# map over the socket, and the bridged run must match the same run without it.
check-monitor: build
    {{bin}} --model {{model}} --command explore --seconds 6 --out runs/monitor/plain > /dev/null
    {{python}} tools/monitor_probe.py --listen-seconds 4 --expect-packets 50 --request-map & \
        probe=$!; \
        {{bin}} --model {{model}} --command explore --seconds 60 --out runs/monitor/bridged --monitor > /dev/null; \
        wait $probe
    {{bin}} --model {{model}} --command explore --seconds 6 --out runs/monitor/quiet --monitor > /dev/null
    {{python}} tools/compare_run.py runs/monitor/quiet runs/monitor/plain

# Prove a window changes nothing: the same scenario headless and watched must agree.
# Skipped with a notice when there is no display, so a headless machine still passes.
check-viewer: build
    if command -v xvfb-run > /dev/null; then \
        show="xvfb-run -a"; \
    elif [ -n "${DISPLAY:-}" ]; then \
        show=""; \
    else \
        echo "no display and no xvfb-run, skipping the viewer comparison"; \
        exit 0; \
    fi; \
    {{bin}} --model {{model}} --command explore --seconds 3 --button short --out runs/viewer/headless > /dev/null; \
    $show {{bin}} --model {{model}} --command explore --seconds 3 --button short --out runs/viewer/watched \
        --viewer --viewer-size 640x480 > /dev/null; \
    {{python}} tools/compare_run.py runs/viewer/watched runs/viewer/headless

# Run every hardware test briefly, to catch a program that stops handing ticks back.
smoke-tests: build-tests
    for name in {{hardware_tests}}; do \
        echo "== $name"; \
        {{build_dir}}/hardware_tests/$name --model {{model}} --seconds 1 --out runs/smoke/$name > /dev/null; \
    done
    {{build_dir}}/hardware_tests/test_fan --model {{model}} --seconds 1 --button short --out runs/smoke/test_fan \
        > /dev/null

# The gate: build, tests, drift, scenarios, analysis, assertions, baseline.
check: build test drift check-options smoke-tests check-viewer check-monitor run-idle run-explore run-explore-legacy
    for run in {{check_runs}}; do {{python}} tools/analyze.py runs/$run; done
    {{python}} tools/check_run.py --pool-columns {{pool_columns}} runs/idle runs/explore_v2 runs/explore_legacy
    just compare-baseline

# Configure every optional subsystem off, so the options stay buildable.
check-options:
    cmake -S . -B {{build_dir}}-minimal -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DMICRAS_VIEWER=OFF -DMICRAS_VIDEO=OFF -DMICRAS_BRIDGE=OFF -DMICRAS_TESTS=OFF > /dev/null
    cmake --build {{build_dir}}-minimal > /dev/null

# Remove the build directory of the chosen preset.
clean:
    rm -rf {{build_dir}}

# Remove every build directory.
clean-all:
    rm -rf build
