#!/bin/bash
# check_generic.sh <source dir>
#
# Fails if anything outside targets/ names a specific robot. hal_host/ is the Micras HAL's host backend
# (its canonical copy is micras-lib's), the documentation describes the Micras target, the files
# micras-lib shares with every repository name the repositories, and this script lists the names, so
# none of them is scanned.

set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 <source dir>" >&2
    exit 2
fi

cd "$1"

identifiers='micras::Micras|MicrasTarget|SimulationContext|ProxyState|proxy_state|robot_v2|"micras"|FSM State|Desired Linear|Desired Angular|Odometry Linear|Odometry Angular|Grid Pose|grid_pose|fsm_state|odometry_state|front wheel|MicrasFirmware|MicrasBoard|micras_firmware|targets/micras|wall_adc|dip_switch|button_config|loop_time_us\{1042|hadc[0-9]|htim[0-9]|hspi[0-9]|huart[0-9]|MX_[A-Z0-9]+_Init|lsm6dsv|as5047'

skipped=(micras_warnings.cmake run_clang_tidy.sh.in "$(basename "$0")")
scanned=()
excluded=()

for path in engine view bridge app tests tools cmake scripts CMakeLists.txt CMakePresets.json .github; do
    if [[ -e "${path}" ]]; then
        scanned+=("${path}")
    fi
done

for file in "${skipped[@]}"; do
    excluded+=(--exclude="${file}")
done

found=0

if grep -rnE -I "${excluded[@]}" --exclude-dir=__pycache__ "${identifiers}" "${scanned[@]}"; then
    found=1
fi

if [[ -f .docker/Dockerfile ]] &&
    sed '/^# >>> host stage >>>$/,/^# <<< host stage <<<$/d' .docker/Dockerfile | grep -nE "${identifiers}"; then
    found=1
fi

if [[ ${found} -ne 0 ]]; then
    echo "robot identifiers outside targets/: move them into the robot's target"
    exit 1
fi

echo "ok   nothing outside targets/ names a robot"
