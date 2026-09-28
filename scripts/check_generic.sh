#!/bin/bash
# check_generic.sh <source dir>
#
# Fails if anything outside targets/ names a specific robot: every file of the repository is scanned,
# documentation and build files included, except the files micras-lib shares with every repository
# (they name the repositories that carry them) and this script, which lists the names. Of the
# Dockerfile, only the shared host stage is left out. In a git checkout the files are the tracked and
# the untracked but not ignored ones; elsewhere, every file outside build/.

set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 <source dir>" >&2
    exit 2
fi

cd "$1"

identifiers='micras::Micras|MicrasTarget|SimulationContext|ProxyState|proxy_state|robot_v2|"micras"|FSM State|Desired Linear|Desired Angular|Odometry Linear|Odometry Angular|Grid Pose|grid_pose|fsm_state|odometry_state|front wheel|MicrasFirmware|MicrasBoard|micras_firmware|targets/micras|wall_adc|dip_switch|button_config|loop_time_us\{1042|hadc[0-9]|htim[0-9]|hspi[0-9]|huart[0-9]|MX_[A-Z0-9]+_Init|lsm6dsv|as5047'

shared=(
    .clang-format
    .clang-tidy
    tests/.clang-tidy
    cmake/micras_warnings.cmake
    cmake/templates/run_clang_tidy.sh.in
    .docker/Dockerfile
    scripts/check_generic.sh
)

files=()

if git rev-parse --is-inside-work-tree > /dev/null 2>&1; then
    while IFS= read -r -d '' file; do
        files+=("${file}")
    done < <(git ls-files -z --cached --others --exclude-standard)
else
    while IFS= read -r -d '' file; do
        files+=("${file#./}")
    done < <(find . \( -path ./build -o -path ./.git -o -name __pycache__ \) -prune -o -type f -print0)
fi

scanned=()

for file in "${files[@]}"; do
    if [[ "${file}" == targets/* || ! -f "${file}" ]]; then
        continue
    fi

    for skipped in "${shared[@]}"; do
        if [[ "${file}" == "${skipped}" ]]; then
            continue 2
        fi
    done

    scanned+=("${file}")
done

found=0

if [[ ${#scanned[@]} -gt 0 ]] && grep -nE -I "${identifiers}" "${scanned[@]}"; then
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

echo "ok   nothing outside targets/ names a robot (${#scanned[@]} files)"
