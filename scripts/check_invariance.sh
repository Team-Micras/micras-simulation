#!/bin/bash
# check_invariance.sh <binary> <scenario> <tools dir> <out dir>
#
# Proves that nothing but the physics decides a run: the same scenario run twice, then with a window,
# with a video and with a bridge nobody connects to, each data.csv and meta.json the same as the plain
# run's (tools/compare_run.py). The window needs xvfb-run or a display; MICRAS_SKIP_VIEWER=1 skips it
# even where one exists.
# The video needs EGL, a GPU or Mesa's surfaceless platform.

set -euo pipefail

if [[ $# -ne 4 ]]; then
    echo "usage: $0 <binary> <scenario> <tools dir> <out dir>" >&2
    exit 2
fi

binary=$1
scenario=$2
tools=$3
out=$4

rm -rf "${out}"
mkdir -p "${out}"

plain_run() {
    local name=$1
    shift
    "${binary}" --scenario "${scenario}" --out "${out}/${name}" "$@" > "${out}/${name}.log"
}

compare() {
    python3 "${tools}/compare_run.py" "${out}/$1" "${out}/plain"
}

# The window and the video load the system's GL and font libraries, which keep what they cache until
# the process exits; a sanitized build still checks for leaks in every other run.
graphics_options="${ASAN_OPTIONS:+${ASAN_OPTIONS}:}detect_leaks=0"

plain_run plain
plain_run again
compare again

if [[ "${MICRAS_SKIP_VIEWER:-0}" == 1 ]]; then
    echo "MICRAS_SKIP_VIEWER=1: skipping the viewer comparison"
elif command -v xvfb-run > /dev/null; then
    ASAN_OPTIONS="${graphics_options}" xvfb-run -a "${binary}" --scenario "${scenario}" --out "${out}/viewer" --viewer --viewer-size 640x480 \
        > "${out}/viewer.log"
    compare viewer
elif [[ -n "${DISPLAY:-}" ]]; then
    ASAN_OPTIONS="${graphics_options}" "${binary}" --scenario "${scenario}" --out "${out}/viewer" --viewer --viewer-size 640x480 > "${out}/viewer.log"
    compare viewer
else
    echo "no display and no xvfb-run; install xvfb or set MICRAS_SKIP_VIEWER=1" >&2
    exit 1
fi

mkdir -p "${out}/video"
ASAN_OPTIONS="${graphics_options}" plain_run video --video "${out}/video/run.mp4" --video-size 640x480
grep -q "^recording " "${out}/video.log"
compare video

port=$(python3 -c 'import socket; s = socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1])')
plain_run monitor --monitor --monitor-port "${port}"
grep -q "monitor bridge listening" "${out}/monitor.log"
compare monitor
