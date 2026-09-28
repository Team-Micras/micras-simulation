#!/bin/bash
# run_scenario.sh <simulator> <scenarios dir> <runs dir> <scenario> [simulator option]...
#
# One scenario of <scenarios dir> -> <runs dir>/<scenario>, with any further options passed to the
# simulator. The run starts from the runs directory with a copy of its scenario, so the paths meta.json
# and a baseline summary record are the same on every machine.

set -euo pipefail

if [[ $# -lt 4 ]]; then
    echo "usage: $0 <simulator> <scenarios dir> <runs dir> <scenario> [simulator option]..." >&2
    exit 2
fi

simulator=$1
scenarios=$2
runs=$3
scenario=$4
shift 4

mkdir -p "${runs}"
rm -rf "${runs:?}/${scenario}"
cp "${scenarios}/${scenario}.toml" "${runs}/${scenario}.toml"
cd "${runs}"
"${simulator}" --scenario "${scenario}.toml" --out "${scenario}" "$@"
