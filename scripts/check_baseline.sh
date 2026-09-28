#!/bin/bash
# check_baseline.sh compare|record <binary> <scenarios dir> <baseline dir> <tools dir> <out dir> <exact> <scenario>...
#
# Runs each named scenario and compares its summary with <baseline dir>/<scenario> (tools/baseline.py),
# byte for byte when <exact> is ON, then checks the health of every run (tools/check_run.py). record
# writes the summaries instead, and refuses to overwrite a version.

set -euo pipefail

if [[ $# -lt 8 ]]; then
    echo "usage: $0 compare|record <binary> <scenarios dir> <baseline dir> <tools dir> <out dir> <exact> <scenario>..." >&2
    exit 2
fi

action=$1
binary=$2
scenarios=$3
baseline=$4
tools=$5
out=$6
exact=$7
shift 7

exact_flag=()

if [[ "${exact}" == ON ]]; then
    exact_flag=(--exact)
fi

runs=()
mkdir -p "${out}"

# Each run starts from the output directory with a copy of its scenario, so the paths a summary records
# are the same on every machine.
for name in "$@"; do
    rm -rf "${out:?}/${name}"
    cp "${scenarios}/${name}.toml" "${out}/${name}.toml"
    (cd "${out}" && "${binary}" --scenario "${name}.toml" --out "${name}" > "${name}.log")
    runs+=("${out}/${name}")

    if [[ "${action}" == record ]]; then
        python3 "${tools}/baseline.py" record "${out}/${name}" "${baseline}/${name}"
    else
        python3 "${tools}/baseline.py" compare "${out}/${name}" "${baseline}/${name}" "${exact_flag[@]}"
    fi
done

python3 "${tools}/check_run.py" "${runs[@]}"
