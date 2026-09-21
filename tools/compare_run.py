#!/usr/bin/env python3
"""Compare a run directory against a baseline run.

data.csv is compared byte for byte, or, with --columns-subset, on every column
the baseline has (new columns may only be appended at the end). meta.json is
compared on the behaviour fields only; provenance such as compiler or arguments
is ignored. Baseline files may be gzip compressed (data.csv.gz).
"""

import argparse
import gzip
import json
import sys
from pathlib import Path

BEHAVIOUR_FIELDS = (
    "model_sha256",
    "loop_time_us",
    "timestep",
    "steps_per_tick",
    "ticks",
    "sim_time",
    "final_z",
    "pool_columns",
    "telemetry_resyncs",
    "warnings_total",
)


def read_bytes(directory: Path, name: str) -> bytes:
    """Read a run artefact, transparently decompressing a .gz baseline."""
    plain = directory / name
    compressed = directory / f"{name}.gz"
    if plain.exists():
        return plain.read_bytes()
    if compressed.exists():
        return gzip.decompress(compressed.read_bytes())
    raise FileNotFoundError(f"{plain} (or .gz) not found")


def compare_csv(run: bytes, baseline: bytes, columns_subset: bool) -> list[str]:
    """Compare two data.csv files, byte for byte or on the baseline columns only."""
    if run == baseline:
        return []

    run_lines = run.decode().splitlines()
    base_lines = baseline.decode().splitlines()

    if not columns_subset:
        return ["data.csv differs from the baseline"] + locate_difference(run_lines, base_lines)
    if len(run_lines) != len(base_lines):
        return [f"data.csv has {len(run_lines)} lines, baseline has {len(base_lines)}"]

    run_header = run_lines[0].split(",")
    base_header = base_lines[0].split(",")
    if run_header[: len(base_header)] != base_header:
        return ["data.csv header does not start with the baseline columns"]

    kept = len(base_header)
    problems = []
    for number, (run_line, base_line) in enumerate(zip(run_lines[1:], base_lines[1:]), start=2):
        if run_line.split(",")[:kept] != base_line.split(","):
            problems.append(f"data.csv row {number} differs on a baseline column")
            if len(problems) >= 5:
                problems.append("...")
                break
    return problems


def locate_difference(run_lines: list[str], base_lines: list[str]) -> list[str]:
    """Name the first rows and columns that differ, to make a failure actionable."""
    if len(run_lines) != len(base_lines):
        return [f"  data.csv has {len(run_lines)} lines, baseline has {len(base_lines)}"]

    header = base_lines[0].split(",")
    problems = []
    for number, (run_line, base_line) in enumerate(zip(run_lines, base_lines), start=1):
        if run_line == base_line:
            continue
        run_cells = run_line.split(",")
        base_cells = base_line.split(",")
        for index, (got, want) in enumerate(zip(run_cells, base_cells)):
            if got != want:
                name = header[index] if index < len(header) else f"column {index}"
                problems.append(f"  row {number}, {name}: {got} != baseline {want}")
                break
        if len(problems) >= 5:
            problems.append("  ...")
            break
    return problems


def compare_meta(run: dict, baseline: dict) -> list[str]:
    """Compare the behaviour fields of two meta.json files."""
    problems = []
    for field in BEHAVIOUR_FIELDS:
        if field not in baseline:
            problems.append(f"meta.json baseline has no field {field}")
        elif run.get(field) != baseline[field]:
            problems.append(f"meta.json {field}: {run.get(field)!r} != baseline {baseline[field]!r}")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", type=Path)
    parser.add_argument("baseline", type=Path)
    parser.add_argument(
        "--columns-subset",
        action="store_true",
        help="accept new columns appended after the baseline ones (extension slices)",
    )
    args = parser.parse_args()

    problems = compare_csv(
        read_bytes(args.run, "data.csv"), read_bytes(args.baseline, "data.csv"), args.columns_subset
    )
    problems += compare_meta(
        json.loads(read_bytes(args.run, "meta.json")), json.loads(read_bytes(args.baseline, "meta.json"))
    )

    if problems:
        for problem in problems:
            print(f"FAIL {args.run}: {problem}")
        return 1

    print(f"ok   {args.run} matches {args.baseline}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
