#!/usr/bin/env python3
"""Compare two run directories made on the same machine.

data.csv is compared byte for byte, and meta.json on every field but the
command line, which is what differs between the two runs being compared, and the
paths of the robot, the maze and the target, which differ between two checkouts.

This is how a window, a monitor or a video is proven to change nothing. The
stored baselines are summaries, compared by ``tools/baseline.py``.
"""

import argparse
import json
import sys
from pathlib import Path

#: meta.json fields that may differ between two runs of the same behavior.
IGNORED_FIELDS = ("args", "robot_path", "maze_path", "target_dir")


def locate_difference(run_lines: list[str], reference_lines: list[str]) -> list[str]:
    """Name the first rows and columns that differ, to make a failure actionable."""
    if len(run_lines) != len(reference_lines):
        return [f"  data.csv has {len(run_lines)} lines, the reference has {len(reference_lines)}"]

    header = reference_lines[0].split(",")
    problems = []
    for number, (run_line, reference_line) in enumerate(zip(run_lines, reference_lines), start=1):
        if run_line == reference_line:
            continue
        for index, (got, want) in enumerate(zip(run_line.split(","), reference_line.split(","))):
            if got != want:
                name = header[index] if index < len(header) else f"column {index}"
                problems.append(f"  row {number}, {name}: {got} != reference {want}")
                break
        if len(problems) >= 5:
            problems.append("  ...")
            break
    return problems


def compare_csv(run: bytes, reference: bytes) -> list[str]:
    """Compare two data.csv files byte for byte."""
    if run == reference:
        return []

    return ["data.csv differs from the reference"] + locate_difference(
        run.decode().splitlines(), reference.decode().splitlines()
    )


def compare_meta(run: dict, reference: dict) -> list[str]:
    """Compare two meta.json files on every field that describes the behavior."""
    problems = []
    for field in sorted((run.keys() | reference.keys()) - set(IGNORED_FIELDS)):
        if run.get(field) != reference.get(field):
            problems.append(f"meta.json {field}: {run.get(field)!r} != reference {reference.get(field)!r}")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", type=Path)
    parser.add_argument("reference", type=Path)
    args = parser.parse_args()

    problems = compare_csv((args.run / "data.csv").read_bytes(), (args.reference / "data.csv").read_bytes())
    problems += compare_meta(
        json.loads((args.run / "meta.json").read_text()), json.loads((args.reference / "meta.json").read_text())
    )

    if problems:
        for problem in problems:
            print(f"FAIL {args.run}: {problem}")
        return 1

    print(f"ok   {args.run} matches {args.reference}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
