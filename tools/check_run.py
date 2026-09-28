#!/usr/bin/env python3
"""Assert the health invariants of one or more finished run directories.

Reads ``meta.json`` and ``data.csv`` and fails the build when a run is not clean:

* ``warnings_total`` is 0 (no MuJoCo solver or constraint warnings),
* no chassis geom collided with anything,
* no non-finite samples (the ``*_penetration`` NaN sentinel is excluded by
  ``analyze.py`` itself, every other NaN is a defect),
* every tick that was asked for actually ran, unless the scenario's stop
  condition ended the run,
* nobody reached into the run through a window or a monitor,
* each ``--expect key=value`` holds for the robot target's own ``meta.json``
  fields, for example ``unbound_ports=0``,
* with ``--expect-state NAME``, the last state the run logged is NAME.

Exit code 0 when every run passes, 1 otherwise.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import analyze  # noqa: E402


def check(run: Path, expected: dict[str, int], state: str | None = None) -> list[str]:
    """Return the list of failures for one run directory."""
    failures: list[str] = []

    for path in (run / "meta.json", run / "data.csv"):
        if not path.exists():
            failures.append(f"{path} is missing")

    if failures:
        return failures

    meta = json.loads((run / "meta.json").read_text())

    if meta.get("warnings_total") != 0:
        failures.append(f"warnings_total is {meta.get('warnings_total')}, expected 0")

    collisions = [event for event in meta.get("events", []) if event.get("kind") == "collision"]

    if collisions:
        failures.append(
            f"{len(collisions)} collision(s), the first of {collisions[0]['detail']} at {collisions[0]['time']} s"
        )

    for key, value in expected.items():
        if meta.get(key) != value:
            failures.append(f"{key} is {meta.get(key)!r}, expected {value}")

    if meta.get("interactive") is not False:
        failures.append(f"interactive is {meta.get('interactive')!r}; an interactive run is not reproducible")

    stopped = meta.get("stopped_at", -1) is not None and meta.get("stopped_at", -1) >= 0

    if meta.get("ticks") != meta.get("requested_ticks") and not stopped:
        failures.append(f"only {meta.get('ticks')} of {meta.get('requested_ticks')} ticks ran")

    states = [event["detail"] for event in meta.get("events", []) if event.get("kind") == "state"]

    if state is not None and (not states or states[-1] != state):
        failures.append(f"the last state is {states[-1] if states else None!r}, expected {state}")

    nonfinite = analyze.finite_stats(analyze.Run(run, None, None))

    if nonfinite["nonfinite_samples"] != 0:
        failures.append(
            f"{nonfinite['nonfinite_samples']} non-finite sample(s), first at "
            f"{nonfinite.get('first_nonfinite_time')} s"
        )

    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument(
        "--expect", action="append", default=[], metavar="KEY=VALUE", help="an integer meta.json field that must hold"
    )
    parser.add_argument("--expect-state", default=None, metavar="NAME", help="the state every run must end in")
    arguments = parser.parse_args()
    expected = {}

    for item in arguments.expect:
        key, _, value = item.partition("=")
        expected[key] = int(value)

    failed = 0

    for run in arguments.runs:
        failures = check(run, expected, arguments.expect_state)

        if failures:
            failed += 1
            print(f"FAIL {run}")

            for failure in failures:
                print(f"     {failure}")
        else:
            print(f"ok   {run}")

    if failed:
        print(f"\nFAIL: {failed} run(s) did not pass")
        return 1

    print("\nOK: every run is clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())
