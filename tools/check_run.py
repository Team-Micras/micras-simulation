#!/usr/bin/env python3
"""Assert the health invariants of one or more finished run directories.

Reads ``meta.json`` and the ``report.json`` written by ``tools/analyze.py`` and
fails the build when a run is not clean:

* ``warnings_total`` is 0 (no MuJoCo solver or constraint warnings),
* no non-finite samples (the ``*_penetration`` NaN sentinel is excluded by
  ``analyze.py`` itself, every other NaN is a defect),
* exactly ``--pool-columns`` firmware pool CSV columns were decoded (custom
  serializable variables expand into one column per field, so this counts
  columns and not pool variables),
* the telemetry decoder never had to resynchronise,
* every tick that was asked for actually ran,
* nobody reached into the run through a window.

Exit code 0 when every run passes, 1 otherwise.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def check(run: Path, pool_columns: int) -> list[str]:
    """Return the list of failures for one run directory."""
    failures: list[str] = []

    meta_path = run / "meta.json"
    report_path = run / "report.json"

    for path in (meta_path, report_path):
        if not path.exists():
            failures.append(f"{path} is missing")

    if failures:
        return failures

    meta = json.loads(meta_path.read_text())
    report = json.loads(report_path.read_text())

    if meta.get("warnings_total") != 0:
        failures.append(f"warnings_total is {meta.get('warnings_total')}, expected 0")

    if meta.get("pool_columns") != pool_columns:
        failures.append(f"pool_columns is {meta.get('pool_columns')}, expected {pool_columns}")

    if meta.get("telemetry_resyncs", 0) != 0:
        failures.append(f"telemetry_resyncs is {meta.get('telemetry_resyncs')}, expected 0")

    if meta.get("interactive") is not False:
        failures.append(f"interactive is {meta.get('interactive')!r}; an interactive run is not reproducible")

    if meta.get("ticks") != meta.get("requested_ticks"):
        failures.append(f"only {meta.get('ticks')} of {meta.get('requested_ticks')} ticks ran")

    nonfinite = report.get("nonfinite", {})

    if nonfinite.get("nonfinite_samples", 0) != 0:
        failures.append(
            f"{nonfinite['nonfinite_samples']} non-finite sample(s), first at "
            f"{nonfinite.get('first_nonfinite_time')} s"
        )

    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument("--pool-columns", type=int, default=29)
    arguments = parser.parse_args()

    failed = 0

    for run in arguments.runs:
        failures = check(run, arguments.pool_columns)

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
