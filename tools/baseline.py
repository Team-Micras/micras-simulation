#!/usr/bin/env python3
"""Record and compare run baselines: a hash of the bytes and a summary with tolerances.

A baseline is one ``summary.json`` per scenario:

* ``data_sha256``: the hash of ``data.csv``. Equal hashes mean the run is byte
  identical to the recorded one. A different hash is expected on another
  machine (compiler, libm and the MuJoCo build all move the last bits), so it
  only fails the comparison with ``--exact``.
* ``states``: the state timeline from ``meta.json``, compared by name and, within
  ``state_time`` seconds, by time.
* ``values``: numbers that say how the run went, each with its tolerance in
  ``tolerances``. The engine contributes the tick count, warnings, collisions,
  when the goal was first reached and when the scenario stopped the run; the
  robot's analysis plugin may add its own through a ``baseline(report)`` hook
  returning ``{name: (value, tolerance)}``.

The tolerances are written into the summary when it is recorded, so a reviewer
sees them next to the values. ``record`` refuses to overwrite a summary: a new
behavior is a new baseline version, never a re-recording that makes a
difference go away.

The robot's plugin is found as ``analyze.py`` finds it: ``--plugin``, then
``$MICRAS_SIM_PLUGIN``, then the target's folder that ``meta.json`` names.

Usage::

    baseline.py record  <run> <baseline>/<scenario> [--plugin <analysis.py>]
    baseline.py compare <run> <baseline>/<scenario> [--plugin <analysis.py>] [--exact]

``--exact`` is the byte identity check: it also fails when ``data.csv`` is not the
recorded run's bytes.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import analyze  # noqa: E402

#: Seconds a state change may move and still match.
STATE_TIME_TOLERANCE = 0.25

#: The engine's own values: name -> tolerance, None meaning exact.
ENGINE_TOLERANCES = {
    "ticks": None,
    "warnings_total": None,
    "collisions": 2,
    "goal_time": 1.0,
    "stopped_at": 1.0,
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)

    return digest.hexdigest()


def summarize(run_directory: Path, plugin_override: Path | None = None) -> dict:
    """Build the summary of one run from its files."""
    run = analyze.Run(run_directory, None, None)
    plugin = analyze.load_plugin(run, plugin_override)
    report = analyze.build_report(run, analyze.resolve_maze(run), plugin)
    meta = run.meta
    collisions = [event for event in meta.get("events", []) if event.get("kind") == "collision"]
    stopped_at = meta.get("stopped_at", -1)

    values = {
        "ticks": meta.get("ticks"),
        "warnings_total": meta.get("warnings_total"),
        "collisions": len(collisions),
        "goal_time": report["maze"]["goal_time"],
        "stopped_at": stopped_at if stopped_at is not None and stopped_at >= 0 else None,
    }
    tolerances = {"state_time": STATE_TIME_TOLERANCE, **ENGINE_TOLERANCES}
    extra = analyze.hook(plugin, "baseline")

    if extra:
        for name, (value, tolerance) in extra(report).items():
            values[name] = value
            tolerances[name] = tolerance

    return {
        "target": meta.get("target"),
        "scenario": meta.get("scenario_path"),
        "args": meta.get("args"),
        "firmware_sha": meta.get("firmware_sha"),
        "mujoco_version": meta.get("mujoco_version"),
        "compiler": meta.get("compiler"),
        "data_sha256": sha256(run_directory / "data.csv"),
        "states": [
            [event["time"], event["detail"]] for event in meta.get("events", []) if event.get("kind") == "state"
        ],
        "values": values,
        "tolerances": tolerances,
    }


def compare_value(name: str, got, want, tolerance) -> str | None:
    """Describe how a value misses the recorded one, or None when it matches."""
    if want is None or got is None:
        return None if got is want else f"{name}: {got!r} != recorded {want!r}"

    if tolerance is None:
        return None if got == want else f"{name}: {got!r} != recorded {want!r}"

    if abs(got - want) <= tolerance:
        return None

    return f"{name}: {got!r} is more than {tolerance} from recorded {want!r}"


def compare(summary: dict, recorded: dict, exact: bool) -> tuple[list[str], list[str]]:
    """Return (failures, notes) of a summary against the recorded one."""
    failures: list[str] = []
    notes: list[str] = []
    tolerances = recorded.get("tolerances", {})

    if summary["data_sha256"] == recorded["data_sha256"]:
        notes.append("data.csv is byte identical to the recorded run")
    elif exact:
        failures.append("data.csv differs from the recorded run's bytes")
    else:
        notes.append("data.csv bytes differ from the recorded run; comparing the summary")

    got_names = [name for _, name in summary["states"]]
    want_names = [name for _, name in recorded["states"]]

    if got_names != want_names:
        failures.append(f"state sequence {got_names} != recorded {want_names}")
    else:
        window = tolerances.get("state_time", STATE_TIME_TOLERANCE)

        for (got_time, name), (want_time, _) in zip(summary["states"], recorded["states"]):
            if abs(got_time - want_time) > window:
                failures.append(f"state {name} at {got_time} s, recorded at {want_time} s")

    for name, want in recorded["values"].items():
        problem = compare_value(name, summary["values"].get(name), want, tolerances.get(name))

        if problem:
            failures.append(problem)

    return failures, notes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=("record", "compare"))
    parser.add_argument("run", type=Path)
    parser.add_argument("baseline", type=Path, help="the scenario's folder in a baseline version")
    parser.add_argument("--exact", action="store_true", help="also fail when the bytes differ")
    parser.add_argument(
        "--plugin", type=Path, default=None, help="analysis plugin to use instead of the target's (also $MICRAS_SIM_PLUGIN)"
    )
    arguments = parser.parse_args()

    summary = summarize(arguments.run, arguments.plugin)
    path = arguments.baseline / "summary.json"

    if arguments.action == "record":
        if path.exists():
            print(f"FAIL {path} already exists; record a new baseline version instead")
            return 1

        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(summary, indent=2) + "\n")
        print(f"ok   recorded {path}")
        return 0

    failures, notes = compare(summary, json.loads(path.read_text()), arguments.exact)

    for note in notes:
        print(f"     {arguments.run}: {note}")

    if failures:
        for failure in failures:
            print(f"FAIL {arguments.run}: {failure}")
        return 1

    print(f"ok   {arguments.run} matches {arguments.baseline}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
