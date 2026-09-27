#!/usr/bin/env python3
"""Analyse one simulation run directory.

Reads ``<run>/data.csv`` and ``<run>/meta.json`` and writes ``report.json`` plus a
set of PNGs next to them.

This script knows the engine's columns only: the body block every row starts with
(``x``, ``y``, ``z``, ``roll``, ``pitch``, ``yaw``, ``v_forward``, ``wz_body``) and
the kinds of column a robot target configures, recognised by their suffix or
prefix (``<label>_ncon``, ``<label>_fn``, ``<label>_slip``, ``<label>_penetration``,
``wheel_speed_<label>``, ``motor_torque_<label>``, and the ``<device>_voltage``
columns of the devices). A contact label with a ``_slip`` column is a wheel.

What a robot means by its own columns comes from its plugin, a module at
``targets/<target>/tools/analysis.py`` loaded when ``meta.json`` names the target,
or from ``--plugin``. A plugin may define any of:

* ``run_start(run)``: the time the robot starts its task, marked on every plot,
* ``report(run, report)``: extra report sections, merged into the report,
* ``speed_traces(run)``: ``(label, values, "linear" | "angular")`` for the speed plot,
* ``trajectory_overlays(run)``: ``(label, x, y)`` drawn over the trajectory,
* ``plots(run, report, directory)``: extra figures,
* ``summarize(report)``: extra summary lines.

``*_penetration`` columns use NaN as the "geom had no contact this tick" sentinel,
so they are excluded from the non-finite sample count.

Only numpy and matplotlib are used.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from pathlib import Path
from types import ModuleType

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

CELL_SIZE = 0.18
FLAT_Z_TOLERANCE = 1e-4
MAX_INTERVALS = 20
REPOSITORY = Path(__file__).resolve().parent.parent


def load_csv(path: Path) -> tuple[dict[str, np.ndarray], dict]:
    """Read the CSV into a name -> float array mapping.

    A run killed by a firmware crash leaves a partial final line, so rows that
    do not carry exactly one field per header column, or whose fields do not
    parse as numbers, are dropped and reported instead of raising.
    """
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        rows = [row for row in reader if row]

    parsed: list[list[float]] = []
    dropped = 0

    for row in rows:
        if len(row) != len(header):
            dropped += 1
            continue

        try:
            parsed.append([float(field) for field in row])
        except ValueError:
            dropped += 1

    table = np.array(parsed, dtype=float) if parsed else np.zeros((0, len(header)))
    columns = {name: table[:, index] for index, name in enumerate(header)}
    crash = {
        "truncated": dropped > 0,
        "dropped_rows": dropped,
        "last_complete_tick": int(table[-1, 0]) if parsed else None,
        "last_complete_time": float(table[-1, 1]) if parsed and len(header) > 1 else None,
    }

    return columns, crash


class Run:
    """Convenience accessor tolerating columns a robot does not write."""

    def __init__(self, directory: Path, t0: float | None, t1: float | None):
        self.directory = directory
        self.columns, self.crash = load_csv(directory / "data.csv")
        self.meta = {}
        meta_path = directory / "meta.json"

        if meta_path.exists():
            self.meta = json.loads(meta_path.read_text())

        time = self.columns.get("sim_time", np.zeros(0))
        mask = np.ones(time.shape, dtype=bool)

        if t0 is not None:
            mask &= time >= t0

        if t1 is not None:
            mask &= time <= t1

        self.columns = {name: values[mask] for name, values in self.columns.items()}
        self.time = self.columns.get("sim_time", np.zeros(0))

    def get(self, name: str) -> np.ndarray:
        """Return a column, or an all-NaN column of the right length if absent."""
        if name in self.columns:
            return self.columns[name]

        return np.full(self.time.shape, np.nan)

    def labels(self, suffix: str) -> list[str]:
        """Labels of every ``<label><suffix>`` column, in header order."""
        return [name[: -len(suffix)] for name in self.columns if name.endswith(suffix)]

    def prefixed(self, prefix: str) -> list[str]:
        """Every column starting with a prefix, in header order."""
        return [name for name in self.columns if name.startswith(prefix)]

    @property
    def dt(self) -> float:
        if self.time.size < 2:
            return 0.0

        return float(np.median(np.diff(self.time)))


def load_plugin(run: Run, override: Path | None) -> ModuleType | None:
    """Load the robot's analysis plugin, if it has one."""
    path = override

    if path is None:
        target = run.meta.get("target")

        if not target:
            return None

        path = REPOSITORY / "targets" / target / "tools" / "analysis.py"

        if not path.exists():
            return None

    spec = importlib.util.spec_from_file_location(f"analysis_{path.parent.parent.name}", path)

    if spec is None or spec.loader is None:
        raise SystemExit(f"cannot load the analysis plugin {path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def hook(plugin: ModuleType | None, name: str):
    """A plugin function, or None when there is no plugin or it lacks that hook."""
    return getattr(plugin, name, None) if plugin is not None else None


def zero_intervals(values: np.ndarray, time: np.ndarray, minimum: int = 2) -> list[list[float]]:
    """Contiguous (start, end) times where values == 0 for at least `minimum` ticks."""
    intervals: list[list[float]] = []
    flag = values == 0
    start = None

    for index, is_zero in enumerate(flag):
        if is_zero and start is None:
            start = index
        elif not is_zero and start is not None:
            if index - start >= minimum:
                intervals.append([float(time[start]), float(time[index - 1])])
            start = None

    if start is not None and len(flag) - start >= minimum:
        intervals.append([float(time[start]), float(time[-1])])

    return intervals[:MAX_INTERVALS]


def finite_stats(run: Run) -> dict:
    """Count non-finite samples over every column but the penetration sentinels."""
    count = 0
    first_time = None

    for name, values in run.columns.items():
        if name.endswith("_penetration"):
            continue

        bad = ~np.isfinite(values)

        if not bad.any():
            continue

        count += int(bad.sum())
        when = float(run.time[np.argmax(bad)])
        first_time = when if first_time is None else min(first_time, when)

    return {"nonfinite_samples": count, "first_nonfinite_time": first_time}


def wheel_report(run: Run, label: str) -> dict:
    """Airborne time, slip and penetration of one wheel; no penetration when it never touched anything."""
    ncon = run.get(f"{label}_ncon")
    slip = run.get(f"{label}_slip")
    penetration = run.get(f"{label}_penetration")

    return {
        "airborne_fraction": float(np.mean(ncon == 0)) if ncon.size else None,
        "airborne_intervals": zero_intervals(ncon, run.time),
        "max_abs_slip": float(np.nanmax(np.abs(slip))) if slip.size else None,
        "min_penetration": (
            float(np.nanmin(penetration)) if penetration.size and np.isfinite(penetration).any() else None
        ),
    }


def contact_fractions(run: Run, wheels: list[str]) -> dict:
    """Fraction of ticks each non-wheel geom touched anything."""
    fractions = {}

    for label in run.labels("_ncon"):
        if label in wheels:
            continue

        ncon = run.get(f"{label}_ncon")
        fractions[label] = float(np.mean(ncon > 0)) if ncon.size and np.isfinite(ncon).any() else None

    return fractions


def z_report(run: Run) -> dict:
    """Height of the body, and the first time after which it stays within 100 um of its final value."""
    z = run.get("z")

    if z.size == 0:
        return {"min": None, "max": None, "final": None, "settle_time": None}

    final = float(z[-1])
    within = np.abs(z - final) < FLAT_Z_TOLERANCE
    settle = None
    outside = np.nonzero(~within)[0]
    index = 0 if outside.size == 0 else int(outside[-1]) + 1

    if index < z.size:
        settle = float(run.time[index])

    return {"min": float(np.min(z)), "max": float(np.max(z)), "final": final, "settle_time": settle}


def cell_index(values: np.ndarray) -> np.ndarray:
    """Maze cell index of a world coordinate, same origin as the topdown plot."""
    with np.errstate(invalid="ignore"):
        return np.floor(values / CELL_SIZE)


def resolve_maze(run: Run) -> Path:
    """Find the ASCII maze the run drove in, or fail.

    ``meta.json`` names it as ``maze_path``. A relative path is tried against the
    current directory and the directory the ``runs/`` tree sits in.
    """
    roots = [Path.cwd(), run.directory.resolve().parent.parent]
    named = run.meta.get("maze_path") or ""
    candidates: list[Path] = [root / named for root in roots] if named else []

    maze_path = next((candidate for candidate in candidates if candidate.exists()), None)

    if maze_path is None:
        tried = ", ".join(str(candidate) for candidate in candidates) or "nothing: meta.json names no maze"
        raise SystemExit(f"cannot find the maze of {run.directory}; tried {tried}")

    return maze_path


def maze_goal_cells(maze_path: Path) -> list[tuple[int, int]]:
    """Read the goal cells, marked ``G``, from the ASCII maze, bottom-up like the walls."""
    inverted = maze_path.read_text().splitlines()[::-1]
    cells = []

    for index, line in enumerate(inverted):
        if index % 2 == 0:
            continue

        for column, start in enumerate(range(0, len(line), 4)):
            if line[start + 1 : start + 4].strip() == "G":
                cells.append((column, index // 2))

    return cells


def maze_segments(maze_path: Path) -> list[tuple[float, float, float, float]]:
    """Parse the ASCII maze into world-frame wall segments.

    Lines are read bottom-up, as ``Maze::parse`` reads them, alternating rows of
    posts, which hold the horizontal walls, and rows of cells, which hold the
    vertical ones: a post sits at ``(col, row) * 0.18``, a horizontal wall spans
    one cell in x at ``y = row * 0.18`` and a vertical wall spans one cell in y at
    ``x = col * 0.18``.
    """
    inverted = maze_path.read_text().splitlines()[::-1]
    segments = []

    for index, line in enumerate(inverted):
        row = index // 2

        if index % 2 == 0:
            for column, start in enumerate(range(0, len(line), 4)):
                if line[start + 1 : start + 4] == "---":
                    y = row * CELL_SIZE
                    segments.append((column * CELL_SIZE, y, (column + 1) * CELL_SIZE, y))
        else:
            for column, start in enumerate(range(0, len(line), 4)):
                if start < len(line) and line[start] == "|":
                    x = column * CELL_SIZE
                    segments.append((x, row * CELL_SIZE, x, (row + 1) * CELL_SIZE))

    return segments


def maze_report(run: Run, maze_path: Path) -> dict:
    """True maze cell of the robot, and whether it reached a goal cell."""
    goal_cells = maze_goal_cells(maze_path)
    true_x = cell_index(run.get("x"))
    true_y = cell_index(run.get("y"))
    in_goal = np.zeros(true_x.shape, dtype=bool)

    for goal_x, goal_y in goal_cells:
        in_goal |= (true_x == goal_x) & (true_y == goal_y)

    reached = np.nonzero(in_goal)[0]

    return {
        "goal_cells": [[x, y] for x, y in goal_cells],
        "goal_reached": bool(reached.size),
        "goal_time": float(run.time[reached[0]]) if reached.size else None,
        "final_true_cell": (
            [int(true_x[-1]), int(true_y[-1])] if run.time.size and np.isfinite(true_x[-1]) else None
        ),
    }


def build_report(run: Run, maze_path: Path, plugin: ModuleType | None) -> dict:
    pitch = run.get("pitch")
    roll = run.get("roll")
    wheels = run.labels("_slip")
    start = hook(plugin, "run_start")

    report = {
        "run": str(run.directory),
        "target": run.meta.get("target"),
        "robot": run.meta.get("robot_path"),
        "scenario": run.meta.get("scenario_path"),
        "args": run.meta.get("args"),
        "ticks": int(run.time.size),
        "duration": float(run.time[-1] - run.time[0]) if run.time.size else 0.0,
        "dt": run.dt,
        "run_start_time": start(run) if start else None,
        "warnings_total": float(run.meta.get("warnings_total", 0)),
        "events": run.meta.get("events", []),
        "collisions": [event["time"] for event in run.meta.get("events", []) if event.get("kind") == "collision"],
        "nonfinite": finite_stats(run),
        "wheels": {label: wheel_report(run, label) for label in wheels},
        "contact_fractions": contact_fractions(run, wheels),
        "z": z_report(run),
        "max_abs_pitch_deg": float(np.degrees(np.nanmax(np.abs(pitch)))) if pitch.size else None,
        "max_abs_roll_deg": float(np.degrees(np.nanmax(np.abs(roll)))) if roll.size else None,
        "maze_path": str(maze_path),
        "maze": maze_report(run, maze_path),
        "crash": run.crash,
    }

    extra = hook(plugin, "report")

    if extra:
        for key, value in extra(run, report).items():
            if isinstance(value, dict) and isinstance(report.get(key), dict):
                report[key].update(value)
            else:
                report[key] = value

    return report


def mark_run_start(axis, run_start: float | None) -> None:
    if run_start is not None:
        axis.axvline(run_start, color="k", linestyle="--", linewidth=0.8, label="task start")


def save(figure, path: Path) -> None:
    figure.tight_layout()
    figure.savefig(path, dpi=120)
    plt.close(figure)


def plot_speeds(run: Run, run_start: float | None, plugin: ModuleType | None, path: Path) -> None:
    figure, axis = plt.subplots(figsize=(10, 5))
    twin = axis.twinx()
    traces = hook(plugin, "speed_traces")

    for label, values, kind in traces(run) if traces else []:
        target = axis if kind == "linear" else twin
        target.plot(run.time, values, label=label, alpha=1.0 if kind == "linear" else 0.5)

    axis.plot(run.time, run.get("v_forward"), label="v_forward (ground truth)")
    twin.plot(run.time, run.get("wz_body"), color="tab:brown", alpha=0.5, label="wz_body (ground truth)")
    axis.set_xlabel("time [s]")
    axis.set_ylabel("linear speed [m/s]")
    twin.set_ylabel("angular speed [rad/s]")
    mark_run_start(axis, run_start)

    handles, labels = axis.get_legend_handles_labels()
    extra = twin.get_legend_handles_labels()
    axis.legend(handles + extra[0], labels + extra[1], fontsize=7, loc="upper left")
    save(figure, path)


def plot_contacts(run: Run, run_start: float | None, path: Path) -> None:
    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

    for label in run.labels("_ncon"):
        axes[0].step(run.time, run.get(f"{label}_ncon"), where="post", label=f"{label}_ncon", linewidth=0.8)

    axes[0].set_ylabel("contact count")
    axes[0].legend(fontsize=7)

    for label in run.labels("_fn"):
        axes[1].plot(run.time, run.get(f"{label}_fn"), label=f"{label}_fn", linewidth=0.8)

    axes[1].set_ylabel("normal force [N]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(fontsize=7)

    for axis in axes:
        mark_run_start(axis, run_start)

    save(figure, path)


def plot_attitude(run: Run, run_start: float | None, path: Path) -> None:
    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    axes[0].plot(run.time, run.get("z"), label="z")
    axes[0].set_ylabel("height [m]")
    axes[0].legend(fontsize=7)

    axes[1].plot(run.time, np.degrees(run.get("pitch")), label="pitch")
    axes[1].plot(run.time, np.degrees(run.get("roll")), label="roll")
    axes[1].set_ylabel("angle [deg]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(fontsize=7)

    for axis in axes:
        mark_run_start(axis, run_start)

    save(figure, path)


def plot_actuators(run: Run, run_start: float | None, path: Path) -> None:
    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

    for name in [name for name in run.columns if name.endswith("_voltage")]:
        axes[0].plot(run.time, run.get(name), label=name)

    axes[0].set_ylabel("voltage [V]")
    axes[0].legend(fontsize=7)

    for name in run.prefixed("motor_torque_"):
        axes[1].plot(run.time, run.get(name), label=name)

    axes[1].set_ylabel("motor torque [N m]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(fontsize=7)

    for axis in axes:
        mark_run_start(axis, run_start)

    save(figure, path)


def plot_wheels(run: Run, run_start: float | None, path: Path) -> None:
    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

    for name in run.prefixed("wheel_speed_"):
        axes[0].plot(run.time, run.get(name), label=name)

    axes[0].set_ylabel("wheel speed [rad/s]")
    axes[0].legend(fontsize=7)

    for label in run.labels("_slip"):
        axes[1].plot(run.time, run.get(f"{label}_slip"), label=f"{label}_slip")

    axes[1].set_ylabel("slip speed [m/s]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(fontsize=7)

    for axis in axes:
        mark_run_start(axis, run_start)

    save(figure, path)


def plot_topdown(run: Run, maze_path: Path, plugin: ModuleType | None, path: Path) -> None:
    figure, axis = plt.subplots(figsize=(7, 7))

    for x0, y0, x1, y1 in maze_segments(maze_path):
        axis.plot([x0, x1], [y0, y1], color="0.4", linewidth=1.5)

    for cell_x, cell_y in maze_goal_cells(maze_path):
        axis.add_patch(
            plt.Rectangle(
                (cell_x * CELL_SIZE, cell_y * CELL_SIZE),
                CELL_SIZE,
                CELL_SIZE,
                color="tab:green",
                alpha=0.15,
                zorder=0,
            )
        )

    x = run.get("x")
    y = run.get("y")

    if run.time.size > 1:
        points = np.column_stack([x, y]).reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        trajectory = LineCollection(segments, cmap="viridis", linewidth=1.2)
        trajectory.set_array(run.time[:-1])
        axis.add_collection(trajectory)
        figure.colorbar(trajectory, ax=axis, label="time [s]", fraction=0.046)
    elif run.time.size:
        axis.plot(x, y, color="tab:blue", linewidth=1.0)

    overlays = hook(plugin, "trajectory_overlays")

    styles = [("tab:red", "--"), ("tab:orange", ":"), ("tab:purple", "-.")]

    for index, (label, overlay_x, overlay_y) in enumerate(overlays(run) if overlays else []):
        color, linestyle = styles[index % len(styles)]
        axis.plot(overlay_x, overlay_y, color=color, linestyle=linestyle, linewidth=1.0, label=label)

    if run.time.size:
        axis.plot(x[0], y[0], "go", markersize=6, label="start")

    axis.set_aspect("equal")
    axis.autoscale_view()
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.legend(fontsize=7)
    save(figure, path)


def fmt(value, digits: int = 3) -> str:
    """Format a possibly missing number for the stdout summary."""
    return "n/a" if value is None else f"{value:.{digits}f}"


def summarize(report: dict, plugin: ModuleType | None) -> None:
    print(f"run          {report['run']}")
    print(f"ticks        {report['ticks']}  duration {report['duration']:.3f} s  dt {report['dt'] * 1e3:.4f} ms")
    print(f"warnings     {report['warnings_total']:.0f}  non-finite samples {report['nonfinite']['nonfinite_samples']}")
    start = report["run_start_time"]
    print(f"task start   {'never' if start is None else f'{start:.3f} s'}")
    collisions = report["collisions"]
    print(f"collisions   {len(collisions)}  " + " ".join(f"{time:.3f}s" for time in collisions[:10]))

    for label, wheel in report["wheels"].items():
        print(
            f"{label + ' wheel':12} airborne {fmt(wheel['airborne_fraction'])}  "
            f"intervals {len(wheel['airborne_intervals'])}  "
            f"max|slip| {fmt(wheel['max_abs_slip'], 4)}  min penetration {fmt(wheel['min_penetration'], 6)}"
        )

    for label, fraction in report["contact_fractions"].items():
        print(f"{label + ' contact':12} fraction {fmt(fraction)}")

    z = report["z"]
    print(
        f"z            min {fmt(z['min'], 5)}  max {fmt(z['max'], 5)}  "
        f"final {fmt(z['final'], 5)}  settled {z['settle_time']}"
    )
    print(f"attitude     max|pitch| {fmt(report['max_abs_pitch_deg'])} deg  max|roll| {fmt(report['max_abs_roll_deg'])} deg")

    maze = report["maze"]
    goal = "never" if maze["goal_time"] is None else f"{maze['goal_time']:.3f} s"
    print(f"goal         reached {maze['goal_reached']} at {goal}  true cell {maze['final_true_cell']}")

    crash = report["crash"]

    if crash["truncated"]:
        print(
            f"crash        truncated CSV, {crash['dropped_rows']} partial row(s), "
            f"last complete tick {crash['last_complete_tick']} at {crash['last_complete_time']} s"
        )

    extra = hook(plugin, "summarize")

    if extra:
        extra(report)


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyse a simulation run directory")
    parser.add_argument("run", type=Path)
    parser.add_argument("--t0", type=float, default=None)
    parser.add_argument("--t1", type=float, default=None)
    parser.add_argument("--plugin", type=Path, default=None, help="analysis plugin to use instead of the target's")
    arguments = parser.parse_args()

    run = Run(arguments.run, arguments.t0, arguments.t1)
    plugin = load_plugin(run, arguments.plugin)
    maze_path = resolve_maze(run)
    report = build_report(run, maze_path, plugin)
    (arguments.run / "report.json").write_text(json.dumps(report, indent=2) + "\n")

    run_start = report["run_start_time"]
    plot_speeds(run, run_start, plugin, arguments.run / "speeds.png")
    plot_contacts(run, run_start, arguments.run / "contacts.png")
    plot_attitude(run, run_start, arguments.run / "attitude.png")
    plot_actuators(run, run_start, arguments.run / "actuators.png")
    plot_wheels(run, run_start, arguments.run / "wheels.png")
    plot_topdown(run, maze_path, plugin, arguments.run / "topdown.png")

    extra_plots = hook(plugin, "plots")

    if extra_plots:
        extra_plots(run, report, arguments.run)

    summarize(report, plugin)


if __name__ == "__main__":
    main()
