#!/usr/bin/env python3
"""Analyse one micras_simulation run directory.

Reads ``<run>/data.csv`` (and ``<run>/meta.json`` when present) and writes
``report.json`` plus a set of PNGs next to them.

Conventions taken from the harness:

* The model's forward axis is **+y in body frame** (``models/robot_legacy.xml``
  puts the lidars and the caster at positive y).
* ``yaw`` is produced by ``src/sim/recorder.cpp::to_euler`` as
  ``atan2(2(wz + xy), 1 - 2(y^2 + z^2))``, i.e. a ZYX yaw about the world +z
  axis, zero when the body +y axis points along the world +y axis.
  The body forward direction in world coordinates is therefore
  ``(-sin(yaw), cos(yaw))`` and the ground-truth forward speed is
  ``-vx_world*sin(yaw) + vy_world*cos(yaw)``. Current harnesses write that
  projection out as the ``v_forward`` column and this script prefers it;
  older runs whose CSV only has ``vx``/``vy`` are still handled.
* ``*_penetration`` columns use NaN as the "geom had no contact this tick"
  sentinel, so they are excluded from the non-finite sample count.
* Wheel radius is 0.011 m (``models/robot_legacy.xml``, ``class="wheel"``).

Only numpy and matplotlib are used; pandas is not available on this machine.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

WHEEL_RADIUS = 0.011
CELL_SIZE = 0.18
#: Indices of the four centre cells of the 16x16 maze, in both axes.
GOAL_CELLS = (7, 8)
#: Id of Micras::State::RUN in the firmware FSM.
RUN_STATE_ID = 3
SATURATION = 100.0
FLAT_Z_TOLERANCE = 1e-4
DECEL_DROP = 0.05
DECEL_WINDOW_S = 0.05
MAX_INTERVALS = 20


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
    """Convenience accessor tolerating columns an older harness did not write."""

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

    @property
    def dt(self) -> float:
        if self.time.size < 2:
            return 0.0

        return float(np.median(np.diff(self.time)))


#: Columns whose NaN entries are a deliberate "no contact" sentinel, not a defect.
SENTINEL_NAN_COLUMNS = ("left_penetration", "right_penetration")


def forward_speed(run: Run) -> np.ndarray:
    """Ground-truth forward speed: world linear velocity projected on the body +y axis.

    Prefers the ``v_forward`` column the harness now writes; falls back to
    recomputing it from the world velocity columns for older runs.
    """
    if "v_forward" in run.columns:
        return run.columns["v_forward"]

    yaw = run.get("yaw")
    vx = run.get("vx_world") if "vx_world" in run.columns else run.get("vx")
    vy = run.get("vy_world") if "vy_world" in run.columns else run.get("vy")
    return -vx * np.sin(yaw) + vy * np.cos(yaw)


def yaw_rate(run: Run) -> np.ndarray:
    """Ground-truth yaw rate, from the body angular velocity column."""
    return run.get("wz_body") if "wz_body" in run.columns else run.get("wz")


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
    """Count non-finite samples over every column."""
    count = 0
    first_time = None

    for name, values in run.columns.items():
        if name in SENTINEL_NAN_COLUMNS:
            continue

        bad = ~np.isfinite(values)

        if not bad.any():
            continue

        count += int(bad.sum())
        when = float(run.time[np.argmax(bad)])
        first_time = when if first_time is None else min(first_time, when)

    return {"nonfinite_samples": count, "first_nonfinite_time": first_time}


def wheel_report(run: Run, side: str) -> dict:
    ncon = run.get(f"{side}_ncon")
    slip = run.get(f"{side}_slip")
    penetration = run.get(f"{side}_penetration")

    return {
        "airborne_fraction": float(np.mean(ncon == 0)) if ncon.size else None,
        "airborne_intervals": zero_intervals(ncon, run.time),
        "max_abs_slip": float(np.nanmax(np.abs(slip))) if slip.size else None,
        # All NaN means the wheel never touched anything in the window.
        "min_penetration": (
            float(np.nanmin(penetration)) if penetration.size and np.isfinite(penetration).any() else None
        ),
    }


def z_report(run: Run) -> dict:
    z = run.get("z")

    if z.size == 0:
        return {"min": None, "max": None, "final": None, "settle_time": None}

    final = float(z[-1])
    within = np.abs(z - final) < FLAT_Z_TOLERANCE
    settle = None

    # First time after which the height never leaves the 100 um band again.
    outside = np.nonzero(~within)[0]
    index = 0 if outside.size == 0 else int(outside[-1]) + 1

    if index < z.size:
        settle = float(run.time[index])

    return {"min": float(np.min(z)), "max": float(np.max(z)), "final": final, "settle_time": settle}


def odometry_report(run: Run, run_start: float | None) -> dict:
    desired = run.get("desired_linear_speed")
    window = np.isfinite(desired) & (desired != 0)
    truth = forward_speed(run)
    measured = run.get("odometry_linear_velocity")
    valid = window & np.isfinite(truth) & np.isfinite(measured)

    if not valid.any():
        return {"samples": 0, "rms_error": None, "max_error": None, "run_start_time": run_start}

    error = measured[valid] - truth[valid]
    return {
        "samples": int(valid.sum()),
        "rms_error": float(np.sqrt(np.mean(error**2))),
        "max_error": float(np.max(np.abs(error))),
        "run_start_time": run_start,
    }


def cell_index(values: np.ndarray) -> np.ndarray:
    """Maze cell index of a world coordinate, same origin as the topdown plot."""
    with np.errstate(invalid="ignore"):
        return np.floor(values / CELL_SIZE)


def run_mask(run: Run, run_start: float | None) -> np.ndarray:
    """Ticks the firmware spent in the RUN state.

    Uses the ``fsm_state`` column when the firmware exports it and falls back to
    "everything from the first non-zero speed setpoint on" for older runs.
    """
    if "fsm_state" in run.columns:
        return run.columns["fsm_state"] == RUN_STATE_ID

    if run_start is None:
        return np.zeros(run.time.shape, dtype=bool)

    return run.time >= run_start


def maze_report(run: Run, run_start: float | None) -> dict:
    """Ground-truth and believed maze cell, and whether the goal was reached."""
    true_x = cell_index(run.get("x"))
    true_y = cell_index(run.get("y"))
    in_goal = np.isin(true_x, GOAL_CELLS) & np.isin(true_y, GOAL_CELLS)
    reached = np.nonzero(in_goal)[0]

    report = {
        "goal_cells": [[x, y] for x in GOAL_CELLS for y in GOAL_CELLS],
        "goal_reached": bool(reached.size),
        "goal_time": float(run.time[reached[0]]) if reached.size else None,
        "final_true_cell": (
            [int(true_x[-1]), int(true_y[-1])] if run.time.size and np.isfinite(true_x[-1]) else None
        ),
        "final_grid_cell": None,
        "cell_mismatch_fraction": None,
        "first_mismatch_time": None,
    }

    if "grid_pose_x" not in run.columns or "grid_pose_y" not in run.columns:
        return report

    grid_x = run.columns["grid_pose_x"]
    grid_y = run.columns["grid_pose_y"]

    if run.time.size and np.isfinite(grid_x[-1]) and np.isfinite(grid_y[-1]):
        report["final_grid_cell"] = [int(grid_x[-1]), int(grid_y[-1])]

    window = run_mask(run, run_start) & np.isfinite(grid_x) & np.isfinite(grid_y) & np.isfinite(true_x)

    if not window.any():
        return report

    mismatch = window & ((grid_x != true_x) | (grid_y != true_y))
    report["cell_mismatch_fraction"] = float(mismatch.sum() / window.sum())
    first = np.nonzero(mismatch)[0]

    if first.size:
        report["first_mismatch_time"] = float(run.time[first[0]])

    return report


def turn_starts(run: Run) -> np.ndarray:
    """Indices where the angular speed setpoint leaves zero."""
    desired = run.get("desired_angular_speed")

    if desired.size < 2:
        return np.zeros(0, dtype=int)

    moving = np.isfinite(desired) & (desired != 0)
    return np.nonzero(moving[1:] & ~moving[:-1])[0] + 1


def odometry_pose_report(run: Run) -> dict:
    """Distance between the odometry-estimated position and the ground truth."""
    if "odometry_state_x" not in run.columns or "odometry_state_y" not in run.columns:
        return {"samples": 0, "max": None, "max_time": None, "final": None, "at_turn_starts": []}

    error = np.hypot(run.columns["odometry_state_x"] - run.get("x"), run.columns["odometry_state_y"] - run.get("y"))
    valid = np.isfinite(error)

    if not valid.any():
        return {"samples": 0, "max": None, "max_time": None, "final": None, "at_turn_starts": []}

    peak = int(np.nanargmax(np.where(valid, error, -np.inf)))

    return {
        "samples": int(valid.sum()),
        "max": float(error[peak]),
        "max_time": float(run.time[peak]),
        "final": float(error[valid][-1]),
        "at_turn_starts": [
            {"time": float(run.time[index]), "error": float(error[index])}
            for index in turn_starts(run)[:MAX_INTERVALS]
            if np.isfinite(error[index])
        ],
    }


def fsm_timeline(run: Run) -> list[dict]:
    """Transitions of the firmware state machine, empty when not exported."""
    if "fsm_state" not in run.columns or run.time.size == 0:
        return []

    state = run.columns["fsm_state"]
    changed = np.concatenate([[True], state[1:] != state[:-1]])

    return [
        {"time": float(run.time[index]), "state": int(state[index])}
        for index in np.nonzero(changed)[0]
        if np.isfinite(state[index])
    ]


def deceleration_events(run: Run) -> list[dict]:
    """Drops of more than 0.05 m/s within 50 ms while the setpoint is not falling."""
    truth = forward_speed(run)
    desired = run.get("desired_linear_speed")
    dt = run.dt

    if dt <= 0 or truth.size < 3:
        return []

    span = max(1, int(round(DECEL_WINDOW_S / dt)))

    if truth.size <= span:
        return []

    head = np.arange(truth.size - span)
    tail = head + span
    drop = truth[head] - truth[tail]
    moving = (desired[head] != 0) & (desired[tail] != 0)
    not_decreasing = desired[tail] >= desired[head]
    hit = np.nonzero((drop > DECEL_DROP) & moving & not_decreasing)[0]

    events: list[dict] = []

    for index in hit:
        start, end = int(index), int(index) + span

        if events and start <= events[-1]["_end"]:
            events[-1]["_end"] = max(events[-1]["_end"], end)
            events[-1]["_drop"] = max(events[-1]["_drop"], float(drop[index]))
            continue

        events.append({"_start": start, "_end": end, "_drop": float(drop[index])})

    report = []

    for event in events[:MAX_INTERVALS]:
        lo, hi = event["_start"], event["_end"] + 1
        pitch = np.abs(run.get("pitch")[lo:hi])
        report.append(
            {
                "start": float(run.time[lo]),
                "end": float(run.time[min(hi, run.time.size) - 1]),
                "speed_drop": event["_drop"],
                "min_left_ncon": float(np.min(run.get("left_ncon")[lo:hi])),
                "min_right_ncon": float(np.min(run.get("right_ncon")[lo:hi])),
                "max_slip": float(
                    np.nanmax(np.concatenate([run.get("left_slip")[lo:hi], run.get("right_slip")[lo:hi]]))
                ),
                "max_abs_pitch_deg": float(np.degrees(np.nanmax(pitch))) if pitch.size else None,
            }
        )

    return report


def build_report(run: Run) -> dict:
    desired = run.get("desired_linear_speed")
    nonzero = np.nonzero(np.isfinite(desired) & (desired != 0))[0]
    run_start = float(run.time[nonzero[0]]) if nonzero.size else None

    ctrl_left = run.get("ctrl_left")
    ctrl_right = run.get("ctrl_right")
    base_ncon = run.get("base_ncon")
    warnings = run.get("warnings_total")
    pitch = run.get("pitch")
    roll = run.get("roll")

    return {
        "run": str(run.directory),
        "model": run.meta.get("model_path"),
        "args": run.meta.get("args"),
        "ticks": int(run.time.size),
        "duration": float(run.time[-1] - run.time[0]) if run.time.size else 0.0,
        "dt": run.dt,
        "warnings_total": float(np.nanmax(warnings)) if warnings.size and np.isfinite(warnings).any() else 0.0,
        "nonfinite": finite_stats(run),
        "left_wheel": wheel_report(run, "left"),
        "right_wheel": wheel_report(run, "right"),
        "base_contact_fraction": (
            float(np.mean(base_ncon > 0)) if base_ncon.size and np.isfinite(base_ncon).any() else None
        ),
        "z": z_report(run),
        "max_abs_pitch_deg": float(np.degrees(np.nanmax(np.abs(pitch)))) if pitch.size else None,
        "max_abs_roll_deg": float(np.degrees(np.nanmax(np.abs(roll)))) if roll.size else None,
        "ctrl_saturation_fraction": {
            "left": float(np.mean(np.abs(ctrl_left) >= SATURATION)) if ctrl_left.size else None,
            "right": float(np.mean(np.abs(ctrl_right) >= SATURATION)) if ctrl_right.size else None,
        },
        "odometry": odometry_report(run, run_start),
        "odometry_pose_error": odometry_pose_report(run),
        "maze": maze_report(run, run_start),
        "fsm_state": fsm_timeline(run),
        "crash": run.crash,
        "deceleration_events": deceleration_events(run),
    }


def mark_run_start(axis, run_start: float | None) -> None:
    if run_start is not None:
        axis.axvline(run_start, color="k", linestyle="--", linewidth=0.8, label="RUN start")


def save(figure, path: Path) -> None:
    figure.tight_layout()
    figure.savefig(path, dpi=120)
    plt.close(figure)


def plot_speeds(run: Run, run_start: float | None, path: Path) -> None:
    figure, axis = plt.subplots(figsize=(10, 5))
    axis.plot(run.time, run.get("desired_linear_speed"), label="desired_linear_speed")
    axis.plot(run.time, run.get("odometry_linear_velocity"), label="odometry_linear_velocity")
    axis.plot(run.time, forward_speed(run), label="ground-truth forward speed")
    axis.set_xlabel("time [s]")
    axis.set_ylabel("linear speed [m/s]")
    mark_run_start(axis, run_start)

    twin = axis.twinx()
    twin.plot(run.time, run.get("desired_angular_speed"), color="tab:red", alpha=0.5, label="desired_angular_speed")
    twin.plot(
        run.time, run.get("odometry_angular_velocity"), color="tab:purple", alpha=0.5, label="odometry_angular_velocity"
    )
    twin.plot(run.time, yaw_rate(run), color="tab:brown", alpha=0.5, label="wz_body (ground truth)")
    twin.set_ylabel("angular speed [rad/s]")

    handles, labels = axis.get_legend_handles_labels()
    extra = twin.get_legend_handles_labels()
    axis.legend(handles + extra[0], labels + extra[1], fontsize=7, loc="upper left")
    save(figure, path)


def plot_contacts(run: Run, run_start: float | None, path: Path) -> None:
    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

    for name in ("left_ncon", "right_ncon", "caster_ncon", "base_ncon"):
        axes[0].step(run.time, run.get(name), where="post", label=name, linewidth=0.8)

    axes[0].set_ylabel("contact count")
    axes[0].legend(fontsize=7)

    for name in ("left_fn", "right_fn", "caster_fn"):
        axes[1].plot(run.time, run.get(name), label=name, linewidth=0.8)

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


def plot_control(run: Run, run_start: float | None, path: Path) -> None:
    figure, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    axes[0].plot(run.time, run.get("linear_pid_response"), label="linear_pid_response")
    axes[0].plot(run.time, run.get("angular_pid_response"), label="angular_pid_response")
    axes[0].set_ylabel("PID response")
    axes[0].legend(fontsize=7)

    axes[1].plot(run.time, run.get("left_feed_forward_response"), label="left_feed_forward_response")
    axes[1].plot(run.time, run.get("right_feed_forward_response"), label="right_feed_forward_response")
    axes[1].plot(run.time, run.get("ctrl_left"), label="ctrl_left")
    axes[1].plot(run.time, run.get("ctrl_right"), label="ctrl_right")
    axes[1].set_ylabel("command [%]")
    axes[1].legend(fontsize=7)

    axes[2].plot(run.time, run.get("act_force_left"), label="act_force_left")
    axes[2].plot(run.time, run.get("act_force_right"), label="act_force_right")
    axes[2].set_ylabel("actuator force [N m]")
    axes[2].set_xlabel("time [s]")
    axes[2].legend(fontsize=7)

    for axis in axes:
        mark_run_start(axis, run_start)

    save(figure, path)


def plot_wheels(run: Run, run_start: float | None, path: Path) -> None:
    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    axes[0].plot(run.time, run.get("wheel_qvel_left") * WHEEL_RADIUS, label="left wheel surface speed")
    axes[0].plot(run.time, run.get("wheel_qvel_right") * WHEEL_RADIUS, label="right wheel surface speed")
    axes[0].plot(run.time, forward_speed(run), label="ground-truth forward speed", linewidth=0.8)
    axes[0].set_ylabel("speed [m/s]")
    axes[0].legend(fontsize=7)

    axes[1].plot(run.time, run.get("left_slip"), label="left_slip")
    axes[1].plot(run.time, run.get("right_slip"), label="right_slip")
    axes[1].set_ylabel("slip speed [m/s]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(fontsize=7)

    for axis in axes:
        mark_run_start(axis, run_start)

    save(figure, path)


def maze_segments(maze_path: Path) -> list[tuple[float, float, float, float]]:
    """Parse the ASCII maze into world-frame wall segments.

    Matches ``models/gen_maze.py``: lines are read bottom-up, a post sits at
    ``(col, row) * 0.18``, a horizontal wall spans one cell in x at ``y = row * 0.18``
    and a vertical wall spans one cell in y at ``x = col * 0.18``.
    """
    lines = [line.rstrip("\n") for line in maze_path.read_text().splitlines()]
    inverted = lines[::-1]
    segments = []

    for index, line in enumerate(inverted):
        row = index // 2

        if index % 2 == 0:  # post row: horizontal walls
            for column, start in enumerate(range(0, len(line), 4)):
                if line[start + 1 : start + 4] == "---":
                    y = row * CELL_SIZE
                    segments.append((column * CELL_SIZE, y, (column + 1) * CELL_SIZE, y))
        else:  # cell row: vertical walls
            for column, start in enumerate(range(0, len(line), 4)):
                if start < len(line) and line[start] == "|":
                    x = column * CELL_SIZE
                    segments.append((x, row * CELL_SIZE, x, (row + 1) * CELL_SIZE))

    return segments


def plot_topdown(run: Run, path: Path) -> None:
    figure, axis = plt.subplots(figsize=(7, 7))

    model = run.meta.get("model_path")

    if model:
        # meta.json stores the model path as it was typed on the command line,
        # so try it relative to the current directory and to the run's parent.
        candidates = [
            Path(model).parent / "maze.txt",
            (run.directory.resolve().parent.parent / Path(model).parent / "maze.txt"),
        ]
        maze_path = next((candidate for candidate in candidates if candidate.exists()), None)

        if maze_path is not None:
            for x0, y0, x1, y1 in maze_segments(maze_path):
                axis.plot([x0, x1], [y0, y1], color="0.4", linewidth=1.5)

    for cell_x in GOAL_CELLS:
        for cell_y in GOAL_CELLS:
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

    if "odometry_state_x" in run.columns and "odometry_state_y" in run.columns:
        axis.plot(
            run.columns["odometry_state_x"],
            run.columns["odometry_state_y"],
            color="tab:red",
            linestyle="--",
            linewidth=1.0,
            label="odometry estimate",
        )

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


def summarize(report: dict) -> None:
    print(f"run          {report['run']}")
    print(f"ticks        {report['ticks']}  duration {report['duration']:.3f} s  dt {report['dt'] * 1e3:.4f} ms")
    print(f"warnings     {report['warnings_total']:.0f}  non-finite samples {report['nonfinite']['nonfinite_samples']}")
    odometry = report["odometry"]
    start = odometry["run_start_time"]
    print(f"RUN start    {'never' if start is None else f'{start:.3f} s'}")

    print(f"odometry     rms {fmt(odometry['rms_error'], 4)} m/s  max {fmt(odometry['max_error'], 4)} m/s")

    for side in ("left_wheel", "right_wheel"):
        wheel = report[side]
        print(
            f"{side:12} airborne {fmt(wheel['airborne_fraction'])}  "
            f"intervals {len(wheel['airborne_intervals'])}  "
            f"max|slip| {fmt(wheel['max_abs_slip'], 4)}  min penetration {fmt(wheel['min_penetration'], 6)}"
        )

    z = report["z"]
    print(
        f"z            min {fmt(z['min'], 5)}  max {fmt(z['max'], 5)}  "
        f"final {fmt(z['final'], 5)}  settled {z['settle_time']}"
    )
    print(f"attitude     max|pitch| {fmt(report['max_abs_pitch_deg'])} deg  max|roll| {fmt(report['max_abs_roll_deg'])} deg")
    saturation = report["ctrl_saturation_fraction"]
    print(f"saturation   left {fmt(saturation['left'])}  right {fmt(saturation['right'])}")
    print(f"base contact fraction {report['base_contact_fraction']}")
    print(f"deceleration events   {len(report['deceleration_events'])}")

    maze = report["maze"]
    goal = "never" if maze["goal_time"] is None else f"{maze['goal_time']:.3f} s"
    print(
        f"goal         reached {maze['goal_reached']} at {goal}  "
        f"true cell {maze['final_true_cell']}  grid cell {maze['final_grid_cell']}"
    )
    mismatch = "n/a" if maze["first_mismatch_time"] is None else f"{maze['first_mismatch_time']:.3f} s"
    print(f"cell mismatch fraction {fmt(maze['cell_mismatch_fraction'])}  first at {mismatch}")

    pose = report["odometry_pose_error"]
    print(
        f"odom pose    max {fmt(pose['max'], 4)} m at {fmt(pose['max_time'])} s  "
        f"final {fmt(pose['final'], 4)} m  turn samples {len(pose['at_turn_starts'])}"
    )

    timeline = report["fsm_state"]

    if timeline:
        print("fsm          " + "  ".join(f"{entry['time']:.3f}s->{entry['state']}" for entry in timeline))

    crash = report["crash"]

    if crash["truncated"]:
        print(
            f"crash        truncated CSV, {crash['dropped_rows']} partial row(s), "
            f"last complete tick {crash['last_complete_tick']} at {crash['last_complete_time']} s"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyse a micras_simulation run directory")
    parser.add_argument("run", type=Path)
    parser.add_argument("--t0", type=float, default=None)
    parser.add_argument("--t1", type=float, default=None)
    arguments = parser.parse_args()

    run = Run(arguments.run, arguments.t0, arguments.t1)
    report = build_report(run)
    (arguments.run / "report.json").write_text(json.dumps(report, indent=2) + "\n")

    run_start = report["odometry"]["run_start_time"]
    plot_speeds(run, run_start, arguments.run / "speeds.png")
    plot_contacts(run, run_start, arguments.run / "contacts.png")
    plot_attitude(run, run_start, arguments.run / "attitude.png")
    plot_control(run, run_start, arguments.run / "control.png")
    plot_wheels(run, run_start, arguments.run / "wheels.png")
    plot_topdown(run, arguments.run / "topdown.png")

    summarize(report)


if __name__ == "__main__":
    main()
