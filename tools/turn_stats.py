#!/usr/bin/env python3
"""Per-turn geometry table for one run.

A *turn* is a contiguous interval where ``|desired_angular_speed| > 0.5``.
For each one the script reports, from ground truth only:

- the type guessed from the integrated commanded angle (``turn90`` or ``spin``),
- the lateral offset from the corridor centre line and the heading error at the
  moment the turn starts, both measured against the cell grid
  (cell centre = ``(i + 0.5) * cell_size``),
- where along the cell the turn starts, signed, relative to the centre of the
  cell the robot is in: ``ActionQueuer`` queues ``move_to_turn`` of
  ``cell_size / 2 - curve_radius`` before the curve, so the nominal value is
  ``-curve_radius`` (-45 mm with the current constants),
- the executed radius ``v_forward / wz_body`` (median over the interval) and the
  executed versus commanded angle,
- the smallest distance from the chassis footprint to any wall over the turn and
  over the 0.3 s that follow it, and
- whether the chassis geom touched anything (``base_ncon > 0``) or any wall
  sensor saturated within 0.3 s of the end of the turn.

The chassis footprint is the ``base`` mesh of ``models/robot_v2.xml`` projected
on the floor: a hexagon 66 mm wide spanning -33..+66 mm in body y.  Walls are
rebuilt from ``models/maze.txt`` with the thickness the generator uses
(12.6 mm), posts included at every lattice point.
"""

import json
import math
import sys

import numpy as np

CELL = 0.18
WALL_HALF = 0.0063
TURN_GATE = 0.5          # rad/s, the |desired_angular_speed| threshold
SENSOR_SATURATION = 0.5  # wall sensor reading counted as "against the wall"
AFTER = 0.3              # s, the window looked at after a turn
MIN_ANGLE = math.pi / 4  # rad, below this the interval is a steering correction

# base mesh footprint, body frame, x right / y forward, metres
CHASSIS = np.array(
    [
        (0.031, -0.033),
        (0.033, 0.041),
        (0.0155, 0.066),
        (-0.0155, 0.066),
        (-0.033, 0.041),
        (-0.031, -0.033),
    ]
)


def load(run):
    with open(f"{run}/data.csv") as handle:
        names = handle.readline().strip().split(",")
    data = np.loadtxt(f"{run}/data.csv", delimiter=",", skiprows=1)
    return {name: data[:, i] for i, name in enumerate(names)}


def parse_maze(path):
    """Return the wall rectangles of the maze as (xmin, xmax, ymin, ymax)."""
    with open(path) as handle:
        lines = [line.rstrip("\n") for line in handle if line.strip()]

    width = (len(lines[0]) - 1) // 4
    height = (len(lines) - 1) // 2
    rects = []

    # text row 0 is the top of the maze, so y increases upwards through lines
    for row, line in enumerate(lines):
        y = (height - row / 2.0) * CELL
        if row % 2 == 0:  # horizontal walls
            for i in range(width):
                if line[4 * i + 1 : 4 * i + 4] == "---":
                    rects.append(
                        (i * CELL + WALL_HALF, (i + 1) * CELL - WALL_HALF, y - WALL_HALF, y + WALL_HALF)
                    )
        else:  # vertical walls
            for i in range(width + 1):
                if 4 * i < len(line) and line[4 * i] == "|":
                    rects.append(
                        (i * CELL - WALL_HALF, i * CELL + WALL_HALF, y - CELL / 2, y + CELL / 2)
                    )

    for i in range(width + 1):
        for j in range(height + 1):
            rects.append(
                (i * CELL - WALL_HALF, i * CELL + WALL_HALF, j * CELL - WALL_HALF, j * CELL + WALL_HALF)
            )

    return np.array(rects)


def footprint(x, y, yaw, samples=8):
    """World-frame points along the chassis outline for one pose."""
    forward = np.array([-math.sin(yaw), math.cos(yaw)])
    right = np.array([math.cos(yaw), math.sin(yaw)])
    corners = np.array([x, y]) + np.outer(CHASSIS[:, 0], right) + np.outer(CHASSIS[:, 1], forward)
    points = []
    for a, b in zip(corners, np.roll(corners, -1, axis=0)):
        for k in range(samples):
            points.append(a + (b - a) * k / samples)
    return np.array(points)


def clearance(points, rects):
    """Smallest distance from any point to any rectangle, negative if inside."""
    px = points[:, 0][:, None]
    py = points[:, 1][:, None]
    dx = np.maximum(rects[:, 0][None, :] - px, px - rects[:, 1][None, :])
    dy = np.maximum(rects[:, 2][None, :] - py, py - rects[:, 3][None, :])
    outside = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))
    inside = np.minimum(np.maximum(dx, dy), 0.0)
    return float(np.min(np.where((dx > 0) | (dy > 0), outside, inside)))


def nominal_heading(yaw):
    """Closest axis-aligned heading and the signed error from it, in radians."""
    quadrant = round(yaw / (math.pi / 2))
    target = quadrant * math.pi / 2
    return quadrant % 4, math.remainder(yaw - target, 2 * math.pi)


def grid_geometry(x, y, yaw):
    """Lateral offset from the corridor centre and travel along the cell."""
    quadrant, heading_error = nominal_heading(yaw)
    # heading 0 -> body forward is +y (quadrant 0 means yaw 0, forward = +y)
    forward = np.array([-math.sin(quadrant * math.pi / 2), math.cos(quadrant * math.pi / 2)])
    left = np.array([-forward[1], forward[0]])
    centre = np.array([(math.floor(x / CELL) + 0.5) * CELL, (math.floor(y / CELL) + 0.5) * CELL])
    delta = np.array([x, y]) - centre
    return float(np.dot(delta, left)), float(np.dot(delta, forward)), heading_error


def turns(run, maze):
    col = load(run)
    time = col["sim_time"]
    rects = parse_maze(maze)
    gate = np.abs(col["desired_angular_speed"]) > TURN_GATE
    sensors = np.maximum.reduce([col[f"wall_sensors_{k}"] for k in range(4)])

    edges = np.diff(gate.astype(int))
    starts = list(np.flatnonzero(edges == 1) + 1)
    ends = list(np.flatnonzero(edges == -1) + 1)
    if gate[0]:
        starts.insert(0, 0)
    if gate[-1]:
        ends.append(len(gate) - 1)

    rows = []
    for a, b in zip(starts, ends):
        if time[b] - time[a] < 0.05:
            continue
        if abs(np.sum(col["desired_angular_speed"][a:b] * np.diff(time[a : b + 1]))) < MIN_ANGLE:
            # A FollowWall steering correction, not an action: with the
            # saturation at 4.0 those exceed the gate but never a few degrees.
            continue
        dt = np.diff(time[a : b + 1])
        commanded = float(np.sum(col["desired_angular_speed"][a:b] * dt))
        executed = float(np.sum(np.unwrap(col["yaw"][a : b + 1])[1:] - np.unwrap(col["yaw"][a : b + 1])[:-1]))
        lateral, along, heading_error = grid_geometry(col["x"][a], col["y"][a], col["yaw"][a])

        omega = col["wz_body"][a:b]
        speed = col["v_forward"][a:b]
        good = np.abs(omega) > 1.0
        radius = float(np.median(speed[good] / omega[good])) if good.any() else float("nan")

        after = (time > time[b]) & (time <= time[b] + AFTER)
        during = slice(a, b + 1)
        min_during = min(
            clearance(footprint(col["x"][k], col["y"][k], col["yaw"][k]), rects)
            for k in range(a, b + 1, 4)
        )
        idx_after = np.flatnonzero(after)
        min_after = min(
            (clearance(footprint(col["x"][k], col["y"][k], col["yaw"][k]), rects) for k in idx_after[::4]),
            default=float("nan"),
        )

        rows.append(
            {
                "t0": round(float(time[a]), 3),
                "t1": round(float(time[b]), 3),
                "type": "spin" if abs(commanded) > 2.0 else "turn90",
                "x": round(float(col["x"][a]), 4),
                "y": round(float(col["y"][a]), 4),
                "lateral_mm": round(lateral * 1000, 1),
                "along_mm": round(along * 1000, 1),
                "heading_err_deg": round(math.degrees(heading_error), 2),
                "cmd_deg": round(math.degrees(commanded), 1),
                "exec_deg": round(math.degrees(executed), 1),
                "ratio": round(executed / commanded, 3) if commanded else None,
                "radius_mm": round(radius * 1000, 1) if radius == radius else None,
                "v_fwd": round(float(np.median(speed)), 3),
                "clear_turn_mm": round(min_during * 1000, 1),
                "clear_after_mm": round(min_after * 1000, 1),
                "contact": bool(col["base_ncon"][during].max() > 0 or col["base_ncon"][after].max() > 0),
                "sensor_sat": bool(sensors[after].max() > SENSOR_SATURATION) if after.any() else False,
            }
        )
    return rows


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    maze = "models/maze.txt"
    for a in sys.argv[1:]:
        if a.startswith("--maze="):
            maze = a.split("=", 1)[1]
    for run in args:
        rows = turns(run, maze)
        print(f"== {run}  ({len(rows)} turns)")
        header = (
            f"{'t0':>7} {'t1':>7} {'type':>7} {'lat':>7} {'along':>7} {'hdg':>7} "
            f"{'cmd':>7} {'exec':>7} {'ratio':>6} {'R':>7} {'v':>6} {'clr':>7} {'clrA':>7} {'hit':>4} {'sat':>4}"
        )
        print(header)
        for r in rows:
            print(
                f"{r['t0']:7.2f} {r['t1']:7.2f} {r['type']:>7} {r['lateral_mm']:7.1f} {r['along_mm']:7.1f} "
                f"{r['heading_err_deg']:7.2f} {r['cmd_deg']:7.1f} {r['exec_deg']:7.1f} "
                f"{(r['ratio'] if r['ratio'] is not None else float('nan')):6.3f} "
                f"{(r['radius_mm'] if r['radius_mm'] is not None else float('nan')):7.1f} {r['v_fwd']:6.3f} "
                f"{r['clear_turn_mm']:7.1f} {r['clear_after_mm']:7.1f} "
                f"{'Y' if r['contact'] else '.':>4} {'Y' if r['sensor_sat'] else '.':>4}"
            )
        turn90 = [r for r in rows if r["type"] == "turn90"]
        if turn90:
            print(
                json.dumps(
                    {
                        "turn90_count": len(turn90),
                        "ratio_mean": round(float(np.mean([r["ratio"] for r in turn90])), 3),
                        "radius_mean_mm": round(
                            float(np.mean([abs(r["radius_mm"]) for r in turn90 if r["radius_mm"]])), 1
                        ),
                        "along_mean_mm": round(float(np.mean([r["along_mm"] for r in turn90])), 1),
                        "lateral_abs_mean_mm": round(
                            float(np.mean([abs(r["lateral_mm"]) for r in turn90])), 1
                        ),
                        "min_clearance_mm": round(min(r["clear_turn_mm"] for r in rows), 1),
                        "turns_with_contact": sum(1 for r in rows if r["contact"]),
                    },
                    sort_keys=True,
                )
            )


if __name__ == "__main__":
    main()
