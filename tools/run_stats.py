#!/usr/bin/env python3
"""Compact per-run metrics for the harness iteration table.

Reports, for one or more run directories: goal_reached / goal_time and the
first ERROR time (from report.json, so analyze.py must have run first), the
odometry pose error, the post-correction events detected as jumps of the
believed pose, the base-geom contact fraction and the wheel slip.

A post correction is a tick where the believed pose jumps by more than
``--jump`` metres in a single tick; ``FollowWall::clear_position_error`` is the
only thing that writes the pose outside the integration, so every such jump is
one correction. For each one the script prints the true cell advance, that is
the ground-truth travel along the heading since the last cell boundary, modulo
the cell size, which is the quantity ``post_reference`` is supposed to match.
"""

import json
import math
import sys

import numpy as np

CELL = 0.18


def load(run):
    with open(f"{run}/data.csv") as handle:
        names = handle.readline().strip().split(",")
    data = np.loadtxt(f"{run}/data.csv", delimiter=",", skiprows=1)
    return {name: data[:, i] for i, name in enumerate(names)}


def stats(run, jump):
    col = load(run)
    time = col["sim_time"]
    true_x, true_y, yaw = col["x"], col["y"], col["yaw"]
    bel_x, bel_y = col["odometry_state_x"], col["odometry_state_y"]
    error = np.hypot(bel_x - true_x, bel_y - true_y)

    step = np.hypot(np.diff(bel_x), np.diff(bel_y))
    events = np.flatnonzero(step > jump) + 1

    posts = []
    for i in events:
        # Travel along the heading inside the current cell, which is what the
        # post reference is measured against.
        advance = (true_x[i] * math.cos(yaw[i]) + true_y[i] * math.sin(yaw[i])) % CELL
        posts.append(
            {
                "t": round(float(time[i]), 3),
                "advance_mm": round(float(advance) * 1000, 1),
                "error_before_mm": round(float(error[i - 1]) * 1000, 1),
                "error_after_mm": round(float(error[i]) * 1000, 1),
            }
        )

    worse = sum(1 for p in posts if p["error_after_mm"] > p["error_before_mm"])

    with open(f"{run}/report.json") as handle:
        report = json.load(handle)
    maze = report.get("maze", {})
    fsm = report.get("fsm_state", [])
    error_time = next((t["time"] for t in fsm if t.get("state") == 6), None)

    wheels = {
        side: {
            "airborne_fraction": round(report[f"{side}_wheel"]["airborne_fraction"], 4),
            "max_abs_slip": round(report[f"{side}_wheel"]["max_abs_slip"], 3),
        }
        for side in ("left", "right")
    }
    return {
        "run": run,
        "goal_reached": maze.get("goal_reached"),
        "goal_time": maze.get("goal_time"),
        "error_time": error_time,
        "odometry_error_max_mm": round(float(error.max()) * 1000, 1),
        "odometry_error_final_mm": round(float(error[-1]) * 1000, 1),
        "post_corrections": len(posts),
        "post_corrections_worse": worse,
        "post_advance_mm": [p["advance_mm"] for p in posts],
        "posts": posts,
        "base_contact_fraction": round(float((col["base_ncon"] > 0).mean()), 4),
        "wheels": wheels,
    }


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    jump = 0.003
    for a in sys.argv[1:]:
        if a.startswith("--jump="):
            jump = float(a.split("=", 1)[1])
    for run in args:
        print(json.dumps(stats(run, jump), indent=2))


if __name__ == "__main__":
    main()
