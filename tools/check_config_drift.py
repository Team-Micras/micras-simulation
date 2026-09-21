#!/usr/bin/env python3
"""Compare the harness config headers against the firmware ones, field by field.

``config/constants.hpp`` and ``config/target.hpp`` are copies of
``MicrasFirmware/config/*.hpp`` that the harness is allowed to diverge from --
but only in the ways that are written down. This script re-derives the diff on
every run so that an unintended edit (or a firmware submodule bump) cannot slip
through unnoticed.

Parsing is a deliberately small brace scanner over designated initialisers
(``.name = value``) and ``constexpr T name{value}`` declarations. It is not a
C++ parser: anything it cannot evaluate to a number is skipped rather than
guessed at.

Exit code 0 when every difference is allowlisted below, 1 otherwise.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# --------------------------------------------------------------------------
# Allowlists
# --------------------------------------------------------------------------

#: constants.hpp divergences documented in the file-level @note of
#: config/constants.hpp. Keys are dotted paths, values are the reason.
CONSTANTS_ALLOWLIST: dict[str, str] = {
    "crash_acceleration": "1e6 disables crash detection; simulated contacts trip the real 35 m/s^2 threshold",
    "speed_controller_config.linear_pid.kp": "firmware kp rescaled to sim command units by the linear feed-forward ratio 102 / 12.706",
    "speed_controller_config.linear_pid.ki": "10x the firmware ki so the integral closes inside one action, see the constants.hpp @note",
    "speed_controller_config.angular_pid.kp": "firmware kp rescaled to sim command units by the angular feed-forward ratio 4.9 / 0.971",
    "speed_controller_config.angular_pid.ki": "10x the firmware ki so the integral closes inside a 1.6 s pivot, see the constants.hpp @note",
    "speed_controller_config.left_feed_forward.linear_speed": "calibrated to the MuJoCo motor (1.01 m/s no load at ctrl=100)",
    "speed_controller_config.right_feed_forward.linear_speed": "calibrated to the MuJoCo motor (1.01 m/s no load at ctrl=100)",
    "follow_wall_config.pid.kp": "simulated rangefinders report metres, not raw ADC counts",
    "follow_wall_config.pid.kd": "damping for the metre-scale rangefinder error, see the constants.hpp @note",
    "follow_wall_config.pid.saturation": "the lateral loop must stay proportional out to a full half corridor, see the constants.hpp @note",
    "follow_wall_config.post_threshold": "simulated rangefinders report metres, not raw ADC counts",
    "odometry_config.linear_cutoff_frequency": "simulated encoders carry no noise to filter out",
}

#: Further constants.hpp divergences that exist today but are NOT described in
#: the config/constants.hpp @note. They are listed explicitly so the check stays
#: actionable instead of red by default; each one still needs a decision:
#: either document it as intentional in the @note and move it above, or restore
#: the firmware value. Do not extend this block -- new drift belongs in a bug
#: report, not here.
CONSTANTS_PENDING_REVIEW: dict[str, str] = {
    "wall_thickness": "geometry: matches models/maze.xml, not the real maze",
    "start_offset": "geometry: matches models/robot_legacy.xml start pose",
    "max_angular_acceleration": "ROS-sim motion budget",
    "action_queuer_config.curve_safety_margin": "ROS-sim motion budget",
    "action_queuer_config.exploring.max_linear_speed": "ROS-sim motion budget",
    "action_queuer_config.exploring.max_linear_acceleration": "ROS-sim motion budget",
    "action_queuer_config.exploring.max_linear_deceleration": "ROS-sim motion budget",
    "action_queuer_config.exploring.max_centrifugal_acceleration": "sets the 45 mm exploring curve radius that clears the 167.4 mm corridor",
    "action_queuer_config.exploring.max_angular_acceleration": "ROS-sim motion budget",
    "action_queuer_config.solving.max_linear_speed": "ROS-sim motion budget",
    "action_queuer_config.solving.max_linear_acceleration": "ROS-sim motion budget",
    "action_queuer_config.solving.max_linear_deceleration": "ROS-sim motion budget",
    "action_queuer_config.solving.max_centrifugal_acceleration": "ROS-sim motion budget",
    "action_queuer_config.solving.max_angular_acceleration": "ROS-sim motion budget",
    "follow_wall_config.max_angular_acceleration": "follows max_angular_acceleration above",
    "follow_wall_config.post_clearance": "follow-wall retuned for the metre-scale rangefinders",
    "odometry_config.wheel_radius": "geometry: matches the wheel geom in models/robot_legacy.xml",
    "speed_controller_config.linear_pid.saturation": "paired with the retuned PID gains",
    "speed_controller_config.angular_pid.saturation": "paired with the retuned PID gains",
    "speed_controller_config.left_feed_forward.linear_acceleration": "feed forward calibrated to the MuJoCo motor",
    "speed_controller_config.left_feed_forward.angular_speed": "feed forward calibrated to the MuJoCo motor",
    "speed_controller_config.left_feed_forward.angular_acceleration": "feed forward calibrated to the MuJoCo motor",
    "speed_controller_config.right_feed_forward.linear_acceleration": "feed forward calibrated to the MuJoCo motor",
    "speed_controller_config.right_feed_forward.angular_speed": "feed forward calibrated to the MuJoCo motor",
    "speed_controller_config.right_feed_forward.angular_acceleration": "feed forward calibrated to the MuJoCo motor",
}

#: Field names compared between the two target.hpp files. The two headers are
#: structurally unrelated (ours names MuJoCo entities, the firmware's names STM32
#: peripherals), so only these numeric leaves are meaningful.
TARGET_FIELDS: tuple[str, ...] = (
    "long_press_delay",
    "extra_long_press_delay",
    "base_readings.0",
    "base_readings.1",
    "base_readings.2",
    "base_readings.3",
    "uncertainty",
    "max_sensor_reading",
    "min_sensor_reading",
    "max_sensor_distance",
    "filter_cutoff",
    "max_voltage",
    "deadzone",
    "max_stopped_command",
    "max_acceleration",
)

#: target.hpp divergences that are intentional.
TARGET_ALLOWLIST: dict[str, str] = {
    "wall_sensors_config.base_readings.0": "MuJoCo rangefinders report metres, the real sensors report normalised ADC counts",
    "wall_sensors_config.base_readings.1": "MuJoCo rangefinders report metres, the real sensors report normalised ADC counts",
    "wall_sensors_config.base_readings.2": "MuJoCo rangefinders report metres, the real sensors report normalised ADC counts",
    "wall_sensors_config.base_readings.3": "MuJoCo rangefinders report metres, the real sensors report normalised ADC counts",
    "wall_sensors_config.filter_cutoff": "noise free simulated rangefinders need much less filtering",
    "battery_config.max_voltage": "the harness battery is a constant, the firmware derives it from a divider",
}

# --------------------------------------------------------------------------
# Parsing
# --------------------------------------------------------------------------

COMMENT_BLOCK = re.compile(r"/\*.*?\*/", re.S)
COMMENT_LINE = re.compile(r"//[^\n]*")
PREPROCESSOR = re.compile(r"^\s*#[^\n]*$", re.M)
ASSIGNMENT = re.compile(r"^\.?([A-Za-z_]\w*)\s*=\s*(.+)$", re.S)
DECLARATION = re.compile(r"([A-Za-z_]\w*)\s*=?\s*$")
NUMERIC_SUFFIX = re.compile(r"(?<=[\d.])[FfUuLl]+\b")


class Node:
    """One brace-delimited initialiser block."""

    def __init__(self, label: str | None):
        self.label = label
        self.entries: list[tuple[str | None, str | Node]] = []


def strip_comments(text: str) -> str:
    text = COMMENT_BLOCK.sub(" ", text)
    text = COMMENT_LINE.sub(" ", text)
    return PREPROCESSOR.sub(" ", text)


def parse(text: str) -> Node:
    """Turn a config header into a tree of initialiser blocks."""
    root = Node(None)
    stack = [root]
    buffer = ""

    def flush() -> None:
        nonlocal buffer
        item = buffer.strip()
        buffer = ""

        if not item:
            return

        match = ASSIGNMENT.match(item)

        if match:
            stack[-1].entries.append((match.group(1), match.group(2).strip()))
        else:
            stack[-1].entries.append((None, item))

    for character in strip_comments(text):
        if character == "{":
            match = ASSIGNMENT.match(buffer.strip()) or DECLARATION.search(buffer.strip())
            label = match.group(1) if match else None
            buffer = ""
            node = Node(label)
            stack[-1].entries.append((label, node))
            stack.append(node)
        elif character == "}":
            flush()

            if len(stack) > 1:
                stack.pop()
        elif character in ",;":
            flush()
        else:
            buffer += character

    flush()
    return root


def evaluate(expression: str, symbols: dict[str, float]) -> float | None:
    """Evaluate a numeric literal or a small arithmetic expression, else None."""
    cleaned = NUMERIC_SUFFIX.sub("", expression).strip()

    if not cleaned or not re.fullmatch(r"[\w\s.+\-*/()]+", cleaned):
        return None

    try:
        value = eval(cleaned, {"__builtins__": {}}, dict(symbols))  # noqa: S307
    except Exception:
        return None

    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def flatten(node: Node, prefix: str, symbols: dict[str, float], out: dict[str, float]) -> None:
    anonymous = 0

    for name, value in node.entries:
        if isinstance(value, Node):
            if value.label:
                key = f"{prefix}.{value.label}" if prefix else value.label
            else:
                key = f"{prefix}.{anonymous}" if prefix else str(anonymous)
                anonymous += 1

            # constexpr float cell_size{0.18}: a block with one anonymous scalar.
            if len(value.entries) == 1 and value.entries[0][0] is None and isinstance(value.entries[0][1], str):
                number = evaluate(value.entries[0][1], symbols)

                if number is not None:
                    out[key] = number
                    continue

            flatten(value, key, symbols, out)
            continue

        if name is None:
            key = f"{prefix}.{anonymous}" if prefix else str(anonymous)
            anonymous += 1
        else:
            key = f"{prefix}.{name}" if prefix else name

        number = evaluate(value, symbols)

        if number is not None:
            out[key] = number


#: The whole file lives inside `namespace micras`, which the scanner sees as an
#: outer block; its label carries no information.
NAMESPACE_PREFIX = "micras."


def strip_namespace(mapping: dict[str, float]) -> dict[str, float]:
    return {
        key[len(NAMESPACE_PREFIX) :] if key.startswith(NAMESPACE_PREFIX) else key: value
        for key, value in mapping.items()
    }


def fields(path: Path) -> dict[str, float]:
    """Parse a config header into a dotted-path -> number mapping."""
    tree = parse(path.read_text())

    # Two passes: the first collects the top level constants so that the second
    # can evaluate expressions such as `0.44F * cell_size`.
    scalars: dict[str, float] = {}
    flatten(tree, "", {}, scalars)
    symbols = {key: value for key, value in strip_namespace(scalars).items() if "." not in key}

    resolved: dict[str, float] = {}
    flatten(tree, "", symbols, resolved)
    return strip_namespace(resolved)


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------


def compare(
    ours: dict[str, float],
    theirs: dict[str, float],
    allowlist: dict[str, str],
    keep: tuple[str, ...] | None,
) -> list[tuple[str, float, float, str | None]]:
    """Return (path, ours, firmware, reason) for every field that differs."""
    differences = []

    for key, value in sorted(ours.items()):
        if key not in theirs:
            continue

        if keep is not None and not any(key == leaf or key.endswith("." + leaf) for leaf in keep):
            continue

        if value == theirs[key]:
            continue

        differences.append((key, value, theirs[key], allowlist.get(key)))

    return differences


def report(title: str, differences: list[tuple[str, float, float, str | None]], pending: dict[str, str]) -> int:
    print(f"== {title}")

    if not differences:
        print("   no differences")
        return 0

    width = max(len(key) for key, _, _, _ in differences)
    unlisted = 0
    todo = 0

    for key, ours, theirs, reason in differences:
        if reason is None:
            status = "NEW "
            unlisted += 1
        elif key in pending:
            status = "TODO"
            todo += 1
        else:
            status = "ok  "

        print(f"   {status} {key:<{width}}  harness {ours:<12g} firmware {theirs:<12g}  {reason or 'NOT ALLOWLISTED'}")

    print(f"   {len(differences)} difference(s), {todo} pending review, {unlisted} not allowlisted")
    return unlisted


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent.parent)
    arguments = parser.parse_args()

    root: Path = arguments.root
    firmware = root / "MicrasFirmware" / "config"

    constants = compare(
        fields(root / "config" / "constants.hpp"),
        fields(firmware / "constants.hpp"),
        CONSTANTS_ALLOWLIST | CONSTANTS_PENDING_REVIEW,
        None,
    )
    target = compare(
        fields(root / "config" / "target.hpp"),
        fields(firmware / "target.hpp"),
        TARGET_ALLOWLIST,
        TARGET_FIELDS,
    )

    unlisted = report("constants.hpp", constants, CONSTANTS_PENDING_REVIEW) + report("target.hpp", target, {})

    if unlisted:
        print(f"\nFAIL: {unlisted} unlisted difference(s); document them or restore the firmware value")
        return 1

    pending = sum(1 for key, _, _, _ in constants if key in CONSTANTS_PENDING_REVIEW)

    if pending:
        print(f"\nOK: every difference is allowlisted ({pending} still marked TODO, pending review)")
    else:
        print("\nOK: every difference is allowlisted")
    return 0


if __name__ == "__main__":
    sys.exit(main())
