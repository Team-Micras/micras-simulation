# Micras baselines

Each version holds one `summary.json` per checked run (`CLAUDE.md` says what a summary is and why a
version is never recorded again). `MICRAS_SIM_BASELINE` in `targets/micras/CMakeLists.txt` names the
version the gate compares with; the others stay as they were recorded.

| Version | Firmware | Runs | What changed |
|---|---|---|---|
| v1 | `4deedba` | idle, explore | the navigation of the firmware after the fourth review of #54 |
| v2 | `4dac6e2` | idle, explore | the firmware's own SPI chip proxies over the chip models |
| v3 | `af5f629` | idle, explore, explore_stop | the second version of the link protocol, and an exploration stopped over the radio |
| v4 | `92ddffa` | v3 and every stop scenario | a stop brakes a moving robot to rest in BRAKE before it is idle |
| v5 | `8ae3bbd` | v4, explore_stop_twice, explore_stop_press, solve_stop_turn, solve_stop_line | the review of the brake: a braked curve runs right to its end with the sharpest bending of each step, the gyroscope calibration holds the angle it braked to until the robot is at rest, a second stop during the brake cuts the motors, a press of the button during the brake is forgotten, and the link clock is read right after a save |
| v6 | `b809784` | v5, explore_stop_twice_early, solve_stop_twice | a second stop during the brake shorts the motors until the robot has settled instead of letting it coast, and a brake that shorts the motors, the identification's too, ends only once the robot has settled |
| v7 | `b73fab1` | v6 | a second stop during the brake ramps the speeds of the wheels down to rest at the traction the runs brake with, through the loop on the speeds alone, before it shorts the motors; the gyroscope calibration brake also ends only once the robot has settled; and the estimate of the pose starts at the start of the maze once the initialization ends, not at its corner |
| v8 | `792ccbc` | v7, explore_stop_twice_repeated, identify_stop_twice, solve_stop_line_twice | once the wheels brake, a further stop only confirms and neither restarts the ramp nor clears the time the robot has been at rest |

v4 was recorded with `92ddffa` while the submodule had moved on to `5ee37e2`, whose link clock
changes no run; v5 is recorded with the firmware the submodule points at.
