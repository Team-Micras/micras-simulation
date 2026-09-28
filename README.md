# micras-simulation

A simulator for the robots built on `micras_hal`. It runs a robot's firmware as
it is, its own `main`, its own proxies and its own configuration, over the host
backend of `micras_hal`, against a MuJoCo model of the robot and its arena. The
host backend and the models of the SPI chips live in
[micras-lib](https://github.com/Team-Micras/micras-lib), next to `micras_hal`
itself; a robot's project adds both. The micromouse, Micras, is the first robot
built on it: its firmware adds this repository as a submodule and holds its
target, its scenarios and its baselines in its own `sim/` folder, whose README
covers them.

Two ways to use it, both from the same binary:

- **Headless and deterministic**, for measuring. Every run writes a CSV of the
  whole state plus a metadata file, and two runs of the same binary with the
  same arguments produce byte-identical output.
- **Live**, for looking. A window with the robot, a control panel with the
  board's buttons and switches, plots of the firmware's own variables, and a
  WebSocket bridge that micras-monitor connects to as if it were the radio.

Linux only, x86-64. CMake downloads the pinned MuJoCo release (3.14.0) and
checks its SHA-256; point `MUJOCO_DIR` (a CMake cache variable or an environment
variable) at an installation to use that instead. The style targets want clang
22's `clang-format-22`, `clang-tidy-22`, `run-clang-tidy-22` and
`clang-apply-replacements-22`.

## Quick start

```bash
cmake --preset host-release
cmake --build --preset host-release
./build/host-release/targets/toy/toy_sim --scenario targets/toy/scenarios/drive.toml --out runs/drive
python3 tools/analyze.py runs/drive                               # report.json and the plots
./build/host-release/targets/toy/toy_sim --scenario targets/toy/scenarios/drive.toml --out runs/window --viewer
cmake --build --preset host --target micras_sim_check          # the simulator's gate
```

Everything is CMake: the presets configure, and the checks are targets over the
bash scripts of `scripts/`; `cmake --build --preset <preset> --target help` lists
them. A run calls the binary directly (see [Running](#running)).

| Preset | Build | For |
|---|---|---|
| `host` | Debug, with the address and undefined behavior sanitizers | the unit tests and the checks |
| `host-release` | RelWithDebInfo | runs and baselines |
| `host-ci` | `host`, with `MICRAS_SIM_WERROR=ON` | the CI |

Each builds in `build/<preset>/`, with GCC 15 (`gcc-15`/`g++-15`) and Ninja, since
another compiler may move the last bits of a run.

## Running

```bash
./build/host-release/targets/toy/toy_sim --scenario targets/toy/scenarios/drive.toml --out runs/toy
```

| Flag | Meaning |
|---|---|
| `--out <dir>` | required, output directory, created if missing |
| `--scenario <toml>` | the scenario to play; without one the robot is switched on and left alone |
| `--maze <name>\|<txt>` | the arena: a maze of `engine/arenas/maze/mazes/` by name, or a file (default `maze1`) |
| `--seconds <float>` | simulated seconds, overriding the scenario's |
| `--ticks <int>` | exact tick count, overrides `--seconds` |
| `--seed <int>` | seed of every noise stream (default the scenario's, else 1) |
| `--ideal` | no noise at all |
| `--record-every <ticks>` | write one CSV row every N ticks (default 1); events are still detected every tick |
| `--viewer` | open a window on the run |
| `--viewer-camera <name>\|free` | starting camera |
| `--viewer-fps <int>` | redraws per simulated second (default 30) |
| `--viewer-size WxH` | window size, default 1200x900 |
| `--monitor` | open the micras-monitor bridge |
| `--monitor-port <int>` | port it listens on (default 8080) |
| `--video <out.mp4>` | record an offscreen mp4 of the run |
| `--video-fps <int>` | frame rate, default 30 |
| `--video-camera <name>` | `side tracking` (the robot's default), `top tracking`, `onboard` (the robot's own view), `maze_top`, or `free` |
| `--video-size WxH` | frame size, default 1280x720 |
| `--video-trail` | draw the path the robot has traveled behind it |

A target adds options of its own (the toy's `--turn-angle`), which the usage
message lists.

A window, a recording and a bridge are all optional and all off by default, and
none of them changes what the run produces. `micras_sim_check` proves it for
each on the toy target, by running the same scenario with and without and
comparing the two runs byte for byte.

## Scenarios

```toml
# targets/toy/scenarios/drive.toml
robot = "toy"
arena = "maze1"
seconds = 10
seed = 1

[start]
x = 0.09
y = 0.05
yaw_deg = 90.0

[[events]]
at = 0.2
press = "button"
for = 0.1

[[events]]
at = 3.2
send = "stop"

[stop]
when = "state"
equals = ["STOP"]
after = 0.5
```

An event can `press` an input for a time, `set` it, or `send` a link command
(the target names the inputs and the commands it knows). An event
with `when` and `equals` waits, from its `at` on, for the named firmware variable
to take one of those values, and fires on the first tick it does. The run stops
at `seconds`, or earlier when the stop condition's variable takes one of the
named values after `after` seconds, for the `count`-th time (default 1), or at
once when it takes one of the `abort` values.

## Output

`<out>/data.csv` is one row per tick (125 us) or per `--record-every` ticks: the
body pose and velocity, the robot's ground truth (wheels, motors, contacts,
solver), every monitoring variable the firmware publishes, and what each
simulated device produced.

`<out>/meta.json` is the provenance and the outcome of the run: firmware commit,
hashes of `robot.toml` and of the composed model, MuJoCo and compiler versions,
the arguments, seed, tick counts, when the scenario stopped the run, the board's
counters (`unbound_ports`, `watchdog_expiries`, `emergency_stops`,
`serial_dropped_bytes`, `bridge_dropped_frames`), and the `events`: every state
change and collision, detected at full rate. Nothing measured off the wall clock
goes in it; the wall time is printed on stdout only.

`interactive` is the honest flag: it is `true` only if a human actually touched
a control the firmware can see or a monitor sent it something. A window that was
merely open leaves it `false`, and the run is still reproducible from `args`.

`<out>/model.xml` is the MJCF the run simulated: the robot generated from its
`robot.toml`, composed with the arena.

## Analysis

```bash
python3 tools/analyze.py runs/drive
```

Writes `report.json` and plots into the run directory: speeds, contacts,
attitude, actuators, wheels, a top-down trajectory over the maze with the goal
cells shaded and the firmware's pose estimate dashed, and whatever the target's
plugin adds. The
report covers warnings, non-finite samples, airborne fraction, slip,
penetration, chassis contact, the state timeline, collisions, whether and when
the goal was reached, the pose estimate's error against the ground truth, the
tracking error and voltage saturation. Only numpy and matplotlib are needed.

`tools/analyze.py` knows only the engine's columns and needs only numpy and
matplotlib; what a robot's own columns mean comes from its plugin, `tools/analysis.py` in the target's folder, which
`meta.json` records as `target_dir`. `--plugin` or `$MICRAS_SIM_PLUGIN` name
another; `tools/baseline.py` finds it the same way. `baseline.py compare --exact`
is the byte identity check, and `tools/compare_run.py` compares two runs of one
build, ignoring the fields that name paths.

## Watching a run

```bash
./build/host-release/targets/toy/toy_sim --scenario targets/toy/scenarios/drive.toml --out runs/window --viewer
```

| Key | Action |
|---|---|
| space | pause and resume |
| right | one tick while paused |
| tab | cycle cameras |
| c / f / t / r | contact points, contact forces, transparency, rangefinder rays |
| esc | quit |

Left drag orbits, right drag pans, middle drag and scroll zoom, and ctrl with
left drag pushes the robot around. The panel holds the board itself: the
button, the DIP switches, the lamps the firmware drives, the readouts and plots
of its variables, plus pause, step, a speed limiter and quit. Touching any board
control hands the run over: the scenario stops driving the inputs, and the run
is marked `interactive`.

## micras-monitor

```bash
./build/host-release/targets/toy/toy_sim --scenario targets/toy/scenarios/idle.toml --seconds 600 --out runs/link --monitor
```

The bridge puts the firmware's radio on `ws://localhost:8080` and carries raw
bytes both ways, with no framing of its own: micras-monitor speaks the
firmware's protocol to it exactly as it would over the air. The link is as fast
as the radio's baud rate, not faster.

## The gate

```bash
cmake --build --preset host --target micras_sim_check
```

The simulator's gate, on the toy target (`targets/toy/`): builds everything, runs
every unit test through CTest, configures and builds every optional part off to
keep it buildable (`micras_sim_check_options`), checks that nothing outside
`targets/` names a robot (`micras_sim_check_generic`), and proves on the toy that
a run is reproducible and that a window, a video and a bridge nobody connects to
change nothing, and compares the toy's runs with its baseline
(`micras_sim_toy_check`). The window runs under `xvfb-run`, or on the display;
`MICRAS_SKIP_VIEWER=1` skips it, even where `xvfb-run` or a display exists.

Baselines live in `targets/<robot>/baselines/`; `CLAUDE.md` explains what they
hold and the rules around them. `micras_sim_toy_record_baseline` records the toy's
version `MICRAS_SIM_TOY_BASELINE` names and refuses to overwrite one: bump it
instead, and nothing is ever lost. With `-DMICRAS_SIM_EXACT=ON` a baseline
comparison also fails when a checked run's `data.csv` is not byte identical to the
recorded one: the check for a change that must not move a byte, such as a
refactoring. A project that adds the simulator reads the same option for its own
baselines.

`micras_sim_format`, `micras_sim_format_check`, `micras_sim_lint` and
`micras_sim_lint_fix` are the style targets; they use the configuration shared
with micras-lib and the firmware (`.clang-format`, `.clang-tidy`).

## A robot target

A robot is a folder of `targets/` (or a directory of another project that adds
this one) with a class implementing `micras::sim::Target` and a `main` that is one
call to `micras::sim::run`. The target gives its name, loop period, robot file,
ground truth columns, options and firmware program, and `wire()` builds its
devices and returns what it adds to the run. The program is called with the
`FirmwareThread` it runs on, so a program that drives the handover itself calls
`yield_tick()` on it.

The public headers, which a target may include and which change only with notice:

| Header | What |
|---|---|
| `micras/sim/app/target.hpp` | `Target`, the interface |
| `micras/sim/app/wiring.hpp` | `Wiring`, `WorldInfo`, `CliOption` |
| `micras/sim/app/application.hpp` | `micras::sim::run` |
| `micras/sim/view/panel_spec.hpp` | the panel and the overlay |
| `micras/sim/core/run_context.hpp` | the world, the clock, the serial bus, the noise and the devices of a run |
| `micras/sim/core/firmware_thread.hpp` | the firmware thread and its handover |
| `micras/sim/core/variable_source.hpp` | the firmware variables the panel, the overlay and the scenarios read |
| `micras/sim/core/span_at.hpp` | checked indexing of a `std::span` |
| `micras/sim/recording/column_source.hpp`, `csv_writer.hpp` | CSV columns and the writer they fill |
| `micras/sim/recording/ground_truth.hpp`, `run_metadata.hpp` | ground truth, `meta.json` counters |
| `micras/sim/robot/robot_description.hpp` | the robot's `robot.toml` |
| `micras/sim/robot/robot_model.hpp` | `RobotModelNames`, the names the generated model gives the robot's parts |
| `micras/sim/scenario/scenario.hpp` | `ScenarioHooks`: the inputs, messages and state names a scenario uses |
| `micras/sim/devices/*.hpp` | the devices a target builds |

A target over `micras_hal` links micras-lib's host backend and defines the board
target it asks for, the Cube layer its firmware includes, written by hand;
micras-lib's README describes that contract. A project that adds this repository
with `add_subdirectory` gets the libraries (`micras::sim_app` and the rest) and
`MICRAS_SIM_TOOLS_DIR` and `MICRAS_SIM_MAZES_DIR`, and keeps its own style and
check targets: the simulator defines its own only when it is the top-level
project.

The robot is micromouse-shaped and runs in a maze: its `robot.toml` has every
section of the schema (chassis, wheels, drive, encoders, IMU, fan, battery, link,
wall sensors), and the arena is one of the mazes.

## The toy target

`targets/toy/` is the simulator's reference: the smallest robot that uses every
feature of the engine and of the `Target` interface, with no HAL and nothing of a
real robot. Its `robot.toml` is the engine tests' tiny robot with a second wall
sensor looking ahead. Its program, which yields on its own firmware thread once
per 1 ms loop, waits; a press of the button or `go` over the link starts it; it
drives with the fan on, counting its encoders, until the front wall sensor sees a
wall, turns right by `--turn-angle` degrees (90 by default) integrating its gyro,
drives on, and stops at `stop` over the link, which it answers. Its variables are
its state, odometry, heading, front reading, pack voltage and motor currents; its
panel has the button, a lamp per state and the pack voltage; its scenarios are
`idle` and `drive`.

## CI

`.github/workflows/ci.yaml` runs on every push and pull request, in the image of
`.docker/Dockerfile` (Ubuntu 26.04 with GCC 15, clang 22, CMake and Python; its
host stage is the one micras-lib and the firmware use too): the `host-ci` build,
CTest, `micras_sim_format_check`, `micras_sim_lint` and `micras_sim_check`. The
baselines are compared within their tolerances there: a hash belongs to the
machine that recorded it.

## Layout

```
engine/          world, clock, run loop, devices, robot description, scenarios, recording
engine/arenas/   the maze arena and its mazes
view/            window, panel, overlay, video
bridge/          WebSocket server and monitor bridge
app/             CLI and application wiring, the Target interface
tests/           the engine's unit tests, on a tiny robot of their own
tools/           analysis, baselines, run health and byte comparison
scripts/         the checks' scripts, which the CMake targets call
cmake/           dependencies, warnings, style and check targets
targets/toy/     the reference target: toy robot, scenarios, baseline
```

`CLAUDE.md` has the architecture and the invariants that hold the determinism
together.
