# micras-simulation

A simulator for the robots built on `micras_hal`. It runs a robot's firmware as
it is, its own `main`, its own proxies and its own configuration, over a host
implementation of `micras_hal`, against a MuJoCo model of the robot and its
arena. The micromouse, Micras, is the first robot target.

Two ways to use it, both from the same binary:

- **Headless and deterministic**, for measuring. Every run writes a CSV of the
  whole state plus a metadata file, and two runs of the same binary with the
  same arguments produce byte-identical output.
- **Live**, for looking. A window with the robot, a control panel with the
  board's buttons and switches, plots of the firmware's own variables, and a
  WebSocket bridge that micras-monitor connects to as if it were the radio.

Linux only, x86-64. CMake downloads the pinned MuJoCo release (3.14.0) and checks
its SHA-256; point `MUJOCO_DIR` (a CMake cache variable or an environment
variable) at an installation to use that instead. Clone with the firmware:
`git clone --recurse-submodules`, or `git submodule update --init` afterwards.
The style targets want clang 22's `clang-format-22`, `clang-tidy-22`,
`run-clang-tidy-22` and `clang-apply-replacements-22`.

## Quick start

```bash
cmake --preset host-release
cmake --build --preset host-release
cmake --build --preset host-release --target sim_run_explore   # 30 s of an exploration -> build/host-release/targets/micras/runs/explore
python3 tools/analyze.py build/host-release/targets/micras/runs/explore   # report.json and the plots
cmake --build --preset host-release --target sim_watch         # an exploration in a window
cmake --build --preset host-release --target sim_serve         # an exploration open to micras-monitor
cmake --build --preset host --target micras_sim_check          # the simulator's gate
```

Everything is CMake: the presets configure, and the recipes are targets over the
bash scripts of `scripts/` (the simulator's) and `targets/<robot>/scripts/` (a
robot's); `cmake --build --preset <preset> --target help` lists them. An arbitrary
run calls the binary directly (see [Running](#running)).

| Preset | Build | For |
|---|---|---|
| `host` | Debug, with the address and undefined behavior sanitizers | the unit tests and the checks |
| `host-release` | RelWithDebInfo | runs, the contest and baselines |
| `host-ci` | `host`, with `MICRAS_SIM_WERROR=ON` | the CI |

Each builds in `build/<preset>/`, with GCC 15 (`gcc-15`/`g++-15`) and Ninja, since
another compiler may move the last bits of a run.

## Running

```bash
./build/host-release/targets/micras/micras_sim --scenario targets/micras/scenarios/explore.toml --out runs/x
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
| `--monitor` | open the micras-monitor bridge, and pace the run to real time |
| `--monitor-port <int>` | port it listens on (default 8080) |
| `--video <out.mp4>` | record an offscreen mp4 of the run |
| `--video-fps <int>` | frame rate, default 30 |
| `--video-camera <name>` | `side tracking` (the robot's default), `top tracking`, `onboard` (the robot's own view), `maze_top`, or `free` |
| `--video-size WxH` | frame size, default 1280x720 |
| `--video-trail` | draw the path the robot has traveled behind it |

Micras adds one option of its own:

| Flag | Meaning |
|---|---|
| `--flash <file>` | load the flash from the file before the run and save it back after, so a map the firmware saved survives into the next run |

A window, a recording and a bridge are all optional and all off by default, and
none of them changes what the run produces. `micras_sim_check` proves it for
each on the toy target, by running the same scenario with and without and
comparing the two runs byte for byte.

## Scenarios

```toml
# targets/micras/scenarios/explore.toml
robot = "micras"
arena = "maze1"
seconds = 180
seed = 1

[[events]]
at = 0.5
press = "button"
for = 0.25

[stop]
when = "state"
equals = ["IDLE", "ERROR"]
after = 5.0
```

An event can `press` an input for a time, `set` it, or `send` a link command
(`explore`, `solve`, `calibrate`, `save`, `reset`, `stop`, `leave_error`). An event
with `when` and `equals` waits, from its `at` on, for the named firmware variable
to take one of those values, and fires on the first tick it does. The run stops
at `seconds`, or earlier when the stop condition's variable takes one of the
named values after `after` seconds, for the `count`-th time (default 1), or at
once when it takes one of the `abort` values.

The Micras scenarios:

| Scenario | What it plays |
|---|---|
| `idle` | the robot switched on and left alone |
| `explore` | an exploration started by the button |
| `explore_link` | an exploration started over the radio |
| `explore_stop` | the same, stopped over the radio a tenth of a second into the run |
| `explore_stop_straight` | an exploration stopped on a straight at its top speed, 1 m/s |
| `explore_stop_turn` | an exploration stopped halfway through a search turn, which it brakes along |
| `explore_stop_spin` | an exploration stopped as it turns around in place at a dead end |
| `explore_stop_save` | an exploration stopped while it saves the map at the goal: the stop waits for the save |
| `explore_stop_twice` | an exploration stopped on a straight at 1 m/s and stopped again 100 ms into the brake: the brake is dropped and the wheels are ramped to rest, then the motors are shorted until it has settled |
| `explore_stop_twice_early` | the same, stopped again 50 ms into the brake, at 0.76 m/s: the wheels stop it well before the wall a coasting robot hits |
| `explore_stop_twice_repeated` | the same, then a stop every 10 ms for a second: once the wheels brake a stop only confirms, so the robot is idle as soon as after a single second stop |
| `explore_stop_press` | an exploration stopped on a straight at 1 m/s with a press of the button during the brake, which is forgotten |
| `calibrate_stop` | the calibration of the wall sensors, stopped while it measures the side sensors |
| `gyroscope_stop` | the calibration of the gyroscope, stopped halfway through its turn at 12 rad/s |
| `identify_stop` | the identification of the drive train, stopped as it drives forward at 1.5 m/s |
| `identify_stop_twice` | the same, stopped again 50 ms into the brake: the wheels are ramped to rest before the motors are shorted |
| `solve_unmapped` | a fastest run with no map: stopped while it plans, then left in ERROR and taken out of it |
| `solve_stop` | the whole contest, stopped in the fastest run on a straight at 3 m/s |
| `solve_stop_twice` | the same, stopped again 50 ms into the brake: the wheels are ramped to rest about as short as the brake along the route, clear of the walls |
| `solve_stop_turn` | the same, stopped halfway through a turn at 2.6 m/s, braked along it and on along the route |
| `solve_stop_line` | the whole contest on the racing line, stopped as the line bends at 3 m/s |
| `solve_stop_line_twice` | the same, stopped again 50 ms into the brake: the wheels are ramped to rest from the speed the bend left |
| `explore_solve` | the whole contest: explore, come back, then a long press for the fastest run, with the fan switch on |
| `explore_solve_all` | the same with every switch of the fast run on: fan, racing line, boost and risky turns |
| `solve` | the fastest run alone, from a map a previous run saved: pass its `--flash` |
| `solve_all` | the same with every switch on |

A stop over the radio brakes a robot that drives its motors to a standstill in
BRAKE before it is idle, and the Micras analysis reports how far, how much it
turned and how long it took from the stop to rest. A second stop during the
brake trusts neither the path nor the pose: the firmware ramps the speeds the
wheels and the gyroscope measure down to rest through the loop on the speeds
alone, then shorts the motors through the bridge, with the drivers on, until the
robot has settled. Once the wheels brake, a further stop only confirms: it does not restart the ramp
nor clear the time the robot has been at rest. The stop scenarios are checked runs; the `solve_stop` ones
are recorded one row a millisecond (`--record-every 8`), which keeps their 90 s
small. `targets/micras/baselines/README.md` lists what each baseline version
recorded.

```bash
./build/host-release/targets/micras/micras_sim --scenario targets/micras/scenarios/explore_solve.toml \
    --maze apec2017 --out runs/contest --flash runs/contest/flash.bin
cmake --build --preset host-release --target sim_contest       # explore_solve in every maze at once, and their health
cmake --build --preset host-release --target sim_contest_all   # the same with every switch on
```

`MICRAS_SIM_CONTEST_MAZES` (a cache variable, all ten by default) names the mazes
the contest runs in.

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
python3 tools/analyze.py build/host-release/targets/micras/runs/explore
```

Writes `report.json` and plots into the run directory: speeds, contacts,
attitude, actuators, wheels, a top-down trajectory over the maze with the goal
cells shaded and the firmware's pose estimate dashed, and, from the Micras
plugin, the controller's terms and the wall sensors. The
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
cmake --build --preset host-release --target sim_watch
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
cmake --build --preset host-release --target sim_serve
```

The bridge puts the firmware's radio on `ws://localhost:8080` and carries raw
bytes both ways, with no framing of its own: micras-monitor speaks the
firmware's protocol to it exactly as it would over the air. The link is as fast
as the radio's baud rate, not faster, and the run is held to real time as the
robot is: a monitor budgets its link in wall time. A run that falls more than 5 ms
behind is not made to catch up later. Only the wall clock waits, so the recording
is the same.

One monitor holds the link at a time, as one peer pairs with the radio: a monitor
takes it with the first bytes it sends, and the one that held it is closed with
code 4001 and the reason `another monitor took the link`, which micras-monitor
shows instead of reconnecting. A client that connects and sends nothing, such as
a probe or an idle tab, takes nothing.

## Robot tools

```bash
cmake --build --preset host-release --target sim_robot_report       # robot.toml against the firmware's robot.hpp, field by field
cmake --build --preset host-release --target sim_wall_calibration   # each wall sensor's gain for robot.toml
./build/host-release/targets/micras/micras_wall_calibration --sweep
cmake --build --preset host-release --target sim_turn_designs       # the firmware's turns of two bends, into config/two_bend_turns.hpp
```

The firmware checks every turn of two bends when it is compiled, and stops the
build if one no longer clears the walls; `sim_turn_designs` searches for them again,
after a change to the robot's outline, the maze or the margins.

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
`MICRAS_SKIP_VIEWER=1` skips it where there is neither.

```bash
cmake --build --preset host-release --target sim_check
```

The Micras gate: its unit tests, the flash surviving from one run into the next,
the checked scenarios (idle, the first 30 s of an exploration, and every stop
scenario above), their health (no warnings, no collision, no non-finite
sample, no unbound port, no watchdog expiry, no emergency stop, no dropped byte)
and the recorded baseline summaries.

Baselines live in `targets/<robot>/baselines/`; `CLAUDE.md` explains what they
hold and the rules around them. `sim_record_baseline` records the version
`MICRAS_SIM_BASELINE` names and refuses to overwrite one: bump it instead, and
nothing is ever lost. `sim_run_idle`, `sim_run_explore`, `sim_run_stops`,
`sim_check_flash` and `sim_compare_baseline` are the gate's steps on their own. With
`-DMICRAS_SIM_EXACT=ON` a baseline comparison also fails when a checked run's
`data.csv` is not byte identical to the recorded one: the check for a change that
must not move a byte, such as a refactoring.

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
hal_host/        micras_hal implemented on a PC
  models/          SPI chip models: the LSM6DSV IMU and the AS5047U encoders
tests/           the engine's unit tests, on a tiny robot of their own
tools/           analysis, baselines, run health and byte comparison
scripts/         the checks' scripts, which the CMake targets call
cmake/           dependencies, warnings, style and check targets
targets/toy/     the reference target: toy robot, scenarios, baseline
targets/micras/  the micromouse:
  MicrasFirmware/    submodule
  cube/              the Cube layer the firmware includes, by hand
  src/               target, bindings, variables
  robot.toml         the physical description
  scenarios/         idle, explore, explore_link, explore_solve(_all), solve(_all), the stops
  baselines/         recorded summaries
  scripts/           the recipes' scripts
  tools/             analysis plugin, wall calibration, robot report, turn designer
```

`CLAUDE.md` has the architecture, the invariants that hold the determinism
together, and the measurements behind the numbers in `robot.toml`.
