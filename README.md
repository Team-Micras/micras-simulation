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
just build
just micras run-explore                   # 30 s of an exploration -> targets/micras/runs/explore
just micras analyze runs/explore          # report.json and the plots
just micras watch                         # an exploration in a window
just micras serve                         # an exploration open to micras-monitor
just check                                # the gate: the engine's checks, then every target's
```

`just` with no arguments lists every recipe, the engine's and each target's. The
build uses `CMakePresets.json`; `MICRAS_PRESET` picks between `default`
(RelWithDebInfo), `debug` and `release`. Only `default` has a test preset, so
`just test` and `just check` need it. Every preset builds with GCC 15
(`gcc-15`/`g++-15`), since another compiler may move the last bits of a run.

## Running

```bash
./build/default/targets/micras/micras_sim --scenario targets/micras/scenarios/explore.toml --out runs/x
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

Micras adds one option of its own:

| Flag | Meaning |
|---|---|
| `--flash <file>` | load the flash from the file before the run and save it back after, so a map the firmware saved survives into the next run |

A window, a recording and a bridge are all optional and all off by default, and
none of them changes what the run produces. `just micras check` proves it for
each, by running the same scenario with and without and comparing the two CSVs
byte for byte.

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
(`explore`, `solve`, `calibrate`, `save`, `reset`). An event
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
| `explore_solve` | the whole contest: explore, come back, then a long press for the fastest run, with the fan switch on |
| `explore_solve_all` | the same with every switch of the fast run on: fan, racing line, boost and risky turns |
| `solve` | the fastest run alone, from a map a previous run saved: pass its `--flash` |
| `solve_all` | the same with every switch on |

```bash
just micras run --scenario scenarios/explore_solve.toml --maze apec2017 --out runs/contest --flash runs/contest/flash.bin
just micras contest                       # explore_solve in all ten mazes at once, and their health
just micras contest explore_solve_all     # the same with every switch on
```

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
just micras analyze runs/explore
```

Writes `report.json` and plots into the run directory: speeds, contacts,
attitude, actuators, wheels, a top-down trajectory over the maze with the goal
cells shaded and the firmware's pose estimate dashed, and, from the Micras
plugin, the controller's terms and the wall sensors. The
report covers warnings, non-finite samples, airborne fraction, slip,
penetration, chassis contact, the state timeline, collisions, whether and when
the goal was reached, the pose estimate's error against the ground truth, the
tracking error and voltage saturation. Only numpy and matplotlib are needed.

`tools/analyze.py` knows only the engine's columns; what a robot's own columns
mean comes from its plugin, `tools/analysis.py` in the target's folder, which
`meta.json` records as `target_dir`. `--plugin` or `$MICRAS_SIM_PLUGIN` name
another; `tools/baseline.py` finds it the same way. `baseline.py compare --exact`
is the byte identity check, and `tools/compare_run.py` compares two runs of one
build, ignoring the fields that name paths.

## Watching a run

```bash
just micras watch
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
just micras serve
```

The bridge puts the firmware's radio on `ws://localhost:8080` and carries raw
bytes both ways, with no framing of its own: micras-monitor speaks the
firmware's protocol to it exactly as it would over the air. The link is as fast
as the radio's baud rate, not faster.

## Robot tools

```bash
just micras robot-report         # robot.toml against the firmware's robot.hpp, field by field
just micras wall-calibration     # each wall sensor's gain for robot.toml
just micras wall-calibration --sweep
just micras turn-designs         # the firmware's turns of two bends, into config/two_bend_turns.hpp
```

The firmware checks every turn of two bends when it is compiled, and stops the
build if one no longer clears the walls; `turn-designs` searches for them again,
after a change to the robot's outline, the maze or the margins.

## The gate

```bash
just check
```

For the engine: builds, runs every unit test through CTest, configures every
optional subsystem off to keep it buildable, and checks that nothing outside
`targets/` names a robot. For Micras: proves that a window, a video and an idle
bridge change nothing, that the flash survives from one run into the next, runs
the checked scenarios, checks their health (no warnings, no collision, no
non-finite sample, no unbound port, no watchdog expiry, no emergency stop, no
dropped byte), and compares them with the recorded baseline summaries.

Baselines live in `targets/micras/baselines/`; `CLAUDE.md` explains what they
hold and the rules around them. `just micras record-baseline` refuses to
overwrite a version: bump it instead, and nothing is ever lost.
`MICRAS_EXACT=1 just check` also fails when a checked run's `data.csv` is not
byte identical to the recorded one: the check for a change that must not move a
byte, such as a refactoring.

`just format`, `just format-check` and `just lint` are the style targets; they
use the firmware's own `.clang-format` and `.clang-tidy`.

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
| `micras/sim/recording/column_source.hpp`, `ground_truth.hpp`, `run_metadata.hpp` | CSV columns, ground truth, meta.json counters |
| `micras/sim/core/variable_source.hpp` | the firmware variables the panel, the overlay and the scenarios read |
| `micras/sim/robot/robot_description.hpp` | the robot's `robot.toml` |
| `micras/sim/devices/*.hpp` | the devices a target builds |

The robot is micromouse-shaped and runs in a maze: its `robot.toml` has every
section of the schema (chassis, wheels, drive, encoders, IMU, fan, battery, link,
wall sensors), and the arena is one of the mazes.

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
targets/micras/  the micromouse:
  MicrasFirmware/    submodule
  cube/              the Cube layer the firmware includes, by hand
  src/               target, bindings, variables
  robot.toml         the physical description
  scenarios/         idle, explore, explore_link, explore_solve(_all), solve(_all)
  baselines/         recorded summaries
  tools/             analysis plugin, wall calibration, robot report, turn designer
```

`CLAUDE.md` has the architecture, the invariants that hold the determinism
together, and the measurements behind the numbers in `robot.toml`.
