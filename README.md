# micras-simulation

Runs the Micras firmware, unmodified, against a MuJoCo model of the robot and
the maze. The firmware sees a proxy layer backed by physics instead of by
hardware, so the same `micras::Micras` that flies on the board drives a
simulated one here.

Two ways to use it, both from the same binary:

- **Headless and deterministic**, for measuring. Every run writes a CSV of the
  whole state per tick plus a metadata file, and two runs of the same binary
  with the same arguments produce byte-identical output.
- **Live**, for looking. A window with the robot, a control panel with the
  board's buttons and switches, plots of the firmware's own variables, and a
  WebSocket bridge that micras-monitor connects to as if it were the radio.

Linux only. MuJoCo 3.3.6 is expected at `$HOME/.mujoco/mujoco-3.3.6`; override
with `-DMUJOCO_DIR=<path>`.

## Quick start

```bash
just build
just run-explore                 # 8 s explore run -> runs/explore_v2
just analyze runs/explore_v2     # report.json and the plots
just watch                       # the same run in a window
just serve                       # the same run with micras-monitor able to connect
just check                       # the gate: everything above, with assertions
```

`just` with no arguments lists every recipe. The build uses `CMakePresets.json`;
`MICRAS_PRESET` picks between `default` (RelWithDebInfo), `debug` and `release`.

## Running

```bash
./build/default/micras_simulation --model models/robot_v2.xml --seconds 10 --out runs/t1
```

| Flag | Meaning |
|---|---|
| `--model <xml>` | required, robot model |
| `--out <dir>` | required, output directory, created if missing |
| `--seconds <float>` | simulated seconds (default 10) |
| `--ticks <int>` | exact firmware tick count, overrides `--seconds` |
| `--maze <xml>` | recorded in `meta.json` only; the model already attaches its maze |
| `--command none\|explore\|solve\|calibrate` | injects one `SERIAL_VARIABLE` packet writing `true` to the pool variable of the same name |
| `--command-at <s>` | when to inject it (default 0.5); never before tick 1, which carries the map request |
| `--dip fan=0,diagonal=0,boost=0,risky=0` | DIP switch state before the first tick; indices follow `Interface::DipSwitchPins` |
| `--button none\|short\|long\|extra_long` | start through the `Button` proxy instead of a packet |
| `--button-at <s>` | when the button goes down (default 0.5) |
| `--no-fan` | the `Fan` proxy writes 0 to its actuator instead of the requested speed |
| `--viewer` | open a window on the run |
| `--viewer-camera <name>\|free` | starting camera (default `free`) |
| `--viewer-fps <int>` | redraws per simulated second (default 30) |
| `--viewer-size WxH` | window size, default 1200x900 |
| `--monitor` | open the micras-monitor bridge |
| `--monitor-port <int>` | port it listens on (default 8080) |
| `--video <out.mp4>` | record an offscreen mp4 of the run |
| `--video-fps <int>` | frame rate, default 30 |
| `--video-camera <name>` | `overhead`, `side tracking` (default), `robot pov`, `maze_top_view`, or `free` |
| `--video-size WxH` | frame size, default 1280x720 |

A window, a recording and a bridge are all optional and all off by default. None
of them changes what the run produces: `just check` proves it by running the
same scenario twice and comparing the two CSVs byte for byte.

## Output

`<out>/data.csv` is one row per firmware tick: the ground truth from MuJoCo,
what crossed the proxy boundary in both directions, and every monitoring
variable the firmware publishes.

`<out>/meta.json` is the provenance of the run — firmware SHA, model hash,
MuJoCo and compiler versions, the arguments, the tick and time counts, and the
assertions the gate checks (`warnings_total`, `pool_columns`,
`telemetry_resyncs`, `interactive`). Nothing measured off the wall clock goes in
it, so it is hash-stable too; the wall time is printed on stdout only.

`interactive` is the honest flag: it is `true` only if a human actually touched
a control the firmware can see. A window that was merely open leaves it `false`,
and the run is still reproducible from `args`.

## Analysis

```bash
just analyze runs/explore_v2
```

Writes `report.json` and six plots into the run directory: speeds, contacts,
attitude, control, wheels, and a top-down trajectory coloured by time with the
goal cells shaded and the odometry estimate dashed. The report covers duration,
warnings, non-finite samples, per-wheel airborne fraction, slip, penetration,
chassis contact, control saturation, odometry error against ground truth,
deceleration events, the FSM transition timeline, and whether and when the goal
was reached. Only numpy and matplotlib are needed.

`tools/run_stats.py` prints the per-run metrics, `tools/turn_stats.py` the
per-turn table (offset and heading error at entry, executed versus commanded
angle, and the smallest distance from the chassis to any wall during the turn).

## Watching a run

```bash
just watch
```

| Key | Action |
|---|---|
| space | pause and resume |
| right | one tick while paused |
| tab | cycle cameras |
| c / f / t / r | contact points, contact forces, transparency, rangefinder rays |
| esc | quit |

Left drag orbits, right drag pans, scroll zooms, and ctrl with left drag pushes
the robot around. The panel holds the board itself: the button, the four DIP
switches, the fan override, the LED, the two addressable LEDs and the buzzer as
the firmware is driving them, the wheel commands and wall readings, and plots of
the desired versus measured linear and angular speed straight from the firmware
pool. Pausing, stepping and a speed limiter are there too.

Touching any board control hands the run over: the scripted `--command` and
`--button` stop being applied, and the run is marked `interactive`.

## micras-monitor

```bash
just serve
```

The bridge puts the firmware's radio on `ws://localhost:8080`. It speaks the
same packet protocol the real board does, so micras-monitor connects to it
without knowing it is a simulation. One binary frame per tick carries everything
the firmware wrote; whatever the monitor sends is framed and queued on the bus,
which hands the firmware one packet per tick exactly as the radio would.

`tools/monitor_probe.py` is a dependency-free client for checking the bridge
without the real monitor.

## Firmware hardware tests

The firmware's own `tests/` programs build against the same proxies:

```bash
just run-test test_imu --seconds 4 --out runs/imu
```

They take the same command line as the main program. `just smoke-tests` runs
each of them briefly, which is what catches a program that stops handing ticks
back to the simulation.

## The gate

```bash
just check
```

Builds, runs the unit tests through CTest, checks the harness config against the
firmware submodule for drift, configures every optional subsystem off to keep it
buildable, smoke-runs the hardware tests, proves that a window and a bridge
change nothing, runs the three scenarios, analyses them, asserts their metadata,
and compares all three against every recorded baseline.

Baselines live in `baseline/`. The current one is compared byte for byte; older
ones keep their columns compared as a prefix, so a run recorded long ago stays
comparable even after new columns are appended. `just record-baseline` refuses
to overwrite a version — bump it instead, and nothing is ever lost.

`just format`, `just format-check` and `just lint` are the style targets; they
use the firmware's own `.clang-format` and `.clang-tidy`.

## Layout

```
config/          target.hpp (MuJoCo wiring) and constants.hpp (tuning)
include/micras/proxy/, src/proxy/    the 16 proxies the firmware sees
include/micras/sim/, src/sim/        core/, telemetry/, recording/, view/, bridge/, app/
models/          robot and maze models, plus the tuning experiments
tools/           analysis, drift, assertions, comparison, monitor probe
tests/           unit tests, and the firmware hardware tests under tests/src/
baseline/        recorded runs the gate compares against
MicrasFirmware/  submodule
```

`CLAUDE.md` has the architecture, the invariants that hold the determinism
together, and the tuning history behind the numbers in `config/constants.hpp`.
