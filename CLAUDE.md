# micras-simulation

`README.md` is how to use this. This file is why it is built the way it is: the
invariants that hold it together and the subtleties that look like bugs and are
not. A robot's own measurements live with its target.

Linux only, by decision. CMake is the entry point: presets configure, and every
recipe is a CMake target over a bash script that takes its paths as arguments, so
no script guesses a build layout. There is no task runner.

## The rule everything else serves

**A run is reproducible.** Two runs of the same binary with the same arguments
produce byte-identical `data.csv` and `meta.json`. Nothing in the engine may make
the firmware's view of the world depend on wall time, on a window being open, on
a client being connected, or on how fast the machine is. Noise is part of the
world and is seeded: `--seed` picks the stream, `--ideal` turns it off.

This is enforced, not hoped for. `micras_sim_check` runs the toy target's
scenario twice, headless and with a window, with a video, and with a bridge nobody
connects to, and compares each run with the plain one as bytes
(`scripts/check_invariance.sh`, `tools/compare_run.py`).

Baselines are summaries, not recordings. Each version under
`targets/<robot>/baselines/` holds one `summary.json` per checked run: the hash of
`data.csv`, the state timeline, and a handful of numbers with their tolerances
(`tools/baseline.py`).
On the machine that recorded it the hash matches; on another it will not, because
the compiler, libm and the MuJoCo build all move the last bits, and the summary is
what is compared. Two rules:

- **A refactoring must not move a byte.** On one machine, a refactoring leaves
  the hash where it was. If it moves, the refactoring is wrong.
- **Never re-record to make a difference go away.** The toy's
  `micras_sim_toy_record_baseline`, like a robot project's own recording target,
  refuses to overwrite a version; a change of behavior is a new version, with the
  reason in the commit.

## Layers

Two layers here, and a third in the project of each robot:

```
engine/, view/, bridge/, app/   the simulator: knows no robot and no HAL
targets/toy/                    the reference target: no HAL, every feature of the engine
a robot's project               its target, bindings and robot.toml, over micras-lib's host HAL
```

`micras_sim_check_generic` greps everything outside `targets/` for names that
belong to a robot and fails on any; only the files shared with micras-lib, which
name the repositories that carry them, and the script itself are not scanned. The
root CMakeLists adds every folder of `targets/` that has a CMakeLists; nothing
above `targets/` names one.

Four engine libraries. `micras_sim_app` sits on top; `micras_sim_view` links only
the engine, so nothing about drawing can reach the recording layer:

```
micras_sim_engine   world, clock, run loop, firmware thread, devices, arenas,
                    robot description and model, scenarios, recording
micras_sim_view     window, declarative panel and overlay, video (GLFW/ImGui/EGL)
micras_sim_bridge   WebSocket server, monitor bridge (IXWebSocket)
micras_sim_app      CLI, application wiring, crash reporter; the Target interface
```

A robot target implements `Target` (`app/include/micras/sim/app/target.hpp`): its
name, folder, firmware commit, loop period, options, robot file, ground truth
columns and program, and `wire()`, which binds its devices to the host ports and
returns what it adds to the run (`Wiring` in `app/include/micras/sim/app/wiring.hpp`:
columns, variables, panel, overlay, scenario hooks). Its `main` is one call to
`micras::sim::run`. The program receives the `FirmwareThread` it runs on, so a
target with no HAL yields on it directly and needs no pointer kept from `wire()`.
The README lists the headers a target may include; they are the public API.

`MICRAS_SIM_VIEWER`, `MICRAS_SIM_VIDEO`, `MICRAS_SIM_BRIDGE`, `MICRAS_SIM_TESTS`
and `MICRAS_SIM_TARGETS` must all still compile when OFF; `micras_sim_check_options`
configures every one of them off, because stale stubs are otherwise only found by
whoever first tries to build without a GPU. The first three are ON by default; the
tests and the robot targets are ON only when the simulator is the top-level project.

### Added to another project

A project can add the simulator with `add_subdirectory(... EXCLUDE_FROM_ALL)` and
build its own target against `micras::sim_app`. Nothing leaks into it: the style
targets are the simulator's own (`micras_sim_format`, `micras_sim_format_check`,
`micras_sim_lint`) and exist only at top level; the build type is defaulted only at
top level, and never forced; the dependencies' switches are normal variables, not
cache entries; paths are anchored to `PROJECT_SOURCE_DIR`/`PROJECT_BINARY_DIR`; the
engine libraries require C++23 of whoever links them; the tests and `targets/` are
off. `MICRAS_SIM_TOOLS_DIR` and `MICRAS_SIM_MAZES_DIR` (internal cache variables)
name the tools and the mazes for the project's own recipes.

### The run loop

`Simulation::run` drives `IRunListener`s. Order per tick:

```
on_before_tick   -> RunControl{RUN, QUIT}   crash reporter, scenario, monitor
firmware.run_until_yield()                  the firmware runs until its timer crosses a step
device->actuate()                           motors, fan: from what the firmware wrote
world.step(steps_per_tick)                  125 us, one MuJoCo step
clock.advance()
device->sample()                            encoders, IMU, wall sensors, ADCs, link
on_after_tick                               event log, recorder, video, viewer
```

Listeners run in registration order, which `Application` fixes. Devices sample
after the step, so what the firmware reads in tick N+1 is the world at the end of
tick N: the true boundary, not a rounding choice.

`has_finished()` is checked both before `before_tick()` and after
`run_until_yield()`, so a firmware that has exited gets no physics step and no
CSV row. A `FinishGuard` runs `finish()` however `run()` leaves, including
through an exception from a listener; without it a thrown listener would leave
the firmware thread parked on the handoff with nobody left to wake it.

### Time: the host timer hands over

The firmware keeps its own notion of time through `micras_hal`'s timer, and the
host backend's `Clock` (micras-lib) is the only place that time exists. **Every read of the
timer costs one microsecond of simulated time**, since a read on the robot takes
time too and a loop that polls the timer must see it advance. When a read
crosses the end of a 125 us step, the clock calls its handover, which is the
firmware thread's `yield_tick()`: the world advances one step and the read
returns after it. So a busy wait runs the world exactly as long as it waits, the
firmware's 125 us loop is one step, and the time inside one iteration is the
number of timer reads times the quantum: deterministic, but not a measurement of
anything. There are no spin guards and no special cases.

### The two threads

The firmware runs its own `main`, compiled under another name, in its own
thread. This is not concurrency: it is a **strict handoff**, a mutex and a
condition variable around a `Turn` flag, and the two threads are never runnable
at the same time. The firmware thread runs until the host timer yields, then
blocks; the simulation thread advances physics, then wakes it. A
ThreadSanitizer build reported no race, which is what the design predicts;
nothing in the repo runs that build routinely.

`finish()` latches `stopping` before checking `joinable`, and if the program
refuses to stop it says so on stderr and calls `std::_Exit`. It does not detach:
a detached thread running over a destroyed world is worse than a loud exit.
`yield_tick()` returns false instead of parking when the thread is stopping
during unwinding. The thread body is called `thread_body()` and not `main()`,
because the firmware's `main` is renamed with `-Dmain=...`, which would rename a
member called `main` too.

### The viewer draws on the simulation thread

There is no render thread. `MujocoViewer` draws inside `on_after_tick` and
blocks inside `on_before_tick` while paused; `--viewer-fps` is the lever on wall
time. The panel and the overlay are declarative: a target returns a
`PanelSpec` of buttons, switches, lamps and readouts and an `OverlaySpec` of
lines, and the view draws them without knowing the robot. The first touch of a
board control hands the run over: the scenario stops driving the inputs and the
run is marked `interactive`. Panel input uses ImGui **edges**
(`IsItemActivated`/`IsItemDeactivated`); writing every frame from the held state
would overwrite a scripted press.

EGL and GLX contexts cannot both be current on one thread. `--viewer --video`
therefore requires that `VideoRecorder` make its EGL context current in
`capture()` and **release it afterwards**, and that `MujocoViewer::draw` make the
GLFW context current first. Without that pair the run dies with
`BadAccess X_GLXMakeCurrent` and leaves an empty CSV.

## The host HAL

The host backend of `micras_hal` lives in
[micras-lib](https://github.com/Team-Micras/micras-lib) (`micras_hal/host/`),
with the SPI device slot and the models of the SPI chips (`micras_proxy/models/`);
its README describes the port registry keyed by the Cube handles, the slot and
the models. The simulator includes no HAL. A robot's target binds the backend's
ports to the engine's devices, and nothing in the backend knows a physics engine.
A port the firmware used that no binding claimed is counted, named on stderr at
the end of the run and written to `meta.json` as `unbound_ports`; a gate expects
zero, and so it does of `watchdog_expiries` and `emergency_stops`, which the
backend's `Mcu` counts instead of resetting a process.

## The toy target

`targets/toy/` exists so that the simulator is checked without any real robot:
it uses every device, the firmware thread's handover (its program yields a tick
on the thread it is given, once per 1 ms loop), a command line option, the
panel, the overlay and the scenario hooks, and it is what the invariance checks
and the simulator's baseline run on. It has no HAL: its devices write into a
plain struct its program reads, which is safe because the two threads never run
at once. Its `robot.toml` is `tests/models/tiny_robot.toml` with the name, a
125 us timestep and a second wall sensor, since the wall sensors fire in two
groups. Its baseline runs start from their output directory with a copy of the
scenario, so a summary records no machine's paths.

## Presets, recipes and sanitizers

`host` is Debug with the address and undefined behavior sanitizers, and is where
the tests and `micras_sim_check` run; `host-release` (RelWithDebInfo) is for runs
and baselines; `host-ci` is `host` with warnings as errors. A run's bytes do not
depend on the build type (a Debug sanitized toy run equals the RelWithDebInfo
baseline). The invariance check turns leak detection off for the window and the
video runs only: the system's GL and font libraries keep their caches until exit.

The simulator's checks (`cmake/checks.cmake`, `scripts/`) and a robot's recipes
(`targets/<robot>/CMakeLists.txt`, `targets/<robot>/scripts/`) exist only when the
simulator is the top-level project, and every script takes its paths as
arguments.

## The robot description

A target's `robot.toml` is the robot's physical truth, written by hand from the
CAD, the board and the datasheets. Every value is either a number or
`{ value, source }` naming where it came from, and `RobotDescription` rejects a
missing key, an unknown key, a wrong type and an unknown schema version. The
engine generates the MJCF from it (`robot_mjcf`) and composes it with the arena
(`Maze::mjcf`, attached under the prefix `maze_`); the composed model is saved to
`<out>/model.xml` and hashed into `meta.json`. What each number of a real robot
rests on is written next to its target.

## The CSV

One row per tick, or per `--record-every` ticks. The engine's block first: the
tick, the simulated time, the body pose and velocity. Then the robot's ground
truth columns, then the firmware's own monitoring variables, then each device's
columns. Two sources naming the same column is an error at the first row.

The first row is the first one due once every column source is ready
(`ColumnSource::ready`). A firmware's variables may exist only once its robot is
constructed, so a firmware whose start-up waits on its chips starts its CSV
that many ticks late.

- A MuJoCo free joint splits its six velocity dofs across two frames, so the
  linear columns are `vx_world`, `vy_world`, `vz_world` and the angular one is
  `wz_body`. `v_forward` is the world linear velocity projected on the body
  forward axis, which for this robot is body +x.
- `*_penetration` is `nan` on ticks where the geom had no contact at all. `0`
  means "in contact, exactly touching".
- The variable columns come from the firmware's `VariablePool`, read through a
  read-only accessor, and are named from its own variable names. State ids come
  from the firmware; it has no names for them, so the target lists the names and
  the build checks that there is one for each state. The device columns are
  what the simulated hardware produced: the IMU samples, the `wall_*` readings,
  the `motor_*_voltage` the bridge applied, `pack_voltage`.

What the recording decimates, the event log does not: collisions and state
changes are detected every tick and written to `meta.json`'s `events`. A
collision is a chassis geom touching anything but the floor after having been
clear of everything for 50 ms; a robot pressed against a wall makes and breaks
contact every few steps, and that is one collision, not hundreds.

The CSV is flushed after every row, and a SIGSEGV/SIGABRT handler writes
`firmware crashed at tick N` to stderr before re-raising, so a firmware crash
still leaves an analysable run behind.

## The link and the bridge

The firmware's radio is a UART behind `hal::UartDma`; the `SerialLink` device
moves bytes between it and the engine's `SerialBus` no faster than the configured
baud rate, and the bus keeps whatever is pending (up to 64 KiB, counting what it
drops in `serial_dropped_bytes`). There is no framing in the simulator: the
firmware's `comm::Link` does its own.

The monitor bridge carries raw bytes both ways. Incoming bytes land in a
mutex-guarded buffer on an IXWebSocket thread and are handed to the bus on the
simulation thread in `on_before_tick`; each tick's output goes out as one binary
frame in `on_after_tick`, through a bounded queue and a sender thread, so a
client that stops reading loses frames (`bridge_dropped_frames`) instead of
stalling the run. A port already taken is a warning, not a failure.

Note what the gate proves: the bridge check compares a bridged run **with nobody
connected** against a plain one. A run a client talks to is marked `interactive`
and is not reproducible, by definition.

## Scenarios

A scenario is a TOML file: the arena, the duration, the seed, the start pose,
timed events (`press` an input for a time, `set` it, `send` a link command) and a
stop condition on a firmware variable. CLI flags override what they name. Starts
go through the robot's real paths: a press of the button starts it, exactly as
on the robot. A scenario never touches the physics, so a
scripted run is one a human could have driven.

## Style

The `.clang-format`, `.clang-tidy`, `tests/.clang-tidy`, `cmake/micras_warnings.cmake`
and `cmake/templates/run_clang_tidy.sh.in` of micras-lib, copied byte for byte
(micras-lib holds the canonical copy and the firmware's CI compares the three
repositories), with clang 22's tools found by their versioned names; configuring
without them fails. The tidy header filter lints this repository's `include/`
headers and never a dependency's. Every target of the simulator gets the shared
warning list through `micras_apply_warnings`, and `MICRAS_SIM_WERROR` makes them errors
(the CI sets it); the dependencies are `SYSTEM`, so their headers raise nothing.
Containers are indexed with `at()`, and a span, which has no `at()` before C++26,
through `micras::sim::at` (`core/span_at.hpp`). `engine/src/.clang-tidy` tells
include-cleaner that toml++ is included through `toml.hpp`, and that a TOML
table's `operator[]` is a lookup, not an unchecked access.
Doxygen on every declaration. **No comments inside function bodies** — if
something needs explaining, it goes in an `@note` on the declaration, where a
reader finds it before reading the code. Every `NOLINT` names its check and says
why. `micras_sim_lint` is clean and stays clean.

## Known gaps

- **No "new run" from the panel.** It would need `RunControl::RESTART`,
  restartable listeners, and a fresh `FirmwareThread` and firmware, because a
  firmware's state machine, its map and its flash all carry state.
- **No minimum wall clearance in the baselines.** The event log has collisions,
  but nothing measures the distance to the nearest wall yet.
- **No pinned container.** Byte identity is only checked between runs on one
  machine.
