# micras-simulation

`README.md` is how to use this. This file is why it is built the way it is: the
invariants that hold it together, the subtleties that look like bugs and are
not, and the measurements behind the numbers in `targets/micras/robot.toml`.

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
- **Never re-record to make a difference go away.** `sim_record_baseline` (and
  the toy's `micras_sim_toy_record_baseline`) refuses to
  overwrite a version; a change of behaviour is a new version, with the reason in
  the commit.

## Layers

Everything is in this repository, in three layers:

```
engine/, view/, bridge/, app/   the simulator: knows no robot and no HAL
hal_host/                       micras_hal implemented on a PC, and chip models: knows no physics
targets/toy/                    the reference target: no HAL, every feature of the engine
targets/micras/                 the micromouse: firmware submodule, bindings, robot.toml
```

`micras_sim_check_generic` greps everything outside `targets/` for names that
belong to a robot and fails on any (`hal_host/`, the documentation and the files
shared with micras-lib are not scanned). The root CMakeLists adds every folder of
`targets/` that has a CMakeLists; nothing above `targets/` names one.

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

The firmware keeps its own notion of time through `micras_hal`'s timer, and
`hal_host`'s `Clock` is the only place that time exists. **Every read of the
timer costs one microsecond of simulated time**, since a read on the robot takes
time too and a loop that polls the timer must see it advance. When a read
crosses the end of a 125 us step, the clock calls its handover, which is the
firmware thread's `yield_tick()`: the world advances one step and the read
returns after it. So a busy wait runs the world exactly as long as it waits, the
firmware's 125 us loop is one step, and the time inside one iteration is the
number of timer reads times the quantum: deterministic, but not a measurement of
anything. There are no spin guards and no special cases.

### The two threads

The firmware runs its own `main` (compiled as `micras_firmware_main`) in its own
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

`hal_host/` implements every `micras_hal` class the firmware uses on a PC, and
includes nothing but `micras_hal` headers and its own. `Board` is a registry of
ports (`GpioPort`, `PwmPort`, `PwmDmaPort`, `AdcPort`, `UartPort`, `FlashPort`,
`McuPort`, `EncoderPort`, `SpiPort`) **keyed by the address of the Cube handle
or GPIO port** that names them, so the firmware's own `target.hpp` is the key:
`.handle = &htim4` there and `Board::pwm(&htim4, ...)` in a binding reach the same
port. Nothing in `hal_host` knows a physics engine; the target's bindings connect
ports to devices.

A port the firmware used that no binding claimed is counted, named on stderr at
the end of the run and written to `meta.json` as `unbound_ports`; the gate
expects zero. So do
`watchdog_expiries` and `emergency_stops`, which `Mcu` counts instead of
resetting a process.

`hal_host/include/stm32_host.h` holds the handle types and HAL constants. The
Cube layer the firmware includes (`main.h`, `tim.h`, `adc.h`, ...) is written by
hand per robot in `targets/micras/cube/`, with the handles configured as
CubeMX configures them on the board (timer clock 275 MHz, TIM4 centre-aligned at
PSC 274 / ARR 250, and so on), because the host `Pwm` and `Timer` compute
frequencies and duties from those registers.

Of the firmware, everything compiles unchanged except `micras_hal` (replaced by
`hal_host`), and `FmacFilter` is left out because nothing uses it and it needs
the FMAC. The proxies of the SPI chips, `Imu` and `RotarySensor`, are the
firmware's own, and so is ST's register driver of the LSM6DSV, compiled as C.

### The SPI device slot

A chip on an SPI bus is a `SpiDevice`
(`hal_host/include/micras/hal/host/spi_device.hpp`): `select()`,
`exchange(tx, rx)`, `deselect()` and the SPI mode it answers in. A
binding attaches it with `Board::spi_device(handle, cs_port, cs_pin, device)`,
**keyed by the bus and the chip select**, because `hspi3` carries the IMU and both
encoders. The host `Spi` selects the device in `select_device`, routes every
`transmit`, `receive` and `transmit_receive` to it, and deselects it in
`unselect_device`, so a register read that the driver makes of two HAL calls is
one transaction for the chip, as on the bus.

- **The mode is checked on every transfer.** A device whose mode differs from the
  one `select_device` wrote into the handle is not reached and the firmware reads
  all ones, as from a chip clocked on the wrong edge. So is a chip select with no
  device, which is also reported as an unbound port.
- **`start_transfer` completes on the host clock.** The bytes are exchanged at the
  start, and the transfer ends when the host clock passes the time its bytes take
  at the bus's bit rate: the kernel clock the fake Cube layer writes into the
  handle's `Instance` (125 MHz for SPI3), divided by the baud rate prescaler (32),
  so the IMU's 17-byte burst takes 34.8 us. `get_transfer` and a `select_device`
  that finds the bus busy end it through `on_transfer_end`, which raises the chip
  select and sets `COMPLETE`, as the DMA interrupt does. A `select_device` on a
  busy bus waits for it on the timer, as the firmware's own does.
- **Blocking transfers take no time.** Only the timer costs time on the host; the
  driver's own waits (`sleep_ms`, the encoder's 1 us deselect time) cost what they
  cost on the robot.
- **The firmware's objects outlive the run.** `Micras` is a static of the
  firmware's `main`, destroyed when the process exits, after the target and its
  chips; the IMU's `Spi` ends its last transfer then. `MicrasTarget::unwire`
  therefore forgets every port, which detaches the chips.

### The chip models

`hal_host/models/` builds `micras_hal_host_models` (`micras::hal_host_models`,
namespace `micras::models`): SPI devices that know the slot and nothing else, with
their own unit tests. The Micras target owns one of each per chip
(`MicrasChips`) and attaches them to `hspi3` by the chip selects `target.hpp`
names.

- **`Lsm6dsvModel`**: the 128 registers of the main page with the datasheet
  defaults (WHO_AM_I 0x70), `SW_POR` and `SW_RESET`, auto-increment under
  `IF_INC`, SPI mode 3. The binding turns each sample of the engine's `Imu` device
  back into rad/s and m/s^2 and calls `push_sample`, which encodes it with the
  **full scale in CTRL6/CTRL8, the one the firmware wrote**, and sets `GDA`/`XLDA`;
  reading a sensor's output high byte clears its bit, so the burst that reads a
  sample clears it. A sensor whose ODR is off ignores samples.
- **`As5047uModel`**: 24-bit frames with the CRC-8 (polynomial 0x1D, initial 0xC4,
  final XOR 0xFF) checked on every frame, answers pipelined by one frame, the
  volatile registers the firmware writes and reads back (DISABLE, ZPOSM, ZPOSL,
  SETTINGS1 to 3, ECC), ERRFL with the CRC and framing error bits, SPI mode 1. The
  position is not modeled: it reaches the firmware through the timer encoder, as
  on the robot.

Two timings follow from the real drivers. The `Imu` reads the burst it started
one `update()` earlier, so a sample reaches the firmware one loop after the
update that asked for it. Its constructor waits 10 ms, then 30 ms after the
software power-on reset, so the robot exists, and INIT starts, 40 ms into the
run.

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
the tests and `micras_sim_check` run; `host-release` (RelWithDebInfo) is for runs,
the contest and the Micras baselines, whose recorded runs it reproduces byte for
byte; `host-ci` is `host` with warnings as errors. A run's bytes do not
depend on the build type (a Debug sanitized toy run equals the RelWithDebInfo
baseline). The invariance check turns leak detection off for the window and the
video runs only: the system's GL and font libraries keep their caches until exit.

The simulator's checks (`cmake/checks.cmake`, `scripts/`) and a robot's recipes
(`targets/<robot>/CMakeLists.txt`, `targets/<robot>/scripts/`) exist only when the
simulator is the top-level project, and every script takes its paths as
arguments.

## The robot description

`targets/micras/robot.toml` is the robot's physical truth, written by hand from
the CAD, the board and the datasheets. Every value is either a number or
`{ value, source }` naming where it came from, and `RobotDescription` rejects a
missing key, an unknown key, a wrong type and an unknown schema version. The
engine generates the MJCF from it (`robot_mjcf`) and composes it with the arena
(`Maze::mjcf`, attached under the prefix `maze_`); the composed model is saved to
`<out>/model.xml` and hashed into `meta.json`.

`robot.hpp` in the firmware is the firmware's *belief*, and the two are never
forced equal. `sim_robot_report` prints them side by side. Today they
differ in the emitter half angle (3 deg datasheet against 5.2 deg with mounting
tolerance), the gyro noise (datasheet against a third more), the maze wall
thickness (12 mm arena against 12.6 mm) and the front length (53.5 mm board
against 54.9 mm with the sensor housings), all on purpose.

Things in `robot.toml` that look arbitrary and are not:

- **The tyres have no rolling friction** (condim 4, not 6). MuJoCo's convex
  contact separates two surfaces in proportion to how fast their friction is
  slipping, and a rolling wheel keeps a rolling-friction constraint slipping all
  the time. With it the tyres lost the floor on 18 % of steps at 0.4 m/s and 55 %
  at 1.5 m/s, at every timestep tried (125, 62.5 and 31.25 us), and so did a
  bare sphere. The same separation happens where tyres really slide, in a pivot;
  a tyre soft enough (`contact_time_constant` 5 ms or more) absorbs it inside its
  own deflection. 8 ms is the estimate for the 1 mm silicone band. With the old
  2 ms the hopping wheels hardly scrubbed, and a pivot at 0.6 V spun at 3.2 rad/s
  instead of 1.1.
- **The timestep is 125 us**, one firmware loop. Halving it changed neither
  the contacts nor the trace.
- **The chassis mass is 62 g**, not 70: the firmware's 70 g and 2.9e-5 kg m^2 are
  the whole robot, and the wheels are modelled separately.
- **Each wall sensor has a gain.** `tools/wall_calibration` places the robot
  where the firmware calibrates (centred in a corridor for the side sensors,
  facing a wall for the front ones) and sets each gain so that the simulated
  reading equals the robot's `reference_readings` in `target.hpp`. Without the
  gains the readings were about four times low and the localizer corrected
  against them. `--sweep` shows how the firmware's distances then follow the
  true ones.
- **Two skids the CAD does not have.** The robot rests on the edges of its board:
  the rear one at rest, since the centre of mass is 0.8 mm behind the axle, and
  the front one with the fan on, since the fan pulls 17.5 mm ahead of it. Two
  1 mm spheres stand in for those edges. They are frictionless because a sliding
  frictional contact separates in MuJoCo like the tyres above; the board's own
  friction would lift it off the floor.
- **The fan pulls straight down**, not along the board's normal: its actuator's
  reference is a site fixed in the world. With the fan on, the board tilts onto
  its nose and a pull along its normal has a backward part, about 1 % of it. On
  the robot the friction of the board edge holds that. On the frictionless skids
  it rolled the robot 4 mm into the back wall while it waited 5 s for the fan.
- **The fan's 3 N is the owner's figure**, at full speed on the charged pack
  (`nominal_voltage` 12.3 V). The nose carries a third of it, so the tyres get
  2 N, and they sink 0.15 mm into the floor under it. Their rolling radius is
  then 0.6 % short of 11 mm, and the firmware's odometry runs 0.6 % long with the
  fan on. The same is expected of the real band, so calibrate the wheel radius
  with the fan running.

## The CSV

One row per tick, or per `--record-every` ticks. The engine's block first: the
tick, the simulated time, the body pose and velocity. Then the robot's ground
truth columns, then the firmware's own monitoring variables, then each device's
columns. Two sources naming the same column is an error at the first row.

The first row is the first one due once every column source is ready
(`ColumnSource::ready`). The firmware's variables exist only once its robot is
constructed, and the constructor takes the IMU's 40 ms of start-up waits, so
Micras's CSV starts at tick 320, not at tick 1.

- A MuJoCo free joint splits its six velocity dofs across two frames, so the
  linear columns are `vx_world`, `vy_world`, `vz_world` and the angular one is
  `wz_body`. `v_forward` is the world linear velocity projected on the body
  forward axis, which for this robot is body +x.
- `*_penetration` is `nan` on ticks where the geom had no contact at all. `0`
  means "in contact, exactly touching".
- The variable columns come from the firmware's `VariablePool`, read through a
  read-only accessor, and are named from its own variable names. State ids come
  from the firmware; it has no names for them, so the target lists the names and
  the build checks that there is one for each state. The device columns are what the simulated hardware produced: the
  `lsm6dsv_*` IMU samples, the `wall_*` ADC readings, the `motor_*_voltage` the
  bridge applied, `pack_voltage`.

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
go through the robot's real paths: a short press of the button starts an
exploration, exactly as on the robot. A scenario never touches the physics, so a
scripted run is one a human could have driven.

## The firmware submodule

`targets/micras/MicrasFirmware/` follows the firmware's `main` branch. **Edit it
only with intent**, in the firmware's own style, and only with changes that make
sense on the real robot too. The simulator needs three accessors the firmware
keeps for it: `Micras::get_instance()`, `get_variables()` and `get_state()`, for
the variable columns, the state events and the stop conditions. Nothing else in
the simulator reaches into the firmware: every proxy is the firmware's own.

## Style

The `.clang-format`, `.clang-tidy`, `tests/.clang-tidy`, `cmake/micras_warnings.cmake`
and `cmake/templates/run_clang_tidy.sh.in` of micras-lib, copied byte for byte
(micras-lib holds the canonical copy and the firmware's CI compares the three
repositories), with clang 22's tools found by their versioned names; configuring
without them fails. The tidy header filter lints this repository's `include/`
headers and never a dependency's; `targets/micras/.clang-tidy` also leaves out the
firmware's headers and the host HAL's, which their own repositories lint, and
`hal_host/` is formatted here but linted in micras-lib. Every target of the
simulator, the firmware compiled for the host included, gets the shared warning
list through `micras_apply_warnings`, and `MICRAS_SIM_WERROR` makes them errors
(the CI sets it); the dependencies are `SYSTEM`, so their headers raise nothing.
Containers are indexed with `at()`, and a span, which has no `at()` before C++26,
through `micras::sim::at` (`core/span_at.hpp`). `engine/src/.clang-tidy` tells
include-cleaner that toml++ is included through `toml.hpp`, and that a TOML
table's `operator[]` is a lookup, not an unchecked access.
Doxygen on every declaration. **No comments inside function bodies** — if
something needs explaining, it goes in an `@note` on the declaration, where a
reader finds it before reading the code. Every `NOLINT` names its check and says
why. `micras_sim_lint` is clean and stays clean.

## State of the port

The firmware runs on the host HAL end to end, its SPI chip drivers included.
Every proxy that `Micras::check_initialization` checks comes up: the IMU's
start-up waits take 40 ms, and INIT reaches IDLE on the tick after it starts. A
button press starts an exploration through the firmware's own paths. The checked
runs end with no unbound port, no watchdog expiry and no emergency stop.
`--flash` carries a saved map into the next run.

**The whole contest runs clean on ten mazes**, which `sim_contest` shows: a
short press explores, the firmware comes back to the start on its own and saves the
map, and a long press then plans and runs the fastest route with the fan.
`sim_contest_all` does the same with every switch on (fan,
racing line, boost, risky), and there too every maze is clean; japan2017ef and apec2016,
whose boards grazed walls while the line through the risky turns kept only 8 mm, are clean
on eight seeds of the fast run. The mazes are maze1, maze2, apec2016 to apec2019,
japan2013ef, japan2017ef, uk2016f and alljapan-033-2012-exp-fin, in
`engine/arenas/maze/mazes/`. Diagonals are always allowed; the second switch selects
the racing line.

The search takes 64 to 118 s before the fast run starts. The fast run takes, with the
fan, 4.2 s on maze 1 and 3.9 to 9.2 s on the others; with every switch on, 3.7 s on
maze 1 and 3.6 to 8.6 s on the others.

What the firmware does that the fast modes depend on, each with its evidence in the
firmware commit that made it:
- **The edge tracker times a wall edge by the reading and only names it from the map.**
  An edge is the only reference along a corridor.
- **The odometry rolls on a radius the load flattens.** With the fan the tyres carry
  four times the load.
- **Boost asks for 0.65 of the traction.** More slides the tyres sideways in the turns.
- **The wall observer keeps voting through the search turns**, so the search never
  stops in a cell to look.
- **A range is corrected for the angle it meets the wall at.** Along a diagonal the
  diagonal sensors meet the walls square on.
- **Turns of two bends inside one cell**, on a planner of labels that only drops one
  when another is as fast for every route. The planner advances by edges, and
  `sim_turn_designs` designs the turns, which the firmware's build checks.
- **Turns are braked and accelerated through as their curvature allows.**
- **The tyres slide to the outside of a curve**, 4.8 mm/s per m/s^2 of lateral
  acceleration. The localizer predicts it and the controller points into it.
- **The racing line**, through the cells of the planned route, at 0.8 of the lateral
  grip and 15 mm from the walls. With risky on, the line goes through the route planned
  without the risky turns, since it keeps their margin, and replaces the risky route
  only when it plans faster.

Findings worth checking on the robot:

1. **Front sensor parallax.** Each front emitter sits 6.5 mm above its receiver and
   the receiver's lobe is 10 degrees, so the reading falls slower than 1/d^2. The
   firmware models it (`receiver_offset`, `receiver_half_angle`). A bench sweep
   toward a wall confirms both.
2. **The rolling radius.** 56 um less per newton on a tyre in the simulation. Driving a
   known distance with the fan on and off measures the real one.
3. **The fan tips the robot onto its nose.** It pulls 17.5 mm ahead of the axle,
   and a thin-gap flow estimate puts its centre of suction at 14 to 16 mm. So a
   third of the 3 N rests on the front edge of the board, which also drags. Scales
   under the wheels and under the nose, with the fan running, measure the share.
4. **A wall start is seen early.** Toward the start of a wall a diagonal sensor also
   lights the wall's end face, by the wall thickness times the slope of the beam.
5. **Past 120 mm the readings come out long**, because part of the beam lands on the
   floor. The localizer stops at 120 mm and the wall observer at 130 mm.
6. **The mass, inertia and tyre friction are estimates**, and so is the fan's
   position. `robot-report` lists what differs from the firmware's belief.
7. **The lateral compliance of the tyres.** `robot.hpp` has the simulation's 4.8 mm/s per
   m/s^2. A circle of known radius driven at a few speeds with the fan on measures the real
   one, and the racing line depends on it.

## Known gaps

- **No "new run" from the panel.** It would need `RunControl::RESTART`,
  restartable listeners, and a fresh `FirmwareThread` and firmware, because the
  FSM, the maze map and the flash all carry state.
- **The firmware's hardware tests are not built.** The design supports
  them, since each is a program with its own `main` over the same HAL, but
  `test_imu` spins forever when the IMU is not initialised and two tests were
  deleted upstream; they are out of scope for now.
- **No minimum wall clearance in the baselines.** The event log has collisions,
  but nothing measures the distance to the nearest wall yet.
- **No pinned container.** Byte identity is only checked between runs on one
  machine.
