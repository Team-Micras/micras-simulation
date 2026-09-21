# micras-simulation

`README.md` is how to use this. This file is why it is built the way it is: the
invariants that hold it together, the subtleties that look like bugs and are
not, and the measurements behind the numbers in `config/constants.hpp`.

Linux only, by decision. `just` is the entry point; there is no Makefile.

## The rule everything else serves

**A run is reproducible.** Two runs of the same binary with the same arguments
produce byte-identical `data.csv` and `meta.json`. Nothing in `sim/` may make
the firmware's view of the world depend on wall time, on a window being open, on
a client being connected, or on how fast the machine is.

This is enforced, not hoped for. `just check` runs the same scenario headless
and with a window, headless and with a bridge, and compares the two CSVs as
bytes; and it compares the three standard scenarios against the recorded
baselines in `baseline/`.

Two invariants govern the baselines:

- **A refactoring must not move a byte.** If `data.csv` differs from
  `baseline/v2`, the refactoring is wrong. Revert it. Never re-record the
  baseline to make a diff go away.
- **An extension only appends columns.** Every row's prefix up to the first new
  column stays byte-identical, and the previous baseline is kept and compared
  with `--columns-subset`, so old recordings stay comparable forever. Note what
  that check is: a header prefix. New columns therefore have to go at the very
  end — appending a pool variable would shift every `proxy_*` column right and
  break it.

## Architecture

Five libraries. `micras_sim_app` sits on top of all of them; `micras_sim_view`
links only the core, so nothing about drawing can reach the recording layer:

```
micras_sim_core     world, clock, serial bus, proxy state, run loop, firmware thread
micras_sim          + scenario, telemetry, recording          (links the firmware shadow)
micras_sim_view       window, control panel, video recorder   (GLFW/ImGui/EGL, on core)
micras_sim_bridge   + WebSocket server, monitor bridge        (IXWebSocket)
micras_sim_app      + CLI, application wiring, crash reporter
```

`src/main.cpp` is one call; `Application::main` is the `try`. The application is the only layer
that knows a run can have a window, so it is the only one that links the view.

`MICRAS_VIEWER`, `MICRAS_VIDEO`, `MICRAS_BRIDGE` and `MICRAS_TESTS` are all ON
by default and all must still compile when OFF; `just check-options` configures
every one of them off as part of the gate, because stale stubs are otherwise
only found by whoever first tries to build without a GPU.

### The run loop

`Simulation::run` drives `IRunListener`s. Order per tick:

```
on_before_tick   -> RunControl{RUN, QUIT}   crash reporter, scenario, monitor, pause
firmware.run_until_yield()                  the firmware thread runs one loop
world.step(steps_per_tick)                  2 x 0.000521 s = 1042 us
clock.advance()
on_after_tick                               monitor, telemetry, recorder, video, viewer
```

Listeners run in registration order, which `Application` fixes. `Telemetry`
overrides only `on_after_tick`, so the pool values a row carries are decoded
after the tick they belong to and before the recorder writes it.

`has_finished()` is checked both before `before_tick()` and after
`run_until_yield()`, so a firmware that has exited gets no physics step and no
CSV row. A `FinishGuard` runs `finish()` however `run()` leaves, including
through an exception from a listener; without it a thrown listener would leave
the firmware thread parked on the handoff with nobody left to wake it.

### The two threads

The firmware runs `micras::Micras` in its own thread. This is not concurrency:
it is a **strict handoff**, a mutex and a condition variable around a `Turn`
flag, and the two threads are never runnable at the same time. The firmware
thread runs until it calls `yield_tick()`, then blocks; the simulation thread
advances physics, then wakes it. A ThreadSanitizer build reported no race, which
is what the design predicts; nothing in the repo runs that build routinely.

The thread exists because the firmware's loop is a loop: `Micras::update()`
busy-waits on its own stopwatch, and there is no way to return from the middle
of it. Everything else — the window, the panel, the bridge callbacks that matter
— is drained on the simulation thread.

`finish()` latches `stopping` before checking `joinable`, and if the program
refuses to stop it says so on stderr and calls `std::_Exit`. It does not detach:
a detached thread running over a destroyed world is worse than a loud exit.
`yield_tick()` returns false instead of parking when the thread is stopping
during unwinding, which is how a firmware test that throws gets to leave.

The thread body is called `thread_body()` and not `main()`, because the hardware
test executables compile the firmware's `main` under `-Dmain=micras_test_main`,
which would rename a member called `main` too.

### The viewer draws on the simulation thread

There is no render thread, and therefore no snapshot and no mutex around the
proxy state — the snapshot only ever existed to protect a render thread that was
never built. `MujocoViewer` draws inside `on_after_tick` and blocks inside
`on_before_tick` while paused. Rendering dominated wall time when it was last
measured, roughly 70 % of it; `--viewer-fps` is the lever, and the figure is an
estimate rather than something the repo records.

EGL and GLX contexts cannot both be current on one thread. `--viewer --video`
therefore requires that `VideoRecorder` make its EGL context current in
`capture()` and **release it afterwards**, and that `MujocoViewer::draw` make the
GLFW context current first. Without that pair the run dies with
`BadAccess X_GLXMakeCurrent` and leaves an empty CSV.

## The shadow proxy layer

`config/` and `include/micras/proxy/` shadow the real `micras_proxy`. Include
order matters: `config` and `include` come before `MicrasFirmware/include` so
the firmware picks up ours. `MicrasFirmware/src/main.cpp`, `micras_hal` and
`micras_proxy` are excluded from the build.

Proxies are constructed as members of `Micras`, initialised at their
declarations from the `*_config` globals in `config/target.hpp`. There is no
constructor to inject anything into. So each `Config` carries a
`sim::SimulationContext*`, and `SimulationContext::instance()` is a
function-local static, so it is constructed on first use and never before
whoever depends on it. Exactly three places name it: `config/target.hpp`, once
per `Config`; `Application`'s member initialiser, which is where the process's
one context is picked up; and the context's own file, for the yield helper the
proxies call. **No proxy and nothing else in `sim/` may call it** — they receive
the facet they need
(`MujocoWorld&`, `Clock&`, `SerialBus&`, `ProxyState&`) through the `Config`. A
proxy built without a context, or before a model is loaded, throws at
construction. Tests build their own `SimulationContext` and pass it through the
same field.

`tests/unit/shadow_conformance_test.cpp` instantiates every shadow `Config` with
the firmware's own designated initialisers, which is what catches a field that
drifted (it already caught `Storage::Config`'s `start_page` against
`start_sector`).

`tools/check_config_drift.py` diffs `config/constants.hpp` and
`config/target.hpp` against the firmware's field by field and fails on anything
not in the allowlist at the top of the script. Entries still marked `TODO` there
are known divergences not yet written into the `constants.hpp` `@note`.

### Stopwatch, the one that bites

`Stopwatch` reports **simulated** time, and it is honest: it returns what the
clock actually says. That alone deadlocks, because the firmware busy-waits on it
while physics is frozen. So it has spin guards, and they are deliberately
asymmetric: `elapsed_time_us` yields on the **second** repeated read at the same
simulated instant (`max_repeated_reads = 2`), `elapsed_time_ms` only on the
**fourth** (`max_repeated_ms_reads = 4`), because the firmware legitimately reads
the same millisecond twice in one tick when it classifies a button release.
`sleep_us` yields until the target passes.

`reset_us` and `reset_ms` move `counter`, which is the point of them. What
deliberately **survives a reset** are `last_read_us` and the two spin counters.
This looks wrong and is load-bearing: `Fan::update` resets its own stopwatch every call, so a counter
that reset with it would never reach the threshold, and `test_fan` would spin
forever. The constructor seeds `counter = now_us - us_per_tick`, so the
firmware's first loop reads the nominal period instead of zero.

## The CSV

85 columns: the tick, the simulated time and the firmware's own loop time, then
33 of MuJoCo ground truth, 29 from the firmware pool, and 20 of what crossed the
proxy boundary.

- A MuJoCo free joint splits its six velocity dofs across two frames, so the
  linear columns are `vx_world`, `vy_world`, `vz_world` and the angular one is
  `wz_body`. `v_forward` is the world linear velocity projected on the body
  forward axis, `-vx_world*sin(yaw) + vy_world*cos(yaw)`; the model's forward
  axis is body +y.
- `left_penetration` and `right_penetration` are `nan` on ticks where the geom
  had no contact at all. `0` means "in contact, exactly touching".
- The `proxy_*` block is what the firmware read and wrote at the **start** of
  the tick, so those sensor columns are one tick behind the ground truth on the
  same row. That is the true boundary, not a rounding choice.
- `grid_pose` is the cell the robot is *moving into*, so it legitimately leads
  the ground-truth cell by one cell for most of a run. `grid_pose_side` follows
  `nav::Side` (RIGHT=0, UP=1, LEFT=2, DOWN=3) and `fsm_state` follows
  `Micras::State` (INIT=0, IDLE=1, WAIT_FOR_RUN=2, RUN=3, WAIT_FOR_CALIBRATE=4,
  CALIBRATE=5, ERROR=6).

Columns the firmware never feeds are **not** in the CSV. The battery voltage and
the LED/ARGB/buzzer block were removed once measured: this build never reads the
battery, and the only call that reaches those proxies is the `buzzer->update()`
in `Micras::update`, so the columns were constant. `ProxyState` still carries `interface_output`, because the panel shows
it.

The CSV is flushed after every row, and a SIGSEGV/SIGABRT handler writes
`firmware crashed at tick N` to stderr before re-raising, so a firmware crash
still leaves an analysable run behind.

## Telemetry and the bus

`BluetoothSerial` is an in-process byte queue. At tick 0 the harness injects one
`SERIAL_VARIABLE_MAP_REQUEST` and names the pool columns from the response. The
firmware clears its whole receive deque after framing one packet, so **never
inject more than one packet per tick** — the `SerialBus` enforces exactly that,
which is also what keeps a burst from a connected monitor out of the run's
determinism.

The map reports types through `core::type_name`, so they arrive fully qualified
(`micras::nav::GridPose`, `micras::nav::State`, alongside plain `float` and
`unsigned char`). The decoder matches on the last component only and expands custom serializables into one column per field.

The monitor bridge is the same bus with a socket on it. Incoming bytes land in a
mutex-guarded `PacketFramer` on an IXWebSocket thread and are drained on the
simulation thread in `on_before_tick`; the tick's output goes out as one binary
frame in `on_after_tick`. A port already taken is a warning, not a failure.

That one-packet-per-tick rule is what keeps a burst from a connected monitor out
of the run, but note what the gate actually proves: `check-monitor` compares a
bridged run **with nobody connected** against a plain one. The bridged run that
does have a client attached is not compared against anything, so immunity to a
talking monitor is enforced by construction and not yet by a test.

## Real versus stub proxies

- Backed by MuJoCo: `Motor`/`Locomotion`, `Fan`, `RotarySensor`, `Imu`,
  `TWallSensors<4>`.
- Deterministic but synthetic: `Stopwatch` (sim clock), `Battery` (constant),
  `BluetoothSerial` (in-process), `Storage` (in-memory, blank every run).
- Stubs recording the last value: `Led`, `TArgb<2>`, `Buzzer`, `Button`,
  `TDipSwitch<4>`, `TTorqueSensors<2>` (always 0).

`Button` and `TDipSwitch` read `ProxyState::interface_input`, written by the
scenario or by the panel. `config/target.hpp` uses the firmware button delays,
`long_press_delay = 500` and `extra_long_press_delay = 2000`, and
`Button::update` classifies on `elapsed_time_ms` strictly greater than each, so
the scenario holds the button for 250 ms, 501 ms and 2001 ms.

The panel hands over rather than fights: the first touch of any board control
sets `driven_by_human`, and `Scenario::on_before_tick` returns early from then
on. Panel input uses ImGui **edges** (`IsItemActivated`/`IsItemDeactivated`), not
`IsItemActive`; writing every frame from the held state overwrites the scripted
press and silently turns an `extra_long` into a `SHORT_PRESS`.

## The firmware submodule

`MicrasFirmware/` sits on `feature/sim-harness`, branched from `ad34254`.
**Edit it only with intent**, in the firmware's own style
(`.clang-format`, Doxygen on every declaration), and only with changes that make
sense on the real robot too. What is there now:

- `Micras::run` sets `navigation_failed` when `push_exploring` queues nothing,
  stops locomotion and logs; `RunState` turns that into `State::ERROR` the way
  `check_crash` does. Before this the firmware popped an empty deque and crashed.
- `ActionQueuer::pop` returns the pre-built stop action instead of popping an
  empty deque, because every caller dereferences the result unchecked.
- New monitoring variables, appended after the existing ones so no id shifts:
  `FSM State`, `Grid Pose`, `Odometry State`, `Left/Right Command`,
  `Linear/Angular PID Integral`, `Odometry Linear Raw`, plus the accessors they
  need (`core::Fsm::get_current_state`, `core::PidController::get_error_acc`,
  `nav::SpeedController::get_pid_error_acc`,
  `nav::Odometry::get_raw_linear_velocity`).
- The Butterworth fixes described below.

`nav::TMaze::get_next_goal` returns the default `GridPose{{0, 0}, RIGHT}` when no
side qualifies: every open neighbour must beat the initial `current_cost` (0x1FFF),
so a cell whose open neighbours are all still at `max_cost` yields the default and
`push_exploring` then matches none of its four cases. That is reached through
odometry drift writing walls into the wrong cells, not through a bug in the search.

## Style

The firmware's `.clang-format` and `.clang-tidy`, with the tidy header filter
anchored to this repo. Doxygen on every declaration. **No comments inside
function bodies** — if something needs explaining, it goes in an `@note` on the
declaration, where a reader finds it before reading the code. Every `NOLINT`
names its check and says why. `just lint` is clean and stays clean.

## State of the tuning (2026-09-16)

`models/robot_v2.xml` plus the retuned `config/constants.hpp` **reach the goal**:
a 120 s explore on `models/maze.txt` touches a goal cell at 38.8 s, returns to
`{0, 0}` and goes IDLE at 94.6 s. The run that follows, started automatically,
still trips `ERROR` at 98.2 s. The same binary on `models/maze2.txt` (the 2019
Portugal qualifier) does not reach the goal and dies at 69.2 s, so the tuning is
not maze independent yet.

What was measured and fixed, in order:

1. **Butterworth sampling frequency and normalisation.** `core::ButterworthFilter`
   normalised the analog prototype by `f_c` rather than by `2 * pi * f_c`, and
   every call site took the 100 Hz default `sampling_frequency` while being fed at
   `loop_frequency` (959.7 Hz). The two errors compounded into a factor of
   `loop_frequency / (2 * pi * 100) = 1.527`: the real cutoffs were 1.527 times
   the configured numbers, i.e. the filters were *faster* than intended. Both are
   fixed in the firmware, `sampling_frequency` is now an explicit field with no
   default, and every `filter_cutoff` was restated as the cutoff actually in
   effect, so the tuning carried over unchanged and the fix is behaviour-neutral.
   `FollowWall::check_posts` reads `get_adc_reading`, which is unfiltered, so
   filter lag never affected post detection at all.
2. **Speed PIDs closed.** Both were `kp = 0`, i.e. identically zero. They now
   carry the firmware structure rescaled by the feed-forward gain of each axis.
   The decisive term was `ki`: `TurnAction` is open loop in time, so the 4 %
   steady-state angular error that `ki = 1` could not close inside a 1.6 s pivot
   came straight out as a 21 deg heading error on every 180 deg turn. Executed
   turn angle 0.886 -> 0.987 of the commanded one.
3. **Post correction.** Left at `post_reference = 0.44 * cell_size` and
   `post_threshold = 400`. Lowering the threshold is a clear negative:
   `-d(adc_reading)/d(distance)` on the side sensors peaks at 32 (left) and 456
   (right) over a whole 120 s run, so at 400 the correction fires about four times
   and at 100 or below it fires constantly and *injects* error. Thresholds
   400/100/50/30/20 give a first ERROR at 98.2/45.8/40.8/40.8/40.8 s on maze 1 and
   the goal is only reached at 400. The reference is what is wrong, not the
   sensitivity, and fixing it needs a measurement of where the sim post edge
   actually falls.
4. **Chassis clearance.** The `base` geom is raised 2.5 mm in `robot_v2.xml`,
   putting the chassis 3 mm above the floor instead of 0.5 mm. Floor scraping went
   from 63 % of ticks to under 1 %.
5. **Exploring curve radius.** `exploring.max_centrifugal_acceleration` 1.0 -> 2.0
   shrinks the exploring curve from a 90 mm to a 45 mm radius. A 90 mm curve does
   not fit a 167.4 mm corridor with a 66 mm wide chassis.
6. **Linear PID `ki` 10 -> 40.** `Micras::run` calls `speed_controller.reset()` at
   every action boundary, and an exploring action is 45 to 180 mm long, so the
   integral only gets 0.16 to 0.6 s to close the 5 % steady-state speed error
   before being wiped. That error is not cosmetic: `TurnAction` holds a constant
   linear speed for a fixed time, so at -5.5 % the exploring curve displaces
   41.7 mm instead of the 45 mm `ActionQueuer` assumes, and `MoveAction` measures
   distance travelled rather than position in the cell, so the missing millimetres
   never come back. First ERROR 22.3 -> 29.0 s, maximum odometry error 131 ->
   57 mm. Raising the feed-forward `linear_speed` instead changed nothing
   measurable — the PID simply unwound the same amount — and was reverted.
7. **The lateral loop was bang-bang.** This was the one that mattered.
   `FollowWall` returns `state.velocity.linear * pid(left_error - right_error)`;
   around the corridor centre that error is 5.34 per metre of lateral offset, so
   with `kp = 30` the proportional term alone reached the old `saturation = 1.0` at
   6.2 mm of offset. Past that the loop was pure bang-bang at +-v rad/s and
   limit-cycled with a 1.9 s period, half a metre of travel per swing: the robot
   crossed the centre line with its full heading error still on it, entered the
   next curve 8 deg off and walked into the wall. `saturation` 1.0 -> 4.0 keeps the
   loop proportional out to 25 mm, a full half corridor, and `kd` 0.008 -> 0.05
   damps what is left. Over a 120 s explore: lateral rms 25.0 -> 8.7 mm, first
   ERROR 29.0 -> 98.2 s, goal reached at 38.8 s instead of never. `kd` above 0.05
   lets the derivative kick on the step the reading takes when a wall ends.

**The remaining failure mode is the 180 deg spin.** `push_exploring` queues `stop`
(`cell_size / 2`), `turn_back`, `move_half`, and `turn_back` pivots about the body
origin with no safety margin. The chassis mesh puts the front tip at
`sqrt(15.5^2 + 66^2) = 67.8` mm from that origin and the corridor half-width is
83.7 mm, so the spin needs the body origin within **15.9 mm** of the cell centre.
It never is: `tools/turn_stats.py` shows every spin in every run starting 16 to
18 mm past the cell centre, because the `stop` action starts wherever the
accumulated longitudinal error left the robot and only measures 90 mm of travel
from there. Every spin therefore scrapes, and on maze 2 the spin at 25.7 s is the
first chassis contact of the run and the start of the 2.19 m odometry runaway that
kills it at 69.2 s. The spin is the least forgiving action in the repertoire and
the only one with no geometric margin; the exploring path has nothing equivalent
to the `curve_safety_margin` that `ActionQueuer::get_trim_distances` applies on
the solving path.

Worth noting alongside that: the chassis mesh in `robot_v2.xml` is 66 mm wide and
99 mm long (-33..+66 in body y), while the sibling Micrasverse repo's `src/config/constants.hpp`
models the same robot as 50 x 80 mm. The mesh is 32 % wider and 24 % longer than
the other simulator's idea of the robot, and the wheels stick out a further 4 mm
each side. If the Micrasverse numbers are the measured robot then the mesh is
simply wrong, and correcting it would take the spin margin from 15.9 to about
25 mm. That is a measurement on the real chassis, not a simulation choice, so
nothing was changed here.

## Known gaps

- **No "new run" from the panel.** It would need `RunControl::RESTART`,
  restartable listeners (the recorder would have to reopen into `run_002/`), and
  a fresh `FirmwareThread` and `Micras`, because the FSM, the maze map and the
  in-memory `Storage` all carry state. That is an `Application` lifecycle change.
- **The video path is ungated.** `just check` proves a window and a bridge change
  nothing, through `check-viewer` and `check-monitor`. There is no `check-video`,
  and `just video` writes into `runs/explore_v2`, the very directory the baseline
  comparison reads. Determinism under `--video` is believed, not tested.
- **`PlotTrace` drops its oldest sample with `erase(begin())`** rather than
  being a real ring buffer. At 2048 samples and 30 fps it does not show, but it
  is the wrong data structure.
- **Nine of the seventeen hardware tests are not built**: the human-interface
  ones (`led`, `argb`, `buzzer`, `button`, `dip_switch`), plus `storage`,
  `stopwatch`, `torque_sensors` and `comm_service`. `test_fan` needs `--button short`; the other seven run
  unscripted.
