# Handoff

For whoever picks this up next, human or agent. It says where the harness
stands, what the three rules are that must not be broken while changing it, and
the six jobs that are queued, in the order they are worth doing.

Read `README.md` for how to use the thing and `CLAUDE.md` for why it is built
the way it is. This file only covers what is *left*.

## Where it stands

The harness runs the Micras firmware unmodified against MuJoCo, deterministic
and headless for measuring, with an optional window, control panel, mp4
recorder and a WebSocket bridge that micras-monitor connects to as if it were
the radio. `just check` is the gate and it is green: 80 unit tests, config drift
against the firmware, an all-options-off build, seven hardware-test smoke runs,
a window-versus-headless CSV comparison, a bridge round trip, three scenarios
analysed and asserted, and all three compared against three recorded baselines.
`just lint` is clean, zero diagnostics.

Everything lives on branches, nothing is on `main`:

- `Team-Micras/micras-simulation`, branch `feature/simulation-harness`
- `Team-Micras/MicrasFirmware`, branch `feature/sim-harness` (the submodule)

To build, you need MuJoCo unpacked at `$HOME/.mujoco/mujoco-3.3.6` (or pass
`-DMUJOCO_DIR=`), then `just build`. Clone with `--recurse-submodules`; the
submodule gitlink already points at the firmware commits the harness needs.

The documentation was fact-checked against the code and its findings applied.
The bridge was reviewed too, and it came back with a reproduced CRITICAL — that
is job 0 below, and it should be fixed before anyone demos the monitor.

## The three rules

Break these and the harness stops being worth having.

**1. A run is reproducible.** Two runs of the same binary with the same
arguments produce byte-identical `data.csv` and `meta.json`. Nothing may make
the firmware's view of the world depend on wall time, on a window being open, on
a client being connected, or on how fast the machine is.

**2. A refactoring must not move a byte.** If `data.csv` stops matching
`baseline/v2` after a change that was supposed to be behaviour-neutral, the
change is wrong. Revert it. Do not re-record the baseline to make the diff go
away — that is the one move that destroys the whole safety net, and it is
exactly the tempting one.

**3. A change that legitimately alters physics or columns gets a NEW baseline
version, never an overwrite.** `just record-baseline` refuses to overwrite, on
purpose. Bump `baseline` in the `justfile` to `baseline/v3`, move the old one
into `previous_baselines`, and the older recordings stay comparable as a header
prefix. This is the correct path for job 1 below.

Corollary of rule 3: new CSV columns go at the **very end**. The prefix check is
literally a header prefix, so appending a pool variable in the middle would
shift every `proxy_*` column right and break it.

## Job 0 — The bridge hangs the simulation, and two things around it lie

An independent review of the bridge reproduced all of this, with a gdb
backtrace and with the real binary. Do this before the rest.

**The hang (critical).** `WebSocketServer::broadcast` calls `sendBinary`
synchronously on the simulation thread (`src/sim/bridge/web_socket_server.cpp`,
from `MonitorBridge::on_after_tick`). IXWebSocket's `flushSendBuffer` loops on
`isReadyToWrite(10)` with **no deadline and no give-up**; the only thing that
would break it out is a `stop()` that the blocked thread itself would have to
call. So a monitor that stops draining — a throttled browser tab, a paused
debugger, a stalled tunnel — applies TCP backpressure and the run freezes
indefinitely. Reproduced: row count frozen for 15 s and 18 s across three runs
with the firmware thread parked in `futex_do_wait`, resuming only when the
client's connection actually died. The `FirmwareThread` watchdog does not cover
it, because the firmware thread is the one waiting. `just check-monitor` in CI
would hang forever rather than fail.

This is not exotic. Measured throughput to a *reading* client is about
3.2 MiB/s, roughly 232 000 packets/s, because the harness runs about 12× real
time. Anything slower than that is backpressure.

The fix is to get `sendBinary` off the simulation thread: hand `outgoing` to a
bounded ring drained by a dedicated sender thread, drop the oldest frames when
it is full, and count the drops. The monitor is a view, not a ledger. Coalescing
several ticks into one frame is worth doing at the same time — at ~12 000
frames/s the framing overhead is most of the traffic. See also job 4, which is
the same problem seen from the radio's side.

**A monitor-driven run claims to be reproducible when it is not (high).**
`application.cpp` computes `interactive` from the viewer alone. `MonitorBridge`
queues whatever the monitor sends straight into the firmware and never sets
`driven_by_human`, so the scenario also keeps scripting alongside the human.
Reproduced: two 20 s runs, one with a monitor injecting a `SERIAL_VARIABLE`
packet, produced different CSVs and both reported `interactive: false`. A
perturbed run is currently indistinguishable from a clean one and would be
accepted by `just check` or recorded as a baseline. Set `driven_by_human` (or a
`monitor_input_seen` flag) the first time the bridge queues a packet and OR it
into `RunMetadata::interactive`. The plan requires exactly this.

**The check that was supposed to catch that can pass while testing nothing
(high).** When port 8080 is taken the binary prints `monitor disabled: Address
already in use` and **exits 0**, so `check-monitor`'s `runs/monitor/quiet` is
just another bridge-less run and the comparison passes trivially. Reproduced.
Make the recipe fail when the bridge could not listen, and pick a per-invocation
port instead of the hardcoded default.

**Dropped packets are invisible (high).** `SerialBus::queue_for_firmware`
silently discards everything past 256 queued packets and increments a counter
that **nothing ever reads**. Flooding the bridge with 100 000 valid packets let
256 through and ~99 744 vanished without a trace in `meta.json`, stderr or the
CSV. Surface `dropped_packets` and the bridge's own `resync_count` in the
metadata next to `telemetry_resyncs`.

Smaller ones from the same review, worth folding in while you are there: neither
bridge class has any test; `server` should be declared **last** in
`MonitorBridge` so the language guarantees the destruction order the destructor
body currently guarantees by hand; the bridge should reject frames failing
`comm::Packet::is_valid` before queueing them, because a client can otherwise
occupy the one-packet-per-tick slot with garbage and delay the scenario's own
injection; and `--monitor`/`--monitor-port` are parsed inside
`apply_viewer_option`, which is the wrong place and makes that function's own
`@brief` false.

What the review confirmed as solid, so you do not need to re-derive it: the wire
protocol matches the firmware exactly (checked three ways, including a payload
full of control bytes), determinism holds for a no-client run, a refused port
and a well-behaved client, hostile input neither crashes nor leaks across 18 MB
of adversarial traffic, and the server binds to loopback only.

## Job 1 — MuJoCo 3.3.6 to 3.13.0

Worth doing and nothing structural is in the way. 3.13.0 is real and current
(released 2026-09-09); the pin at 3.3.6 is just where the work started.

MuJoCo is located in exactly one place, `cmake/mujoco.cmake`, which points
`MUJOCO_DIR` at `$HOME/.mujoco/mujoco-3.3.6` and imports the shared library by
hand because the tarball ships no CMake config. `application.cpp` checks
`mj_version() != mjVERSION_HEADER` at startup, so a library/header mismatch is
caught loudly rather than as corruption.

Expect this to change the numbers. Solver, contact and integrator work landed
across ten minor releases; `data.csv` will almost certainly differ. **That is
fine and it is rule 3, not rule 2.** The procedure:

1. Unpack 3.13.0 next to the old one and bump the default in
   `cmake/mujoco.cmake`. Keep the old directory around until the end.
2. `just build`. Fix whatever the API moved. Compile errors will concentrate in
   `src/sim/core/mujoco_world.cpp`, `src/sim/recording/ground_truth.cpp` (it
   touches `mjData` contact arrays directly) and `src/sim/view/*` (`mjv_`/`mjr_`
   are the most churn-prone surface).
3. `just test` and `just lint` first — those must pass unchanged, because
   nothing in them depends on physics.
4. `just check` will now fail at `compare-baseline`. Before accepting that,
   confirm the difference is *physics* and not a bug you introduced: run
   `just analyze` on the three scenarios and compare `report.json` against the
   old one. Warnings must still be zero, `telemetry_resyncs` zero, no
   non-finite samples, and the explore run must still behave qualitatively the
   same. A solver change moves trajectories a little; a mistake usually shows up
   as warnings, NaNs, or the robot falling through the floor.
5. Only then bump to `baseline/v3` and `just record-baseline`. Keep v2, v1 and
   v0 in `previous_baselines`.
6. Re-check the tuning claims in `CLAUDE.md`'s "State of the tuning" section.
   Those numbers (goal at 38.8 s, first ERROR at 98.2 s, lateral rms 8.7 mm)
   were measured under 3.3.6 and will move. Restate them or mark them as
   measured under the old version — do not leave them silently wrong.

`mujoco_version` is recorded in `meta.json` but is deliberately not one of
`compare_run.py`'s `BEHAVIOUR_FIELDS`, so the provenance is kept without the
version itself failing a comparison.

## Job 2 — Get Micras out of the command line

The complaint is fair: `Cli` knows far too much about this specific robot.
`--command none|explore|solve|calibrate` hardcodes four firmware pool variable
names; `--dip fan=0,diagonal=0,boost=0,risky=0` hardcodes the four switch
meanings in a `std::array` in `src/sim/app/cli.cpp`; `--no-fan` is a flag for
one actuator. Every one of those is a Micras fact compiled into the harness.

What they all actually are: a script of timed writes to the board interface,
plus one timed write to a pool variable. That is the general shape, and it
belongs in **data**, not in C++:

- a run script (a small file, or a repeatable `--at <t> <action>` flag) with two
  primitives: set pool variable `<name>` to `<value>` at time `t`, and hold the
  button for `<ms>` starting at time `t`, plus an initial switch word;
- `ScenarioScript`'s `Command` enum and the `dip_names` array deleted, replaced
  by names carried in the script;
- the current flags kept as thin sugar over it, or dropped, but only after the
  scripts that use them are updated — `tools/` and the `justfile` depend on the
  current spelling of `--command explore`.

Do it under rule 2: an identical scenario expressed the new way must produce a
byte-identical `data.csv`. That is a strong test and it is cheap to run.

## Job 3 — Get Micras out of the CSV recorder

Same complaint, bigger blast radius, and the reason to do it after job 2.

`src/sim/recording/ground_truth.cpp` is where it concentrates: wheel joint
names, the left/right wheel geoms, the caster, the `base` geom, four wall
sensors, the body forward axis being +y. A different robot, or the same robot
with a renamed geom, needs a C++ edit today.

The shape to aim for is a column source interface — something with `headers()`
and `values()` — where `GroundTruth` is *built from a description* (which bodies,
which geoms, which sensors) rather than from constants, and the description
lives with the model. `CsvRecorder` already composes three sources (ground
truth, `ProxyState`, the firmware pool), so the seam exists; it is `GroundTruth`
itself that is hardcoded.

Two things to preserve while doing it, both non-obvious and both load-bearing:

- the exact column order and the `%.9g` formatting in `CsvWriter`, or rule 2
  fails immediately;
- the `nan` versus `0` distinction in `left_penetration`/`right_penetration`.
  `nan` means "no contact at all" and `0` means "touching exactly". Collapsing
  them loses real information that `analyze.py` reads.

## Job 4 — The radio is lying about its bandwidth

This one is not cosmetic and it is the one most likely to cost a race.

Measured on this branch: the firmware pushes **290 bytes per tick** through the
serial proxy, which at the 959.7 Hz loop is **about 278 kB/s, roughly 2.8 Mbaud
at 8N1**. A classic SPP link at 115200 baud carries 11.5 kB/s. The harness is
running about **24 times over** what the real radio can sustain, and nothing
anywhere notices.

So the simulated robot reports state the real robot physically cannot send. Any
conclusion about telemetry drawn in simulation is optimistic by more than an
order of magnitude, and the firmware's own `enable_mask` — which exists exactly
to ration this — is never under pressure here, so it is never exercised.

Two ways to fix it, and they are not equivalent:

- **Cheap and non-breaking:** count the bytes and report the achieved rate in
  `meta.json`, plus a threshold that flags a run exceeding a configured link
  budget. New metadata field only, so `data.csv` does not move and rule 2 holds.
  Do this first — it makes the problem visible immediately.
- **Honest and behaviour-changing:** give `SerialBus` a byte budget per
  simulated second and make it actually back up when the firmware exceeds it,
  the way a real UART does. This changes what the firmware sees, so it is a new
  baseline under rule 3, and it will probably expose firmware behaviour nobody
  has looked at yet. That is the point of it.

The budget should be configurable, because the answer depends on which radio the
robot actually carries.

## Job 5 — The singleton

`sim::SimulationContext::instance()` is a function-local static. It is not there
because anyone liked it; `CLAUDE.md`'s shadow-proxy section and Appendix A of
`HARNESS_REDESIGN_PLAN.md` have the full argument, but the short version is that
the firmware builds its proxies as member initialisers from `const` globals in
`target.hpp`, so at construction time the only route to the world is something
reachable from a `Config`. A static class would have the same property and the
same problem — the issue is process-wide reachability, not the spelling.

It is already fenced: exactly three places name `instance()` — `target.hpp`,
`Application`'s member initialiser, and the context's own file. Everything else
receives facets through its `Config`, and the tests build their own context,
which is what makes the fence real rather than aspirational.

Removing it properly needs the firmware to grow an explicit proxy-layer contract
(a `Micras` that can be constructed with its dependencies). When that lands,
`Application` constructs and injects the context and nothing else changes.
Until then, tightening the fence is worth more than replacing the mechanism.

## Also queued, smaller

- **The video path is ungated.** `just check` proves a window and a bridge change
  nothing; there is no `check-video`. Worse, `just video` writes into
  `runs/explore_v2`, the directory the baseline comparison reads. Adding a
  `check-video` step is a small job and closes a real hole.
- **No "new run" from the panel.** It needs `RunControl::RESTART`, restartable
  listeners, and a fresh `FirmwareThread` and `Micras` — the FSM, the maze map
  and the in-memory `Storage` all carry state. It is an `Application` lifecycle
  change, not panel work.
- **`PlotTrace` is not a ring buffer.** It drops its oldest sample with
  `erase(begin())`. Invisible at 2048 samples and 30 fps, still the wrong data
  structure.
- **Nine of the seventeen firmware hardware tests are not built** — the
  human-interface ones plus `storage`, `stopwatch`, `torque_sensors` and
  `comm_service`.

## Traps

Short list of things that have already cost a debugging session here.

- **EGL and GLX cannot both be current on one thread.** `--viewer --video`
  requires `VideoRecorder` to release its EGL context after every capture and
  `MujocoViewer::draw` to make the GLFW context current. Break the pair and you
  get `BadAccess X_GLXMakeCurrent` and a zero-byte CSV.
- **The panel must use ImGui edge detection** (`IsItemActivated` /
  `IsItemDeactivated`) for the board button. Writing it every frame from
  `IsItemActive()` overwrites the scripted press and silently turns an
  `extra_long` press into a short one.
- **The `Stopwatch` spin counters must survive a reset.** `Fan::update` resets
  its own stopwatch every call, so a counter that reset with it would never
  reach its threshold and `test_fan` would spin forever.
- **Never inject more than one serial packet per tick.** The firmware clears its
  whole receive deque after framing one packet.
- **Use `xvfb-run -a` for anything with a window.** Running window tests on a
  live display fights whoever is using the machine, and it has already killed a
  test run here.
