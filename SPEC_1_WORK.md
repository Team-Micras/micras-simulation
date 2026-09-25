# Spec 1 — Work with no decisions

2026-09-23 · companion to [`SPEC_2_DECISIONS.md`](SPEC_2_DECISIONS.md)

This spec is the work itself: what to delete, fix, move and build, and in what
order. Every design choice behind it is recorded in spec 2, where all fourteen
decisions are now made. Section 10 lists what is deliberately deferred. Steps 1 to 6
are implemented on `refactor/cleanup` (2026-09-24, not yet committed); section 12
records where the implementation departs from the text and what it found.

The direction is decided. This repository becomes the team's simulator for every
robot built on `micras_hal`: the micromouse now, and a line follower later. The code
is organised in three layers, all in this repository:

| Piece | What it is | Where |
|---|---|---|
| **Engine** | World, clock, run loop, devices, arenas, recording, viewer, bridge. It knows no robot and no HAL. | The repository root |
| **`micras_hal` host backend** | `micras_hal` implemented on a PC | `hal_host/`; it moves next to `micras_hal` once that has its own repository (spec 2 D3) |
| **Robot target** | One per robot: its firmware as a submodule, device bindings, robot description, scenarios, baselines | `targets/<robot>/`, starting with `targets/micras/` |

State this spec was written against:

| Repository | Ref |
|---|---|
| micras-simulation | `refactor/cleanup` @ `85cc1ab` |
| MicrasFirmware submodule (pinned) | `9136d7b` on `feature/sim-harness`, branched from `ad34254` |
| MicrasFirmware port target | `high-level-review` @ `0be27df`, 174 commits past the pin |

In this spec, **"Micras"** alone means the micromouse robot. The `micras_` prefix of
the libraries (`micras_hal`, `micras::sim`) is the team's, and stays.

---

## 1. Ground rules

There are two kinds of items:

- **Pre-port work (P)**: done on the currently pinned firmware. This is sections 2
  to 8.
- **Port requirements (R)**: constraints the port to `high-level-review` must meet
  under every option in spec 2. This is section 9.

Rules for pre-port work:

- **P1. Nothing moves a byte.** After every step, `idle` and `explore_v2` still
  match `baseline/v2`:
  - `data.csv` is byte-identical.
  - The ten behaviour fields of `meta.json` are identical (`BEHAVIOUR_FIELDS` in
    `tools/compare_run.py`).
  - `model_path` and `args` are not compared, so moving files is allowed.
    `model_sha256` *is* compared, so not one byte of `models/robot_v2.xml` or
    `models/maze.xml` may change before the port, not even a comment.
- **P2. The firmware is not touched** before the port.
- **P3. Move Micras code, don't redesign it, when the port will rewrite it.** Code
  with a known death date (section 9) is relocated as-is, not polished: the old
  serial protocol, the shadow proxies, `ProxyState`.

---

## 2. Repository cleanup

| # | Path | Evidence | Action |
|---|---|---|---|
| C1 | `models/experiments/` (43 files, 228 KB) | Nothing loads them: not the justfile, CMake, tests or tools. The only mention is a comment at `robot_v2.xml:17`. `experiments/maze.xml` is a byte copy of `models/maze.xml`. The write-up they belong to (`../HARNESS_PLAN.md`) is not in any repository. | Delete. Git history keeps them. |
| C2 | `baseline/v0/`, `baseline/v1/` (6.5 of 11 MB) | Their headers are strict prefixes of v2's (65 → 80 → 85 columns), and v2 is compared byte for byte, so a prefix check against them can only fail when the v2 check already fails. The port invalidates all three anyway. | Delete. Remove `previous_baselines` and its inner loop from `compare-baseline`. |
| C3 | `models/robot_legacy.xml` and the `explore_legacy` scenario | A second robot kept alive only to feed the gate: `legacy_model`, `run-explore-legacy`, `explore_legacy` in `check_runs`, `baseline/v2/explore_legacy/`, the `check_run.py` arguments in `check`, and the drift allowlist entries justified by "robot_legacy.xml". Nobody develops against it. | Delete all of them. |
| C4 | `tools/run_stats.py`, `tools/turn_stats.py` | Referenced only by the README. They are one-off investigations of the old navigation (post corrections, `turn_back` spins) with a hard-coded `CELL=0.18`, maze path and chassis hexagon. | Delete, and remove the README paragraph. |
| C5 | `HANDOFF.md` (442 lines) | Mostly duplicates `CLAUDE.md`. It cites `../HARNESS_PLAN.md` and `../HARNESS_REDESIGN_PLAN.md`, which do not exist. See the table below for where its unique content goes. | Delete once its content is placed. |
| C6 | `.gitignore` entry `.omc/` | Left over by another tool. | Remove the entry. |
| C7 | `models/gen_maze.py` | A generic tool living among the Micras models. | Move to `tools/gen_maze.py`. |

Where each part of `HANDOFF.md` goes:

| Part | Destination |
|---|---|
| Job 0 (bridge hang, lying flags) | B1–B5 below |
| Job 1 (maze 2 not finished) | Obsolete: it is about the old navigation |
| Job 2 (MuJoCo upgrade) | R4 |
| Jobs 3, 4, 6 (Micras out of CLI and CSV, the singleton) | Sections 6 and 7, and spec 2 D1 and D2 |
| Job 5 (radio bandwidth) | Spec 2 D8 |
| Firmware backlog | Obsolete at the port: every item is old navigation or old protocol |
| "Not there" list | Spec 2 D12 and D14 |
| Traps | Stay in `CLAUDE.md`; most of them disappear with spec 2 D2 |

---

## 3. Bugs

| # | Where | Bug | Fix |
|---|---|---|---|
| B1 | `WebSocketServer::broadcast`, called from `MonitorBridge::on_after_tick` | `sendBinary` runs synchronously on the simulation thread. IXWebSocket's `flushSendBuffer` loops on `isReadyToWrite(10)` with no deadline, so a client that stops draining freezes the run forever. This was reproduced (HANDOFF job 0). | Push outgoing frames into a bounded queue drained by a sender thread. Drop the oldest frame when it is full, and count the drops. |
| B2 | `application.cpp`, `RunMetadata::interactive` | `interactive` is computed from the viewer alone. Bytes from a monitor reach the firmware without marking the run, so a perturbed run is recorded as reproducible. | The bridge marks the run interactive on the first byte it queues. OR that into `interactive`. |
| B3 | `just check-monitor` | When port 8080 is taken, the binary prints `monitor disabled` and exits 0. The "quiet" run is then bridge-less, and the comparison passes while testing nothing. | Fail when the bridge does not listen. Use a free port per invocation. |
| B4 | `just check-viewer` | With no display and no `xvfb-run`, the recipe prints "skipping" and exits 0, so the gate goes green without testing. | Fail unless an explicit skip variable is set. |
| B5 | `SerialBus::queue_for_firmware` | Packets beyond 256 are discarded, and the counter `dropped_packets` is never read. | Report `dropped_packets` and the bridge's frame drops (B1) in `meta.json`, as non-behaviour fields. |
| B6 | `tools/analyze.py:618–621` | Always loads `<model dir>/maze.txt`, so maze-2 runs are plotted and scored with maze 1's walls. `GOAL_CELLS` is fixed at (7, 8). | Resolve the maze from `meta.json`, or from the model's maze include. Fail when it cannot be found. |
| B7 | `just video` | Writes into `runs/explore_v2`, the directory `compare-baseline` reads. | Write into `runs/video/`. |
| B8 | `cmake/firmware.cmake:7–10` | The firmware SHA is read once at configure time. After a submodule bump without reconfiguring, `meta.json` names the wrong firmware. `baseline/v2` records `ad34254` while the submodule is at `9136d7b`, and today you cannot tell which is true. | Regenerate the SHA at build time with a custom command. It must write the header only when the SHA changed, so it does not rebuild every time. |
| B9 | `MonitorBridge` | Destruction order is guaranteed by hand in the destructor body. | Declare `server` last, so the language guarantees it. |
| B10 | `cli.cpp`, `apply_viewer_option` | `--monitor` and `--monitor-port` are parsed inside the viewer option handler, which makes its `@brief` false. | Parse them in their own handler. |

Not fixed, because the port deletes the code: bridge frame validation against
`comm::Packet::is_valid`, and the old-protocol firmware backlog items.

---

## 4. Wrong statements

| Where | Says | Truth | Fix |
|---|---|---|---|
| `CLAUDE.md`, shadow proxy layer | "Exactly three places name" `SimulationContext::instance()` | There is a fourth: `tests/include/test_core.hpp:54` | Correct the text |
| `CLAUDE.md`, CSV | `fw_loop_time` is "the firmware's own loop time" | It is the hard-coded constant `firmware_loop_time = 1042.0e-6` in `ground_truth.cpp:20` | Correct the text; section 7 derives it from the clock |
| `CLAUDE.md`, Stopwatch | Documents two spin guards | `Button::update` has a third one, `max_repeated_updates = 2`, which calls `yield_tick()` (`button.cpp:31–35`) | Document it |
| `config/constants.hpp:4,8` | "constants for the ROS-simulation baseline" | These are the MuJoCo harness tuning | Correct the comment |
| `tools/check_config_drift.py:34` | Linear `ki` is "10x the firmware ki" | The harness value is 40 | Correct the reason text |
| `tools/check_config_drift.py:53` | `wall_thickness` "matches models/maze.xml" | The harness uses `0.012` (`constants.hpp:110`); `maze.xml`, the `gen_maze` default and the firmware use `0.0126` | Correct the reason text only. Changing the value would move bytes (P1), and the port replaces the constants anyway. |
| `tools/check_config_drift.py:55–66` | "ROS-sim motion budget" | MuJoCo | Correct |
| `tools/analyze.py:9,11,21` | Cites `src/sim/recorder.cpp::to_euler` and `robot_legacy.xml` | The code is `recording/ground_truth.cpp`, and C3 deletes the legacy model | Correct |
| `models/robot_v2.xml:3,16` | "Four changed lines", "constants.hpp has kp=0", and it cites `runs/v2_explore` and `HARNESS_PLAN.md` | There are six FIX blocks, and `kp` is 120 | **Not before the port**: `model_sha256` is compared (P1). The model is replaced at the port (spec 2 D7). |
| `README.md:182` | "plus the tuning experiments" | Deleted by C1 | Remove |

---

## 5. Dead code and duplication

| Item | Where | Action |
|---|---|---|
| `WebSocketServer::client_count`, `MonitorBridge::client_count` | `web_socket_server.hpp:77`, `.cpp:86,114`, `monitor_bridge.hpp:65` | Never called: delete |
| `Scenario::command_delivered` | `scenario.hpp:156` | Never called: delete |
| `MujocoWorld::is_loaded` | `mujoco_world.hpp:52` | Never called: delete |
| `Clock::reset`, `Clock::total_ticks(double)`, `SerialBus::clear`, `SerialBus::pending_packets` | `clock.hpp`, `serial_bus.hpp` | Called only by tests. Delete each one whose test exists only to exercise it; keep the ones a test needs to observe behaviour that matters. |
| Micras FSM state-name table, written twice | `control_panel.cpp:52`, `video_recorder.cpp:162` (with different NaN handling) | Keep one table in the Micras target (section 6) |
| DIP switch names, written twice | `cli.cpp:38`, `control_panel.cpp:39` | Keep one definition in the Micras target |
| `PlotTrace` drops its oldest sample with `erase(begin())` | `view/` | Replace with a fixed-size ring buffer |

---

## 6. Layers and folders

### 6.1 Layout

```
<repo>/
  CMakeLists.txt        engine project; adds each robot target
  cmake/                mujoco.cmake, third_party.cmake, lint.cmake, micras_simConfig.cmake.in
  engine/               include/micras/sim/..., src/
                        world, clock, run loop, recording, run metadata, crash reporter,
                        byte streams, devices (6.3), arenas/maze
  view/                 viewer, generic panel, video       (optional: MICRAS_VIEWER, MICRAS_VIDEO)
  bridge/               WebSocket <-> byte stream          (optional: MICRAS_BRIDGE)
  app/                  CLI and wiring, as a library exposing micras::sim::run(); no main()
  tests/                engine unit tests, with a tiny test robot in tests/models/
  tools/                compare_run.py, check_run.py, analyze.py (generic part), gen_maze.py
  hal_host/             the micras_hal host backend: written at the port (spec 2 D1, D3)
  targets/
    micras/             everything that knows the micromouse
      CMakeLists.txt    links only the engine's targets and hal_host
      justfile
      MicrasFirmware/   the submodule, moved here from the root
```

### 6.2 Layering rules

| Rule | Statement | How it is enforced |
|---|---|---|
| M1 | The engine (`engine/`, `view/`, `bridge/`, `app/`) includes no firmware header: nothing from `micras/hal/`, `micras/proxy/`, `micras/nav/`, `micras/core/` or `micras/comm/`, nor `micras/micras.hpp`, `target.hpp`, `constants.hpp` or `robot.hpp`. | Structurally: the engine's CMake targets have no firmware include directory, so such an include does not compile. On from step 4 (section 11). |
| M2 | Nothing outside `targets/` names a specific robot: no `micras::Micras`, no Micras state or variable names, no Micras MuJoCo body or geom names, no board-v1 names. | `just check-generic`, a `git grep` for a maintained list of those identifiers outside `targets/`, which must print nothing. On from step 5. |
| M3 | Each target uses the engine only through its CMake targets (`micras::sim_engine`, `micras::sim_view`, `micras::sim_bridge`, `micras::sim_app`, the `hal_host` sources) and its public headers, never through relative paths into the engine folders. | Review; the engine's targets carry every include directory a target needs. |
| M4 | `hal_host/` includes only `micras_hal` headers and its own. It names no proxy, no navigation code and no robot. | The same structural check as M1, from the port on. |
| M5 | Everything a target needs lives under its folder: models, scenarios, baselines, config, tools, tests, and its own README. | Review. |
| M6 | Target recipes live in `targets/<name>/justfile`. | The root justfile loads each as a just module (`mod micras 'targets/micras'`), so `just micras check` works. |
| M7 | Arena code (today only the maze) sits in `engine/arenas/maze/`. The run loop, recorder and viewer assume no maze, so a line track can be added beside it (spec 2 D9). | Review. |

### 6.3 What goes where

Files marked "as-is" die or are rewritten at the port (P3).

| Today | Destination | Notes |
|---|---|---|
| `src/sim/core/{clock,mujoco_world,simulation,serial_bus,firmware_thread}` | `engine/` | The run loop and `IRunListener` are robot-agnostic. `FirmwareThread` stays: at the port it becomes the handover the host `Timer` triggers (R16). |
| `SimulationContext` | Split | World, clock and bus stay in the engine as a plain aggregate with no singleton (`RunContext`). `instance()`, `require_context()` and `yield_tick()` move to `targets/micras` as-is: they exist only for the shadow proxies. |
| `proxy_state.hpp` | `targets/micras/` as-is | It is the Micras board shape: 4 DIP switches, 2 ARGB LEDs, the fan override. |
| `scenario.{hpp,cpp}` | `targets/micras/` as-is | Micras commands and button delays; it sits in `core/` today but builds into `micras_sim`. |
| `telemetry/packet_framer`, `telemetry/telemetry` | `targets/micras/` as-is | The old protocol (R8). |
| `recording/{csv_writer,run_metadata}` | `engine/` | Robot-agnostic. |
| `recording/{ground_truth,csv_recorder}` | `engine/` | After the parameterisation in section 7. |
| `app/{application,cli,crash_reporter}` | `app/` | After the CLI extension in section 7. |
| `app/hardware_test_main.cpp`, `src/main.cpp` | `targets/micras/` | Micras entry points. |
| `bridge/web_socket_server` | `bridge/` | Robot-agnostic. |
| `bridge/monitor_bridge` | Split | The byte-stream bridge goes to `bridge/`. Its `PacketFramer` use stays in `targets/micras` as-is. |
| `view/{mujoco_viewer,video_recorder,view_options}` | `view/` | After section 7. |
| `view/control_panel`, the video overlay text | Stay put until step 5 | Split declaratively (spec 2 D12). |
| `include/micras/proxy/`, `src/proxy/`, `config/` | `targets/micras/` as-is | Shadow layer, deleted at the port (R15). |
| `cmake/firmware.cmake`, `cmake/hardware_tests.cmake` | `targets/micras/cmake/` | |
| `tests/unit/{packet_framer,telemetry,proxy_boundary,shadow_conformance}_test.cpp`, `tests/include/test_core.hpp` | `targets/micras/tests/` | |
| `tests/unit/{cli,csv_recorder,scenario}_test.cpp` | Split | Robot-agnostic parts stay in the engine; Micras parts (DIP names, `explore`) move. |
| `tests/unit/simulation_test.cpp` | `tests/` | Switched from `robot_v2.xml` to the tiny test robot. |
| `models/` (`robot_v2.xml`, `robot_v2_maze2.xml`, `maze*.xml`, `maze*.txt`) | `targets/micras/models/`, **unchanged and together** | `robot_v2.xml` includes `maze.xml` by relative path, so they move together to keep `model_sha256` (P1). Mazes get their arena home at the port (R7). |
| `tools/{check_config_drift,monitor_probe}.py` | `targets/micras/tools/` | Both die at the port (R8, R15). |
| `tools/analyze.py` | Split | The robot-agnostic part (contacts, slip, attitude, trajectory, run metadata) stays in `tools/`. The Micras part (`GOAL_CELLS`, `RUN_STATE_ID`, `fsm_state`, `grid_pose_*`, `odometry_*`) becomes `targets/micras/tools/analysis.py`. `analyze.py` loads it when `meta.json` names the target, or when `--plugin` is given. |
| `tools/{compare_run,check_run}.py` | `tools/` | Robot-agnostic. |
| `justfile` | Split | Engine recipes stay at the root; Micras recipes move to `targets/micras/justfile` (M6). |

---

## 7. Removing robot assumptions from the engine

Each change below has one obvious design and keeps the CSV header and contents
byte-identical (P1).

| Code | Micras assumption | Change |
|---|---|---|
| `GroundTruth` | Body `"micras"`; geoms `left_wheel`, `right_wheel`, `"front wheel"`, `base`; actuators `motor_left/right`; forward axis +y (`ground_truth.cpp:179`); `firmware_loop_time = 1042e-6` | The target supplies a `GroundTruthConfig{body, wheels[], chassis_geoms[], actuators[], forward_axis}` with a column label per item, so the header is unchanged. The loop time comes from `Clock::us_per_tick()`, which gives the same value. |
| `CsvRecorder::proxy_columns` and `PoolTelemetry` | Hard-coded `wall_adc_*`, `dip_*`, `fan`, and a nullable `Telemetry*` | A `ColumnSource` interface (`names()`, `append(row)`). The recorder writes tick, time, ground truth, then the target's sources in registration order. Micras registers the pool source, then the proxy source, which is today's order. The `PoolTelemetry` enum is deleted. |
| `ViewConfig` | Defaults `camera{"side tracking"}` and `us_per_tick{1042}` | Camera defaults to `free` and the target may set its preferred one. `us_per_tick` has no default and comes from the clock, as the app already passes it. |
| `MujocoViewer` | Perturbs body `"micras"`; title "micras simulation" | The robot body comes from `GroundTruthConfig.body`; the title from the target name. |
| `Cli` | `--command`, `--command-at`, `--dip`, `--button`, `--button-at`, `--no-fan`; usage text says `micras_simulation` | The target registers its options (`name, argument, help, handler`) with the app. The usage text takes the program name from `argv[0]`. The robot-agnostic flags stay in the app: `--model`, `--out`, `--seconds`, `--ticks`, viewer, video, monitor. |
| `Clock::ticks_for_elapsed_ms` | Exists only to reproduce the Micras `Button` classification | Moves to `targets/micras` next to the scenario. |
| `Application` | Includes `constants.hpp` and `target.hpp` for `loop_time_us` and `button_config` | The target passes the loop period and its own listeners. The app includes no firmware headers (M1). |
| `micras_sim_view` | Calls `Telemetry::value_of`, defined in `micras_sim`, without linking it; it builds only because the app links both | The engine defines a `VariableSource` interface (`value_of(name)`) that telemetry implements. The view depends on that interface only. |
| `micras_sim_bridge` | Links all of `micras_sim`, and so the firmware shadow, just to reach `PacketFramer` | Solved by the `monitor_bridge` split in 6.3. |

---

## 8. Gates, build and tooling

| # | Addition | Why |
|---|---|---|
| G1 | `just check-generic` (M2), plus the structural M1 and M4 checks | Keeps each layer free of the layers above it. |
| G2 | *Deferred:* CI | Out of scope for now (section 10). Every gate here is a `just` recipe, so CI can run them unchanged later. |
| G3 | MuJoCo fetched automatically when `MUJOCO_DIR` is not set | `cmake/mujoco.cmake` expects a manual unpack at `$HOME/.mujoco/mujoco-3.3.6`. Download the pinned release tarball, check its SHA-256, and keep the `MUJOCO_DIR` override. MuJoCo is third-party, so a CMake fetch is right for it; dependencies you own are submodules (spec 2 D6). |
| G4 | `just check-video`: the same scenario with and without `--video`, CSVs compared byte for byte | The video path is the only output path not gated. It needs EGL: a GPU, or a surfaceless Mesa EGL. |
| G5 | Bridge tests: a client that never reads must not stall the run (B1); a taken port must fail the recipe (B3) | Neither bridge class has a test today. |
| G6 | Firmware SHA at build time | B8. |

---

## 9. Requirements for the port, valid under every option

The port moves the firmware from `9136d7b` to `high-level-review` and implements
spec 2's decisions. The requirements below are the ones that are easy to miss; R15
and R16 spell out D1 and D2.

| # | Requirement | Reason |
|---|---|---|
| R1 | The submodule tracks `high-level-review`, and `feature/sim-harness` is dropped | Its patches are obsolete. `ActionQueuer` and the old monitoring variables no longer exist, and the Butterworth fix landed upstream (`9717dcf`). Check each patch against HEAD before dropping the branch. |
| R2 | A new baseline epoch, `v3`; `baseline/v2` is deleted | The loop period, protocol, columns and state ids all change, so "only append columns" cannot hold across the port. From v3 on, "a refactoring must not move a byte" applies again. Storage and comparison follow spec 2 D10. |
| R3 | The physics timestep divides the new 125 µs loop | 521 µs does not. Pick 125 µs or 62.5 µs by measuring contact stability (`warnings_total`, penetration, airborne fraction), not by preference. `Clock::from_model` already rejects a non-divisor. |
| R4 | Upgrade MuJoCo from 3.3.6 to the current release in the same step (HANDOFF job 2 names 3.13.0) | The baseline epoch changes anyway, so the upgrade costs nothing extra now and a full re-baseline later. |
| R5 | No constant and no duplicated columns in the new schema | Today `fw_loop_time` and `loop_time` are constant, `rotary_sensor_left` equals `proxy_encoder_left`, and the two odometry velocities equal their `odometry_state_*` twins on 100 % of rows. Rule: a column constant across every standard scenario is not recorded. At 8 kHz a 120 s run has about 960 000 rows (format: spec 2 D11). |
| R6 | State names, state ids and variable names come from the firmware, never from engine code | The ids changed: 13 states, with `ERROR` at 12 instead of 6. |
| R7 | Robot and arena are separate inputs, composed when the model loads (MuJoCo's spec attach), so `--maze` actually selects the maze | `--maze` is only written to `meta.json` today, and `robot_v2_maze2.xml` is a 150-line copy that exists only to switch the maze. Delete it. Maze files move to `engine/arenas/maze/` (M7). |
| R8 | The old protocol is deleted | `PacketFramer`, `Telemetry` (`SERIAL_VARIABLE_MAP_*`), `monitor_probe.py`, the `SERIAL_VARIABLE` injection behind `--command`, the one-packet-per-tick queue in `SerialBus` (the new `comm::Link` keeps leftover bytes), and `check_config_drift.py` (its allowlist names configs that no longer exist). |
| R9 | The bridge carries raw bytes both ways and does no framing | micras-monitor's `feature/bluetooth-link` branch already speaks the new COBS protocol over a WebSocket byte stream. |
| R10 | Every proxy checked by `Micras::check_initialization()` reports `was_initialized() == true` | Otherwise INIT goes straight to ERROR. Ten proxies are checked. |
| R11 | *Deferred:* the firmware's hardware tests | Out of scope for now (section 10). The port drops today's `hardware_tests` build instead of updating it. For later: `test_odometry` and `test_calibrate_feed_forward` were deleted upstream, `test_controller` and `test_localizer` are new, and `test_imu` spins in `while (true) {}` when the IMU is not initialised. |
| R12 | Scripted starts use the firmware's own paths | A short press gives EXPLORE, a long press SOLVE, an extra-long press CALIBRATE. The link commands are EXPLORE, SOLVE, CALIBRATE, SAVE and RESET. |
| R13 | Documentation rewritten | `CLAUDE.md` ("State of the tuning", "The firmware submodule", the Stopwatch and CSV sections), the README flag table, and the comments of whatever replaces `robot_v2.xml`. |
| R14 | Wall-time cost measured and recorded in the port commit | 8.3 times more ticks per simulated second. The old harness ran at about 12 times real time; the new ratio has to be measured, not assumed. |
| R15 | The seam is hybrid (spec 2 D1, decided; split in spec 2 section 2.2) | Every firmware header is compiled unchanged, including the real `config/targets/v1/target.hpp`, `constants.hpp` and `robot.hpp`. `hal_host/` implements `Gpio`, `Pwm`, `PwmDma`, `AdcDma`, `UartDma`, `Flash`/`FlashWord`, `Mcu`, `Timer` (the simulated clock, wrapping at 32 bits like the cycle counter) and `Crc` (a software CRC driven by `hcrc.Init`). Every proxy built on them runs for real, `Tick` and `Stopwatch` included. Only `Imu` and `RotarySensor` are swapped at the `.cpp` level. `hal_host/` includes nothing from the engine (M4). The shadow proxies, the shadow `config/`, `check_config_drift.py` and `shadow_conformance_test.cpp` are deleted. |
| R16 | Two threads, handed over by the host `Timer` (spec 2 D2, decided; spec 2 section 2.5) | The firmware's own `main` runs unchanged on the firmware thread, compiled with `-Dmain=micras_firmware_main`; a hardware test would run the same way once the tests are in scope. The engine's generic `main` runs them. The host `hal::Timer` is the only place time exists. Every read costs a fixed quantum (for example 1 µs). A read that crosses a simulation step boundary hands over to the engine thread, which runs that step and hands back. The two threads never run at once. At the end of the run, the parked firmware thread is woken with a stop flag and the timer read throws `RunFinished`. `FirmwareThread` is reworked into that handover. `yield_tick()`, `SimulationContext::instance()`, the Stopwatch and Button spin guards, `hardware_test_main.cpp` and the shadow `test_core.hpp` are deleted. Recording becomes per simulation step. A wall-clock watchdog reports a firmware that stops reading the timer. |

---

## 10. Deferred

Out of scope for now, by decision:

| Item | Where it would go |
|---|---|
| CI, in both repositories | Every gate is a `just` recipe, ready to be run by CI |
| The firmware's hardware tests in simulation | The two-thread design runs them unchanged (spec 2 2.5); only the build is missing |
| The pinned build container for cross-machine byte identity | Spec 2 D10, with CI |
| A binary recording format | Spec 2 D11, if CSV size hurts |
| Arenas other than the maze | Spec 2 D9, with the line follower's track |
| Calibration from robot logs | Spec 2 D8; parameters come from datasheets for now |
| SPI chip models behind a host `Spi` | Spec 2 D1; `Imu` and `RotarySensor` stay swapped |
| "New run" from the panel, as a relaunch | Spec 2 D14 |

---

## 11. Order of work and acceptance

Every step ends with `just check` green, and every step before the port ends with
`idle` and `explore_v2` matching `baseline/v2` (P1).

| Step | Content | Accepted when |
|---|---|---|
| 1 | Cleanup (section 2) and wrong statements (section 4), except `robot_v2.xml` | P1 holds, and `just check` no longer mentions the legacy robot or v0/v1 |
| 2 | Bugs (section 3) and dead code (section 5) | P1 holds, and the new bridge tests (G5) pass: a stalled client no longer stalls the run, a taken port fails the recipe |
| 3 | Gates and build (section 8, except G1 and G2) | A clean clone, with `git submodule update --init`, configures and builds with MuJoCo downloaded by CMake, and every gate passes |
| 4 | Layers and folders (section 6) and parameterisation (section 7), except the panel and overlay | P1 holds; M1 is structural |
| 5 | Panel and overlay split (declarative, spec 2 D12), then `check-generic` (M2) turned on | `just check-generic` is green |
| 6 | Port (section 9) | v3 recorded; `hal_host/` exists and satisfies M4 |

## 12. Implementation notes

What the implementation did that the text above does not say, or says differently.

**Versions and tooling.**
- MuJoCo is 3.14.0, fetched by CMake with its SHA-256 (R4). Its release ships its
  own `imgui`/`implot` headers in `include/`, so only a `mujoco/` folder is exposed
  to the build. `mjv_moveCamera` lost its scene argument in 3.14.
- C++23 throughout. clang-format and clang-tidy are pinned to 18: 20 and later
  reflow the firmware's own style.

**The firmware.** Three kinds of change, all made to make sense on the robot too:
- `Micras::get_instance()`, `get_variables()` and `get_state()`: the read-only
  accessor of D5, as three members and a pointer the constructor sets, since the
  firmware's `main` owns its `Micras` and the simulator never sees it.
- `locomotion_config`'s motor `deadzone` 15 -> 0. The controller's feed-forward
  already adds the robot model's static friction voltage, so the dead zone counted
  it twice and made the motors chatter around zero.
- `Micras::measure()` read the chip's X and Y axes as the robot's. The LSM6DSV is
  mounted turned by 90 degrees (robot x is chip Y, robot y is minus chip X);
  fixed there and in `test_controller` and `test_localizer`.
- `robot.hpp`'s physical constants and notes were restated from section 14.

**Where the implementation departs.**
- There is no `baselines/` hash-only store per epoch but one `summary.json` per
  checked scenario (`tools/baseline.py`), carrying the CSV hash, the state
  timeline and the values with their tolerances. Minimum wall clearance is not in
  it yet.
- The checked explore is its first 30 s; the whole scenario takes about three
  minutes of wall time.
- The engine's own name for any two-wheeled robot it generates (`left_wheel`,
  `motor_left`, the `side tracking` camera) is not a robot identifier any more;
  `check-generic` looks for `MicrasFirmware`, `MicrasBoard` and the like instead,
  and the root CMakeLists adds every folder of `targets/` rather than naming one.
- Collisions are debounced: a geom has to be clear for 50 ms before its next
  contact is a new collision.
- `fw_loop_time` is gone from the CSV (R5: constant by construction), and two
  column sources naming the same column is an error.

**What R3 measured.** The timestep is 125 us: 62.5 and 31.25 us changed neither
the contacts nor the trace. What did matter was the contact model. With rolling
friction on the tyres (condim 6), MuJoCo's convex contact lifted a rolling wheel
off the floor on 18 % of steps at 0.4 m/s and 55 % at 1.5 m/s, at every timestep,
bare sphere included; the tyres are condim 4. The same separation in a pivot,
where tyres really scrub, needs a tyre time constant of 5 ms or more to be
absorbed; 8 ms matches the estimated stiffness of the 1 mm silicone band.

**What R14 measured.** A 180 s exploration with the wall sensors' rays runs in
about 181 s of wall time on the development machine (WSL2): roughly real time,
against the old harness's twelve times. Where the time goes has not been
profiled.

**Calibration.** Each wall sensor has a gain in `robot.toml`, set by
`tools/wall_calibration` so the simulated reading at the firmware's calibration
pose equals `target.hpp`'s `reference_readings`.

