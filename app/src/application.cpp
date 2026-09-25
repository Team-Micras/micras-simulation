/**
 * @file
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include <mujoco/mujoco.h>

#include "micras/sim/app/application.hpp"
#include "micras/sim/app/crash_reporter.hpp"
#include "micras/sim/recording/run_metadata.hpp"
#include "micras/sim/robot/robot_model.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Duration of a run when neither the command line nor the scenario gives one.
 */
constexpr double default_seconds{10.0};

/**
 * @brief Maze a run uses when neither the command line nor the scenario names one.
 */
constexpr std::string_view default_maze{"maze1"};

/**
 * @brief Gap left between the robot's back and the wall behind it at the start, in metres.
 */
constexpr double start_gap{0.001};

/**
 * @brief Find a maze drawing: a path, or a name from the arena's collection.
 *
 * @param name The path or name.
 * @return The drawing's path.
 */
std::filesystem::path resolve_maze(const std::string& name) {
    std::filesystem::path given{name};

    if (std::filesystem::exists(given)) {
        return given;
    }

    const std::filesystem::path collected = std::filesystem::path{MICRAS_SIM_MAZES_DIR} / (name + ".txt");

    if (std::filesystem::exists(collected)) {
        return collected;
    }

    throw std::runtime_error("no maze named " + name + ": neither a file nor one of " + MICRAS_SIM_MAZES_DIR);
}

/**
 * @brief Choose the maze a run uses: the command line's, else the scenario's, else the default.
 *
 * @param options Parsed command line.
 * @param scenario The scenario the run plays.
 * @return The maze's path or name.
 */
std::string maze_name(const CliOptions& options, const Scenario& scenario) {
    if (not options.maze.empty()) {
        return options.maze;
    }

    if (not scenario.arena.empty()) {
        return scenario.arena;
    }

    return std::string{default_maze};
}

/**
 * @brief Place a robot in the maze's start cell, its back against the wall behind it, facing up.
 *
 * @param maze The maze.
 * @param config Its dimensions.
 * @param robot The robot.
 * @return The placement.
 */
MujocoWorld::Placement start_of(const Maze& maze, const MazeConfig& config, const RobotDescription& robot) {
    double rear = 0.0;

    for (const auto& point : robot.chassis.outline) {
        rear = std::max(rear, -point[0]);
    }

    const auto [column, row] = maze.start();

    return {
        .x = (static_cast<double>(column) + 0.5) * config.cell_size,
        .y = static_cast<double>(row) * config.cell_size + config.wall_thickness / 2 + rear + start_gap,
        .yaw = std::numbers::pi / 2,
    };
}

/**
 * @brief Read the scenario the run plays.
 *
 * @param options Parsed command line.
 * @param target Robot to run.
 * @param path Filled with the scenario's path, empty for none.
 * @return The scenario, empty when there is none.
 */
Scenario read_scenario(const CliOptions& options, const Target& target, std::filesystem::path& path) {
    path = options.scenario.empty() ? std::filesystem::path{target.default_scenario()} : options.scenario;

    if (path.empty()) {
        return {};
    }

    Scenario scenario = Scenario::load(path);

    if (not scenario.robot.empty() and scenario.robot != target.name()) {
        throw std::runtime_error(path.string() + " is a scenario for " + scenario.robot + ", not " + target.name());
    }

    return scenario;
}

/**
 * @brief Build the world, configure the clock and the noise.
 *
 * @param robot The robot's description.
 * @param maze_path Path of the maze drawing.
 * @param config The maze's dimensions.
 * @param options Parsed command line.
 * @param scenario The scenario, which may place the robot.
 * @param target Robot to run.
 * @return The target's context.
 */
RunContext& prepare(
    const RobotDescription& robot, const std::filesystem::path& maze_path, const MazeConfig& config,
    const CliOptions& options, const Scenario& scenario, Target& target
) {
    RunContext& context = target.context();
    const Maze  maze = Maze::load(maze_path);

    context.world.build(
        robot_mjcf(robot), RobotModelNames::of(robot).body, maze.mjcf(config), Maze::body_name,
        scenario.start.value_or(start_of(maze, config, robot))
    );
    context.clock = Clock::from_model(context.world.timestep(), target.loop_time_us());
    context.world.reset();
    context.noise = {.seed = options.seed.value_or(scenario.seed.value_or(1)), .ideal = options.ideal};
    return context;
}

/**
 * @brief Get how many ticks the run lasts.
 *
 * @param options Parsed command line.
 * @param scenario The scenario.
 * @param target Robot to run.
 * @return Number of ticks.
 */
uint64_t ticks_of(const CliOptions& options, const Scenario& scenario, const Target& target) {
    if (options.ticks.has_value()) {
        return *options.ticks;
    }

    const double   seconds = options.seconds.value_or(scenario.seconds > 0.0 ? scenario.seconds : default_seconds);
    const uint64_t ticks = Clock::total_ticks(seconds, target.loop_time_us());

    if (ticks == 0) {
        throw std::runtime_error(std::to_string(seconds) + " s is shorter than one firmware loop period");
    }

    return ticks;
}

/**
 * @brief Get how many ticks pass between two frames at a given rate.
 *
 * @param fps Frames per second of simulated time.
 * @param clock Clock the frame period is derived from.
 * @return Number of ticks per frame, at least one.
 */
uint64_t ticks_per_frame(int fps, const Clock& clock) {
    const double loop_period = clock.us_per_tick() * 1e-6;
    return std::max<uint64_t>(1, static_cast<uint64_t>(std::llround(1.0 / (fps * loop_period))));
}

/**
 * @brief Open the monitor bridge if the command line asked for it.
 *
 * @note A port that is already taken is reported and the run carries on
 *       without the bridge, rather than failing a run that did not need it.
 *
 * @param options Parsed command line.
 * @param serial Bus the bridge attaches to.
 * @param inbound Hands the monitors' bytes to the firmware, or null to queue them as they came.
 * @return The bridge, or null when disabled or when the port was taken.
 */
std::unique_ptr<MonitorBridge>
    make_monitor(const CliOptions& options, SerialBus& serial, MonitorBridge::Inbound inbound) {
    if (not options.monitor_enabled) {
        return nullptr;
    }

    if (not inbound) {
        inbound = [&serial](std::span<const uint8_t> bytes) { serial.queue_for_firmware(bytes); };
    }

    std::string error;
    auto        bridge = std::make_unique<MonitorBridge>(serial, std::move(inbound), options.monitor_port, error);

    if (not bridge->is_open()) {
        std::cerr << "monitor disabled: " << error << '\n';
        return nullptr;
    }

    std::cout << "monitor bridge listening on ws://localhost:" << options.monitor_port << '\n';
    return bridge;
}

/**
 * @brief Open the live window if the command line asked for it.
 *
 * @param options Parsed command line.
 * @param context Context the window draws.
 * @param target Robot whose name and body the window uses.
 * @param wiring What the robot put on the panel.
 * @return The viewer, or null when disabled or when no window could open.
 */
std::unique_ptr<MujocoViewer>
    make_viewer(const CliOptions& options, RunContext& context, const Target& target, const Wiring& wiring) {
    if (not options.viewer_enabled) {
        return nullptr;
    }

    ViewerConfig config = options.viewer;
    config.title = target.name();
    config.body = target.ground_truth().body;
    config.ticks_per_frame = ticks_per_frame(options.viewer_fps, context.clock);
    config.us_per_tick = context.clock.us_per_tick();

    std::string error;
    auto        viewer = MujocoViewer::create(context.world, wiring.panel, wiring.variables, config, error);

    if (viewer == nullptr) {
        std::cerr << "viewer disabled: " << error << '\n';
        return nullptr;
    }

    std::cout << "window open at " << config.width << "x" << config.height << ", camera '" << config.camera
              << "', one frame every " << config.ticks_per_frame << " ticks\n";

    return viewer;
}

/**
 * @brief Start the offscreen recorder if the command line asked for it.
 *
 * @param options Parsed command line.
 * @param context Context the recorder draws.
 * @return The recorder, or null when disabled or when EGL is unavailable.
 */
std::unique_ptr<VideoRecorder> make_video_recorder(const CliOptions& options, RunContext& context) {
    if (not options.video_enabled) {
        return nullptr;
    }

    mjModel* model = context.world.model();
    model->vis.global.offwidth = std::max(model->vis.global.offwidth, options.video.width);
    model->vis.global.offheight = std::max(model->vis.global.offheight, options.video.height);

    std::string error;
    auto        recorder = VideoRecorder::create(model, options.video, error);

    if (recorder == nullptr) {
        std::cerr << "video disabled: " << error << '\n';
        return nullptr;
    }

    std::cout << "recording " << options.video.path << " at " << options.video.width << "x" << options.video.height
              << " " << options.video.fps << " fps, camera '" << options.video.camera << "', one frame every "
              << ticks_per_frame(options.video.fps, context.clock) << " ticks\n";

    return recorder;
}

/**
 * @brief Build what the target adds, and the pusher every scenario may use.
 *
 * @param target Robot to run.
 * @param firmware Thread that will run the program.
 * @param robot The robot's description.
 * @param config The maze's surfaces.
 * @param pusher Filled with the pusher, which the context's devices own.
 * @return The target's wiring.
 */
Wiring wire(
    Target& target, FirmwareThread& firmware, const RobotDescription& robot, const MazeConfig& config, Pusher*& pusher
) {
    RunContext&     context = target.context();
    const WorldInfo world{
        .robot = &robot,
        .reflectance =
            [&context, config](int geom) {
                const char* name = mj_id2name(context.world.model(), mjOBJ_GEOM, geom);
                return Maze::reflectance(name == nullptr ? "" : name, config);
            },
    };

    Wiring wiring = target.wire(firmware, world);
    auto   owned = std::make_unique<Pusher>(context.world, target.ground_truth().body);
    pusher = owned.get();
    context.devices.push_back(std::move(owned));
    return wiring;
}

/**
 * @brief Collect the robot's geoms other than its wheels, whose wall contacts are collisions.
 *
 * @param world The world.
 * @param config The robot's ground truth, naming its wheels.
 * @return Geom ids.
 */
std::vector<int> chassis_geoms(const MujocoWorld& world, const GroundTruthConfig& config) {
    const mjModel*             model = world.model();
    const int                  root = world.require_id(mjOBJ_BODY, config.body);
    const std::span<const int> bodies(model->geom_bodyid, static_cast<std::size_t>(model->ngeom));
    std::vector<int>           geoms;

    for (int geom = 0; geom < model->ngeom; geom++) {
        if (bodies[static_cast<std::size_t>(geom)] == root) {
            geoms.push_back(geom);
        }
    }

    return geoms;
}
}  // namespace

Application::Application(const CliOptions& options, Target& target) :
    options{options},
    target{target},
    scenario{read_scenario(options, target, this->scenario_path)},
    robot{RobotDescription::load(target.robot_file())},
    maze_path{resolve_maze(maze_name(options, this->scenario))},
    context{prepare(this->robot, this->maze_path, this->maze_config, options, this->scenario, target)},
    ticks{ticks_of(options, this->scenario, target)},
    firmware{target.program()},
    wiring{wire(target, this->firmware, this->robot, this->maze_config, this->pusher)},
    player{this->scenario, this->wiring.hooks, this->context.serial, *this->pusher, this->wiring.variables},
    events{
        chassis_geoms(this->context.world, target.ground_truth()),
        {this->context.world.require_id(mjOBJ_GEOM, std::string{Maze::body_name} + "_floor")},
        {},
        this->wiring.variables
    },
    device_columns{this->context.devices},
    recorder{this->context.world, target.ground_truth(), options.out / "data.csv", options.record_every},
    video{make_video_recorder(options, this->context)},
    viewer{make_viewer(options, this->context, target, this->wiring)},
    monitor{make_monitor(options, this->context.serial, this->wiring.monitor_inbound)},
    simulation{this->context, this->firmware} {
    for (const auto& [variable, names] : this->wiring.hooks.state_names) {
        this->events.watch({.variable = variable, .names = names});
    }

    for (ColumnSource* source : this->wiring.columns) {
        this->recorder.add_source(*source);
    }

    this->recorder.add_source(this->device_columns);

    this->simulation.add_listener(CrashReporter::instance());
    this->simulation.add_listener(this->player);

    for (IRunListener* listener : this->wiring.inputs) {
        this->simulation.add_listener(*listener);
    }

    if (this->monitor != nullptr) {
        this->simulation.add_listener(*this->monitor);
    }

    for (IRunListener* listener : this->wiring.observers) {
        this->simulation.add_listener(*listener);
    }

    this->simulation.add_listener(this->events);
    this->simulation.add_listener(this->recorder);

    if (this->video != nullptr) {
        this->video->attach(
            this->context.world, this->wiring.overlay, this->wiring.variables,
            ticks_per_frame(options.video.fps, this->context.clock),
            options.video.trail ? target.ground_truth().body : std::string{}
        );
        this->simulation.add_listener(*this->video);
    }

    if (this->viewer != nullptr) {
        this->simulation.add_listener(*this->viewer);
    }

    std::ofstream(options.out / "model.xml") << this->context.world.composed_xml();
}

Application::~Application() {
    this->target.unwire();
}

void Application::run(std::span<char*> arguments) {
    const auto wall_start = std::chrono::steady_clock::now();

    CrashReporter::install();
    this->simulation.run(this->ticks);

    const auto    wall_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    const int64_t warnings = GroundTruth::warnings_total(this->context.world);

    const RunMetadata metadata{
        .target = this->target.name(),
        .firmware_sha = this->target.firmware_sha(),
        .robot_path = this->target.robot_file().string(),
        .robot_sha256 = RunMetadata::sha256_of(this->target.robot_file()),
        .scenario_path = this->scenario_path.string(),
        .maze_path = this->maze_path.string(),
        .model_sha256 = RunMetadata::sha256_of_text(this->context.world.composed_xml()),
        .mujoco_version = mj_versionString(),
        .compiler = MICRAS_COMPILER_ID " " MICRAS_COMPILER_VERSION,
        .build_type = MICRAS_BUILD_TYPE,
        .args = RunMetadata::join_args(arguments),
        .seed = this->context.noise.seed,
        .ideal = this->context.noise.ideal,
        .loop_time_us = this->target.loop_time_us(),
        .timestep = this->context.world.timestep(),
        .steps_per_tick = this->context.clock.steps_per_tick(),
        .record_every = this->options.record_every,
        .requested_ticks = this->ticks,
        .ticks = this->simulation.completed_ticks(),
        .sim_time = this->context.world.time(),
        .stopped_at = this->player.stopped_at().value_or(-1.0),
        .final_z = this->recorder.last_z(),
        .target_fields = this->target.metadata(),
        .warnings_total = warnings,
        .serial_dropped_bytes = this->context.serial.dropped_bytes(),
        .bridge_dropped_frames = this->monitor == nullptr ? 0 : this->monitor->dropped_frames(),
        .interactive = (this->viewer != nullptr and this->viewer->was_interactive()) or
                       (this->monitor != nullptr and this->monitor->was_interactive()),
        .events = this->events.entries(),
    };
    metadata.write(this->options.out / "meta.json");

    std::cout << "ticks=" << this->simulation.completed_ticks() << "/" << this->ticks
              << " sim_time=" << this->context.world.time() << "s wall=" << wall_seconds << "s";

    for (const MetadataField& field : metadata.target_fields) {
        std::cout << " " << field.name << "=" << field.value;
    }

    std::cout << " collisions=" << this->events.collisions() << " warnings=" << warnings;

    if (this->video != nullptr) {
        std::cout << " video_frames=" << this->video->frame_count();
    }

    std::cout << '\n';
}

int Application::main(std::span<char*> arguments, Target& target) {
    const std::vector<CliOption> target_options = target.options();
    const std::string            program =
        arguments.empty() ? target.name() : std::filesystem::path(arguments.front()).filename().string();

    try {
        std::cout << "MuJoCo " << mj_versionString() << '\n';

        if (mj_version() != mjVERSION_HEADER) {
            std::cerr << "MuJoCo library/header version mismatch: " << mj_version() << " != " << mjVERSION_HEADER
                      << '\n';
            return 1;
        }

        CliOptions options = Cli::parse(arguments, target_options);
        target.check_options();

        if (not options.video_camera_given) {
            options.video.camera = target.video_camera();
        }

        std::filesystem::create_directories(options.out);

        Application application(options, target);
        application.run(arguments);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n' << Cli::usage(program, target_options);
        return 1;
    }
}

int run(std::span<char*> arguments, Target& target) {
    return Application::main(arguments, target);
}
}  // namespace micras::sim
