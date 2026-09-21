/**
 * @file
 */

#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <mujoco/mujoco.h>

#include "constants.hpp"
#include "micras/sim/app/application.hpp"
#include "micras/sim/app/crash_reporter.hpp"
#include "micras/sim/recording/run_metadata.hpp"
#include "target.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Load the model and configure the clock the run needs.
 *
 * @param options Parsed command line.
 * @param context Context to set up.
 * @return The context, set up.
 */
SimulationContext& prepare(const CliOptions& options, SimulationContext& context) {
    context.world.load(options.model);
    context.clock = Clock::from_model(context.world.timestep(), loop_time_us);
    context.world.reset();
    return context;
}

/**
 * @brief Get the press durations the board classifies against.
 *
 * @return The delays from the Button config.
 */
ButtonDelays board_delays() {
    return {
        .long_press = button_config.long_press_delay,
        .extra_long_press = button_config.extra_long_press_delay,
    };
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
 * @brief Open the bridge to micras-monitor, reporting why if it cannot listen.
 *
 * @note A port that is already taken is a warning, not a failure: the run is
 *       worth having without a monitor watching it.
 *
 * @param options Parsed command line.
 * @param serial Bus the monitor is bridged onto.
 * @return The bridge, or nullptr when none was asked for or it could not listen.
 */
std::unique_ptr<MonitorBridge> make_monitor(const CliOptions& options, SerialBus& serial) {
    if (not options.monitor_enabled) {
        return nullptr;
    }

    std::string error;
    auto        bridge = std::make_unique<MonitorBridge>(serial, options.monitor_port, error);

    if (not bridge->is_open()) {
        std::cerr << "monitor disabled: " << error << '\n';
        return nullptr;
    }

    std::cout << "monitor bridge listening on ws://localhost:" << options.monitor_port << '\n';
    return bridge;
}

/**
 * @brief Open the live window, reporting why if it cannot open.
 *
 * @param options Parsed command line.
 * @param context Context whose world is drawn.
 * @return The viewer, or nullptr when no window was asked for or possible.
 */
std::unique_ptr<MujocoViewer>
    make_viewer(const CliOptions& options, SimulationContext& context, const Telemetry& telemetry) {
    if (not options.viewer_enabled) {
        return nullptr;
    }

    ViewerConfig config = options.viewer;
    config.ticks_per_frame = ticks_per_frame(options.viewer_fps, context.clock);
    config.us_per_tick = context.clock.us_per_tick();

    std::string error;
    auto        viewer = MujocoViewer::create(context.world, context.proxy_state, telemetry, config, error);

    if (viewer == nullptr) {
        std::cerr << "viewer disabled: " << error << '\n';
        return nullptr;
    }

    std::cout << "window open at " << config.width << "x" << config.height << ", camera '" << config.camera
              << "', one frame every " << config.ticks_per_frame << " ticks\n";

    return viewer;
}

/**
 * @brief Bring up the offscreen recorder, reporting why if it cannot run.
 *
 * @note The offscreen framebuffer is sized from the model, which carries no
 *       <visual><global offwidth .../>, so it is widened before the render
 *       context is built. Nothing in mjModel::vis affects physics.
 *
 * @param options Parsed command line.
 * @param context Context whose model is widened to fit the requested frame size.
 * @return The recorder, or nullptr when video is off or unavailable.
 */
std::unique_ptr<VideoRecorder> make_video_recorder(const CliOptions& options, SimulationContext& context) {
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
}  // namespace

Application::Application(const CliOptions& options, FirmwareThread::Program program, PoolTelemetry pool) :
    options{options},
    context{prepare(options, SimulationContext::instance())},
    firmware{std::move(program)},
    pool{pool},
    telemetry{this->context.serial},
    scenario{options.scenario, this->context.proxy_state, this->telemetry, this->context.clock, board_delays()},
    recorder{
        this->context.world, this->context.proxy_state, pool == PoolTelemetry::SERVED ? &this->telemetry : nullptr,
        options.out / "data.csv"
    },
    video{make_video_recorder(options, this->context)},
    viewer{make_viewer(options, this->context, this->telemetry)},
    monitor{make_monitor(options, this->context.serial)},
    simulation{this->context, this->firmware} {
    this->context.firmware = &this->firmware;

    if (this->pool == PoolTelemetry::NOT_SERVED and options.scenario.command != Command::NONE) {
        throw std::runtime_error("this program does not serve the variable pool, so --command cannot be delivered");
    }

    this->simulation.add_listener(CrashReporter::instance());
    this->simulation.add_listener(this->scenario);

    if (this->monitor != nullptr) {
        this->simulation.add_listener(*this->monitor);
    }

    if (this->pool == PoolTelemetry::SERVED) {
        this->telemetry.request_variable_map();
        this->simulation.add_listener(this->telemetry);
    }

    this->simulation.add_listener(this->recorder);

    if (this->video != nullptr) {
        this->video->attach(
            this->context.world, this->telemetry, ticks_per_frame(options.video.fps, this->context.clock)
        );
        this->simulation.add_listener(*this->video);
    }

    if (this->viewer != nullptr) {
        this->simulation.add_listener(*this->viewer);
    }
}

Application::~Application() {
    this->context.firmware = nullptr;
}

void Application::run(std::span<char*> arguments) {
    const auto wall_start = std::chrono::steady_clock::now();

    CrashReporter::install();
    this->simulation.run(this->options.ticks);

    const auto    wall_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    const int64_t warnings = GroundTruth::warnings_total(this->context.world);

    const RunMetadata metadata{
        .firmware_sha = MICRAS_FIRMWARE_SHA,
        .model_path = this->options.model.string(),
        .model_sha256 = RunMetadata::sha256_of(this->options.model),
        .maze_path = this->options.maze,
        .mujoco_version = mj_versionString(),
        .compiler = MICRAS_COMPILER_ID " " MICRAS_COMPILER_VERSION,
        .build_type = MICRAS_BUILD_TYPE,
        .args = RunMetadata::join_args(arguments),
        .loop_time_us = loop_time_us,
        .timestep = this->context.world.timestep(),
        .steps_per_tick = this->context.clock.steps_per_tick(),
        .requested_ticks = this->options.ticks,
        .ticks = this->simulation.completed_ticks(),
        .sim_time = this->context.world.time(),
        .final_z = this->recorder.last_z(),
        .pool_columns = this->recorder.pool_column_count(),
        .telemetry_resyncs = this->telemetry.resync_count(),
        .warnings_total = warnings,
        .interactive = this->viewer != nullptr and this->viewer->was_interactive(),
    };
    metadata.write(this->options.out / "meta.json");

    std::cout << "ticks=" << this->simulation.completed_ticks() << "/" << this->options.ticks
              << " sim_time=" << this->context.world.time() << "s wall=" << wall_seconds
              << "s pool_columns=" << this->recorder.pool_column_count() << " warnings=" << warnings;

    if (this->video != nullptr) {
        std::cout << " video_frames=" << this->video->frame_count();
    }

    std::cout << '\n';
}

int Application::main(std::span<char*> arguments, const FirmwareThread::Program& program, PoolTelemetry pool) {
    try {
        std::cout << "MuJoCo " << mj_versionString() << '\n';

        if (mj_version() != mjVERSION_HEADER) {
            std::cerr << "MuJoCo library/header version mismatch: " << mj_version() << " != " << mjVERSION_HEADER
                      << '\n';
            return 1;
        }

        const CliOptions options = Cli::parse(arguments, loop_time_us);
        std::filesystem::create_directories(options.out);

        Application application(options, program, pool);
        application.run(arguments);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n' << Cli::usage();
        return 1;
    }
}
}  // namespace micras::sim
