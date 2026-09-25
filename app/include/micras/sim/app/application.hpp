/**
 * @file
 *
 * @brief Wires a simulation together and runs one robot target in it.
 */

#ifndef MICRAS_SIM_APP_APPLICATION_HPP
#define MICRAS_SIM_APP_APPLICATION_HPP

#include <memory>
#include <span>

#include "micras/sim/app/cli.hpp"
#include "micras/sim/app/target.hpp"
#include "micras/sim/arenas/maze.hpp"
#include "micras/sim/bridge/monitor_bridge.hpp"
#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/devices/device_columns.hpp"
#include "micras/sim/recording/csv_recorder.hpp"
#include "micras/sim/recording/event_log.hpp"
#include "micras/sim/robot/robot_description.hpp"
#include "micras/sim/scenario/scenario.hpp"
#include "micras/sim/view/mujoco_viewer.hpp"
#include "micras/sim/view/video_recorder.hpp"

namespace micras::sim {
/**
 * @brief Builds the run around a robot target, then runs it.
 *
 * @note The only place that decides what a run is made of: it reads the
 *       scenario, builds the world from the robot's description and the arena,
 *       and registers every listener of one Simulation in the order Wiring
 *       documents, the scenario first. It is also the only layer that knows a
 *       run can have a window.
 */
class Application {
public:
    /**
     * @brief Set a simulation up for a parsed command line.
     *
     * @param options Parsed command line.
     * @param target Robot to run.
     */
    Application(const CliOptions& options, Target& target);

    Application(const Application&) = delete;
    Application(Application&&) = delete;
    Application& operator=(const Application&) = delete;
    Application& operator=(Application&&) = delete;

    /**
     * @brief Let the target undo what it pointed at the firmware thread.
     */
    ~Application();

    /**
     * @brief Run the simulation and write the metadata.
     *
     * @param arguments Whole command line, recorded in the metadata.
     */
    void run(std::span<char*> arguments);

    /**
     * @brief Entry point shared by every simulation executable.
     *
     * @note Catches everything, prints it with the usage text and returns
     *       non-zero, so a failed run is loud and never looks successful.
     *
     * @param arguments Whole command line.
     * @param target Robot to run.
     * @return Process exit code.
     */
    static int main(std::span<char*> arguments, Target& target);

private:
    /**
     * @brief Parsed command line.
     */
    CliOptions options;

    /**
     * @brief Robot being run.
     *
     * @note Owned by the executable's main, which outlives the application.
     */
    Target& target;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Path of the scenario file, empty for none.
     */
    std::filesystem::path scenario_path;

    /**
     * @brief Scenario being played.
     */
    Scenario scenario;

    /**
     * @brief The robot's physical description.
     */
    RobotDescription robot;

    /**
     * @brief Path of the maze drawing.
     */
    std::filesystem::path maze_path;

    /**
     * @brief Dimensions and surfaces of the maze.
     */
    MazeConfig maze_config;

    /**
     * @brief The target's context, with the world built and the clock configured.
     *
     * @note Bound for the life of the run; the target owns it.
     */
    RunContext& context;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Number of ticks to run.
     */
    uint64_t ticks;

    /**
     * @brief The target's program, in its own thread.
     */
    FirmwareThread firmware;

    /**
     * @brief Pushes the robot when the scenario says so; owned by the context's devices.
     */
    Pusher* pusher{nullptr};

    /**
     * @brief What the target adds to the run.
     */
    Wiring wiring;

    /**
     * @brief Plays the scenario.
     */
    ScenarioPlayer player;

    /**
     * @brief Notices collisions and state changes at full rate.
     */
    EventLog events;

    /**
     * @brief What the devices record, after the target's own columns.
     */
    DeviceColumns device_columns;

    /**
     * @brief CSV output.
     */
    CsvRecorder recorder;

    /**
     * @brief Offscreen video, or null.
     */
    std::unique_ptr<VideoRecorder> video;

    /**
     * @brief Live window, or null.
     */
    std::unique_ptr<MujocoViewer> viewer;

    /**
     * @brief Bridge to micras-monitor, or null.
     */
    std::unique_ptr<MonitorBridge> monitor;

    /**
     * @brief The run itself.
     */
    Simulation simulation;
};

/**
 * @brief Run a robot target from its executable's main.
 *
 * @param arguments Whole command line.
 * @param target Robot to run.
 * @return Process exit code.
 */
int run(std::span<char*> arguments, Target& target);
}  // namespace micras::sim

#endif  // MICRAS_SIM_APP_APPLICATION_HPP
