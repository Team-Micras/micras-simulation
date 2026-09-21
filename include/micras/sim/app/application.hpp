/**
 * @file
 *
 * @brief Wires a simulation together and runs one program in it.
 */

#ifndef MICRAS_SIM_APP_APPLICATION_HPP
#define MICRAS_SIM_APP_APPLICATION_HPP

#include <memory>
#include <span>

#include "micras/sim/app/cli.hpp"
#include "micras/sim/bridge/monitor_bridge.hpp"
#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/scenario.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/core/simulation_context.hpp"
#include "micras/sim/recording/csv_recorder.hpp"
#include "micras/sim/telemetry/telemetry.hpp"
#include "micras/sim/view/mujoco_viewer.hpp"
#include "micras/sim/view/video_recorder.hpp"

namespace micras::sim {
/**
 * @brief Whether the program being run serves the firmware variable pool.
 */
enum class PoolTelemetry : uint8_t {
    SERVED,
    NOT_SERVED,
};

/**
 * @brief Builds the context, the listeners and the program, then runs them.
 *
 * @note This is the only place besides config/target.hpp that names the
 *       process-wide SimulationContext, and the only place that decides what a
 *       run is made of. Everything it builds is a listener of one Simulation.
 */
class Application {
public:
    /**
     * @brief Set a simulation up for a parsed command line.
     *
     * @note The variable map is requested here rather than during the run,
     *       because the firmware answers it on the very first tick and the CSV
     *       header has to know every pool column before the first row.
     *
     * @param options Parsed command line.
     * @param program Program to run in lockstep, typically the firmware loop.
     * @param pool Whether the program answers variable pool requests.
     */
    Application(const CliOptions& options, FirmwareThread::Program program, PoolTelemetry pool);

    Application(const Application&) = delete;
    Application(Application&&) = delete;
    Application& operator=(const Application&) = delete;
    Application& operator=(Application&&) = delete;

    /**
     * @brief Detach the program from the process-wide context.
     */
    ~Application();

    /**
     * @brief Run the whole program and write the run directory.
     *
     * @param arguments Whole command line, recorded in the metadata.
     */
    void run(std::span<char*> arguments);

    /**
     * @brief Parse, build and run, reporting any failure on stderr.
     *
     * @param arguments Whole command line, program name included.
     * @param program Program to run in lockstep.
     * @param pool Whether the program answers variable pool requests.
     * @return Process exit code.
     */
    static int main(
        std::span<char*> arguments, const FirmwareThread::Program& program, PoolTelemetry pool = PoolTelemetry::SERVED
    );

private:
    /**
     * @brief Parsed command line.
     */
    CliOptions options;

    /**
     * @brief Simulation everything is wired into.
     *
     * @note Bound for the life of the application; the context outlives main.
     */
    SimulationContext& context;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Program running in lockstep with the physics.
     */
    FirmwareThread firmware;

    /**
     * @brief Whether the program answers variable pool requests.
     */
    PoolTelemetry pool;

    /**
     * @brief Decoder of everything the firmware writes.
     */
    Telemetry telemetry;

    /**
     * @brief Scripted input for the run.
     */
    Scenario scenario;

    /**
     * @brief Writer of data.csv.
     */
    CsvRecorder recorder;

    /**
     * @brief Offscreen recorder, null unless --video asked for one and it came up.
     */
    std::unique_ptr<VideoRecorder> video;

    /**
     * @brief Live window, null unless --viewer asked for one and it came up.
     */
    std::unique_ptr<MujocoViewer> viewer;

    /**
     * @brief Bridge to micras-monitor, null unless --monitor asked for one.
     */
    std::unique_ptr<MonitorBridge> monitor;

    /**
     * @brief The run itself.
     */
    Simulation simulation;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_APP_APPLICATION_HPP
