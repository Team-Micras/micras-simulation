/**
 * @file
 *
 * @brief What a robot target gives the application, and what it gets back.
 */

#ifndef MICRAS_SIM_APP_TARGET_HPP
#define MICRAS_SIM_APP_TARGET_HPP

#include <cstdint>
#include <filesystem>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/panel_spec.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/core/variable_source.hpp"
#include "micras/sim/recording/column_source.hpp"
#include "micras/sim/recording/ground_truth.hpp"
#include "micras/sim/recording/run_metadata.hpp"
#include "micras/sim/robot/robot_description.hpp"
#include "micras/sim/scenario/scenario.hpp"

namespace micras::sim {
/**
 * @brief A command line option a robot target adds.
 */
struct CliOption {
    /**
     * @brief Option as typed, for example "--button".
     */
    std::string name;

    /**
     * @brief What the value looks like in the usage text, empty for a flag.
     */
    std::string argument;

    /**
     * @brief Called with the value, or with an empty string for a flag.
     *
     * @note Throws std::runtime_error on a value it cannot accept.
     */
    std::function<void(const std::string& value)> apply;
};

/**
 * @brief How a robot target plugs into one run.
 *
 * @note Listener order is part of what a run records, so it is fixed here: the
 *       crash reporter, the inputs, the monitor bridge, the observers, the
 *       recorder, the video, the window.
 */
struct Wiring {
    /**
     * @brief Listeners that feed the firmware, run before the monitor bridge.
     */
    std::vector<IRunListener*> inputs;

    /**
     * @brief Listeners that decode the firmware, run after the bridge and before the recorder.
     */
    std::vector<IRunListener*> observers;

    /**
     * @brief Column blocks written after the ground truth, in this order.
     */
    std::vector<ColumnSource*> columns;

    /**
     * @brief Firmware variables the panel plots and the overlay prints, or null.
     */
    const VariableSource* variables{nullptr};

    /**
     * @brief Hands bytes from a monitor to the firmware; null queues them as they came.
     */
    std::function<void(std::span<const uint8_t>)> monitor_inbound;

    /**
     * @brief What the panel shows.
     */
    PanelSpec panel;

    /**
     * @brief What the video overlay prints.
     */
    OverlaySpec overlay;

    /**
     * @brief Inputs, link messages and state names scenarios act on.
     */
    ScenarioHooks hooks;
};

/**
 * @brief What the world was built from, for the devices a target builds.
 */
struct WorldInfo {
    /**
     * @brief The robot's physical description, which outlives the wiring.
     */
    const RobotDescription* robot;

    /**
     * @brief Infrared reflectance of an arena geom, by id.
     */
    std::function<double(int)> reflectance;
};

/**
 * @brief One robot, as the application sees it.
 *
 * @note Everything the engine must not know lives behind this interface: the
 *       firmware, its loop period, the model's names, the robot's own options,
 *       columns, panel and listeners. The application calls it in this order:
 *       options(), check_options(), context(), program(), wire(), and after
 *       the run metadata() and unwire().
 */
class Target {
public:
    Target() = default;

    Target(const Target&) = delete;
    Target(Target&&) = delete;
    Target& operator=(const Target&) = delete;
    Target& operator=(Target&&) = delete;

    virtual ~Target() = default;

    /**
     * @brief Get the name of the robot, used for the window and meta.json.
     *
     * @return The name.
     */
    virtual std::string name() const = 0;

    /**
     * @brief Get the commit of the firmware this binary was built from.
     *
     * @return The commit, or "unknown".
     */
    virtual std::string firmware_sha() const = 0;

    /**
     * @brief Get the firmware loop period, which the physics timestep must divide.
     *
     * @return Period in microseconds.
     */
    virtual uint32_t loop_time_us() const = 0;

    /**
     * @brief Get the options this robot adds to the command line.
     *
     * @return The options; their handlers may capture the target.
     */
    virtual std::vector<CliOption> options() { return {}; }

    /**
     * @brief Reject combinations of options the robot cannot honour.
     *
     * @note Called once the whole command line was read.
     */
    virtual void check_options() const { }

    /**
     * @brief Get the context the run advances.
     *
     * @note The application builds the world in it and configures its clock.
     *
     * @return The context, which must outlive the run.
     */
    virtual RunContext& context() = 0;

    /**
     * @brief Get the robot's physical description.
     *
     * @return Path of its robot.toml, which the world is built from.
     */
    virtual std::filesystem::path robot_file() const = 0;

    /**
     * @brief Get the scenario a run plays when the command line names none.
     *
     * @return Path of a scenario file, or empty for none.
     */
    virtual std::string default_scenario() const { return {}; }

    /**
     * @brief Get the body, forward axis and robot columns of the ground truth.
     *
     * @return The recorder configuration.
     */
    virtual GroundTruthConfig ground_truth() const = 0;

    /**
     * @brief Get the camera a video uses when the command line names none.
     *
     * @return A model camera name, or "free".
     */
    virtual std::string video_camera() const { return "free"; }

    /**
     * @brief Get the firmware program, run on the firmware thread.
     *
     * @return The program.
     */
    virtual FirmwareThread::Program program() = 0;

    /**
     * @brief Build what the robot adds to the run: its devices, listeners, columns and panel.
     *
     * @note Called once the world is built and the clock configured; the devices
     *       it adds to the context's run in the order added.
     *
     * @param firmware Thread that will run the program.
     * @param world The robot the world was built from, and the arena's surfaces.
     * @return Listeners, columns, variables and panel of the robot.
     */
    virtual Wiring wire(FirmwareThread& firmware, const WorldInfo& world) = 0;

    /**
     * @brief Undo whatever wire() pointed at the firmware thread.
     */
    virtual void unwire() { }

    /**
     * @brief Get the robot's own counters for meta.json.
     *
     * @return The counters, in the order they are written.
     */
    virtual std::vector<MetadataField> metadata() const { return {}; }
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_APP_TARGET_HPP
