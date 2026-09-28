/**
 * @file
 *
 * @brief What a robot target gives the application, and what it gets back.
 */

#ifndef MICRAS_SIM_APP_TARGET_HPP
#define MICRAS_SIM_APP_TARGET_HPP

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "micras/sim/app/wiring.hpp"
#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/recording/ground_truth.hpp"
#include "micras/sim/recording/run_metadata.hpp"

namespace micras::sim {
/**
 * @brief One robot, as the application sees it.
 *
 * @note Everything the engine must not know lives behind this interface: the
 *       firmware, its loop period, the model's names, the robot's own options,
 *       columns and panel. The application calls it in this order: options(),
 *       context(), program(), wire(), and after the run metadata() and
 *       unwire().
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
     * @brief Get the context the run advances.
     *
     * @note The application builds the world in it and configures its clock.
     *
     * @return The context, which must outlive the run.
     */
    virtual RunContext& context() = 0;

    /**
     * @brief Get the folder of the target, whose tools/analysis.py is its analysis plugin.
     *
     * @return The folder, written to meta.json as target_dir.
     */
    virtual std::filesystem::path directory() const = 0;

    /**
     * @brief Get the robot's physical description.
     *
     * @return Path of its robot.toml, which the world is built from.
     */
    virtual std::filesystem::path robot_file() const = 0;

    /**
     * @brief Get the body and robot columns of the ground truth.
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
     * @brief Build what the robot adds to the run: its devices, columns and panel.
     *
     * @note Called once the world is built and the clock configured; the devices
     *       it adds to the context's run in the order added.
     *
     * @param firmware Thread that will run the program.
     * @param world The robot the world was built from, and the arena's surfaces.
     * @return Columns, variables, panel and scenario hooks of the robot.
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
