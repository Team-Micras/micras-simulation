/**
 * @file
 *
 * @brief What a robot target adds to a run: its options, its wiring, and what the world was built from.
 */

#ifndef MICRAS_SIM_APP_WIRING_HPP
#define MICRAS_SIM_APP_WIRING_HPP

#include <functional>
#include <string>
#include <vector>

#include "micras/sim/core/variable_source.hpp"
#include "micras/sim/recording/column_source.hpp"
#include "micras/sim/robot/robot_description.hpp"
#include "micras/sim/scenario/scenario.hpp"
#include "micras/sim/view/panel_spec.hpp"

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
 * @note Listener order is part of what a run records, so the application fixes
 *       it: the crash reporter, the scenario, the monitor bridge, the event log,
 *       the recorder, the video, the window. The robot's devices run between
 *       the listeners, as the run loop says.
 */
struct Wiring {
    /**
     * @brief Column blocks written after the ground truth, in this order.
     */
    std::vector<ColumnSource*> columns;

    /**
     * @brief Firmware variables the panel plots and the overlay prints, or null.
     */
    const VariableSource* variables{nullptr};

    /**
     * @brief What the panel shows, which the application completes with the handover.
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
}  // namespace micras::sim

#endif  // MICRAS_SIM_APP_WIRING_HPP
