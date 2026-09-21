/**
 * @file
 */

#ifndef MICRAS_PROXY_MOTOR_HPP
#define MICRAS_PROXY_MOTOR_HPP

#include <string>

#include "micras/sim/core/mujoco_world.hpp"

namespace micras::sim {
struct SimulationContext;
}  // namespace micras::sim

namespace micras::proxy {
/**
 * @brief Motor driver writing directly to a MuJoCo actuator control input.
 */
class Motor {
public:
    /**
     * @brief Configuration struct for the motor.
     */
    struct Config {
        std::string actuator;
        /**
         * @brief Simulation holding the actuator this motor drives.
         */
        sim::SimulationContext* context{nullptr};
    };

    /**
     * @brief Construct a new motor object.
     *
     * @param config Configuration for the motor driver.
     */
    explicit Motor(const Config& config);

    /**
     * @brief Set the command for the motor.
     *
     * @note The firmware deadzone and max_stopped_command remap is deliberately
     *       not mirrored here: it compensates for real H-bridge and gearbox
     *       stiction, below which the physical motor does not turn at all. The
     *       MuJoCo actuator is an ideal torque source with no deadzone, so the
     *       remap would inject an offset the simulated plant never needed.
     *
     * @param command Command for the motor in percentage.
     */
    void set_command(float command);

private:
    /**
     * @brief Simulation the actuator lives in.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    sim::MujocoWorld& world;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Id of the MuJoCo actuator driven by this motor.
     */
    int actuator_id;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_MOTOR_HPP
