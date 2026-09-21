/**
 * @file
 */

#ifndef MICRAS_PROXY_FAN_HPP
#define MICRAS_PROXY_FAN_HPP

#include <string>

#include "micras/proxy/stopwatch.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/proxy_state.hpp"

namespace micras::sim {
struct SimulationContext;
}  // namespace micras::sim

namespace micras::proxy {
/**
 * @brief Fan driver writing directly to a MuJoCo actuator control input.
 *
 * @note The speed ramp mirrors the firmware Fan: set_speed only moves the
 *       target and update() slews the current speed towards it at
 *       max_acceleration percent per millisecond of simulated time.
 */
class Fan {
public:
    /**
     * @brief Configuration struct for the fan.
     */
    struct Config {
        std::string actuator;
        float       max_acceleration;
        /**
         * @brief Simulation holding the actuator this fan drives.
         */
        sim::SimulationContext* context{nullptr};
    };

    /**
     * @brief Construct a new fan object.
     *
     * @param config Configuration for the fan driver.
     */
    explicit Fan(const Config& config);

    /**
     * @brief Enable the fan.
     */
    void enable();

    /**
     * @brief Disable the fan.
     */
    void disable();

    /**
     * @brief Set the target speed of the fans.
     *
     * @param speed Speed percentage of the fan.
     */
    void set_speed(float speed);

    /**
     * @brief Slew the current speed towards the target and drive the actuator.
     *
     * @note A zero speed parks the actuator instead of writing a zero duty
     *       cycle through the direction logic, matching the firmware.
     *
     * @return Current speed value.
     */
    float update();

    /**
     * @brief Stop the fan, zeroing the actuator without clearing the target.
     */
    void stop();

private:
    /**
     * @brief Simulation the actuator lives in.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    sim::MujocoWorld& world;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Boundary record: the switch that silences the fan comes in here,
     *        the speed the fan settled on goes out.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    sim::ProxyState& state;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Write the current speed to the actuator.
     */
    void apply();

    /**
     * @brief Id of the MuJoCo actuator driven by this fan.
     */
    int actuator_id;

    /**
     * @brief Maximum acceleration of the fan in percentage per millisecond.
     */
    float max_acceleration;

    /**
     * @brief Current speed percentage written to the actuator.
     */
    float current_speed{0.0F};

    /**
     * @brief Last requested speed percentage.
     */
    float target_speed{0.0F};

    /**
     * @brief Flag to check if the fan is enabled.
     */
    bool enabled{false};

    /**
     * @brief Stopwatch for limiting the acceleration of the fan.
     */
    proxy::Stopwatch acceleration_stopwatch;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_FAN_HPP
