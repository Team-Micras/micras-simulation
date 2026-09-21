/**
 * @file
 */

#ifndef MICRAS_PROXY_ROTARY_SENSOR_HPP
#define MICRAS_PROXY_ROTARY_SENSOR_HPP

#include <string>

#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/proxy_state.hpp"

namespace micras::sim {
struct SimulationContext;
}  // namespace micras::sim

namespace micras::proxy {
/**
 * @brief Rotary sensor reading a MuJoCo jointpos sensor.
 */
class RotarySensor {
public:
    /**
     * @brief Rotary sensor configuration struct.
     */
    struct Config {
        std::string sensor;

        /**
         * @brief Wheel this encoder belongs to, left first.
         */
        std::size_t wheel{0};
        /**
         * @brief Simulation holding the sensor this encoder reads.
         */
        sim::SimulationContext* context{nullptr};
    };

    /**
     * @brief Construct a new RotarySensor object.
     *
     * @param config Configuration for the rotary sensor.
     */
    explicit RotarySensor(const Config& config);

    /**
     * @brief Get the rotary sensor position over an axis.
     *
     * @return Current angular position of the sensor in radians.
     */
    float get_position() const;

private:
    /**
     * @brief Boundary record this proxy writes its reading into.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    sim::ProxyState& state;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Wheel this encoder belongs to.
     */
    std::size_t wheel;

    /**
     * @brief Simulation the readings come from.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    const sim::MujocoWorld& world;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Address of the sensor value inside mjData::sensordata.
     */
    int sensor_address;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_ROTARY_SENSOR_HPP
