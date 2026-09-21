/**
 * @file
 */

#ifndef MICRAS_PROXY_IMU_HPP
#define MICRAS_PROXY_IMU_HPP

#include <array>
#include <cstdint>
#include <string>

#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/proxy_state.hpp"

namespace micras::sim {
struct SimulationContext;
}  // namespace micras::sim

namespace micras::proxy {
/**
 * @brief IMU reading the MuJoCo gyro and accelerometer sensors.
 *
 * @note The imu site frame matches the robot body frame: +y forward, +z up,
 *       +x right, so the yaw rate is the gyro z component and the
 *       accelerometer reports +g on z while at rest.
 */
class Imu {
public:
    /**
     * @brief IMU configuration struct.
     */
    struct Config {
        std::string gyro_sensor;
        std::string accelerometer_sensor;
        /**
         * @brief Simulation holding the sensors this IMU reads.
         */
        sim::SimulationContext* context{nullptr};
    };

    /**
     * @brief Enum to select the axis of the IMU.
     */
    enum class Axis : uint8_t {
        X = 0,
        Y = 1,
        Z = 2
    };

    /**
     * @brief Construct a new Imu object.
     *
     * @param config Configuration for the IMU.
     */
    explicit Imu(const Config& config);

    /**
     * @brief Update the IMU data.
     */
    void update();

    /**
     * @brief Get the IMU angular velocity over an axis.
     *
     * @param axis Axis to get the angular velocity from.
     * @return Angular velocity over the desired axis in rad/s.
     */
    float get_angular_velocity(Axis axis) const;

    /**
     * @brief Get the IMU linear acceleration over an axis.
     *
     * @param axis Axis to get the linear acceleration from.
     * @return Linear acceleration over the desired axis in m/s².
     */
    float get_linear_acceleration(Axis axis) const;

    /**
     * @brief Define the base reading to be removed from the IMU value.
     */
    void calibrate();

    /**
     * @brief Check if IMU was initialized.
     *
     * @return Always true in the simulation.
     */
    bool was_initialized() const;

private:
    /**
     * @brief Boundary record this proxy writes its readings into.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    sim::ProxyState& state;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Simulation the readings come from.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    const sim::MujocoWorld& world;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Check the identity of the simulated device.
     *
     * @return Always true in the simulation.
     */
    bool check_whoami();

    /**
     * @brief Address of the gyroscope values inside mjData::sensordata.
     */
    int gyro_address;

    /**
     * @brief Address of the accelerometer values inside mjData::sensordata.
     */
    int accelerometer_address;

    /**
     * @brief Current angular velocity on each axis.
     */
    std::array<float, 3> angular_velocity{};

    /**
     * @brief Current linear acceleration on each axis.
     */
    std::array<float, 3> linear_acceleration{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_IMU_HPP
