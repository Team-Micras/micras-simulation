/**
 * @file
 *
 * @brief IMU proxy reading the MuJoCo gyroscope and accelerometer sensors.
 */

#include "micras/proxy/imu.hpp"
#include "micras/sim/core/simulation_context.hpp"

namespace micras::proxy {
Imu::Imu(const Config& config) :
    state{sim::require_context(config.context).proxy_state},
    world{sim::require_context(config.context).world},
    gyro_address{this->world.sensor_address(config.gyro_sensor)},
    accelerometer_address{this->world.sensor_address(config.accelerometer_sensor)} {
    this->update();
}

void Imu::update() {
    for (std::size_t i = 0; i < sim::Sensors::axis_count; i++) {
        const int offset = static_cast<int>(i);

        this->angular_velocity.at(i) = static_cast<float>(this->world.sensor_value(this->gyro_address + offset));
        this->linear_acceleration.at(i) =
            static_cast<float>(this->world.sensor_value(this->accelerometer_address + offset));
    }

    this->state.sensors.angular_velocity = this->angular_velocity;
    this->state.sensors.linear_acceleration = this->linear_acceleration;
}

float Imu::get_angular_velocity(Axis axis) const {
    return this->angular_velocity.at(static_cast<uint8_t>(axis));
}

float Imu::get_linear_acceleration(Axis axis) const {
    return this->linear_acceleration.at(static_cast<uint8_t>(axis));
}

void Imu::calibrate() { }

// NOLINTNEXTLINE(readability-convert-member-functions-to-static): mirrors the firmware proxy API.
bool Imu::was_initialized() const {
    return true;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static): mirrors the firmware proxy API.
bool Imu::check_whoami() {
    return true;
}
}  // namespace micras::proxy
