/**
 * @file
 *
 * @brief Rotary sensor proxy reading a MuJoCo wheel joint position.
 */

#include <stdexcept>

#include "micras/proxy/rotary_sensor.hpp"
#include "micras/sim/core/simulation_context.hpp"

namespace micras::proxy {
RotarySensor::RotarySensor(const Config& config) :
    state{sim::require_context(config.context).proxy_state},
    wheel{config.wheel},
    world{sim::require_context(config.context).world},
    sensor_address{this->world.sensor_address(config.sensor)} {
    if (this->wheel >= sim::Sensors::wheel_count) {
        throw std::out_of_range("rotary sensor wheel index is outside the wheels the boundary records");
    }
}

float RotarySensor::get_position() const {
    const auto position = static_cast<float>(this->world.sensor_value(this->sensor_address));
    this->state.sensors.encoder_positions.at(this->wheel) = position;
    return position;
}
}  // namespace micras::proxy
