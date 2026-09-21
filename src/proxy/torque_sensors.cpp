/**
 * @file
 *
 * @brief Torque sensor stub always reporting zero torque and current.
 */

#ifndef MICRAS_PROXY_TORQUE_SENSORS_CPP
#define MICRAS_PROXY_TORQUE_SENSORS_CPP

#include "micras/proxy/torque_sensors.hpp"  // NOLINT(misc-header-include-cycle)

namespace micras::proxy {
template <uint8_t num_of_sensors>
TTorqueSensors<num_of_sensors>::TTorqueSensors(const Config& /*config*/) { }

template <uint8_t num_of_sensors>
void TTorqueSensors<num_of_sensors>::calibrate() { }

template <uint8_t num_of_sensors>
void TTorqueSensors<num_of_sensors>::update() { }

template <uint8_t num_of_sensors>
float TTorqueSensors<num_of_sensors>::get_torque(uint8_t /*sensor_index*/) const {
    return 0.0F;
}

template <uint8_t num_of_sensors>
float TTorqueSensors<num_of_sensors>::get_torque_raw(uint8_t /*sensor_index*/) const {
    return 0.0F;
}

template <uint8_t num_of_sensors>
float TTorqueSensors<num_of_sensors>::get_current(uint8_t /*sensor_index*/) const {
    return 0.0F;
}

template <uint8_t num_of_sensors>
float TTorqueSensors<num_of_sensors>::get_current_raw(uint8_t /*sensor_index*/) const {
    return 0.0F;
}

template <uint8_t num_of_sensors>
float TTorqueSensors<num_of_sensors>::get_adc_reading(uint8_t /*sensor_index*/) const {
    return 0.0F;
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_TORQUE_SENSORS_CPP
