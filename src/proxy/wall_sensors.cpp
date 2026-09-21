/**
 * @file
 *
 * @brief Wall sensor proxy reading the MuJoCo rangefinder sensors.
 */

#ifndef MICRAS_PROXY_WALL_SENSORS_CPP
#define MICRAS_PROXY_WALL_SENSORS_CPP

#include <cmath>

#include "micras/core/utils.hpp"
#include "micras/proxy/wall_sensors.hpp"  // NOLINT(misc-header-include-cycle)
#include "micras/sim/core/simulation_context.hpp"

namespace micras::proxy {
template <uint8_t num_of_sensors>
TWallSensors<num_of_sensors>::TWallSensors(const Config& config) :
    state{sim::require_context(config.context).proxy_state},
    world{sim::require_context(config.context).world},
    max_sensor_reading{config.max_sensor_reading},
    filters{core::make_array<core::ButterworthFilter, num_of_sensors>(config.filter_cutoff, config.sampling_frequency)},
    base_readings{config.base_readings},
    uncertainty{config.uncertainty},
    constant{static_cast<float>(
        -std::pow(config.max_sensor_distance, 2) *
        std::log(1.0F - config.min_sensor_reading / config.max_sensor_reading)
    )} {
    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->sensor_addresses.at(i) = this->world.sensor_address(config.sensors.at(i));
    }

    this->turn_off();
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::turn_on() {
    this->leds_on = true;
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::turn_off() {
    this->leds_on = false;
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::update() {
    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->filters.at(i).update(this->get_adc_reading(i));
    }
}

template <uint8_t num_of_sensors>
bool TWallSensors<num_of_sensors>::get_wall(uint8_t sensor_index, bool disturbed) const {
    return this->filters.at(sensor_index).get_last() >
           this->base_readings.at(sensor_index) * this->uncertainty * (disturbed ? 1.2F : 1.0F);
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_reading(uint8_t sensor_index) const {
    return this->filters.at(sensor_index).get_last();
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_adc_reading(uint8_t sensor_index) const {
    const auto raw_reading = static_cast<float>(this->world.sensor_value(this->sensor_addresses.at(sensor_index)));

    if (raw_reading < 0.0F) {
        return this->record(sensor_index, 0.0F);
    }

    if (raw_reading == 0.0F) {
        return this->record(sensor_index, this->max_sensor_reading);
    }

    const float intensity = 1.0F / (raw_reading * raw_reading);

    return this->record(sensor_index, this->max_sensor_reading * (1.0F - std::exp(-this->constant * intensity)));
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::record(uint8_t sensor_index, float reading) const {
    static_assert(
        num_of_sensors <= sim::Sensors::wall_sensor_count,
        "the board has more wall sensors than the simulated boundary records"
    );

    this->state.sensors.wall_adc_readings.at(sensor_index) = reading;
    return reading;
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_sensor_error(uint8_t sensor_index) const {
    return this->get_reading(sensor_index) - this->base_readings.at(sensor_index);
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_sensor(uint8_t sensor_index) {
    this->base_readings.at(sensor_index) = this->get_reading(sensor_index);
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_WALL_SENSORS_CPP
