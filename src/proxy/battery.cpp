/**
 * @file
 *
 * @brief Battery proxy reporting a constant, noise free voltage.
 */

#include "micras/proxy/battery.hpp"

namespace micras::proxy {
Battery::Battery(const Config& config) :
    reading{config.max_voltage}, max_voltage{config.max_voltage}, max_reading{config.max_reading} { }

void Battery::update() { }

float Battery::get_voltage() const {
    return this->reading;
}

float Battery::get_voltage_raw() const {
    return (this->reading / this->max_voltage) * static_cast<float>(this->max_reading);
}

float Battery::get_adc_reading() const {
    return this->reading / this->max_voltage;
}
}  // namespace micras::proxy
