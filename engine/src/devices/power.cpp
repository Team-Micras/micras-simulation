/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <utility>

#include "micras/sim/devices/power.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Convert a voltage to ADC counts.
 *
 * @param voltage Voltage at the ADC input.
 * @param reference ADC reference voltage.
 * @param max_counts Counts at the reference.
 * @param noise Noise added, in counts.
 * @return Counts, clipped to the ADC's range.
 */
uint32_t to_counts(double voltage, double reference, double max_counts, double noise) {
    return static_cast<uint32_t>(std::clamp(std::round(voltage / reference * max_counts + noise), 0.0, max_counts));
}
}  // namespace

Battery::Battery(Config config, const NoiseConfig& noise) :
    config{std::move(config)},
    noise{noise, this->config.name},
    terminal_voltage{this->config.description.cells * this->config.description.cell_voltage} { }

void Battery::sample(MujocoWorld& /*world*/, const Clock& /*clock*/) {
    this->config.write(to_counts(
        this->terminal_voltage / this->config.divider, this->config.adc_reference, this->config.adc_max_counts,
        this->noise.gaussian(this->config.adc_noise_counts)
    ));
}

std::vector<std::string> Battery::columns() const {
    return {this->config.name + "_voltage"};
}

void Battery::append(std::vector<CsvCell>& row) const {
    row.emplace_back(this->terminal_voltage);
}

Fan::Fan(const MujocoWorld& world, Config config) :
    config{std::move(config)}, actuator_id{world.require_id(mjOBJ_ACTUATOR, this->config.actuator)} { }

void Fan::actuate(MujocoWorld& world, const Clock& clock) {
    const FanDescription& fan = this->config.description;
    double                target = 0.0;

    if (this->config.enabled()) {
        const double speed = std::clamp(static_cast<double>(this->config.duty()), 0.0, 100.0) / 100.0 *
                             this->config.supply_voltage() / fan.nominal_voltage;
        target = fan.max_downforce * speed * speed;
    }

    const double period = clock.us_per_tick() * 1e-6;
    const double blend = 1.0 - std::exp(-period / fan.time_constant);
    this->downforce += blend * (target - this->downforce);
    world.set_control(this->actuator_id, this->downforce);
}

std::vector<std::string> Fan::columns() const {
    return {this->config.name + "_downforce"};
}

void Fan::append(std::vector<CsvCell>& row) const {
    row.emplace_back(this->downforce);
}

CurrentSense::CurrentSense(Config config, const NoiseConfig& noise) :
    config{std::move(config)}, noise{noise, "current_sense"} { }

void CurrentSense::sample(MujocoWorld& /*world*/, const Clock& /*clock*/) {
    for (std::size_t channel = 0; channel < this->config.currents.size(); channel++) {
        const double current = std::abs(this->config.currents.at(channel)());
        this->config.write(
            channel, to_counts(
                         current * this->config.volts_per_amp, this->config.adc_reference, this->config.adc_max_counts,
                         this->noise.gaussian(this->config.adc_noise_counts)
                     )
        );
    }
}
}  // namespace micras::sim
