/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>
#include <utility>

#include "micras/sim/devices/imu.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Largest and smallest raw word of a 16-bit output register.
 */
///@{
constexpr double word_max{32767.0};
constexpr double word_min{-32768.0};
///@}
}  // namespace

Imu::Imu(const MujocoWorld& world, Config config, const NoiseConfig& noise) :
    config{std::move(config)},
    noise{noise, this->config.name},
    gyro_address{world.sensor_address(this->config.gyro)},
    accelerometer_address{world.sensor_address(this->config.accelerometer)} {
    const ImuDescription& chip = this->config.description;

    for (std::size_t axis = 0; axis < 3; axis++) {
        this->bias.at(axis) = this->noise.gaussian(chip.gyro_bias);
        this->bias.at(3 + axis) = this->noise.gaussian(chip.accel_bias);
        this->scale.at(axis) = 1.0 + this->noise.gaussian(chip.gyro_scale_error);
        this->scale.at(3 + axis) = 1.0;
    }

    this->rate_error = this->noise.gaussian(chip.clock_error);
}

void Imu::sample(MujocoWorld& world, const Clock& clock) {
    const ImuDescription&       chip = this->config.description;
    const double                period = clock.us_per_tick() * 1e-6;
    const double                alpha = 1.0 - std::exp(-2.0 * std::numbers::pi * chip.bandwidth * period);
    const double                bandwidth = std::numbers::pi / 2.0 * chip.bandwidth;
    const std::array<double, 2> sigma{
        chip.gyro_noise_density * std::sqrt(bandwidth), chip.accel_noise_density * std::sqrt(bandwidth)
    };
    const std::array<double, 2> resolution{chip.gyro_resolution, chip.accel_resolution};

    for (std::size_t channel = 0; channel < channels; channel++) {
        const int    address = channel < 3 ? this->gyro_address : this->accelerometer_address;
        const double truth = world.sensor_value(address + static_cast<int>(channel % 3));
        this->filtered.at(channel) =
            this->started ? this->filtered.at(channel) + alpha * (truth - this->filtered.at(channel)) : truth;
    }

    this->started = true;
    this->phase += chip.output_rate * (1.0 + this->rate_error) * period;

    while (this->phase >= 1.0) {
        this->phase -= 1.0;

        if (this->has_pending) {
            this->delivered = this->pending;
            this->config.write(this->delivered);
        }

        for (std::size_t channel = 0; channel < channels; channel++) {
            const std::size_t kind = channel / 3;
            const double      value = this->scale.at(channel) * this->filtered.at(channel) + this->bias.at(channel) +
                                      this->noise.gaussian(sigma.at(kind));
            this->pending.at(channel) =
                static_cast<float>(std::clamp(std::round(value / resolution.at(kind)), word_min, word_max));
        }

        this->has_pending = true;
    }
}

std::vector<std::string> Imu::columns() const {
    const std::string& name = this->config.name;
    return {name + "_gyro_x",  name + "_gyro_y",  name + "_gyro_z",
            name + "_accel_x", name + "_accel_y", name + "_accel_z"};
}

void Imu::append(std::vector<CsvCell>& row) const {
    const ImuDescription& chip = this->config.description;

    for (std::size_t channel = 0; channel < channels; channel++) {
        row.emplace_back(
            static_cast<double>(this->delivered.at(channel)) *
            (channel < 3 ? chip.gyro_resolution : chip.accel_resolution)
        );
    }
}
}  // namespace micras::sim
