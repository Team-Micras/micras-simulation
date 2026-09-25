/**
 * @file
 */

#include <cmath>
#include <stdexcept>
#include <string>

#include "micras/sim/core/clock.hpp"

namespace micras::sim {
Clock Clock::from_model(double timestep, uint32_t loop_time_us) {
    const double loop_period = loop_time_us * 1e-6;
    const int    steps = static_cast<int>(std::llround(loop_period / timestep));

    if (steps < 1) {
        throw std::runtime_error(
            "model timestep " + std::to_string(timestep) + " s does not fit in the firmware loop period " +
            std::to_string(loop_period) + " s (" + std::to_string(loop_time_us) +
            " us); the model needs a timestep of at most one loop period"
        );
    }

    if (std::abs(steps * timestep - loop_period) > 1e-12) {
        throw std::runtime_error(
            "model timestep " + std::to_string(timestep) + " s does not divide the firmware loop period " +
            std::to_string(loop_period) + " s (" + std::to_string(loop_time_us) + " us): " + std::to_string(steps) +
            " steps cover " + std::to_string(steps * timestep) + " s"
        );
    }

    Clock clock;
    clock.tick_us = loop_time_us;
    clock.tick_steps = steps;
    return clock;
}

void Clock::advance() {
    if (this->tick_us == 0) {
        throw std::logic_error("the simulated clock was advanced before from_model() configured it");
    }

    this->elapsed_us += this->tick_us;
    this->ticks++;
}

uint64_t Clock::total_ticks(double seconds, uint32_t us_per_tick) {
    return static_cast<uint64_t>(seconds * 1e6 / us_per_tick);
}

uint64_t Clock::tick_at(double seconds) const {
    return static_cast<uint64_t>(std::llround(seconds * 1e6 / this->tick_us));
}
}  // namespace micras::sim
