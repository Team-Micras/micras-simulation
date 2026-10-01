/**
 * @file
 */

#include <chrono>
#include <cstdint>
#include <thread>

#include "micras/sim/bridge/real_time_pacer.hpp"

namespace micras::sim {
namespace {
/**
 * @brief The process's steady clock.
 */
class SteadyWallClock : public IWallClock {
public:
    std::chrono::steady_clock::time_point now() const override { return std::chrono::steady_clock::now(); }

    void sleep_until(std::chrono::steady_clock::time_point deadline) override {
        std::this_thread::sleep_until(deadline);
    }
};
}  // namespace

IWallClock& steady_wall_clock() {
    static SteadyWallClock clock;
    return clock;
}

RealTimePacer::RealTimePacer(IWallClock& wall) : wall{wall}, anchor_wall{wall.now()} { }

void RealTimePacer::start(uint64_t simulated_us) {
    this->anchor_at(simulated_us, this->wall.now());
}

void RealTimePacer::pace(uint64_t simulated_us) {
    const auto wall_now = this->wall.now();
    const auto target = this->anchor_wall + std::chrono::microseconds(simulated_us - this->anchor_us);

    if (target - wall_now > ahead_slack) {
        this->wall.sleep_until(target);
    } else if (wall_now - target > behind_slack) {
        this->anchor_at(simulated_us, wall_now);
    }
}

void RealTimePacer::anchor_at(uint64_t simulated_us, std::chrono::steady_clock::time_point wall_now) {
    this->anchor_wall = wall_now;
    this->anchor_us = simulated_us;
}
}  // namespace micras::sim
