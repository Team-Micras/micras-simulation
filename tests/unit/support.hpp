/**
 * @file
 *
 * @brief What several of the engine's tests share: the tiny robot's world, bytes collected from the
 * firmware, variables set by hand, and a wall clock that only moves when told.
 */

#ifndef MICRAS_SIM_TESTS_SUPPORT_HPP
#define MICRAS_SIM_TESTS_SUPPORT_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "micras/sim/bridge/real_time_pacer.hpp"
#include "micras/sim/core/clock.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/serial_bus.hpp"
#include "micras/sim/core/variable_source.hpp"

namespace micras::sim {
/**
 * @brief Load the tiny robot, with a clock of two steps per 1042 us tick, and reset it.
 *
 * @param world World to load it into.
 * @param clock Clock to configure.
 */
inline void load_tiny_world(MujocoWorld& world, Clock& clock) {
    world.load(MICRAS_TEST_MODEL);
    clock = Clock::from_model(world.timestep(), 1042);
    world.reset();
}

/**
 * @brief Listener that keeps every byte the firmware sent.
 */
class ByteCollector : public ISerialListener {
public:
    /**
     * @brief Keep the bytes.
     *
     * @param bytes Bytes leaving the firmware.
     */
    void on_firmware_bytes(std::span<const uint8_t> bytes) override {
        this->received.insert(this->received.end(), bytes.begin(), bytes.end());
    }

    // NOLINTNEXTLINE(*-non-private-member-variables-in-classes): the test reads what arrived.
    std::vector<uint8_t> received;
};

/**
 * @brief Variables the test sets, NaN for any name it has not set.
 */
class MapVariables : public VariableSource {
public:
    /**
     * @brief Start with some values.
     *
     * @param values Values by name.
     */
    explicit MapVariables(std::map<std::string, double> values = {}) : values{std::move(values)} { }

    /**
     * @brief Get a variable.
     *
     * @param name Name of the variable.
     * @return Its value, or NaN when it was never set.
     */
    double value_of(const std::string& name) const override {
        const auto found = this->values.find(name);
        return found == this->values.end() ? std::nan("") : found->second;
    }

    /**
     * @brief Set a variable.
     *
     * @param name Name of the variable.
     * @param value Its new value.
     */
    void set(const std::string& name, double value) { this->values[name] = value; }

private:
    /**
     * @brief Values by name.
     */
    std::map<std::string, double> values;
};

/**
 * @brief Wall clock that only moves when the test advances it, or when something sleeps on it.
 */
class FakeWallClock : public IWallClock {
public:
    /**
     * @brief Get the fake time.
     *
     * @return The time now.
     */
    std::chrono::steady_clock::time_point now() const override { return this->current; }

    /**
     * @brief Jump to the deadline at once, and count the sleep.
     *
     * @param deadline Time to wake up at.
     */
    void sleep_until(std::chrono::steady_clock::time_point deadline) override {
        this->current = std::max(this->current, deadline);
        this->sleeps++;
    }

    /**
     * @brief Let wall time pass, as a run's own work would.
     *
     * @param duration Time to pass.
     */
    void advance(std::chrono::steady_clock::duration duration) { this->current += duration; }

    /**
     * @brief Get how many times something slept.
     *
     * @return Number of sleeps.
     */
    int sleep_count() const { return this->sleeps; }

private:
    /**
     * @brief The fake time.
     */
    std::chrono::steady_clock::time_point current;

    /**
     * @brief Number of sleeps.
     */
    int sleeps{0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_TESTS_SUPPORT_HPP
