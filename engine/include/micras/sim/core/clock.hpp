/**
 * @file
 *
 * @brief Simulated clock and every conversion between seconds, ticks and physics steps.
 */

#ifndef MICRAS_SIM_CORE_CLOCK_HPP
#define MICRAS_SIM_CORE_CLOCK_HPP

#include <cstdint>

namespace micras::sim {
/**
 * @brief Deterministic simulated clock, advanced once per firmware tick.
 *
 * @note This is the only place that converts between seconds, firmware ticks
 *       and physics steps. Nothing here reads the wall clock.
 */
class Clock {
public:
    /**
     * @brief Build a clock for a model timestep and a firmware loop period.
     *
     * @note One firmware tick must be an exact whole number of physics steps;
     *       anything else silently changes the effective dt the firmware
     *       integrates with, so it is rejected here.
     *
     * @param timestep Model timestep in seconds.
     * @param loop_time_us Firmware loop period in microseconds.
     * @return Clock configured for that pair.
     */
    static Clock from_model(double timestep, uint32_t loop_time_us);

    /**
     * @brief Advance the clock by exactly one firmware tick.
     */
    void advance();

    /**
     * @brief Get the current simulated time.
     *
     * @return Microseconds since the start of the run.
     */
    uint64_t now_us() const { return this->elapsed_us; }

    /**
     * @brief Get how many firmware ticks were executed so far.
     *
     * @return Tick count.
     */
    uint64_t tick_count() const { return this->ticks; }

    /**
     * @brief Get the duration of one firmware tick.
     *
     * @return Simulated microseconds per tick.
     */
    uint32_t us_per_tick() const { return this->tick_us; }

    /**
     * @brief Get how many physics steps make up one firmware tick.
     *
     * @return Number of mj_step calls per tick.
     */
    int steps_per_tick() const { return this->tick_steps; }

    /**
     * @brief Get how many ticks a duration spans.
     *
     * @note Also available before a model is loaded, which is when the command
     *       line is parsed; that is why it takes the tick length explicitly.
     *
     * @param seconds Duration in simulated seconds.
     * @param us_per_tick Simulated microseconds per firmware tick.
     * @return Number of whole ticks that fit in the duration.
     */
    static uint64_t total_ticks(double seconds, uint32_t us_per_tick);

    /**
     * @brief Get the tick an instant falls on.
     *
     * @param seconds Instant in simulated seconds.
     * @return Index of the nearest tick.
     */
    uint64_t tick_at(double seconds) const;

private:
    /**
     * @brief Simulated microseconds elapsed since the start of the run.
     */
    uint64_t elapsed_us{0};

    /**
     * @brief Number of firmware ticks executed so far.
     */
    uint64_t ticks{0};

    /**
     * @brief Simulated microseconds elapsed per firmware tick.
     *
     * @note Zero until from_model() supplies it, so a clock that was never
     *       configured fails on its first tick instead of quietly running at a
     *       plausible looking rate.
     */
    uint32_t tick_us{0};

    /**
     * @brief Physics steps executed per firmware tick.
     */
    int tick_steps{1};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_CLOCK_HPP
