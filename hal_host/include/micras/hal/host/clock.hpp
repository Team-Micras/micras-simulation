/**
 * @file
 *
 * @brief The only place time exists for a firmware running on a host.
 */

#ifndef MICRAS_HAL_HOST_CLOCK_HPP
#define MICRAS_HAL_HOST_CLOCK_HPP

#include <cstdint>
#include <functional>

namespace micras::hal::host {
/**
 * @brief Counts the core cycles the firmware sees, and hands time over to the world.
 *
 * @note Every read of the timer costs a fixed quantum of simulated time, one
 *       microsecond unless configured otherwise, since a read on the robot also
 *       takes time and a loop that polls the timer must see it advance. When a
 *       read crosses the end of a step, the handover callback runs: whatever
 *       drives the world advances it by one step and returns, and the read
 *       returns after it. So a busy wait runs the world exactly as long as it
 *       waits, and time inside an iteration is the number of timer reads times
 *       the quantum: deterministic, but not a measurement of anything.
 *
 * @note The counter wraps at 32 bits exactly as the cycle counter does.
 */
class Clock {
public:
    /**
     * @brief Called at every step boundary a read crosses.
     */
    using Handover = std::function<void()>;

    /**
     * @brief Get the process-wide clock.
     *
     * @return The clock.
     */
    static Clock& instance();

    /**
     * @brief Set the core frequency and the cost of one read.
     *
     * @param cycles_per_microsecond Core clock, in cycles per microsecond.
     * @param quantum_us Simulated time one read costs, in microseconds.
     */
    void configure(uint32_t cycles_per_microsecond, uint32_t quantum_us = 1);

    /**
     * @brief Hand time over to the world at every step.
     *
     * @param step_us Length of a step, in microseconds.
     * @param handover Called when a read crosses a step boundary.
     */
    void set_handover(uint32_t step_us, Handover handover);

    /**
     * @brief Stop handing time over; reads keep counting.
     */
    void clear_handover();

    /**
     * @brief Read the cycle counter, as the firmware does.
     *
     * @return Cycles since the start, modulo 2^32.
     */
    uint32_t read_cycles();

    /**
     * @brief Read the millisecond counter, as the firmware does.
     *
     * @return Milliseconds since the start, modulo 2^32.
     */
    uint32_t read_ms();

    /**
     * @brief Get the time without charging for it.
     *
     * @return Cycles since the start, not wrapped.
     */
    uint64_t now() const { return this->cycles; }

    /**
     * @brief Get the core frequency.
     *
     * @return Cycles per microsecond.
     */
    uint32_t cycles_per_microsecond() const { return this->cycles_per_us; }

    /**
     * @brief Get how many step boundaries were handed over.
     *
     * @return Number of handovers.
     */
    uint64_t steps() const { return this->handed_over; }

    /**
     * @brief Rewind to the start, with no handover.
     */
    void reset();

private:
    /**
     * @brief Charge one read and hand over at every boundary it crosses.
     */
    void charge();

    /**
     * @brief Core cycles per microsecond.
     */
    uint32_t cycles_per_us{1};

    /**
     * @brief Cycles one read costs.
     */
    uint64_t quantum{1};

    /**
     * @brief Cycles since the start.
     */
    uint64_t cycles{0};

    /**
     * @brief Cycles per step, zero when nothing is handed over.
     */
    uint64_t step{0};

    /**
     * @brief Cycle count of the next step boundary.
     */
    uint64_t next_boundary{0};

    /**
     * @brief Number of boundaries handed over.
     */
    uint64_t handed_over{0};

    /**
     * @brief What runs at a boundary.
     */
    Handover handover;
};
}  // namespace micras::hal::host

#endif  // MICRAS_HAL_HOST_CLOCK_HPP
