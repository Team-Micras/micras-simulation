/**
 * @file
 */

#ifndef MICRAS_PROXY_STOPWATCH_HPP
#define MICRAS_PROXY_STOPWATCH_HPP

#include <cstdint>

namespace micras::sim {
struct SimulationContext;
}  // namespace micras::sim

namespace micras::proxy {
/**
 * @brief Class to measure the time elapsed between two events.
 *
 * @note Backed by the simulated clock, never by wall time. A freshly built
 *       stopwatch reports one loop period rather than zero, so the firmware's
 *       very first loop sees the nominal period instead of a division by an
 *       uninitialised timer. At time zero that seed wraps the unsigned counter,
 *       which is well defined and gives exactly the intended reading.
 */
class Stopwatch {
public:
    /**
     * @brief Stopwatch configuration struct.
     */
    struct Config {
        /**
         * @brief Simulation holding the clock this stopwatch reads.
         */
        sim::SimulationContext* context{nullptr};
    };

    /**
     * @brief Construct a new Stopwatch object on the process-wide simulation.
     *
     * @note The firmware default-constructs stopwatches inside its own headers
     *       (states/wait.hpp), where no Config can be threaded through, so this
     *       constructor delegates to the one target.hpp declares. Anything that
     *       has a Config should pass it instead, so that a test running on its
     *       own SimulationContext is honoured.
     */
    Stopwatch();

    /**
     * @brief Construct a new Stopwatch object.
     *
     * @param config Configuration for the timer.
     */
    explicit Stopwatch(const Config& config);

    /**
     * @brief Reset the milliseconds timer counter.
     */
    void reset_ms();

    /**
     * @brief Reset the microseconds timer counter.
     */
    void reset_us();

    /**
     * @brief Get the simulated time elapsed since the last reset.
     *
     * @note Like the microsecond reading, a program that keeps asking at the
     *       same simulated instant is waiting for time it cannot get any other
     *       way, so it is handed a tick. The threshold is higher here because
     *       the firmware legitimately reads the same stopwatch twice in a tick
     *       when it classifies a button release.
     *
     * @return Time elapsed in milliseconds.
     */
    uint32_t elapsed_time_ms() const;

    /**
     * @brief Get the simulated time elapsed since the last reset.
     *
     * @note This is the one place where the simulated clock is not a pure
     *       observation: a program that keeps reading the same instant is
     *       waiting for time that only the simulation can give it, so the
     *       repeated read hands a tick over. The reading itself is always
     *       honest, now minus the last reset; a short measurement stays short.
     *
     * @return Time elapsed in microseconds.
     */
    uint32_t elapsed_time_us() const;

    /**
     * @brief Sleep on the process-wide simulation.
     *
     * @note Static in the firmware, and the hardware tests call it that way, so
     *       it reaches the same context the default constructor does. Returns
     *       at once when no program is being driven.
     *
     * @param time Time to sleep in milliseconds.
     */
    static void sleep_ms(uint32_t time);

    /**
     * @brief Hand ticks over until the simulated clock reaches the target.
     *
     * @note Returns at once when no program is being driven, since there would
     *       be nothing to hand the tick to.
     *
     * @param time Time to sleep in microseconds.
     */
    void sleep_us(uint32_t time) const;

private:
    /**
     * @brief Hand the tick over to the simulation, if a program is running.
     *
     * @return False when no tick was handed over, so a wait knows to give up.
     */
    bool yield_tick() const;

    /**
     * @brief Simulation the readings and the handoff come from.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    sim::SimulationContext& simulation;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Number of microsecond reads at the same simulated time that count as a spin.
     *
     * @note Two, because a firmware loop legitimately reads the same instant
     *       twice: once to close the previous loop and once to open the next.
     *       A third read means the program is waiting for time to pass.
     */
    static constexpr uint8_t max_repeated_reads{2};

    /**
     * @brief Number of millisecond reads at the same simulated time that count as a spin.
     *
     * @note Four: no firmware path reads one stopwatch in milliseconds more
     *       than twice within a tick, and the margin keeps the guard clear of
     *       any legitimate pattern.
     */
    static constexpr uint8_t max_repeated_ms_reads{4};

    /**
     * @brief Simulated time in microseconds at the last reset.
     */
    uint64_t counter{};

    /**
     * @brief Simulated time of the last reading.
     *
     * @note Not touched by a reset: the guard measures how many times a program
     *       asked at the same instant, and a proxy that resets its own
     *       stopwatch on every call, as the fan does, must not be able to hide
     *       a spin behind it.
     */
    mutable uint64_t last_read_us{};

    /**
     * @brief Forget both spin counters after a tick was handed over.
     *
     * @note Both, because the two readings share the instant they compare
     *       against: leaving one behind would make a stopwatch read in both
     *       units interfere with itself.
     */
    void note_yield() const;

    /**
     * @brief Consecutive readings taken at the same simulated time.
     */
    ///@{
    mutable uint8_t repeated_reads{0};
    mutable uint8_t repeated_ms_reads{0};
    ///@}
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_STOPWATCH_HPP
