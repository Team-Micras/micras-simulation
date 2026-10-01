/**
 * @file
 *
 * @brief Holds a run back to the wall clock, for a frontend that lives in real time.
 */

#ifndef MICRAS_SIM_BRIDGE_REAL_TIME_PACER_HPP
#define MICRAS_SIM_BRIDGE_REAL_TIME_PACER_HPP

#include <chrono>
#include <cstdint>

namespace micras::sim {
/**
 * @brief The wall clock a pacer reads and waits on.
 */
class IWallClock {
public:
    IWallClock() = default;

    IWallClock(const IWallClock&) = delete;
    IWallClock(IWallClock&&) = delete;
    IWallClock& operator=(const IWallClock&) = delete;
    IWallClock& operator=(IWallClock&&) = delete;

    virtual ~IWallClock() = default;

    /**
     * @brief Get the current wall time.
     *
     * @return The time now.
     */
    virtual std::chrono::steady_clock::time_point now() const = 0;

    /**
     * @brief Block the calling thread until a wall time.
     *
     * @param deadline Time to wake up at.
     */
    virtual void sleep_until(std::chrono::steady_clock::time_point deadline) = 0;
};

/**
 * @brief The process's steady clock, shared by every pacer that is not under test.
 *
 * @return The steady wall clock.
 */
IWallClock& steady_wall_clock();

/**
 * @brief Keeps simulated time from running ahead of the wall clock.
 *
 * @note A monitor budgets its link in wall time, as the robot's radio does, and
 *       a run left free produces faster than the link carries. The pacer only
 *       waits: simulated time, and so every recorded sample, is the same paced
 *       or not.
 *
 * @note A run that falls behind by more than behind_slack is not made to catch
 *       up: the pacer moves its anchor to the present instead, so time lost in a
 *       slow stretch is never paid back beyond the slack by running faster than
 *       real time later. A shorter lag, such as a spike in the work of one tick
 *       or a sleep that overran, is recovered, which keeps the rate at real time
 *       under jitter. The trade-off is the burst: the run may outpace the wall
 *       clock by at most behind_slack plus ahead_slack, the lag it recovers and
 *       the lead it is allowed.
 */
class RealTimePacer {
public:
    /**
     * @brief How far simulated time may run ahead of the wall clock before the pacer waits.
     */
    static constexpr std::chrono::microseconds ahead_slack{1000};

    /**
     * @brief How far simulated time may fall behind the wall clock and still be recovered.
     */
    static constexpr std::chrono::microseconds behind_slack{5000};

    /**
     * @brief Pace against a wall clock.
     *
     * @param wall Clock to read and wait on; it must outlive the pacer.
     */
    explicit RealTimePacer(IWallClock& wall);

    /**
     * @brief Anchor simulated time to the present.
     *
     * @param simulated_us Simulated time now, in microseconds.
     */
    void start(uint64_t simulated_us);

    /**
     * @brief Wait until the wall clock reaches simulated time, or re-anchor when it is past it.
     *
     * @param simulated_us Simulated time now, in microseconds.
     */
    void pace(uint64_t simulated_us);

private:
    /**
     * @brief Re-anchor simulated time to the present.
     *
     * @param simulated_us Simulated time now, in microseconds.
     * @param wall_now Wall time now.
     */
    void anchor_at(uint64_t simulated_us, std::chrono::steady_clock::time_point wall_now);

    /**
     * @brief Clock to read and wait on.
     *
     * @note Bound for the life of the pacer; the clock outlives it.
     */
    IWallClock& wall;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Wall time the anchor was taken at.
     */
    std::chrono::steady_clock::time_point anchor_wall;

    /**
     * @brief Simulated time the anchor was taken at, in microseconds.
     */
    uint64_t anchor_us{0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_BRIDGE_REAL_TIME_PACER_HPP
