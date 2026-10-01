/**
 * @file
 */

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>

#include <doctest/doctest.h>

#include "micras/sim/bridge/real_time_pacer.hpp"
#include "support.hpp"

namespace micras::sim {
namespace {
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::steady_clock;

/**
 * @brief How a paced run of ticks went against the wall clock.
 */
struct PacedRun {
    /**
     * @brief Simulated time the run covered.
     */
    microseconds simulated{0};

    /**
     * @brief Wall time it took.
     */
    steady_clock::duration wall{0};

    /**
     * @brief The most simulated time ever ran ahead of wall time over any stretch of the run.
     */
    steady_clock::duration burst{0};
};
}  // namespace

/**
 * @brief Pace a run whose ticks each take some wall time of their own.
 *
 * @param ticks Number of ticks to run.
 * @param work Wall time the work of a tick takes, by tick.
 * @return How the run went.
 */
static PacedRun run_paced(uint64_t ticks, const std::function<steady_clock::duration(uint64_t)>& work) {
    constexpr uint64_t tick_us = 1042;

    FakeWallClock wall;
    RealTimePacer pacer(wall);
    const auto    start = wall.now();
    pacer.start(0);

    PacedRun result;
    auto     worst_lag = steady_clock::duration::min();

    for (uint64_t tick = 0; tick < ticks; tick++) {
        const uint64_t     simulated_us = tick * tick_us;
        const microseconds simulated(simulated_us);
        pacer.pace(simulated_us);
        const auto lag = (wall.now() - start) - simulated;
        worst_lag = std::max(worst_lag, lag);
        result.burst = std::max(result.burst, worst_lag - lag);
        wall.advance(work(tick));
    }

    result.simulated = microseconds(ticks * tick_us);
    result.wall = wall.now() - start;
    return result;
}

/**
 * @brief Get how fast a run went against the wall clock.
 *
 * @param run The run.
 * @return Simulated seconds per wall second.
 */
static double rate_of(const PacedRun& run) {
    return std::chrono::duration<double>(run.simulated).count() / std::chrono::duration<double>(run.wall).count();
}

TEST_CASE("RealTimePacer.HoldsARunThatIsAheadToTheWallClock") {
    FakeWallClock wall;
    RealTimePacer pacer(wall);
    const auto    start = wall.now();
    pacer.start(0);

    wall.advance(milliseconds(1));
    pacer.pace(5000);

    CHECK_EQ(wall.sleep_count(), 1);
    CHECK_EQ(wall.now() - start, milliseconds(5));
}

TEST_CASE("RealTimePacer.LeavesADriftWithinTheSlackAlone") {
    FakeWallClock wall;
    RealTimePacer pacer(wall);
    const auto    start = wall.now();
    pacer.start(0);

    pacer.pace(1000);
    wall.advance(milliseconds(3));
    pacer.pace(2000);
    pacer.pace(4000);

    CHECK_EQ(wall.sleep_count(), 0);
    CHECK_EQ(wall.now() - start, milliseconds(3));
}

TEST_CASE("RealTimePacer.NeverMakesARunThatFellBehindCatchUp") {
    FakeWallClock wall;
    RealTimePacer pacer(wall);
    pacer.start(0);

    wall.advance(milliseconds(10));
    pacer.pace(2000);
    const auto behind = wall.now();

    pacer.pace(3000);
    CHECK_EQ(wall.sleep_count(), 0);

    pacer.pace(5000);
    CHECK_EQ(wall.sleep_count(), 1);
    CHECK_EQ(wall.now() - behind, milliseconds(3));
}

TEST_CASE("RealTimePacer.AnchorsWhereTheRunStarts") {
    FakeWallClock wall;
    RealTimePacer pacer(wall);
    wall.advance(milliseconds(50));
    const auto start = wall.now();
    pacer.start(20000);

    pacer.pace(22500);

    CHECK_EQ(wall.now() - start, microseconds(2500));
}

TEST_CASE("RealTimePacer.KeepsRealTimeWhenSomeTicksSpike") {
    const PacedRun run =
        run_paced(5000, [](uint64_t tick) { return tick % 10 == 0 ? microseconds(4000) : microseconds(300); });

    CHECK_EQ(rate_of(run), doctest::Approx(1.0).epsilon(0.002));
    CHECK_LE(run.burst, RealTimePacer::behind_slack);
}

TEST_CASE("RealTimePacer.KeepsRealTimeWhenTheWorkAlternates") {
    const PacedRun run =
        run_paced(5000, [](uint64_t tick) { return tick % 2 == 0 ? microseconds(150) : microseconds(900); });

    CHECK_EQ(rate_of(run), doctest::Approx(1.0).epsilon(0.002));
    CHECK_LE(run.burst, RealTimePacer::behind_slack);
}

TEST_CASE("RealTimePacer.BoundsTheBurstAfterALongStall") {
    const PacedRun run = run_paced(5000, [](uint64_t tick) {
        if (tick == 1000) {
            return steady_clock::duration{milliseconds(200)};
        }

        return steady_clock::duration{tick % 10 == 0 ? microseconds(4900) : microseconds(100)};
    });

    CHECK_GE(run.wall - run.simulated, milliseconds(190));
    CHECK_LE(run.burst, RealTimePacer::behind_slack + RealTimePacer::ahead_slack);
}
}  // namespace micras::sim
