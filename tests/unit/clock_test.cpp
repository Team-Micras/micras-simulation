#include <doctest/doctest.h>
#include <stdexcept>

#include "micras/sim/core/clock.hpp"

namespace micras::sim {
namespace {
/**
 * @brief A timestep that divides the loop into two steps.
 */
Clock harness_clock() {
    return Clock::from_model(0.000521, 1042);
}

TEST_CASE("Clock.DerivesTheStepCountFromTheModel") {
    const Clock clock = harness_clock();
    CHECK_EQ(clock.steps_per_tick(), 2);
    CHECK_EQ(clock.us_per_tick(), 1042U);
    CHECK_EQ(clock.now_us(), 0U);
    CHECK_EQ(clock.tick_count(), 0U);
}

TEST_CASE("Clock.RejectsATimestepThatDoesNotDivideTheLoop") {
    CHECK_THROWS_AS(Clock::from_model(0.0004, 1042), std::runtime_error);
    CHECK_THROWS_AS(Clock::from_model(0.002, 1042), std::runtime_error);
}

TEST_CASE("Clock.AdvancesOneTickAtATime") {
    Clock clock = harness_clock();
    clock.advance();
    clock.advance();

    CHECK_EQ(clock.tick_count(), 2U);
    CHECK_EQ(clock.now_us(), 2084U);
}

TEST_CASE("Clock.CountsTheWholeTicksOfADuration") {
    const Clock clock = harness_clock();
    CHECK_EQ(clock.total_ticks(4.0), 3838U);
    CHECK_EQ(clock.total_ticks(8.0), 7677U);
}

TEST_CASE("Clock.RoundsInstantsToTheNearestTick") {
    const Clock clock = harness_clock();
    CHECK_EQ(clock.tick_at(0.5), 480U);
    CHECK_EQ(clock.tick_at(0.0), 0U);
}
}  // namespace
}  // namespace micras::sim
