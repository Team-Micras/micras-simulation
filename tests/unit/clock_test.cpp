#include <gtest/gtest.h>

#include "micras/sim/core/clock.hpp"

namespace micras::sim {
namespace {
/**
 * @brief The pair every recorded run uses: a 521 us timestep and a 1042 us loop.
 */
Clock harness_clock() {
    return Clock::from_model(0.000521, 1042);
}

TEST(Clock, DerivesTheStepCountFromTheModel) {
    const Clock clock = harness_clock();
    EXPECT_EQ(clock.steps_per_tick(), 2);
    EXPECT_EQ(clock.us_per_tick(), 1042U);
    EXPECT_EQ(clock.now_us(), 0U);
    EXPECT_EQ(clock.tick_count(), 0U);
}

TEST(Clock, RejectsATimestepThatDoesNotDivideTheLoop) {
    EXPECT_THROW(Clock::from_model(0.0004, 1042), std::runtime_error);
    EXPECT_THROW(Clock::from_model(0.002, 1042), std::runtime_error);
}

TEST(Clock, AdvancesOneTickAtATime) {
    Clock clock = harness_clock();
    clock.advance();
    clock.advance();

    EXPECT_EQ(clock.tick_count(), 2U);
    EXPECT_EQ(clock.now_us(), 2084U);

    clock.reset();
    EXPECT_EQ(clock.tick_count(), 0U);
    EXPECT_EQ(clock.now_us(), 0U);
}

TEST(Clock, CountsTheTicksOfTheRecordedScenarios) {
    const Clock clock = harness_clock();
    EXPECT_EQ(clock.total_ticks(4.0), 3838U);
    EXPECT_EQ(clock.total_ticks(8.0), 7677U);
}

TEST(Clock, RoundsInstantsToTheNearestTick) {
    const Clock clock = harness_clock();
    EXPECT_EQ(clock.tick_at(0.5), 480U);
    EXPECT_EQ(clock.tick_at(0.0), 0U);
}

TEST(Clock, CountsTheTicksTheButtonMustBeHeldFor) {
    const Clock clock = harness_clock();

    for (const uint32_t target : {250U, 501U, 2001U}) {
        const uint64_t ticks = clock.ticks_for_elapsed_ms(target);
        EXPECT_GE(ticks * clock.us_per_tick() / 1000, target);
        EXPECT_LT((ticks - 1) * clock.us_per_tick() / 1000, target);
    }
}
}  // namespace
}  // namespace micras::sim
