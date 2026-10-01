/**
 * @file
 */

#include <chrono>

#include <doctest/doctest.h>

#include "micras/sim/bridge/real_time_pacer.hpp"
#include "support.hpp"

namespace micras::sim {
namespace {
using std::chrono::microseconds;
using std::chrono::milliseconds;
}  // namespace

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
}  // namespace micras::sim
