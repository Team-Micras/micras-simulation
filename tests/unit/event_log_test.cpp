#include <cmath>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

#include <doctest/doctest.h>
#include <mujoco/mjtype.h>
#include <mujoco/mujoco.h>

#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/core/span_at.hpp"
#include "micras/sim/recording/event_log.hpp"
#include "support.hpp"

namespace micras::sim {
namespace {
/**
 * @brief The tiny robot, lifted clear of the floor or lowered onto it at chosen instants.
 */
class Logging {
protected:
    Logging() {
        load_tiny_world(this->context.world, this->context.clock);
        this->chassis = this->context.world.require_id(mjOBJ_GEOM, "chassis");
        this->floor = this->context.world.require_id(mjOBJ_GEOM, "floor");
    }

    /**
     * @brief Put the robot on the floor or in the air at an instant, and let a log see it.
     *
     * @param log Log to show the tick to.
     * @param time Simulated time of the tick.
     * @param touching Whether the chassis rests on the floor.
     */
    void tick(EventLog& log, double time, bool touching) {
        MujocoWorld&            world = this->context.world;
        const std::span<mjtNum> positions(world.data()->qpos, static_cast<std::size_t>(world.model()->nq));

        at(positions, 2) = touching ? 0.005 : 0.1;
        world.data()->time = time;
        mj_forward(world.model(), world.data());
        log.on_after_tick(this->simulation);
    }

    // NOLINTBEGIN(*-non-private-member-variables-in-classes): the fixture is the test's own scope.
    RunContext     context;
    FirmwareThread firmware{[](FirmwareThread&) { }};
    Simulation     simulation{this->context, this->firmware};
    MapVariables   variables;
    int            chassis{-1};
    int            floor{-1};
    // NOLINTEND(*-non-private-member-variables-in-classes)
};

TEST_CASE_FIXTURE(Logging, "Logging.LogsOneCollisionPerContactEpisode") {
    EventLog log({this->chassis}, {}, {}, nullptr);

    this->tick(log, 0.0, false);
    this->tick(log, 0.001, true);
    this->tick(log, 0.002, true);
    this->tick(log, 0.003, true);

    CHECK_EQ(log.collisions(), 1U);
    REQUIRE_EQ(log.entries().size(), 1U);
    CHECK_EQ(log.entries().front().kind, "collision");
    CHECK_EQ(log.entries().front().detail, "chassis");
    CHECK_EQ(log.entries().front().time, doctest::Approx(0.001).epsilon(1e-12));
}

TEST_CASE_FIXTURE(Logging, "Logging.CountsAContactWithinTheSeparationTimeAsTheSameCollision") {
    EventLog log({this->chassis}, {}, {}, nullptr);

    this->tick(log, 0.0, true);
    this->tick(log, 0.01, false);
    this->tick(log, 0.03, false);
    this->tick(log, 0.04, true);

    CHECK_EQ(log.collisions(), 1U);
}

TEST_CASE_FIXTURE(Logging, "Logging.CountsAContactAfterTheSeparationTimeAsANewCollision") {
    EventLog log({this->chassis}, {}, {}, nullptr);

    this->tick(log, 0.0, true);
    this->tick(log, 0.02, true);
    this->tick(log, 0.05, false);
    this->tick(log, 0.08, true);

    CHECK_EQ(log.collisions(), 2U);
    REQUIRE_EQ(log.entries().size(), 2U);
    CHECK_EQ(log.entries().back().time, doctest::Approx(0.08).epsilon(1e-12));
}

TEST_CASE_FIXTURE(Logging, "Logging.NeverCountsAContactWithAnIgnoredGeom") {
    EventLog log({this->chassis}, {this->floor}, {}, nullptr);

    this->tick(log, 0.0, true);
    this->tick(log, 1.0, false);
    this->tick(log, 2.0, true);

    CHECK_EQ(log.collisions(), 0U);
    CHECK(log.entries().empty());
}

TEST_CASE_FIXTURE(Logging, "Logging.WatchesOnlyTheGeomsItWasGiven") {
    EventLog log({}, {}, {}, nullptr);

    this->tick(log, 0.0, true);

    CHECK_EQ(log.collisions(), 0U);
}

TEST_CASE_FIXTURE(Logging, "Logging.NamesEachNewStateFromItsTable") {
    EventLog log({}, {}, {{"state", {"INIT", "IDLE", "RUN"}}}, &this->variables);

    this->tick(log, 0.0, false);
    this->variables.set("state", 0);
    this->tick(log, 0.1, false);
    this->tick(log, 0.2, false);
    this->variables.set("state", 1);
    this->tick(log, 0.3, false);
    this->variables.set("state", 2);
    this->tick(log, 0.4, false);

    REQUIRE_EQ(log.entries().size(), 3U);
    CHECK_EQ(log.entries().at(0).kind, "state");
    CHECK_EQ(log.entries().at(0).detail, "INIT");
    CHECK_EQ(log.entries().at(0).time, doctest::Approx(0.1).epsilon(1e-12));
    CHECK_EQ(log.entries().at(1).detail, "IDLE");
    CHECK_EQ(log.entries().at(2).detail, "RUN");
    CHECK_EQ(log.entries().at(2).time, doctest::Approx(0.4).epsilon(1e-12));
}

TEST_CASE_FIXTURE(Logging, "Logging.WritesAValueWithNoNameAsANumber") {
    EventLog log({}, {}, {{"state", {"INIT"}}}, &this->variables);

    this->variables.set("state", 7);
    this->tick(log, 0.0, false);
    this->variables.set("state", -1);
    this->tick(log, 0.1, false);

    REQUIRE_EQ(log.entries().size(), 2U);
    CHECK_EQ(log.entries().at(0).detail, "7");
    CHECK_EQ(log.entries().at(1).detail, "-1");
}

TEST_CASE_FIXTURE(Logging, "Logging.SkipsAVariableThatIsNotReportedYet") {
    EventLog log({}, {}, {{"state", {"INIT", "IDLE"}}}, &this->variables);

    this->variables.set("state", 1);
    this->tick(log, 0.0, false);
    this->variables.set("state", std::nan(""));
    this->tick(log, 0.1, false);
    this->variables.set("state", 1);
    this->tick(log, 0.2, false);

    REQUIRE_EQ(log.entries().size(), 1U);
    CHECK_EQ(log.entries().front().detail, "IDLE");
}

TEST_CASE_FIXTURE(Logging, "Logging.WatchesEveryVariableItWasGiven") {
    EventLog log({}, {}, {{"mode", {"SLOW", "FAST"}}, {"state", {"INIT"}}}, &this->variables);

    this->variables.set("mode", 1);
    this->tick(log, 0.0, false);

    REQUIRE_EQ(log.entries().size(), 1U);
    CHECK_EQ(log.entries().front().kind, "mode");
    CHECK_EQ(log.entries().front().detail, "FAST");
}

TEST_CASE_FIXTURE(Logging, "Logging.KeepsTheFirstEntriesOnlyButKeepsCounting") {
    EventLog log({this->chassis}, {}, {{"counter", {}}}, &this->variables);

    for (std::size_t i = 0; i < EventLog::max_events + 10; i++) {
        this->variables.set("counter", static_cast<double>(i));
        this->tick(log, static_cast<double>(i), false);
    }

    this->tick(log, 1000.0, true);

    CHECK_EQ(log.entries().size(), EventLog::max_events);
    CHECK_EQ(log.entries().back().detail, std::to_string(EventLog::max_events - 1));
    CHECK_EQ(log.collisions(), 1U);
}
}  // namespace
}  // namespace micras::sim
