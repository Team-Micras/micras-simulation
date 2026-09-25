#include <cmath>
#include <map>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/recording/event_log.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Variables the test sets by hand, NaN until set.
 */
class SettableVariables : public VariableSource {
public:
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
    std::map<std::string, double> values;
};

/**
 * @brief The tiny robot, lifted clear of the floor or lowered onto it at chosen instants.
 */
class Logging : public testing::Test {
protected:
    void SetUp() override {
        this->context.world.load(MICRAS_TEST_MODEL);
        this->context.clock = Clock::from_model(this->context.world.timestep(), 1042);
        this->context.world.reset();
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

        positions[2] = touching ? 0.005 : 0.1;
        world.data()->time = time;
        mj_forward(world.model(), world.data());
        log.on_after_tick(this->simulation);
    }

    // NOLINTBEGIN(*-non-private-member-variables-in-classes): the fixture is the test's own scope.
    RunContext        context;
    FirmwareThread    firmware{[] {}};
    Simulation        simulation{this->context, this->firmware};
    SettableVariables variables;
    int               chassis{-1};
    int               floor{-1};
    // NOLINTEND(*-non-private-member-variables-in-classes)
};

TEST_F(Logging, LogsOneCollisionPerContactEpisode) {
    EventLog log({this->chassis}, {}, {}, nullptr);

    this->tick(log, 0.0, false);
    this->tick(log, 0.001, true);
    this->tick(log, 0.002, true);
    this->tick(log, 0.003, true);

    EXPECT_EQ(log.collisions(), 1U);
    ASSERT_EQ(log.entries().size(), 1U);
    EXPECT_EQ(log.entries().front().kind, "collision");
    EXPECT_EQ(log.entries().front().detail, "chassis");
    EXPECT_DOUBLE_EQ(log.entries().front().time, 0.001);
}

TEST_F(Logging, CountsAContactWithinTheSeparationTimeAsTheSameCollision) {
    EventLog log({this->chassis}, {}, {}, nullptr);

    this->tick(log, 0.0, true);
    this->tick(log, 0.01, false);
    this->tick(log, 0.03, false);
    this->tick(log, 0.04, true);

    EXPECT_EQ(log.collisions(), 1U);
}

TEST_F(Logging, CountsAContactAfterTheSeparationTimeAsANewCollision) {
    EventLog log({this->chassis}, {}, {}, nullptr);

    this->tick(log, 0.0, true);
    this->tick(log, 0.02, true);
    this->tick(log, 0.05, false);
    this->tick(log, 0.08, true);

    EXPECT_EQ(log.collisions(), 2U);
    ASSERT_EQ(log.entries().size(), 2U);
    EXPECT_DOUBLE_EQ(log.entries().back().time, 0.08);
}

TEST_F(Logging, NeverCountsAContactWithAnIgnoredGeom) {
    EventLog log({this->chassis}, {this->floor}, {}, nullptr);

    this->tick(log, 0.0, true);
    this->tick(log, 1.0, false);
    this->tick(log, 2.0, true);

    EXPECT_EQ(log.collisions(), 0U);
    EXPECT_TRUE(log.entries().empty());
}

TEST_F(Logging, WatchesOnlyTheGeomsItWasGiven) {
    EventLog log({}, {}, {}, nullptr);

    this->tick(log, 0.0, true);

    EXPECT_EQ(log.collisions(), 0U);
}

TEST_F(Logging, NamesEachNewStateFromItsTable) {
    EventLog log({}, {}, {{.variable = "state", .names = {"INIT", "IDLE", "RUN"}}}, &this->variables);

    this->tick(log, 0.0, false);
    this->variables.set("state", 0);
    this->tick(log, 0.1, false);
    this->tick(log, 0.2, false);
    this->variables.set("state", 1);
    this->tick(log, 0.3, false);
    this->variables.set("state", 2);
    this->tick(log, 0.4, false);

    ASSERT_EQ(log.entries().size(), 3U);
    EXPECT_EQ(log.entries().at(0).kind, "state");
    EXPECT_EQ(log.entries().at(0).detail, "INIT");
    EXPECT_DOUBLE_EQ(log.entries().at(0).time, 0.1);
    EXPECT_EQ(log.entries().at(1).detail, "IDLE");
    EXPECT_EQ(log.entries().at(2).detail, "RUN");
    EXPECT_DOUBLE_EQ(log.entries().at(2).time, 0.4);
}

TEST_F(Logging, WritesAValueWithNoNameAsANumber) {
    EventLog log({}, {}, {{.variable = "state", .names = {"INIT"}}}, &this->variables);

    this->variables.set("state", 7);
    this->tick(log, 0.0, false);
    this->variables.set("state", -1);
    this->tick(log, 0.1, false);

    ASSERT_EQ(log.entries().size(), 2U);
    EXPECT_EQ(log.entries().at(0).detail, "7");
    EXPECT_EQ(log.entries().at(1).detail, "-1");
}

TEST_F(Logging, SkipsAVariableThatIsNotReportedYet) {
    EventLog log({}, {}, {{.variable = "state", .names = {"INIT", "IDLE"}}}, &this->variables);

    this->variables.set("state", 1);
    this->tick(log, 0.0, false);
    this->variables.set("state", std::nan(""));
    this->tick(log, 0.1, false);
    this->variables.set("state", 1);
    this->tick(log, 0.2, false);

    ASSERT_EQ(log.entries().size(), 1U);
    EXPECT_EQ(log.entries().front().detail, "IDLE");
}

TEST_F(Logging, WatchesAVariableAddedLater) {
    EventLog log({}, {}, {}, &this->variables);
    log.watch({.variable = "mode", .names = {"SLOW", "FAST"}});

    this->variables.set("mode", 1);
    this->tick(log, 0.0, false);

    ASSERT_EQ(log.entries().size(), 1U);
    EXPECT_EQ(log.entries().front().kind, "mode");
    EXPECT_EQ(log.entries().front().detail, "FAST");
}

TEST_F(Logging, KeepsTheFirstEntriesOnlyButKeepsCounting) {
    EventLog log({this->chassis}, {}, {{.variable = "counter", .names = {}}}, &this->variables);

    for (std::size_t i = 0; i < EventLog::max_events + 10; i++) {
        this->variables.set("counter", static_cast<double>(i));
        this->tick(log, static_cast<double>(i), false);
    }

    this->tick(log, 1000.0, true);

    EXPECT_EQ(log.entries().size(), EventLog::max_events);
    EXPECT_EQ(log.entries().back().detail, std::to_string(EventLog::max_events - 1));
    EXPECT_EQ(log.collisions(), 1U);
}
}  // namespace
}  // namespace micras::sim
