#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/scenario/scenario.hpp"
#include "support.hpp"

namespace micras::sim {
namespace {
/**
 * @brief A scenario that uses every section of the format, its events out of order.
 */
constexpr std::string_view full_scenario{R"(
robot = "tiny"
arena = "maze1"
seconds = 12
seed = 7

[start]
x = 0.09
y = 0.18
yaw_deg = 90

[[events]]
at = 2.0
send = "hello"

[[events]]
at = 0.5
press = "button"
for = 0.25

[[events]]
at = 1.0
set = "switch"
value = true

[stop]
when = "state"
equals = "IDLE"
after = 5.0
)"};

/**
 * @brief A variable that takes a new value on given ticks, updated before the player reads it.
 */
class SteppedVariables : public VariableSource, public IRunListener {
public:
    SteppedVariables(std::string name, std::map<uint64_t, double> steps) :
        name{std::move(name)}, steps{std::move(steps)} { }

    double value_of(const std::string& name) const override {
        return name == this->name ? this->current : std::nan("");
    }

    RunControl on_before_tick(const Simulation& simulation) override {
        const auto step = this->steps.find(simulation.tick());

        if (step != this->steps.end()) {
            this->current = step->second;
        }

        return RunControl::RUN;
    }

private:
    std::string                name;
    std::map<uint64_t, double> steps;
    double                     current{std::nan("")};
};

/**
 * @brief Records, before every tick, what the scenario has done so far.
 */
class Probe : public IRunListener {
public:
    explicit Probe(const DigitalInput& input) : input{input} { }

    RunControl on_before_tick(const Simulation& simulation) override {
        if (this->input.is_active()) {
            this->pressed_ticks.push_back(simulation.tick());
        }

        return RunControl::RUN;
    }

    // NOLINTBEGIN(*-non-private-member-variables-in-classes): the test reads what the probe saw.
    std::vector<uint64_t> pressed_ticks;
    // NOLINTEND(*-non-private-member-variables-in-classes)

private:
    const DigitalInput& input;  // NOLINT(*-avoid-const-or-ref-data-members): the fixture outlives the probe.
};

/**
 * @brief Parse a scenario and return the message it was refused with.
 *
 * @param text Contents of a scenario file.
 * @return The error message, or empty when the scenario was accepted.
 */
std::string refusal_of(std::string_view text) {
    try {
        Scenario::parse(text, "test.toml");
    } catch (const std::runtime_error& error) {
        return error.what();
    }

    return {};
}

TEST(Scenario, ReadsTheRunSettings) {
    const Scenario scenario = Scenario::parse(full_scenario);

    EXPECT_EQ(scenario.robot, "tiny");
    EXPECT_EQ(scenario.arena, "maze1");
    EXPECT_DOUBLE_EQ(scenario.seconds, 12.0);
    EXPECT_EQ(scenario.seed, 7U);
    ASSERT_TRUE(scenario.start.has_value());

    const MujocoWorld::Placement start = scenario.start.value_or(MujocoWorld::Placement{});
    EXPECT_DOUBLE_EQ(start.x, 0.09);
    EXPECT_DOUBLE_EQ(start.y, 0.18);
    EXPECT_DOUBLE_EQ(start.yaw, std::numbers::pi / 2);
}

TEST(Scenario, LeavesOutWhatTheFileDoesNotSay) {
    const Scenario scenario = Scenario::parse("seconds = 4.5\n");

    EXPECT_TRUE(scenario.robot.empty());
    EXPECT_TRUE(scenario.arena.empty());
    EXPECT_DOUBLE_EQ(scenario.seconds, 4.5);
    EXPECT_FALSE(scenario.seed.has_value());
    EXPECT_FALSE(scenario.start.has_value());
    EXPECT_TRUE(scenario.events.empty());
    EXPECT_FALSE(scenario.stop.has_value());
}

TEST(Scenario, ReadsEveryKindOfEventInTimeOrder) {
    const Scenario scenario = Scenario::parse(full_scenario);

    ASSERT_EQ(scenario.events.size(), 3U);

    const ScenarioEvent& press = scenario.events.at(0);
    EXPECT_EQ(press.kind, ScenarioEvent::Kind::PRESS);
    EXPECT_DOUBLE_EQ(press.at, 0.5);
    EXPECT_DOUBLE_EQ(press.duration, 0.25);
    EXPECT_EQ(press.target, "button");

    const ScenarioEvent& set = scenario.events.at(1);
    EXPECT_EQ(set.kind, ScenarioEvent::Kind::SET);
    EXPECT_EQ(set.target, "switch");
    EXPECT_TRUE(set.value);

    const ScenarioEvent& send = scenario.events.at(2);
    EXPECT_EQ(send.kind, ScenarioEvent::Kind::SEND);
    EXPECT_EQ(send.target, "hello");
}

TEST(Scenario, ReadsTheStopConditionAsANameOrANumber) {
    const Scenario named = Scenario::parse(full_scenario);
    const Scenario numbered = Scenario::parse("[stop]\nwhen = \"lap\"\nequals = 3\n");

    ASSERT_TRUE(named.stop.has_value());
    ASSERT_TRUE(numbered.stop.has_value());

    const StopCondition by_name = named.stop.value_or(StopCondition{});
    const StopCondition by_number = numbered.stop.value_or(StopCondition{});
    EXPECT_EQ(by_name.variable, "state");
    EXPECT_EQ(by_name.equals, std::vector<std::string>{"IDLE"});
    EXPECT_DOUBLE_EQ(by_name.after, 5.0);
    EXPECT_EQ(by_number.equals, std::vector<std::string>{"3"});
    EXPECT_DOUBLE_EQ(by_number.after, 0.0);
}

TEST(Scenario, ReadsAStopConditionWithSeveralValues) {
    const Scenario scenario = Scenario::parse("[stop]\nwhen = \"state\"\nequals = [\"IDLE\", 2]\n");

    ASSERT_TRUE(scenario.stop.has_value());
    EXPECT_EQ(scenario.stop.value_or(StopCondition{}).equals, (std::vector<std::string>{"IDLE", "2"}));
    EXPECT_TRUE(refusal_of("[stop]\nwhen = \"a\"\nequals = []\n").contains("equals names no value"));
}

TEST(Scenario, RefusesAnUnknownKeyNamingIt) {
    EXPECT_TRUE(refusal_of("second = 4\n").contains("unknown key second"));
    EXPECT_TRUE(refusal_of("[start]\nx = 0\ny = 0\nyaw = 0\n").contains("unknown key yaw"));
    EXPECT_TRUE(refusal_of("[[events]]\nat = 1\nsend = \"a\"\nuntil = 2\n").contains("events[0]: unknown key until"));
    EXPECT_TRUE(refusal_of("[stop]\nwhen = \"a\"\nequals = 1\nbefore = 2\n").contains("unknown key before"));
}

TEST(Scenario, RefusesAnEventThatIsNotExactlyOneThing) {
    EXPECT_FALSE(refusal_of("[[events]]\nat = 1\n").empty());
    EXPECT_FALSE(refusal_of("[[events]]\nat = 1\nsend = \"a\"\npress = \"b\"\nfor = 1\n").empty());
    EXPECT_FALSE(refusal_of("events = [1, 2]\n").empty());
}

TEST(Scenario, RefusesAnEventMissingWhatItNeeds) {
    EXPECT_TRUE(refusal_of("[[events]]\nsend = \"a\"\n").contains("at must be a number"));
    EXPECT_TRUE(refusal_of("[[events]]\nat = 1\npress = \"a\"\n").contains("for must be a number"));
    EXPECT_TRUE(refusal_of("[[events]]\nat = 1\nset = \"a\"\n").contains("true or false"));
    EXPECT_TRUE(refusal_of("[[events]]\nat = 1\n").contains("exactly one of press, set or send"));
}

TEST(Scenario, RefusesNegativeTimes) {
    EXPECT_TRUE(refusal_of("[[events]]\nat = -1\nsend = \"a\"\n").contains("negative"));
    EXPECT_TRUE(refusal_of("[[events]]\nat = 1\npress = \"a\"\nfor = -0.5\n").contains("negative"));
}

TEST(Scenario, RefusesAnIncompleteStartOrStop) {
    EXPECT_TRUE(refusal_of("[start]\nx = 0\nyaw_deg = 0\n").contains("y must be a number"));
    EXPECT_TRUE(refusal_of("[stop]\nequals = 1\n").contains("when must name"));
    EXPECT_TRUE(refusal_of("[stop]\nwhen = \"a\"\nequals = true\n").contains("equals must be"));
}

TEST(Scenario, RefusesAFileThatIsNotToml) {
    EXPECT_TRUE(refusal_of("seconds = = 4\n").starts_with("test.toml"));
    EXPECT_THROW(Scenario::load("no_such_scenario.toml"), std::runtime_error);
}

/**
 * @brief A world, one input and one message: what a robot target hands a scenario.
 */
class Playing : public testing::Test {
protected:
    void SetUp() override {
        load_tiny_world(this->context.world, this->context.clock);

        this->hooks.inputs.emplace("button", &this->button);
        this->hooks.messages.emplace("hello", std::vector<uint8_t>{1, 2, 3});
        this->hooks.state_names.emplace("state", std::vector<std::string>{"INIT", "IDLE", "RUN"});
    }

    /**
     * @brief Play a scenario for a number of ticks.
     *
     * @param text The scenario.
     * @param ticks Most ticks to run.
     * @param variables Where the stop condition reads, or null.
     * @param human Whether a human took the board over before the run.
     * @param first Listener that runs before the player each tick, or null.
     * @return The player, after the run.
     */
    std::unique_ptr<ScenarioPlayer> play(
        std::string_view text, uint64_t ticks, const VariableSource* variables = nullptr, bool human = false,
        IRunListener* first = nullptr
    ) {
        auto player =
            std::make_unique<ScenarioPlayer>(Scenario::parse(text), this->hooks, this->context.serial, variables);

        if (human) {
            player->hand_over();
        }

        Simulation simulation(this->context, this->firmware);

        if (first != nullptr) {
            simulation.add_listener(*first);
        }

        simulation.add_listener(*player);
        simulation.add_listener(this->probe);
        simulation.run(ticks);
        this->completed = simulation.completed_ticks();
        return player;
    }

    // NOLINTBEGIN(*-non-private-member-variables-in-classes): the fixture is the test's own scope.
    RunContext    context;
    bool          button_level{true};
    DigitalInput  button{{.name = "button", .active_low = true, .drive = [this](bool level) {
                             this->button_level = level;
                          }}};
    ScenarioHooks hooks;
    Probe         probe{this->button};
    uint64_t      completed{0};

    /**
     * @brief Stands in for the firmware: yields once per tick and never ends.
     */
    FirmwareThread firmware{[](FirmwareThread& thread) {
        while (true) {
            thread.yield_tick();
        }
    }};
    // NOLINTEND(*-non-private-member-variables-in-classes)
};

TEST_F(Playing, PressesAnInputForTheTimeTheEventSays) {
    this->play("[[events]]\nat = 0.01\npress = \"button\"\nfor = 0.02\n", 50);

    const uint64_t first = this->context.clock.tick_at(0.01);
    const uint64_t last = first + this->context.clock.tick_at(0.02) - 1;

    ASSERT_FALSE(this->probe.pressed_ticks.empty());
    EXPECT_EQ(this->probe.pressed_ticks.front(), first);
    EXPECT_EQ(this->probe.pressed_ticks.back(), last);
    EXPECT_EQ(this->probe.pressed_ticks.size(), last - first + 1);
    EXPECT_FALSE(this->button.is_active());
    EXPECT_TRUE(this->button_level);
}

TEST_F(Playing, DrivesTheInputsPinWithItsPolarity) {
    this->play("[[events]]\nat = 0\nset = \"button\"\nvalue = true\n", 1);

    EXPECT_TRUE(this->button.is_active());
    EXPECT_FALSE(this->button_level);
}

TEST_F(Playing, LeavesTheInputsAloneOnceAHumanHasTheBoard) {
    this->play("[[events]]\nat = 0.01\npress = \"button\"\nfor = 0.02\n", 50, nullptr, true);

    EXPECT_TRUE(this->probe.pressed_ticks.empty());
}

TEST_F(Playing, QueuesAMessageForTheFirmware) {
    this->play("[[events]]\nat = 0.005\nsend = \"hello\"\n", 10);

    EXPECT_EQ(this->context.serial.take_for_firmware(10), (std::vector<uint8_t>{1, 2, 3}));
}

TEST_F(Playing, RefusesAnEventOnSomethingTheRobotDoesNotHave) {
    EXPECT_THROW(this->play("[[events]]\nat = 0\npress = \"lever\"\nfor = 1\n", 1), std::runtime_error);
    EXPECT_THROW(this->play("[[events]]\nat = 0\nsend = \"goodbye\"\n", 1), std::runtime_error);
}

TEST_F(Playing, RefusesAStopOnAStateTheVariableDoesNotHave) {
    EXPECT_THROW(this->play("[stop]\nwhen = \"state\"\nequals = \"LOST\"\n", 1), std::runtime_error);
    EXPECT_THROW(this->play("[stop]\nwhen = \"lap\"\nequals = \"LAST\"\n", 1), std::runtime_error);
}

TEST_F(Playing, StopsTheRunOnceTheConditionHoldsAfterItsDelay) {
    const MapVariables variables({{"state", 1.0}});
    const auto player = this->play("[stop]\nwhen = \"state\"\nequals = \"IDLE\"\nafter = 0.1\n", 1000, &variables);

    const auto stop_tick = static_cast<uint64_t>(std::ceil(0.1 / 1042e-6));

    ASSERT_TRUE(player->stopped_at().has_value());
    EXPECT_EQ(this->completed, stop_tick);
    EXPECT_DOUBLE_EQ(player->stopped_at().value_or(-1.0), static_cast<double>(stop_tick) * 1042 * 1e-6);
    EXPECT_GE(player->stopped_at().value_or(-1.0), 0.1);
}

TEST_F(Playing, StopsOnANumericValue) {
    const MapVariables variables({{"lap", 3.0}});
    const auto         player = this->play("[stop]\nwhen = \"lap\"\nequals = 3\n", 100, &variables);

    EXPECT_EQ(player->stopped_at(), 0.0);
    EXPECT_EQ(this->completed, 0U);
}

TEST_F(Playing, StopsOnAnyOfSeveralValues) {
    const MapVariables variables({{"state", 2.0}});
    const auto         player = this->play("[stop]\nwhen = \"state\"\nequals = [\"IDLE\", \"RUN\"]\n", 100, &variables);

    EXPECT_EQ(player->stopped_at(), 0.0);
    EXPECT_EQ(this->completed, 0U);
}

TEST_F(Playing, StopsOnlyOnceTheValueIsReachedTheCountedTimes) {
    SteppedVariables variables("state", {{0, 1.0}, {10, 2.0}, {20, 1.0}, {30, 2.0}, {40, 1.0}});
    const auto       player =
        this->play("[stop]\nwhen = \"state\"\nequals = \"IDLE\"\ncount = 3\n", 100, &variables, false, &variables);

    EXPECT_EQ(this->completed, 40U);
    EXPECT_TRUE(player->stopped_at().has_value());
}

TEST_F(Playing, AbortsOnTheFirstAbortValueWhateverTheCount) {
    SteppedVariables variables("state", {{0, 1.0}, {10, 0.0}});
    const auto       player = this->play(
        "[stop]\nwhen = \"state\"\nequals = \"IDLE\"\ncount = 3\nabort = [\"INIT\"]\n", 100, &variables, false,
        &variables
    );

    EXPECT_EQ(this->completed, 10U);
    EXPECT_TRUE(player->stopped_at().has_value());
}

TEST_F(Playing, HoldsAnEventUntilItsConditionHolds) {
    SteppedVariables variables("state", {{0, 1.0}, {25, 2.0}});
    this->play(
        "[[events]]\nat = 0\npress = \"button\"\nfor = 0.002\nwhen = \"state\"\nequals = \"RUN\"\n", 40, &variables,
        false, &variables
    );

    ASSERT_FALSE(this->probe.pressed_ticks.empty());
    EXPECT_EQ(this->probe.pressed_ticks.front(), 25U);
}

TEST(Scenario, ReadsAnEventConditionAndAStopCount) {
    const Scenario scenario = Scenario::parse(
        "[[events]]\nat = 1\nsend = \"a\"\nwhen = \"state\"\nequals = \"IDLE\"\n"
        "[stop]\nwhen = \"state\"\nequals = \"IDLE\"\ncount = 2\n"
    );

    ASSERT_EQ(scenario.events.size(), 1U);
    EXPECT_EQ(scenario.events.front().when, "state");
    EXPECT_EQ(scenario.events.front().equals, "IDLE");
    EXPECT_EQ(scenario.stop.value_or(StopCondition{}).count, 2);
    EXPECT_TRUE(refusal_of("[[events]]\nat = 1\nsend = \"a\"\nwhen = \"state\"\n").contains("needs both"));
    EXPECT_TRUE(refusal_of("[stop]\nwhen = \"a\"\nequals = 1\ncount = 0\n").contains("count must be"));
}

TEST_F(Playing, RunsToTheEndWhileTheConditionDoesNotHold) {
    const MapVariables variables({{"state", 2.0}});
    const auto         player = this->play("[stop]\nwhen = \"state\"\nequals = \"IDLE\"\n", 20, &variables);

    EXPECT_FALSE(player->stopped_at().has_value());
    EXPECT_EQ(this->completed, 20U);
}

TEST_F(Playing, NeverStopsWithoutAnythingToReadTheVariableFrom) {
    const auto player = this->play("[stop]\nwhen = \"state\"\nequals = \"INIT\"\n", 20);

    EXPECT_FALSE(player->stopped_at().has_value());
    EXPECT_EQ(this->completed, 20U);
}
}  // namespace
}  // namespace micras::sim
