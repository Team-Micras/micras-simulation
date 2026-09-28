#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <doctest/doctest.h>

#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/core/variable_source.hpp"
#include "micras/sim/devices/digital_input.hpp"
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
}  // namespace

/**
 * @brief Parse a scenario and return the message it was refused with.
 *
 * @param text Contents of a scenario file.
 * @return The error message, or empty when the scenario was accepted.
 */
static std::string refusal_of(std::string_view text) {
    try {
        Scenario::parse(text, "test.toml");
    } catch (const std::runtime_error& error) {
        return error.what();
    }

    return {};
}

namespace {
TEST_CASE("Scenario.ReadsTheRunSettings") {
    const Scenario scenario = Scenario::parse(full_scenario);

    CHECK_EQ(scenario.robot, "tiny");
    CHECK_EQ(scenario.arena, "maze1");
    CHECK_EQ(scenario.seconds, doctest::Approx(12.0).epsilon(1e-12));
    CHECK_EQ(scenario.seed, 7U);
    REQUIRE(scenario.start.has_value());

    const MujocoWorld::Placement start = scenario.start.value_or(MujocoWorld::Placement{});
    CHECK_EQ(start.x, doctest::Approx(0.09).epsilon(1e-12));
    CHECK_EQ(start.y, doctest::Approx(0.18).epsilon(1e-12));
    CHECK_EQ(start.yaw, doctest::Approx(std::numbers::pi / 2).epsilon(1e-12));
}

TEST_CASE("Scenario.LeavesOutWhatTheFileDoesNotSay") {
    const Scenario scenario = Scenario::parse("seconds = 4.5\n");

    CHECK(scenario.robot.empty());
    CHECK(scenario.arena.empty());
    CHECK_EQ(scenario.seconds, doctest::Approx(4.5).epsilon(1e-12));
    CHECK_FALSE(scenario.seed.has_value());
    CHECK_FALSE(scenario.start.has_value());
    CHECK(scenario.events.empty());
    CHECK_FALSE(scenario.stop.has_value());
}

TEST_CASE("Scenario.ReadsEveryKindOfEventInTimeOrder") {
    const Scenario scenario = Scenario::parse(full_scenario);

    REQUIRE_EQ(scenario.events.size(), 3U);

    const ScenarioEvent& press = scenario.events.at(0);
    CHECK_EQ(press.kind, ScenarioEvent::Kind::PRESS);
    CHECK_EQ(press.at, doctest::Approx(0.5).epsilon(1e-12));
    CHECK_EQ(press.duration, doctest::Approx(0.25).epsilon(1e-12));
    CHECK_EQ(press.target, "button");

    const ScenarioEvent& set = scenario.events.at(1);
    CHECK_EQ(set.kind, ScenarioEvent::Kind::SET);
    CHECK_EQ(set.target, "switch");
    CHECK(set.value);

    const ScenarioEvent& send = scenario.events.at(2);
    CHECK_EQ(send.kind, ScenarioEvent::Kind::SEND);
    CHECK_EQ(send.target, "hello");
}

TEST_CASE("Scenario.ReadsTheStopConditionAsANameOrANumber") {
    const Scenario named = Scenario::parse(full_scenario);
    const Scenario numbered = Scenario::parse("[stop]\nwhen = \"lap\"\nequals = 3\n");

    REQUIRE(named.stop.has_value());
    REQUIRE(numbered.stop.has_value());

    const StopCondition by_name = named.stop.value_or(StopCondition{});
    const StopCondition by_number = numbered.stop.value_or(StopCondition{});
    CHECK_EQ(by_name.variable, "state");
    CHECK_EQ(by_name.equals, std::vector<std::string>{"IDLE"});
    CHECK_EQ(by_name.after, doctest::Approx(5.0).epsilon(1e-12));
    CHECK_EQ(by_number.equals, std::vector<std::string>{"3"});
    CHECK_EQ(by_number.after, doctest::Approx(0.0).epsilon(1e-12));
}

TEST_CASE("Scenario.ReadsAStopConditionWithSeveralValues") {
    const Scenario scenario = Scenario::parse("[stop]\nwhen = \"state\"\nequals = [\"IDLE\", 2]\n");

    REQUIRE(scenario.stop.has_value());
    CHECK_EQ(scenario.stop.value_or(StopCondition{}).equals, (std::vector<std::string>{"IDLE", "2"}));
    CHECK(refusal_of("[stop]\nwhen = \"a\"\nequals = []\n").contains("equals names no value"));
}

TEST_CASE("Scenario.RefusesAnUnknownKeyNamingIt") {
    CHECK(refusal_of("second = 4\n").contains("unknown key second"));
    CHECK(refusal_of("[start]\nx = 0\ny = 0\nyaw = 0\n").contains("unknown key yaw"));
    CHECK(refusal_of("[[events]]\nat = 1\nsend = \"a\"\nuntil = 2\n").contains("events[0]: unknown key until"));
    CHECK(refusal_of("[stop]\nwhen = \"a\"\nequals = 1\nbefore = 2\n").contains("unknown key before"));
}

TEST_CASE("Scenario.RefusesAnEventThatIsNotExactlyOneThing") {
    CHECK_FALSE(refusal_of("[[events]]\nat = 1\n").empty());
    CHECK_FALSE(refusal_of("[[events]]\nat = 1\nsend = \"a\"\npress = \"b\"\nfor = 1\n").empty());
    CHECK_FALSE(refusal_of("events = [1, 2]\n").empty());
}

TEST_CASE("Scenario.RefusesAnEventMissingWhatItNeeds") {
    CHECK(refusal_of("[[events]]\nsend = \"a\"\n").contains("at must be a number"));
    CHECK(refusal_of("[[events]]\nat = 1\npress = \"a\"\n").contains("for must be a number"));
    CHECK(refusal_of("[[events]]\nat = 1\nset = \"a\"\n").contains("true or false"));
    CHECK(refusal_of("[[events]]\nat = 1\n").contains("exactly one of press, set or send"));
}

TEST_CASE("Scenario.RefusesNegativeTimes") {
    CHECK(refusal_of("[[events]]\nat = -1\nsend = \"a\"\n").contains("negative"));
    CHECK(refusal_of("[[events]]\nat = 1\npress = \"a\"\nfor = -0.5\n").contains("negative"));
}

TEST_CASE("Scenario.RefusesAnIncompleteStartOrStop") {
    CHECK(refusal_of("[start]\nx = 0\nyaw_deg = 0\n").contains("y must be a number"));
    CHECK(refusal_of("[stop]\nequals = 1\n").contains("when must name"));
    CHECK(refusal_of("[stop]\nwhen = \"a\"\nequals = true\n").contains("equals must be"));
}

TEST_CASE("Scenario.RefusesAFileThatIsNotToml") {
    CHECK(refusal_of("seconds = = 4\n").starts_with("test.toml"));
    CHECK_THROWS_AS(Scenario::load("no_such_scenario.toml"), std::runtime_error);
}

/**
 * @brief A world, one input and one message: what a robot target hands a scenario.
 */
class Playing {
protected:
    Playing() {
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

TEST_CASE_FIXTURE(Playing, "Playing.PressesAnInputForTheTimeTheEventSays") {
    this->play("[[events]]\nat = 0.01\npress = \"button\"\nfor = 0.02\n", 50);

    const uint64_t first = this->context.clock.tick_at(0.01);
    const uint64_t last = first + this->context.clock.tick_at(0.02) - 1;

    REQUIRE_FALSE(this->probe.pressed_ticks.empty());
    CHECK_EQ(this->probe.pressed_ticks.front(), first);
    CHECK_EQ(this->probe.pressed_ticks.back(), last);
    CHECK_EQ(this->probe.pressed_ticks.size(), last - first + 1);
    CHECK_FALSE(this->button.is_active());
    CHECK(this->button_level);
}

TEST_CASE_FIXTURE(Playing, "Playing.DrivesTheInputsPinWithItsPolarity") {
    this->play("[[events]]\nat = 0\nset = \"button\"\nvalue = true\n", 1);

    CHECK(this->button.is_active());
    CHECK_FALSE(this->button_level);
}

TEST_CASE_FIXTURE(Playing, "Playing.LeavesTheInputsAloneOnceAHumanHasTheBoard") {
    this->play("[[events]]\nat = 0.01\npress = \"button\"\nfor = 0.02\n", 50, nullptr, true);

    CHECK(this->probe.pressed_ticks.empty());
}

TEST_CASE_FIXTURE(Playing, "Playing.QueuesAMessageForTheFirmware") {
    this->play("[[events]]\nat = 0.005\nsend = \"hello\"\n", 10);

    CHECK_EQ(this->context.serial.take_for_firmware(10), (std::vector<uint8_t>{1, 2, 3}));
}

TEST_CASE_FIXTURE(Playing, "Playing.RefusesAnEventOnSomethingTheRobotDoesNotHave") {
    CHECK_THROWS_AS(this->play("[[events]]\nat = 0\npress = \"lever\"\nfor = 1\n", 1), std::runtime_error);
    CHECK_THROWS_AS(this->play("[[events]]\nat = 0\nsend = \"goodbye\"\n", 1), std::runtime_error);
}

TEST_CASE_FIXTURE(Playing, "Playing.RefusesAStopOnAStateTheVariableDoesNotHave") {
    CHECK_THROWS_AS(this->play("[stop]\nwhen = \"state\"\nequals = \"LOST\"\n", 1), std::runtime_error);
    CHECK_THROWS_AS(this->play("[stop]\nwhen = \"lap\"\nequals = \"LAST\"\n", 1), std::runtime_error);
}

TEST_CASE_FIXTURE(Playing, "Playing.StopsTheRunOnceTheConditionHoldsAfterItsDelay") {
    const MapVariables variables({{"state", 1.0}});
    const auto player = this->play("[stop]\nwhen = \"state\"\nequals = \"IDLE\"\nafter = 0.1\n", 1000, &variables);

    const auto stop_tick = static_cast<uint64_t>(std::ceil(0.1 / 1042e-6));

    REQUIRE(player->stopped_at().has_value());
    CHECK_EQ(this->completed, stop_tick);
    CHECK_EQ(
        player->stopped_at().value_or(-1.0),
        doctest::Approx(static_cast<double>(stop_tick) * 1042 * 1e-6).epsilon(1e-12)
    );
    CHECK_GE(player->stopped_at().value_or(-1.0), 0.1);
}

TEST_CASE_FIXTURE(Playing, "Playing.StopsOnANumericValue") {
    const MapVariables variables({{"lap", 3.0}});
    const auto         player = this->play("[stop]\nwhen = \"lap\"\nequals = 3\n", 100, &variables);

    CHECK_EQ(player->stopped_at(), 0.0);
    CHECK_EQ(this->completed, 0U);
}

TEST_CASE_FIXTURE(Playing, "Playing.StopsOnAnyOfSeveralValues") {
    const MapVariables variables({{"state", 2.0}});
    const auto         player = this->play("[stop]\nwhen = \"state\"\nequals = [\"IDLE\", \"RUN\"]\n", 100, &variables);

    CHECK_EQ(player->stopped_at(), 0.0);
    CHECK_EQ(this->completed, 0U);
}

TEST_CASE_FIXTURE(Playing, "Playing.StopsOnlyOnceTheValueIsReachedTheCountedTimes") {
    SteppedVariables variables("state", {{0, 1.0}, {10, 2.0}, {20, 1.0}, {30, 2.0}, {40, 1.0}});
    const auto       player =
        this->play("[stop]\nwhen = \"state\"\nequals = \"IDLE\"\ncount = 3\n", 100, &variables, false, &variables);

    CHECK_EQ(this->completed, 40U);
    CHECK(player->stopped_at().has_value());
}

TEST_CASE_FIXTURE(Playing, "Playing.AbortsOnTheFirstAbortValueWhateverTheCount") {
    SteppedVariables variables("state", {{0, 1.0}, {10, 0.0}});
    const auto       player = this->play(
        "[stop]\nwhen = \"state\"\nequals = \"IDLE\"\ncount = 3\nabort = [\"INIT\"]\n", 100, &variables, false,
        &variables
    );

    CHECK_EQ(this->completed, 10U);
    CHECK(player->stopped_at().has_value());
}

TEST_CASE_FIXTURE(Playing, "Playing.HoldsAnEventUntilItsConditionHolds") {
    SteppedVariables variables("state", {{0, 1.0}, {25, 2.0}});
    this->play(
        "[[events]]\nat = 0\npress = \"button\"\nfor = 0.002\nwhen = \"state\"\nequals = \"RUN\"\n", 40, &variables,
        false, &variables
    );

    REQUIRE_FALSE(this->probe.pressed_ticks.empty());
    CHECK_EQ(this->probe.pressed_ticks.front(), 25U);
}

TEST_CASE("Scenario.ReadsAnEventConditionAndAStopCount") {
    const Scenario scenario = Scenario::parse(
        "[[events]]\nat = 1\nsend = \"a\"\nwhen = \"state\"\nequals = \"IDLE\"\n"
        "[stop]\nwhen = \"state\"\nequals = \"IDLE\"\ncount = 2\n"
    );

    REQUIRE_EQ(scenario.events.size(), 1U);
    CHECK_EQ(scenario.events.front().when, "state");
    CHECK_EQ(scenario.events.front().equals, "IDLE");
    CHECK_EQ(scenario.stop.value_or(StopCondition{}).count, 2);
    CHECK(refusal_of("[[events]]\nat = 1\nsend = \"a\"\nwhen = \"state\"\n").contains("needs both"));
    CHECK(refusal_of("[stop]\nwhen = \"a\"\nequals = 1\ncount = 0\n").contains("count must be"));
}

TEST_CASE_FIXTURE(Playing, "Playing.RunsToTheEndWhileTheConditionDoesNotHold") {
    const MapVariables variables({{"state", 2.0}});
    const auto         player = this->play("[stop]\nwhen = \"state\"\nequals = \"IDLE\"\n", 20, &variables);

    CHECK_FALSE(player->stopped_at().has_value());
    CHECK_EQ(this->completed, 20U);
}

TEST_CASE_FIXTURE(Playing, "Playing.NeverStopsWithoutAnythingToReadTheVariableFrom") {
    const auto player = this->play("[stop]\nwhen = \"state\"\nequals = \"INIT\"\n", 20);

    CHECK_FALSE(player->stopped_at().has_value());
    CHECK_EQ(this->completed, 20U);
}
}  // namespace
}  // namespace micras::sim
