#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/simulation.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Listener that records the order it was called in.
 */
class TracingListener : public IRunListener {
public:
    TracingListener(std::vector<std::string>& trace, std::string name, RunControl control = RunControl::RUN) :
        trace{trace}, name{std::move(name)}, control{control} { }

    void on_start(const Simulation& /*simulation*/) override { this->trace.push_back(this->name + ":start"); }

    RunControl on_before_tick(const Simulation& simulation) override {
        this->trace.push_back(this->name + ":before" + std::to_string(simulation.tick()));
        return this->control;
    }

    void on_after_tick(const Simulation& simulation) override {
        this->trace.push_back(this->name + ":after" + std::to_string(simulation.tick()));
    }

    void on_finish(const Simulation& /*simulation*/) override { this->trace.push_back(this->name + ":finish"); }

private:
    std::vector<std::string>& trace;  // NOLINT(*-avoid-const-or-ref-data-members)
    std::string               name;
    RunControl                control;
};

/**
 * @brief A world and a clock configured like every recorded run.
 */
class Run : public testing::Test {
protected:
    void SetUp() override {
        this->context.world.load(MICRAS_TEST_MODEL);
        this->context.clock = Clock::from_model(this->context.world.timestep(), 1042);
        this->context.world.reset();
    }

    // NOLINTNEXTLINE(*-non-private-member-variables-in-classes): the fixture is the test's own scope.
    RunContext context;

    /**
     * @brief Stands in for the firmware: yields once per tick and never ends.
     */
    // NOLINTNEXTLINE(*-non-private-member-variables-in-classes): the fixture is the test's own scope.
    FirmwareThread firmware{[this] {
        while (true) {
            this->firmware.yield_tick();
        }
    }};
};

TEST_F(Run, AdvancesTheClockOncePerTick) {
    Simulation simulation(this->context, this->firmware);
    simulation.run(5);

    EXPECT_EQ(this->context.clock.tick_count(), 5U);
    EXPECT_EQ(this->context.clock.now_us(), 5U * 1042U);
    EXPECT_EQ(simulation.completed_ticks(), 5U);
}

TEST_F(Run, CallsListenersInRegistrationOrder) {
    std::vector<std::string> trace;
    TracingListener          first(trace, "a");
    TracingListener          second(trace, "b");

    Simulation simulation(this->context, this->firmware);
    simulation.add_listener(first);
    simulation.add_listener(second);
    simulation.run(1);

    const std::vector<std::string> expected{"a:start",  "b:start",  "a:before0", "b:before0",
                                            "a:after0", "b:after0", "a:finish",  "b:finish"};
    EXPECT_EQ(trace, expected);
}

TEST_F(Run, AListenerCanStopTheRunBeforeTheTick) {
    std::vector<std::string> trace;
    TracingListener          quitter(trace, "q", RunControl::QUIT);

    Simulation simulation(this->context, this->firmware);
    simulation.add_listener(quitter);
    simulation.run(10);

    EXPECT_EQ(simulation.completed_ticks(), 0U);
    EXPECT_EQ(this->context.clock.tick_count(), 0U);
    EXPECT_EQ(trace, (std::vector<std::string>{"q:start", "q:before0", "q:finish"}));
}

TEST_F(Run, StopsWhenTheProgramEnds) {
    int             ticks = 0;
    FirmwareThread* handle = nullptr;

    FirmwareThread program([&] {
        for (ticks = 0; ticks < 3; ticks++) {
            handle->yield_tick();
        }
    });
    handle = &program;

    Simulation simulation(this->context, program);
    simulation.run(100);

    EXPECT_LT(simulation.completed_ticks(), 100U);
    EXPECT_EQ(ticks, 3);
}

TEST_F(Run, CarriesAProgramErrorOutOfRun) {
    FirmwareThread program([] { throw std::runtime_error("the firmware gave up"); });
    Simulation     simulation(this->context, program);

    EXPECT_THROW(simulation.run(5), std::runtime_error);
}
}  // namespace
}  // namespace micras::sim
