#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <doctest/doctest.h>

#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/simulation.hpp"
#include "support.hpp"

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
 * @brief The tiny robot's world and a clock of two steps per tick.
 */
class Run {
protected:
    Run() { load_tiny_world(this->context.world, this->context.clock); }

    // NOLINTNEXTLINE(*-non-private-member-variables-in-classes): the fixture is the test's own scope.
    RunContext context;

    /**
     * @brief Stands in for the firmware: yields once per tick and never ends.
     */
    // NOLINTNEXTLINE(*-non-private-member-variables-in-classes): the fixture is the test's own scope.
    FirmwareThread firmware{[](FirmwareThread& thread) {
        while (true) {
            thread.yield_tick();
        }
    }};
};

TEST_CASE_FIXTURE(Run, "Run.AdvancesTheClockOncePerTick") {
    Simulation simulation(this->context, this->firmware);
    simulation.run(5);

    CHECK_EQ(this->context.clock.tick_count(), 5U);
    CHECK_EQ(this->context.clock.now_us(), 5U * 1042U);
    CHECK_EQ(simulation.completed_ticks(), 5U);
}

TEST_CASE_FIXTURE(Run, "Run.CallsListenersInRegistrationOrder") {
    std::vector<std::string> trace;
    TracingListener          first(trace, "a");
    TracingListener          second(trace, "b");

    Simulation simulation(this->context, this->firmware);
    simulation.add_listener(first);
    simulation.add_listener(second);
    simulation.run(1);

    const std::vector<std::string> expected{"a:start",  "b:start",  "a:before0", "b:before0",
                                            "a:after0", "b:after0", "a:finish",  "b:finish"};
    CHECK_EQ(trace, expected);
}

TEST_CASE_FIXTURE(Run, "Run.AListenerCanStopTheRunBeforeTheTick") {
    std::vector<std::string> trace;
    TracingListener          quitter(trace, "q", RunControl::QUIT);

    Simulation simulation(this->context, this->firmware);
    simulation.add_listener(quitter);
    simulation.run(10);

    CHECK_EQ(simulation.completed_ticks(), 0U);
    CHECK_EQ(this->context.clock.tick_count(), 0U);
    CHECK_EQ(trace, (std::vector<std::string>{"q:start", "q:before0", "q:finish"}));
}

TEST_CASE_FIXTURE(Run, "Run.StopsWhenTheProgramEnds") {
    int ticks = 0;

    FirmwareThread program([&](FirmwareThread& thread) {
        for (ticks = 0; ticks < 3; ticks++) {
            thread.yield_tick();
        }
    });

    Simulation simulation(this->context, program);
    simulation.run(100);

    CHECK_LT(simulation.completed_ticks(), 100U);
    CHECK_EQ(ticks, 3);
}

TEST_CASE_FIXTURE(Run, "Run.CarriesAProgramErrorOutOfRun") {
    FirmwareThread program([](FirmwareThread&) { throw std::runtime_error("the firmware gave up"); });
    Simulation     simulation(this->context, program);

    CHECK_THROWS_AS(simulation.run(5), std::runtime_error);
}
}  // namespace
}  // namespace micras::sim
