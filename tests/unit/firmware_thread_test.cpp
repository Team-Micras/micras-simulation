#include <atomic>
#include <stdexcept>
#include <vector>

#include <doctest/doctest.h>

#include "micras/sim/core/firmware_thread.hpp"

namespace micras::sim {
namespace {
TEST_CASE("FirmwareThread.RunsTheProgramUpToEachYield") {
    std::vector<int> trace;

    FirmwareThread firmware([&](FirmwareThread& thread) {
        for (int step = 0; step < 3; step++) {
            trace.push_back(step);
            thread.yield_tick();
        }
    });

    firmware.run_until_yield();
    CHECK_EQ(trace, (std::vector<int>{0}));

    firmware.run_until_yield();
    CHECK_EQ(trace, (std::vector<int>{0, 1}));

    firmware.run_until_yield();
    CHECK_EQ(trace, (std::vector<int>{0, 1, 2}));
}

TEST_CASE("FirmwareThread.NeverRunsBesideTheSimulation") {
    std::atomic<int> awake{0};
    std::atomic<int> overlaps{0};

    FirmwareThread firmware([&](FirmwareThread& thread) {
        while (true) {
            if (awake.fetch_add(1) != 0) {
                overlaps++;
            }

            awake--;
            thread.yield_tick();
        }
    });

    for (int tick = 0; tick < 200; tick++) {
        firmware.run_until_yield();

        if (awake.fetch_add(1) != 0) {
            overlaps++;
        }

        awake--;
    }

    CHECK_EQ(overlaps, 0);
}

/**
 * @brief Records that the firmware thread was unwound rather than killed.
 */
class UnwindSentinel {
public:
    explicit UnwindSentinel(bool& flag) : flag{flag} { }

    UnwindSentinel(const UnwindSentinel&) = delete;
    UnwindSentinel(UnwindSentinel&&) = delete;
    UnwindSentinel& operator=(const UnwindSentinel&) = delete;
    UnwindSentinel& operator=(UnwindSentinel&&) = delete;

    ~UnwindSentinel() { this->flag = true; }

private:
    bool& flag;  // NOLINT(*-avoid-const-or-ref-data-members): the test owns the flag.
};

TEST_CASE("FirmwareThread.EndlessProgramsAreUnwoundByFinish") {
    bool destructor_ran = false;

    {
        FirmwareThread firmware([&](FirmwareThread& thread) {
            const UnwindSentinel sentinel(destructor_ran);

            while (true) {
                thread.yield_tick();
            }
        });

        firmware.run_until_yield();
        CHECK_FALSE(firmware.has_finished());
        firmware.finish();
    }

    CHECK(destructor_ran);
}

/**
 * @brief Hands a tick over while being destroyed, as a waiting destructor would.
 */
class YieldingSentinel {
public:
    YieldingSentinel(FirmwareThread& thread, bool& flag) : thread{thread}, flag{flag} { }

    YieldingSentinel(const YieldingSentinel&) = delete;
    YieldingSentinel(YieldingSentinel&&) = delete;
    YieldingSentinel& operator=(const YieldingSentinel&) = delete;
    YieldingSentinel& operator=(YieldingSentinel&&) = delete;

    ~YieldingSentinel() {
        this->thread.yield_tick();
        this->flag = true;
    }

private:
    FirmwareThread& thread;  // NOLINT(*-avoid-const-or-ref-data-members): the test owns the thread.
    bool&           flag;    // NOLINT(*-avoid-const-or-ref-data-members): the test owns the flag.
};

TEST_CASE("FirmwareThread.UnwindingIsNotBlockedByAYieldingDestructor") {
    bool destructor_ran = false;

    FirmwareThread firmware([&](FirmwareThread& thread) {
        const YieldingSentinel sentinel(thread, destructor_ran);

        while (true) {
            thread.yield_tick();
        }
    });

    firmware.run_until_yield();
    firmware.finish();

    CHECK(destructor_ran);
}

TEST_CASE("FirmwareThread.ReportsAProgramThatReturns") {
    FirmwareThread firmware([](FirmwareThread&) { });

    firmware.run_until_yield();
    CHECK(firmware.has_finished());
    CHECK_NOTHROW(firmware.rethrow_any_error());
}

TEST_CASE("FirmwareThread.CarriesAProgramErrorBackToTheSimulation") {
    FirmwareThread firmware([](FirmwareThread&) { throw std::runtime_error("the firmware gave up"); });

    firmware.run_until_yield();
    CHECK(firmware.has_finished());
    CHECK_THROWS_AS(firmware.rethrow_any_error(), std::runtime_error);
}

TEST_CASE("FirmwareThread.RunningAFinishedProgramIsHarmless") {
    FirmwareThread firmware([](FirmwareThread&) { });

    firmware.run_until_yield();
    CHECK_NOTHROW(firmware.run_until_yield());
    CHECK_NOTHROW(firmware.finish());
}
}  // namespace
}  // namespace micras::sim
