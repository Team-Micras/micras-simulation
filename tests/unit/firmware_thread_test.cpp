#include <atomic>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "micras/sim/core/firmware_thread.hpp"

namespace micras::sim {
namespace {
TEST(FirmwareThread, RunsTheProgramUpToEachYield) {
    std::vector<int> trace;
    FirmwareThread*  handle = nullptr;

    FirmwareThread firmware([&] {
        for (int step = 0; step < 3; step++) {
            trace.push_back(step);
            handle->yield_tick();
        }
    });
    handle = &firmware;

    firmware.run_until_yield();
    EXPECT_EQ(trace, (std::vector<int>{0}));

    firmware.run_until_yield();
    EXPECT_EQ(trace, (std::vector<int>{0, 1}));

    firmware.run_until_yield();
    EXPECT_EQ(trace, (std::vector<int>{0, 1, 2}));
}

TEST(FirmwareThread, NeverRunsBesideTheSimulation) {
    std::atomic<int> awake{0};
    std::atomic<int> overlaps{0};
    FirmwareThread*  handle = nullptr;

    FirmwareThread firmware([&] {
        while (true) {
            if (awake.fetch_add(1) != 0) {
                overlaps++;
            }

            awake--;
            handle->yield_tick();
        }
    });
    handle = &firmware;

    for (int tick = 0; tick < 200; tick++) {
        firmware.run_until_yield();

        if (awake.fetch_add(1) != 0) {
            overlaps++;
        }

        awake--;
    }

    EXPECT_EQ(overlaps, 0);
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

TEST(FirmwareThread, EndlessProgramsAreUnwoundByFinish) {
    bool            destructor_ran = false;
    FirmwareThread* handle = nullptr;

    {
        FirmwareThread firmware([&] {
            const UnwindSentinel sentinel(destructor_ran);

            while (true) {
                handle->yield_tick();
            }
        });
        handle = &firmware;

        firmware.run_until_yield();
        EXPECT_FALSE(firmware.has_finished());
        firmware.finish();
    }

    EXPECT_TRUE(destructor_ran);
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

TEST(FirmwareThread, UnwindingIsNotBlockedByAYieldingDestructor) {
    FirmwareThread* handle = nullptr;
    bool            destructor_ran = false;

    FirmwareThread firmware([&] {
        const YieldingSentinel sentinel(*handle, destructor_ran);

        while (true) {
            handle->yield_tick();
        }
    });
    handle = &firmware;

    firmware.run_until_yield();
    firmware.finish();

    EXPECT_TRUE(destructor_ran);
}

TEST(FirmwareThread, ReportsAProgramThatReturns) {
    FirmwareThread firmware([] {});

    firmware.run_until_yield();
    EXPECT_TRUE(firmware.has_finished());
    EXPECT_NO_THROW(firmware.rethrow_any_error());
}

TEST(FirmwareThread, CarriesAProgramErrorBackToTheSimulation) {
    FirmwareThread firmware([] { throw std::runtime_error("the firmware gave up"); });

    firmware.run_until_yield();
    EXPECT_TRUE(firmware.has_finished());
    EXPECT_THROW(firmware.rethrow_any_error(), std::runtime_error);
}

TEST(FirmwareThread, RunningAFinishedProgramIsHarmless) {
    FirmwareThread firmware([] {});

    firmware.run_until_yield();
    EXPECT_NO_THROW(firmware.run_until_yield());
    EXPECT_NO_THROW(firmware.finish());
}
}  // namespace
}  // namespace micras::sim
