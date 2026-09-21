/**
 * @file
 *
 * @brief Entry points a firmware hardware test is written against.
 */

#ifndef MICRAS_TEST_CORE_HPP
#define MICRAS_TEST_CORE_HPP

#include <concepts>

#include "micras/sim/core/simulation_context.hpp"
#include "target.hpp"

namespace micras {
template <typename F>
concept VoidFunction = requires(F void_function) {
    { void_function() } -> std::same_as<void>;
};

/**
 * @brief Core class to the tests, backed by the simulation instead of the MCU.
 *
 * @note The hardware tests are compiled unchanged: their main is renamed and
 *       run inside the firmware thread, so by the time init() is reached the
 *       model is loaded and the proxies can be built. There is nothing left for
 *       init() to do.
 *
 * @note The loop hands a tick over only when the iteration did not already do
 *       so, so a test that waits on a stopwatch keeps its own pace and a test
 *       that never looks at the clock still lets the world advance.
 */
class TestCore {
public:
    /**
     * @brief Delete the default constructor.
     */
    TestCore() = delete;

    /**
     * @brief Initialize the test core.
     *
     * @param argc Number of main arguments.
     * @param argv Main arguments.
     */
    static void init(int /*argc*/ = 0, char** /*argv*/ = nullptr) { }

    /**
     * @brief Loop the test core with a custom function.
     *
     * @param loop_func Custom loop function.
     */
    static void loop(VoidFunction auto loop_func) {
        const sim::Clock& clock = sim::SimulationContext::instance().clock;

        while (true) {
            const uint64_t started_at = clock.tick_count();

            loop_func();

            if (clock.tick_count() == started_at) {
                sim::yield_tick();
            }
        }
    }
};
}  // namespace micras

#endif  // MICRAS_TEST_CORE_HPP
