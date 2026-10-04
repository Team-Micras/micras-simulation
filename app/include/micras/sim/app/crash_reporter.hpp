/**
 * @file
 *
 * @brief Names the tick a fatal signal interrupted.
 */

#ifndef MICRAS_SIM_APP_CRASH_REPORTER_HPP
#define MICRAS_SIM_APP_CRASH_REPORTER_HPP

#include "micras/sim/core/simulation.hpp"

namespace micras::sim {
/**
 * @brief Reports which tick was running when the process died.
 *
 * @note A firmware crash is the failure this harness exists to catch, and the
 *       CSV is flushed per row, so the tick number is all that is missing to
 *       replay it. The handler runs in signal context, so the tick is kept in a
 *       sig_atomic_t and rendered by hand.
 */
class CrashReporter : public IRunListener {
public:
    /**
     * @brief Get the single reporter the signal handler reads from.
     *
     * @return The reporter.
     */
    static CrashReporter& instance();

    /**
     * @brief Handle the fatal signals that a firmware fault raises.
     */
    static void install();

    /**
     * @brief Record the tick about to run.
     *
     * @param simulation Run about to advance.
     * @return Always RunControl::RUN; reporting never stops a run.
     */
    RunControl on_before_tick(const Simulation& simulation) override;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_APP_CRASH_REPORTER_HPP
