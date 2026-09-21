/**
 * @file
 */

#include <array>
#include <csignal>
#include <cstdint>

#include <unistd.h>

#include "micras/sim/app/crash_reporter.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Tick the firmware is currently executing, read by the signal handler.
 */
// NOLINTNEXTLINE(*-avoid-non-const-global-variables): the only state a signal handler may read.
volatile std::sig_atomic_t running_tick = -1;

/**
 * @brief Name the tick a fatal signal interrupted and re-raise it.
 *
 * @note Only async-signal-safe calls are allowed here, so the tick is rendered
 *       by hand and written straight to the descriptor. Nothing useful is left
 *       to do if stderr itself fails, so the results are bound and discarded.
 *
 * @param signal_number Signal being handled.
 */
extern "C" void report_and_reraise(int signal_number) {
    constexpr std::array<char, 26> prefix{"firmware crashed at tick "};
    std::array<char, 24>           digits{};
    int64_t                        tick = running_tick;
    std::size_t                    length = 0;

    if (tick < 0) {
        digits.at(length++) = '?';
    } else {
        std::array<char, 24> reversed{};
        std::size_t          count = 0;

        do {  // NOLINT(cppcoreguidelines-avoid-do-while): at least one digit must be emitted.
            reversed.at(count++) = static_cast<char>('0' + (tick % 10));
            tick /= 10;
        } while (tick > 0);

        while (count > 0) {
            digits.at(length++) = reversed.at(--count);
        }
    }

    digits.at(length++) = '\n';

    const ssize_t prefix_written = write(STDERR_FILENO, prefix.data(), prefix.size() - 1);
    const ssize_t digits_written = write(STDERR_FILENO, digits.data(), length);
    static_cast<void>(prefix_written);
    static_cast<void>(digits_written);

    std::signal(signal_number, SIG_DFL);
    std::raise(signal_number);
}
}  // namespace

CrashReporter& CrashReporter::instance() {
    static CrashReporter reporter;
    return reporter;
}

void CrashReporter::install() {
    std::signal(SIGSEGV, report_and_reraise);
    std::signal(SIGABRT, report_and_reraise);
}

RunControl CrashReporter::on_before_tick(const Simulation& simulation) {
    running_tick = static_cast<std::sig_atomic_t>(simulation.tick());
    return RunControl::RUN;
}
}  // namespace micras::sim
