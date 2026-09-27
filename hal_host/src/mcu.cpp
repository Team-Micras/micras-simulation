/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>

#include <unistd.h>

#include "micras/hal/host/board.hpp"
#include "micras/hal/host/clock.hpp"
#include "micras/hal/mcu.hpp"
#include "micras/hal/timer.hpp"

namespace micras::hal {
void Mcu::init(const Config& config) {
    config.clock_init();

    if (config.peripheral_clock_init != nullptr) {
        config.peripheral_clock_init();
    }

    Timer::init();

    for (const InitFunction init_function : config.peripheral_inits) {
        init_function();
    }

    host::Board::mcu().touched = true;
}

void Mcu::emergency_stop(std::span<const Pwm::Config> pwm_outputs, std::span<const Gpio::Config> enable_gpios) {
    for (const auto& pwm_output : pwm_outputs) {
        host::Board::pwm(pwm_output.handle, pwm_output.timer_channel).duty_cycle = 0.0F;
    }

    for (const auto& enable_gpio : enable_gpios) {
        host::Board::gpio(enable_gpio.port, enable_gpio.pin).output = false;
    }

    host::McuPort& mcu = host::Board::mcu();
    mcu.emergency_stops++;

    const host::Clock&   clock = host::Clock::instance();
    std::array<char, 96> message{};
    const auto           result = std::format_to_n(
        message.data(), static_cast<std::ptrdiff_t>(message.size()), "firmware emergency stop at step {} ({:.6f} s)\n",
        clock.steps(), static_cast<double>(clock.now()) / (1e6 * clock.cycles_per_microsecond())
    );
    const auto    length = std::min(static_cast<std::size_t>(result.size), message.size());
    const ssize_t written = ::write(STDERR_FILENO, message.data(), length);
    static_cast<void>(written);
}

void Mcu::set_watchdog_timeout(uint32_t timeout_ms) {
    host::McuPort& mcu = host::Board::mcu();
    mcu.touched = true;
    mcu.watchdog_timeout_ms = timeout_ms;
    mcu.last_refresh = host::Clock::instance().now();
}

void Mcu::refresh_watchdog() {
    host::McuPort&     mcu = host::Board::mcu();
    const host::Clock& clock = host::Clock::instance();
    const uint64_t     timeout = static_cast<uint64_t>(mcu.watchdog_timeout_ms) * 1000 * clock.cycles_per_microsecond();

    if (mcu.watchdog_timeout_ms != 0 and clock.now() - mcu.last_refresh > timeout) {
        mcu.watchdog_expiries++;
    }

    mcu.last_refresh = clock.now();
}
}  // namespace micras::hal
