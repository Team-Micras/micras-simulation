/**
 * @file
 */

#include <cstdint>

#include <main.h>

#include "micras/hal/host/clock.hpp"
#include "micras/hal/timer.hpp"

namespace micras::hal {
uint32_t Timer::cycles_per_microsecond{1};

void Timer::init() {
    cycles_per_microsecond = SystemCoreClock / 1000000;
}

uint32_t Timer::get_counter() {
    return host::Clock::instance().read_cycles();
}

uint32_t Timer::get_counter_ms() {
    return host::Clock::instance().read_ms();
}

uint32_t Timer::to_microseconds(uint32_t cycles) {
    return cycles / cycles_per_microsecond;
}

uint32_t Timer::to_cycles(uint32_t microseconds) {
    return microseconds * cycles_per_microsecond;
}
}  // namespace micras::hal
