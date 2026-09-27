/**
 * @file
 */

#include <utility>

#include "micras/hal/host/clock.hpp"

namespace micras::hal::host {
Clock& Clock::instance() {
    static Clock clock;
    return clock;
}

void Clock::configure(uint32_t cycles_per_microsecond) {
    this->cycles_per_us = cycles_per_microsecond;
    this->quantum = cycles_per_microsecond;
}

void Clock::set_handover(uint32_t step_us, Handover handover) {
    this->step = static_cast<uint64_t>(step_us) * this->cycles_per_us;
    this->next_boundary = (this->cycles / this->step + 1) * this->step;
    this->handover = std::move(handover);
}

void Clock::clear_handover() {
    this->step = 0;
    this->handover = nullptr;
}

void Clock::charge() {
    this->cycles += this->quantum;

    while (this->step != 0 and this->cycles >= this->next_boundary) {
        this->next_boundary += this->step;
        this->handed_over++;

        if (this->handover) {
            this->handover();
        }
    }
}

uint32_t Clock::read_cycles() {
    this->charge();
    return static_cast<uint32_t>(this->cycles);
}

uint32_t Clock::read_ms() {
    this->charge();
    return static_cast<uint32_t>(this->cycles / (1000ULL * this->cycles_per_us));
}

void Clock::reset() {
    *this = Clock{};
}
}  // namespace micras::hal::host
