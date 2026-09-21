/**
 * @file
 */

#include "micras/proxy/stopwatch.hpp"
#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/simulation_context.hpp"
#include "target.hpp"

namespace micras::proxy {
Stopwatch::Stopwatch() : Stopwatch(stopwatch_config) { }

Stopwatch::Stopwatch(const Config& config) :
    simulation{sim::require_context(config.context)},
    counter{this->simulation.clock.now_us() - this->simulation.clock.us_per_tick()},
    last_read_us{this->simulation.clock.now_us()} { }

void Stopwatch::reset_ms() {
    this->reset_us();
}

void Stopwatch::reset_us() {
    this->counter = this->simulation.clock.now_us();
}

uint32_t Stopwatch::elapsed_time_ms() const {
    const uint64_t before = this->simulation.clock.now_us();
    this->repeated_ms_reads = before == this->last_read_us ? this->repeated_ms_reads + 1 : 1;

    if (this->repeated_ms_reads >= max_repeated_ms_reads) {
        this->yield_tick();
        this->note_yield();
    }

    this->last_read_us = this->simulation.clock.now_us();
    return static_cast<uint32_t>((this->last_read_us - this->counter) / 1000);
}

uint32_t Stopwatch::elapsed_time_us() const {
    const uint64_t before = this->simulation.clock.now_us();
    this->repeated_reads = before == this->last_read_us ? this->repeated_reads + 1 : 1;

    if (this->repeated_reads >= max_repeated_reads) {
        this->yield_tick();
        this->note_yield();
    }

    this->last_read_us = this->simulation.clock.now_us();
    return static_cast<uint32_t>(this->last_read_us - this->counter);
}

void Stopwatch::sleep_ms(uint32_t time) {
    Stopwatch(stopwatch_config).sleep_us(time * 1000);
}

void Stopwatch::sleep_us(uint32_t time) const {
    const uint64_t target = this->simulation.clock.now_us() + time;

    while (this->simulation.clock.now_us() < target and this->yield_tick()) { }
}

bool Stopwatch::yield_tick() const {
    sim::FirmwareThread* firmware = this->simulation.firmware.load();
    return firmware != nullptr and firmware->yield_tick();
}

void Stopwatch::note_yield() const {
    this->repeated_reads = 0;
    this->repeated_ms_reads = 0;
}
}  // namespace micras::proxy
