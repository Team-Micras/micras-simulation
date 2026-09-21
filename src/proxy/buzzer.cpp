/**
 * @file
 *
 * @brief Buzzer stub recording the last requested tone.
 */

#include "micras/proxy/buzzer.hpp"
#include "micras/sim/core/simulation_context.hpp"

namespace micras::proxy {
Buzzer::Buzzer(const Config& config) :
    state{sim::require_context(config.context).proxy_state}, stopwatch{Stopwatch::Config{.context = config.context}} {
    this->stop();
}

void Buzzer::publish() const {
    this->state.interface_output.buzzer_frequency = this->frequency;
}

void Buzzer::play(uint32_t frequency, uint32_t duration) {
    this->frequency = frequency;
    this->is_playing = true;
    this->duration = duration;
    this->publish();

    if (duration > 0) {
        this->stopwatch.reset_ms();
    }
}

void Buzzer::update() {
    if (this->is_playing and this->duration > 0 and this->stopwatch.elapsed_time_ms() > this->duration) {
        this->stop();
    }
}

void Buzzer::wait(uint32_t /*interval*/) {
    this->stop();
}

void Buzzer::stop() {
    this->frequency = 0;
    this->is_playing = false;
    this->publish();
}

uint32_t Buzzer::get_frequency() const {
    return this->frequency;
}
}  // namespace micras::proxy
