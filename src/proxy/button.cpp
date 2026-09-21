/**
 * @file
 *
 * @brief Button proxy reading its pressed state from the simulated world.
 */

#include "micras/proxy/button.hpp"
#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/simulation_context.hpp"

namespace micras::proxy {
Button::Button(const Config& config) :
    simulation{sim::require_context(config.context)},
    last_update_us{this->simulation.clock.now_us()},
    long_press_delay{config.long_press_delay},
    extra_long_press_delay{config.extra_long_press_delay},
    status_stopwatch{Stopwatch::Config{.context = config.context}} { }

bool Button::is_pressed() const {
    return this->current_state;
}

Button::Status Button::get_status() const {
    return this->current_status;
}

void Button::update() {
    const uint64_t now = this->simulation.clock.now_us();
    this->repeated_updates = now == this->last_update_us ? this->repeated_updates + 1 : 1;

    sim::FirmwareThread* firmware = this->simulation.firmware.load();

    if (this->repeated_updates >= max_repeated_updates and firmware != nullptr) {
        firmware->yield_tick();
        this->repeated_updates = 0;
    }

    this->last_update_us = this->simulation.clock.now_us();
    this->previous_state = this->current_state;
    this->update_state();

    if (this->is_rising_edge()) {
        this->status_stopwatch.reset_ms();
    } else if (this->is_falling_edge()) {
        if (this->status_stopwatch.elapsed_time_ms() > this->extra_long_press_delay) {
            this->current_status = EXTRA_LONG_PRESS;
            return;
        }

        if (this->status_stopwatch.elapsed_time_ms() > this->long_press_delay) {
            this->current_status = LONG_PRESS;
            return;
        }

        this->current_status = SHORT_PRESS;
        return;
    }

    this->current_status = NO_PRESS;
}

void Button::update_state() {
    this->current_state = this->simulation.proxy_state.interface_input.button_pressed;
}

bool Button::is_rising_edge() const {
    return this->current_state and not this->previous_state;
}

bool Button::is_falling_edge() const {
    return not this->current_state and this->previous_state;
}
}  // namespace micras::proxy
