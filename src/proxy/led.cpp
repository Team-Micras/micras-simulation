/**
 * @file
 *
 * @brief Led stub recording the last state written.
 */

#include "micras/proxy/led.hpp"
#include "micras/sim/core/simulation_context.hpp"

namespace micras::proxy {
Led::Led(const Config& config) : state{sim::require_context(config.context).proxy_state} {
    this->publish();
}

void Led::publish() const {
    this->state.interface_output.led_on = this->state_on;
}

void Led::turn_on() {
    this->state_on = true;
    this->publish();
}

void Led::turn_off() {
    this->state_on = false;
    this->publish();
}

void Led::toggle() {
    this->state_on = not this->state_on;
    this->publish();
}

bool Led::is_on() const {
    return this->state_on;
}
}  // namespace micras::proxy
