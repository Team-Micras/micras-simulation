/**
 * @file
 *
 * @brief Addressable RGB stub recording the last colour written to each led.
 */

#ifndef MICRAS_PROXY_ARGB_CPP
#define MICRAS_PROXY_ARGB_CPP

#include "micras/proxy/argb.hpp"  // NOLINT(misc-header-include-cycle)
#include "micras/sim/core/simulation_context.hpp"

namespace micras::proxy {
template <uint8_t num_of_leds>
TArgb<num_of_leds>::TArgb(const Config& config) : state{sim::require_context(config.context).proxy_state} {
    this->turn_off();
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::publish() const {
    static_assert(
        num_of_leds <= sim::InterfaceOutput::argb_count,
        "the board has more addressable LEDs than the simulated interface records"
    );

    for (uint8_t i = 0; i < num_of_leds; i++) {
        const Color& colour = this->colors.at(i);
        this->state.interface_output.argb.at(i) = {colour.red, colour.green, colour.blue};
    }
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::set_color(const Color& color, uint8_t index) {
    if (index >= num_of_leds) {
        return;
    }

    this->colors.at(index) = color;
    this->publish();
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::set_color(const Color& color) {
    this->colors.fill(color);
    this->publish();
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::set_colors(const std::array<Color, num_of_leds>& colors) {
    this->colors = colors;
    this->publish();
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::turn_off(uint8_t index) {
    this->set_color({0, 0, 0}, index);
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::turn_off() {
    this->set_color({0, 0, 0});
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::update() { }

template <uint8_t num_of_leds>
typename TArgb<num_of_leds>::Color TArgb<num_of_leds>::get_color(uint8_t index) const {
    return this->colors.at(index);
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_ARGB_CPP
