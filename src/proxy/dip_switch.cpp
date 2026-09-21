/**
 * @file
 *
 * @brief DIP switch proxy reading its state from the simulated world.
 */

#ifndef MICRAS_PROXY_DIP_SWITCH_CPP
#define MICRAS_PROXY_DIP_SWITCH_CPP

#include "micras/proxy/dip_switch.hpp"  // NOLINT(misc-header-include-cycle)
#include "micras/sim/core/simulation_context.hpp"

namespace micras::proxy {
template <uint8_t num_of_switches>
TDipSwitch<num_of_switches>::TDipSwitch(const Config& config) :
    state{sim::require_context(config.context).proxy_state} { }

template <uint8_t num_of_switches>
bool TDipSwitch<num_of_switches>::get_switch_state(uint8_t switch_index) const {
    if (switch_index >= num_of_switches) {
        return false;
    }

    static_assert(
        num_of_switches <= sim::InterfaceInput::dip_switch_count,
        "the board has more DIP switches than the simulated interface records"
    );

    return this->state.interface_input.dip_switches.at(switch_index);
}

template <uint8_t num_of_switches>
uint8_t TDipSwitch<num_of_switches>::get_switches_value() const {
    uint8_t switches_value = 0;

    for (uint8_t i = 0; i < num_of_switches; i++) {
        switches_value |= static_cast<uint8_t>(this->get_switch_state(i)) << i;
    }

    return switches_value;
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_DIP_SWITCH_CPP
