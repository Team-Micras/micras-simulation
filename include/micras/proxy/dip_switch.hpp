/**
 * @file
 */

#ifndef MICRAS_PROXY_DIP_SWITCH_HPP
#define MICRAS_PROXY_DIP_SWITCH_HPP

#include <array>
#include <cstdint>
#include <string>

#include "micras/sim/core/proxy_state.hpp"

namespace micras::sim {
struct SimulationContext;
}  // namespace micras::sim

namespace micras::proxy {
/**
 * @brief DIP switch driven by the scenario through sim::ProxyState.
 */
template <uint8_t num_of_switches>
class TDipSwitch {
public:
    /**
     * @brief Configuration struct for the Dip Switch.
     */
    struct Config {
        std::array<std::string, num_of_switches> names;
        /**
         * @brief Simulation holding the state the switches are read from.
         */
        sim::SimulationContext* context{nullptr};
    };

    /**
     * @brief Construct a new Dip Switch object.
     *
     * @param config Configuration struct for the DipSwitch.
     */
    explicit TDipSwitch(const Config& config);

    /**
     * @brief Get the state of a switch.
     *
     * @param switch_index Index of the switch.
     * @return True if the switch is on, false otherwise.
     */
    bool get_switch_state(uint8_t switch_index) const;

    /**
     * @brief Get the value of all switches.
     *
     * @return Value of all switches.
     */
    uint8_t get_switches_value() const;

private:
    /**
     * @brief State the scenario or the GUI writes the switches into.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    const sim::ProxyState& state;  // NOLINT(*-avoid-const-or-ref-data-members)
};
}  // namespace micras::proxy

#include "../src/proxy/dip_switch.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_DIP_SWITCH_HPP
