/**
 * @file
 */

#ifndef MICRAS_PROXY_LED_HPP
#define MICRAS_PROXY_LED_HPP

#include <string>

#include "micras/sim/core/proxy_state.hpp"

namespace micras::sim {
struct SimulationContext;
}  // namespace micras::sim

namespace micras::proxy {
/**
 * @brief Stub LED that only records its last state.
 */
class Led {
public:
    /**
     * @brief Configuration struct for LED.
     */
    struct Config {
        std::string name;

        /**
         * @brief Simulation holding the state this LED is shown in.
         */
        sim::SimulationContext* context{nullptr};
    };

    /**
     * @brief Construct a new Led object.
     *
     * @param config Configuration for the LED.
     */
    explicit Led(const Config& config);

    /**
     * @brief Turn the LED on.
     */
    void turn_on();

    /**
     * @brief Turn the LED off.
     */
    void turn_off();

    /**
     * @brief Toggle the LED.
     */
    void toggle();

    /**
     * @brief Get the current state of the LED.
     *
     * @return True if the LED is on, false otherwise.
     */
    bool is_on() const;

private:
    /**
     * @brief Publish the current state to the boundary record.
     */
    void publish() const;

    /**
     * @brief Boundary record this proxy writes what it shows into.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    sim::ProxyState& state;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Current state of the LED.
     */
    bool state_on{false};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_LED_HPP
