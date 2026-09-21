/**
 * @file
 */

#ifndef MICRAS_PROXY_BUTTON_HPP
#define MICRAS_PROXY_BUTTON_HPP

#include <cstdint>
#include <string>

#include "micras/proxy/stopwatch.hpp"
#include "micras/sim/core/simulation_context.hpp"

namespace micras::sim {
struct SimulationContext;
}  // namespace micras::sim

namespace micras::proxy {
/**
 * @brief Button driven by the scenario through sim::ProxyState.
 */
class Button {
public:
    /**
     * @brief Enum for button status.
     */
    enum Status : uint8_t {
        NO_PRESS = 0,
        SHORT_PRESS = 1,
        LONG_PRESS = 2,
        EXTRA_LONG_PRESS = 3
    };

    /**
     * @brief Enum for button pull resistor.
     */
    enum PullResistor : uint8_t {
        PULL_UP = 0,
        PULL_DOWN = 1,
    };

    /**
     * @brief Configuration structure for button.
     */
    struct Config {
        std::string name;
        uint16_t    long_press_delay{500};
        uint16_t    extra_long_press_delay{2000};
        /**
         * @brief Simulation holding the state the button is read from.
         */
        sim::SimulationContext* context{nullptr};
    };

    /**
     * @brief Construct a new Button object.
     *
     * @param config Button configuration.
     */
    explicit Button(const Config& config);

    /**
     * @brief Check if button is pressed.
     *
     * @return True if button is pressed, false otherwise.
     */
    bool is_pressed() const;

    /**
     * @brief Get button status.
     *
     * @return Current button status.
     */
    Status get_status() const;

    /**
     * @brief Update the status of the button.
     *
     * @note A program that polls the button in a tight loop is waiting for a
     *       press only the simulation can deliver, so repeated updates at the
     *       same simulated time hand a tick over. The firmware calls this once
     *       per tick and therefore never reaches that point.
     */
    void update();

private:
    /**
     * @brief Number of updates at the same simulated time that count as polling.
     */
    static constexpr uint8_t max_repeated_updates{2};

    /**
     * @brief Simulation the state and the tick handoff come from.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    sim::SimulationContext& simulation;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Simulated time of the last update.
     *
     * @note Seeded from the clock so the first update is never mistaken for a
     *       repeat of an update that never happened.
     */
    uint64_t last_update_us{};

    /**
     * @brief Consecutive updates at the same simulated time.
     */
    uint8_t repeated_updates{0};

    /**
     * @brief Update button state from the simulation.
     */
    void update_state();

    /**
     * @brief Check if button was just pressed.
     *
     * @return True if button was just pressed, false otherwise.
     */
    bool is_rising_edge() const;

    /**
     * @brief Check if button was just released.
     *
     * @return True if button was just released, false otherwise.
     */
    bool is_falling_edge() const;

    /**
     * @brief Button pressing delays in ms.
     */
    ///@{
    uint16_t long_press_delay;
    uint16_t extra_long_press_delay;
    ///@}

    /**
     * @brief Stopwatch to determine type of button press.
     */
    Stopwatch status_stopwatch;

    /**
     * @brief Flag to know if button was being pressed.
     */
    bool previous_state{false};

    /**
     * @brief Flag to know if button is being pressed.
     */
    bool current_state{false};

    /**
     * @brief Current status of the button.
     */
    Status current_status{NO_PRESS};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BUTTON_HPP
