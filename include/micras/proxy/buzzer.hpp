/**
 * @file
 */

#ifndef MICRAS_PROXY_BUZZER_HPP
#define MICRAS_PROXY_BUZZER_HPP

#include <cstdint>
#include <string>

#include "micras/proxy/stopwatch.hpp"
#include "micras/sim/core/proxy_state.hpp"

namespace micras::sim {
struct SimulationContext;
}  // namespace micras::sim

namespace micras::proxy {
/**
 * @brief Stub buzzer that only records the last tone requested.
 */
class Buzzer {
public:
    /**
     * @brief Configuration struct for the buzzer.
     */
    struct Config {
        std::string name;

        /**
         * @brief Simulation holding the state this buzzer is heard in.
         */
        sim::SimulationContext* context{nullptr};
    };

    /**
     * @brief Construct a new Buzzer object.
     *
     * @param config Configuration for the buzzer.
     */
    explicit Buzzer(const Config& config);

    /**
     * @brief Play a tone for a duration.
     *
     * @param frequency Buzzer sound frequency in Hz.
     * @param duration Duration of the sound in ms.
     */
    void play(uint32_t frequency, uint32_t duration = 0);

    /**
     * @brief Update the buzzer state.
     */
    void update();

    /**
     * @brief Stop the buzzer sound.
     */
    void stop();

    /**
     * @brief Wait for a time interval.
     *
     * @note Blocking waits cannot advance simulated time, so this only stops the
     *       tone. It is never called by the firmware.
     *
     * @param interval Time to wait in ms.
     */
    void wait(uint32_t interval);

    /**
     * @brief Get the frequency currently being played.
     *
     * @return Frequency in Hz, zero when silent.
     */
    uint32_t get_frequency() const;

private:
    /**
     * @brief Publish the current tone to the boundary record.
     */
    void publish() const;

    /**
     * @brief Boundary record this proxy writes its tone into.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    sim::ProxyState& state;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Stopwatch to time the sound duration.
     */
    Stopwatch stopwatch;

    /**
     * @brief Frequency currently being played.
     */
    uint32_t frequency{};

    /**
     * @brief Flag to check if the buzzer is playing.
     */
    bool is_playing{};

    /**
     * @brief Duration of the sound in ms.
     */
    uint32_t duration{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BUZZER_HPP
