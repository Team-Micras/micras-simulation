/**
 * @file
 */

#ifndef MICRAS_PROXY_BATTERY_HPP
#define MICRAS_PROXY_BATTERY_HPP

#include <cstdint>

namespace micras::proxy {
/**
 * @brief Battery stub reporting a constant, fully charged pack.
 */
class Battery {
public:
    /**
     * @brief Configuration struct for the battery.
     */
    struct Config {
        float    max_voltage;
        uint16_t max_reading;
    };

    /**
     * @brief Construct a new Battery object.
     *
     * @param config Configuration for the battery.
     */
    explicit Battery(const Config& config);

    /**
     * @brief Update the battery reading.
     */
    void update();

    /**
     * @brief Get the battery voltage.
     *
     * @return Battery voltage in volts.
     */
    float get_voltage() const;

    /**
     * @brief Get the battery reading from the ADC.
     *
     * @return Raw battery reading.
     */
    float get_voltage_raw() const;

    /**
     * @brief Get the battery reading from the ADC.
     *
     * @return Battery reading from 0 to 1.
     */
    float get_adc_reading() const;

private:
    /**
     * @brief Constant battery voltage in volts.
     */
    float reading;

    /**
     * @brief Max battery voltage for the adc conversion.
     */
    float max_voltage;

    /**
     * @brief Max adc reading for the battery.
     */
    uint16_t max_reading;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BATTERY_HPP
