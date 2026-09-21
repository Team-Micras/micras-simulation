/**
 * @file
 */

#ifndef MICRAS_PROXY_WALL_SENSORS_HPP
#define MICRAS_PROXY_WALL_SENSORS_HPP

#include <array>
#include <cstdint>
#include <string>

#include "micras/core/butterworth_filter.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/proxy_state.hpp"

namespace micras::sim {
struct SimulationContext;
}  // namespace micras::sim

namespace micras::proxy {
/**
 * @brief Wall sensors backed by MuJoCo rangefinder sensors.
 */
template <uint8_t num_of_sensors>
class TWallSensors {
public:
    /**
     * @brief Configuration struct for wall sensors.
     *
     * @note No member carries a default: every field is required from
     *       target.hpp, so a missing one is a compile error rather than a
     *       silent zero.
     */
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init)
    struct Config {
        std::array<std::string, num_of_sensors> sensors;
        float                                   uncertainty;
        std::array<float, num_of_sensors>       base_readings;
        float                                   max_sensor_reading;
        float                                   min_sensor_reading;
        float                                   max_sensor_distance;
        float                                   filter_cutoff;
        float                                   sampling_frequency;
        /**
         * @brief Simulation holding the rangefinders these sensors read.
         */
        sim::SimulationContext* context{nullptr};
    };

    /**
     * @brief Construct a new WallSensors object.
     *
     * @param config Configuration for the wall sensors.
     */
    explicit TWallSensors(const Config& config);

    /**
     * @brief Turn on the wall sensors IR LED.
     */
    void turn_on();

    /**
     * @brief Turn off the wall sensors IR LED.
     */
    void turn_off();

    /**
     * @brief Update the wall sensors readings.
     */
    void update();

    /**
     * @brief Get the observation from a sensor.
     *
     * @param sensor_index Index of the sensor.
     * @param disturbed Whether or not there is another wall perpendicular to the one being measured.
     * @return True if the sensor detects a wall, false otherwise.
     */
    bool get_wall(uint8_t sensor_index, bool disturbed = false) const;

    /**
     * @brief Get the reading from a sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return Reading from the sensor.
     */
    float get_reading(uint8_t sensor_index) const;

    /**
     * @brief Get the ADC reading from a sensor.
     *
     * @note A MuJoCo rangefinder returns -1 when its ray hits nothing, which
     *       maps to no reflection at all, and a zero distance would make the
     *       intensity infinite, so it saturates instead.
     *
     * @param sensor_index Index of the sensor.
     * @return ADC reading from the sensor from 0 to 1.
     */
    float get_adc_reading(uint8_t sensor_index) const;

    /**
     * @brief Get the deviation of a wall sensor reading from its calibrated baseline.
     *
     * @param sensor_index Index of the sensor.
     * @return The reading error relative to the baseline; positive if above baseline.
     */
    float get_sensor_error(uint8_t sensor_index) const;

    /**
     * @brief Calibrate a wall sensor base reading.
     *
     * @param sensor_index Index of the sensor.
     */
    void calibrate_sensor(uint8_t sensor_index);

private:
    /**
     * @brief Publish an unfiltered reading and hand it back to the caller.
     *
     * @param sensor_index Index of the sensor.
     * @param reading Reading to publish.
     * @return The same reading.
     */
    float record(uint8_t sensor_index, float reading) const;

    /**
     * @brief Boundary record this proxy writes its unfiltered readings into.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    sim::ProxyState& state;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Simulation the readings come from.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    const sim::MujocoWorld& world;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Addresses of the rangefinder values inside mjData::sensordata.
     */
    std::array<int, num_of_sensors> sensor_addresses{};

    /**
     * @brief Maximum reading detectable by the sensor.
     */
    float max_sensor_reading;

    /**
     * @brief LED state.
     */
    bool leds_on{false};

    /**
     * @brief Butterworth filter for the ADC readings.
     */
    std::array<core::ButterworthFilter, num_of_sensors> filters;

    /**
     * @brief Measured wall values during calibration.
     */
    std::array<float, num_of_sensors> base_readings;

    /**
     * @brief Ratio of the base reading to still consider as seeing a wall.
     */
    float uncertainty;

    /**
     * @brief Constant value for the wall sensor distance to reading conversion.
     */
    float constant;
};
}  // namespace micras::proxy

#include "../src/proxy/wall_sensors.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_WALL_SENSORS_HPP
