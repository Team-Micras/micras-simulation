/**
 * @file
 *
 * @brief The battery, the suction fan and the motor current sensors.
 */

#ifndef MICRAS_SIM_DEVICES_POWER_HPP
#define MICRAS_SIM_DEVICES_POWER_HPP

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "micras/sim/core/noise.hpp"
#include "micras/sim/devices/device.hpp"
#include "micras/sim/robot/robot_description.hpp"

namespace micras::sim {
/**
 * @brief A battery measured by an ADC through a divider.
 *
 * @note The open circuit voltage of charged cells, which the loads do not pull
 *       down.
 */
class Battery : public Device {
public:
    /**
     * @brief The pack and its measurement.
     */
    struct Config {
        std::string                   name;
        BatteryDescription            description;
        double                        divider;
        double                        adc_reference;
        double                        adc_max_counts;
        double                        adc_noise_counts;
        std::function<void(uint32_t)> write;
    };

    /**
     * @brief Take the pack.
     *
     * @param config The pack.
     * @param noise The run's noise settings.
     */
    Battery(Config config, const NoiseConfig& noise);

    /**
     * @brief Convert the terminal voltage.
     *
     * @param world World that just advanced.
     * @param clock Clock of the run.
     */
    void sample(MujocoWorld& world, const Clock& clock) override;

    /**
     * @brief Get the terminal voltage.
     *
     * @return Voltage in volts.
     */
    double voltage() const { return this->terminal_voltage; }

    /**
     * @brief Get the recorded column: the terminal voltage.
     *
     * @return Column names.
     */
    std::vector<std::string> columns() const override;

    /**
     * @brief Append the terminal voltage.
     *
     * @param row Row being built.
     */
    void append(std::vector<CsvCell>& row) const override;

private:
    Config config;
    Noise  noise;
    double terminal_voltage;
};

/**
 * @brief A suction fan pressing the robot down.
 *
 * @note The downforce goes as the square of the fan speed, which goes as the duty
 *       cycle times the supply, and follows its target with the motor's time
 *       constant. It is applied through the robot model's fan actuator.
 */
class Fan : public Device {
public:
    /**
     * @brief The fan and what drives it.
     */
    struct Config {
        std::string             name;
        std::string             actuator;
        FanDescription          description;
        std::function<float()>  duty;
        std::function<bool()>   enabled;
        std::function<double()> supply_voltage;
    };

    /**
     * @brief Find the actuator.
     *
     * @param world Loaded world.
     * @param config The fan.
     */
    Fan(const MujocoWorld& world, Config config);

    /**
     * @brief Apply the downforce.
     *
     * @param world World about to be advanced.
     * @param clock Clock of the run.
     */
    void actuate(MujocoWorld& world, const Clock& clock) override;

    /**
     * @brief Get the recorded column: the downforce.
     *
     * @return Column names.
     */
    std::vector<std::string> columns() const override;

    /**
     * @brief Append the downforce.
     *
     * @param row Row being built.
     */
    void append(std::vector<CsvCell>& row) const override;

private:
    Config config;
    int    actuator_id;
    double downforce{0.0};
};

/**
 * @brief Current sense amplifiers read by an ADC.
 */
class CurrentSense : public Device {
public:
    /**
     * @brief The channels and their scaling.
     */
    struct Config {
        std::vector<std::function<double()>>       currents;
        double                                     volts_per_amp;
        double                                     adc_reference;
        double                                     adc_max_counts;
        double                                     adc_noise_counts;
        std::function<void(std::size_t, uint32_t)> write;
    };

    /**
     * @brief Take the channels.
     *
     * @param config The channels.
     * @param noise The run's noise settings.
     */
    CurrentSense(Config config, const NoiseConfig& noise);

    /**
     * @brief Convert every channel.
     *
     * @param world World that just advanced.
     * @param clock Clock of the run.
     */
    void sample(MujocoWorld& world, const Clock& clock) override;

private:
    Config config;
    Noise  noise;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_DEVICES_POWER_HPP
