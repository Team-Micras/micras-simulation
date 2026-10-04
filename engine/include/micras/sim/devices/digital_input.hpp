/**
 * @file
 *
 * @brief A button or a switch, as the firmware's pin sees it.
 */

#ifndef MICRAS_SIM_DEVICES_DIGITAL_INPUT_HPP
#define MICRAS_SIM_DEVICES_DIGITAL_INPUT_HPP

#include <functional>
#include <string>
#include <vector>

#include "micras/sim/devices/device.hpp"

namespace micras::sim {
/**
 * @brief A two-position input a scenario or a human sets: a button, a switch.
 */
class DigitalInput : public Device {
public:
    /**
     * @brief The input and the pin it drives.
     */
    struct Config {
        std::string               name;
        bool                      active_low;
        std::function<void(bool)> drive;
    };

    /**
     * @brief Take the input, released.
     *
     * @param config The input.
     */
    explicit DigitalInput(Config config);

    /**
     * @brief Press or release it.
     *
     * @param active True for pressed, or on.
     */
    void set(bool active);

    /**
     * @brief Check whether it is pressed, or on.
     *
     * @return The state.
     */
    bool is_active() const { return this->active; }

    /**
     * @brief Get the name.
     *
     * @return The name.
     */
    const std::string& name() const { return this->config.name; }

    /**
     * @brief Get the recorded column: the state.
     *
     * @return Column names.
     */
    std::vector<std::string> columns() const override;

    /**
     * @brief Append the state.
     *
     * @param row Row being built.
     */
    void append(std::vector<CsvCell>& row) const override;

private:
    Config config;
    bool   active{false};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_DEVICES_DIGITAL_INPUT_HPP
