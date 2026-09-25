/**
 * @file
 *
 * @brief A serial line between the firmware's UART and the outside world.
 */

#ifndef MICRAS_SIM_DEVICES_SERIAL_LINK_HPP
#define MICRAS_SIM_DEVICES_SERIAL_LINK_HPP

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>

#include "micras/sim/core/serial_bus.hpp"
#include "micras/sim/devices/device.hpp"

namespace micras::sim {
/**
 * @brief Moves bytes both ways at the line's baud rate.
 *
 * @note Ten bit times per byte, 8N1. Each direction has its own budget, carried
 *       over between ticks, so the throughput is exactly the line's whatever the
 *       tick. The bus is the world side: its listeners hear what the firmware
 *       sends, and what is queued on it reaches the firmware.
 */
class SerialLink : public Device {
public:
    /**
     * @brief The line and the firmware's side of it.
     */
    struct Config {
        /**
         * @brief Bits per second.
         */
        uint32_t baud_rate;

        /**
         * @brief Takes the next byte the firmware is sending, if any.
         */
        std::function<std::optional<uint8_t>()> take_sent;

        /**
         * @brief Puts a byte where the firmware receives it.
         */
        std::function<void(uint8_t)> receive;
    };

    /**
     * @brief Connect the line to the bus.
     *
     * @param bus The world side.
     * @param config The line.
     */
    SerialLink(SerialBus& bus, Config config);

    /**
     * @brief Move this tick's bytes.
     *
     * @param world World that just advanced.
     * @param clock Clock of the run.
     */
    void sample(MujocoWorld& world, const Clock& clock) override;

private:
    SerialBus& bus;  // NOLINT(*-avoid-const-or-ref-data-members): the run's bus outlives its devices.
    Config     config;
    double     send_budget{0.0};
    double     receive_budget{0.0};
};

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

#endif  // MICRAS_SIM_DEVICES_SERIAL_LINK_HPP
