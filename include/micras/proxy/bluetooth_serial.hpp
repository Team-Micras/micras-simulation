/**
 * @file
 */

#ifndef MICRAS_PROXY_BLUETOOTH_SERIAL_HPP
#define MICRAS_PROXY_BLUETOOTH_SERIAL_HPP

#include <cstdint>
#include <vector>

#include "micras/sim/core/serial_bus.hpp"

namespace micras::sim {
struct SimulationContext;
}  // namespace micras::sim

namespace micras::proxy {
/**
 * @brief In-process replacement for the bluetooth serial link.
 *
 * @note Bytes are exchanged through sim::ProxyState so the harness can decode
 *       the firmware telemetry and inject command packets without any socket.
 */
class BluetoothSerial {
public:
    /**
     * @brief Configuration struct for the BluetoothSerial.
     */
    struct Config {
        /**
         * @brief Simulation holding the serial bus this radio writes to.
         */
        sim::SimulationContext* context{nullptr};
    };

    /**
     * @brief Construct a new BluetoothSerial object.
     *
     * @param config Configuration for the BluetoothSerial.
     */
    explicit BluetoothSerial(const Config& config);

    /**
     * @brief Process the serial channel. Nothing is buffered, so this is a no-op.
     */
    void update();

    /**
     * @brief Send data to the harness.
     *
     * @param data Bytes to be sent.
     */
    void send_data(std::vector<uint8_t> data);

    /**
     * @brief Get the data queued by the harness since the last call.
     *
     * @return Vector containing received bytes, empty if there is no new data.
     */
    std::vector<uint8_t> get_data();

private:
    /**
     * @brief Byte channel shared with the harness.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    sim::SerialBus& serial;  // NOLINT(*-avoid-const-or-ref-data-members)
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BLUETOOTH_SERIAL_HPP
