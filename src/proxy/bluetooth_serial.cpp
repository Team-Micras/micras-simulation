/**
 * @file
 *
 * @brief Bluetooth serial proxy backed by the in-process byte queues of the world.
 */

#include "micras/proxy/bluetooth_serial.hpp"
#include "micras/sim/core/simulation_context.hpp"

namespace micras::proxy {
BluetoothSerial::BluetoothSerial(const Config& config) : serial{sim::require_context(config.context).serial} { }

void BluetoothSerial::update() { }

void BluetoothSerial::send_data(std::vector<uint8_t> data) {
    this->serial.send_from_firmware(data);
}

std::vector<uint8_t> BluetoothSerial::get_data() {
    return this->serial.take_for_firmware();
}
}  // namespace micras::proxy
