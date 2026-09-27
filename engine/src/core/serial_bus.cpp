/**
 * @file
 */

#include <algorithm>

#include "micras/sim/core/serial_bus.hpp"

namespace micras::sim {
void SerialBus::add_listener(ISerialListener& listener) {
    if (std::ranges::find(this->listeners, &listener) == this->listeners.end()) {
        this->listeners.push_back(&listener);
    }
}

void SerialBus::remove_listener(const ISerialListener& listener) {
    std::erase(this->listeners, &listener);
}

void SerialBus::send_from_firmware(std::span<const uint8_t> bytes) {
    for (ISerialListener* listener : this->listeners) {
        listener->on_firmware_bytes(bytes);
    }
}

void SerialBus::queue_for_firmware(std::span<const uint8_t> bytes) {
    for (const uint8_t byte : bytes) {
        if (this->to_firmware.size() >= max_pending_bytes) {
            this->dropped++;
            continue;
        }

        this->to_firmware.push_back(byte);
    }
}

std::vector<uint8_t> SerialBus::take_for_firmware(std::size_t count) {
    const std::size_t    taken = std::min(count, this->to_firmware.size());
    std::vector<uint8_t> bytes(
        this->to_firmware.begin(), this->to_firmware.begin() + static_cast<std::ptrdiff_t>(taken)
    );
    this->to_firmware.erase(this->to_firmware.begin(), this->to_firmware.begin() + static_cast<std::ptrdiff_t>(taken));
    return bytes;
}
}  // namespace micras::sim
