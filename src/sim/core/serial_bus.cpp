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
    const std::vector<ISerialListener*> delivering = this->listeners;

    for (ISerialListener* listener : delivering) {
        listener->on_firmware_bytes(bytes);
    }
}

void SerialBus::queue_for_firmware(std::span<const uint8_t> packet) {
    if (this->to_firmware.size() >= max_pending_packets) {
        this->dropped++;
        return;
    }

    this->to_firmware.emplace_back(packet.begin(), packet.end());
}

std::vector<uint8_t> SerialBus::take_for_firmware() {
    if (this->to_firmware.empty()) {
        return {};
    }

    std::vector<uint8_t> packet = std::move(this->to_firmware.front());
    this->to_firmware.pop_front();
    return packet;
}

void SerialBus::clear() {
    this->to_firmware.clear();
    this->dropped = 0;
}
}  // namespace micras::sim
