/**
 * @file
 */

#include "micras/sim/bridge/monitor_bridge.hpp"

namespace micras::sim {
MonitorBridge::MonitorBridge(SerialBus& serial, int port, std::string& error) : serial{serial} {
    this->server.set_on_binary([this](std::span<const uint8_t> bytes) {
        const std::lock_guard lock(this->mutex);
        this->incoming.push(bytes);
    });

    if (this->server.start(port, error)) {
        this->open = true;
        this->serial.add_listener(*this);
    }
}

MonitorBridge::~MonitorBridge() {
    this->server.stop();

    if (this->open) {
        this->serial.remove_listener(*this);
    }
}

void MonitorBridge::on_firmware_bytes(std::span<const uint8_t> bytes) {
    this->outgoing.insert(this->outgoing.end(), bytes.begin(), bytes.end());
}

RunControl MonitorBridge::on_before_tick(const Simulation& /*simulation*/) {
    std::vector<std::vector<uint8_t>> packets;

    {
        const std::lock_guard lock(this->mutex);
        packets = this->incoming.drain();
    }

    for (const auto& packet : packets) {
        this->serial.queue_for_firmware(packet);
    }

    return RunControl::RUN;
}

void MonitorBridge::on_after_tick(const Simulation& /*simulation*/) {
    if (this->outgoing.empty()) {
        return;
    }

    this->server.broadcast(this->outgoing);
    this->outgoing.clear();
}
}  // namespace micras::sim
