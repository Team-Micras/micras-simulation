/**
 * @file
 */

#include <cstdint>
#include <mutex>
#include <span>
#include <string>
#include <vector>

#include "micras/sim/bridge/monitor_bridge.hpp"
#include "micras/sim/core/serial_bus.hpp"
#include "micras/sim/core/simulation.hpp"

namespace micras::sim {
MonitorBridge::MonitorBridge(SerialBus& serial, int port, std::string& error) : serial{serial} {
    this->server.set_on_binary([this](std::span<const uint8_t> bytes) {
        const std::scoped_lock lock(this->mutex);
        this->incoming.insert(this->incoming.end(), bytes.begin(), bytes.end());
    });

    if (this->server.start(port, error)) {
        this->open = true;
        this->serial.add_listener(*this);
    }
}

MonitorBridge::~MonitorBridge() {
    if (this->open) {
        this->serial.remove_listener(*this);
    }
}

void MonitorBridge::on_firmware_bytes(std::span<const uint8_t> bytes) {
    this->outgoing.insert(this->outgoing.end(), bytes.begin(), bytes.end());
}

RunControl MonitorBridge::on_before_tick(const Simulation& /*simulation*/) {
    std::vector<uint8_t> bytes;

    {
        const std::scoped_lock lock(this->mutex);
        bytes.swap(this->incoming);
    }

    if (not bytes.empty()) {
        this->received = true;
        this->serial.queue_for_firmware(bytes);
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
