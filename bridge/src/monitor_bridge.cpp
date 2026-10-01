/**
 * @file
 */

#include <cstdint>
#include <mutex>
#include <span>
#include <string>
#include <vector>

#include "micras/sim/bridge/monitor_bridge.hpp"
#include "micras/sim/bridge/real_time_pacer.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/serial_bus.hpp"
#include "micras/sim/core/simulation.hpp"

namespace micras::sim {
MonitorBridge::MonitorBridge(SerialBus& serial, int port, std::string& error, IWallClock& wall) :
    serial{serial}, pacer{wall} {
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

void MonitorBridge::on_start(const Simulation& simulation) {
    this->pacer.start(simulation.context().clock.now_us());
}

RunControl MonitorBridge::on_before_tick(const Simulation& simulation) {
    if (this->open) {
        this->pacer.pace(simulation.context().clock.now_us());
    }

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
