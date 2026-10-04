/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "micras/sim/core/clock.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/serial_bus.hpp"
#include "micras/sim/devices/serial_link.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Bit times per byte on an 8N1 line.
 */
constexpr double bits_per_byte{10.0};
}  // namespace

SerialLink::SerialLink(SerialBus& bus, Config config) : bus{bus}, config{std::move(config)} { }

void SerialLink::sample(MujocoWorld& /*world*/, const Clock& clock) {
    const double bytes_per_tick = this->config.baud_rate / bits_per_byte * clock.us_per_tick() * 1e-6;

    this->send_budget = std::min(this->send_budget + bytes_per_tick, 1.0 + bytes_per_tick);
    std::vector<uint8_t> sent;

    while (this->send_budget >= 1.0) {
        const std::optional<uint8_t> byte = this->config.take_sent();

        if (not byte.has_value()) {
            break;
        }

        sent.push_back(*byte);
        this->send_budget -= 1.0;
    }

    if (not sent.empty()) {
        this->bus.send_from_firmware(sent);
    }

    this->receive_budget = std::min(this->receive_budget + bytes_per_tick, 1.0 + bytes_per_tick);
    const auto count = static_cast<std::size_t>(std::floor(this->receive_budget));

    for (const uint8_t byte : this->bus.take_for_firmware(count)) {
        this->config.receive(byte);
        this->receive_budget -= 1.0;
    }
}
}  // namespace micras::sim
