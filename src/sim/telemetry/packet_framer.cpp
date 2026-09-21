/**
 * @file
 */

#include "micras/comm/packet.hpp"
#include "micras/sim/telemetry/packet_framer.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Offset of the first payload byte from the header byte.
 */
constexpr std::size_t payload_offset{6};
}  // namespace

void PacketFramer::push(std::span<const uint8_t> bytes) {
    this->buffer.insert(this->buffer.end(), bytes.begin(), bytes.end());
}

std::vector<std::vector<uint8_t>> PacketFramer::drain() {
    std::vector<std::vector<uint8_t>> packets;
    std::size_t                       consumed = 0;
    std::size_t                       index = 0;

    while (index < this->buffer.size()) {
        if (this->buffer[index] != comm::Packet::header_byte) {
            index++;
            continue;
        }

        if (index + payload_offset > this->buffer.size()) {
            break;
        }

        const std::size_t tail = this->find_tail(index);

        if (tail >= this->buffer.size()) {
            break;
        }

        const auto first = this->buffer.begin() + static_cast<std::ptrdiff_t>(index);
        const auto last = this->buffer.begin() + static_cast<std::ptrdiff_t>(tail) + 1;
        packets.emplace_back(first, last);
        index = tail + 1;
        consumed = index;
    }

    this->buffer.erase(this->buffer.begin(), this->buffer.begin() + static_cast<std::ptrdiff_t>(consumed));
    this->resync_if_overflowed();

    return packets;
}

std::size_t PacketFramer::find_tail(std::size_t start) const {
    std::size_t index = start + payload_offset;

    while (index < this->buffer.size()) {
        if (this->buffer[index] == comm::Packet::escape_byte) {
            index += 2;
            continue;
        }

        if (this->buffer[index] == comm::Packet::tail_byte) {
            return index;
        }

        index++;
    }

    return this->buffer.size();
}

void PacketFramer::resync_if_overflowed() {
    if (this->buffer.size() <= max_buffer_size) {
        return;
    }

    this->buffer.clear();
    this->resyncs++;
}
}  // namespace micras::sim
