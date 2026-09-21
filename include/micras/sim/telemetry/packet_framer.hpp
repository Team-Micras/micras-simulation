/**
 * @file
 *
 * @brief Splits the firmware byte stream into serialized packets.
 */

#ifndef MICRAS_SIM_TELEMETRY_PACKET_FRAMER_HPP
#define MICRAS_SIM_TELEMETRY_PACKET_FRAMER_HPP

#include <cstdint>
#include <span>
#include <vector>

namespace micras::sim {
/**
 * @brief Accumulates raw bytes and yields complete comm::Packet frames.
 *
 * @note Packet::serialize only escapes the payload: the five bytes after the
 *       header (message type, id, payload size) are written raw and may hold
 *       an escape or tail value, so the escape-aware walk starts at the payload.
 *       The checksum byte is never a control byte, so the first unescaped tail
 *       from the payload on ends the packet.
 */
class PacketFramer {
public:
    /**
     * @brief Largest amount of unframed bytes kept before the stream is considered desynchronised.
     *
     * @note A packet carries at most a 65535 byte payload, which escapes to
     *       131070 bytes, so the limit sits above the longest frame the
     *       firmware can emit and only a stream with no closing frame at all
     *       reaches it.
     */
    static constexpr std::size_t max_buffer_size{128UL * 1024UL + 16UL};

    /**
     * @brief Append bytes received from the firmware.
     *
     * @param bytes Raw bytes, in stream order.
     */
    void push(std::span<const uint8_t> bytes);

    /**
     * @brief Extract every complete packet accumulated so far.
     *
     * @return Serialized packets, header to tail inclusive, in stream order.
     */
    std::vector<std::vector<uint8_t>> drain();

    /**
     * @brief Get how many times the buffer overflowed and was truncated.
     *
     * @return Number of resynchronisations, always 0 on a healthy stream.
     */
    uint32_t resync_count() const { return this->resyncs; }

private:
    /**
     * @brief Find the end of the packet starting at a header position.
     *
     * @param start Index of the header byte.
     * @return Index of the tail byte, or the buffer size if the packet is incomplete.
     */
    std::size_t find_tail(std::size_t start) const;

    /**
     * @brief Drop the whole buffer when no frame closes within the limit.
     *
     * @note Once that much has arrived without a single closing frame, nothing
     *       buffered can still become a packet, and keeping part of it would
     *       only let the scan latch onto a payload byte and swallow the next
     *       real packet. Framing restarts clean from the following byte.
     */
    void resync_if_overflowed();

    /**
     * @brief Bytes received but not yet framed.
     */
    std::vector<uint8_t> buffer;

    /**
     * @brief Number of times the buffer overflowed and was truncated.
     */
    uint32_t resyncs{0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_TELEMETRY_PACKET_FRAMER_HPP
