/**
 * @file
 *
 * @brief In-process replacement for the bluetooth UART.
 */

#ifndef MICRAS_SIM_CORE_SERIAL_BUS_HPP
#define MICRAS_SIM_CORE_SERIAL_BUS_HPP

#include <cstdint>
#include <deque>
#include <span>
#include <vector>

namespace micras::sim {
/**
 * @brief Receives every byte the firmware writes.
 */
class ISerialListener {
public:
    ISerialListener() = default;

    ISerialListener(const ISerialListener&) = delete;
    ISerialListener(ISerialListener&&) = delete;
    ISerialListener& operator=(const ISerialListener&) = delete;
    ISerialListener& operator=(ISerialListener&&) = delete;

    virtual ~ISerialListener() = default;

    /**
     * @brief Handle bytes the firmware just wrote.
     *
     * @param bytes Bytes leaving the firmware, in stream order.
     */
    virtual void on_firmware_bytes(std::span<const uint8_t> bytes) = 0;
};

/**
 * @brief Byte channel between the firmware and everything listening to it.
 *
 * @note The firmware frames every complete packet it finds in one batch, so
 *       handing over one packet per call is not about what it can parse: it is
 *       what keeps a tick's input independent of how fast an asynchronous
 *       producer happens to be queueing. Output is fanned out to every listener
 *       as it is written, so no listener can consume bytes another one needed.
 */
class SerialBus {
public:
    /**
     * @brief Start delivering firmware output to a listener.
     *
     * @note Listeners must not be added or removed while bytes are being
     *       delivered. Adding the same listener twice is ignored.
     *
     * @param listener Listener to add; it must outlive the bus or remove itself.
     */
    void add_listener(ISerialListener& listener);

    /**
     * @brief Stop delivering firmware output to a listener.
     *
     * @param listener Listener to remove; unknown listeners are ignored.
     */
    void remove_listener(const ISerialListener& listener);

    /**
     * @brief Deliver bytes the firmware just wrote to every listener.
     *
     * @param bytes Bytes leaving the firmware.
     */
    void send_from_firmware(std::span<const uint8_t> bytes);

    /**
     * @brief Queue one complete packet for the firmware to read.
     *
     * @note The queue is bounded: a producer that outruns the one packet per
     *       tick the firmware is handed would otherwise grow it without limit
     *       and add unbounded latency. Packets past the bound are dropped and
     *       counted.
     *
     * @param packet Serialized packet, header to tail inclusive.
     */
    void queue_for_firmware(std::span<const uint8_t> packet);

    /**
     * @brief Take the next queued packet.
     *
     * @return The oldest queued packet, or an empty vector when none is waiting.
     */
    std::vector<uint8_t> take_for_firmware();

    /**
     * @brief Get how many packets are waiting for the firmware.
     *
     * @return Number of queued packets.
     */
    std::size_t pending_packets() const { return this->to_firmware.size(); }

    /**
     * @brief Get how many packets were dropped because the queue was full.
     *
     * @return Number of dropped packets, always 0 on a healthy run.
     */
    uint32_t dropped_packets() const { return this->dropped; }

    /**
     * @brief Largest number of packets allowed to wait for the firmware.
     */
    static constexpr std::size_t max_pending_packets{256};

    /**
     * @brief Drop every queued packet.
     */
    void clear();

private:
    /**
     * @brief Listeners receiving firmware output, in registration order.
     */
    std::vector<ISerialListener*> listeners;

    /**
     * @brief Packets queued by the host, one handed to the firmware per call.
     */
    std::deque<std::vector<uint8_t>> to_firmware;

    /**
     * @brief Packets dropped because the queue was full.
     */
    uint32_t dropped{0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_SERIAL_BUS_HPP
