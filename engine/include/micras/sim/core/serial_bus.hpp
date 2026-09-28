/**
 * @file
 *
 * @brief The byte stream between a firmware's radio link and everything outside.
 */

#ifndef MICRAS_SIM_CORE_SERIAL_BUS_HPP
#define MICRAS_SIM_CORE_SERIAL_BUS_HPP

#include <cstddef>
#include <cstdint>
#include <deque>
#include <span>
#include <vector>

namespace micras::sim {
/**
 * @brief Receives every byte the firmware sends.
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
     * @brief Handle bytes the firmware just sent.
     *
     * @param bytes Bytes leaving the firmware, in stream order.
     */
    virtual void on_firmware_bytes(std::span<const uint8_t> bytes) = 0;
};

/**
 * @brief Byte stream between the firmware and everything listening to it.
 *
 * @note Bytes only, no framing: the protocol is the firmware's business. The
 *       link device moves bytes between this bus and the firmware's UART at the
 *       line's baud rate, so how fast a producer queues them never changes when
 *       they arrive. Output is fanned out to every listener as it leaves, so no
 *       listener can consume bytes another one needed.
 */
class SerialBus {
public:
    /**
     * @brief Largest number of bytes allowed to wait for the firmware.
     */
    static constexpr std::size_t max_pending_bytes{65536};

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
     * @brief Deliver bytes the firmware just sent to every listener.
     *
     * @param bytes Bytes leaving the firmware.
     */
    void send_from_firmware(std::span<const uint8_t> bytes);

    /**
     * @brief Queue bytes for the firmware to receive.
     *
     * @note Bytes past the bound are dropped and counted: a producer that
     *       outruns the line would otherwise add latency without limit.
     *
     * @param bytes Bytes to send.
     */
    void queue_for_firmware(std::span<const uint8_t> bytes);

    /**
     * @brief Take the next bytes for the firmware.
     *
     * @param count Most bytes to take.
     * @return The oldest queued bytes, at most count of them.
     */
    std::vector<uint8_t> take_for_firmware(std::size_t count);

    /**
     * @brief Get how many bytes were dropped because the queue was full.
     *
     * @return Number of dropped bytes, always 0 on a healthy run.
     */
    uint64_t dropped_bytes() const { return this->dropped; }

private:
    /**
     * @brief Listeners receiving firmware output, in registration order.
     */
    std::vector<ISerialListener*> listeners;

    /**
     * @brief Bytes waiting for the firmware.
     */
    std::deque<uint8_t> to_firmware;

    /**
     * @brief Bytes dropped because the queue was full.
     */
    uint64_t dropped{0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_SERIAL_BUS_HPP
