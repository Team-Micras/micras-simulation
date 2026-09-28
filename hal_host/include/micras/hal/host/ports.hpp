/**
 * @file
 *
 * @brief What each host peripheral holds: the state the firmware and the outside world share.
 */

#ifndef MICRAS_HAL_HOST_PORTS_HPP
#define MICRAS_HAL_HOST_PORTS_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "micras/hal/host/spi_device.hpp"

namespace micras::hal::host {
/**
 * @brief What every port has: a name and who has used it.
 *
 * @note touched is set by the firmware side, bound by whatever connects the port
 *       to the outside world. A port the firmware touched that nothing is bound
 *       to is a wiring mistake, and is reported.
 */
struct Port {
    /**
     * @brief Label from the Cube layer, or a description of the handle.
     */
    std::string name;

    /**
     * @brief Whether the firmware used the port.
     */
    bool touched{false};

    /**
     * @brief Whether something outside the firmware is connected to it.
     */
    bool bound{false};
};

/**
 * @brief One GPIO pin.
 */
struct GpioPort : Port {
    /**
     * @brief Level the firmware drives, as its output data register holds it.
     */
    bool output{false};

    /**
     * @brief Level driven from outside, which is what a read returns when set.
     */
    std::optional<bool> input;

    /**
     * @brief Get the level the pin reads.
     *
     * @return The external level when one is driven, the output level otherwise.
     */
    bool read() const { return this->input.value_or(this->output); }
};

/**
 * @brief One PWM output channel.
 */
struct PwmPort : Port {
    /**
     * @brief Fraction of the period the output is active, in percent, polarity applied.
     */
    float duty_cycle{0.0F};

    /**
     * @brief Frequency the channel runs at, in hertz.
     */
    float frequency{0.0F};
};

/**
 * @brief One timer channel whose compare register a DMA stream feeds.
 */
struct PwmDmaPort : Port {
    /**
     * @brief Compare values of the last transfer the firmware started.
     */
    std::vector<uint32_t> compares;

    /**
     * @brief Counts in one period, the autoreload plus one.
     */
    uint32_t period{0};

    /**
     * @brief Whether a transfer is still going out.
     */
    bool busy{false};

    /**
     * @brief Host clock cycle the current transfer ends at, one timer period per value.
     */
    uint64_t transfer_end{0};
};

/**
 * @brief One ADC with its DMA stream.
 *
 * @note The firmware hands over the buffer the DMA writes; a device fills it
 *       with a whole sequence and then calls finish_sequence(), which is what
 *       the transfer complete interrupt does on the robot.
 */
struct AdcPort : Port {
    /**
     * @brief Buffer the firmware handed to the DMA, in one of its two widths.
     */
    ///@{
    std::span<uint16_t> buffer16;
    std::span<uint32_t> buffer32;
    ///@}

    /**
     * @brief Called at the end of a sequence, as the conversion complete interrupt would.
     */
    std::function<void()> complete;

    /**
     * @brief Get how many conversions the buffer holds.
     *
     * @return Size of the buffer, zero before the DMA was started.
     */
    std::size_t size() const { return this->buffer16.empty() ? this->buffer32.size() : this->buffer16.size(); }

    /**
     * @brief Write one conversion result.
     *
     * @param index Index in the buffer; ignored when out of range.
     * @param counts Result, in ADC counts.
     *
     * @note Const because the buffer belongs to the firmware and is only viewed through a span: the port itself
     *       does not change.
     */
    void write(std::size_t index, uint32_t counts) const;

    /**
     * @brief End the sequence, as the DMA interrupt does.
     */
    void finish_sequence() const;
};

/**
 * @brief One UART with its DMA streams.
 */
struct UartPort : Port {
    /**
     * @brief Circular buffer the firmware handed to the receive DMA.
     */
    std::span<uint8_t> rx_buffer;

    /**
     * @brief Index the next received byte goes to, as the DMA counter implies it.
     */
    std::size_t rx_head{0};

    /**
     * @brief Bytes the firmware handed to the transmit DMA and not yet sent.
     */
    std::deque<uint8_t> tx;

    /**
     * @brief Put one received byte where the DMA would.
     *
     * @param byte Byte on the wire.
     */
    void receive(uint8_t byte);

    /**
     * @brief Check whether a transmission is in progress.
     *
     * @return True while bytes are waiting to leave.
     */
    bool transmitting() const { return not this->tx.empty(); }
};

/**
 * @brief The part of the flash the firmware may use, as a byte array.
 */
struct FlashPort : Port {
    /**
     * @brief Contents, erased to 0xFF.
     */
    std::vector<uint8_t> bytes;
};

/**
 * @brief What the microcontroller core reports.
 */
struct McuPort : Port {
    /**
     * @brief Independent watchdog timeout the firmware set, zero before it set one.
     */
    uint32_t watchdog_timeout_ms{0};

    /**
     * @brief Host time of the last refresh, in timer cycles.
     */
    uint64_t last_refresh{0};

    /**
     * @brief Number of times the watchdog would have reset the chip.
     */
    uint32_t watchdog_expiries{0};

    /**
     * @brief Number of emergency stops the firmware performed.
     */
    uint32_t emergency_stops{0};
};

/**
 * @brief A quadrature count, as a timer in encoder mode holds it.
 */
struct EncoderPort : Port {
    /**
     * @brief Count, positive forward.
     */
    int32_t count{0};
};

/**
 * @brief Samples a swapped chip driver reads, such as an inertial measurement unit's.
 */
struct SamplePort : Port {
    /**
     * @brief Largest number of values one sample holds.
     */
    static constexpr std::size_t max_values{16};

    /**
     * @brief Latest sample.
     */
    std::array<float, max_values> values{};

    /**
     * @brief Incremented with every new sample.
     */
    uint32_t sequence{0};
};

/**
 * @brief One chip select of an SPI bus, and the device behind it.
 *
 * @note The firmware touches the port when it selects the device; a binding
 *       binds it by attaching a device with Board::spi_device().
 */
struct SpiPort : Port {
    /**
     * @brief Device attached to the chip select, none before a binding attaches one.
     */
    SpiDevice* device{nullptr};

    /**
     * @brief Whether the chip select is low, between the firmware's select and unselect.
     */
    bool selected{false};
};
}  // namespace micras::hal::host

#endif  // MICRAS_HAL_HOST_PORTS_HPP
