/**
 * @file
 *
 * @brief A chip on an SPI bus, as the host backend sees it: a chip select and a stream of bytes.
 */

#ifndef MICRAS_HAL_HOST_SPI_DEVICE_HPP
#define MICRAS_HAL_HOST_SPI_DEVICE_HPP

#include <cstdint>
#include <span>

namespace micras::hal::host {
/**
 * @brief A device behind one chip select of an SPI bus.
 *
 * @note The host Spi calls select() when the firmware pulls the device's chip
 *       select low, exchange() for every transfer while it is low, however many
 *       HAL calls the firmware splits the transaction into, and deselect() when
 *       the chip select goes high again. A transaction is therefore everything
 *       between select() and deselect(), which is what a chip sees on the bus.
 *
 * @note A device only answers in the SPI mode it expects. When the mode the bus
 *       runs in differs, the host Spi does not call exchange() and the firmware
 *       reads all ones, as a chip clocked on the wrong edge answers on the robot.
 */
class SpiDevice {
public:
    /**
     * @brief SPI mode: the clock polarity in the high bit, the clock phase in the low one.
     */
    enum class Mode : uint8_t {
        MODE_0 = 0,
        MODE_1 = 1,
        MODE_2 = 2,
        MODE_3 = 3,
    };

    /**
     * @brief Special member functions: a device is attached by reference, so it cannot be copied or moved.
     */
    ///@{
    virtual ~SpiDevice() = default;
    SpiDevice(const SpiDevice&) = delete;
    SpiDevice(SpiDevice&&) = delete;
    SpiDevice& operator=(const SpiDevice&) = delete;
    SpiDevice& operator=(SpiDevice&&) = delete;

    ///@}

    /**
     * @brief Get the SPI mode the device answers in.
     *
     * @return The mode.
     */
    Mode mode() const { return this->spi_mode; }

    /**
     * @brief Start a transaction: the chip select went low.
     */
    virtual void select() = 0;

    /**
     * @brief Clock bytes through the device while it is selected.
     *
     * @param transmitted Bytes the controller sends, one per received byte.
     * @param received Bytes the device answers with, as many as were transmitted.
     */
    virtual void exchange(std::span<const uint8_t> transmitted, std::span<uint8_t> received) = 0;

    /**
     * @brief End the transaction: the chip select went high.
     */
    virtual void deselect() = 0;

protected:
    /**
     * @brief Construct a device that answers in one SPI mode.
     *
     * @param mode The mode.
     */
    explicit SpiDevice(Mode mode) : spi_mode{mode} { }

private:
    /**
     * @brief SPI mode the device answers in.
     */
    Mode spi_mode;
};
}  // namespace micras::hal::host

#endif  // MICRAS_HAL_HOST_SPI_DEVICE_HPP
