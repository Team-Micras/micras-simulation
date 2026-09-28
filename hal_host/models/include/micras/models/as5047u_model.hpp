/**
 * @file
 *
 * @brief The AS5047U magnetic position sensor, as its SPI interface shows it.
 */

#ifndef MICRAS_MODELS_AS5047U_MODEL_HPP
#define MICRAS_MODELS_AS5047U_MODEL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "micras/hal/host/spi_device.hpp"

namespace micras::models {
/**
 * @brief The registers of an AS5047U behind an SPI chip select, in its 24-bit frame mode.
 *
 * @details Every frame is three bytes. A command frame carries a don't care
 *          bit, the read flag, a 14-bit address and a CRC; a data frame carries
 *          the warning and error flags, 14 bits of data and a CRC. The CRC is a
 *          CRC-8 of the first two bytes, polynomial 0x1D, initial value 0xC4,
 *          final XOR 0xFF. Answers are pipelined by one frame: what the sensor
 *          sends during a frame answers the frame before it, whatever the chip
 *          select did in between.
 *
 * @details A read command is answered with the register's content. A write
 *          command is answered with the register's old content and is followed
 *          by a data frame holding the new one, which is answered with the new
 *          content.
 *
 * @note Every frame's CRC is checked. A frame with a wrong CRC is ignored, and
 *       a write whose data frame it was is dropped; it sets the CRC error bit of
 *       ERRFL. A chip select raised in the middle of a frame drops it and sets
 *       the framing error bit. The error flag of every answer is set while
 *       ERRFL is not zero, and reading ERRFL clears it.
 *
 * @note DISABLE, ZPOSM, ZPOSL, SETTINGS1 to SETTINGS3 and ECC are held, zero
 *       after power-on, as volatile registers. Writes to any other address are
 *       ignored and every other register reads zero: the angle, the magnitude
 *       and the diagnostics are not modeled. So is the position: on the robot it
 *       reaches the firmware through the quadrature output and a timer in
 *       encoder mode, and so it does in a simulation.
 */
class As5047uModel : public hal::host::SpiDevice {
public:
    /**
     * @brief Bytes in a frame.
     */
    static constexpr std::size_t frame_size{3};

    /**
     * @brief Addresses of the registers the model holds.
     */
    ///@{
    static constexpr uint16_t nop_address{0x0000};
    static constexpr uint16_t errfl_address{0x0001};
    static constexpr uint16_t disable_address{0x0015};
    static constexpr uint16_t zposm_address{0x0016};
    static constexpr uint16_t zposl_address{0x0017};
    static constexpr uint16_t settings1_address{0x0018};
    static constexpr uint16_t settings2_address{0x0019};
    static constexpr uint16_t settings3_address{0x001A};
    static constexpr uint16_t ecc_address{0x001B};
    ///@}

    /**
     * @brief Bits of ERRFL the model sets.
     */
    ///@{
    static constexpr uint16_t framing_error{1U << 4U};
    static constexpr uint16_t crc_error{1U << 6U};
    ///@}

    /**
     * @brief Power the sensor on, in SPI mode 1.
     */
    As5047uModel();

    /**
     * @brief Start a transaction at the start of a frame.
     */
    void select() override;

    /**
     * @brief Shift the pending answer out and the frame in, acting on every frame completed.
     *
     * @param transmitted Bytes the controller sends.
     * @param received Bytes of the answer to the previous frame.
     */
    void exchange(std::span<const uint8_t> transmitted, std::span<uint8_t> received) override;

    /**
     * @brief End the transaction, dropping an incomplete frame.
     */
    void deselect() override;

    /**
     * @brief Get a register without the side effects of reading it over SPI.
     *
     * @param address Register address.
     * @return The register's content, zero for one not modeled.
     */
    uint16_t peek(uint16_t address) const;

    /**
     * @brief Compute the CRC of a frame.
     *
     * @param high First byte of the frame.
     * @param low Second byte of the frame.
     * @return The third byte of a valid frame.
     */
    static uint8_t crc(uint8_t high, uint8_t low);

private:
    /**
     * @brief Act on one complete frame and prepare the answer to it.
     *
     * @param frame The frame, most significant byte first.
     */
    void process(const std::array<uint8_t, frame_size>& frame);

    /**
     * @brief Build a data frame answering with a value, the error flag set while ERRFL is not zero.
     *
     * @param data The value.
     * @return The frame, in its low 24 bits.
     */
    uint32_t answer_with(uint16_t data) const;

    /**
     * @brief Get the register a write to an address changes.
     *
     * @param address Register address.
     * @return The register, or null when the address is not writable.
     */
    uint8_t* writable(uint16_t address);

    /**
     * @brief Volatile registers, each 8 bits wide.
     */
    ///@{
    uint8_t disable{0};
    uint8_t zposm{0};
    uint8_t zposl{0};
    uint8_t settings1{0};
    uint8_t settings2{0};
    uint8_t settings3{0};
    uint8_t ecc{0};
    ///@}

    /**
     * @brief Error flags, in ERRFL's layout.
     */
    uint16_t errfl{0};

    /**
     * @brief Frame shifted out next: the answer to the last complete frame.
     */
    uint32_t answer;

    /**
     * @brief Bytes of the frame being received.
     */
    std::array<uint8_t, frame_size> frame{};

    /**
     * @brief Number of bytes of the frame received so far.
     */
    std::size_t received_bytes{0};

    /**
     * @brief Address whose write command was received, so the next frame holds its data.
     */
    std::optional<uint16_t> pending_write;
};
}  // namespace micras::models

#endif  // MICRAS_MODELS_AS5047U_MODEL_HPP
