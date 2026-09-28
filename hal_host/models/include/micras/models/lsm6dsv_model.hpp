/**
 * @file
 *
 * @brief The LSM6DSV inertial measurement unit, as its SPI interface shows it.
 */

#ifndef MICRAS_MODELS_LSM6DSV_MODEL_HPP
#define MICRAS_MODELS_LSM6DSV_MODEL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "micras/hal/host/spi_device.hpp"

namespace micras::models {
/**
 * @brief The registers of an LSM6DSV behind an SPI chip select, fed with samples of the motion it measures.
 *
 * @details The first byte of a transaction is the command: the read flag in the
 *          top bit and the register address below it. Every following byte
 *          reads or writes one register, and the address advances after each
 *          one while CTRL3 IF_INC is set, as it is after reset.
 *
 * @note The register file is the 128 registers of the main page, with the
 *       datasheet's defaults: WHO_AM_I 0x70, PIN_CTRL 0x23, CTRL3 0x44
 *       (IF_INC and BDU) and zero everywhere else. Writes to read-only and
 *       reserved registers are ignored. While FUNC_CFG_ACCESS selects the
 *       embedded functions or the sensor hub page, which are not modeled, every
 *       other register reads zero and ignores writes.
 *
 * @note Writing SW_POR in FUNC_CFG_ACCESS or SW_RESET in CTRL3 restores the
 *       defaults, and the rest of that transaction is ignored.
 *
 * @note A sample updates the outputs of each sensor whose output data rate is
 *       not off (ODR_G in CTRL2, ODR_XL in CTRL1), encoded with the full scale
 *       its register holds (FS_G in CTRL6, FS_XL in CTRL8), and sets that
 *       sensor's data ready bit in STATUS_REG (GDA, XLDA). Reading the high
 *       byte of one of the sensor's axes clears the bit, so the burst that
 *       reads a sample clears it. A transaction happens between two samples,
 *       which is what block data update guarantees on the chip.
 *
 * @note The rates in CTRL1, CTRL2 and HAODR_CFG only say whether a sensor is on
 *       and are decoded for the one feeding the samples: the samples arrive at
 *       whatever rate push_sample is called at.
 */
class Lsm6dsvModel : public hal::host::SpiDevice {
public:
    /**
     * @brief Registers of the main page.
     */
    static constexpr std::size_t register_count{128};

    /**
     * @brief Value of WHO_AM_I.
     */
    static constexpr uint8_t device_id{0x70};

    /**
     * @brief Power the chip on, in SPI mode 3.
     */
    Lsm6dsvModel();

    /**
     * @brief Start a transaction.
     */
    void select() override;

    /**
     * @brief Take the command, then read or write one register per byte.
     *
     * @param transmitted Bytes the controller sends.
     * @param received Register values read, zero while the command is being sent and during writes.
     */
    void exchange(std::span<const uint8_t> transmitted, std::span<uint8_t> received) override;

    /**
     * @brief End the transaction.
     */
    void deselect() override;

    /**
     * @brief Deliver a new sample of the motion the chip measures.
     *
     * @param angular_velocity Angular velocity about the chip's x, y and z axes, in rad/s.
     * @param linear_acceleration Specific force along the chip's x, y and z axes, in m/s^2.
     */
    void push_sample(const std::array<double, 3>& angular_velocity, const std::array<double, 3>& linear_acceleration);

    /**
     * @brief Get a register without the side effects of reading it over SPI.
     *
     * @param address Register address; the top bit is ignored.
     * @return The register's value.
     */
    uint8_t peek(uint8_t address) const;

    /**
     * @brief Get the output data rate of the gyroscope, from CTRL2 and HAODR_CFG.
     *
     * @return The rate, in hertz, zero when the gyroscope is off.
     */
    double gyroscope_data_rate() const;

    /**
     * @brief Get the output data rate of the accelerometer, from CTRL1 and HAODR_CFG.
     *
     * @return The rate, in hertz, zero when the accelerometer is off.
     */
    double accelerometer_data_rate() const;

    /**
     * @brief Get the sensitivity of the gyroscope at the full scale in CTRL6.
     *
     * @return Angular velocity of one count, in rad/s, zero for a reserved full scale.
     */
    double gyroscope_sensitivity() const;

    /**
     * @brief Get the sensitivity of the accelerometer at the full scale in CTRL8.
     *
     * @return Specific force of one count, in m/s^2.
     */
    double accelerometer_sensitivity() const;

private:
    /**
     * @brief Restore every register to its default.
     */
    void reset();

    /**
     * @brief Read a register over SPI, clearing the data ready bit it completes.
     *
     * @param address Register address.
     * @return The register's value.
     */
    uint8_t read(uint8_t address);

    /**
     * @brief Write a register over SPI, applying what the write triggers.
     *
     * @param address Register address.
     * @param value Value written.
     */
    void write(uint8_t address, uint8_t value);

    /**
     * @brief Check whether the main page is the one the SPI interface addresses.
     *
     * @return True unless FUNC_CFG_ACCESS selects another page.
     */
    bool main_page() const;

    /**
     * @brief Write the three axes of one sensor into its output registers.
     *
     * @param first Address of the low byte of the x axis.
     * @param values Values of the three axes.
     * @param sensitivity Value of one count; zero writes zero.
     */
    void encode(uint8_t first, const std::array<double, 3>& values, double sensitivity);

    /**
     * @brief Decode an output data rate.
     *
     * @param odr ODR field of CTRL1 or CTRL2.
     * @return The rate, in hertz, with the high accuracy set HAODR_CFG selects.
     */
    double data_rate(uint8_t odr) const;

    /**
     * @brief The register file.
     */
    std::array<uint8_t, register_count> registers{};

    /**
     * @brief Register the next byte of the transaction reads or writes.
     */
    uint8_t address{0};

    /**
     * @brief Whether the command byte of the transaction was received.
     */
    bool addressed{false};

    /**
     * @brief Whether the transaction reads.
     */
    bool reading{false};

    /**
     * @brief Whether the rest of the transaction is ignored, after a reset inside it.
     */
    bool ignoring{false};
};
}  // namespace micras::models

#endif  // MICRAS_MODELS_LSM6DSV_MODEL_HPP
