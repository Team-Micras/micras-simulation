/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>
#include <utility>

#include "micras/models/lsm6dsv_model.hpp"

namespace micras::models {
namespace {
/**
 * @brief Top bit of the command byte: set to read, clear to write.
 */
constexpr uint8_t read_flag{0x80};

/**
 * @brief Bits of the command byte and of an address that name a register.
 */
constexpr uint8_t address_mask{0x7F};

/**
 * @brief Addresses of the registers the model gives a behavior to.
 */
///@{
constexpr uint8_t func_cfg_access{0x01};
constexpr uint8_t pin_ctrl{0x02};
constexpr uint8_t who_am_i{0x0F};
constexpr uint8_t ctrl1{0x10};
constexpr uint8_t ctrl2{0x11};
constexpr uint8_t ctrl3{0x12};
constexpr uint8_t ctrl6{0x15};
constexpr uint8_t ctrl8{0x17};
constexpr uint8_t status_reg{0x1E};
constexpr uint8_t outx_l_g{0x22};
constexpr uint8_t outx_l_a{0x28};
constexpr uint8_t haodr_cfg{0x62};
///@}

/**
 * @brief Bits of FUNC_CFG_ACCESS: the software power-on reset, and the two that select another page.
 */
///@{
constexpr uint8_t sw_por{0x04};
constexpr uint8_t other_pages{0xC0};
///@}

/**
 * @brief Bits of CTRL3: the software reset, the self-clearing reboot, and the address auto-increment.
 */
///@{
constexpr uint8_t sw_reset{0x01};
constexpr uint8_t boot{0x80};
constexpr uint8_t if_inc{0x04};
///@}

/**
 * @brief Data ready bits of STATUS_REG.
 */
///@{
constexpr uint8_t xlda{0x01};
constexpr uint8_t gda{0x02};
///@}

/**
 * @brief Defaults of the registers that do not reset to zero.
 */
constexpr std::array<std::pair<uint8_t, uint8_t>, 3> nonzero_defaults{{
    {pin_ctrl, 0x23},
    {who_am_i, Lsm6dsvModel::device_id},
    {ctrl3, 0x44},
}};

/**
 * @brief First and last address of every range of registers the SPI interface can write.
 */
constexpr std::array<std::pair<uint8_t, uint8_t>, 7> writable_ranges{{
    {0x01, 0x03},
    {0x06, 0x0E},
    {0x10, 0x19},
    {0x50, 0x51},
    {0x54, 0x5F},
    {0x62, 0x6B},
    {0x6F, 0x75},
}};

/**
 * @brief Sensitivity of the gyroscope for each FS_G code, in mdps per count; zero for the reserved codes.
 */
constexpr std::array<double, 16> gyroscope_mdps_per_count{
    4.375, 8.75, 17.5, 35.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 140.0, 0.0, 0.0, 0.0,
};

/**
 * @brief Sensitivity of the accelerometer for each FS_XL code, in mg per count.
 */
constexpr std::array<double, 4> accelerometer_mg_per_count{0.061, 0.122, 0.244, 0.488};

/**
 * @brief Conversions from the datasheet's units.
 */
///@{
constexpr double mdps_to_radps{std::numbers::pi / 180000.0};
constexpr double mg_to_mps2{0.00980665};
///@}

/**
 * @brief Rate of ODR code 3 for each high accuracy set of HAODR_CFG, in hertz; each code above doubles it.
 */
constexpr std::array<double, 4> base_rates{15.0, 15.625, 12.5, 15.0};

/**
 * @brief Check whether the SPI interface can write a register.
 *
 * @param address Register address.
 * @return True for a read-write register.
 */
bool writable(uint8_t address) {
    return std::ranges::any_of(writable_ranges, [address](const auto& range) {
        return address >= range.first and address <= range.second;
    });
}
}  // namespace

Lsm6dsvModel::Lsm6dsvModel() : SpiDevice{Mode::MODE_3} {
    this->reset();
}

void Lsm6dsvModel::select() {
    this->addressed = false;
    this->ignoring = false;
}

void Lsm6dsvModel::exchange(std::span<const uint8_t> transmitted, std::span<uint8_t> received) {
    const std::size_t size = std::min(transmitted.size(), received.size());

    for (std::size_t index = 0; index < size; index++) {
        received[index] = 0;

        if (this->ignoring) {
            continue;
        }

        if (not this->addressed) {
            this->reading = (transmitted[index] & read_flag) != 0;
            this->address = transmitted[index] & address_mask;
            this->addressed = true;
            continue;
        }

        if (this->reading) {
            received[index] = this->read(this->address);
        } else {
            this->write(this->address, transmitted[index]);
        }

        if ((this->registers.at(ctrl3) & if_inc) != 0) {
            this->address = (this->address + 1) & address_mask;
        }
    }
}

void Lsm6dsvModel::deselect() {
    this->addressed = false;
    this->ignoring = false;
}

void Lsm6dsvModel::push_sample(
    const std::array<double, 3>& angular_velocity, const std::array<double, 3>& linear_acceleration
) {
    if ((this->registers.at(ctrl2) & 0x0F) != 0) {
        this->encode(outx_l_g, angular_velocity, this->gyroscope_sensitivity());
        this->registers.at(status_reg) |= gda;
    }

    if ((this->registers.at(ctrl1) & 0x0F) != 0) {
        this->encode(outx_l_a, linear_acceleration, this->accelerometer_sensitivity());
        this->registers.at(status_reg) |= xlda;
    }
}

uint8_t Lsm6dsvModel::peek(uint8_t address) const {
    return this->registers.at(address & address_mask);
}

double Lsm6dsvModel::gyroscope_data_rate() const {
    return this->data_rate(this->registers.at(ctrl2) & 0x0F);
}

double Lsm6dsvModel::accelerometer_data_rate() const {
    return this->data_rate(this->registers.at(ctrl1) & 0x0F);
}

double Lsm6dsvModel::gyroscope_sensitivity() const {
    return gyroscope_mdps_per_count.at(this->registers.at(ctrl6) & 0x0F) * mdps_to_radps;
}

double Lsm6dsvModel::accelerometer_sensitivity() const {
    return accelerometer_mg_per_count.at(this->registers.at(ctrl8) & 0x03) * mg_to_mps2;
}

void Lsm6dsvModel::reset() {
    this->registers.fill(0);

    for (const auto& [address, value] : nonzero_defaults) {
        this->registers.at(address) = value;
    }
}

uint8_t Lsm6dsvModel::read(uint8_t address) {
    if (not this->main_page() and address != func_cfg_access) {
        return 0;
    }

    const uint8_t value = this->registers.at(address);

    if (address >= outx_l_g and address < outx_l_a and (address & 1) != 0) {
        this->registers.at(status_reg) &= ~gda;
    } else if (address >= outx_l_a and address < outx_l_a + 6 and (address & 1) != 0) {
        this->registers.at(status_reg) &= ~xlda;
    }

    return value;
}

void Lsm6dsvModel::write(uint8_t address, uint8_t value) {
    if ((not this->main_page() and address != func_cfg_access) or not writable(address)) {
        return;
    }

    if ((address == func_cfg_access and (value & sw_por) != 0) or (address == ctrl3 and (value & sw_reset) != 0)) {
        this->reset();
        this->ignoring = true;
        return;
    }

    this->registers.at(address) = address == ctrl3 ? value & ~boot : value;
}

bool Lsm6dsvModel::main_page() const {
    return (this->registers.at(func_cfg_access) & other_pages) == 0;
}

void Lsm6dsvModel::encode(uint8_t first, const std::array<double, 3>& values, double sensitivity) {
    for (std::size_t axis = 0; axis < values.size(); axis++) {
        const double counts = sensitivity > 0.0 ? std::round(values.at(axis) / sensitivity) : 0.0;
        const auto   word = static_cast<uint16_t>(static_cast<int16_t>(std::clamp(counts, -32768.0, 32767.0)));
        const auto   low = static_cast<std::size_t>(first + (2 * axis));

        this->registers.at(low) = static_cast<uint8_t>(word);
        this->registers.at(low + 1) = static_cast<uint8_t>(word >> 8U);
    }
}

double Lsm6dsvModel::data_rate(uint8_t odr) const {
    if (odr == 0 or odr > 12) {
        return 0.0;
    }

    if (odr == 1) {
        return 1.875;
    }

    if (odr == 2) {
        return 7.5;
    }

    return base_rates.at(this->registers.at(haodr_cfg) & 0x03) * static_cast<double>(1U << (odr - 3U));
}
}  // namespace micras::models
