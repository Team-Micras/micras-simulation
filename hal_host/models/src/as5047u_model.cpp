/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

#include "micras/models/as5047u_model.hpp"

namespace micras::models {
namespace {
/**
 * @brief Generator polynomial of the frame CRC, without its top bit.
 */
constexpr uint8_t crc_polynomial{0x1D};

/**
 * @brief Initial value of the frame CRC.
 */
constexpr uint8_t crc_initial_value{0xC4};

/**
 * @brief Final XOR of the frame CRC.
 */
constexpr uint8_t crc_final_xor{0xFF};

/**
 * @brief Bits of the 14-bit address or data field.
 */
constexpr uint16_t field_mask{0x3FFF};

/**
 * @brief Position of the address or data field in a frame.
 */
constexpr uint32_t field_shift{8};

/**
 * @brief Read flag of a command frame, and error flag of a data frame.
 */
constexpr uint32_t flag_bit{1U << 22U};
}  // namespace

As5047uModel::As5047uModel() : SpiDevice{Mode::MODE_1}, answer{this->answer_with(0)} { }

void As5047uModel::select() {
    this->received_bytes = 0;
}

void As5047uModel::exchange(std::span<const uint8_t> transmitted, std::span<uint8_t> received) {
    const std::size_t size = std::min(transmitted.size(), received.size());

    for (std::size_t index = 0; index < size; index++) {
        const uint32_t shift = 8U * static_cast<uint32_t>(frame_size - 1 - this->received_bytes);
        received[index] = static_cast<uint8_t>(this->answer >> shift);
        this->frame.at(this->received_bytes) = transmitted[index];
        this->received_bytes++;

        if (this->received_bytes == frame_size) {
            this->received_bytes = 0;
            this->process(this->frame);
        }
    }
}

void As5047uModel::deselect() {
    if (this->received_bytes != 0) {
        this->errfl |= framing_error;
        this->pending_write.reset();
    }

    this->received_bytes = 0;
}

uint16_t As5047uModel::peek(uint16_t address) const {
    switch (address) {
        case errfl_address:
            return this->errfl;

        case disable_address:
            return this->disable;

        case zposm_address:
            return this->zposm;

        case zposl_address:
            return this->zposl;

        case settings1_address:
            return this->settings1;

        case settings2_address:
            return this->settings2;

        case settings3_address:
            return this->settings3;

        case ecc_address:
            return this->ecc;

        default:
            return 0;
    }
}

uint8_t As5047uModel::crc(uint8_t high, uint8_t low) {
    uint8_t remainder = crc_initial_value;

    for (const uint8_t byte : {high, low}) {
        remainder ^= byte;

        for (uint8_t bit = 0; bit < 8; bit++) {
            remainder = (remainder & 0x80U) != 0 ? static_cast<uint8_t>((remainder << 1U) ^ crc_polynomial) :
                                                   static_cast<uint8_t>(remainder << 1U);
        }
    }

    return remainder ^ crc_final_xor;
}

void As5047uModel::process(const std::array<uint8_t, frame_size>& frame) {
    const uint32_t raw = static_cast<uint32_t>(std::get<0>(frame)) << 16U |
                         static_cast<uint32_t>(std::get<1>(frame)) << 8U | std::get<2>(frame);
    const auto     field = static_cast<uint16_t>((raw >> field_shift) & field_mask);

    if (crc(std::get<0>(frame), std::get<1>(frame)) != std::get<2>(frame)) {
        this->errfl |= crc_error;
        this->pending_write.reset();
        this->answer = this->answer_with(0);
        return;
    }

    if (this->pending_write.has_value()) {
        const uint16_t address = this->pending_write.value();
        this->pending_write.reset();

        if (uint8_t* const target = this->writable(address); target != nullptr) {
            *target = static_cast<uint8_t>(field);
        }

        this->answer = this->answer_with(this->peek(address));
        return;
    }

    this->answer = this->answer_with(this->peek(field));

    if ((raw & flag_bit) == 0) {
        this->pending_write = field;
    } else if (field == errfl_address) {
        this->errfl = 0;
    }
}

uint32_t As5047uModel::answer_with(uint16_t data) const {
    const uint32_t upper = (this->errfl != 0 ? flag_bit : 0U) | static_cast<uint32_t>(data & field_mask) << field_shift;
    return upper | crc(static_cast<uint8_t>(upper >> 16U), static_cast<uint8_t>(upper >> 8U));
}

uint8_t* As5047uModel::writable(uint16_t address) {
    switch (address) {
        case disable_address:
            return &this->disable;

        case zposm_address:
            return &this->zposm;

        case zposl_address:
            return &this->zposl;

        case settings1_address:
            return &this->settings1;

        case settings2_address:
            return &this->settings2;

        case settings3_address:
            return &this->settings3;

        case ecc_address:
            return &this->ecc;

        default:
            return nullptr;
    }
}
}  // namespace micras::models
