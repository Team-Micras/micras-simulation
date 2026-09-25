/**
 * @file
 */

#include <cstdint>
#include <span>

#include "micras/hal/crc.hpp"

namespace micras::hal {
namespace {
/**
 * @brief Reverse the order of the lowest bits of a value.
 *
 * @param value Value to reflect.
 * @param width Number of bits to reflect.
 * @return The reflected bits.
 */
uint32_t reflect(uint32_t value, uint32_t width) {
    uint32_t reflected = 0;

    for (uint32_t bit = 0; bit < width; bit++) {
        reflected |= ((value >> bit) & 1U) << (width - 1 - bit);
    }

    return reflected;
}

/**
 * @brief Get the width of the CRC a configuration computes.
 *
 * @param length CRCLength field.
 * @return Width in bits.
 */
uint32_t width_of(uint32_t length) {
    if (length == CRC_POLYLENGTH_7B) {
        return 7;
    }

    if (length == CRC_POLYLENGTH_8B) {
        return 8;
    }

    if (length == CRC_POLYLENGTH_16B) {
        return 16;
    }

    return 32;
}
}  // namespace

Crc::Crc(const Config& config) : handle{config.handle} { }

uint32_t Crc::calculate(std::span<const uint8_t> data) {
    const CRC_InitTypeDef& init = this->handle->Init;
    const uint32_t         width = width_of(init.CRCLength);
    const uint32_t         mask = width == 32 ? 0xFFFFFFFFU : (1U << width) - 1;
    const uint32_t         polynomial =
        (init.DefaultPolynomialUse == DEFAULT_POLYNOMIAL_ENABLE ? DEFAULT_CRC32_POLY : init.GeneratingPolynomial) &
        mask;
    uint32_t crc =
        (init.DefaultInitValueUse == DEFAULT_INIT_VALUE_ENABLE ? DEFAULT_CRC_INITVALUE : init.InitValue) & mask;

    for (const uint8_t byte : data) {
        const uint32_t input = init.InputDataInversionMode == CRC_INPUTDATA_INVERSION_NONE ? byte : reflect(byte, 8);

        for (int bit = 7; bit >= 0; bit--) {
            const uint32_t top = (crc >> (width - 1)) & 1U;
            crc = (crc << 1U) & mask;

            if ((top ^ ((input >> static_cast<uint32_t>(bit)) & 1U)) != 0) {
                crc ^= polynomial;
            }
        }
    }

    return init.OutputDataInversionMode == CRC_OUTPUTDATA_INVERSION_ENABLE ? reflect(crc, width) : crc;
}
}  // namespace micras::hal
