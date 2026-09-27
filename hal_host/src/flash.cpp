/**
 * @file
 */

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

#include "micras/hal/flash.hpp"
#include "micras/hal/host/board.hpp"

namespace micras::hal {
namespace {
/**
 * @brief Round a size up to whole flash words.
 *
 * @param size Size in bytes.
 * @return The rounded size.
 */
constexpr uint32_t align_size(uint32_t size) {
    return (size + FlashWord::size - 1) / FlashWord::size * FlashWord::size;
}

/**
 * @brief Get the flash contents, erased on first use.
 *
 * @return The port.
 */
host::FlashPort& storage() {
    host::FlashPort& port = host::Board::flash();

    if (port.bytes.size() != Flash::total_size) {
        port.bytes.assign(Flash::total_size, FlashWord::erased_value);
    }

    port.touched = true;
    return port;
}
}  // namespace

FlashWord::FlashWord(std::span<const uint8_t> data) {
    const auto data_address = std::bit_cast<uintptr_t>(data.data());

    if (data.size() >= size and data_address % alignof(uint32_t) == 0) {
        this->source = std::bit_cast<const uint32_t*>(data.data());
        return;
    }

    const std::span<uint8_t> bytes{std::bit_cast<uint8_t*>(this->buffer.data()), size};

    std::ranges::fill(bytes, erased_value);
    std::ranges::copy(data.first(std::min<std::size_t>(data.size(), size)), bytes.begin());

    this->source = this->buffer.data();
}

const uint32_t* FlashWord::data() const {
    return this->source;
}

bool FlashWord::is_padded() const {
    return this->source == this->buffer.data();
}

std::span<const uint8_t> Flash::read(uint32_t address, uint32_t size) {
    if (address > total_size or size > total_size - address) {
        return {};
    }

    return std::span<const uint8_t>{storage().bytes}.subspan(address, size);
}

std::span<const uint8_t> Flash::read(uint16_t sector, uint32_t sector_address, uint32_t size) {
    if (sector >= total_sectors or sector_address > sector_size) {
        return {};
    }

    return read(sector * sector_size + sector_address, size);
}

Flash::Status Flash::write(uint32_t address, std::span<const uint8_t> data) {
    if (address % FlashWord::size != 0) {
        return Status::MISALIGNED;
    }

    if (address > total_size or align_size(data.size()) > total_size - address) {
        return Status::OUT_OF_BOUNDS;
    }

    host::FlashPort&         port = storage();
    const std::span<uint8_t> memory{port.bytes};

    for (uint32_t offset = 0; offset < data.size(); offset += FlashWord::size) {
        const FlashWord          word{data.subspan(offset)};
        const std::span<uint8_t> target = memory.subspan(address + offset, FlashWord::size);

        if (not std::ranges::all_of(target, [](uint8_t byte) { return byte == FlashWord::erased_value; })) {
            return Status::ERROR;
        }

        const std::span<const uint8_t> bytes{std::bit_cast<const uint8_t*>(word.data()), FlashWord::size};
        std::ranges::copy(bytes, target.begin());
    }

    return Status::OK;
}

Flash::Status Flash::write(uint16_t sector, uint32_t sector_address, std::span<const uint8_t> data) {
    if (sector >= total_sectors or sector_address > sector_size) {
        return Status::OUT_OF_BOUNDS;
    }

    return write(sector * sector_size + sector_address, data);
}

Flash::Status Flash::erase_sectors(uint16_t start_sector, uint16_t number_of_sectors) {
    if (start_sector >= total_sectors or number_of_sectors > total_sectors - start_sector) {
        return Status::OUT_OF_BOUNDS;
    }

    host::FlashPort& port = storage();
    const auto       first = static_cast<std::ptrdiff_t>(start_sector) * sector_size;
    const auto       count = static_cast<std::ptrdiff_t>(number_of_sectors) * sector_size;

    std::fill(port.bytes.begin() + first, port.bytes.begin() + first + count, FlashWord::erased_value);

    return Status::OK;
}
}  // namespace micras::hal
