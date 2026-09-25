/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

#include "micras/hal/spi.hpp"

namespace micras::hal {
std::array<Spi*, Spi::max_transfers> Spi::transferring{};

Spi::Spi(const Config& config) :
    handle{config.handle},
    cs_gpio{config.cs_gpio},
    timeout{config.timeout},
    clock_polarity{config.clock_polarity},
    clock_phase{config.clock_phase} {
    this->unselect_device();

    if (this->handle->State == HAL_SPI_STATE_RESET) {
        config.init_function();
    }

    this->initialized = this->handle->State == HAL_SPI_STATE_READY;
}

Spi::~Spi() {
    std::ranges::replace(transferring, this, static_cast<Spi*>(nullptr));
}

bool Spi::select_device() {
    this->handle->Init.CLKPolarity = this->clock_polarity;
    this->handle->Init.CLKPhase = this->clock_phase;
    this->cs_gpio.write(false);
    return true;
}

void Spi::unselect_device() {
    this->cs_gpio.write(true);
}

// NOLINTBEGIN(readability-convert-member-functions-to-static): hal::Spi's interface, declared by the firmware.
bool Spi::transmit(std::span<const uint8_t> /*data*/) {
    return false;
}

bool Spi::receive(std::span<uint8_t> /*data*/) {
    return false;
}

bool Spi::transmit_receive(std::span<const uint8_t> /*transmitted*/, std::span<uint8_t> /*received*/) {
    return false;
}

// NOLINTEND(readability-convert-member-functions-to-static)

bool Spi::start_transfer(std::span<const uint8_t> /*transmitted*/, std::span<uint8_t> /*received*/) {
    this->transfer = Transfer::FAILED;
    return false;
}

Spi::Transfer Spi::get_transfer() const {
    return this->transfer;
}

void Spi::on_transfer_end(const SPI_HandleTypeDef* /*handle*/, bool /*succeeded*/) { }

bool Spi::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
