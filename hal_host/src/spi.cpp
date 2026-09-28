/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <span>
#include <vector>

#include "micras/hal/host/board.hpp"
#include "micras/hal/host/clock.hpp"
#include "micras/hal/host/spi_device.hpp"
#include "micras/hal/spi.hpp"
#include "micras/hal/timer.hpp"

namespace micras::hal {
namespace {
/**
 * @brief What the host keeps for each Spi that the firmware's class has no member for.
 */
struct HostState {
    /**
     * @brief Chip select of the device, which keys its port together with the handle.
     */
    Gpio::Config cs{};

    /**
     * @brief Host clock cycle the running transfer ends at.
     */
    uint64_t transfer_end{0};
};

/**
 * @brief Get the host state of every Spi alive.
 *
 * @return The states, keyed by the object.
 */
std::map<const Spi*, HostState>& host_states() {
    static std::map<const Spi*, HostState> states;
    return states;
}

/**
 * @brief Get the port of the chip select an Spi drives.
 *
 * @param spi The Spi.
 * @param handle Its handle.
 * @return The port.
 */
host::SpiPort& port_of(const Spi* spi, const SPI_HandleTypeDef* handle) {
    const Gpio::Config& cs = host_states().at(spi).cs;
    return host::Board::spi(handle, cs.port, cs.pin);
}

/**
 * @brief Get the SPI mode the bus runs in, from the polarity and phase its handle holds.
 *
 * @param handle Handle of the bus.
 * @return The mode.
 */
host::SpiDevice::Mode mode_of(const SPI_HandleTypeDef& handle) {
    const bool polarity = handle.Init.CLKPolarity == SPI_POLARITY_HIGH;
    const bool phase = handle.Init.CLKPhase == SPI_PHASE_2EDGE;
    return static_cast<host::SpiDevice::Mode>((polarity ? 2 : 0) + (phase ? 1 : 0));
}

/**
 * @brief Clock bytes through the device a port selects.
 *
 * @param handle Handle of the bus.
 * @param port Port of the chip select.
 * @param transmitted Bytes sent.
 * @param received Bytes answered, as many as were sent; all ones when no device drives the bus in its mode.
 */
void exchange(
    const SPI_HandleTypeDef& handle, host::SpiPort& port, std::span<const uint8_t> transmitted,
    std::span<uint8_t> received
) {
    if (not port.selected or port.device == nullptr or port.device->mode() != mode_of(handle)) {
        std::ranges::fill(received, 0xFF);
        return;
    }

    port.device->exchange(transmitted, received);
}

/**
 * @brief Get how long a transfer takes on a bus.
 *
 * @param handle Handle of the bus.
 * @param bytes Bytes transferred.
 * @return Host clock cycles, zero when the bus has no clock to time it by.
 */
uint64_t transfer_cycles(const SPI_HandleTypeDef& handle, std::size_t bytes) {
    if (handle.Instance == nullptr or handle.Instance->kernel_clock == 0) {
        return 0;
    }

    const uint64_t kernel_clock = handle.Instance->kernel_clock;
    const uint64_t divider = 2ULL << (handle.Init.BaudRatePrescaler >> 28U);
    const uint64_t cycles_per_second = host::Clock::instance().cycles_per_microsecond() * 1000000ULL;

    return ((bytes * 8 * divider * cycles_per_second) + kernel_clock - 1) / kernel_clock;
}
}  // namespace

std::array<Spi*, Spi::max_transfers> Spi::transferring{};

Spi::Spi(const Config& config) :
    handle{config.handle},
    cs_gpio{config.cs_gpio},
    timeout{config.timeout},
    clock_polarity{config.clock_polarity},
    clock_phase{config.clock_phase} {
    host_states()[this] = HostState{.cs = config.cs_gpio};
    this->unselect_device();

    if (this->handle->State == HAL_SPI_STATE_RESET) {
        config.init_function();
    }

    this->initialized = this->handle->State == HAL_SPI_STATE_READY;
}

Spi::~Spi() {
    if (this->transfer == Transfer::RUNNING) {
        this->unselect_device();
    }

    std::ranges::replace(transferring, this, static_cast<Spi*>(nullptr));
    host_states().erase(this);
}

bool Spi::select_device() {
    const auto bus_busy = [this] {
        return std::ranges::any_of(transferring, [this](const Spi* device) {
            return device != nullptr and device->handle == this->handle and device->get_transfer() == Transfer::RUNNING;
        });
    };

    if (bus_busy()) {
        const uint32_t start = Timer::get_counter();
        const uint32_t limit = Timer::to_cycles(1000 * this->timeout);

        while (bus_busy()) {
            if (Timer::get_counter() - start > limit) {
                return false;
            }
        }
    }

    this->handle->Init.CLKPolarity = this->clock_polarity;
    this->handle->Init.CLKPhase = this->clock_phase;
    this->cs_gpio.write(false);

    host::SpiPort& port = port_of(this, this->handle);
    port.touched = true;

    if (not port.selected) {
        port.selected = true;

        if (port.device != nullptr) {
            port.device->select();
        }
    }

    return true;
}

void Spi::unselect_device() {
    this->cs_gpio.write(true);

    host::SpiPort& port = port_of(this, this->handle);

    if (port.selected) {
        port.selected = false;

        if (port.device != nullptr) {
            port.device->deselect();
        }
    }
}

bool Spi::transmit(std::span<const uint8_t> data) {
    if (this->handle->State != HAL_SPI_STATE_READY) {
        return false;
    }

    std::vector<uint8_t> ignored(data.size());
    exchange(*this->handle, port_of(this, this->handle), data, ignored);
    return true;
}

bool Spi::receive(std::span<uint8_t> data) {
    if (this->handle->State != HAL_SPI_STATE_READY) {
        return false;
    }

    const std::vector<uint8_t> zeros(data.size());
    exchange(*this->handle, port_of(this, this->handle), zeros, data);
    return true;
}

bool Spi::transmit_receive(std::span<const uint8_t> transmitted, std::span<uint8_t> received) {
    if (received.size() < transmitted.size() or this->handle->State != HAL_SPI_STATE_READY) {
        return false;
    }

    exchange(*this->handle, port_of(this, this->handle), transmitted, received.first(transmitted.size()));
    return true;
}

bool Spi::start_transfer(std::span<const uint8_t> transmitted, std::span<uint8_t> received) {
    auto* const slot = std::ranges::find(transferring, nullptr);

    if (received.size() < transmitted.size() or slot == transferring.end() or
        this->handle->State != HAL_SPI_STATE_READY or not this->select_device()) {
        this->transfer = Transfer::FAILED;
        return false;
    }

    *slot = this;
    this->transfer = Transfer::RUNNING;
    exchange(*this->handle, port_of(this, this->handle), transmitted, received.first(transmitted.size()));
    host_states().at(this).transfer_end =
        host::Clock::instance().now() + transfer_cycles(*this->handle, transmitted.size());

    return true;
}

Spi::Transfer Spi::get_transfer() const {
    if (this->transfer == Transfer::RUNNING and host::Clock::instance().now() >= host_states().at(this).transfer_end) {
        on_transfer_end(this->handle, true);
    }

    return this->transfer;
}

void Spi::on_transfer_end(const SPI_HandleTypeDef* handle, bool succeeded) {
    for (Spi*& device : transferring) {
        if (device != nullptr and device->handle == handle) {
            device->unselect_device();
            device->transfer = succeeded ? Transfer::COMPLETE : Transfer::FAILED;
            device = nullptr;
            return;
        }
    }
}

bool Spi::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
