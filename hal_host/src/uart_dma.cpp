/**
 * @file
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "micras/hal/host/board.hpp"
#include "micras/hal/uart_dma.hpp"

namespace micras::hal {
UartDma::UartDma(const Config& config) : handle{config.handle} {
    if (this->handle->gState == HAL_UART_STATE_RESET) {
        config.init_function();
    }

    host::Board::uart(this->handle).baud_rate = this->handle->Init.BaudRate;
}

bool UartDma::start_rx(std::span<uint8_t> buffer) {
    host::UartPort& port = host::Board::uart(this->handle);
    port.touched = true;
    port.rx_buffer = buffer;
    port.rx_head = 0;

    this->rx_buffer = buffer;
    this->rx_tail = 0;
    this->handle->RxState = HAL_UART_STATE_BUSY_RX;
    this->initialized = not buffer.empty();

    return this->initialized;
}

std::size_t UartDma::rx_head() const {
    return host::Board::uart(this->handle).rx_head;
}

std::size_t UartDma::read(std::span<uint8_t> into) {
    if (this->rx_buffer.empty()) {
        return 0;
    }

    const std::size_t size = this->rx_buffer.size();
    const std::size_t available = (this->rx_head() + size - this->rx_tail) % size;
    const std::size_t taken = std::min(available, into.size());
    const std::size_t first = std::min(taken, size - this->rx_tail);

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access): bounded just above.
    std::memcpy(into.data(), &this->rx_buffer[this->rx_tail], first);

    if (first < taken) {
        std::memcpy(&into[first], this->rx_buffer.data(), taken - first);
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)

    this->rx_tail = (this->rx_tail + taken) % size;

    return taken;
}

bool UartDma::start_tx(std::span<const uint8_t> from) {
    if (from.empty() or this->is_transmitting()) {
        return false;
    }

    host::UartPort& port = host::Board::uart(this->handle);
    port.touched = true;
    port.tx.insert(port.tx.end(), from.begin(), from.end());
    return true;
}

bool UartDma::is_transmitting() const {
    return host::Board::uart(this->handle).transmitting();
}

bool UartDma::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
