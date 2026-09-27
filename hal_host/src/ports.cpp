/**
 * @file
 */

#include "micras/hal/host/ports.hpp"

namespace micras::hal::host {
void AdcPort::write(std::size_t index, uint32_t counts) const {
    if (index < this->buffer16.size()) {
        this->buffer16[index] = static_cast<uint16_t>(counts);
    } else if (index < this->buffer32.size()) {
        this->buffer32[index] = counts;
    }
}

void AdcPort::finish_sequence() const {
    if (this->complete) {
        this->complete();
    }
}

void UartPort::receive(uint8_t byte) {
    if (this->rx_buffer.empty()) {
        return;
    }

    this->rx_buffer[this->rx_head] = byte;
    this->rx_head = (this->rx_head + 1) % this->rx_buffer.size();
}
}  // namespace micras::hal::host
