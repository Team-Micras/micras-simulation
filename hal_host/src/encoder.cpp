/**
 * @file
 */

#include <cstdint>

#include "micras/hal/encoder.hpp"
#include "micras/hal/host/board.hpp"

namespace micras::hal {
Encoder::Encoder(const Config& config) : handle{config.handle}, start_count{0} {
    if (this->handle->State == HAL_TIM_STATE_RESET) {
        config.init_function();
    }

    host::EncoderPort& port = host::Board::encoder(this->handle);
    port.touched = true;
    this->start_count = static_cast<uint32_t>(port.count);
    this->initialized = this->handle->State == HAL_TIM_STATE_READY;
}

int32_t Encoder::get_counter() const {
    return static_cast<int32_t>(static_cast<uint32_t>(host::Board::encoder(this->handle).count) - this->start_count);
}

bool Encoder::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
