/**
 * @file
 */

#include <cstdint>
#include <span>

#include "micras/hal/host/board.hpp"
#include "micras/hal/host/clock.hpp"
#include "micras/hal/pwm_dma.hpp"

namespace micras::hal {
namespace {
/**
 * @brief Start a transfer of compare values.
 *
 * @param handle Timer handle.
 * @param channel Timer channel.
 * @param compares Values the DMA feeds the compare register with.
 */
template <typename T>
void start(TIM_HandleTypeDef* handle, uint32_t channel, std::span<const T> compares) {
    host::PwmDmaPort& port = host::Board::pwm_dma(handle, channel);
    port.touched = true;
    port.compares.assign(compares.begin(), compares.end());
    port.period = handle->Instance->ARR + 1;
    port.busy = true;
    port.transfers++;

    const uint64_t cycles_per_count = static_cast<uint64_t>(handle->Instance->PSC + 1) *
                                      host::Clock::instance().cycles_per_microsecond() * 1000000ULL /
                                      handle->Instance->kernel_clock;

    port.transfer_end = host::Clock::instance().now() + compares.size() * port.period * cycles_per_count;
}
}  // namespace

PwmDma::PwmDma(const Config& config) : handle{config.handle}, channel{config.timer_channel} {
    if (this->handle->State == HAL_TIM_STATE_RESET) {
        config.init_function();
    }

    this->initialized = this->handle->State == HAL_TIM_STATE_READY;
}

void PwmDma::start_dma(std::span<uint32_t> buffer) {
    if (this->is_busy()) {
        return;
    }

    start<uint32_t>(this->handle, this->channel, buffer);
}

void PwmDma::start_dma(std::span<uint16_t> buffer) {
    if (this->is_busy()) {
        return;
    }

    start<uint16_t>(this->handle, this->channel, buffer);
}

void PwmDma::stop_dma() {
    host::Board::pwm_dma(this->handle, this->channel).busy = false;
}

uint32_t PwmDma::get_compare(float duty_cycle) const {
    const float scaled = duty_cycle * static_cast<float>(this->handle->Instance->ARR + 1) * 0.01F;

    // NOLINTNEXTLINE(bugprone-incorrect-roundings): the rounding the firmware's own driver does.
    return static_cast<uint32_t>(scaled + 0.5F);
}

bool PwmDma::is_busy() {
    host::PwmDmaPort& port = host::Board::pwm_dma(this->handle, this->channel);

    if (port.busy and host::Clock::instance().now() >= port.transfer_end) {
        port.busy = false;
    }

    return port.busy;
}

bool PwmDma::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
