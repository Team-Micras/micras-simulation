/**
 * @file
 */

#include <algorithm>
#include <cstdint>

#include "micras/hal/host/board.hpp"
#include "micras/hal/pwm.hpp"

namespace micras::hal {
namespace {
/**
 * @brief Mask of the channel offset, which the vendor HAL uses as a bit shift.
 */
constexpr uint32_t channel_shift_mask{0x1F};

/**
 * @brief Check whether a timer counts up and down.
 *
 * @param handle Timer handle.
 * @return True in a center-aligned mode.
 */
bool is_center_aligned(const TIM_HandleTypeDef* handle) {
    return (handle->Instance->CR1 & TIM_CR1_CMS) != 0;
}
}  // namespace

Pwm::Pwm(const Config& config) : handle{config.handle}, channel{config.timer_channel}, inverted{config.inverted} {
    if (this->handle->State == HAL_TIM_STATE_RESET) {
        config.init_function();
    }

    if (this->inverted) {
        this->handle->Instance->CCER |= TIM_CCER_CC1P << (this->channel & channel_shift_mask);
    }

    this->set_duty_cycle(0.0F);
    this->initialized = this->handle->State == HAL_TIM_STATE_READY;

    host::PwmPort& port = host::Board::pwm(this->handle, this->channel);
    port.frequency = this->get_frequency();
}

void Pwm::set_duty_cycle(float duty_cycle) {
    duty_cycle = std::clamp(duty_cycle, 0.0F, 100.0F);

    if (this->inverted) {
        duty_cycle = 100.0F - duty_cycle;
    }

    const uint32_t autoreload = this->handle->Instance->ARR;
    const float    scaled = duty_cycle * static_cast<float>(autoreload + 1) * 0.01F;

    // NOLINTNEXTLINE(bugprone-incorrect-roundings): the rounding the firmware's own driver does.
    const auto  compare = static_cast<uint32_t>(scaled + 0.5F);
    const float counts =
        is_center_aligned(this->handle) ? static_cast<float>(autoreload) : static_cast<float>(autoreload + 1);
    const float active = std::min(100.0F, 100.0F * static_cast<float>(compare) / counts);

    host::PwmPort& port = host::Board::pwm(this->handle, this->channel);
    port.touched = true;
    port.duty_cycle = this->inverted ? 100.0F - active : active;
}

void Pwm::set_frequency(uint32_t frequency) {
    const uint32_t prescaler = this->handle->Instance->PSC;

    this->handle->Instance->ARR = this->handle->Instance->kernel_clock / ((prescaler + 1) * frequency) - 1;

    host::PwmPort& port = host::Board::pwm(this->handle, this->channel);
    port.touched = true;
    port.frequency = this->get_frequency();
}

float Pwm::get_frequency() const {
    const uint32_t autoreload = this->handle->Instance->ARR;
    const uint32_t period = is_center_aligned(this->handle) ? 2 * autoreload : autoreload + 1;
    const auto     ticks = static_cast<float>(this->handle->Instance->PSC + 1) * static_cast<float>(period);

    return static_cast<float>(this->handle->Instance->kernel_clock) / ticks;
}

bool Pwm::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
