/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

#include "micras/hal/adc_dma.hpp"
#include "micras/hal/host/board.hpp"

namespace micras::hal {
std::array<AdcDma*, AdcDma::max_instances> AdcDma::instances{};
uint32_t                                   AdcDma::restarts{};

AdcDma::AdcDma(const Config& config) :
    max_reading{config.max_reading}, reference_voltage{config.reference_voltage}, handle{config.handle} {
    if (this->handle->State == HAL_ADC_STATE_RESET) {
        config.init_function();
    }

    auto* const slot = std::ranges::find(instances, nullptr);

    if (slot == instances.end()) {
        return;
    }

    *slot = this;
    this->initialized = this->handle->State == HAL_ADC_STATE_READY;
}

AdcDma::~AdcDma() {
    std::ranges::replace(instances, this, static_cast<AdcDma*>(nullptr));
}

bool AdcDma::start_dma(std::span<uint32_t> buffer) {
    host::AdcPort& port = host::Board::adc(this->handle);
    port.touched = true;
    port.buffer32 = buffer;
    port.buffer16 = {};
    port.complete = [adc = this->handle] { on_sequence_complete(adc); };
    return true;
}

bool AdcDma::start_dma(std::span<uint16_t> buffer) {
    host::AdcPort& port = host::Board::adc(this->handle);
    port.touched = true;
    port.buffer16 = buffer;
    port.buffer32 = {};
    port.complete = [adc = this->handle] { on_sequence_complete(adc); };
    return true;
}

bool AdcDma::start_dma(std::span<uint16_t> buffer, std::span<uint16_t> snapshot) {
    if (snapshot.size() != buffer.size()) {
        this->initialized = false;
        return false;
    }

    this->buffer = buffer;
    this->snapshot = snapshot;

    return this->start_dma(buffer);
}

uint32_t AdcDma::read_snapshot(std::span<uint16_t> destination) const {
    uint32_t before = this->sequence;

    while (true) {
        std::ranges::copy(this->snapshot, destination.begin());

        const uint32_t after = this->sequence;

        if (after == before) {
            return after;
        }

        before = after;
    }
}

void AdcDma::on_sequence_complete(const ADC_HandleTypeDef* handle) {
    AdcDma* const instance = find(handle);

    if (instance == nullptr) {
        return;
    }

    std::ranges::copy(instance->buffer, instance->snapshot.begin());
    instance->sequence = instance->sequence + 1;
}

void AdcDma::on_error(const ADC_HandleTypeDef* handle) {
    AdcDma* const instance = find(handle);

    if (instance != nullptr) {
        instance->stopped = true;
    }
}

AdcDma* AdcDma::find(const ADC_HandleTypeDef* handle) {
    const auto* const found = std::ranges::find_if(instances, [handle](const AdcDma* instance) {
        return instance != nullptr and instance->handle == handle;
    });

    return found == instances.end() ? nullptr : *found;
}

void AdcDma::stop_dma() {
    host::AdcPort& port = host::Board::adc(this->handle);
    port.buffer16 = {};
    port.buffer32 = {};
    port.complete = nullptr;
}

void AdcDma::recover() {
    if (not this->stopped) {
        return;
    }

    this->stopped = false;
    restarts++;
}

uint32_t AdcDma::get_restarts() {
    return restarts;
}

uint16_t AdcDma::get_max_reading() const {
    return this->max_reading;
}

float AdcDma::get_reference_voltage() const {
    return this->reference_voltage;
}

bool AdcDma::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
