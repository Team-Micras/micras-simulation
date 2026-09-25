/**
 * @file
 */

#include "micras/hal/gpio.hpp"
#include "micras/hal/host/board.hpp"

namespace micras::hal {
Gpio::Gpio(const Config& config) : port{config.port}, pin{config.pin} { }

bool Gpio::read() const {
    host::GpioPort& gpio = host::Board::gpio(this->port, this->pin);
    gpio.touched = true;
    return gpio.read();
}

void Gpio::write(bool state) {
    host::GpioPort& gpio = host::Board::gpio(this->port, this->pin);
    gpio.touched = true;
    gpio.output = state;
}

void Gpio::toggle() {
    this->write(not host::Board::gpio(this->port, this->pin).output);
}
}  // namespace micras::hal
