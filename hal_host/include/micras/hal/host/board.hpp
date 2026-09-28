/**
 * @file
 *
 * @brief Every host peripheral port, found by the handle or pin that names it.
 */

#ifndef MICRAS_HAL_HOST_BOARD_HPP
#define MICRAS_HAL_HOST_BOARD_HPP

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "micras/hal/host/ports.hpp"
#include "micras/hal/host/spi_device.hpp"

struct GPIO_TypeDef;
struct SPI_HandleTypeDef;

namespace micras::hal::host {
/**
 * @brief Registry of the ports the host backend and the outside world share.
 *
 * @note A port is created the first time either side asks for it, keyed by the
 *       address of the Cube handle or GPIO port that names it, so the firmware's
 *       own configuration is the key: `.port = LED_Red_GPIO_Port, .pin =
 *       LED_Red_Pin` in target.hpp and the same two names in a robot's bindings
 *       reach the same port. Nothing here knows a physics engine; bindings
 *       connect ports to whatever simulates the board.
 *
 * @note Not thread safe, and it does not need to be: the firmware and whatever
 *       drives the ports take strict turns.
 */
class Board {
public:
    Board() = delete;

    /**
     * @brief Get the port of a GPIO pin.
     *
     * @param port GPIO port of the pin.
     * @param pin Pin mask.
     * @return The port.
     */
    static GpioPort& gpio(const void* port, uint16_t pin);

    /**
     * @brief Get the port of a PWM channel.
     *
     * @param timer Timer handle.
     * @param channel Timer channel.
     * @return The port.
     */
    static PwmPort& pwm(const void* timer, uint32_t channel);

    /**
     * @brief Get the port of a DMA-fed timer channel.
     *
     * @param timer Timer handle.
     * @param channel Timer channel.
     * @return The port.
     */
    static PwmDmaPort& pwm_dma(const void* timer, uint32_t channel);

    /**
     * @brief Get the port of an ADC.
     *
     * @param adc ADC handle.
     * @return The port.
     */
    static AdcPort& adc(const void* adc);

    /**
     * @brief Get the port of a UART.
     *
     * @param uart UART handle.
     * @return The port.
     */
    static UartPort& uart(const void* uart);

    /**
     * @brief Get the port of a timer in encoder mode.
     *
     * @param timer Timer handle.
     * @return The port.
     */
    static EncoderPort& encoder(const void* timer);

    /**
     * @brief Get a named sample port, for a swapped chip driver.
     *
     * @param name Name both sides agree on.
     * @return The port.
     */
    static SamplePort& samples(std::string_view name);

    /**
     * @brief Get the port of one chip select of an SPI bus.
     *
     * @param spi SPI handle.
     * @param cs_port GPIO port of the chip select.
     * @param cs_pin Pin mask of the chip select.
     * @return The port.
     */
    static SpiPort& spi(const void* spi, const void* cs_port, uint16_t cs_pin);

    /**
     * @brief Attach a device to one chip select of an SPI bus.
     *
     * @note Keyed by the bus and the chip select, since several devices share
     *       a bus. Binds the SPI port and the chip select's GPIO port. The device
     *       is held by reference and must outlive the run.
     *
     * @param spi SPI handle.
     * @param cs_port GPIO port of the chip select.
     * @param cs_pin Pin mask of the chip select.
     * @param device The device.
     */
    static void
        spi_device(const SPI_HandleTypeDef* spi, const GPIO_TypeDef* cs_port, uint16_t cs_pin, SpiDevice& device);

    /**
     * @brief Get the flash.
     *
     * @return The port.
     */
    static FlashPort& flash();

    /**
     * @brief Get the microcontroller core.
     *
     * @return The port.
     */
    static McuPort& mcu();

    /**
     * @brief Name a GPIO pin, as the Cube layer labels it.
     *
     * @param port GPIO port of the pin.
     * @param pin Pin mask.
     * @param name Label.
     */
    static void name_gpio(const void* port, uint16_t pin, std::string_view name);

    /**
     * @brief Name a peripheral handle, as the Cube layer calls it.
     *
     * @note Ports created for the handle later are named after it.
     *
     * @param handle Peripheral handle.
     * @param name Name.
     */
    static void name_handle(const void* handle, std::string_view name);

    /**
     * @brief List the ports the firmware used that nothing is bound to.
     *
     * @return Their names, one per port.
     */
    static std::vector<std::string> unbound();

    /**
     * @brief Forget every port and name.
     *
     * @note Called at the start of every run. References to ports obtained before are dangling after.
     */
    static void reset();
};
}  // namespace micras::hal::host

#endif  // MICRAS_HAL_HOST_BOARD_HPP
