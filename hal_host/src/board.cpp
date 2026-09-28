/**
 * @file
 */

#include <algorithm>
#include <bit>
#include <format>
#include <map>
#include <tuple>
#include <utility>

#include "micras/hal/host/board.hpp"

namespace micras::hal::host {
namespace {
/**
 * @brief Key of a port: the handle or GPIO port, and the channel or pin.
 */
using Key = std::pair<const void*, uint32_t>;

/**
 * @brief Key of an SPI port: the handle, and the GPIO port and pin of the chip select.
 */
using SpiKey = std::tuple<const void*, const void*, uint16_t>;

/**
 * @brief Every port and name, created on first use.
 */
struct Registry {
    std::map<Key, GpioPort>            gpios;
    std::map<Key, PwmPort>             pwms;
    std::map<Key, PwmDmaPort>          pwm_dmas;
    std::map<const void*, AdcPort>     adcs;
    std::map<const void*, UartPort>    uarts;
    std::map<const void*, EncoderPort> encoders;
    std::map<SpiKey, SpiPort>          spis;
    std::map<Key, std::string>         gpio_names;
    std::map<const void*, std::string> handle_names;
    FlashPort                          flash{{.name = "flash"}, {}};
    McuPort                            mcu{{.name = "mcu"}};
};

/**
 * @brief Get the registry.
 *
 * @return The process-wide registry.
 */
Registry& registry() {
    static Registry instance;
    return instance;
}

/**
 * @brief Name a handle, falling back to its address.
 *
 * @param handle Peripheral handle.
 * @return Its name.
 */
std::string handle_name(const void* handle) {
    const auto found = registry().handle_names.find(handle);
    return found != registry().handle_names.end() ? found->second :
                                                    std::format("handle@{:#x}", std::bit_cast<uintptr_t>(handle));
}

/**
 * @brief Name a GPIO pin, falling back to its port and mask.
 *
 * @param key GPIO port and pin mask.
 * @return Its name.
 */
std::string gpio_name(const Key& key) {
    const auto found = registry().gpio_names.find(key);
    return found != registry().gpio_names.end() ? found->second :
                                                  std::format("{} pin {:#06x}", handle_name(key.first), key.second);
}

/**
 * @brief Find a port, creating it with a name on first use.
 *
 * @param ports Ports of one kind.
 * @param key Key of the port.
 * @param name Makes the name of a new port.
 * @return The port.
 */
template <typename Map, typename Namer>
typename Map::mapped_type& find_or_create(Map& ports, const typename Map::key_type& key, const Namer& name) {
    auto found = ports.find(key);

    if (found == ports.end()) {
        found = ports.emplace(key, typename Map::mapped_type{}).first;
        found->second.name = name();
    }

    return found->second;
}
}  // namespace

GpioPort& Board::gpio(const void* port, uint16_t pin) {
    const Key key{port, pin};

    return find_or_create(registry().gpios, key, [&key] { return gpio_name(key); });
}

PwmPort& Board::pwm(const void* timer, uint32_t channel) {
    return find_or_create(registry().pwms, Key{timer, channel}, [timer, channel] {
        return std::format("{} channel {}", handle_name(timer), channel / 4 + 1);
    });
}

PwmDmaPort& Board::pwm_dma(const void* timer, uint32_t channel) {
    return find_or_create(registry().pwm_dmas, Key{timer, channel}, [timer, channel] {
        return std::format("{} channel {} DMA", handle_name(timer), channel / 4 + 1);
    });
}

AdcPort& Board::adc(const void* adc) {
    return find_or_create(registry().adcs, adc, [adc] { return handle_name(adc); });
}

UartPort& Board::uart(const void* uart) {
    return find_or_create(registry().uarts, uart, [uart] { return handle_name(uart); });
}

EncoderPort& Board::encoder(const void* timer) {
    return find_or_create(registry().encoders, timer, [timer] { return handle_name(timer) + " encoder"; });
}

SpiPort& Board::spi(const void* spi, const void* cs_port, uint16_t cs_pin) {
    return find_or_create(registry().spis, SpiKey{spi, cs_port, cs_pin}, [spi, cs_port, cs_pin] {
        return std::format("{} {}", handle_name(spi), gpio_name(Key{cs_port, cs_pin}));
    });
}

void Board::spi_device(const SPI_HandleTypeDef* spi, const GPIO_TypeDef* cs_port, uint16_t cs_pin, SpiDevice& device) {
    SpiPort& port = Board::spi(spi, cs_port, cs_pin);
    port.device = &device;
    port.bound = true;
    Board::gpio(cs_port, cs_pin).bound = true;
}

FlashPort& Board::flash() {
    return registry().flash;
}

McuPort& Board::mcu() {
    return registry().mcu;
}

void Board::name_gpio(const void* port, uint16_t pin, std::string_view name) {
    const Key key{port, pin};
    registry().gpio_names[key] = std::string{name};

    const auto found = registry().gpios.find(key);

    if (found != registry().gpios.end()) {
        found->second.name = std::string{name};
    }
}

void Board::name_handle(const void* handle, std::string_view name) {
    registry().handle_names[handle] = std::string{name};
}

std::vector<std::string> Board::unbound() {
    std::vector<std::string> names;

    const auto collect = [&names](const auto& ports) {
        for (const auto& [key, port] : ports) {
            if (port.touched and not port.bound) {
                names.push_back(port.name);
            }
        }
    };

    collect(registry().gpios);
    collect(registry().pwms);
    collect(registry().pwm_dmas);
    collect(registry().adcs);
    collect(registry().uarts);
    collect(registry().encoders);
    collect(registry().spis);

    return names;
}

void Board::reset() {
    registry() = Registry{};
}
}  // namespace micras::hal::host
