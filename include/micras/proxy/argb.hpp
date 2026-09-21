/**
 * @file
 */

#ifndef MICRAS_PROXY_ARGB_HPP
#define MICRAS_PROXY_ARGB_HPP

#include <array>
#include <cstdint>
#include <string>

#include "micras/sim/core/proxy_state.hpp"

namespace micras::sim {
struct SimulationContext;
}  // namespace micras::sim

namespace micras::proxy {
/**
 * @brief Stub addressable RGB LED strip that only records its last colors.
 */
template <uint8_t num_of_leds>
class TArgb {
public:
    /**
     * @brief Struct for storing color information.
     */
    struct Color {
        uint8_t red;
        uint8_t green;
        uint8_t blue;

        Color operator*(float brightness) const {
            return {
                static_cast<uint8_t>(this->red * brightness),
                static_cast<uint8_t>(this->green * brightness),
                static_cast<uint8_t>(this->blue * brightness),
            };
        }
    };

    /**
     * @brief Configuration struct for the addressable RGB LED.
     */
    struct Config {
        std::array<std::string, num_of_leds> names;

        /**
         * @brief Simulation holding the state this strip is shown in.
         */
        sim::SimulationContext* context{nullptr};
    };

    /**
     * @brief Predefined colors.
     */
    struct Colors {
        Colors() = delete;

        static constexpr Color red{255, 0, 0};
        static constexpr Color green{0, 255, 0};
        static constexpr Color blue{0, 0, 255};
        static constexpr Color yellow{255, 255, 0};
        static constexpr Color cyan{0, 255, 255};
        static constexpr Color magenta{255, 0, 255};
        static constexpr Color white{255, 255, 255};
    };

    /**
     * @brief Construct a new Argb object.
     *
     * @param config Configuration for the addressable RGB LED.
     */
    explicit TArgb(const Config& config);

    /**
     * @brief Set the color of the ARGB at the specified index.
     *
     * @param color The color to set the ARGB to.
     * @param index The index of the ARGB to set the color of.
     */
    void set_color(const Color& color, uint8_t index);

    /**
     * @brief Set the color of all ARGBs.
     *
     * @param color The color to set all ARGBs to.
     */
    void set_color(const Color& color);

    /**
     * @brief Set the colors of all ARGBs.
     *
     * @param colors The colors to set the ARGBs to.
     */
    void set_colors(const std::array<Color, num_of_leds>& colors);

    /**
     * @brief Turn off the ARGB at the specified index.
     *
     * @param index The index of the ARGB to turn off.
     */
    void turn_off(uint8_t index);

    /**
     * @brief Turn off all ARGBs.
     */
    void turn_off();

    /**
     * @brief Send the colors to the addressable RGB LED.
     */
    void update();

    /**
     * @brief Get the last color set on an ARGB.
     *
     * @param index The index of the ARGB.
     * @return Last color set on the ARGB.
     */
    Color get_color(uint8_t index) const;

private:
    /**
     * @brief Publish the current colours to the boundary record.
     */
    void publish() const;

    /**
     * @brief Boundary record this proxy writes its colours into.
     *
     * @note Bound for the life of the proxy; a pointer would only add a null
     *       check to every access.
     */
    sim::ProxyState& state;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Last colors set on the ARGBs.
     */
    std::array<Color, num_of_leds> colors{};
};
}  // namespace micras::proxy

#include "../src/proxy/argb.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_ARGB_HPP
