/**
 * @file
 *
 * @brief What a robot target shows on the panel and the video overlay, as data.
 */

#ifndef MICRAS_SIM_VIEW_PANEL_SPEC_HPP
#define MICRAS_SIM_VIEW_PANEL_SPEC_HPP

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace micras::sim {
/**
 * @brief A color the panel draws.
 */
struct Color {
    uint8_t red{0};
    uint8_t green{0};
    uint8_t blue{0};
};

/**
 * @brief A momentary button: held while the mouse holds it.
 */
struct PanelButton {
    /**
     * @brief Label.
     */
    std::string name;

    /**
     * @brief Called with true when the button goes down and false when it comes up.
     */
    std::function<void(bool pressed)> press;
};

/**
 * @brief A switch that stays where it is put.
 */
struct PanelSwitch {
    /**
     * @brief Label.
     */
    std::string name;

    /**
     * @brief Current position, read every frame.
     */
    std::function<bool()> state;

    /**
     * @brief Called when a human flips it.
     */
    std::function<void(bool on)> set;
};

/**
 * @brief A light the robot drives.
 */
struct PanelLamp {
    /**
     * @brief Label.
     */
    std::string name;

    /**
     * @brief Current color, read every frame.
     */
    std::function<Color()> color;
};

/**
 * @brief A line of text the robot's state produces.
 */
struct PanelReadout {
    /**
     * @brief Label.
     */
    std::string name;

    /**
     * @brief Current text, read every frame.
     */
    std::function<std::string()> text;
};

/**
 * @brief A firmware variable holding a state id, shown by name.
 */
struct StateLabel {
    /**
     * @brief Firmware name of the variable.
     */
    std::string variable;

    /**
     * @brief Name of each state id, starting at zero.
     */
    std::vector<std::string> names;

    /**
     * @brief Name a state id.
     *
     * @note The negated comparison is the NaN guard: a variable not reported
     *       yet reads as NaN, and every comparison with NaN is false.
     *
     * @param state State id as read from the variable.
     * @return Its name, or "?" for NaN and for ids the table does not know.
     */
    std::string name_of(double state) const;
};

/**
 * @brief Everything a robot target puts on the control panel.
 *
 * @note Controls are declared, not drawn: the view draws every robot the same
 *       way and owns the details that are easy to get wrong, such as reacting to
 *       press and release edges rather than to the held state.
 */
struct PanelSpec {
    /**
     * @brief State shown under the tick, if any.
     */
    std::optional<StateLabel> state;

    /**
     * @brief Momentary buttons, in display order.
     */
    std::vector<PanelButton> buttons;

    /**
     * @brief Switches, in display order.
     */
    std::vector<PanelSwitch> switches;

    /**
     * @brief Lights, in display order.
     */
    std::vector<PanelLamp> lamps;

    /**
     * @brief Text readouts, in display order.
     */
    std::vector<PanelReadout> readouts;

    /**
     * @brief Firmware variables plotted against time, one plot each.
     */
    std::vector<std::string> plots;

    /**
     * @brief Called once, on the first button or switch a human touches.
     *
     * @note Set by the application, not by the robot target: a scripted run hands
     *       the board over at that moment, so the script and the human never
     *       fight over the same input.
     */
    std::function<void()> take_over;
};

/**
 * @brief One line of the video overlay: a firmware variable with its unit.
 */
struct OverlayLine {
    /**
     * @brief Label.
     */
    std::string label;

    /**
     * @brief Firmware name of the variable.
     */
    std::string variable;

    /**
     * @brief Unit printed after the value.
     */
    std::string unit;
};

/**
 * @brief What the video overlay prints under the simulated time.
 */
struct OverlaySpec {
    /**
     * @brief State printed by name, if any.
     */
    std::optional<StateLabel> state;

    /**
     * @brief Variables printed with three decimals.
     */
    std::vector<OverlayLine> lines;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_VIEW_PANEL_SPEC_HPP
