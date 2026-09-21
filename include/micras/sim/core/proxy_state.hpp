/**
 * @file
 *
 * @brief Everything that crosses the proxy boundary, as a second source of truth.
 */

#ifndef MICRAS_SIM_CORE_PROXY_STATE_HPP
#define MICRAS_SIM_CORE_PROXY_STATE_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace micras::sim {
/**
 * @brief Human controls the proxies read, written by the scenario or the GUI.
 *
 * @note Same vocabulary as micras::Interface: the DIP switches are indexed as
 *       in Interface::DipSwitchPins (fan, diagonal, boost, risky).
 */
struct InterfaceInput {
    /**
     * @brief Number of DIP switches on the board.
     */
    static constexpr std::size_t dip_switch_count{4};

    /**
     * @brief True while the button is held down.
     */
    bool button_pressed{false};

    /**
     * @brief State of each DIP switch.
     */
    std::array<bool, dip_switch_count> dip_switches{};

    /**
     * @brief Whether a human has taken the board over from the script.
     *
     * @note Set by the panel on the first control it is given, and never
     *       cleared: from that moment the scenario stops writing, so the two
     *       never fight over the same field and the run is marked interactive.
     */
    bool driven_by_human{false};
};

/**
 * @brief One addressable LED, as the firmware set it.
 */
struct Rgb {
    uint8_t red{0};
    uint8_t green{0};
    uint8_t blue{0};

    bool operator==(const Rgb&) const = default;
};

/**
 * @brief What the board is showing, written by the proxies a human watches.
 *
 * @note Same vocabulary as micras::Interface: this is the other half of
 *       InterfaceInput, the side the firmware drives rather than reads.
 */
struct InterfaceOutput {
    /**
     * @brief Number of addressable LEDs on the board.
     */
    static constexpr std::size_t argb_count{2};

    /**
     * @brief Whether the plain LED is lit.
     */
    bool led_on{false};

    /**
     * @brief Colour of each addressable LED.
     */
    std::array<Rgb, argb_count> argb{};

    /**
     * @brief Buzzer tone in hertz, zero while it is silent.
     */
    uint32_t buzzer_frequency{0};
};

/**
 * @brief Harness switches that sit behind the firmware's back.
 *
 * @note Not part of the board: these are simulation-only overrides, kept apart
 *       from InterfaceInput so that section keeps matching micras::Interface.
 */
struct Overrides {
    /**
     * @brief Whether the Fan proxy is allowed to drive its actuator.
     */
    bool fan_enabled{true};
};

/**
 * @brief Commands the firmware handed to the actuator proxies.
 *
 * @note Unlike the sensor block, these are aligned with the ground truth of the
 *       same CSV row: the firmware writes them during the tick whose physics
 *       then consumes them.
 */
struct Actuators {
    /**
     * @brief Wheel commands as Locomotion passed them to the motors, in percent.
     */
    ///@{
    float left_command{0.0F};
    float right_command{0.0F};
    ///@}

    /**
     * @brief Speed the Fan proxy last settled on, in percent.
     */
    float fan_speed{0.0F};
};

/**
 * @brief Readings the sensor proxies handed to the firmware.
 *
 * @note These duplicate the ground truth on purpose: a disagreement between
 *       what the robot physically did and what its sensors reported is the
 *       signal.
 *
 * @note They are one tick behind the ground truth of the same CSV row. The
 *       firmware body runs before the physics of its tick, so the values it
 *       read describe the world at the end of the previous tick, exactly as a
 *       real robot acts on the reading it took a loop ago. Comparing a sensor
 *       column against the ground truth of the same row measures that lag, not
 *       a sensor error; shift one row first.
 */
struct Sensors {
    /**
     * @brief Number of wheels, wall sensors and inertial axes.
     */
    ///@{
    static constexpr std::size_t wheel_count{2};
    static constexpr std::size_t wall_sensor_count{4};
    static constexpr std::size_t axis_count{3};
    ///@}

    /**
     * @brief Encoder positions, left then right, in radians.
     */
    std::array<float, wheel_count> encoder_positions{};

    /**
     * @brief Gyroscope reading per axis, in radians per second.
     */
    std::array<float, axis_count> angular_velocity{};

    /**
     * @brief Accelerometer reading per axis, in metres per second squared.
     */
    std::array<float, axis_count> linear_acceleration{};

    /**
     * @brief Unfiltered wall sensor readings, the ones post detection uses.
     */
    std::array<float, wall_sensor_count> wall_adc_readings{};
};

/**
 * @brief State the proxies expose to the harness, one writer per section.
 *
 * @note This is the second source of truth, independent of the firmware
 *       telemetry: it records what actually crossed the proxy boundary.
 */
struct ProxyState {
    /**
     * @brief Written by the scenario or the GUI, read by the Button and DipSwitch proxies.
     */
    InterfaceInput interface_input;

    /**
     * @brief Written by the Led, Argb and Buzzer proxies, read by the panel and the recorder.
     */
    InterfaceOutput interface_output;

    /**
     * @brief Written by the command line, read by the proxies it silences.
     */
    Overrides overrides;

    /**
     * @brief Written by the Locomotion and Fan proxies, read by the recorder.
     */
    Actuators actuators;

    /**
     * @brief Written by the sensor proxies, read by the recorder.
     */
    Sensors sensors;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_PROXY_STATE_HPP
