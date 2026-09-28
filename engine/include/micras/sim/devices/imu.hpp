/**
 * @file
 *
 * @brief A MEMS inertial measurement unit, with its datasheet imperfections.
 */

#ifndef MICRAS_SIM_DEVICES_IMU_HPP
#define MICRAS_SIM_DEVICES_IMU_HPP

#include <array>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include "micras/sim/core/noise.hpp"
#include "micras/sim/devices/device.hpp"
#include "micras/sim/robot/robot_description.hpp"

namespace micras::sim {
/**
 * @brief Produces the chip's raw gyroscope and accelerometer words.
 *
 * @note The truth comes from MuJoCo's gyro and accelerometer sensors on a site
 *       whose frame is the chip's, so the axes are the chip's. Each output
 *       sample is that truth low pass filtered at the configured bandwidth,
 *       the gyroscope's scaled by a turn-on scale error, offset by a turn-on
 *       bias, with white noise at the datasheet density over the filter's noise
 *       bandwidth, then quantized to the configured resolution and delivered
 *       one sample late, as the chip's pipeline does. Samples come at the output rate times a
 *       small seeded clock error, so the firmware sometimes finds no new sample
 *       and sometimes finds one skipped, as with the real chip's own oscillator.
 *
 * @note MuJoCo computes sensors at the start of a step, so the truth lags the
 *       end of the tick by one physics step.
 */
class Imu : public Device {
public:
    /**
     * @brief The chip and where its samples go.
     */
    struct Config {
        /**
         * @brief Prefix of the recorded columns.
         */
        std::string name;

        /**
         * @brief Gyro and accelerometer sensors of the robot model.
         */
        ///@{
        std::string gyro;
        std::string accelerometer;
        ///@}

        /**
         * @brief Datasheet and configuration values.
         */
        ImuDescription description;

        /**
         * @brief Receives each sample: gyroscope x, y, z then accelerometer x, y, z, in raw words.
         */
        std::function<void(std::span<const float>)> write;
    };

    /**
     * @brief Find the sensors and draw the turn-on errors.
     *
     * @param world Loaded world.
     * @param config The chip.
     * @param noise The run's noise settings.
     */
    Imu(const MujocoWorld& world, Config config, const NoiseConfig& noise);

    /**
     * @brief Filter the truth and deliver the samples that fell in this tick.
     *
     * @param world World that just advanced.
     * @param clock Clock of the run.
     */
    void sample(MujocoWorld& world, const Clock& clock) override;

    /**
     * @brief Get the recorded columns: the six readings in SI units.
     *
     * @return Column names.
     */
    std::vector<std::string> columns() const override;

    /**
     * @brief Append the last delivered sample.
     *
     * @param row Row being built.
     */
    void append(std::vector<CsvCell>& row) const override;

private:
    /**
     * @brief Values per sample.
     */
    static constexpr std::size_t channels{6};

    Config                       config;
    Noise                        noise;
    int                          gyro_address;
    int                          accelerometer_address;
    std::array<double, channels> filtered{};
    std::array<double, channels> bias{};
    std::array<double, channels> scale{};
    std::array<float, channels>  pending{};
    std::array<float, channels>  delivered{};
    double                       rate_error{0.0};
    double                       phase{0.0};
    bool                         has_pending{false};
    bool                         started{false};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_DEVICES_IMU_HPP
