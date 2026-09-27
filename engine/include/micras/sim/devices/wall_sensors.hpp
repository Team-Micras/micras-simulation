/**
 * @file
 *
 * @brief Infrared emitter and receiver pairs ranging walls by reflected light.
 */

#ifndef MICRAS_SIM_DEVICES_WALL_SENSORS_HPP
#define MICRAS_SIM_DEVICES_WALL_SENSORS_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "micras/sim/core/noise.hpp"
#include "micras/sim/devices/device.hpp"
#include "micras/sim/robot/robot_description.hpp"

namespace micras::sim {
/**
 * @brief The ADC conversions of a set of wall sensors, lit and dark.
 *
 * @note The emitters fire in groups: every scan period one group is lit and every
 *       receiver is converted, the lit group's own receivers reading their
 *       emitter and the others reading whatever the lit group throws their way,
 *       which is the cross-talk. There are two groups, so the other group's scan
 *       is each sensor's dark reading. The sequence the firmware sees is one scan
 *       per group, in group order, and completes after the second one.
 *
 * @note Light model: each emitter is a cone of rays whose intensity falls off as
 *       2^-(angle / half angle)^2 around its axis, down to 2.5 half angles. Each
 *       ray that hits the arena reflects as a Lambertian spot of the surface's
 *       reflectance; the receiver collects it by the inverse square of its
 *       distance, the cosine of the spot's emission angle and the receiver's own
 *       angular response. The photocurrent is the responsivity times that
 *       irradiance, plus ambient light, and becomes a voltage across the load
 *       resistor, clipped at saturation. The emitter's contribution reaches only
 *       the settled fraction of its value by the time the conversion starts.
 */
class WallSensors : public Device {
public:
    /**
     * @brief The sensors and where their conversions go.
     */
    struct Config {
        /**
         * @brief Prefix of the recorded columns.
         */
        std::string name;

        /**
         * @brief Optics, electronics and poses; the robot model has an
         *        "<name>_emitter" and an "<name>_receiver" site per sensor.
         */
        WallSensorsDescription description;

        /**
         * @brief Ticks between two scans.
         */
        uint32_t scan_ticks;

        /**
         * @brief Duty cycle of a sensor's emitter, in percent; an emitter only lights when it is above zero.
         */
        std::function<float(std::size_t)> emitter_duty;

        /**
         * @brief Receives one conversion: its index in the sequence and its counts.
         */
        std::function<void(std::size_t, uint32_t)> write;

        /**
         * @brief Called when the sequence completes.
         */
        std::function<void()> finish_sequence;

        /**
         * @brief Reflectance of an arena geom, by id.
         *
         * @note The rays pass through the geoms of MujocoWorld::unseen_group.
         */
        std::function<double(int)> reflectance;
    };

    /**
     * @brief Find the sites and lay out the ray cone.
     *
     * @note Throws unless the sensors fall in exactly two groups.
     *
     * @param world Loaded world.
     * @param config The sensors.
     * @param noise The run's noise settings.
     */
    WallSensors(const MujocoWorld& world, Config config, const NoiseConfig& noise);

    /**
     * @brief Convert every receiver with the next group lit, on the ticks a scan falls on.
     *
     * @param world World that just advanced.
     * @param clock Clock of the run.
     */
    void sample(MujocoWorld& world, const Clock& clock) override;

    /**
     * @brief Get the recorded columns: each sensor's lit minus dark reading, as the firmware normalises it.
     *
     * @return Column names.
     */
    std::vector<std::string> columns() const override;

    /**
     * @brief Append each sensor's last lit minus dark reading.
     *
     * @param row Row being built.
     */
    void append(std::vector<CsvCell>& row) const override;

private:
    /**
     * @brief One ray of an emitter's cone, in the emitter's frame.
     */
    struct Ray {
        std::array<double, 3> direction;
        double                flux;
    };

    /**
     * @brief Farthest distance a ray looks for a surface, in metres.
     */
    static constexpr double cutoff{0.5};

    /**
     * @brief Compute the irradiance one emitter puts on one receiver.
     *
     * @param world The world.
     * @param emitter Index of the emitting sensor.
     * @param receiver Index of the receiving sensor.
     * @return Irradiance in W/m^2.
     */
    double irradiance(MujocoWorld& world, std::size_t emitter, std::size_t receiver);

    Config                 config;
    Noise                  noise;
    std::vector<int>       emitter_sites;
    std::vector<int>       receiver_sites;
    std::vector<Ray>       rays;
    std::array<int, 2>     groups{};
    std::vector<uint32_t>  counts;
    std::vector<double>    intensities;
    std::array<uint8_t, 6> geom_groups{};
    std::vector<mjtNum>    directions;
    std::vector<mjtNum>    distances;
    std::vector<mjtNum>    normals;
    std::vector<int>       geoms;
    uint64_t               scans{0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_DEVICES_WALL_SENSORS_HPP
