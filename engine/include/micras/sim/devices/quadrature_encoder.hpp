/**
 * @file
 *
 * @brief An incremental encoder counting a joint's rotation.
 */

#ifndef MICRAS_SIM_DEVICES_QUADRATURE_ENCODER_HPP
#define MICRAS_SIM_DEVICES_QUADRATURE_ENCODER_HPP

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "micras/sim/devices/device.hpp"

namespace micras::sim {
/**
 * @brief Counts edges of a quadrature output, as a timer in encoder mode does.
 *
 * @note The count is the joint angle quantised to the encoder's resolution; a
 *       magnet on the wheel axle turns one to one with the wheel, so there is
 *       no backlash between them.
 */
class QuadratureEncoder : public Device {
public:
    /**
     * @brief What the encoder counts and where the count goes.
     */
    struct Config {
        /**
         * @brief Name of the recorded column.
         */
        std::string name;

        /**
         * @brief Joint of the robot model.
         */
        std::string joint;

        /**
         * @brief Edges per revolution.
         */
        uint32_t counts_per_revolution;

        /**
         * @brief Receives the count after every tick.
         */
        std::function<void(int32_t)> write;
    };

    /**
     * @brief Find the joint in the model.
     *
     * @param world Loaded world.
     * @param config The encoder.
     */
    QuadratureEncoder(const MujocoWorld& world, Config config);

    /**
     * @brief Count the joint's angle.
     *
     * @param world World that just advanced.
     * @param clock Clock of the run.
     */
    void sample(MujocoWorld& world, const Clock& clock) override;

    /**
     * @brief Get the recorded column: the count.
     *
     * @return Column names.
     */
    std::vector<std::string> columns() const override;

    /**
     * @brief Append the count.
     *
     * @param row Row being built.
     */
    void append(std::vector<CsvCell>& row) const override;

private:
    Config  config;
    int     position_address;
    int32_t count{0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_DEVICES_QUADRATURE_ENCODER_HPP
