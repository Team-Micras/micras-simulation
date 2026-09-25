/**
 * @file
 *
 * @brief Seeded, reproducible noise, one independent stream per device.
 */

#ifndef MICRAS_SIM_CORE_NOISE_HPP
#define MICRAS_SIM_CORE_NOISE_HPP

#include <cstdint>
#include <random>
#include <string_view>

namespace micras::sim {
/**
 * @brief The run's noise settings: a seed, and whether noise is on at all.
 */
struct NoiseConfig {
    /**
     * @brief Seed every stream derives from; recorded in meta.json.
     */
    uint64_t seed{1};

    /**
     * @brief Whether the world is ideal: no noise, no bias, no scale error.
     */
    bool ideal{false};
};

/**
 * @brief A stream of Gaussian and uniform samples for one device.
 *
 * @note The generator is std::mt19937_64, whose sequence the standard fixes, and
 *       the Gaussian comes from a Box-Muller transform written here, because the
 *       standard distributions are free to differ between library versions. Each
 *       stream's seed mixes the run's seed with the stream's name, so adding a
 *       device never changes the noise another device draws.
 */
class Noise {
public:
    /**
     * @brief Open a stream.
     *
     * @param config The run's noise settings.
     * @param stream Name of the stream, usually the device's.
     */
    Noise(const NoiseConfig& config, std::string_view stream);

    /**
     * @brief Draw from a centred Gaussian.
     *
     * @param sigma Standard deviation.
     * @return A sample, or 0 when the world is ideal.
     */
    double gaussian(double sigma);

    /**
     * @brief Check whether this stream draws anything.
     *
     * @return False when the world is ideal.
     */
    bool enabled() const { return this->on; }

private:
    /**
     * @brief Draw from the unit interval, excluding zero.
     *
     * @return A sample in (0, 1].
     */
    double unit();

    std::mt19937_64 engine;
    bool            on;
    bool            has_spare{false};
    double          spare{0.0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_NOISE_HPP
