/**
 * @file
 */

#include <cmath>
#include <numbers>

#include "micras/sim/core/noise.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Mix a seed with a name, FNV-1a over the name's bytes.
 *
 * @param seed The run's seed.
 * @param stream Name of the stream.
 * @return The stream's seed.
 */
uint64_t mix(uint64_t seed, std::string_view stream) {
    uint64_t hash = 0xcbf29ce484222325ULL ^ seed;

    for (const char character : stream) {
        hash ^= static_cast<unsigned char>(character);
        hash *= 0x100000001b3ULL;
    }

    return hash;
}
}  // namespace

Noise::Noise(const NoiseConfig& config, std::string_view stream) :
    engine{mix(config.seed, stream)}, on{not config.ideal} { }

double Noise::unit() {
    return (static_cast<double>(this->engine() >> 11U) + 1.0) * 0x1.0p-53;
}

double Noise::gaussian(double sigma) {
    if (not this->on or sigma == 0.0) {
        return 0.0;
    }

    if (this->has_spare) {
        this->has_spare = false;
        return sigma * this->spare;
    }

    const double radius = std::sqrt(-2.0 * std::log(this->unit()));
    const double angle = 2.0 * std::numbers::pi * this->unit();
    this->spare = radius * std::sin(angle);
    this->has_spare = true;
    return sigma * radius * std::cos(angle);
}
}  // namespace micras::sim
