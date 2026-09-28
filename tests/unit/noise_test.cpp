#include <cmath>
#include <cstddef>
#include <string_view>
#include <vector>

#include <doctest/doctest.h>

#include "micras/sim/core/noise.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Draw a number of samples from a fresh stream.
 *
 * @param config The run's noise settings.
 * @param stream Name of the stream.
 * @param count Number of samples.
 * @param sigma Standard deviation of every sample.
 * @return The samples.
 */
std::vector<double> draw(const NoiseConfig& config, std::string_view stream, std::size_t count, double sigma = 1.0) {
    Noise               noise(config, stream);
    std::vector<double> samples;
    samples.reserve(count);

    for (std::size_t i = 0; i < count; i++) {
        samples.push_back(noise.gaussian(sigma));
    }

    return samples;
}

TEST_CASE("Noise.RepeatsAStreamForTheSameSeedAndName") {
    CHECK_EQ(draw({.seed = 3}, "gyro", 100), draw({.seed = 3}, "gyro", 100));
}

TEST_CASE("Noise.GivesEachNameItsOwnStream") {
    CHECK_NE(draw({.seed = 3}, "gyro", 100), draw({.seed = 3}, "accelerometer", 100));
    CHECK_NE(draw({.seed = 3}, "a", 100), draw({.seed = 3}, "b", 100));
}

TEST_CASE("Noise.GivesEachSeedItsOwnStream") {
    CHECK_NE(draw({.seed = 3}, "gyro", 100), draw({.seed = 4}, "gyro", 100));
}

TEST_CASE("Noise.ScalesTheSameSamplesBySigma") {
    const std::vector<double> unit = draw({.seed = 5}, "gyro", 50);
    const std::vector<double> scaled = draw({.seed = 5}, "gyro", 50, 2.5);

    for (std::size_t i = 0; i < unit.size(); i++) {
        CHECK_EQ(scaled.at(i), doctest::Approx(2.5 * unit.at(i)).epsilon(1e-12));
    }
}

TEST_CASE("Noise.DrawsACentredGaussianOfTheRequestedSpread") {
    const std::vector<double> samples = draw({.seed = 11}, "gyro", 20000, 2.0);
    double                    sum = 0.0;
    double                    squares = 0.0;

    for (const double sample : samples) {
        sum += sample;
        squares += sample * sample;
    }

    const auto   count = static_cast<double>(samples.size());
    const double mean = sum / count;

    CHECK_LE(std::abs(mean - 0.0), 0.05);
    CHECK_LE(std::abs(std::sqrt(squares / count - mean * mean) - 2.0), 0.05);
}

TEST_CASE("Noise.DrawsNothingInAnIdealWorld") {
    Noise noise({.seed = 3, .ideal = true}, "gyro");

    for (int i = 0; i < 100; i++) {
        CHECK_EQ(noise.gaussian(1.0), 0.0);
    }
}

TEST_CASE("Noise.DrawsNothingForAZeroSigma") {
    Noise noise({.seed = 3}, "gyro");

    CHECK_EQ(noise.gaussian(0.0), 0.0);
}
}  // namespace
}  // namespace micras::sim
