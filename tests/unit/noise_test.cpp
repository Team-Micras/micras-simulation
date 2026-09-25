#include <cmath>
#include <cstddef>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

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

TEST(Noise, RepeatsAStreamForTheSameSeedAndName) {
    EXPECT_EQ(draw({.seed = 3}, "gyro", 100), draw({.seed = 3}, "gyro", 100));
}

TEST(Noise, GivesEachNameItsOwnStream) {
    EXPECT_NE(draw({.seed = 3}, "gyro", 100), draw({.seed = 3}, "accelerometer", 100));
    EXPECT_NE(draw({.seed = 3}, "a", 100), draw({.seed = 3}, "b", 100));
}

TEST(Noise, GivesEachSeedItsOwnStream) {
    EXPECT_NE(draw({.seed = 3}, "gyro", 100), draw({.seed = 4}, "gyro", 100));
}

TEST(Noise, ScalesTheSameSamplesBySigma) {
    const std::vector<double> unit = draw({.seed = 5}, "gyro", 50);
    const std::vector<double> scaled = draw({.seed = 5}, "gyro", 50, 2.5);

    for (std::size_t i = 0; i < unit.size(); i++) {
        EXPECT_DOUBLE_EQ(scaled.at(i), 2.5 * unit.at(i));
    }
}

TEST(Noise, DrawsACentredGaussianOfTheRequestedSpread) {
    const std::vector<double> samples = draw({.seed = 11}, "gyro", 20000, 2.0);
    double                    sum = 0.0;
    double                    squares = 0.0;

    for (const double sample : samples) {
        sum += sample;
        squares += sample * sample;
    }

    const auto   count = static_cast<double>(samples.size());
    const double mean = sum / count;

    EXPECT_NEAR(mean, 0.0, 0.05);
    EXPECT_NEAR(std::sqrt(squares / count - mean * mean), 2.0, 0.05);
}

TEST(Noise, DrawsNothingInAnIdealWorld) {
    Noise noise({.seed = 3, .ideal = true}, "gyro");

    EXPECT_FALSE(noise.enabled());

    for (int i = 0; i < 100; i++) {
        EXPECT_EQ(noise.gaussian(1.0), 0.0);
    }
}

TEST(Noise, DrawsNothingForAZeroSigma) {
    Noise noise({.seed = 3}, "gyro");

    EXPECT_TRUE(noise.enabled());
    EXPECT_EQ(noise.gaussian(0.0), 0.0);
}
}  // namespace
}  // namespace micras::sim
