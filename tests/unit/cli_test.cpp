#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include <doctest/doctest.h>

#include "micras/sim/app/cli.hpp"
#include "micras/sim/app/wiring.hpp"

namespace micras::sim {
/**
 * @brief Parse a command line written the way a shell would pass it.
 *
 * @param words Arguments after the program name.
 * @param target_options Options the robot target adds.
 * @return Parsed options.
 */
static CliOptions parse(std::vector<std::string> words, std::span<const CliOption> target_options = {}) {
    std::vector<std::string> owned{"robot_sim"};
    owned.insert(owned.end(), words.begin(), words.end());

    std::vector<char*> argv;
    argv.reserve(owned.size());

    for (std::string& word : owned) {
        argv.push_back(word.data());
    }

    return Cli::parse(std::span(argv), target_options);
}

namespace {
TEST_CASE("Cli.ReadsTheRequiredOptions") {
    const CliOptions options = parse({"--out", "runs/here"});

    CHECK_EQ(options.out, "runs/here");
    CHECK(options.scenario.empty());
    CHECK_FALSE(options.seconds.has_value());
    CHECK_FALSE(options.seed.has_value());
    CHECK_FALSE(options.ideal);
    CHECK_EQ(options.record_every, 1U);
    CHECK(options.video.path.empty());
    CHECK(options.video.camera.empty());
    CHECK_FALSE(options.viewer_enabled);
}

TEST_CASE("Cli.ReadsWhatOverridesTheScenario") {
    const CliOptions options = parse(
        {"--out", "o", "--scenario", "explore.toml", "--maze", "apec2018", "--seconds", "4", "--seed", "7", "--ideal",
         "--record-every", "8"}
    );

    CHECK_EQ(options.scenario, "explore.toml");
    CHECK_EQ(options.maze, "apec2018");
    CHECK_EQ(options.seconds, 4.0);
    CHECK_EQ(options.seed, 7U);
    CHECK(options.ideal);
    CHECK_EQ(options.record_every, 8U);
    CHECK_EQ(parse({"--out", "o", "--ticks", "12"}).ticks, 12U);
}

TEST_CASE("Cli.ReadsTheRecordingOptions") {
    const CliOptions options = parse(
        {"--out", "o", "--video", "run.mp4", "--video-fps", "60", "--video-camera", "overhead", "--video-size",
         "640x480", "--video-trail"}
    );

    CHECK(options.video.trail);
    CHECK_EQ(options.video.path, "run.mp4");
    CHECK_EQ(options.video.fps, 60);
    CHECK_EQ(options.video.camera, "overhead");
    CHECK_EQ(options.video.width, 640);
    CHECK_EQ(options.video.height, 480);
}

TEST_CASE("Cli.ReadsTheWindowOptions") {
    const CliOptions options = parse(
        {"--out", "o", "--viewer", "--viewer-camera", "overhead", "--viewer-size", "800x600", "--viewer-fps", "15"}
    );

    CHECK(options.viewer_enabled);
    CHECK_EQ(options.viewer.camera, "overhead");
    CHECK_EQ(options.viewer.width, 800);
    CHECK_EQ(options.viewer.height, 600);
    CHECK_EQ(options.viewer_fps, 15);
}

TEST_CASE("Cli.RejectsAnIncompleteCommandLine") {
    CHECK_THROWS_AS(parse({"--seconds", "1"}), std::runtime_error);
    CHECK_THROWS_AS(parse({"--out"}), std::runtime_error);
    CHECK_THROWS_AS(parse({"--out", "o", "--nonsense"}), std::runtime_error);
}

TEST_CASE("Cli.RejectsValuesTheRunCouldNotHonour") {
    CHECK_THROWS_AS(parse({"--out", "o", "--seconds", "0"}), std::runtime_error);
    CHECK_THROWS_AS(parse({"--out", "o", "--ticks", "0"}), std::runtime_error);
    CHECK_THROWS_AS(parse({"--out", "o", "--record-every", "0"}), std::runtime_error);
    CHECK_THROWS_AS(parse({"--out", "o", "--video", "v.mp4", "--video-fps", "0"}), std::runtime_error);
    CHECK_THROWS_AS(parse({"--out", "o", "--video", "v.mp4", "--video-size", "big"}), std::runtime_error);
    CHECK_THROWS_AS(parse({"--out", "o", "--viewer", "--viewer-size", "0x0"}), std::runtime_error);
    CHECK_THROWS_AS(parse({"--out", "o", "--viewer", "--viewer-fps", "0"}), std::runtime_error);
    CHECK_THROWS_AS(parse({"--out", "o", "--viewer", "--viewer-size", "wide"}), std::runtime_error);
}

TEST_CASE("Cli.HandsTargetOptionsToTheirHandlers") {
    std::string                  color;
    bool                         flag = false;
    const std::vector<CliOption> target_options{
        {.name = "--color", .argument = "<name>", .apply = [&color](const std::string& value) { color = value; }},
        {.name = "--flag", .argument = "", .apply = [&flag](const std::string&) { flag = true; }},
    };

    const CliOptions options = parse({"--color", "red", "--maze", "m", "--flag", "--out", "o"}, target_options);

    CHECK_EQ(color, "red");
    CHECK(flag);
    CHECK_EQ(options.maze, "m");
}

TEST_CASE("Cli.UsageNamesEveryOptionItAccepts") {
    const std::vector<CliOption> target_options{{.name = "--color", .argument = "<name>", .apply = {}}};
    const std::string            usage = Cli::usage("robot_sim", target_options);

    CHECK(usage.starts_with("usage: robot_sim "));

    for (const std::string option :
         {"--out",          "--scenario",    "--maze",         "--seconds",      "--ticks",
          "--seed",         "--ideal",       "--record-every", "--video",        "--video-fps",
          "--video-camera", "--video-size",  "--video-trail",  "--viewer",       "--viewer-camera",
          "--viewer-fps",   "--viewer-size", "--monitor",      "--monitor-port", "--color <name>"}) {
        CHECK_MESSAGE(usage.find(option) != std::string::npos, option);
    }
}
}  // namespace
}  // namespace micras::sim
