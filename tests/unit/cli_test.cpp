#include <array>
#include <span>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "micras/sim/app/cli.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Parse a command line written the way a shell would pass it.
 *
 * @param words Arguments after the program name.
 * @return Parsed options.
 */
CliOptions parse(std::vector<std::string> words) {
    std::vector<std::string> owned{"robot_sim"};
    owned.insert(owned.end(), words.begin(), words.end());

    std::vector<char*> argv;
    argv.reserve(owned.size());

    for (std::string& word : owned) {
        argv.push_back(word.data());
    }

    return Cli::parse(std::span(argv));
}

TEST(Cli, ReadsTheRequiredOptions) {
    const CliOptions options = parse({"--out", "runs/here"});

    EXPECT_EQ(options.out, "runs/here");
    EXPECT_TRUE(options.scenario.empty());
    EXPECT_FALSE(options.seconds.has_value());
    EXPECT_FALSE(options.seed.has_value());
    EXPECT_FALSE(options.ideal);
    EXPECT_EQ(options.record_every, 1U);
    EXPECT_FALSE(options.video_enabled);
    EXPECT_FALSE(options.video_camera_given);
}

TEST(Cli, ReadsWhatOverridesTheScenario) {
    const CliOptions options = parse(
        {"--out", "o", "--scenario", "explore.toml", "--maze", "apec2018", "--seconds", "4", "--seed", "7", "--ideal",
         "--record-every", "8"}
    );

    EXPECT_EQ(options.scenario, "explore.toml");
    EXPECT_EQ(options.maze, "apec2018");
    EXPECT_EQ(options.seconds, 4.0);
    EXPECT_EQ(options.seed, 7U);
    EXPECT_TRUE(options.ideal);
    EXPECT_EQ(options.record_every, 8U);
    EXPECT_EQ(parse({"--out", "o", "--ticks", "12"}).ticks, 12U);
}

TEST(Cli, ReadsTheRecordingOptions) {
    const CliOptions options = parse(
        {"--out", "o", "--video", "run.mp4", "--video-fps", "60", "--video-camera", "overhead", "--video-size",
         "640x480"}
    );

    EXPECT_TRUE(options.video_enabled);
    EXPECT_TRUE(options.video_camera_given);
    EXPECT_EQ(options.video.path, "run.mp4");
    EXPECT_EQ(options.video.fps, 60);
    EXPECT_EQ(options.video.camera, "overhead");
    EXPECT_EQ(options.video.width, 640);
    EXPECT_EQ(options.video.height, 480);
}

TEST(Cli, ReadsTheWindowOptions) {
    const CliOptions off = parse({"--out", "o"});
    EXPECT_FALSE(off.viewer_enabled);

    const CliOptions options = parse(
        {"--out", "o", "--viewer", "--viewer-camera", "overhead", "--viewer-size", "800x600", "--viewer-fps", "15"}
    );

    EXPECT_TRUE(options.viewer_enabled);
    EXPECT_EQ(options.viewer.camera, "overhead");
    EXPECT_EQ(options.viewer.width, 800);
    EXPECT_EQ(options.viewer.height, 600);
    EXPECT_EQ(options.viewer_fps, 15);
}

TEST(Cli, RejectsAnIncompleteCommandLine) {
    EXPECT_THROW(parse({"--seconds", "1"}), std::runtime_error);
    EXPECT_THROW(parse({"--out", "o", "--model", "m.xml"}), std::runtime_error);
    EXPECT_THROW(parse({"--out"}), std::runtime_error);
    EXPECT_THROW(parse({"--out", "o", "--nonsense"}), std::runtime_error);
}

TEST(Cli, RejectsValuesTheRunCouldNotHonour) {
    EXPECT_THROW(parse({"--out", "o", "--seconds", "0"}), std::runtime_error);
    EXPECT_THROW(parse({"--out", "o", "--ticks", "0"}), std::runtime_error);
    EXPECT_THROW(parse({"--out", "o", "--record-every", "0"}), std::runtime_error);
    EXPECT_THROW(parse({"--out", "o", "--video", "v.mp4", "--video-fps", "0"}), std::runtime_error);
    EXPECT_THROW(parse({"--out", "o", "--video", "v.mp4", "--video-size", "big"}), std::runtime_error);
    EXPECT_THROW(parse({"--out", "o", "--viewer", "--viewer-size", "0x0"}), std::runtime_error);
    EXPECT_THROW(parse({"--out", "o", "--viewer", "--viewer-fps", "0"}), std::runtime_error);
    EXPECT_THROW(parse({"--out", "o", "--viewer", "--viewer-size", "wide"}), std::runtime_error);
}

TEST(Cli, HandsTargetOptionsToTheirHandlers) {
    std::string                  color;
    bool                         flag = false;
    const std::vector<CliOption> target_options{
        {.name = "--color", .argument = "<name>", .apply = [&color](const std::string& value) { color = value; }},
        {.name = "--flag", .argument = "", .apply = [&flag](const std::string&) { flag = true; }},
    };

    std::vector<std::string> owned{"robot_sim", "--color", "red", "--maze", "m", "--flag", "--out", "o"};
    std::vector<char*>       argv;
    argv.reserve(owned.size());

    for (std::string& word : owned) {
        argv.push_back(word.data());
    }

    const CliOptions options = Cli::parse(std::span(argv), target_options);

    EXPECT_EQ(color, "red");
    EXPECT_TRUE(flag);
    EXPECT_EQ(options.maze, "m");
}

TEST(Cli, UsageNamesEveryOptionItAccepts) {
    const std::vector<CliOption> target_options{{.name = "--color", .argument = "<name>", .apply = {}}};
    const std::string            usage = Cli::usage("robot_sim", target_options);

    EXPECT_TRUE(usage.starts_with("usage: robot_sim "));

    for (const std::string option :
         {"--out", "--scenario", "--maze", "--seconds", "--ticks", "--seed", "--ideal", "--record-every", "--video",
          "--video-fps", "--video-camera", "--video-size", "--viewer", "--viewer-camera", "--viewer-fps",
          "--viewer-size", "--monitor", "--monitor-port", "--color <name>"}) {
        EXPECT_NE(usage.find(option), std::string::npos) << option;
    }
}
}  // namespace
}  // namespace micras::sim
