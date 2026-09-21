#include <array>
#include <span>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "micras/sim/app/cli.hpp"

namespace micras::sim {
namespace {
constexpr uint32_t firmware_loop_us{1042};

/**
 * @brief Parse a command line written the way a shell would pass it.
 *
 * @param words Arguments after the program name.
 * @return Parsed options.
 */
CliOptions parse(std::vector<std::string> words) {
    std::vector<std::string> owned{"micras_simulation"};
    owned.insert(owned.end(), words.begin(), words.end());

    std::vector<char*> argv;
    argv.reserve(owned.size());

    for (std::string& word : owned) {
        argv.push_back(word.data());
    }

    return Cli::parse(std::span(argv), firmware_loop_us);
}

TEST(Cli, ReadsTheRequiredOptions) {
    const CliOptions options = parse({"--model", "robot.xml", "--out", "runs/here"});

    EXPECT_EQ(options.model, "robot.xml");
    EXPECT_EQ(options.out, "runs/here");
    EXPECT_EQ(options.scenario.command, Command::NONE);
    EXPECT_EQ(options.scenario.button, ButtonPress::NONE);
    EXPECT_TRUE(options.scenario.fan_enabled);
    EXPECT_FALSE(options.video_enabled);
}

TEST(Cli, TurnsSecondsIntoTheTickCountEveryRunUses) {
    EXPECT_EQ(parse({"--model", "m.xml", "--out", "o", "--seconds", "4"}).ticks, 3838U);
    EXPECT_EQ(parse({"--model", "m.xml", "--out", "o", "--seconds", "8"}).ticks, 7677U);
    EXPECT_EQ(parse({"--model", "m.xml", "--out", "o", "--ticks", "12"}).ticks, 12U);
}

TEST(Cli, ReadsTheScenario) {
    const CliOptions options = parse(
        {"--model", "m.xml", "--out", "o", "--command", "explore", "--command-at", "1.5", "--button", "extra_long",
         "--button-at", "2.0", "--dip", "fan=1,risky=1", "--no-fan"}
    );

    EXPECT_EQ(options.scenario.command, Command::EXPLORE);
    EXPECT_DOUBLE_EQ(options.scenario.command_at, 1.5);
    EXPECT_EQ(options.scenario.button, ButtonPress::EXTRA_LONG);
    EXPECT_DOUBLE_EQ(options.scenario.button_at, 2.0);
    EXPECT_EQ(options.scenario.dip, (std::array<bool, 4>{true, false, false, true}));
    EXPECT_FALSE(options.scenario.fan_enabled);
}

TEST(Cli, ReadsTheRecordingOptions) {
    const CliOptions options = parse(
        {"--model", "m.xml", "--out", "o", "--video", "run.mp4", "--video-fps", "60", "--video-camera", "overhead",
         "--video-size", "640x480"}
    );

    EXPECT_TRUE(options.video_enabled);
    EXPECT_EQ(options.video.path, "run.mp4");
    EXPECT_EQ(options.video.fps, 60);
    EXPECT_EQ(options.video.camera, "overhead");
    EXPECT_EQ(options.video.width, 640);
    EXPECT_EQ(options.video.height, 480);
}

TEST(Cli, ReadsTheWindowOptions) {
    const CliOptions off = parse({"--model", "m.xml", "--out", "o"});
    EXPECT_FALSE(off.viewer_enabled);

    const CliOptions options = parse(
        {"--model", "m.xml", "--out", "o", "--viewer", "--viewer-camera", "overhead", "--viewer-size", "800x600",
         "--viewer-fps", "15"}
    );

    EXPECT_TRUE(options.viewer_enabled);
    EXPECT_EQ(options.viewer.camera, "overhead");
    EXPECT_EQ(options.viewer.width, 800);
    EXPECT_EQ(options.viewer.height, 600);
    EXPECT_EQ(options.viewer_fps, 15);
}

TEST(Cli, RejectsAnIncompleteCommandLine) {
    EXPECT_THROW(parse({"--out", "o"}), std::runtime_error);
    EXPECT_THROW(parse({"--model", "m.xml"}), std::runtime_error);
    EXPECT_THROW(parse({"--model", "m.xml", "--out"}), std::runtime_error);
    EXPECT_THROW(parse({"--model", "m.xml", "--out", "o", "--nonsense"}), std::runtime_error);
}

TEST(Cli, RejectsValuesTheRunCouldNotHonour) {
    EXPECT_THROW(parse({"--model", "m.xml", "--out", "o", "--seconds", "0"}), std::runtime_error);
    EXPECT_THROW(parse({"--model", "m.xml", "--out", "o", "--ticks", "0"}), std::runtime_error);
    EXPECT_THROW(parse({"--model", "m.xml", "--out", "o", "--seconds", "0.0005"}), std::runtime_error);
    EXPECT_THROW(parse({"--model", "m.xml", "--out", "o", "--command", "wander"}), std::runtime_error);
    EXPECT_THROW(parse({"--model", "m.xml", "--out", "o", "--button", "double"}), std::runtime_error);
    EXPECT_THROW(parse({"--model", "m.xml", "--out", "o", "--dip", "turbo=1"}), std::runtime_error);
    EXPECT_THROW(parse({"--model", "m.xml", "--out", "o", "--dip", "fan=2"}), std::runtime_error);
    EXPECT_THROW(parse({"--model", "m.xml", "--out", "o", "--video", "v.mp4", "--video-fps", "0"}), std::runtime_error);
    EXPECT_THROW(
        parse({"--model", "m.xml", "--out", "o", "--video", "v.mp4", "--video-size", "big"}), std::runtime_error
    );
    EXPECT_THROW(parse({"--model", "m.xml", "--out", "o", "--viewer", "--viewer-size", "0x0"}), std::runtime_error);
    EXPECT_THROW(parse({"--model", "m.xml", "--out", "o", "--viewer", "--viewer-fps", "0"}), std::runtime_error);
    EXPECT_THROW(parse({"--model", "m.xml", "--out", "o", "--viewer", "--viewer-size", "wide"}), std::runtime_error);
}

TEST(Cli, UsageNamesEveryOptionItAccepts) {
    const std::string usage = Cli::usage();

    for (const std::string& option :
         {"--model", "--out", "--maze", "--seconds", "--ticks", "--command", "--command-at", "--dip", "--button",
          "--button-at", "--no-fan", "--video", "--video-fps", "--video-camera", "--video-size", "--viewer",
          "--viewer-camera", "--viewer-fps", "--viewer-size"}) {
        EXPECT_NE(usage.find(option), std::string::npos) << option;
    }
}
}  // namespace
}  // namespace micras::sim
