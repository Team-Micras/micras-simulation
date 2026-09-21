#include <array>
#include <filesystem>
#include <fstream>
#include <span>

#include <gtest/gtest.h>

#include "micras/sim/recording/run_metadata.hpp"

namespace micras::sim {
namespace {
TEST(RunMetadata, SerializesInTheBaselineLayout) {
    const RunMetadata metadata{
        .firmware_sha = "ad342541267bb3c8db263630f27fc7edafba5b09",
        .model_path = "models/robot_v2.xml",
        .model_sha256 = "3ef384df",
        .maze_path = "",
        .mujoco_version = "3.3.6",
        .compiler = "GNU 13.3.0",
        .build_type = "RelWithDebInfo",
        .args = "--model models/robot_v2.xml --seconds 4 --out runs/idle",
        .loop_time_us = 1042,
        .timestep = 0.000521,
        .steps_per_tick = 2,
        .requested_ticks = 3838,
        .ticks = 3838,
        .sim_time = 3.9992,
        .final_z = 0.000806901,
        .pool_columns = 29,
        .telemetry_resyncs = 0,
        .warnings_total = 0,
        .interactive = false,
    };

    const std::string expected = "{\n"
                                 "  \"firmware_sha\": \"ad342541267bb3c8db263630f27fc7edafba5b09\",\n"
                                 "  \"model_path\": \"models/robot_v2.xml\",\n"
                                 "  \"model_sha256\": \"3ef384df\",\n"
                                 "  \"maze_path\": \"\",\n"
                                 "  \"mujoco_version\": \"3.3.6\",\n"
                                 "  \"compiler\": \"GNU 13.3.0\",\n"
                                 "  \"build_type\": \"RelWithDebInfo\",\n"
                                 "  \"args\": \"--model models/robot_v2.xml --seconds 4 --out runs/idle\",\n"
                                 "  \"loop_time_us\": 1042,\n"
                                 "  \"timestep\": 0.000521,\n"
                                 "  \"steps_per_tick\": 2,\n"
                                 "  \"requested_ticks\": 3838,\n"
                                 "  \"ticks\": 3838,\n"
                                 "  \"sim_time\": 3.9992,\n"
                                 "  \"final_z\": 0.000806901,\n"
                                 "  \"pool_columns\": 29,\n"
                                 "  \"telemetry_resyncs\": 0,\n"
                                 "  \"warnings_total\": 0,\n"
                                 "  \"interactive\": false\n"
                                 "}\n";

    EXPECT_EQ(metadata.to_json(), expected);
}

TEST(RunMetadata, EscapesJsonStrings) {
    RunMetadata metadata;
    metadata.args = std::string("quote\" backslash\\ newline\n tab\t bell") + '\001';

    const std::string json = metadata.to_json();
    const std::string expected = R"("args": "quote\" backslash\\ newline\n tab\t bell\u0001")";
    EXPECT_NE(json.find(expected), std::string::npos);
}

TEST(RunMetadata, HashesFilesWithSha256) {
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "micras_sha256_test.txt";
    std::ofstream(path) << "abc";

    EXPECT_EQ(RunMetadata::sha256_of(path), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    EXPECT_EQ(RunMetadata::sha256_of("/nonexistent/file"), "");
    std::filesystem::remove(path);
}

TEST(RunMetadata, JoinsArgumentsAfterTheProgramName) {
    std::array<std::string, 4> arguments{"bin", "--model", "m.xml", "--out"};
    std::array<char*, 4>       argv{};

    for (std::size_t i = 0; i < arguments.size(); i++) {
        argv.at(i) = arguments.at(i).data();
    }

    EXPECT_EQ(RunMetadata::join_args(argv), "--model m.xml --out");
    EXPECT_EQ(RunMetadata::join_args(std::span(argv).first(1)), "");
}
}  // namespace
}  // namespace micras::sim
