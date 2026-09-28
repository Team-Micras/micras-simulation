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
        .target = "tiny",
        .target_dir = "targets/tiny",
        .firmware_sha = "0be27df",
        .robot_path = "robot.toml",
        .robot_sha256 = "a1",
        .scenario_path = "idle.toml",
        .maze_path = "maze1.txt",
        .model_sha256 = "b2",
        .mujoco_version = "3.14.0",
        .compiler = "GNU 13.4.0",
        .build_type = "RelWithDebInfo",
        .args = "--out runs/idle",
        .seed = 1,
        .ideal = false,
        .loop_time_us = 125,
        .timestep = 0.000125,
        .steps_per_tick = 1,
        .record_every = 1,
        .requested_ticks = 32000,
        .ticks = 32000,
        .sim_time = 4,
        .stopped_at = -1,
        .final_z = 0.0001,
        .target_fields = {{.name = "unbound_ports", .value = 0}},
        .warnings_total = 0,
        .interactive = false,
        .events = {{.time = 0.5, .kind = "state", .detail = "IDLE"}},
    };

    const std::string expected = "{\n"
                                 "  \"target\": \"tiny\",\n"
                                 "  \"target_dir\": \"targets/tiny\",\n"
                                 "  \"firmware_sha\": \"0be27df\",\n"
                                 "  \"robot_path\": \"robot.toml\",\n"
                                 "  \"robot_sha256\": \"a1\",\n"
                                 "  \"scenario_path\": \"idle.toml\",\n"
                                 "  \"maze_path\": \"maze1.txt\",\n"
                                 "  \"model_sha256\": \"b2\",\n"
                                 "  \"mujoco_version\": \"3.14.0\",\n"
                                 "  \"compiler\": \"GNU 13.4.0\",\n"
                                 "  \"build_type\": \"RelWithDebInfo\",\n"
                                 "  \"args\": \"--out runs/idle\",\n"
                                 "  \"seed\": 1,\n"
                                 "  \"ideal\": false,\n"
                                 "  \"loop_time_us\": 125,\n"
                                 "  \"timestep\": 0.000125,\n"
                                 "  \"steps_per_tick\": 1,\n"
                                 "  \"record_every\": 1,\n"
                                 "  \"requested_ticks\": 32000,\n"
                                 "  \"ticks\": 32000,\n"
                                 "  \"sim_time\": 4,\n"
                                 "  \"stopped_at\": -1,\n"
                                 "  \"final_z\": 0.0001,\n"
                                 "  \"unbound_ports\": 0,\n"
                                 "  \"warnings_total\": 0,\n"
                                 "  \"serial_dropped_bytes\": 0,\n"
                                 "  \"bridge_dropped_frames\": 0,\n"
                                 "  \"interactive\": false,\n"
                                 "  \"events\": [\n"
                                 "    {\"time\": 0.5, \"kind\": \"state\", \"detail\": \"IDLE\"}\n"
                                 "  ]\n"
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
    std::array<std::string, 4> arguments{"bin", "--maze", "maze1", "--out"};
    std::array<char*, 4>       argv{};

    for (std::size_t i = 0; i < arguments.size(); i++) {
        argv.at(i) = arguments.at(i).data();
    }

    EXPECT_EQ(RunMetadata::join_args(argv), "--maze maze1 --out");
    EXPECT_EQ(RunMetadata::join_args(std::span(argv).first(1)), "");
}
}  // namespace
}  // namespace micras::sim
