#include <algorithm>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include <gtest/gtest.h>

#include "micras/sim/arenas/maze.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/robot/robot_description.hpp"
#include "micras/sim/robot/robot_model.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Read the tiny robot's description as text.
 *
 * @return Contents of tiny_robot.toml.
 */
std::string tiny_text() {
    const std::ifstream file(MICRAS_TEST_ROBOT);
    std::ostringstream  text;
    text << file.rdbuf();
    return text.str();
}

/**
 * @brief Replace the one occurrence of a piece of text, failing the test when it is not there.
 *
 * @param text Text to edit.
 * @param from Text to replace.
 * @param to Replacement.
 * @return The edited text.
 */
std::string replaced(std::string text, std::string_view from, std::string_view to) {
    const std::size_t position = text.find(from);
    EXPECT_NE(position, std::string::npos) << "the tiny robot has no " << from;

    if (position != std::string::npos) {
        text.replace(position, from.size(), to);
    }

    return text;
}

/**
 * @brief Parse a description and return the message it was refused with.
 *
 * @param text Contents of a robot.toml.
 * @return The error message, or empty when the description was accepted.
 */
std::string refusal_of(std::string_view text) {
    try {
        RobotDescription::parse(text, "tiny.toml");
    } catch (const std::runtime_error& error) {
        return error.what();
    }

    return {};
}

TEST(RobotDescription, ReadsEverySection) {
    const RobotDescription robot = RobotDescription::load(MICRAS_TEST_ROBOT);

    EXPECT_EQ(robot.name, "tiny");
    EXPECT_DOUBLE_EQ(robot.integration.timestep, 0.000521);
    EXPECT_DOUBLE_EQ(robot.chassis.mass, 0.05);
    EXPECT_EQ(robot.chassis.outline.size(), 4U);
    EXPECT_DOUBLE_EQ(robot.wheels.track, 0.05);
    EXPECT_DOUBLE_EQ(robot.drive.resistance(), 10.5);
    EXPECT_EQ(robot.encoders.counts_per_revolution, 4096U);
    EXPECT_DOUBLE_EQ(robot.imu.axes[1][1], 1.0);
    EXPECT_DOUBLE_EQ(robot.fan.nominal_voltage, 7.4);
    EXPECT_EQ(robot.battery.cells, 2);
    EXPECT_EQ(robot.link.baud_rate, 9600U);
    EXPECT_EQ(robot.wall_sensors.rays, 4);
}

TEST(RobotDescription, ReadsArraysOfParts) {
    const RobotDescription robot = RobotDescription::load(MICRAS_TEST_ROBOT);

    ASSERT_EQ(robot.chassis.boxes.size(), 1U);
    EXPECT_EQ(robot.chassis.boxes.front().name, "cargo");
    EXPECT_DOUBLE_EQ(robot.chassis.boxes.front().max[2], 0.02);
    ASSERT_EQ(robot.chassis.cylinders.size(), 1U);
    EXPECT_DOUBLE_EQ(robot.chassis.cylinders.front().radius, 0.005);
    ASSERT_EQ(robot.wall_sensors.sensors.size(), 1U);
    EXPECT_EQ(robot.wall_sensors.sensors.front().name, "eye");
}

TEST(RobotDescription, ConvertsDegreesToRadians) {
    const RobotDescription robot = RobotDescription::load(MICRAS_TEST_ROBOT);

    EXPECT_DOUBLE_EQ(robot.wall_sensors.sensors.front().yaw, std::numbers::pi / 2);
    EXPECT_DOUBLE_EQ(robot.wall_sensors.emitter_half_angle, 5.0 * std::numbers::pi / 180.0);
}

TEST(RobotDescription, AcceptsPlainValuesAndSourcedTablesAlike) {
    const std::string      text = tiny_text();
    const RobotDescription plain = RobotDescription::parse(
        replaced(text, R"(winding_resistance = { value = 10.0, source = "made up" })", "winding_resistance = 10.0")
    );
    const RobotDescription sourced = RobotDescription::parse(
        replaced(text, "bridge_resistance = 0.5", R"(bridge_resistance = { value = 0.5, source = "datasheet" })")
    );

    EXPECT_DOUBLE_EQ(plain.drive.winding_resistance, 10.0);
    EXPECT_DOUBLE_EQ(sourced.drive.bridge_resistance, 0.5);
    EXPECT_DOUBLE_EQ(plain.drive.resistance(), sourced.drive.resistance());
}

TEST(RobotDescription, RefusesAMissingKeyNamingIt) {
    const std::string message = refusal_of(replaced(tiny_text(), "bridge_resistance = 0.5\n", ""));

    EXPECT_TRUE(message.contains("drive.bridge_resistance")) << message;
    EXPECT_TRUE(message.contains("missing")) << message;
}

TEST(RobotDescription, RefusesAMissingSectionNamingIt) {
    const std::string message = refusal_of(replaced(tiny_text(), "[link]\nbaud_rate = 9600\n", ""));

    EXPECT_TRUE(message.contains("[link]")) << message;
}

TEST(RobotDescription, RefusesAnUnknownKeyNamingIt) {
    const std::string message = refusal_of(replaced(tiny_text(), "[battery]\n", "[battery]\ncolour = \"red\"\n"));

    EXPECT_TRUE(message.contains("unknown key battery.colour")) << message;
}

TEST(RobotDescription, RefusesAnUnknownKeyInAnArrayOfTables) {
    const std::string message = refusal_of(replaced(tiny_text(), "name = \"eye\"\n", "name = \"eye\"\nshiny = true\n"));

    EXPECT_TRUE(message.contains("wall_sensors.sensors[0].shiny")) << message;
}

TEST(RobotDescription, RefusesAValueOfTheWrongTypeNamingIt) {
    const std::string text_instead = refusal_of(replaced(tiny_text(), "cells = 2", "cells = \"two\""));
    const std::string fraction = refusal_of(replaced(tiny_text(), "cells = 2", "cells = 2.5"));
    const std::string short_vector =
        refusal_of(replaced(tiny_text(), "chip_y = [0.0, 1.0, 0.0]", "chip_y = [0.0, 1.0]"));
    const std::string sourced = refusal_of(replaced(
        tiny_text(), R"(mass = { value = 0.05, source = "made up" })", R"(mass = { value = "heavy", source = "x" })"
    ));

    EXPECT_TRUE(text_instead.contains("battery.cells must be a whole number")) << text_instead;
    EXPECT_TRUE(fraction.contains("battery.cells must be a whole number")) << fraction;
    EXPECT_TRUE(short_vector.contains("imu.chip_y must be 3 numbers")) << short_vector;
    EXPECT_TRUE(sourced.contains("chassis.mass must be a number")) << sourced;
}

TEST(RobotDescription, RefusesASourcedTableThatIsNotExactlyAValueAndASource) {
    const std::string unsourced = refusal_of(replaced(tiny_text(), R"(, source = "made up" })", " }"));
    const std::string extra =
        refusal_of(replaced(tiny_text(), R"(source = "made up" })", R"(source = "made up", unit = "kg" })"));

    EXPECT_TRUE(unsourced.contains("chassis.mass")) << unsourced;
    EXPECT_TRUE(extra.contains("chassis.mass")) << extra;
}

TEST(RobotDescription, RefusesASchemaVersionItDoesNotKnow) {
    const std::string message = refusal_of(replaced(tiny_text(), "schema = 1", "schema = 2"));

    EXPECT_TRUE(message.contains("schema 2")) << message;
}

TEST(RobotDescription, RefusesAnOutlineOfFewerThanThreePoints) {
    const std::string message = refusal_of(replaced(
        tiny_text(), "outline = [[0.04, 0.02], [-0.03, 0.02], [-0.03, -0.02], [0.04, -0.02]]",
        "outline = [[0.04, 0.02], [-0.03, 0.02]]"
    ));

    EXPECT_TRUE(message.contains("chassis.outline")) << message;
}

TEST(RobotDescription, NamesTheFileInEveryRefusal) {
    const std::string syntax = refusal_of("schema = = 1");
    const std::string missing = refusal_of(replaced(tiny_text(), "bridge_resistance = 0.5\n", ""));

    EXPECT_TRUE(syntax.starts_with("tiny.toml")) << syntax;
    EXPECT_TRUE(missing.starts_with("tiny.toml")) << missing;
}

TEST(RobotDescription, RefusesAFileThatDoesNotExist) {
    EXPECT_THROW(
        RobotDescription::load(std::filesystem::path{MICRAS_TEST_ROBOT}.parent_path() / "no_such_robot.toml"),
        std::runtime_error
    );
}

TEST(RobotDescription, GeneratesAModelThatComposesWithAnArena) {
    const RobotDescription robot = RobotDescription::load(MICRAS_TEST_ROBOT);
    const RobotModelNames  names = RobotModelNames::of(robot);
    const Maze             maze = Maze::parse("o---o\n| S |\no---o\n");
    MujocoWorld            world;

    world.build(robot_mjcf(robot), names.body, maze.mjcf({}), Maze::body_name, {.x = 0.09, .y = 0.09});
    world.reset();

    EXPECT_EQ(names.body, "tiny");
    EXPECT_DOUBLE_EQ(world.timestep(), robot.integration.timestep);
    EXPECT_NO_THROW(world.require_id(mjOBJ_BODY, names.body));
    EXPECT_EQ(world.model()->njnt, 3) << "the free joint and one per wheel";
    EXPECT_EQ(world.model()->nu, 3) << "one motor per wheel and the fan";
    EXPECT_NO_THROW(world.require_id(mjOBJ_GEOM, "cargo"));
    EXPECT_NO_THROW(world.require_id(mjOBJ_GEOM, "mast"));
    EXPECT_NO_THROW(world.require_id(mjOBJ_SITE, "eye_emitter"));
    EXPECT_NO_THROW(world.sensor_address(names.gyro));
}
}  // namespace
}  // namespace micras::sim
