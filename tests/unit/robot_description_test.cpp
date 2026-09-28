#include <cstddef>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include <doctest/doctest.h>
#include <mujoco/mjtype.h>

#include "micras/sim/arenas/maze.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/robot/robot_description.hpp"
#include "micras/sim/robot/robot_model.hpp"

namespace micras::sim {
/**
 * @brief Read the tiny robot's description as text.
 *
 * @return Contents of tiny_robot.toml.
 */
static std::string tiny_text() {
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
static std::string replaced(std::string text, std::string_view from, std::string_view to) {
    const std::size_t position = text.find(from);
    CHECK_MESSAGE(position != std::string::npos, "the tiny robot has no " << from);

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
static std::string refusal_of(std::string_view text) {
    try {
        RobotDescription::parse(text, "tiny.toml");
    } catch (const std::runtime_error& error) {
        return error.what();
    }

    return {};
}

namespace {
TEST_CASE("RobotDescription.ReadsEverySection") {
    const RobotDescription robot = RobotDescription::load(MICRAS_TEST_ROBOT);

    CHECK_EQ(robot.name, "tiny");
    CHECK_EQ(robot.integration.timestep, doctest::Approx(0.000521).epsilon(1e-12));
    CHECK_EQ(robot.chassis.mass, doctest::Approx(0.05).epsilon(1e-12));
    CHECK_EQ(robot.chassis.outline.size(), 4U);
    CHECK_EQ(robot.wheels.track, doctest::Approx(0.05).epsilon(1e-12));
    CHECK_EQ(robot.drive.resistance(), doctest::Approx(10.5).epsilon(1e-12));
    CHECK_EQ(robot.encoders.counts_per_revolution, 4096U);
    CHECK_EQ(robot.imu.axes.at(1).at(1), doctest::Approx(1.0).epsilon(1e-12));
    CHECK_EQ(robot.fan.nominal_voltage, doctest::Approx(7.4).epsilon(1e-12));
    CHECK_EQ(robot.battery.cells, 2);
    CHECK_EQ(robot.link.baud_rate, 9600U);
    CHECK_EQ(robot.wall_sensors.rays, 4);
}

TEST_CASE("RobotDescription.ReadsArraysOfParts") {
    const RobotDescription robot = RobotDescription::load(MICRAS_TEST_ROBOT);

    REQUIRE_EQ(robot.chassis.boxes.size(), 1U);
    CHECK_EQ(robot.chassis.boxes.front().name, "cargo");
    CHECK_EQ(robot.chassis.boxes.front().max.at(2), doctest::Approx(0.02).epsilon(1e-12));
    REQUIRE_EQ(robot.chassis.cylinders.size(), 1U);
    CHECK_EQ(robot.chassis.cylinders.front().radius, doctest::Approx(0.005).epsilon(1e-12));
    REQUIRE_EQ(robot.wall_sensors.sensors.size(), 1U);
    CHECK_EQ(robot.wall_sensors.sensors.front().name, "eye");
}

TEST_CASE("RobotDescription.ConvertsDegreesToRadians") {
    const RobotDescription robot = RobotDescription::load(MICRAS_TEST_ROBOT);

    CHECK_EQ(robot.wall_sensors.sensors.front().yaw, doctest::Approx(std::numbers::pi / 2).epsilon(1e-12));
    CHECK_EQ(robot.wall_sensors.emitter_half_angle, doctest::Approx(5.0 * std::numbers::pi / 180.0).epsilon(1e-12));
}

TEST_CASE("RobotDescription.AcceptsPlainValuesAndSourcedTablesAlike") {
    const std::string      text = tiny_text();
    const RobotDescription plain = RobotDescription::parse(
        replaced(text, R"(winding_resistance = { value = 10.0, source = "made up" })", "winding_resistance = 10.0")
    );
    const RobotDescription sourced = RobotDescription::parse(
        replaced(text, "bridge_resistance = 0.5", R"(bridge_resistance = { value = 0.5, source = "datasheet" })")
    );

    CHECK_EQ(plain.drive.winding_resistance, doctest::Approx(10.0).epsilon(1e-12));
    CHECK_EQ(sourced.drive.bridge_resistance, doctest::Approx(0.5).epsilon(1e-12));
    CHECK_EQ(plain.drive.resistance(), doctest::Approx(sourced.drive.resistance()).epsilon(1e-12));
}

TEST_CASE("RobotDescription.RefusesAMissingKeyNamingIt") {
    const std::string message = refusal_of(replaced(tiny_text(), "bridge_resistance = 0.5\n", ""));

    CHECK_MESSAGE(message.contains("drive.bridge_resistance"), message);
    CHECK_MESSAGE(message.contains("missing"), message);
}

TEST_CASE("RobotDescription.RefusesAMissingSectionNamingIt") {
    const std::string message = refusal_of(replaced(tiny_text(), "[link]\nbaud_rate = 9600\n", ""));

    CHECK_MESSAGE(message.contains("[link]"), message);
}

TEST_CASE("RobotDescription.RefusesAnUnknownKeyNamingIt") {
    const std::string message = refusal_of(replaced(tiny_text(), "[battery]\n", "[battery]\ncolour = \"red\"\n"));

    CHECK_MESSAGE(message.contains("unknown key battery.colour"), message);
}

TEST_CASE("RobotDescription.RefusesAnUnknownKeyInAnArrayOfTables") {
    const std::string message = refusal_of(replaced(tiny_text(), "name = \"eye\"\n", "name = \"eye\"\nshiny = true\n"));

    CHECK_MESSAGE(message.contains("wall_sensors.sensors[0].shiny"), message);
}

TEST_CASE("RobotDescription.RefusesAValueOfTheWrongTypeNamingIt") {
    const std::string text_instead = refusal_of(replaced(tiny_text(), "cells = 2", "cells = \"two\""));
    const std::string fraction = refusal_of(replaced(tiny_text(), "cells = 2", "cells = 2.5"));
    const std::string short_vector =
        refusal_of(replaced(tiny_text(), "chip_y = [0.0, 1.0, 0.0]", "chip_y = [0.0, 1.0]"));
    const std::string sourced = refusal_of(replaced(
        tiny_text(), R"(mass = { value = 0.05, source = "made up" })", R"(mass = { value = "heavy", source = "x" })"
    ));

    CHECK_MESSAGE(text_instead.contains("battery.cells must be a whole number"), text_instead);
    CHECK_MESSAGE(fraction.contains("battery.cells must be a whole number"), fraction);
    CHECK_MESSAGE(short_vector.contains("imu.chip_y must be 3 numbers"), short_vector);
    CHECK_MESSAGE(sourced.contains("chassis.mass must be a number"), sourced);
}

TEST_CASE("RobotDescription.RefusesASourcedTableThatIsNotExactlyAValueAndASource") {
    const std::string unsourced = refusal_of(replaced(tiny_text(), R"(, source = "made up" })", " }"));
    const std::string extra =
        refusal_of(replaced(tiny_text(), R"(source = "made up" })", R"(source = "made up", unit = "kg" })"));

    CHECK_MESSAGE(unsourced.contains("chassis.mass"), unsourced);
    CHECK_MESSAGE(extra.contains("chassis.mass"), extra);
}

TEST_CASE("RobotDescription.RefusesASchemaVersionItDoesNotKnow") {
    const std::string message = refusal_of(replaced(tiny_text(), "schema = 1", "schema = 2"));

    CHECK_MESSAGE(message.contains("schema 2"), message);
}

TEST_CASE("RobotDescription.RefusesAnOutlineOfFewerThanThreePoints") {
    const std::string message = refusal_of(replaced(
        tiny_text(), "outline = [[0.04, 0.02], [-0.03, 0.02], [-0.03, -0.02], [0.04, -0.02]]",
        "outline = [[0.04, 0.02], [-0.03, 0.02]]"
    ));

    CHECK_MESSAGE(message.contains("chassis.outline"), message);
}

TEST_CASE("RobotDescription.NamesTheFileInEveryRefusal") {
    const std::string syntax = refusal_of("schema = = 1");
    const std::string missing = refusal_of(replaced(tiny_text(), "bridge_resistance = 0.5\n", ""));

    CHECK_MESSAGE(syntax.starts_with("tiny.toml"), syntax);
    CHECK_MESSAGE(missing.starts_with("tiny.toml"), missing);
}

TEST_CASE("RobotDescription.RefusesAFileThatDoesNotExist") {
    CHECK_THROWS_AS(
        RobotDescription::load(std::filesystem::path{MICRAS_TEST_ROBOT}.parent_path() / "no_such_robot.toml"),
        std::runtime_error
    );
}

TEST_CASE("RobotDescription.GeneratesAModelThatComposesWithAnArena") {
    const RobotDescription robot = RobotDescription::load(MICRAS_TEST_ROBOT);
    const RobotModelNames  names = RobotModelNames::of(robot);
    const Maze             maze = Maze::parse("o---o\n| S |\no---o\n");
    MujocoWorld            world;

    world.build(robot_mjcf(robot), names.body, maze.mjcf({}), Maze::body_name, {.x = 0.09, .y = 0.09});
    world.reset();

    CHECK_EQ(names.body, "tiny");
    CHECK_EQ(world.timestep(), doctest::Approx(robot.integration.timestep).epsilon(1e-12));
    CHECK_NOTHROW(world.require_id(mjOBJ_BODY, names.body));
    CHECK_MESSAGE(world.model()->njnt == 3, "the free joint and one per wheel");
    CHECK_MESSAGE(world.model()->nu == 3, "one motor per wheel and the fan");
    CHECK_NOTHROW(world.require_id(mjOBJ_GEOM, "cargo"));
    CHECK_NOTHROW(world.require_id(mjOBJ_GEOM, "mast"));
    CHECK_NOTHROW(world.require_id(mjOBJ_SITE, "eye_emitter"));
    CHECK_NOTHROW(world.sensor_address(names.gyro));
}
}  // namespace
}  // namespace micras::sim
