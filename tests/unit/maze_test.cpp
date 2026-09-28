#include <cmath>
#include <filesystem>
#include <fstream>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <doctest/doctest.h>

#include "micras/sim/arenas/maze.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/text_file.hpp"

namespace micras::sim {
namespace {
/**
 * @brief A two by two maze: the start at the lower left, a goal above it, one wall inside each row.
 */
constexpr std::string_view small_maze{"o---o---o\n"
                                      "| G     |\n"
                                      "o   o---o\n"
                                      "| S |   |\n"
                                      "o---o---o\n"};

TEST_CASE("Maze.ReadsTheSizeOfTheDrawing") {
    const Maze maze = Maze::parse(small_maze);

    CHECK_EQ(maze.width(), 2U);
    CHECK_EQ(maze.height(), 2U);
}

TEST_CASE("Maze.ReadsTheWallsBottomUp") {
    const Maze maze = Maze::parse(small_maze);

    CHECK(maze.south_wall(0, 0));
    CHECK(maze.south_wall(1, 0));
    CHECK_FALSE(maze.south_wall(0, 1));
    CHECK(maze.south_wall(1, 1));
    CHECK(maze.south_wall(0, 2));
    CHECK(maze.south_wall(1, 2));

    CHECK(maze.west_wall(0, 0));
    CHECK(maze.west_wall(1, 0));
    CHECK(maze.west_wall(2, 0));
    CHECK(maze.west_wall(0, 1));
    CHECK_FALSE(maze.west_wall(1, 1));
    CHECK(maze.west_wall(2, 1));
}

TEST_CASE("Maze.ReportsNoWallOutsideTheDrawing") {
    const Maze maze = Maze::parse(small_maze);

    CHECK_FALSE(maze.south_wall(2, 0));
    CHECK_FALSE(maze.south_wall(0, 3));
    CHECK_FALSE(maze.west_wall(3, 0));
    CHECK_FALSE(maze.west_wall(0, 2));
}

TEST_CASE("Maze.ReadsTheStartAndTheGoals") {
    const Maze maze = Maze::parse(small_maze);

    CHECK_EQ(maze.start(), (std::pair<std::size_t, std::size_t>{0, 0}));
    REQUIRE_EQ(maze.goals().size(), 1U);
    CHECK_EQ(maze.goals().front(), (std::pair<std::size_t, std::size_t>{0, 1}));
}

TEST_CASE("Maze.StartsAtTheLowerLeftWhenTheDrawingMarksNoStart") {
    const Maze maze = Maze::parse("o---o---o\n|   | S |\no---o---o\n");
    const Maze unmarked = Maze::parse("o---o---o\n|       |\no---o---o\n");

    CHECK_EQ(maze.start(), (std::pair<std::size_t, std::size_t>{1, 0}));
    CHECK_EQ(unmarked.start(), (std::pair<std::size_t, std::size_t>{0, 0}));
    CHECK(unmarked.goals().empty());
}

TEST_CASE("Maze.IgnoresTrailingSpacesCarriageReturnsAndBlankLines") {
    const Maze maze = Maze::parse("o---o---o   \r\n| G     |\r\no   o---o\r\n| S |   |  \r\no---o---o\r\n\n\n");

    CHECK_EQ(maze.width(), 2U);
    CHECK_EQ(maze.height(), 2U);
    CHECK_FALSE(maze.west_wall(1, 1));
    CHECK(maze.west_wall(1, 0));
    CHECK_EQ(maze.goals().size(), 1U);
}

TEST_CASE("Maze.RejectsADrawingWithAnEvenNumberOfLines") {
    CHECK_THROWS_AS(Maze::parse("o---o\n|   |\no---o\n|   |\n"), std::runtime_error);
}

TEST_CASE("Maze.RejectsADrawingTooShortToHoldACell") {
    CHECK_THROWS_AS(Maze::parse(""), std::runtime_error);
    CHECK_THROWS_AS(Maze::parse("o---o\n"), std::runtime_error);
    CHECK_THROWS_AS(Maze::parse("o\n|\no\n"), std::runtime_error);
}

TEST_CASE("Maze.RefusesAFileThatDoesNotExist") {
    CHECK_THROWS_AS(Maze::load(std::filesystem::path{MICRAS_TEST_MAZES_DIR} / "no_such_maze.txt"), std::runtime_error);
}

TEST_CASE("Maze.LoadsABundledMazeClosedAllAround") {
    const Maze maze = Maze::load(std::filesystem::path{MICRAS_TEST_MAZES_DIR} / "maze1.txt");

    REQUIRE_EQ(maze.width(), 16U);
    REQUIRE_EQ(maze.height(), 16U);
    CHECK_EQ(maze.start(), (std::pair<std::size_t, std::size_t>{0, 0}));
    CHECK_FALSE(maze.goals().empty());

    for (std::size_t i = 0; i < 16; i++) {
        CHECK_MESSAGE(maze.south_wall(i, 0), "bottom edge, column " << i);
        CHECK_MESSAGE(maze.south_wall(i, 16), "top edge, column " << i);
        CHECK_MESSAGE(maze.west_wall(0, i), "left edge, row " << i);
        CHECK_MESSAGE(maze.west_wall(16, i), "right edge, row " << i);
    }
}

TEST_CASE("Maze.GeneratesAnArenaAWorldCanAttach") {
    const Maze  maze = Maze::parse(small_maze);
    MujocoWorld world;

    world.build(
        read_text_file(MICRAS_TEST_MODEL, "model"), "robot", maze.mjcf({}), Maze::body_name, {.x = 0.09, .y = 0.09}
    );

    CHECK_NOTHROW(world.require_id(mjOBJ_GEOM, "maze_floor"));
    CHECK_NOTHROW(world.require_id(mjOBJ_GEOM, "maze_south_1_1"));
    CHECK_NOTHROW(world.require_id(mjOBJ_GEOM, "maze_west_1_0"));
    CHECK_NOTHROW(world.require_id(mjOBJ_GEOM, "maze_post_2_2"));
    CHECK_THROWS_AS(world.require_id(mjOBJ_GEOM, "maze_west_1_1"), std::runtime_error);
    CHECK_NOTHROW(world.require_id(mjOBJ_GEOM, "chassis"));
    CHECK_FALSE(world.composed_xml().empty());
}

TEST_CASE("Maze.PlacesTheWallsOnTheCellEdges") {
    const Maze       maze = Maze::parse(small_maze);
    const MazeConfig config{.cell_size = 0.2};
    MujocoWorld      world;

    world.build(
        read_text_file(MICRAS_TEST_MODEL, "model"), "robot", maze.mjcf(config), Maze::body_name, {.x = 0.1, .y = 0.1}
    );
    world.reset();

    const std::span<const mjtNum> positions(
        world.data()->geom_xpos, static_cast<std::size_t>(3 * world.model()->ngeom)
    );
    const auto first = static_cast<std::size_t>(3 * world.require_id(mjOBJ_GEOM, "maze_south_1_1"));

    CHECK_LE(std::abs(positions[first] - 0.3), 1e-9);
    CHECK_LE(std::abs(positions[first + 1] - 0.2), 1e-9);
    CHECK_LE(std::abs(positions[first + 2] - (config.wall_height / 2)), 1e-9);
}

TEST_CASE("Maze.SeesTheFloorDarkAndEverythingElseAsWall") {
    const MazeConfig config{.wall_reflectance = 0.7, .floor_reflectance = 0.1};

    CHECK_EQ(Maze::reflectance("maze_floor", config), doctest::Approx(0.1).epsilon(1e-12));
    CHECK_EQ(Maze::reflectance("floor", config), doctest::Approx(0.1).epsilon(1e-12));
    CHECK_EQ(Maze::reflectance("maze_south_0_0", config), doctest::Approx(0.7).epsilon(1e-12));
    CHECK_EQ(Maze::reflectance("maze_post_1_1", config), doctest::Approx(0.7).epsilon(1e-12));
}
}  // namespace
}  // namespace micras::sim
