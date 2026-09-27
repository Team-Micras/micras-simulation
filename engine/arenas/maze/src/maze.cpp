/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <format>
#include <sstream>
#include <stdexcept>

#include "micras/sim/arenas/maze.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/text_file.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Characters one cell takes in a line of the drawing.
 */
constexpr std::size_t cell_width{4};

/**
 * @brief Split a drawing into lines, dropping trailing spaces and blank lines at the end.
 *
 * @param text The drawing.
 * @return Its lines, top first.
 */
std::vector<std::string> lines_of(std::string_view text) {
    std::vector<std::string> lines;
    std::istringstream       stream{std::string{text}};
    std::string              line;

    while (std::getline(stream, line)) {
        while (not line.empty() and (line.back() == ' ' or line.back() == '\r')) {
            line.pop_back();
        }

        lines.push_back(line);
    }

    while (not lines.empty() and lines.back().empty()) {
        lines.pop_back();
    }

    return lines;
}

/**
 * @brief Get a character of a line, or a space past its end.
 *
 * @param line The line.
 * @param index Index of the character.
 * @return The character.
 */
char at(const std::string& line, std::size_t index) {
    return index < line.size() ? line[index] : ' ';
}

/**
 * @brief Half the thickness of the red paint on top of the walls and posts, in meters.
 *
 * @note The paint is a box of its own that collides with nothing, so the walls the sensors and the
 *       robot meet are the same whatever their color.
 */
constexpr double cap_half_height{0.00025};

/**
 * @brief Number of cells the floor reaches past the maze on every side.
 *
 * @note MuJoCo lays the texture of a plane from its corner, so a whole number of cells keeps the
 *       checker on the middle of the walls.
 */
constexpr double floor_margin{2.0};
}  // namespace

Maze Maze::load(const std::filesystem::path& path) {
    return parse(read_text_file(path, "maze"));
}

Maze Maze::parse(std::string_view text) {
    std::vector<std::string> lines = lines_of(text);

    if (lines.size() < 3 or lines.size() % 2 == 0 or lines.front().size() < cell_width + 1) {
        throw std::runtime_error("a maze drawing needs an odd number of lines, alternating posts and cells");
    }

    std::ranges::reverse(lines);

    Maze maze;
    maze.columns = (lines.front().size() - 1) / cell_width;
    maze.rows = (lines.size() - 1) / 2;
    maze.south.assign(maze.rows + 1, std::vector<bool>(maze.columns, false));
    maze.west.assign(maze.rows, std::vector<bool>(maze.columns + 1, false));

    for (std::size_t row = 0; row <= maze.rows; row++) {
        for (std::size_t column = 0; column < maze.columns; column++) {
            maze.south[row][column] =
                lines[2 * row].substr(std::min(lines[2 * row].size(), cell_width * column + 1), 3) == "---";
        }
    }

    for (std::size_t row = 0; row < maze.rows; row++) {
        const std::string& line = lines[2 * row + 1];

        for (std::size_t column = 0; column <= maze.columns; column++) {
            maze.west[row][column] = at(line, cell_width * column) == '|';
        }

        for (std::size_t column = 0; column < maze.columns; column++) {
            const char mark = at(line, cell_width * column + 2);

            if (mark == 'S') {
                maze.start_cell = {column, row};
            } else if (mark == 'G') {
                maze.goal_cells.emplace_back(column, row);
            }
        }
    }

    return maze;
}

bool Maze::south_wall(std::size_t column, std::size_t row) const {
    return row < this->south.size() and column < this->columns and this->south[row][column];
}

bool Maze::west_wall(std::size_t column, std::size_t row) const {
    return row < this->rows and column < this->west[row].size() and this->west[row][column];
}

double Maze::reflectance(std::string_view geom_name, const MazeConfig& config) {
    return geom_name.ends_with("floor") ? config.floor_reflectance : config.wall_reflectance;
}

bool Maze::is_marked(std::size_t column, std::size_t row) const {
    const std::pair<std::size_t, std::size_t> cell{column, row};
    return cell == this->start_cell or std::ranges::find(this->goal_cells, cell) != this->goal_cells.end();
}

std::string Maze::paint(const MazeConfig& config) const {
    const double cell = config.cell_size;
    const double half_thickness = config.wall_thickness / 2;
    const double half_length = (cell - config.wall_thickness) / 2;
    const double z = config.wall_height + cap_half_height;
    std::string  caps;

    const auto cap = [&caps,
                      z](std::string_view name, std::size_t column, std::size_t row, double x, double y, double half_x,
                         double half_y) {
        caps += std::format(
            "      <geom name=\"{}_top_{}_{}\" class=\"top\" pos=\"{:.6f} {:.6f} {:.6f}\" size=\"{:.6f} {:.6f} "
            "{:.6f}\"/>\n",
            name, column, row, x, y, z, half_x, half_y, cap_half_height
        );
    };

    for (std::size_t row = 0; row <= this->rows; row++) {
        for (std::size_t column = 0; column <= this->columns; column++) {
            const double x = static_cast<double>(column) * cell;
            const double y = static_cast<double>(row) * cell;
            const bool   left = column > 0 and this->is_marked(column - 1, row);
            const bool   below = row > 0 and this->is_marked(column, row - 1);
            const bool   here = this->is_marked(column, row);
            const bool   corner = column > 0 and row > 0 and this->is_marked(column - 1, row - 1);

            if (not(here or left or below or corner)) {
                cap("post", column, row, x, y, half_thickness, half_thickness);
            }

            if (this->south_wall(column, row) and not(here or below)) {
                cap("south", column, row, x + cell / 2, y, half_length, half_thickness);
            }

            if (this->west_wall(column, row) and not(here or left)) {
                cap("west", column, row, x, y + cell / 2, half_thickness, half_length);
            }
        }
    }

    return caps;
}

std::string Maze::mjcf(const MazeConfig& config) const {
    const double cell = config.cell_size;
    const double half_thickness = config.wall_thickness / 2;
    const double half_height = config.wall_height / 2;
    const double half_length = (cell - config.wall_thickness) / 2;
    const double width = static_cast<double>(this->columns) * cell;
    const double height = static_cast<double>(this->rows) * cell;
    std::string  geoms;

    for (std::size_t row = 0; row <= this->rows; row++) {
        for (std::size_t column = 0; column <= this->columns; column++) {
            geoms += std::format(
                "      <geom name=\"post_{}_{}\" class=\"post\" pos=\"{:.6f} {:.6f} {:.6f}\"/>\n", column, row,
                static_cast<double>(column) * cell, static_cast<double>(row) * cell, half_height
            );

            if (this->south_wall(column, row)) {
                geoms += std::format(
                    "      <geom name=\"south_{}_{}\" class=\"horizontal\" pos=\"{:.6f} {:.6f} {:.6f}\"/>\n", column,
                    row, (static_cast<double>(column) + 0.5) * cell, static_cast<double>(row) * cell, half_height
                );
            }

            if (this->west_wall(column, row)) {
                geoms += std::format(
                    "      <geom name=\"west_{}_{}\" class=\"vertical\" pos=\"{:.6f} {:.6f} {:.6f}\"/>\n", column, row,
                    static_cast<double>(column) * cell, (static_cast<double>(row) + 0.5) * cell, half_height
                );
            }
        }
    }

    const std::string caps = this->paint(config);

    return std::format(
        "<mujoco model=\"maze\">\n"
        "  <asset>\n"
        "    <texture name=\"grid\" type=\"2d\" builtin=\"checker\" width=\"512\" height=\"512\" rgb1=\"0.1 0.1 0.1\"\n"
        "             rgb2=\"0.2 0.2 0.2\"/>\n"
        "    <material name=\"floor\" texture=\"grid\" texrepeat=\"{0:.9g} {0:.9g}\" texuniform=\"true\" "
        "specular=\"0\" "
        "reflectance=\"0.2\"/>\n"
        "    <material name=\"wall\" rgba=\"1 1 1 1\"/>\n"
        "    <material name=\"post\" rgba=\"1 1 1 1\"/>\n"
        "    <material name=\"top\" rgba=\"1 0 0 1\"/>\n"
        "  </asset>\n"
        "  <default>\n"
        "    <default class=\"post\">\n"
        "      <geom type=\"box\" material=\"post\" size=\"{1:.6f} {1:.6f} {2:.6f}\"/>\n"
        "    </default>\n"
        "    <default class=\"horizontal\">\n"
        "      <geom type=\"box\" material=\"wall\" size=\"{3:.6f} {1:.6f} {2:.6f}\"/>\n"
        "    </default>\n"
        "    <default class=\"vertical\">\n"
        "      <geom type=\"box\" material=\"wall\" size=\"{1:.6f} {3:.6f} {2:.6f}\"/>\n"
        "    </default>\n"
        "    <default class=\"top\">\n"
        "      <geom type=\"box\" material=\"top\" contype=\"0\" conaffinity=\"0\" group=\"{12}\"/>\n"
        "    </default>\n"
        "  </default>\n"
        "  <worldbody>\n"
        "    <body name=\"{4}\">\n"
        "      <light name=\"sun\" pos=\"{5:.6f} {6:.6f} 3\" dir=\"0 0 -1\" directional=\"true\" castshadow=\"false\" "
        "diffuse=\"0.7 0.7 0.7\"/>\n"
        "      <camera name=\"top\" pos=\"{5:.6f} {6:.6f} 4\" xyaxes=\"1 0 0 0 1 0\" projection=\"orthographic\" "
        "fovy=\"{7:.6f}\"/>\n"
        "      <geom name=\"floor\" type=\"plane\" pos=\"{5:.6f} {6:.6f} 0\" size=\"{8:.6f} {9:.6f} 0.1\" "
        "material=\"floor\"/>\n"
        "{10}{11}"
        "    </body>\n"
        "  </worldbody>\n"
        "</mujoco>\n",
        1.0 / cell, half_thickness, half_height, half_length, body_name, width / 2, height / 2,
        std::max(width, height) + 0.2, width / 2 + floor_margin * cell, height / 2 + floor_margin * cell, geoms, caps,
        MujocoWorld::unseen_group
    );
}
}  // namespace micras::sim
