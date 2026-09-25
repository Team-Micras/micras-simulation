/**
 * @file
 *
 * @brief A micromouse maze, read from the usual ASCII drawing, as an arena.
 */

#ifndef MICRAS_SIM_ARENAS_MAZE_HPP
#define MICRAS_SIM_ARENAS_MAZE_HPP

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace micras::sim {
/**
 * @brief Dimensions and surfaces of a maze.
 *
 * @note The defaults are the competition rules: 180 mm cells, 12 mm thick and
 *       50 mm tall walls. The reflectances are what an infrared range sensor sees:
 *       white wall sides, a black floor.
 */
struct MazeConfig {
    double cell_size{0.18};
    double wall_thickness{0.012};
    double wall_height{0.05};
    double wall_reflectance{0.8};
    double floor_reflectance{0.05};
};

/**
 * @brief A maze: which walls stand, where it starts and where the goal is.
 *
 * @note The drawing is the format of tools/gen_maze.py and of the published maze
 *       collections: posts as 'o', horizontal walls as '---', vertical walls as
 *       '|', the start cell marked 'S' and goal cells 'G', read bottom-up so the
 *       start is at the lower left. The world frame puts the lower left post at
 *       the origin, x to the right and y up; a cell (x, y) spans
 *       [x, x + 1) and [y, y + 1) cells.
 */
class Maze {
public:
    /**
     * @brief Name of the arena body, which prefixes every geom once attached.
     */
    static constexpr std::string_view body_name{"maze"};

    /**
     * @brief Geom group of the paint on top of the walls, which only the eye sees.
     */
    static constexpr int paint_group{4};

    /**
     * @brief Read a maze drawing from a file.
     *
     * @param path Path of the drawing.
     * @return The maze.
     */
    static Maze load(const std::filesystem::path& path);

    /**
     * @brief Read a maze drawing.
     *
     * @param text The drawing.
     * @return The maze.
     */
    static Maze parse(std::string_view text);

    /**
     * @brief Get the size of the maze in cells.
     *
     * @return Width and height.
     */
    ///@{
    std::size_t width() const { return this->columns; }

    std::size_t height() const { return this->rows; }

    ///@}

    /**
     * @brief Check for a wall on the south edge of a cell, which row may be one past the top.
     *
     * @param column Column of the cell.
     * @param row Row of the cell.
     * @return True when the wall stands.
     */
    bool south_wall(std::size_t column, std::size_t row) const;

    /**
     * @brief Check for a wall on the west edge of a cell, which column may be one past the right.
     *
     * @param column Column of the cell.
     * @param row Row of the cell.
     * @return True when the wall stands.
     */
    bool west_wall(std::size_t column, std::size_t row) const;

    /**
     * @brief Get the start cell.
     *
     * @return Its column and row; the lower left cell when the drawing marks none.
     */
    std::pair<std::size_t, std::size_t> start() const { return this->start_cell; }

    /**
     * @brief Get the goal cells.
     *
     * @return Their columns and rows.
     */
    const std::vector<std::pair<std::size_t, std::size_t>>& goals() const { return this->goal_cells; }

    /**
     * @brief Generate the arena's MJCF: a body holding the floor, the posts and the walls.
     *
     * @note The walls and posts are white with a red top, the usual colors, except around the start
     *       and the goal cells, which are all white. The floor is a checker of cells, on the middle of
     *       the walls. The red tops are paint: they are in the paint group and collide with nothing,
     *       so none of that touches what the robot and its sensors meet.
     *
     * @param config Dimensions and surfaces.
     * @return The MJCF text.
     */
    std::string mjcf(const MazeConfig& config) const;

    /**
     * @brief Get the infrared reflectance of an arena geom.
     *
     * @param geom_name Name of the geom, with or without the attach prefix.
     * @param config Dimensions and surfaces.
     * @return The reflectance.
     */
    static double reflectance(std::string_view geom_name, const MazeConfig& config);

private:
    /**
     * @brief Check if a cell is the start or one of the goal cells, whose walls are all white.
     *
     * @param column Column of the cell.
     * @param row Row of the cell.
     * @return True if the cell is marked.
     */
    bool is_marked(std::size_t column, std::size_t row) const;

    /**
     * @brief Generate the red paint on top of every post and wall, but those around the marked cells.
     *
     * @param config Dimensions and surfaces.
     * @return The MJCF geoms of the paint.
     */
    std::string paint(const MazeConfig& config) const;

    std::size_t                                      columns{0};
    std::size_t                                      rows{0};
    std::vector<std::vector<bool>>                   south;
    std::vector<std::vector<bool>>                   west;
    std::pair<std::size_t, std::size_t>              start_cell{0, 0};
    std::vector<std::pair<std::size_t, std::size_t>> goal_cells;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_ARENAS_MAZE_HPP
