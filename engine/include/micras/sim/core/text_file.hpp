/**
 * @file
 *
 * @brief Reading a whole text file, for the loaders of the robot, the scenario and the maze.
 */

#ifndef MICRAS_SIM_CORE_TEXT_FILE_HPP
#define MICRAS_SIM_CORE_TEXT_FILE_HPP

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace micras::sim {
/**
 * @brief Read a whole text file.
 *
 * @param path Path of the file.
 * @param what What the file holds, for the error message.
 * @return The contents.
 */
inline std::string read_text_file(const std::filesystem::path& path, std::string_view what) {
    std::ifstream file(path);

    if (not file.is_open()) {
        throw std::runtime_error("cannot read the " + std::string{what} + " " + path.string());
    }

    std::ostringstream text;
    text << file.rdbuf();
    return text.str();
}
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_TEXT_FILE_HPP
