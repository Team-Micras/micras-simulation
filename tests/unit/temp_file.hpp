/**
 * @file
 *
 * @brief A file in the system's temporary directory that belongs to one test process.
 */

#ifndef MICRAS_SIM_TESTS_TEMP_FILE_HPP
#define MICRAS_SIM_TESTS_TEMP_FILE_HPP

#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>

#include <unistd.h>

namespace micras::sim {
/**
 * @brief Path in the temporary directory named after the process, removed when it goes out of scope.
 *
 * @details ctest runs every test case in its own process, several at once, so the process id keeps two cases
 * that use the same name from writing the same file.
 */
class TempFile {
public:
    /**
     * @brief Name the file, without creating it.
     *
     * @param name Name of the file, unique within the process.
     */
    explicit TempFile(std::string_view name) :
        file_path{
            std::filesystem::temp_directory_path() /
            ("micras_sim_" + std::to_string(::getpid()) + "_" + std::string(name))
        } { }

    /**
     * @brief Remove the file, if it exists.
     */
    ~TempFile() {
        std::error_code error;
        std::filesystem::remove(this->file_path, error);
    }

    /**
     * @brief Copying would remove the file twice.
     */
    TempFile(const TempFile&) = delete;

    /**
     * @brief Moving would remove the file twice.
     */
    TempFile(TempFile&&) = delete;

    /**
     * @brief Copying would remove the file twice.
     *
     * @return Never returns.
     */
    TempFile& operator=(const TempFile&) = delete;

    /**
     * @brief Moving would remove the file twice.
     *
     * @return Never returns.
     */
    TempFile& operator=(TempFile&&) = delete;

    /**
     * @brief Get the path of the file.
     *
     * @return The path.
     */
    const std::filesystem::path& path() const { return this->file_path; }

private:
    /**
     * @brief Path of the file.
     */
    std::filesystem::path file_path;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_TESTS_TEMP_FILE_HPP
