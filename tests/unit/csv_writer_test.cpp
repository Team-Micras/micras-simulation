#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>

#include <gtest/gtest.h>

#include "micras/sim/core/text_file.hpp"
#include "micras/sim/recording/csv_writer.hpp"

namespace micras::sim {
namespace {
TEST(CsvWriter, FormatsCellsWithTwelveSignificantDigits) {
    EXPECT_EQ(CsvWriter::format(CsvCell{0.001042}), "0.001042");
    EXPECT_EQ(CsvWriter::format(CsvCell{3.99920000001}), "3.9992");
    EXPECT_EQ(CsvWriter::format(CsvCell{0.00080690123456}), "0.000806901235");
    EXPECT_EQ(CsvWriter::format(CsvCell{int64_t{-7}}), "-7");
    EXPECT_EQ(CsvWriter::format(CsvCell{uint64_t{7677}}), "7677");
}

TEST(CsvWriter, FormatsTheSpecialValuesTheBaselineDependsOn) {
    EXPECT_EQ(CsvWriter::format(CsvCell{std::nan("")}), "nan");
    EXPECT_EQ(CsvWriter::format(CsvCell{-std::nan("")}), "-nan");
    EXPECT_EQ(CsvWriter::format(CsvCell{std::numeric_limits<double>::infinity()}), "inf");
    EXPECT_EQ(CsvWriter::format(CsvCell{-std::numeric_limits<double>::infinity()}), "-inf");
    EXPECT_EQ(CsvWriter::format(CsvCell{-0.0}), "-0");
    EXPECT_EQ(CsvWriter::format(CsvCell{0.0}), "0");
    EXPECT_EQ(CsvWriter::format(CsvCell{1e20}), "1e+20");
    EXPECT_EQ(CsvWriter::format(CsvCell{1e-5}), "1e-05");
    EXPECT_EQ(CsvWriter::format(CsvCell{9.99999999e-5}), "9.99999999e-05");
}

TEST(CsvWriter, WritesHeaderAndRows) {
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "micras_csv_writer_test.csv";

    {
        CsvWriter                      writer(path);
        const std::vector<std::string> columns{"tick", "x", "count"};
        writer.write_header(columns);
        writer.write_row(std::vector<CsvCell>{uint64_t{0}, 0.5, int64_t{2}});
        writer.write_row(std::vector<CsvCell>{uint64_t{1}, std::nan(""), int64_t{0}});
    }

    EXPECT_EQ(read_text_file(path, "CSV"), "tick,x,count\n0,0.5,2\n1,nan,0\n");
    std::filesystem::remove(path);
}

TEST(CsvWriter, ThrowsWhenTheFileCannotBeOpened) {
    EXPECT_THROW(CsvWriter("/nonexistent/directory/file.csv"), std::runtime_error);
}
}  // namespace
}  // namespace micras::sim
