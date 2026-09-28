#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <doctest/doctest.h>

#include "micras/sim/core/text_file.hpp"
#include "micras/sim/recording/csv_writer.hpp"
#include "temp_file.hpp"

namespace micras::sim {
namespace {
TEST_CASE("CsvWriter.FormatsCellsWithTwelveSignificantDigits") {
    CHECK_EQ(CsvWriter::format(CsvCell{0.001042}), "0.001042");
    CHECK_EQ(CsvWriter::format(CsvCell{3.99920000001}), "3.9992");
    CHECK_EQ(CsvWriter::format(CsvCell{0.00080690123456}), "0.000806901235");
    CHECK_EQ(CsvWriter::format(CsvCell{int64_t{-7}}), "-7");
    CHECK_EQ(CsvWriter::format(CsvCell{uint64_t{7677}}), "7677");
}

TEST_CASE("CsvWriter.FormatsTheSpecialValuesTheBaselineDependsOn") {
    CHECK_EQ(CsvWriter::format(CsvCell{std::nan("")}), "nan");
    CHECK_EQ(CsvWriter::format(CsvCell{-std::nan("")}), "-nan");
    CHECK_EQ(CsvWriter::format(CsvCell{std::numeric_limits<double>::infinity()}), "inf");
    CHECK_EQ(CsvWriter::format(CsvCell{-std::numeric_limits<double>::infinity()}), "-inf");
    CHECK_EQ(CsvWriter::format(CsvCell{-0.0}), "-0");
    CHECK_EQ(CsvWriter::format(CsvCell{0.0}), "0");
    CHECK_EQ(CsvWriter::format(CsvCell{1e20}), "1e+20");
    CHECK_EQ(CsvWriter::format(CsvCell{1e-5}), "1e-05");
    CHECK_EQ(CsvWriter::format(CsvCell{9.99999999e-5}), "9.99999999e-05");
}

TEST_CASE("CsvWriter.WritesHeaderAndRows") {
    const TempFile csv{"csv_writer_test.csv"};

    {
        CsvWriter                      writer(csv.path());
        const std::vector<std::string> columns{"tick", "x", "count"};
        writer.write_header(columns);
        writer.write_row(std::vector<CsvCell>{uint64_t{0}, 0.5, int64_t{2}});
        writer.write_row(std::vector<CsvCell>{uint64_t{1}, std::nan(""), int64_t{0}});
    }

    CHECK_EQ(read_text_file(csv.path(), "CSV"), "tick,x,count\n0,0.5,2\n1,nan,0\n");
}

TEST_CASE("CsvWriter.ThrowsWhenTheFileCannotBeOpened") {
    CHECK_THROWS_AS(CsvWriter("/nonexistent/directory/file.csv"), std::runtime_error);
}
}  // namespace
}  // namespace micras::sim
