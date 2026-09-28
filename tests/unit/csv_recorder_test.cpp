#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <doctest/doctest.h>

#include "micras/sim/recording/csv_recorder.hpp"
#include "micras/sim/recording/ground_truth.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Ground truth of the tiny robot: one column of each kind the probes cover.
 */
GroundTruthConfig tiny_config() {
    return {
        .body = "robot",
        .columns = {
            {.name = "wheel_speed", .probe = Probe::JOINT_VELOCITY, .object = "wheel"},
            {.name = "wheel_angle", .probe = Probe::JOINT_POSITION, .object = "wheel"},
            {.name = "torque", .probe = Probe::ACTUATOR_FORCE, .object = "motor"},
            {.name = "chassis_ncon", .probe = Probe::CONTACT_COUNT, .object = "chassis"},
            {.name = "chassis_fn", .probe = Probe::CONTACT_NORMAL_FORCE, .object = "chassis"},
            {.name = "chassis_slip", .probe = Probe::CONTACT_SLIP, .object = "chassis"},
            {.name = "chassis_penetration", .probe = Probe::CONTACT_PENETRATION, .object = "chassis"},
            {.name = "iterations", .probe = Probe::SOLVER_ITERATIONS, .object = ""},
        },
    };
}

/**
 * @brief A column source with fixed names and values.
 */
class FixedColumns : public ColumnSource {
public:
    explicit FixedColumns(std::size_t cells_per_row) : cells_per_row{cells_per_row} { }

    std::vector<std::string> names() override { return {"first", "second"}; }

    void append(std::vector<CsvCell>& row) override {
        for (std::size_t i = 0; i < this->cells_per_row; i++) {
            row.emplace_back(static_cast<int64_t>(i));
        }
    }

private:
    std::size_t cells_per_row;
};

/**
 * @brief A column source that can name its columns only once it is told it can.
 */
class LateColumns : public FixedColumns {
public:
    LateColumns() : FixedColumns{2} { }

    std::vector<std::string> names() override { return {"third", "fourth"}; }

    bool ready() const override { return this->started; }

    void start() { this->started = true; }

private:
    bool started{false};
};

class Recording {
protected:
    Recording() {
        this->world.load(MICRAS_TEST_MODEL);
        this->world.reset();
    }

    /**
     * @brief Read back every line of the CSV.
     */
    std::vector<std::string> lines() const {
        std::ifstream            file(this->path);
        std::vector<std::string> read;

        for (std::string line; std::getline(file, line);) {
            read.push_back(line);
        }

        return read;
    }

    /**
     * @brief Read back the first line of the CSV.
     */
    std::string header() const {
        std::ifstream file(this->path);
        std::string   line;
        std::getline(file, line);
        return line;
    }

    // NOLINTBEGIN(*-non-private-member-variables-in-classes): the fixture is the test's own scope.
    MujocoWorld           world;
    std::filesystem::path path{std::filesystem::temp_directory_path() / "micras_sim_recorder_test.csv"};
    // NOLINTEND(*-non-private-member-variables-in-classes)
};

TEST_CASE_FIXTURE(Recording, "Recording.StartsEveryRowWithTheBodyBlockThenTheRobotColumns") {
    const GroundTruth              ground_truth(this->world, tiny_config());
    const std::vector<std::string> columns = ground_truth.columns();

    REQUIRE_EQ(columns.size(), 13U + tiny_config().columns.size());
    CHECK_EQ(columns.front(), "tick");
    CHECK_EQ(columns.at(12), "v_forward");
    CHECK_EQ(columns.at(13), "wheel_speed");
    CHECK_EQ(columns.back(), "iterations");
}

TEST_CASE_FIXTURE(Recording, "Recording.WritesTheTickAndOneCellPerColumn") {
    GroundTruth                ground_truth(this->world, tiny_config());
    const std::vector<CsvCell> row = ground_truth.sample(7);

    REQUIRE_EQ(row.size(), ground_truth.columns().size());
    CHECK_EQ(std::get<uint64_t>(row.at(0)), 7U);
    CHECK_EQ(std::get<double>(row.at(1)), doctest::Approx(this->world.data()->time).epsilon(1e-12));
}

TEST_CASE_FIXTURE(Recording, "Recording.RejectsAnObjectTheModelDoesNotHave") {
    GroundTruthConfig config = tiny_config();
    config.columns.push_back({.name = "ghost", .probe = Probe::JOINT_VELOCITY, .object = "ghost"});

    CHECK_THROWS_AS(GroundTruth(this->world, config), std::runtime_error);
}

TEST_CASE_FIXTURE(Recording, "Recording.AppendsTheSourcesAfterTheGroundTruth") {
    FixedColumns source(2);

    {
        CsvRecorder recorder(this->world, tiny_config(), this->path);
        recorder.add_source(source);
        recorder.sample(0);
    }

    CHECK(this->header().ends_with(",iterations,first,second"));
}

TEST_CASE_FIXTURE(Recording, "Recording.WritesNoRowBeforeEverySourceIsReady") {
    FixedColumns early(2);
    LateColumns  late;

    {
        CsvRecorder recorder(this->world, tiny_config(), this->path);
        recorder.add_source(early);
        recorder.add_source(late);
        recorder.sample(1);
        recorder.sample(2);
        late.start();
        recorder.sample(3);
    }

    const std::vector<std::string> written = this->lines();

    REQUIRE_EQ(written.size(), 2U);
    CHECK(written.at(0).starts_with("tick,"));
    CHECK(written.at(1).starts_with("3,"));
}

TEST_CASE_FIXTURE(Recording, "Recording.RejectsASourceThatFillsTooFewCells") {
    FixedColumns source(1);
    CsvRecorder  recorder(this->world, tiny_config(), this->path);
    recorder.add_source(source);

    CHECK_THROWS_AS(recorder.sample(0), std::logic_error);
}

TEST_CASE_FIXTURE(Recording, "Recording.RejectsTwoSourcesWritingTheSameColumn") {
    FixedColumns first(2);
    FixedColumns second(2);
    CsvRecorder  recorder(this->world, tiny_config(), this->path);
    recorder.add_source(first);
    recorder.add_source(second);

    CHECK_THROWS_AS(recorder.sample(0), std::runtime_error);
}
}  // namespace
}  // namespace micras::sim
