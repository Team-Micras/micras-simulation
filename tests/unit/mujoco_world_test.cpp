#include <cmath>
#include <cstddef>
#include <fstream>
#include <span>

#include <doctest/doctest.h>
#include <mujoco/mjtype.h>

#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/span_at.hpp"
#include "temp_file.hpp"

namespace micras::sim {
namespace {
constexpr double gravity{9.81};
constexpr double coefficient{0.15};

constexpr const char* sled_xml = R"(<mujoco model="sled">
  <option timestep="0.000125" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="sled" pos="0 0 0.01">
      <joint name="along" type="slide" axis="1 0 0"/>
      <joint name="up" type="slide" axis="0 0 1"/>
      <geom name="skid" type="sphere" size="0.01" mass="0.1" condim="1" priority="1"/>
    </body>
  </worldbody>
</mujoco>
)";

class Sled {
protected:
    Sled() {
        std::ofstream(this->file.path()) << sled_xml;
        this->world.load(this->file.path());
        this->world.reset();
        this->world.step(800);
    }

    double& forward_speed() {
        const std::span<mjtNum> velocities(this->world.data()->qvel, static_cast<std::size_t>(this->world.model()->nv));
        return at(velocities, 0);
    }

    // NOLINTBEGIN(*-non-private-member-variables-in-classes): the fixture is the test's own scope.
    TempFile    file{"sled.xml"};
    MujocoWorld world;
    // NOLINTEND(*-non-private-member-variables-in-classes)
};
}  // namespace

TEST_CASE_FIXTURE(Sled, "A frictionless skid keeps sliding") {
    this->forward_speed() = 1.0;
    this->world.step(1600);

    CHECK(this->forward_speed() == doctest::Approx(1.0).epsilon(1.0e-6));
}

TEST_CASE_FIXTURE(Sled, "A skid with a sliding friction slows down at its coefficient times gravity") {
    this->world.add_sliding_friction("skid", coefficient);
    this->forward_speed() = 1.0;
    this->world.step(1600);

    CHECK(this->forward_speed() == doctest::Approx(1.0 - (coefficient * gravity * 0.2)).epsilon(0.01));
}

TEST_CASE_FIXTURE(Sled, "A skid with a sliding friction comes to rest and stays there") {
    this->world.add_sliding_friction("skid", coefficient);
    this->forward_speed() = 1.0;
    this->world.step(8000);

    CHECK(std::abs(this->forward_speed()) < 1.0e-4);
}

TEST_CASE_FIXTURE(Sled, "Loading another model forgets the sliding frictions") {
    this->world.add_sliding_friction("skid", coefficient);
    this->world.load(this->file.path());
    this->world.reset();
    this->world.step(800);
    this->forward_speed() = 1.0;
    this->world.step(1600);

    CHECK(this->forward_speed() == doctest::Approx(1.0).epsilon(1.0e-6));
}
}  // namespace micras::sim
