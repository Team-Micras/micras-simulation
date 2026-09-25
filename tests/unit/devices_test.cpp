#include <cmath>
#include <cstdint>
#include <deque>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "micras/sim/core/clock.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/serial_bus.hpp"
#include "micras/sim/devices/dc_motor.hpp"
#include "micras/sim/devices/quadrature_encoder.hpp"
#include "micras/sim/devices/serial_link.hpp"

namespace micras::sim {
namespace {
/**
 * @brief A made-up drive: a 6 V supply, a 10 ohm path, a 5:1 gearbox.
 */
DriveDescription tiny_drive() {
    return {
        .winding_resistance = 9.5,
        .bridge_resistance = 0.5,
        .torque_constant = 0.003,
        .speed_constant = 0.003,
        .no_load_current = 0.007,
        .rotor_inertia = 4.0e-9,
        .gear_ratio = 5.0,
        .gear_efficiency = 0.9,
        .supply_voltage = 6.0,
        .max_motor_speed = 1800.0,
    };
}

/**
 * @brief Listener that keeps every byte the firmware sent.
 */
class Collector : public ISerialListener {
public:
    void on_firmware_bytes(std::span<const uint8_t> bytes) override {
        this->received.insert(this->received.end(), bytes.begin(), bytes.end());
    }

    // NOLINTNEXTLINE(*-non-private-member-variables-in-classes): the test reads what arrived.
    std::vector<uint8_t> received;
};

/**
 * @brief The tiny robot's world and a clock with the usual 1042 us tick.
 */
class Devices : public testing::Test {
protected:
    void SetUp() override {
        this->world.load(MICRAS_TEST_MODEL);
        this->clock = Clock::from_model(this->world.timestep(), 1042);
        this->world.reset();
    }

    /**
     * @brief Get the tiny robot's wheel joint position.
     *
     * @return The joint's entry of qpos.
     */
    mjtNum& wheel_angle() {
        const std::span<mjtNum> positions(this->world.data()->qpos, static_cast<std::size_t>(this->world.model()->nq));
        const std::span<const int> addresses(
            this->world.model()->jnt_qposadr, static_cast<std::size_t>(this->world.model()->njnt)
        );
        return positions[static_cast<std::size_t>(addresses[this->wheel()])];
    }

    /**
     * @brief Get the tiny robot's wheel joint velocity.
     *
     * @return The joint's entry of qvel.
     */
    mjtNum& wheel_speed() {
        const std::span<mjtNum> velocities(this->world.data()->qvel, static_cast<std::size_t>(this->world.model()->nv));
        const std::span<const int> addresses(
            this->world.model()->jnt_dofadr, static_cast<std::size_t>(this->world.model()->njnt)
        );
        return velocities[static_cast<std::size_t>(addresses[this->wheel()])];
    }

    /**
     * @brief Get the control the motor actuator was given.
     *
     * @return The actuator's entry of ctrl.
     */
    double motor_control() const {
        const std::span<const mjtNum> controls(
            this->world.data()->ctrl, static_cast<std::size_t>(this->world.model()->nu)
        );
        return controls[static_cast<std::size_t>(this->world.require_id(mjOBJ_ACTUATOR, "motor"))];
    }

    /**
     * @brief Build a motor on the tiny robot's wheel, driven by the fixture's bridge inputs.
     *
     * @return The motor.
     */
    DcMotor motor() {
        return {
            this->world,
            {.name = "wheel",
             .actuator = "motor",
             .joint = "wheel",
             .drive = tiny_drive(),
             .forward_duty = [this] { return this->forward; },
             .backward_duty = [this] { return this->backward; },
             .enabled = [this] { return this->enabled; }}
        };
    }

    // NOLINTBEGIN(*-non-private-member-variables-in-classes): the fixture is the test's own scope.
    MujocoWorld world;
    Clock       clock;
    float       forward{0.0F};
    float       backward{0.0F};
    bool        enabled{true};
    // NOLINTEND(*-non-private-member-variables-in-classes)

private:
    /**
     * @brief Get the id of the tiny robot's wheel joint.
     *
     * @return The id.
     */
    std::size_t wheel() const { return static_cast<std::size_t>(this->world.require_id(mjOBJ_JOINT, "wheel")); }
};

TEST_F(Devices, AppliesTheDutyCycleDifferenceTimesTheSupply) {
    DcMotor motor = this->motor();

    this->forward = 50.0F;
    motor.actuate(this->world, this->clock);
    EXPECT_DOUBLE_EQ(this->motor_control(), 3.0);

    this->forward = 25.0F;
    this->backward = 75.0F;
    motor.actuate(this->world, this->clock);
    EXPECT_DOUBLE_EQ(this->motor_control(), -3.0);
}

TEST_F(Devices, NeverAppliesMoreThanTheSupply) {
    DcMotor motor = this->motor();

    this->forward = 150.0F;
    motor.actuate(this->world, this->clock);

    EXPECT_DOUBLE_EQ(this->motor_control(), 6.0);
}

TEST_F(Devices, DrivesTheCurrentAgainstTheBackEmf) {
    DcMotor motor = this->motor();
    this->wheel_speed() = 10.0;

    this->forward = 50.0F;
    motor.actuate(this->world, this->clock);

    const double back_emf = 0.003 * 5.0 * 10.0;
    EXPECT_DOUBLE_EQ(motor.current(), (3.0 - back_emf) / 10.0);
}

TEST_F(Devices, BrakesWithBothInputsLow) {
    DcMotor motor = this->motor();
    this->wheel_speed() = 10.0;

    motor.actuate(this->world, this->clock);

    EXPECT_DOUBLE_EQ(this->motor_control(), 0.0);
    EXPECT_LT(motor.current(), 0.0);
}

TEST_F(Devices, LeavesTheWindingOpenWhenTheBridgeIsDisabled) {
    DcMotor motor = this->motor();
    this->wheel_speed() = 10.0;
    this->forward = 100.0F;
    this->enabled = false;

    motor.actuate(this->world, this->clock);

    EXPECT_DOUBLE_EQ(this->motor_control(), 0.003 * 5.0 * 10.0);
    EXPECT_DOUBLE_EQ(motor.current(), 0.0);
}

TEST_F(Devices, RecordsTheMotorsVoltageAndCurrent) {
    DcMotor motor = this->motor();
    this->forward = 50.0F;
    motor.actuate(this->world, this->clock);

    std::vector<CsvCell> row;
    motor.append(row);

    EXPECT_EQ(motor.columns(), (std::vector<std::string>{"wheel_voltage", "wheel_current"}));
    ASSERT_EQ(row.size(), 2U);
    EXPECT_DOUBLE_EQ(std::get<double>(row.at(0)), 3.0);
}

TEST_F(Devices, RefusesAMotorOnAnActuatorTheModelDoesNotHave) {
    EXPECT_THROW(
        DcMotor(
            this->world, {.name = "ghost",
                          .actuator = "ghost",
                          .joint = "wheel",
                          .drive = tiny_drive(),
                          .forward_duty = [] { return 0.0F; },
                          .backward_duty = [] { return 0.0F; },
                          .enabled = [] { return true; }}
        ),
        std::runtime_error
    );
}

TEST_F(Devices, CountsTheWheelAngleAtTheEncodersResolution) {
    std::optional<int32_t> written;
    QuadratureEncoder      encoder(
        this->world, {.name = "wheel",
                           .joint = "wheel",
                           .counts_per_revolution = 4096,
                           .write = [&written](int32_t count) { written = count; }}
    );
    const double radians_per_count = 2.0 * std::numbers::pi / 4096.0;

    this->wheel_angle() = 1024.5 * radians_per_count;
    encoder.sample(this->world, this->clock);
    EXPECT_EQ(written, 1024);

    this->wheel_angle() = (2.5 * 4096.0 + 0.5) * radians_per_count;
    encoder.sample(this->world, this->clock);
    EXPECT_EQ(written, 10240);
}

TEST_F(Devices, CountsDownWhenTheWheelTurnsBackwards) {
    std::optional<int32_t> written;
    QuadratureEncoder      encoder(
        this->world, {.name = "wheel",
                           .joint = "wheel",
                           .counts_per_revolution = 4096,
                           .write = [&written](int32_t count) { written = count; }}
    );

    this->wheel_angle() = -1023.5 * 2.0 * std::numbers::pi / 4096.0;
    encoder.sample(this->world, this->clock);

    EXPECT_EQ(written, -1024);

    std::vector<CsvCell> row;
    encoder.append(row);
    EXPECT_EQ(encoder.columns(), (std::vector<std::string>{"wheel_count"}));
    EXPECT_EQ(std::get<int64_t>(row.at(0)), -1024);
}

/**
 * @brief A link at 115200 baud, fed from a queue the firmware fills.
 */
class Link : public Devices {
protected:
    /**
     * @brief Build the link.
     *
     * @return The link, on the fixture's bus.
     */
    SerialLink link() {
        return {
            this->bus,
            {.baud_rate = 115200,
             .take_sent = [this]() -> std::optional<uint8_t> {
                 if (this->outgoing.empty()) {
                     return std::nullopt;
                 }

                 const uint8_t byte = this->outgoing.front();
                 this->outgoing.pop_front();
                 return byte;
             },
             .receive = [this](uint8_t byte) { this->incoming.push_back(byte); }}
        };
    }

    /**
     * @brief Queue bytes for the firmware to send.
     *
     * @param count Number of bytes, counting up from 0.
     */
    void firmware_sends(std::size_t count) {
        for (std::size_t i = 0; i < count; i++) {
            this->outgoing.push_back(static_cast<uint8_t>(i));
        }
    }

    /**
     * @brief Bytes the line carries in one tick.
     *
     * @return 115200 baud, ten bits a byte, over 1042 us.
     */
    static double bytes_per_tick() { return 115200.0 / 10.0 * 1042e-6; }

    // NOLINTBEGIN(*-non-private-member-variables-in-classes): the fixture is the test's own scope.
    SerialBus            bus;
    Collector            collector;
    std::deque<uint8_t>  outgoing;
    std::vector<uint8_t> incoming;
    // NOLINTEND(*-non-private-member-variables-in-classes)
};

TEST_F(Link, SendsNoMoreThanTheBaudRateAllows) {
    SerialLink link = this->link();
    this->bus.add_listener(this->collector);
    this->firmware_sends(5000);

    link.sample(this->world, this->clock);
    EXPECT_EQ(this->collector.received.size(), static_cast<std::size_t>(bytes_per_tick()));

    for (int tick = 1; tick < 100; tick++) {
        link.sample(this->world, this->clock);
    }

    EXPECT_EQ(this->collector.received.size(), static_cast<std::size_t>(100 * bytes_per_tick()));
    EXPECT_EQ(this->collector.received.at(200), 200U);
}

TEST_F(Link, ReceivesNoMoreThanTheBaudRateAllows) {
    SerialLink                 link = this->link();
    const std::vector<uint8_t> message(5000, 0x55);
    this->bus.queue_for_firmware(message);

    link.sample(this->world, this->clock);
    EXPECT_EQ(this->incoming.size(), static_cast<std::size_t>(bytes_per_tick()));

    for (int tick = 1; tick < 100; tick++) {
        link.sample(this->world, this->clock);
    }

    EXPECT_EQ(this->incoming.size(), static_cast<std::size_t>(100 * bytes_per_tick()));
}

TEST_F(Link, DoesNotSaveUpBandwidthWhileIdle) {
    SerialLink link = this->link();
    this->bus.add_listener(this->collector);

    for (int tick = 0; tick < 100; tick++) {
        link.sample(this->world, this->clock);
    }

    this->firmware_sends(5000);
    this->bus.queue_for_firmware(std::vector<uint8_t>(5000, 0x55));
    link.sample(this->world, this->clock);

    EXPECT_LE(this->collector.received.size(), static_cast<std::size_t>(1.0 + bytes_per_tick()));
    EXPECT_LE(this->incoming.size(), static_cast<std::size_t>(1.0 + bytes_per_tick()));
}

TEST_F(Link, CarriesASlowTrickleWholeAndInOrder) {
    SerialLink link = this->link();
    this->bus.add_listener(this->collector);

    for (int tick = 0; tick < 10; tick++) {
        this->outgoing.push_back(static_cast<uint8_t>(tick));
        link.sample(this->world, this->clock);
    }

    EXPECT_EQ(this->collector.received, (std::vector<uint8_t>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9}));
}

TEST_F(Devices, DrivesADigitalInputsPinWithItsPolarity) {
    std::optional<bool> pin;
    DigitalInput        input({.name = "switch", .active_low = true, .drive = [&pin](bool level) { pin = level; }});

    EXPECT_FALSE(input.is_active());
    EXPECT_EQ(pin, true);

    input.set(true);
    EXPECT_TRUE(input.is_active());
    EXPECT_EQ(pin, false);
    EXPECT_EQ(input.name(), "switch");
}
}  // namespace
}  // namespace micras::sim
