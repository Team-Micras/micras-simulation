/**
 * @file
 */

#include <algorithm>
#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "micras/sim/app/wiring.hpp"
#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/span_at.hpp"
#include "micras/sim/devices/dc_motor.hpp"
#include "micras/sim/devices/digital_input.hpp"
#include "micras/sim/devices/imu.hpp"
#include "micras/sim/devices/power.hpp"
#include "micras/sim/devices/quadrature_encoder.hpp"
#include "micras/sim/devices/serial_link.hpp"
#include "micras/sim/devices/wall_sensors.hpp"
#include "micras/sim/recording/csv_writer.hpp"
#include "micras/sim/recording/ground_truth.hpp"
#include "micras/sim/recording/run_metadata.hpp"
#include "micras/sim/robot/robot_description.hpp"
#include "micras/sim/robot/robot_model.hpp"
#include "micras/sim/scenario/scenario.hpp"
#include "micras/sim/toy/toy_target.hpp"
#include "micras/sim/view/panel_spec.hpp"

namespace micras::sim {
namespace {
constexpr std::array<const char*, 4> state_name_table{"WAIT", "DRIVE", "TURN", "STOP"};

constexpr uint32_t loop_period_us{1000};

constexpr float drive_duty{12.0F};

constexpr float turn_duty{6.0F};

constexpr float fan_duty{50.0F};

constexpr double front_threshold{400.0};

constexpr std::size_t front_reading{3};

constexpr double adc_reference{3.3};

constexpr double adc_max_counts{4095.0};

constexpr double current_zero_voltage{1.65};

constexpr double volts_per_amp{0.5};
}  // namespace

static std::vector<std::string> state_names() {
    return {state_name_table.begin(), state_name_table.end()};
}

template <typename T>
static T* add(RunContext& context, std::unique_ptr<T> device) {
    T* view = device.get();
    context.devices.push_back(std::move(device));
    return view;
}

ToyVariables::ToyVariables(const ToyReadings& readings) : readings{readings} { }

std::vector<std::string> ToyVariables::names() {
    return {"state", "odometry", "heading", "front", "voltage", "current_left", "current_right"};
}

void ToyVariables::append(std::vector<CsvCell>& row) {
    row.emplace_back(static_cast<int64_t>(std::to_underlying(this->readings.state)));
    row.emplace_back(this->readings.odometry);
    row.emplace_back(this->readings.heading);
    row.emplace_back(this->readings.front);
    row.emplace_back(this->readings.voltage);
    row.emplace_back(this->readings.currents.at(0));
    row.emplace_back(this->readings.currents.at(1));
}

double ToyVariables::value_of(const std::string& name) const {
    if (name == "state") {
        return static_cast<double>(std::to_underlying(this->readings.state));
    }

    if (name == "odometry") {
        return this->readings.odometry;
    }

    if (name == "heading") {
        return this->readings.heading;
    }

    if (name == "front") {
        return this->readings.front;
    }

    if (name == "voltage") {
        return this->readings.voltage;
    }

    if (name == "current_left") {
        return this->readings.currents.at(0);
    }

    if (name == "current_right") {
        return this->readings.currents.at(1);
    }

    throw std::runtime_error("the toy has no variable named " + name);
}

std::string ToyTarget::name() const {
    return "toy";
}

std::string ToyTarget::firmware_sha() const {
    return "none";
}

uint32_t ToyTarget::loop_time_us() const {
    return loop_period_us;
}

std::vector<CliOption> ToyTarget::options() {
    return {{
        .name = "--turn-angle",
        .argument = "<degrees>",
        .apply = [this](const std::string& value) {
            double            degrees = 0.0;
            const char* const end = std::to_address(value.end());
            const auto        result = std::from_chars(value.data(), end, degrees);

            if (result.ec != std::errc{} or result.ptr != end or degrees <= 0.0) {
                throw std::runtime_error("--turn-angle expects a positive number of degrees, got '" + value + "'");
            }

            this->turn_angle = degrees * std::numbers::pi / 180.0;
        },
    }};
}

RunContext& ToyTarget::context() {
    return this->run_context;
}

std::filesystem::path ToyTarget::directory() const {
    return std::filesystem::path{MICRAS_SIM_TOY_DIR};
}

std::filesystem::path ToyTarget::robot_file() const {
    return this->directory() / "robot.toml";
}

GroundTruthConfig ToyTarget::ground_truth() const {
    return {
        .body = "toy",
        .columns = {
            {.name = "wheel_speed_left", .probe = Probe::JOINT_VELOCITY, .object = "left_wheel"},
            {.name = "wheel_speed_right", .probe = Probe::JOINT_VELOCITY, .object = "right_wheel"},
            {.name = "motor_torque_left", .probe = Probe::ACTUATOR_FORCE, .object = "motor_left"},
            {.name = "motor_torque_right", .probe = Probe::ACTUATOR_FORCE, .object = "motor_right"},
            {.name = "left_ncon", .probe = Probe::CONTACT_COUNT, .object = "left_wheel"},
            {.name = "left_slip", .probe = Probe::CONTACT_SLIP, .object = "left_wheel"},
            {.name = "right_ncon", .probe = Probe::CONTACT_COUNT, .object = "right_wheel"},
            {.name = "right_slip", .probe = Probe::CONTACT_SLIP, .object = "right_wheel"},
            {.name = "solver_niter", .probe = Probe::SOLVER_ITERATIONS, .object = ""},
        },
    };
}

std::string ToyTarget::video_camera() const {
    return "top tracking";
}

FirmwareThread::Program ToyTarget::program() {
    return [this](FirmwareThread& thread) { this->run(thread); };
}

Wiring ToyTarget::wire(FirmwareThread& /*firmware*/, const WorldInfo& world) {
    const RobotDescription& robot = *world.robot;
    const RobotModelNames   names = RobotModelNames::of(robot);
    RunContext&             context = this->run_context;
    ToyBoard&               hardware = this->board;

    this->wheel_radius = robot.wheels.radius;
    this->counts_per_revolution = static_cast<double>(robot.encoders.counts_per_revolution);
    this->gyro_resolution = robot.imu.gyro_resolution;

    const std::array<std::string, 2> sides{"left", "right"};
    std::array<const DcMotor*, 2>    motors{};

    for (std::size_t index = 0; index < sides.size(); index++) {
        const std::string& side = sides.at(index);

        motors.at(index) =
            add(context,
                std::make_unique<DcMotor>(
                    context.world,
                    DcMotor::Config{
                        .name = "motor_" + side,
                        .actuator = index == 0 ? names.left_motor : names.right_motor,
                        .joint = index == 0 ? names.left_wheel : names.right_wheel,
                        .drive = robot.drive,
                        .forward_duty = [&hardware, index] { return std::max(hardware.motor_duty.at(index), 0.0F); },
                        .backward_duty = [&hardware, index] { return std::max(-hardware.motor_duty.at(index), 0.0F); },
                        .enabled = [&hardware] { return hardware.motors_enabled; },
                    }
                ));

        add(context, std::make_unique<QuadratureEncoder>(
                         context.world,
                         QuadratureEncoder::Config{
                             .name = "encoder_" + side,
                             .joint = index == 0 ? names.left_wheel : names.right_wheel,
                             .counts_per_revolution = robot.encoders.counts_per_revolution,
                             .write = [&hardware, index](int32_t count) { hardware.encoder_counts.at(index) = count; },
                         }
                     ));
    }

    add(context, std::make_unique<Imu>(
                     context.world,
                     Imu::Config{
                         .name = "imu",
                         .gyro = names.gyro,
                         .accelerometer = names.accelerometer,
                         .description = robot.imu,
                         .write = [&hardware](std::span<const float> sample) { hardware.gyro_z_word = at(sample, 2); },
                     },
                     context.noise
                 ));

    add(context,
        std::make_unique<WallSensors>(
            context.world,
            WallSensors::Config{
                .name = "wall",
                .description = robot.wall_sensors,
                .scan_ticks = 1,
                .emitter_duty = [&hardware](std::size_t /*sensor*/) { return hardware.emitters_on ? 100.0F : 0.0F; },
                .write = [&hardware](std::size_t index, uint32_t counts) { hardware.wall_counts.at(index) = counts; },
                .finish_sequence = [] { },
                .reflectance = world.reflectance,
            },
            context.noise
        ));

    const Battery* pack =
        add(context, std::make_unique<Battery>(
                         Battery::Config{
                             .name = "pack",
                             .description = robot.battery,
                             .divider = this->battery_divider,
                             .adc_reference = adc_reference,
                             .adc_max_counts = adc_max_counts,
                             .adc_noise_counts = 1.0,
                             .write = [&hardware](uint32_t counts) { hardware.battery_counts = counts; },
                         },
                         context.noise
                     ));
    this->battery = pack;

    add(context, std::make_unique<Fan>(
                     context.world, Fan::Config{
                                        .name = "fan",
                                        .actuator = names.fan,
                                        .description = robot.fan,
                                        .duty = [&hardware] { return hardware.fan_duty; },
                                        .enabled = [&hardware] { return hardware.fan_duty > 0.0F; },
                                        .supply_voltage = [pack] { return pack->voltage(); },
                                    }
                 ));

    add(context,
        std::make_unique<CurrentSense>(
            CurrentSense::Config{
                .currents = {
                    [motors] { return motors.at(0)->current(); }, [motors] { return motors.at(1)->current(); }
                },
                .zero_voltage = current_zero_voltage,
                .volts_per_amp = volts_per_amp,
                .adc_reference = adc_reference,
                .adc_max_counts = adc_max_counts,
                .adc_noise_counts = 2.0,
                .write =
                    [&hardware](std::size_t index, uint32_t counts) { hardware.current_counts.at(index) = counts; },
            },
            context.noise
        ));

    add(context, std::make_unique<SerialLink>(
                     context.serial, SerialLink::Config{
                                         .baud_rate = robot.link.baud_rate,
                                         .take_sent = [&hardware]() -> std::optional<uint8_t> {
                                             if (hardware.to_send.empty()) {
                                                 return std::nullopt;
                                             }

                                             const uint8_t byte = hardware.to_send.front();
                                             hardware.to_send.pop_front();
                                             return byte;
                                         },
                                         .receive = [&hardware](uint8_t byte) { hardware.received.push_back(byte); },
                                     }
                 ));

    this->button =
        add(context, std::make_unique<DigitalInput>(DigitalInput::Config{
                         .name = "button",
                         .active_low = false,
                         .drive = [&hardware](bool level) { hardware.button_level = level; },
                     }));

    return {
        .columns = {&this->variables},
        .variables = &this->variables,
        .panel = this->make_panel(),
        .overlay =
            {.state = StateLabel{.variable = "state", .names = state_names()},
             .lines = {{.label = "odometry", .variable = "odometry", .unit = "m"}}},
        .hooks = this->make_hooks(),
    };
}

std::vector<MetadataField> ToyTarget::metadata() const {
    return {{.name = "bytes_sent", .value = this->bytes_sent}};
}

void ToyTarget::run(FirmwareThread& thread) {
    const double dt = static_cast<double>(loop_period_us) * 1e-6;

    while (true) {
        this->read_sensors(dt);

        const std::string command = this->read_link();
        const bool        pressed = this->board.button_level;
        const bool        press = pressed and not this->was_pressed;
        this->was_pressed = pressed;

        switch (this->readings.state) {
            case ToyState::WAIT:
                if (press or command == "go") {
                    this->readings.state = ToyState::DRIVE;
                    this->send("driving\n");
                }

                break;
            case ToyState::DRIVE:
                if (this->readings.front >= front_threshold) {
                    this->readings.state = ToyState::TURN;
                    this->turn_start = this->readings.heading;
                }

                break;
            case ToyState::TURN:
                if (this->turn_start - this->readings.heading >= this->turn_angle) {
                    this->readings.state = ToyState::DRIVE;
                }

                break;
            case ToyState::STOP:
                break;
        }

        if (command == "stop" and this->readings.state != ToyState::STOP) {
            this->readings.state = ToyState::STOP;
            this->send("stopped\n");
        }

        this->actuate();
        thread.yield_tick();
    }
}

void ToyTarget::read_sensors(double dt) {
    const std::array<int32_t, 2>& counts = this->board.encoder_counts;
    const double revolutions = static_cast<double>(counts.at(0) + counts.at(1)) / (2.0 * this->counts_per_revolution);

    this->readings.odometry = revolutions * 2.0 * std::numbers::pi * this->wheel_radius;

    if (this->readings.state == ToyState::TURN) {
        this->readings.heading += static_cast<double>(this->board.gyro_z_word) * this->gyro_resolution * dt;
    }

    this->readings.front = static_cast<double>(this->board.wall_counts.at(front_reading));
    this->readings.voltage =
        static_cast<double>(this->board.battery_counts) / adc_max_counts * adc_reference * this->battery_divider;

    for (std::size_t index = 0; index < this->readings.currents.size(); index++) {
        const double volts = static_cast<double>(this->board.current_counts.at(index)) / adc_max_counts * adc_reference;
        this->readings.currents.at(index) = (volts - current_zero_voltage) / volts_per_amp;
    }
}

std::string ToyTarget::read_link() {
    std::string completed;

    while (not this->board.received.empty()) {
        this->command.push_back(static_cast<char>(this->board.received.front()));
        this->board.received.pop_front();

        if (this->command.ends_with("go")) {
            completed = "go";
            this->command.clear();
        } else if (this->command.ends_with("stop")) {
            completed = "stop";
            this->command.clear();
        } else if (this->command.size() > 4) {
            this->command.erase(0, this->command.size() - 4);
        }
    }

    return completed;
}

void ToyTarget::send(const std::string& text) {
    for (const char character : text) {
        this->board.to_send.push_back(static_cast<uint8_t>(character));
    }

    this->bytes_sent += static_cast<int64_t>(text.size());
}

void ToyTarget::actuate() {
    ToyBoard& hardware = this->board;
    hardware.emitters_on = true;

    switch (this->readings.state) {
        case ToyState::DRIVE:
            hardware.motor_duty = {drive_duty, drive_duty};
            hardware.motors_enabled = true;
            hardware.fan_duty = fan_duty;
            break;
        case ToyState::TURN:
            hardware.motor_duty = {turn_duty, -turn_duty};
            hardware.motors_enabled = true;
            hardware.fan_duty = fan_duty;
            break;
        case ToyState::WAIT:
        case ToyState::STOP:
            hardware.motor_duty = {0.0F, 0.0F};
            hardware.motors_enabled = false;
            hardware.fan_duty = 0.0F;
            break;
    }
}

PanelSpec ToyTarget::make_panel() const {
    DigitalInput*      input = this->button;
    const Battery*     pack = this->battery;
    const ToyReadings& known = this->readings;
    PanelSpec          panel{
        .state = StateLabel{.variable = "state", .names = state_names()},
        .buttons = {{.name = "button", .press = [input](bool pressed) { input->set(pressed); }}},
        .switches = {},
        .lamps = {},
        .readouts = {{.name = "battery", .text = [pack] { return std::format("{:.2f} V", pack->voltage()); }}},
        .plots = {"odometry", "heading", "front"},
        .take_over = {},
    };

    for (std::size_t index = 0; index < state_name_table.size(); index++) {
        panel.lamps.push_back({.name = state_name_table.at(index), .color = [&known, index] {
                                   return std::to_underlying(known.state) == index ?
                                              Color{.red = 60, .green = 220, .blue = 60} :
                                              Color{.red = 40, .green = 40, .blue = 40};
                               }});
    }

    return panel;
}

ScenarioHooks ToyTarget::make_hooks() const {
    ScenarioHooks hooks;
    hooks.inputs.emplace("button", this->button);
    hooks.messages.emplace("go", std::vector<uint8_t>{'g', 'o'});
    hooks.messages.emplace("stop", std::vector<uint8_t>{'s', 't', 'o', 'p'});
    hooks.state_names.emplace("state", state_names());
    return hooks;
}
}  // namespace micras::sim
