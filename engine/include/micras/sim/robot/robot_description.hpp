/**
 * @file
 *
 * @brief The physical description of a robot, read from its robot.toml.
 */

#ifndef MICRAS_SIM_ROBOT_ROBOT_DESCRIPTION_HPP
#define MICRAS_SIM_ROBOT_ROBOT_DESCRIPTION_HPP

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace micras::sim {
/**
 * @brief A point or a direction in the robot frame, in meters.
 */
using Vector3 = std::array<double, 3>;

/**
 * @brief A box in the robot frame, given by two opposite corners.
 */
struct BoxPart {
    std::string name;
    Vector3     min{};
    Vector3     max{};
};

/**
 * @brief A cylinder in the robot frame, its axis along robot z.
 */
struct CylinderPart {
    std::string name;
    Vector3     base{};
    double      radius{};
    double      height{};
};

/**
 * @brief A frictionless sphere under the chassis that carries it on the floor.
 *
 * @note A contact point of the simulation, not a part of the robot: a real
 *       chassis that rests on the edge of its board slides on it, and a mesh
 *       edge on a plane is a poor contact in MuJoCo. A skid with no friction
 *       (condim 1) only ever pushes up, so it holds the board off the floor
 *       without braking the robot or lifting itself as a slipping contact does.
 */
struct SkidPart {
    std::string name;
    Vector3     center{};
    double      radius{};
};

/**
 * @brief The rigid chassis: mass properties, outline and the parts that stick out of it.
 */
struct ChassisDescription {
    double                             mass{};
    Vector3                            center_of_mass{};
    Vector3                            inertia{};
    std::vector<std::array<double, 2>> outline;
    double                             board_bottom{};
    double                             board_top{};
    double                             friction{};
    std::vector<BoxPart>               boxes;
    std::vector<CylinderPart>          cylinders;
    std::vector<SkidPart>              skids;
};

/**
 * @brief Both drive wheels, mirrored about the robot's x axis.
 */
struct WheelsDescription {
    double radius{};
    double width{};
    double track{};
    double mass{};
    double spin_inertia{};
    double friction{};
    double torsional_friction{};
    double contact_time_constant{};
    double contact_damping_ratio{};
};

/**
 * @brief One DC motor per wheel, its H-bridge and its gearbox.
 */
struct DriveDescription {
    double winding_resistance{};
    double bridge_resistance{};
    double torque_constant{};
    double speed_constant{};
    double no_load_current{};
    double rotor_inertia{};
    double gear_ratio{};
    double gear_efficiency{};
    double supply_voltage{};

    /**
     * @brief Resistance a motor current flows through: winding and bridge.
     *
     * @return Resistance in ohms.
     */
    double resistance() const { return this->winding_resistance + this->bridge_resistance; }
};

/**
 * @brief The quadrature encoders on the wheel axles.
 */
struct EncoderDescription {
    uint32_t counts_per_revolution{};
};

/**
 * @brief The inertial measurement unit: mounting, axes and datasheet noise.
 */
struct ImuDescription {
    Vector3 position{};

    /**
     * @brief The chip's x and y axes, expressed in the robot frame; z is their cross product.
     */
    std::array<Vector3, 2> axes{};

    double gyro_noise_density{};
    double gyro_bias{};
    double gyro_scale_error{};
    double gyro_resolution{};
    double accel_noise_density{};
    double accel_bias{};
    double accel_resolution{};
    double bandwidth{};
    double output_rate{};
    double clock_error{};
};

/**
 * @brief The suction fan: where it pulls and how hard.
 */
struct FanDescription {
    Vector3 position{};
    double  max_downforce{};
    double  nominal_voltage{};
    double  time_constant{};
};

/**
 * @brief One wall sensor: an emitter and a receiver side by side.
 */
struct WallSensorDescription {
    std::string name;
    Vector3     emitter{};
    Vector3     receiver{};
    double      yaw{};
    int         group{};

    /**
     * @brief Sensitivity of this pair relative to the nominal optics: part spread and alignment.
     */
    double gain{};
};

/**
 * @brief The wall sensors' optics and electronics, shared by every sensor.
 */
struct WallSensorsDescription {
    double emitter_half_angle{};
    double emitter_intensity{};
    double receiver_half_angle{};
    double receiver_responsivity{};
    double load_resistance{};
    double saturation_voltage{};
    double adc_reference{};
    double adc_max_counts{};
    double adc_noise_counts{};
    double settle_fraction{};
    double ambient_irradiance{};
    int    rays{};

    std::vector<WallSensorDescription> sensors;
};

/**
 * @brief The battery the fan and the logic run from.
 */
struct BatteryDescription {
    int    cells{};
    double cell_voltage{};
};

/**
 * @brief The radio link the firmware talks through.
 */
struct LinkDescription {
    uint32_t baud_rate{};
};

/**
 * @brief How the simulator integrates this robot.
 */
struct IntegrationDescription {
    double timestep{};
};

/**
 * @brief Everything physical about one robot.
 *
 * @note Frames follow the firmware's robot model: x forward, y left, z up, the
 *       origin on the floor under the midpoint of the wheel axle. Every length is
 *       in meters, every angle in radians. Each value in the file is a number, an
 *       array, or a table { value = ..., source = "..." } naming where it came
 *       from: a datasheet, the CAD, the firmware, the owner, or an estimate.
 */
struct RobotDescription {
    /**
     * @brief Version of the schema this file follows.
     */
    static constexpr int schema_version{1};

    std::string            name;
    IntegrationDescription integration;
    ChassisDescription     chassis;
    WheelsDescription      wheels;
    DriveDescription       drive;
    EncoderDescription     encoders;
    ImuDescription         imu;
    FanDescription         fan;
    WallSensorsDescription wall_sensors;
    BatteryDescription     battery;
    LinkDescription        link;

    /**
     * @brief Read and check a description.
     *
     * @note Throws on a missing key, an unknown key, a wrong type or a schema
     *       version this build does not know, naming the key.
     *
     * @param path Path of the robot.toml.
     * @return The description.
     */
    static RobotDescription load(const std::filesystem::path& path);

    /**
     * @brief Read and check a description from text.
     *
     * @param text Contents of a robot.toml.
     * @param origin Name of the text, for messages.
     * @return The description.
     */
    static RobotDescription parse(std::string_view text, const std::string& origin = "robot.toml");
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_ROBOT_ROBOT_DESCRIPTION_HPP
