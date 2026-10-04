/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <format>
#include <string>

#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/robot/robot_description.hpp"
#include "micras/sim/robot/robot_model.hpp"

namespace micras::sim {
/**
 * @brief Format a vector for an MJCF attribute.
 *
 * @param vector The vector.
 * @return Space separated components.
 */
static std::string text(const Vector3& vector) {
    return std::format("{:.9g} {:.9g} {:.9g}", vector.at(0), vector.at(1), vector.at(2));
}

/**
 * @brief Generate the geoms of the chassis.
 *
 * @param robot The description.
 * @param names Names of the model.
 * @return MJCF of the geoms.
 */
static std::string chassis_geoms(const RobotDescription& robot, const RobotModelNames& names) {
    const ChassisDescription& chassis = robot.chassis;
    std::string               geoms = std::format(
        "      <geom name=\"{}\" type=\"mesh\" mesh=\"{}_board\" material=\"{}_board\" friction=\"{:.9g}\"/>\n",
        names.board, robot.name, robot.name, chassis.friction
    );

    for (const BoxPart& part : chassis.boxes) {
        const Vector3 center{
            (part.min.at(0) + part.max.at(0)) / 2, (part.min.at(1) + part.max.at(1)) / 2,
            (part.min.at(2) + part.max.at(2)) / 2
        };
        const Vector3 half{
            std::abs(part.max.at(0) - part.min.at(0)) / 2, std::abs(part.max.at(1) - part.min.at(1)) / 2,
            std::abs(part.max.at(2) - part.min.at(2)) / 2
        };

        geoms += std::format(
            "      <geom name=\"{}\" type=\"box\" pos=\"{}\" size=\"{}\" material=\"{}_parts\"/>\n", part.name,
            text(center), text(half), robot.name
        );
    }

    for (const CylinderPart& part : chassis.cylinders) {
        const Vector3 center{part.base.at(0), part.base.at(1), part.base.at(2) + part.height / 2};

        geoms += std::format(
            "      <geom name=\"{}\" type=\"cylinder\" pos=\"{}\" size=\"{:.9g} {:.9g}\" material=\"{}_parts\"/>\n",
            part.name, text(center), part.radius, part.height / 2, robot.name
        );
    }

    for (const SkidPart& part : chassis.skids) {
        geoms += std::format(
            "      <geom name=\"{}\" type=\"sphere\" pos=\"{}\" size=\"{:.9g}\" condim=\"1\" priority=\"1\" "
            "material=\"{}_parts\"/>\n",
            part.name, text(part.center), part.radius, robot.name
        );
    }

    return geoms;
}

/**
 * @brief Generate one wheel body.
 *
 * @param robot The description.
 * @param name Name of the wheel's body, joint and geom.
 * @param side +1 for the left wheel, -1 for the right one.
 * @return MJCF of the wheel.
 *
 * @note The tire has sliding and torsional friction but no rolling friction
 *       (condim 4, not 6). MuJoCo's convex contact separates two surfaces in
 *       proportion to how fast their friction is slipping, and a rolling wheel
 *       keeps a rolling-friction constraint slipping all the time, so the tire
 *       would hop. The same separation during a pivot, where the tires really
 *       do scrub, is absorbed by a soft enough tire, which is part of why
 *       contact_time_constant is what it is.
 */
static std::string wheel_body(const RobotDescription& robot, const std::string& name, double side) {
    const WheelsDescription& wheels = robot.wheels;
    const DriveDescription&  drive = robot.drive;
    const double             armature = drive.rotor_inertia * drive.gear_ratio * drive.gear_ratio;
    const double             friction_torque = drive.torque_constant * drive.no_load_current * drive.gear_ratio;
    const double transverse = wheels.mass * (3 * wheels.radius * wheels.radius + wheels.width * wheels.width) / 12;

    return std::format(
        "      <body name=\"{0}\" pos=\"0 {1:.9g} {2:.9g}\">\n"
        "        <inertial pos=\"0 0 0\" mass=\"{3:.9g}\" diaginertia=\"{4:.9g} {5:.9g} {4:.9g}\"/>\n"
        "        <joint name=\"{0}\" type=\"hinge\" axis=\"0 1 0\" armature=\"{6:.9g}\" frictionloss=\"{7:.9g}\"/>\n"
        "        <geom name=\"{0}\" type=\"cylinder\" size=\"{2:.9g} {8:.9g}\" zaxis=\"0 1 0\" material=\"{9}_tire\"\n"
        "              condim=\"4\" priority=\"1\" friction=\"{10:.9g} {11:.9g}\" solref=\"{12:.9g} {13:.9g}\"/>\n"
        "      </body>\n",
        name, side * wheels.track / 2, wheels.radius, wheels.mass, transverse, wheels.spin_inertia, armature,
        friction_torque, wheels.width / 2, robot.name, wheels.friction, wheels.torsional_friction,
        wheels.contact_time_constant, wheels.contact_damping_ratio
    );
}

/**
 * @brief Generate the sites the devices sample at.
 *
 * @param robot The description.
 * @param names Names of the model.
 * @return MJCF of the sites.
 */
static std::string sites(const RobotDescription& robot, const RobotModelNames& names) {
    const ImuDescription& imu = robot.imu;
    std::string           sites = std::format(
        "      <site name=\"{}\" pos=\"{}\" xyaxes=\"{} {}\"/>\n"
        "      <site name=\"{}\" pos=\"{}\"/>\n",
        names.imu, text(imu.position), text(imu.axes.at(0)), text(imu.axes.at(1)), names.fan, text(robot.fan.position)
    );

    for (const WallSensorDescription& sensor : robot.wall_sensors.sensors) {
        sites += std::format(
            "      <site name=\"{0}_emitter\" pos=\"{1}\" euler=\"0 0 {3:.9g}\" size=\"0.001\" rgba=\"1 0 0 1\"/>\n"
            "      <site name=\"{0}_receiver\" pos=\"{2}\" euler=\"0 0 {3:.9g}\" size=\"0.001\" rgba=\"0 0 1 1\"/>\n",
            sensor.name, text(sensor.emitter), text(sensor.receiver), sensor.yaw
        );
    }

    return sites;
}

/**
 * @brief Generate the board outline as a mesh, extruded between the board's faces.
 *
 * @param robot The description.
 * @return Space separated vertices.
 */
static std::string board_vertices(const RobotDescription& robot) {
    std::string vertices;

    for (const auto& point : robot.chassis.outline) {
        vertices += std::format(
            "{:.9g} {:.9g} {:.9g} {:.9g} {:.9g} {:.9g} ", point.at(0), point.at(1), robot.chassis.board_bottom,
            point.at(0), point.at(1), robot.chassis.board_top
        );
    }

    return vertices;
}

namespace {
/**
 * @brief Height of the onboard camera above the top of the board, in meters.
 *
 * @note Below the top of the walls of a maze, so that the view is the corridor the robot is in.
 */
constexpr double onboard_height{0.025};

/**
 * @brief Distance of the onboard camera ahead of the outline of the board, in meters.
 *
 * @note The sensor housings stand a little past the board, and would fill the bottom of the view.
 */
constexpr double onboard_lead{0.01};
}  // namespace

/**
 * @brief Get where the onboard camera sits ahead of the body origin.
 *
 * @param robot The description.
 * @return The largest forward coordinate of the outline, plus the lead of the camera.
 */
static double front_of(const RobotDescription& robot) {
    double front = 0.0;

    for (const auto& point : robot.chassis.outline) {
        front = std::max(front, static_cast<double>(point.at(0)));
    }

    return front + onboard_lead;
}

std::string robot_mjcf(const RobotDescription& robot) {
    const RobotModelNames   names = RobotModelNames::of(robot);
    const DriveDescription& drive = robot.drive;
    const double            resistance = drive.resistance();
    const double            gain = drive.gear_ratio * drive.gear_efficiency * drive.torque_constant / resistance;
    const double            damping =
        drive.gear_ratio * drive.gear_ratio * drive.torque_constant * drive.speed_constant / resistance;
    const ChassisDescription& chassis = robot.chassis;

    return std::format(
        "<mujoco model=\"{0}\">\n"
        "  <compiler angle=\"radian\"/>\n"
        "  <option timestep=\"{1:.9g}\" integrator=\"implicitfast\" cone=\"elliptic\"/>\n"
        "  <visual>\n"
        "    <global offwidth=\"1920\" offheight=\"1080\"/>\n"
        "    <map znear=\"0.001\"/>\n"
        "  </visual>\n"
        "  <default>\n"
        "    <default class=\"{0}\">\n"
        "      <geom group=\"{2}\"/>\n"
        "    </default>\n"
        "  </default>\n"
        "  <asset>\n"
        "    <mesh name=\"{0}_board\" vertex=\"{3}\"/>\n"
        "    <material name=\"{0}_board\" rgba=\"0.08 0.3 0.12 1\"/>\n"
        "    <material name=\"{0}_parts\" rgba=\"0.85 0.85 0.8 1\"/>\n"
        "    <material name=\"{0}_tire\" rgba=\"0.1 0.1 0.1 1\"/>\n"
        "  </asset>\n"
        "  <worldbody>\n"
        "    <body name=\"{4}\" childclass=\"{0}\">\n"
        "      <freejoint name=\"{4}\"/>\n"
        "      <inertial pos=\"{5}\" mass=\"{6:.9g}\" diaginertia=\"{7}\"/>\n"
        "{8}{9}{10}{11}"
        "      <camera name=\"side tracking\" mode=\"trackcom\" pos=\"-0.3 0.2 0.25\" xyaxes=\"-0.55 -0.83 0 0.4 -0.27 "
        "0.87\"/>\n"
        "      <camera name=\"top tracking\" mode=\"trackcom\" pos=\"0 0 0.7\" xyaxes=\"1 0 0 0 1 0\"/>\n"
        "      <camera name=\"onboard\" pos=\"{26:.9g} 0 {27:.9g}\" xyaxes=\"0 -1 0 0.174 0 0.985\" fovy=\"90\"/>\n"
        "    </body>\n"
        "    <site name=\"{25}\"/>\n"
        "  </worldbody>\n"
        "  <actuator>\n"
        "    <general name=\"{12}\" joint=\"{14}\" gainprm=\"{16:.9g}\" biastype=\"affine\" biasprm=\"0 0 {17:.9g}\"\n"
        "             ctrllimited=\"true\" ctrlrange=\"{18:.9g} {19:.9g}\"/>\n"
        "    <general name=\"{13}\" joint=\"{15}\" gainprm=\"{16:.9g}\" biastype=\"affine\" biasprm=\"0 0 {17:.9g}\"\n"
        "             ctrllimited=\"true\" ctrlrange=\"{18:.9g} {19:.9g}\"/>\n"
        "    <general name=\"{20}\" site=\"{20}\" refsite=\"{25}\" gear=\"0 0 -1 0 0 0\" ctrllimited=\"true\" "
        "ctrlrange=\"0 "
        "{21:.9g}\"/>\n"
        "  </actuator>\n"
        "  <sensor>\n"
        "    <gyro name=\"{22}\" site=\"{24}\"/>\n"
        "    <accelerometer name=\"{23}\" site=\"{24}\"/>\n"
        "  </sensor>\n"
        "</mujoco>\n",
        robot.name, robot.integration.timestep, MujocoWorld::unseen_group, board_vertices(robot), names.body,
        text(chassis.center_of_mass), chassis.mass, text(chassis.inertia), chassis_geoms(robot, names),
        sites(robot, names), wheel_body(robot, names.left_wheel, 1.0), wheel_body(robot, names.right_wheel, -1.0),
        names.left_motor, names.right_motor, names.left_wheel, names.right_wheel, gain, -damping,
        -2 * drive.supply_voltage, 2 * drive.supply_voltage, names.fan, 4 * robot.fan.max_downforce, names.gyro,
        names.accelerometer, names.imu, names.fan_reference, front_of(robot), chassis.board_top + onboard_height
    );
}
}  // namespace micras::sim
