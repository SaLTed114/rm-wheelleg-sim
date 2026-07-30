#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "balance/leg_kinematics.h"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

struct SideAddresses {
    int front_joint;
    int rear_joint;
    int front_actuator;
    int rear_actuator;
    int position_sensor;
};

int require_id(
    const mjModel &model,
    const mjtObj type,
    const char *name
) {
    const int id = mj_name2id(&model, type, name);
    if (id < 0) {
        throw std::runtime_error(
            "missing MuJoCo object '" + std::string(name) + "'");
    }
    return id;
}

std::array<SideAddresses, BC_SIDE_NUM> resolve_addresses(
    const mjModel &model
) {
    const char *front_joints[] = {
        "Left_front_joint", "Right_front_joint",
    };
    const char *rear_joints[] = {
        "Left_rear_joint", "Right_rear_joint",
    };
    const char *front_actuators[] = {
        "Left_front_joint_actuator", "Right_front_joint_actuator",
    };
    const char *rear_actuators[] = {
        "Left_rear_joint_actuator", "Right_rear_joint_actuator",
    };
    const char *position_sensors[] = {
        "Left_leg_position_sensor", "Right_leg_position_sensor",
    };
    std::array<SideAddresses, BC_SIDE_NUM> addresses{};

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const int sensor = require_id(
            model, mjOBJ_SENSOR, position_sensors[side]);
        if (model.sensor_dim[sensor] != 3) {
            throw std::runtime_error(
                "leg position sensor must have dimension 3");
        }
        addresses[side] = SideAddresses{
            require_id(model, mjOBJ_JOINT, front_joints[side]),
            require_id(model, mjOBJ_JOINT, rear_joints[side]),
            require_id(model, mjOBJ_ACTUATOR, front_actuators[side]),
            require_id(model, mjOBJ_ACTUATOR, rear_actuators[side]),
            model.sensor_adr[sensor],
        };
    }
    return addresses;
}

bc_leg_kinematics_t calculate_leg(
    const bc_leg_feedback_t &feedback
) {
    const bc_leg_geometry_t geometry{0.215F, 0.254F};
    bc_leg_kinematics_t kinematics{};
    bc_leg_kinematics_calculate(
        &geometry, &feedback, &kinematics);
    return kinematics;
}

std::array<double, BC_LEG_COORD_NUM> measure_leg(
    const mjData &data,
    const SideAddresses &address
) {
    const double *position = data.sensordata + address.position_sensor;
    const double x = position[0];
    const double z = position[2];
    return {
        std::hypot(x, z),
        std::atan2(z, -x),
    };
}

double angle_error(const double left, const double right) {
    return std::abs(std::remainder(left - right, 2.0 * kPi));
}

bool validate_kinematics(
    balance::sim::MujocoPlant &plant,
    const balance::sim::MujocoAdapter &adapter,
    const std::array<SideAddresses, BC_SIDE_NUM> &addresses
) {
    constexpr std::array<std::array<double, BC_JOINT_NUM>, 5> kTargets{{
        {{0.25, 0.35}},
        {{0.25, 0.75}},
        {{0.65, 0.35}},
        {{0.65, 0.75}},
        {{0.85, 0.55}},
    }};
    constexpr double kLengthTolerance = 0.010;
    constexpr double kAngleTolerance = 2.0 * kPi / 180.0;
    double maximum_length_error = 0.0;
    double maximum_angle_error = 0.0;

    for (const auto &target : kTargets) {
        plant.reset();
        for (int step = 0; step < 4000; ++step) {
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                const double mirror = side == BC_L ? 1.0 : -1.0;
                const auto &address = addresses[side];
                const int front_qpos =
                    plant.model().jnt_qposadr[address.front_joint];
                const int rear_qpos =
                    plant.model().jnt_qposadr[address.rear_joint];
                const int front_dof =
                    plant.model().jnt_dofadr[address.front_joint];
                const int rear_dof =
                    plant.model().jnt_dofadr[address.rear_joint];
                const double front_error = mirror * target[BC_FRONT] -
                    plant.data().qpos[front_qpos];
                const double rear_error = mirror * target[BC_REAR] -
                    plant.data().qpos[rear_qpos];
                plant.data().ctrl[address.front_actuator] = std::clamp(
                    50.0 * front_error -
                        5.0 * plant.data().qvel[front_dof],
                    -40.0, 40.0);
                plant.data().ctrl[address.rear_actuator] = std::clamp(
                    50.0 * rear_error -
                        5.0 * plant.data().qvel[rear_dof],
                    -40.0, 40.0);
            }
            plant.step();
        }

        bc_observation_t observation{};
        adapter.read(plant.data(), observation);
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            const auto calculated = calculate_leg(observation.leg[side]);
            const auto measured = measure_leg(
                plant.data(), addresses[side]);
            maximum_length_error = std::max(
                maximum_length_error,
                std::abs(calculated.length - measured[BC_LEG_LENGTH]));
            maximum_angle_error = std::max(
                maximum_angle_error,
                angle_error(
                    calculated.angle_body,
                    measured[BC_LEG_ANGLE]));
        }
    }

    std::cout << "kinematics/site maximum errors: length="
              << 1000.0 * maximum_length_error << " mm, angle="
              << 180.0 * maximum_angle_error / kPi << " deg\n";
    return maximum_length_error <= kLengthTolerance &&
        maximum_angle_error <= kAngleTolerance;
}

bool validate_leg_control(
    balance::sim::MujocoPlant &plant,
    const balance::sim::MujocoAdapter &adapter
) {
    balance::sim::SimulationRunner runner(plant, adapter);
    runner.reset();

    bc_operator_command_t command{};
    command.enabled = 1U;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        command.leg[side].length = 0.30F;
        command.leg[side].angle_body = static_cast<float>(-0.5 * kPi);
    }
    runner.set_command(command);
    const auto stats = runner.run_for(8.0);
    if (stats.physics_steps != 8000 || stats.control_ticks != 8000) {
        return false;
    }

    bc_observation_t observation{};
    adapter.read(plant.data(), observation);
    bool valid = true;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const auto leg = calculate_leg(observation.leg[side]);
        const double length_error = std::abs(leg.length - 0.30);
        const double leg_angle_error = angle_error(
            leg.angle_body, -0.5 * kPi);
        std::cout << "controlled leg " << side << " errors: length="
                  << 1000.0 * length_error << " mm, angle="
                  << 180.0 * leg_angle_error / kPi << " deg\n";
        valid = valid && length_error <= 0.025;
        valid = valid && leg_angle_error <= 10.0 * kPi / 180.0;
        valid = valid && std::abs(leg.length_velocity) <= 0.02;
        valid = valid && std::abs(leg.angular_velocity) <= 0.1;
    }

    for (int actuator = 0; actuator < plant.model().nu; ++actuator) {
        valid = valid && std::isfinite(plant.data().ctrl[actuator]);
        valid = valid && std::abs(plant.data().ctrl[actuator]) <= 40.0;
    }
    return valid;
}

} // namespace

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: mujoco_leg_test <model.xml>\n";
        return EXIT_FAILURE;
    }

    try {
        balance::sim::MujocoPlant plant(
            std::filesystem::path(argv[1]), 0.001);
        balance::sim::MujocoAdapter adapter(plant.model());
        const auto addresses = resolve_addresses(plant.model());
        if (!validate_kinematics(plant, adapter, addresses)) {
            std::cerr << "leg kinematics do not match MuJoCo sites\n";
            return EXIT_FAILURE;
        }
        if (!validate_leg_control(plant, adapter)) {
            std::cerr << "leg controller did not settle within tolerance\n";
            return EXIT_FAILURE;
        }
    } catch (const std::exception &error) {
        std::cerr << "mujoco_leg_test: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
