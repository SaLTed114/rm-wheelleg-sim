#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "balance/control_core.h"
#include "balance/leg_kinematics.h"
#include "balance/math_utils.h"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"

namespace {

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
        "Right_front_joint", "Left_front_joint",
    };
    const char *rear_joints[] = {
        "Right_rear_joint", "Left_rear_joint",
    };
    const char *front_actuators[] = {
        "Right_front_joint_actuator", "Left_front_joint_actuator",
    };
    const char *rear_actuators[] = {
        "Right_rear_joint_actuator", "Left_rear_joint_actuator",
    };
    const char *position_sensors[] = {
        "Right_leg_position_sensor", "Left_leg_position_sensor",
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
    return std::abs(bc_wrap_angle(left - right));
}

void enable_base_support(balance::sim::MujocoPlant &plant) {
    plant.set_equality_active("base_support_weld", true);
}

class TestControlRunner {
public:
    TestControlRunner(
        balance::sim::MujocoPlant &plant,
        const balance::sim::MujocoAdapter &adapter
    ) : plant_(plant), adapter_(adapter) {
        bc_control_config_t config{};
        bc_control_default_config(&config);
        bc_control_core_init(&control_core_, &config);
    }

    void reset() {
        plant_.reset();
        bc_control_core_reset(&control_core_);
        command_ = {};
    }

    void set_command(const bc_control_command_t &command) {
        command_ = command;
    }

    void step() {
        bc_sensor_feedback_t feedback{};
        bc_actuation_t actuation{};

        adapter_.read(plant_.data(), feedback);
        bc_control_core_update(
            &control_core_, &feedback,
            static_cast<float>(plant_.timestep()), 0U);
        bc_control_core_calculate(&control_core_, &command_);
        bc_control_core_execute(&control_core_, 1U, &actuation);
        adapter_.write(plant_.data(), actuation);
        plant_.step();
    }

    void run_steps(const int steps) {
        for (int step = 0; step < steps; ++step) this->step();
    }

    [[nodiscard]] const bc_state_vector_t &state() const noexcept {
        return control_core_.observer.state;
    }

    [[nodiscard]] uint32_t tick_count() const noexcept {
        return control_core_.tick_count;
    }

private:
    balance::sim::MujocoPlant &plant_;
    const balance::sim::MujocoAdapter &adapter_;
    bc_control_core_t control_core_{};
    bc_control_command_t command_{};
};

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
    constexpr double kAngleTolerance = 2.0 * BC_PI / 180.0;
    double maximum_length_error = 0.0;
    double maximum_angle_error = 0.0;

    for (const auto &target : kTargets) {
        plant.reset();
        enable_base_support(plant);
        for (int step = 0; step < 4000; ++step) {
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                const double mirror = side == BC_L ? -1.0 : 1.0;
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

        bc_sensor_feedback_t feedback{};
        adapter.read(plant.data(), feedback);
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            const auto calculated = calculate_leg(feedback.leg[side]);
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
              << 180.0 * maximum_angle_error / BC_PI << " deg\n";
    return maximum_length_error <= kLengthTolerance &&
        maximum_angle_error <= kAngleTolerance;
}

bool validate_leg_control(
    balance::sim::MujocoPlant &plant,
    const balance::sim::MujocoAdapter &adapter
) {
    TestControlRunner runner(plant, adapter);
    runner.reset();
    enable_base_support(plant);

    bc_control_command_t command{};
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        command.leg[side].length_strategy = BC_LEG_LENGTH_POSITION;
        command.leg[side].angle_strategy = BC_LEG_ANGLE_POSITION;
        command.leg[side].target.length = 0.30F;
        command.leg[side].target.angle_body = -0.5F * BC_PI_F;
    }
    runner.set_command(command);
    runner.run_steps(8000);
    if (runner.tick_count() != 8000U) return false;

    bc_sensor_feedback_t feedback{};
    adapter.read(plant.data(), feedback);
    bool valid = true;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const auto leg = calculate_leg(feedback.leg[side]);
        const double length_error = std::abs(leg.length - 0.30);
        const double leg_angle_error = angle_error(
            leg.angle_body, -0.5 * BC_PI);
        std::cout << "controlled leg " << side << " errors: length="
                  << 1000.0 * length_error << " mm, angle="
                  << 180.0 * leg_angle_error / BC_PI << " deg\n";
        valid = valid && length_error <= 0.025;
        valid = valid && leg_angle_error <= 10.0 * BC_PI / 180.0;
        valid = valid && std::abs(leg.length_velocity) <= 0.02;
        valid = valid && std::abs(leg.angular_velocity) <= 0.1;
    }

    for (int actuator = 0; actuator < plant.model().nu; ++actuator) {
        valid = valid && std::isfinite(plant.data().ctrl[actuator]);
        valid = valid && std::abs(plant.data().ctrl[actuator]) <= 40.0;
    }
    return valid;
}

bool validate_ground_contact(
    balance::sim::MujocoPlant &plant,
    const balance::sim::MujocoAdapter &adapter
) {
    const int ground = require_id(plant.model(), mjOBJ_GEOM, "ground");
    const int wheels[] = {
        require_id(plant.model(), mjOBJ_GEOM, "Left_wheel_collision"),
        require_id(plant.model(), mjOBJ_GEOM, "Right_wheel_collision"),
    };
    std::array<bool, BC_SIDE_NUM> wheel_contact{};

    TestControlRunner runner(plant, adapter);
    runner.reset();
    enable_base_support(plant);

    bc_control_command_t command{};
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        command.leg[side].length_strategy = BC_LEG_LENGTH_POSITION;
        command.leg[side].angle_strategy = BC_LEG_ANGLE_POSITION;
        command.leg[side].target.length = 0.34F;
        command.leg[side].target.angle_body = -0.5F * BC_PI_F;
    }
    runner.set_command(command);

    for (int step = 0; step < 8000; ++step) {
        runner.step();

        for (int index = 0; index < plant.data().ncon; ++index) {
            const mjContact &contact = plant.data().contact[index];
            const bool ground_contact =
                contact.geom[0] == ground || contact.geom[1] == ground;
            if (!ground_contact) return false;

            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                const bool ground_first = contact.geom[0] == ground &&
                    contact.geom[1] == wheels[side];
                const bool ground_second = contact.geom[1] == ground &&
                    contact.geom[0] == wheels[side];
                const bool ground_and_wheel = ground_first || ground_second;
                if (ground_and_wheel) {
                    wheel_contact[side] = true;
                }
            }
        }
    }

    return wheel_contact[BC_L] && wheel_contact[BC_R];
}

bool validate_forward_odometry(
    balance::sim::MujocoPlant &plant,
    const balance::sim::MujocoAdapter &adapter
) {
    constexpr int kSettleSteps = 3000;
    constexpr int kMotionSteps = 4000;
    constexpr double kTravel = 0.08;
    const int support = require_id(
        plant.model(), mjOBJ_BODY, "base_support");
    const int mocap = plant.model().body_mocapid[support];
    if (mocap < 0) return false;
    const int ground = require_id(plant.model(), mjOBJ_GEOM, "ground");
    const int wheels[BC_SIDE_NUM] = {
        require_id(plant.model(), mjOBJ_GEOM, "Right_wheel_collision"),
        require_id(plant.model(), mjOBJ_GEOM, "Left_wheel_collision"),
    };
    std::array<int, BC_SIDE_NUM> contact_steps{};

    TestControlRunner runner(plant, adapter);
    runner.reset();
    enable_base_support(plant);

    bc_control_command_t command{};
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        command.leg[side].length_strategy = BC_LEG_LENGTH_POSITION;
        command.leg[side].angle_strategy = BC_LEG_ANGLE_POSITION;
        command.leg[side].target.length = 0.34F;
        command.leg[side].target.angle_body = -0.5F * BC_PI_F;
    }
    runner.set_command(command);
    for (int step = 0; step < kSettleSteps; ++step) runner.step();
    const double initial_distance = runner.state().value[BC_STATE_S];

    double maximum_velocity = 0.0;
    for (int step = 0; step < kMotionSteps; ++step) {
        const double progress = static_cast<double>(step + 1) / kMotionSteps;
        plant.data().mocap_pos[3 * mocap] =
            0.5 * kTravel * (1.0 - std::cos(BC_PI * progress));
        runner.step();
        maximum_velocity = std::max(
            maximum_velocity,
            static_cast<double>(runner.state().value[BC_STATE_DS]));

        std::array<bool, BC_SIDE_NUM> contacting{};
        for (int index = 0; index < plant.data().ncon; ++index) {
            const mjContact &contact = plant.data().contact[index];
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                contacting[side] = contacting[side] ||
                    (contact.geom[0] == ground &&
                        contact.geom[1] == wheels[side]) ||
                    (contact.geom[1] == ground &&
                        contact.geom[0] == wheels[side]);
            }
        }
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            if (contacting[side]) contact_steps[side] += 1;
        }
    }

    const double distance_change =
        runner.state().value[BC_STATE_S] - initial_distance;
    std::cout << "forward odometry: delta s=" << distance_change
              << " m, max ds=" << maximum_velocity
              << " m/s, contact steps=" << contact_steps[BC_L]
              << '/' << contact_steps[BC_R] << '\n';
    return distance_change > 0.03 && maximum_velocity > 0.01 &&
        contact_steps[BC_L] > kMotionSteps / 2 &&
        contact_steps[BC_R] > kMotionSteps / 2;
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
        if (!validate_ground_contact(plant, adapter)) {
            std::cerr << "wheel-ground contact filtering is incorrect\n";
            return EXIT_FAILURE;
        }
        if (!validate_forward_odometry(plant, adapter)) {
            std::cerr << "forward motion did not produce positive odometry\n";
            return EXIT_FAILURE;
        }
    } catch (const std::exception &error) {
        std::cerr << "mujoco_leg_test: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
