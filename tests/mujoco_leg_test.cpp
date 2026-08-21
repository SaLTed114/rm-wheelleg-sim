#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "balance/control_core.h"
#include "balance/leg_kinematics.h"
#include "balance/math_utils.h"
#include "generated/mujoco_leg_calibration.hpp"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"

namespace {

struct SideAddresses {
    std::array<
        int, balance::sim::calibration::kModelLegJointCount>
        model_joints{};
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
    const std::array<std::array<
        const char *, balance::sim::calibration::kModelLegJointCount>,
        BC_SIDE_NUM> model_joints{{
        {{
            "Left_front_joint", "Left_front_child1_joint",
            "Left_front_child2_joint", "Left_front_child3_joint",
            "Left_rear_joint", "Left_rear_child1_joint",
        }},
        {{
            "Right_front_joint", "Right_front_child1_joint",
            "Right_front_child2_joint", "Right_front_joint3_joint",
            "Right_rear_joint", "Right_rear_child1_joint",
        }},
    }};
    std::array<SideAddresses, BC_SIDE_NUM> addresses{};

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const int sensor = require_id(
            model, mjOBJ_SENSOR, position_sensors[side]);
        if (model.sensor_dim[sensor] != 3) {
            throw std::runtime_error(
                "leg position sensor must have dimension 3");
        }
        for (std::size_t joint = 0; joint < model_joints[side].size();
             ++joint) {
            addresses[side].model_joints[joint] = require_id(
                model, mjOBJ_JOINT, model_joints[side][joint]);
        }
        addresses[side].front_joint = require_id(
            model, mjOBJ_JOINT, front_joints[side]);
        addresses[side].rear_joint = require_id(
            model, mjOBJ_JOINT, rear_joints[side]);
        addresses[side].front_actuator = require_id(
            model, mjOBJ_ACTUATOR, front_actuators[side]);
        addresses[side].rear_actuator = require_id(
            model, mjOBJ_ACTUATOR, rear_actuators[side]);
        addresses[side].position_sensor = model.sensor_adr[sensor];
    }
    return addresses;
}

bc_leg_kinematics_t calculate_leg(
    const bc_leg_feedback_t &feedback
) {
    const bc_leg_geometry_t geometry{0.175F, 0.208F};
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

bool validate_sensor_contract(
    balance::sim::MujocoPlant &plant,
    const balance::sim::MujocoAdapter &adapter
) {
    constexpr double kAngle = 0.2;
    constexpr double kRate = 0.7;
    constexpr double kTolerance = 1.0e-5;
    const int base_joint = require_id(
        plant.model(), mjOBJ_JOINT, "base_free_joint");
    const int base_qpos = plant.model().jnt_qposadr[base_joint];
    const int base_dof = plant.model().jnt_dofadr[base_joint];

    plant.reset();
    bc_sensor_feedback_t reference{};
    adapter.read(plant.data(), reference);

    const std::array<std::array<double, 3>, 3> local_axes{{
        {{1.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0}},
        {{0.0, 0.0, 1.0}},
    }};
    for (int axis = 0; axis < 3; ++axis) {
        plant.reset();
        double delta_quaternion[4];
        mju_axisAngle2Quat(
            delta_quaternion, local_axes[axis].data(), kAngle);
        double reference_quaternion[4];
        std::copy_n(
            plant.data().qpos + base_qpos + 3, 4,
            reference_quaternion);
        mju_mulQuat(
            plant.data().qpos + base_qpos + 3,
            reference_quaternion, delta_quaternion);
        for (int component = 0; component < 3; ++component) {
            plant.data().qvel[base_dof + 3 + component] =
                kRate * local_axes[axis][component];
        }
        mj_forward(&plant.model(), &plant.data());

        bc_sensor_feedback_t feedback{};
        adapter.read(plant.data(), feedback);
        const double attitude[] = {
            feedback.imu.roll,
            feedback.imu.pitch,
            bc_wrap_angle(feedback.imu.yaw - reference.imu.yaw),
        };
        const double angular_rate[] = {
            feedback.imu.roll_rate,
            feedback.imu.pitch_rate,
            feedback.imu.yaw_rate,
        };
        if (std::abs(attitude[axis] - kAngle) > kTolerance ||
            std::abs(angular_rate[axis] - kRate) > kTolerance) {
            std::cerr << "IMU axis " << axis
                      << " does not follow the FLU contract: attitude="
                      << attitude[axis] << " rate=" << angular_rate[axis]
                      << '\n';
            return false;
        }
    }

    plant.reset();
    mj_forward(&plant.model(), &plant.data());
    const int acceleration_sensor = require_id(
        plant.model(), mjOBJ_SENSOR, "imu_acceleration_sensor");
    const int acceleration_address =
        plant.model().sensor_adr[acceleration_sensor];
    const double raw_specific_force[] = {-1.25, 2.5, 9.5};
    std::copy(
        std::begin(raw_specific_force), std::end(raw_specific_force),
        plant.data().sensordata + acceleration_address);
    bc_sensor_feedback_t acceleration_feedback{};
    adapter.read(plant.data(), acceleration_feedback);
    if (std::abs(acceleration_feedback.imu.specific_force_x - 1.25) >
            kTolerance ||
        std::abs(acceleration_feedback.imu.specific_force_y + 2.5) >
            kTolerance ||
        std::abs(acceleration_feedback.imu.specific_force_z - 9.5) >
            kTolerance) {
        std::cerr << "accelerometer does not follow the FLU contract\n";
        return false;
    }

    const int wheel_joints[] = {
        require_id(plant.model(), mjOBJ_JOINT, "Left_Wheel_joint"),
        require_id(plant.model(), mjOBJ_JOINT, "Right_Wheel_joint"),
    };
    const int wheel_actuators[] = {
        require_id(
            plant.model(), mjOBJ_ACTUATOR, "Left_Wheel_joint_actuator"),
        require_id(
            plant.model(), mjOBJ_ACTUATOR, "Right_Wheel_joint_actuator"),
    };
    plant.reset();
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const double raw_sign = side == BC_L ? 1.0 : -1.0;
        plant.data().qpos[plant.model().jnt_qposadr[wheel_joints[side]]] =
            raw_sign * 0.4;
        plant.data().qvel[plant.model().jnt_dofadr[wheel_joints[side]]] =
            raw_sign * 2.0;
    }
    mj_forward(&plant.model(), &plant.data());
    bc_sensor_feedback_t wheel_feedback{};
    adapter.read(plant.data(), wheel_feedback);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (std::abs(wheel_feedback.wheel[side].angle - 0.4) >
                kTolerance ||
            std::abs(wheel_feedback.wheel[side].angular_velocity - 2.0) >
                kTolerance) {
            std::cerr << "wheel " << side
                      << " does not report positive forward motion\n";
            return false;
        }
    }

    bc_actuation_t actuation{};
    actuation.wheel_torque[BC_L] = 3.0F;
    actuation.wheel_torque[BC_R] = 3.0F;
    adapter.write(plant.data(), actuation);
    if (std::abs(plant.data().ctrl[wheel_actuators[BC_L]] - 3.0) >
            kTolerance ||
        std::abs(plant.data().ctrl[wheel_actuators[BC_R]] + 3.0) >
            kTolerance) {
        std::cerr << "wheel torque output does not preserve the FLU sign\n";
        return false;
    }

    std::cout << "adapter sensor-frame contract passed\n";
    return true;
}

struct ErrorSummary {
    double length_sum{};
    double length_squared_sum{};
    double length_minimum{std::numeric_limits<double>::infinity()};
    double length_maximum{-std::numeric_limits<double>::infinity()};
    double length_maximum_absolute{};
    double angle_sum{};
    double angle_squared_sum{};
    double angle_minimum{std::numeric_limits<double>::infinity()};
    double angle_maximum{-std::numeric_limits<double>::infinity()};
    double angle_maximum_absolute{};
    int count{};

    void add(
        const double calculated_length,
        const double calculated_angle,
        const std::array<double, BC_LEG_COORD_NUM> &measured
    ) {
        const double length =
            calculated_length - measured[BC_LEG_LENGTH];
        const double angle = bc_wrap_angle(
            calculated_angle - measured[BC_LEG_ANGLE]);
        length_sum += length;
        length_squared_sum += length * length;
        length_minimum = std::min(length_minimum, length);
        length_maximum = std::max(length_maximum, length);
        length_maximum_absolute = std::max(
            length_maximum_absolute, std::abs(length));
        angle_sum += angle;
        angle_squared_sum += angle * angle;
        angle_minimum = std::min(angle_minimum, angle);
        angle_maximum = std::max(angle_maximum, angle);
        angle_maximum_absolute = std::max(
            angle_maximum_absolute, std::abs(angle));
        ++count;
    }

    [[nodiscard]] double length_mean() const {
        return length_sum / count;
    }

    [[nodiscard]] double length_rms() const {
        return std::sqrt(length_squared_sum / count);
    }

    [[nodiscard]] double angle_mean() const {
        return angle_sum / count;
    }

    [[nodiscard]] double angle_rms() const {
        return std::sqrt(angle_squared_sum / count);
    }
};

void print_error_summary(
    const char *scope,
    const int side,
    const ErrorSummary &summary
) {
    std::cout << scope << ' ' << (side == BC_L ? "left" : "right")
              << " signed length mean/min/max/rms="
              << 1000.0 * summary.length_mean() << '/'
              << 1000.0 * summary.length_minimum << '/'
              << 1000.0 * summary.length_maximum << '/'
              << 1000.0 * summary.length_rms() << " mm, angle="
              << 180.0 * summary.angle_mean() / BC_PI << '/'
              << 180.0 * summary.angle_minimum / BC_PI << '/'
              << 180.0 * summary.angle_maximum / BC_PI << '/'
              << 180.0 * summary.angle_rms() / BC_PI << " deg\n";
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
        const bc_observation_context_t observation_context{
            BC_WHEEL_OBSERVATION_DISABLED};

        adapter_.read(plant_.data(), feedback);
        bc_control_core_update(
            &control_core_, &feedback, &observation_context,
            static_cast<float>(plant_.timestep()));
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
    constexpr double kLengthTolerance = 0.010;
    constexpr double kAngleTolerance = 2.0 * BC_PI / 180.0;
    double maximum_length_error = 0.0;
    double maximum_angle_error = 0.0;
    std::array<ErrorSummary, BC_SIDE_NUM> summaries{};

    for (std::size_t sample = 0;
         sample < balance::sim::calibration::kStandingSampleCount;
         ++sample) {
        plant.reset();
        enable_base_support(plant);
        for (int step = 0; step < 4000; ++step) {
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                const auto &address = addresses[side];
                const int front_qpos =
                    plant.model().jnt_qposadr[address.front_joint];
                const int rear_qpos =
                    plant.model().jnt_qposadr[address.rear_joint];
                const int front_dof =
                    plant.model().jnt_dofadr[address.front_joint];
                const int rear_dof =
                    plant.model().jnt_dofadr[address.rear_joint];
                const double front_target = balance::sim::calibration::
                    kStandingModelJointPositions[side][sample][0];
                const double rear_target = balance::sim::calibration::
                    kStandingModelJointPositions[side][sample][4];
                const double front_error = front_target -
                    plant.data().qpos[front_qpos];
                const double rear_error = rear_target -
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
            summaries[side].add(
                calculated.length, calculated.angle_body, measured);
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

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        print_error_summary("kinematics", side, summaries[side]);
    }

    std::cout << "kinematics/site maximum errors: length="
              << 1000.0 * maximum_length_error << " mm, angle="
              << 180.0 * maximum_angle_error / BC_PI << " deg\n";
    return maximum_length_error <= kLengthTolerance &&
        maximum_angle_error <= kAngleTolerance;
}

bool validate_standing_calibration(
    balance::sim::MujocoPlant &plant,
    const balance::sim::MujocoAdapter &adapter,
    const std::array<SideAddresses, BC_SIDE_NUM> &addresses
) {
    constexpr double kLengthRmsTolerance = 0.00125;
    constexpr double kLengthMaximumTolerance = 0.002;
    constexpr double kAngleMaximumTolerance = 0.5 * BC_PI / 180.0;
    constexpr double kSideDifferenceTolerance = 0.001;
    std::array<ErrorSummary, BC_SIDE_NUM> summaries{};
    double maximum_side_difference = 0.0;

    for (std::size_t sample = 0;
         sample < balance::sim::calibration::kStandingSampleCount;
         ++sample) {
        plant.reset();
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            for (std::size_t joint = 0;
                 joint < balance::sim::calibration::kModelLegJointCount;
                 ++joint) {
                const int joint_id = addresses[side].model_joints[joint];
                const int qpos = plant.model().jnt_qposadr[joint_id];
                const int dof = plant.model().jnt_dofadr[joint_id];
                plant.data().qpos[qpos] = balance::sim::calibration::
                    kStandingModelJointPositions[side][sample][joint];
                plant.data().qvel[dof] = 0.0;
            }
        }
        mj_forward(&plant.model(), &plant.data());

        bc_sensor_feedback_t feedback{};
        adapter.read(plant.data(), feedback);
        std::array<double, BC_SIDE_NUM> calculated_lengths{};
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            const auto calculated = calculate_leg(feedback.leg[side]);
            const auto measured = measure_leg(
                plant.data(), addresses[side]);
            calculated_lengths[side] = calculated.length;
            summaries[side].add(
                calculated.length, calculated.angle_body, measured);
        }
        maximum_side_difference = std::max(
            maximum_side_difference,
            std::abs(
                calculated_lengths[BC_L] - calculated_lengths[BC_R]));
    }

    bool valid = true;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        print_error_summary("standing calibration", side, summaries[side]);
        valid = valid &&
            summaries[side].length_rms() <= kLengthRmsTolerance &&
            summaries[side].length_maximum_absolute <=
                kLengthMaximumTolerance &&
            summaries[side].angle_maximum_absolute <=
                kAngleMaximumTolerance;
    }
    std::cout << "standing calibration maximum side difference="
              << 1000.0 * maximum_side_difference << " mm\n";
    return valid && maximum_side_difference <= kSideDifferenceTolerance;
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
        command.leg[side].target.length = 0.24F;
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
        const double length_error = std::abs(leg.length - 0.24);
        const double leg_angle_error = angle_error(
            leg.angle_body, -0.5 * BC_PI);
        std::cout << "controlled leg " << side << " errors: length="
                  << 1000.0 * length_error << " mm, angle="
                  << 180.0 * leg_angle_error / BC_PI << " deg\n";
        valid = valid && length_error <= 0.010;
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
        command.leg[side].target.length = 0.24F;
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
    const int base = require_id(
        plant.model(), mjOBJ_BODY, "base_link");
    const int wheels[BC_SIDE_NUM] = {
        require_id(plant.model(), mjOBJ_GEOM, "Left_wheel_collision"),
        require_id(plant.model(), mjOBJ_GEOM, "Right_wheel_collision"),
    };
    std::array<int, BC_SIDE_NUM> contact_steps{};

    TestControlRunner runner(plant, adapter);
    runner.reset();
    enable_base_support(plant);

    bc_control_command_t command{};
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        command.leg[side].length_strategy = BC_LEG_LENGTH_POSITION;
        command.leg[side].angle_strategy = BC_LEG_ANGLE_POSITION;
        command.leg[side].target.length = 0.24F;
        command.leg[side].target.angle_body = -0.5F * BC_PI_F;
    }
    runner.set_command(command);
    for (int step = 0; step < kSettleSteps; ++step) runner.step();
    const double initial_distance = runner.state().value[BC_STATE_S];
    const double initial_mocap_x = plant.data().mocap_pos[3 * mocap];
    const double initial_mocap_y = plant.data().mocap_pos[3 * mocap + 1];
    const mjtNum *base_rotation = plant.data().xmat + 9 * base;
    const double heading_x = base_rotation[0];
    const double heading_y = base_rotation[3];

    double maximum_velocity = 0.0;
    for (int step = 0; step < kMotionSteps; ++step) {
        const double progress = static_cast<double>(step + 1) / kMotionSteps;
        const double displacement =
            0.5 * kTravel * (1.0 - std::cos(BC_PI * progress));
        plant.data().mocap_pos[3 * mocap] =
            initial_mocap_x + displacement * heading_x;
        plant.data().mocap_pos[3 * mocap + 1] =
            initial_mocap_y + displacement * heading_y;
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
    return distance_change > 0.03 &&
        maximum_velocity > 0.01 &&
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
        if (!validate_sensor_contract(plant, adapter)) {
            return EXIT_FAILURE;
        }
        if (!validate_kinematics(plant, adapter, addresses)) {
            std::cerr << "leg kinematics do not match MuJoCo sites\n";
            return EXIT_FAILURE;
        }
        if (!validate_standing_calibration(plant, adapter, addresses)) {
            std::cerr << "standing-area leg calibration is outside tolerance\n";
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
