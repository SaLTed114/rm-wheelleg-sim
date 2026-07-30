#include "mujoco_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace balance::sim {
namespace {

struct ChannelSpec {
    const char *joint_name;
    const char *actuator_name;
    double scale;
    double offset;
};

constexpr std::array<std::array<ChannelSpec, BC_JOINT_NUM>, BC_SIDE_NUM>
    kJointSpecs{{
        {{{"Right_front_joint", "Right_front_joint_actuator",
           -1.0, -3.032150759729568},
          {"Right_rear_joint", "Right_rear_joint_actuator",
           +1.0, -0.067812378106530}}},
        {{{"Left_front_joint", "Left_front_joint_actuator",
           +1.0, -3.030735772282508},
          {"Left_rear_joint", "Left_rear_joint_actuator",
           -1.0, -0.032996075602418}}},
    }};

constexpr std::array<ChannelSpec, BC_SIDE_NUM> kWheelSpecs{{
    {"Right_Wheel_joint", "Right_Wheel_joint_actuator", -1.0, 0.0},
    {"Left_Wheel_joint", "Left_Wheel_joint_actuator", +1.0, 0.0},
}};

constexpr std::size_t kActuatorNum =
    BC_SIDE_NUM * (BC_JOINT_NUM + 1);

int require_named_id(
    const mjModel &model, const mjtObj type, const char *name
) {
    const int id = mj_name2id(&model, type, name);
    if (id < 0) {
        throw std::runtime_error(
            "MuJoCo model is missing required object '" +
            std::string(name) + "'");
    }
    return id;
}

template <std::size_t Size>
void require_unique(
    const std::array<int, Size> &ids, const char *kind
) {
    auto sorted = ids;
    std::sort(sorted.begin(), sorted.end());
    if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) {
        throw std::runtime_error(std::string("duplicate ") + kind + " mapping");
    }
}

} // namespace

MujocoAdapter::ChannelAddress MujocoAdapter::resolve_channel(
    const mjModel &model,
    const char *joint_name,
    const char *actuator_name,
    const double scale,
    const double offset
) {
    const int joint_id = require_named_id(model, mjOBJ_JOINT, joint_name);
    const int actuator_id = require_named_id(
        model, mjOBJ_ACTUATOR, actuator_name);

    if (model.jnt_type[joint_id] != mjJNT_HINGE) {
        throw std::runtime_error(
            "required joint is not a hinge: " + std::string(joint_name));
    }
    if (model.actuator_trnid[2 * actuator_id] != joint_id) {
        throw std::runtime_error(
            "actuator does not drive its expected joint: " +
            std::string(actuator_name));
    }

    return ChannelAddress{
        model.jnt_qposadr[joint_id],
        model.jnt_dofadr[joint_id],
        actuator_id,
        scale,
        offset,
    };
}

int MujocoAdapter::resolve_sensor(
    const mjModel &model, const char *name, const int dimension
) {
    const int sensor = require_named_id(model, mjOBJ_SENSOR, name);
    if (model.sensor_dim[sensor] != dimension) {
        throw std::runtime_error(
            "unexpected dimension for sensor '" + std::string(name) + "'");
    }
    return model.sensor_adr[sensor];
}

MujocoAdapter::MujocoAdapter(const mjModel &model)
    : actuator_count_(model.nu) {
    std::array<int, kActuatorNum> qpos_addresses{};
    std::array<int, kActuatorNum> dof_addresses{};
    std::array<int, kActuatorNum> actuator_ids{};
    std::size_t channel = 0;

    for (std::size_t side = 0; side < BC_SIDE_NUM; ++side) {
        for (std::size_t joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const auto &spec = kJointSpecs[side][joint];
            const auto address = resolve_channel(
                model, spec.joint_name, spec.actuator_name,
                spec.scale, spec.offset);
            joint_addresses_[side][joint] = address;
            qpos_addresses[channel] = address.qpos;
            dof_addresses[channel] = address.dof;
            actuator_ids[channel] = address.actuator;
            ++channel;
        }

        const auto &spec = kWheelSpecs[side];
        const auto address = resolve_channel(
            model, spec.joint_name, spec.actuator_name,
            spec.scale, spec.offset);
        wheel_addresses_[side] = address;
        qpos_addresses[channel] = address.qpos;
        dof_addresses[channel] = address.dof;
        actuator_ids[channel] = address.actuator;
        ++channel;
    }

    require_unique(qpos_addresses, "joint position");
    require_unique(dof_addresses, "joint velocity");
    require_unique(actuator_ids, "actuator");

    imu_attitude_address_ = resolve_sensor(
        model, "imu_attitude_sensor", 4);
    imu_gyro_address_ = resolve_sensor(model, "imu_gyro_sensor", 3);
}

void MujocoAdapter::read(
    const mjData &data,
    bc_sensor_feedback_t &feedback
) const {
    for (std::size_t side = 0; side < BC_SIDE_NUM; ++side) {
        for (std::size_t joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const auto &address = joint_addresses_[side][joint];
            auto &joint_feedback = feedback.leg[side].joint[joint];
            joint_feedback.angle = static_cast<float>(
                address.scale * data.qpos[address.qpos] + address.offset);
            joint_feedback.angular_velocity = static_cast<float>(
                address.scale * data.qvel[address.dof]);
        }

        const auto &address = wheel_addresses_[side];
        auto &wheel_feedback = feedback.wheel[side];
        wheel_feedback.angle = static_cast<float>(
            address.scale * data.qpos[address.qpos] + address.offset);
        wheel_feedback.angular_velocity = static_cast<float>(
            address.scale * data.qvel[address.dof]);
    }

    const double *quaternion = data.sensordata + imu_attitude_address_;
    double rotation[9];
    mju_quat2Mat(rotation, quaternion);

    feedback.imu.pitch = static_cast<float>(
        std::asin(std::clamp(-rotation[6], -1.0, 1.0)));
    feedback.imu.yaw = static_cast<float>(
        std::atan2(rotation[3], rotation[0]));

    const double *gyro = data.sensordata + imu_gyro_address_;
    feedback.imu.pitch_rate = static_cast<float>(gyro[1]);
    feedback.imu.yaw_rate = static_cast<float>(gyro[2]);
}

void MujocoAdapter::write(
    mjData &data,
    const bc_actuation_t &actuation
) const {
    std::fill(data.ctrl, data.ctrl + actuator_count_, 0.0);
    for (std::size_t side = 0; side < BC_SIDE_NUM; ++side) {
        for (std::size_t joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const auto &address = joint_addresses_[side][joint];
            data.ctrl[address.actuator] = address.scale *
                actuation.leg[side].joint_torque[joint];
        }

        const int actuator = wheel_addresses_[side].actuator;
        data.ctrl[actuator] = wheel_addresses_[side].scale *
            actuation.wheel_torque[side];
    }
}

} // namespace balance::sim
