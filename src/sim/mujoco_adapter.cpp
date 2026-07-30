#include "mujoco_adapter.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>

namespace balance::sim {
namespace {

struct ChannelSpec {
    const char *joint_name;
    const char *actuator_name;
};

constexpr std::array<std::array<ChannelSpec, BC_JOINT_NUM>, BC_SIDE_NUM>
    kJointSpecs{{
        {{{"Left_front_joint", "Left_front_joint_actuator"},
          {"Left_rear_joint", "Left_rear_joint_actuator"}}},
        {{{"Right_front_joint", "Right_front_joint_actuator"},
          {"Right_rear_joint", "Right_rear_joint_actuator"}}},
    }};

constexpr std::array<ChannelSpec, BC_SIDE_NUM> kWheelSpecs{{
    {"Left_Wheel_joint", "Left_Wheel_joint_actuator"},
    {"Right_Wheel_joint", "Right_Wheel_joint_actuator"},
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
    const char *actuator_name
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
    };
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
                model, spec.joint_name, spec.actuator_name);
            joint_addresses_[side][joint] = address;
            qpos_addresses[channel] = address.qpos;
            dof_addresses[channel] = address.dof;
            actuator_ids[channel] = address.actuator;
            ++channel;
        }

        const auto &spec = kWheelSpecs[side];
        const auto address = resolve_channel(
            model, spec.joint_name, spec.actuator_name);
        wheel_addresses_[side] = address;
        qpos_addresses[channel] = address.qpos;
        dof_addresses[channel] = address.dof;
        actuator_ids[channel] = address.actuator;
        ++channel;
    }

    require_unique(qpos_addresses, "joint position");
    require_unique(dof_addresses, "joint velocity");
    require_unique(actuator_ids, "actuator");
}

void MujocoAdapter::read(
    const mjData &data,
    bc_observation_t &observation
) const {
    for (std::size_t side = 0; side < BC_SIDE_NUM; ++side) {
        for (std::size_t joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const auto &address = joint_addresses_[side][joint];
            auto &feedback = observation.leg[side].joint[joint];
            feedback.angle = static_cast<float>(data.qpos[address.qpos]);
            feedback.angular_velocity =
                static_cast<float>(data.qvel[address.dof]);
        }

        const auto &address = wheel_addresses_[side];
        auto &feedback = observation.wheel[side];
        feedback.angle = static_cast<float>(data.qpos[address.qpos]);
        feedback.angular_velocity =
            static_cast<float>(data.qvel[address.dof]);
    }
}

void MujocoAdapter::write(
    mjData &data,
    const bc_actuation_t &actuation
) const {
    std::fill(data.ctrl, data.ctrl + actuator_count_, 0.0);
    for (std::size_t side = 0; side < BC_SIDE_NUM; ++side) {
        for (std::size_t joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const int actuator = joint_addresses_[side][joint].actuator;
            data.ctrl[actuator] = actuation.leg[side].joint_torque[joint];
        }

        const int actuator = wheel_addresses_[side].actuator;
        data.ctrl[actuator] = actuation.wheel_torque[side];
    }
}

} // namespace balance::sim
