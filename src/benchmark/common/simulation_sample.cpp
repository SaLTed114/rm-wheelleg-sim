#include "simulation_sample.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace balance::benchmark {
namespace {

int require_id(
    const mjModel &model, const mjtObj type, const char *name
) {
    const int id = mj_name2id(&model, type, name);
    if (id < 0) {
        throw std::runtime_error(
            "missing MuJoCo object '" + std::string(name) + "'");
    }
    return id;
}

} // namespace

SimulationSampler::SimulationSampler(const mjModel &model) : model_(model) {
    const int base_joint = require_id(
        model_, mjOBJ_JOINT, "base_free_joint");
    base_qpos_ = model_.jnt_qposadr[base_joint];
    base_dof_ = model_.jnt_dofadr[base_joint];
    base_body_ = require_id(model_, mjOBJ_BODY, "base_link");
    ground_ = require_id(model_, mjOBJ_GEOM, "ground");
    wheel_ = {{
        require_id(model_, mjOBJ_GEOM, "Right_wheel_collision"),
        require_id(model_, mjOBJ_GEOM, "Left_wheel_collision"),
    }};
    wheel_axis_ = {{
        require_id(model_, mjOBJ_SITE, "Right_wheel_axis_site"),
        require_id(model_, mjOBJ_SITE, "Left_wheel_axis_site"),
    }};
    imu_site_ = require_id(model_, mjOBJ_SITE, "imu_site");
}

SimulationSample SimulationSampler::read(
    const mjData &data,
    const bc_controller_snapshot_t &controller
) const {
    return SimulationSample{
        data.time,
        controller,
        read_base(data),
        read_axle_position(data),
        read_wheel_motion(data),
        read_contacts(data),
    };
}

AxlePositionState SimulationSampler::read_axle_position(
    const mjData &data
) const {
    const mjtNum *left = data.site_xpos + 3 * wheel_axis_[BC_L];
    const mjtNum *right = data.site_xpos + 3 * wheel_axis_[BC_R];
    return AxlePositionState{
        0.5 * (left[0] + right[0]),
        0.5 * (left[1] + right[1]),
    };
}

BaseState SimulationSampler::read_base(const mjData &data) const {
    const mjtNum *rotation = data.xmat + 9 * base_body_;
    const double heading_norm = std::hypot(rotation[0], rotation[3]);
    const double heading_x = rotation[0] / heading_norm;
    const double heading_y = rotation[3] / heading_norm;

    return BaseState{
        data.qpos[base_qpos_],
        data.qpos[base_qpos_ + 1],
        data.qpos[base_qpos_ + 2],
        data.qvel[base_dof_] * heading_x +
            data.qvel[base_dof_ + 1] * heading_y,
        data.qvel[base_dof_ + 2],
    };
}

WheelMotionState SimulationSampler::read_wheel_motion(
    const mjData &data
) const {
    const mjtNum *rotation = data.xmat + 9 * base_body_;
    const double heading_norm = std::hypot(rotation[0], rotation[3]);
    const double heading_x = rotation[0] / heading_norm;
    const double heading_y = rotation[3] / heading_norm;
    WheelMotionState state{};

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        mjtNum velocity[6]{};
        mj_objectVelocity(
            &model_, &data, mjOBJ_SITE, wheel_axis_[side], velocity, 0);
        state.forward_velocity[side] =
            velocity[3] * heading_x + velocity[4] * heading_y;
    }
    return state;
}

ImuMotionState SimulationSampler::read_imu_motion(
    const mjData &data
) const {
    mjtNum velocity[6]{};
    mj_objectVelocity(
        &model_, &data, mjOBJ_SITE, imu_site_, velocity, 1);
    return ImuMotionState{
        velocity[3],
        velocity[4],
    };
}

GroundContactState SimulationSampler::read_contacts(
    const mjData &data
) const {
    GroundContactState state{};
    for (int index = 0; index < data.ncon; ++index) {
        const mjContact &contact = data.contact[index];
        const bool has_ground =
            contact.geom[0] == ground_ || contact.geom[1] == ground_;
        if (!has_ground) {
            state.other = true;
            if (state.unexpected.empty()) {
                state.unexpected = contact_name(contact);
            }
            continue;
        }

        bool wheel_contact = false;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            const bool pair =
                (contact.geom[0] == ground_ &&
                 contact.geom[1] == wheel_[side]) ||
                (contact.geom[1] == ground_ &&
                 contact.geom[0] == wheel_[side]);
            state.wheel[side] = state.wheel[side] || pair;
            if (pair) {
                mjtNum force[6] = {};
                mj_contactForce(&model_, &data, index, force);
                state.wheel_normal_force[side] += std::max(0.0, force[0]);
            }
            wheel_contact = wheel_contact || pair;
        }
        state.other = state.other || !wheel_contact;
        if (!wheel_contact && state.unexpected.empty()) {
            state.unexpected = contact_name(contact);
        }
    }
    return state;
}

std::string SimulationSampler::contact_name(
    const mjContact &contact
) const {
    std::string description;
    for (int pair = 0; pair < 2; ++pair) {
        if (!description.empty()) description += '+';
        const char *name = mj_id2name(
            &model_, mjOBJ_GEOM, contact.geom[pair]);
        if (name != nullptr) {
            description += name;
        } else {
            const int body = model_.geom_bodyid[contact.geom[pair]];
            const char *body_name = mj_id2name(
                &model_, mjOBJ_BODY, body);
            description += body_name != nullptr ? body_name :
                "geom_" + std::to_string(contact.geom[pair]);
        }
    }
    return description;
}

} // namespace balance::benchmark
