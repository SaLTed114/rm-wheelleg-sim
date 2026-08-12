#include "drop_scenario.hpp"

#include <algorithm>
#include <stdexcept>

#include "balance/math_utils.h"

namespace balance::benchmark {
namespace {

constexpr double kStandingSeconds = 2.0;
constexpr double kDisabledSettleSeconds = 2.0;
constexpr double kEngagementTimeoutSeconds = 10.0;
constexpr double kPostTouchdownSeconds = 1.0;
constexpr double kDistanceSearchLimit = 2.0;

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

bc_gimbal_feedback_t held_heading_feedback(
    const bc_controller_snapshot_t &snapshot,
    const float held_heading
) {
    return {
        bc_wrap_anglef(
            held_heading - snapshot.state.value[BC_STATE_PSI]),
        -snapshot.state.value[BC_STATE_DPSI],
    };
}

} // namespace

DropScenario::DropScenario(
    const DropCaseSpec &spec, const mjModel &model
) : spec_(spec), name_(drop_case_name(spec)) {
    const int base_joint = require_id(
        model, mjOBJ_JOINT, "base_free_joint");
    base_qpos_ = model.jnt_qposadr[base_joint];
    base_dof_ = model.jnt_dofadr[base_joint];
    ground_ = require_id(model, mjOBJ_GEOM, "ground");
    wheel_ = {{
        require_id(model, mjOBJ_GEOM, "Right_wheel_collision"),
        require_id(model, mjOBJ_GEOM, "Left_wheel_collision"),
    }};
    reset();
}

void DropScenario::reset() noexcept {
    phase_ = DropPhase::disabled_settle;
    touchdown_latch_ = {};
    command_ = {};
    active_start_time_ = -1.0;
    settle_start_time_ = -1.0;
    release_time_ = 0.0;
    touchdown_time_ = 0.0;
    release_clearance_ = {};
    normal_leg_length_ = {};
    balance_engaged_ = false;
    heading_initialized_ = false;
    held_heading_ = 0.0F;
    issue_ = "none";
}

const char *DropScenario::phase_name() const noexcept {
    switch (phase_) {
    case DropPhase::disabled_settle: return "drop_disabled_settle";
    case DropPhase::standing: return "drop_standing";
    case DropPhase::airborne: return "drop_airborne";
    case DropPhase::post_touchdown: return "drop_post_touchdown";
    case DropPhase::complete: return "drop_complete";
    case DropPhase::failed: return "drop_failed";
    }
    return "drop_unknown";
}

std::array<double, BC_SIDE_NUM> DropScenario::wheel_clearance(
    sim::MujocoPlant &plant
) const {
    std::array<double, BC_SIDE_NUM> clearance{};
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        clearance[side] = mj_geomDistance(
            &plant.model(), &plant.data(), ground_, wheel_[side],
            kDistanceSearchLimit, nullptr);
    }
    return clearance;
}

void DropScenario::release(sim::MujocoPlant &plant) {
    const auto initial = wheel_clearance(plant);
    const double minimum = *std::min_element(initial.begin(), initial.end());
    plant.data().qpos[base_qpos_ + 2] += spec_.wheel_clearance - minimum;
    plant.data().qvel[base_dof_ + 4] = spec_.initial_pitch_rate;
    mj_forward(&plant.model(), &plant.data());
    release_clearance_ = wheel_clearance(plant);
    release_time_ = plant.data().time;
    phase_ = DropPhase::airborne;
}

void DropScenario::step(
    sim::MujocoPlant &plant,
    sim::SimulationRunner &runner,
    const SimulationSampler &sampler
) {
    if (finished()) return;

    if (settle_start_time_ < 0.0) settle_start_time_ = plant.data().time;
    command_.system_enabled = static_cast<uint8_t>(
        phase_ != DropPhase::disabled_settle);
    command_.balance_restart = static_cast<uint8_t>(
        command_.system_enabled &&
        runner.snapshot().state_machine.system == BC_SYSTEM_OFF);
    command_.forward_velocity = 0.0F;
    if (runner.snapshot().state_machine.motion == BC_MOTION_ACTIVE) {
        if (!heading_initialized_) {
            held_heading_ = runner.snapshot().state.value[BC_STATE_PSI];
            heading_initialized_ = true;
        }
    } else {
        held_heading_ = runner.snapshot().state.value[BC_STATE_PSI];
        heading_initialized_ = false;
    }
    const bc_gimbal_feedback_t gimbal = held_heading_feedback(
        runner.snapshot(), held_heading_);

    if (phase_ == DropPhase::disabled_settle) {
        runner.step(command_, gimbal);
        if (plant.data().time - settle_start_time_ >=
            kDisabledSettleSeconds) {
            phase_ = DropPhase::standing;
        }
        return;
    }

    if (phase_ == DropPhase::standing) {
        runner.step(command_, gimbal);
        if (runner.snapshot().state_machine.motion == BC_MOTION_ACTIVE) {
            balance_engaged_ = true;
            if (active_start_time_ < 0.0) active_start_time_ = plant.data().time;
            if (plant.data().time - active_start_time_ >= kStandingSeconds) {
                normal_leg_length_ = {{
                    runner.snapshot().leg[BC_L].length,
                    runner.snapshot().leg[BC_R].length,
                }};
                release(plant);
            }
        }
        if (!balance_engaged_ &&
            plant.data().time >= kEngagementTimeoutSeconds) {
            issue_ = "balance_not_engaged";
            phase_ = DropPhase::failed;
        }
        return;
    }

    const GroundContactState contact = sampler.read_contacts(plant.data());
    const bool wheel_contact = contact.wheel[BC_L] || contact.wheel[BC_R];
    const bool latched_before = touchdown_latch_.latched();
    const bool latched = touchdown_latch_.update(wheel_contact);
    if (latched && !latched_before) {
        touchdown_time_ = plant.data().time;
        phase_ = DropPhase::post_touchdown;
    }

    if (!latched) {
        runner.step_with_control_transform(
            command_, gimbal,
            [policy = spec_.policy,
             normal_leg_length = normal_leg_length_](
                bc_control_command_t &control
            ) {
                apply_drop_air_policy(policy, control);
                for (int side = 0; side < BC_SIDE_NUM; ++side) {
                    control.leg[side].target.length =
                        normal_leg_length[side];
                }
            });
    } else {
        runner.step_with_control_transform(
            command_, gimbal,
            [](bc_control_command_t &control) {
                control.wheel_strategy = BC_WHEEL_LQR;
                for (int side = 0; side < BC_SIDE_NUM; ++side) {
                    control.leg[side].length_strategy =
                        BC_LEG_LENGTH_POSITION_SUPPORT;
                    control.leg[side].angle_strategy = BC_LEG_ANGLE_LQR;
                }
            });
        if (plant.data().time - touchdown_time_ >= kPostTouchdownSeconds) {
            phase_ = DropPhase::complete;
        }
    }
}

} // namespace balance::benchmark
