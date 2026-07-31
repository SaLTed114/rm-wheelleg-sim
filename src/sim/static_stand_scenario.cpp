#include "static_stand_scenario.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "balance/math_utils.h"

namespace balance::sim {
namespace {

constexpr float kTargetLength = 0.20F;
constexpr float kTargetAngle = -0.5F * BC_PI_F;
constexpr float kLengthTolerance = 0.025F;
constexpr float kLengthVelocityTolerance = 0.03F;
constexpr float kAngleTolerance = 5.0F * BC_PI_F / 180.0F;
constexpr float kAngleVelocityTolerance = 0.15F;
constexpr double kStableDuration = 0.25;
constexpr double kPreparationTimeout = 5.0;
constexpr double kWheelRadius = 0.05806;
constexpr const char *kSupportWeld = "base_support_weld";

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

StaticStandScenario::StaticStandScenario(
    MujocoPlant &plant, SimulationRunner &runner
) : plant_(plant), runner_(runner) {
    const int base_joint = require_id(
        plant.model(), mjOBJ_JOINT, "base_free_joint");
    base_qpos_ = plant.model().jnt_qposadr[base_joint];
    ground_geom_ = require_id(
        plant.model(), mjOBJ_GEOM, "ground");
    wheel_axis_site_[BC_L] = require_id(
        plant.model(), mjOBJ_SITE, "Right_wheel_axis_site");
    wheel_axis_site_[BC_R] = require_id(
        plant.model(), mjOBJ_SITE, "Left_wheel_axis_site");
}

bc_operator_command_t StaticStandScenario::make_posture_command() const {
    bc_operator_command_t command{};
    command.enabled = 1U;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        command.leg[side].length = kTargetLength;
        command.leg[side].angle_body = kTargetAngle;
    }
    return command;
}

void StaticStandScenario::reset() {
    runner_.reset();
    plant_.set_equality_active(kSupportWeld, true);
    runner_.set_command(make_posture_command());
    phase_ = StaticStandPhase::Preparing;
    stable_duration_ = 0.0;
    release_time_ = -1.0;
    forward_velocity_target_ = 0.0F;
    yaw_rate_target_ = 0.0F;
    balance_command_ = {};
}

void StaticStandScenario::set_motion_target(
    const float forward_velocity, const float yaw_rate
) noexcept {
    forward_velocity_target_ = forward_velocity;
    yaw_rate_target_ = yaw_rate;
}

bool StaticStandScenario::posture_is_stable() const {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const bc_leg_kinematics_t &leg = runner_.leg(
            static_cast<bc_side_t>(side));
        const bool stable =
            std::abs(leg.length - kTargetLength) <= kLengthTolerance &&
            std::abs(leg.length_velocity) <= kLengthVelocityTolerance &&
            std::abs(bc_wrap_anglef(leg.angle_body - kTargetAngle)) <=
                kAngleTolerance &&
            std::abs(leg.angular_velocity) <= kAngleVelocityTolerance;
        if (!stable) return false;
    }
    return true;
}

void StaticStandScenario::release() {
    balance_command_ = make_posture_command();
    balance_command_.balance_enabled = 1U;
    balance_command_.state_reference.value[BC_STATE_S] =
        runner_.state().value[BC_STATE_S];
    balance_command_.state_reference.value[BC_STATE_PSI] =
        runner_.state().value[BC_STATE_PSI];

    runner_.set_command(balance_command_);
    place_wheels_on_ground();
    plant_.set_equality_active(kSupportWeld, false);
    mj_forward(&plant_.model(), &plant_.data());
    phase_ = StaticStandPhase::Balancing;
    release_time_ = plant_.data().time;
}

void StaticStandScenario::update_balance_reference() {
    float *reference = balance_command_.state_reference.value;
    const float timestep = static_cast<float>(plant_.timestep());

    reference[BC_STATE_S] += forward_velocity_target_ * timestep;
    reference[BC_STATE_DS] = forward_velocity_target_;
    reference[BC_STATE_PSI] += yaw_rate_target_ * timestep;
    reference[BC_STATE_DPSI] = yaw_rate_target_;
    runner_.set_command(balance_command_);
}

void StaticStandScenario::place_wheels_on_ground() {
    const double wheel_axis_height = 0.5 * (
        plant_.data().site_xpos[3 * wheel_axis_site_[BC_L] + 2] +
        plant_.data().site_xpos[3 * wheel_axis_site_[BC_R] + 2]);
    const double ground_height =
        plant_.model().geom_pos[3 * ground_geom_ + 2];

    plant_.data().qpos[base_qpos_ + 2] +=
        ground_height + kWheelRadius - wheel_axis_height;
}

void StaticStandScenario::step() {
    if (phase_ == StaticStandPhase::Balancing) {
        update_balance_reference();
        runner_.step();
        return;
    }

    runner_.step();
    if (phase_ != StaticStandPhase::Preparing) return;

    if (posture_is_stable()) stable_duration_ += plant_.timestep();
    else stable_duration_ = 0.0;

    if (stable_duration_ >= kStableDuration) release();
    else if (plant_.data().time >= kPreparationTimeout) {
        phase_ = StaticStandPhase::PreparationTimeout;
    }
}

const char *StaticStandScenario::phase_name() const noexcept {
    switch (phase_) {
    case StaticStandPhase::Preparing:
        return "preparing";
    case StaticStandPhase::Balancing:
        return "balancing";
    case StaticStandPhase::PreparationTimeout:
        return "preparation timeout";
    }
    return "unknown";
}

} // namespace balance::sim
