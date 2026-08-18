#include "balance_cpp/state_machine/motion.hpp"

#include <cmath>

#include "balance_cpp/math.hpp"

namespace balance::control {

MotionStateMachine::MotionStateMachine(MotionConfig config)
    : config_(config) {
    reset();
}

void MotionStateMachine::reset() {
    state_ = MotionState::idle;
    state_reference_ = {};
    leg_stable_hold_.reset();
    engage_hold_.reset();
}

void MotionStateMachine::start() {
    reset();
    state_ = MotionState::self_righting;
}

bool MotionStateMachine::wheel_observation_enabled() const {
    return state_ == MotionState::balance_engaging ||
        state_ == MotionState::active;
}

bool MotionStateMachine::legs_stable(const Estimate &estimate) const {
    for (const auto &leg : estimate.leg) {
        if (std::abs(leg.length - config_.startup_leg_length) >
                config_.length_tolerance ||
            std::abs(leg.length_velocity) >
                config_.length_velocity_tolerance ||
            std::abs(math::wrap_angle(
                leg.angle_body - config_.startup_leg_angle)) >
                config_.angle_tolerance ||
            std::abs(leg.angular_velocity) >
                config_.angular_velocity_tolerance) {
            return false;
        }
    }
    return true;
}

ControlCommand MotionStateMachine::update(
    const Estimate &estimate,
    const float timestep
) {
    transition(estimate, timestep);
    return action();
}

void MotionStateMachine::transition(
    const Estimate &estimate,
    const float timestep
) {
    switch (state_) {
    case MotionState::self_righting:
        state_ = MotionState::leg_positioning;
        break;
    case MotionState::leg_positioning:
        if (leg_stable_hold_.update(
                legs_stable(estimate), config_.stable_duration, timestep)) {
            state_ = MotionState::balance_engaging;
            state_reference_ = {};
            state_reference_[StateIndex::position] =
                estimate.state[StateIndex::position];
            state_reference_[StateIndex::heading] =
                estimate.state[StateIndex::heading];
            engage_hold_.reset();
        }
        break;
    case MotionState::balance_engaging:
        if (engage_hold_.update(
                true, config_.engage_duration, timestep)) {
            state_ = MotionState::active;
            state_reference_ = {};
            state_reference_[StateIndex::position] =
                estimate.state[StateIndex::position];
            state_reference_[StateIndex::heading] =
                estimate.state[StateIndex::heading];
        }
        break;
    case MotionState::idle:
    case MotionState::active:
        break;
    }
}

ControlCommand MotionStateMachine::action() const {
    ControlCommand command{};
    switch (state_) {
    case MotionState::idle:
    case MotionState::self_righting:
        break;
    case MotionState::leg_positioning:
        for (auto &leg : command.leg) {
            leg.length_strategy = LegLengthStrategy::position;
            leg.angle_strategy = LegAngleStrategy::position;
            leg.target_length = config_.startup_leg_length;
            leg.target_angle = config_.startup_leg_angle;
        }
        break;
    case MotionState::balance_engaging:
    case MotionState::active:
        for (auto &leg : command.leg) {
            leg.length_strategy = LegLengthStrategy::position_support;
            leg.angle_strategy = LegAngleStrategy::lqr;
            leg.target_length = config_.startup_leg_length;
        }
        command.wheel_strategy = WheelStrategy::lqr;
        command.state_reference = state_reference_;
        command.suppress_position_feedback =
            state_ == MotionState::balance_engaging;
        command.suppress_heading_feedback =
            state_ == MotionState::balance_engaging;
        break;
    }
    return command;
}

MotionStatus MotionStateMachine::status() const {
    return {
        state_,
        leg_stable_hold_.elapsed(),
        engage_hold_.elapsed(),
    };
}

} // namespace balance::control
