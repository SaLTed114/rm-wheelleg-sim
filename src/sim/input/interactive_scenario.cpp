#include "input/interactive_scenario.hpp"

#include <cmath>

#include "balance/math_utils.h"

namespace balance::sim {

InteractiveScenario::InteractiveScenario(const InteractiveMode mode) noexcept
    : mode_(mode) {}

void InteractiveScenario::reset(
    const bc_controller_snapshot_t &snapshot) noexcept {
    virtual_gimbal_.reset();
    frame_ = {};
    frame_.phase = bc_system_state_name(snapshot.state_machine.system);
    previous_motion_ = BC_MOTION_IDLE;
    balance_start_time_ = -1.0;
    virtual_gimbal_initialized_ = false;
}

const InteractiveScenarioFrame &InteractiveScenario::update(
    const bc_controller_snapshot_t &snapshot,
    const KeyboardDriveInput &keyboard,
    const double simulation_time,
    const float timestep_seconds) noexcept {
    if (previous_motion_ != BC_MOTION_ACTIVE &&
        snapshot.state_machine.motion == BC_MOTION_ACTIVE) {
        balance_start_time_ = simulation_time;
    }
    previous_motion_ = snapshot.state_machine.motion;

    const MotionTarget target = mode_ == InteractiveMode::keyboard ?
        keyboard_target(snapshot, keyboard) :
        demo_target(snapshot, simulation_time);

    frame_.command = {};
    frame_.command.system_enabled = static_cast<uint8_t>(
        simulation_time >= 2.0);
    frame_.command.balance_restart =
        frame_.command.system_enabled &&
        snapshot.state_machine.system == BC_SYSTEM_OFF;

    if (snapshot.state_machine.motion == BC_MOTION_ACTIVE) {
        if (!virtual_gimbal_initialized_) {
            virtual_gimbal_.reset(snapshot.state.value[BC_STATE_PSI]);
            virtual_gimbal_initialized_ = true;
        }
        virtual_gimbal_.update(target.gimbal_yaw_rate, timestep_seconds);
        frame_.command.forward_velocity = target.forward_velocity;
    } else {
        virtual_gimbal_.reset(snapshot.state.value[BC_STATE_PSI]);
        virtual_gimbal_initialized_ = false;
    }

    frame_.gimbal = virtual_gimbal_.state();
    frame_.gimbal_feedback = virtual_gimbal_initialized_ ?
        virtual_gimbal_.feedback(
            snapshot.state.value[BC_STATE_PSI],
            snapshot.state.value[BC_STATE_DPSI]) :
        bc_gimbal_feedback_t{};
    frame_.phase = target.phase;
    return frame_;
}

InteractiveScenario::MotionTarget InteractiveScenario::demo_target(
    const bc_controller_snapshot_t &snapshot,
    const double simulation_time) const noexcept {
    if (snapshot.state_machine.system == BC_SYSTEM_OFF) {
        return {
            0.0F, 0.0F,
            bc_system_state_name(snapshot.state_machine.system),
        };
    }
    if (snapshot.state_machine.motion != BC_MOTION_ACTIVE) {
        return {
            0.0F, 0.0F,
            bc_motion_state_name(snapshot.state_machine.motion),
        };
    }
    if (balance_start_time_ < 0.0) return {0.0F, 0.0F, "standing"};

    constexpr double kCycleDuration = 23.0;
    const double time = std::fmod(
        simulation_time - balance_start_time_, kCycleDuration);
    if (time < 3.0) return {0.0F, 0.0F, "standing"};
    if (time < 6.0) return {0.25F, 0.0F, "forward"};
    if (time < 8.0) return {0.0F, 0.0F, "stopping"};
    if (time < 11.0) return {-0.25F, 0.0F, "reverse"};
    if (time < 13.0) return {0.0F, 0.0F, "stopping"};
    if (time < 16.0) return {0.0F, 1.57F, "yaw left"};
    if (time < 18.0) return {0.0F, 0.0F, "stopping"};
    if (time < 21.0) return {0.0F, -1.57F, "yaw right"};
    return {0.0F, 0.0F, "stopping"};
}

InteractiveScenario::MotionTarget InteractiveScenario::keyboard_target(
    const bc_controller_snapshot_t &snapshot,
    const KeyboardDriveInput &keyboard) noexcept {
    if (snapshot.state_machine.system == BC_SYSTEM_OFF) {
        return {
            0.0F, 0.0F,
            bc_system_state_name(snapshot.state_machine.system),
        };
    }
    if (snapshot.state_machine.motion != BC_MOTION_ACTIVE) {
        return {
            0.0F, 0.0F,
            bc_motion_state_name(snapshot.state_machine.motion),
        };
    }

    constexpr float kForwardVelocity = 2.0F;
    constexpr float kBoostForwardVelocity = 3.0F;
    constexpr float kGimbalYawRate = BC_PI_F;
    return {
        keyboard.forward_axis * (keyboard.boost ?
            kBoostForwardVelocity : kForwardVelocity),
        keyboard.yaw_axis * kGimbalYawRate,
        bc_forward_state_name(snapshot.state_machine.forward),
    };
}

} // namespace balance::sim
