#include "interactive_trace.hpp"

#include "balance/state_machine/motion.h"
#include "balance/state_machine/system.h"

namespace balance::sim {
namespace {

constexpr std::size_t kFlushIntervalRows = 100U;

} // namespace

InteractiveTraceWriter::InteractiveTraceWriter(
    const std::filesystem::path &path
) : csv_(path, {
    "reset_index", "simulation_time", "tick_count", "phase",
    "keyboard_forward_axis", "keyboard_yaw_axis", "keyboard_boost",
    "keyboard_step_task", "command_system_enabled",
    "command_balance_restart", "command_forward", "command_task",
    "gimbal_world_yaw", "gimbal_world_yaw_rate",
    "system", "motion", "forward", "step_task", "step_impact_armed",
    "step_impact_confirm_elapsed", "support", "alignment",
    "base_x", "base_y", "base_z", "base_forward_velocity",
    "base_vertical_velocity",
    "s", "ds", "ref_s", "ref_ds", "psi", "dpsi", "ref_psi",
    "ref_dpsi", "theta_l", "dtheta_l", "theta_r", "dtheta_r",
    "theta_b", "dtheta_b", "mapped_forward", "heading_error",
    "velocity_truth_x", "velocity_truth_y",
    "imu_specific_force_x", "imu_specific_force_y", "imu_specific_force_z",
    "imu_roll", "imu_pitch", "imu_yaw_rate",
    "kf_linear_acceleration_x", "kf_linear_acceleration_y",
    "kf_prior_velocity_x", "kf_prior_velocity_y",
    "kf_velocity_x", "kf_velocity_y",
    "kf_velocity_error_x", "kf_velocity_error_y",
    "kf_acceleration_bias_x", "kf_acceleration_bias_y",
    "kf_wheel_velocity_measurement", "kf_innovation",
    "kf_innovation_variance", "kf_nis", "kf_velocity_variance_x",
    "kf_rejection_elapsed_seconds", "kf_recovery_elapsed_seconds",
    "kf_reacquisition_elapsed_seconds", "kf_reacquisition_active",
    "kf_measurement_accepted",
    "kf_wheel_velocity_reliable", "wheel_odometry_velocity",
    "estimated_axle_velocity", "wheel_angular_velocity_l",
    "wheel_angular_velocity_r", "wheel_center_velocity_l",
    "wheel_center_velocity_r", "wheel_torque_request_l",
    "wheel_torque_request_r", "wheel_torque_applied_l",
    "wheel_torque_applied_r", "wheel_contact_l", "wheel_contact_r",
    "other_contact", "wheel_normal_force_l", "wheel_normal_force_r",
    "leg_length_l", "leg_length_r", "leg_angle_l", "leg_angle_r",
    "support_force_raw_l", "support_force_raw_r",
    "support_force_filtered_l", "support_force_filtered_r",
    "support_contact_l", "support_contact_r",
    "roll", "roll_rate",
    "impact_forward_acceleration", "impact_vertical_acceleration",
    "impact_5ms_valid", "impact_5ms_forward_dv",
    "impact_5ms_vertical_dv", "impact_5ms_pitch_rate_delta",
    "impact_5ms_leg_rate_delta_l", "impact_5ms_leg_rate_delta_r",
    "impact_5ms_wheel_velocity_delta", "impact_5ms_wheel_imu_mismatch",
    "impact_10ms_valid", "impact_10ms_forward_dv",
    "impact_10ms_vertical_dv", "impact_10ms_pitch_rate_delta",
    "impact_10ms_leg_rate_delta_l", "impact_10ms_leg_rate_delta_r",
    "impact_10ms_wheel_velocity_delta", "impact_10ms_wheel_imu_mismatch",
}) {}

void InteractiveTraceWriter::write(
    const std::uint32_t reset_index,
    const std::string_view phase,
    const KeyboardDriveInput &keyboard,
    const InteractiveScenarioFrame &frame,
    const benchmark::SimulationSample &sample,
    const benchmark::ImuMotionState &velocity_truth,
    const bc_sensor_feedback_t &feedback
) {
    const bc_controller_snapshot_t &snapshot = sample.controller;
    const bc_velocity_estimator_output_t &velocity =
        snapshot.velocity_estimator;
    const bool state_changed = state_initialized_ &&
        (snapshot.state_machine.system != previous_system_ ||
         snapshot.state_machine.motion != previous_motion_ ||
         snapshot.state_machine.forward != previous_forward_ ||
         snapshot.state_machine.step_task != previous_step_task_ ||
         snapshot.state_machine.support != previous_support_);

    csv_.begin_row();
    csv_.value(reset_index)
        .value(sample.time)
        .value(snapshot.tick_count)
        .value(phase)
        .value(keyboard.forward_axis)
        .value(keyboard.yaw_axis)
        .value(static_cast<int>(keyboard.boost))
        .value(static_cast<int>(keyboard.step_task))
        .value(static_cast<int>(frame.command.system_enabled))
        .value(static_cast<int>(frame.command.balance_restart))
        .value(frame.command.forward_velocity)
        .value(static_cast<int>(frame.command.task))
        .value(frame.gimbal.world_yaw)
        .value(frame.gimbal.world_yaw_rate)
        .value(bc_system_state_name(snapshot.state_machine.system))
        .value(bc_motion_state_name(snapshot.state_machine.motion))
        .value(bc_forward_state_name(snapshot.state_machine.forward))
        .value(bc_step_task_state_name(snapshot.state_machine.step_task))
        .value(static_cast<int>(snapshot.step_impact_armed))
        .value(snapshot.step_impact_confirm_elapsed)
        .value(bc_support_phase_state_name(snapshot.state_machine.support))
        .value(bc_chassis_alignment_name(snapshot.state_machine.alignment))
        .value(sample.base.x)
        .value(sample.base.y)
        .value(sample.base.z)
        .value(sample.base.forward_velocity)
        .value(sample.base.vertical_velocity)
        .value(snapshot.state.value[BC_STATE_S])
        .value(snapshot.state.value[BC_STATE_DS])
        .value(snapshot.state_reference.value[BC_STATE_S])
        .value(snapshot.state_reference.value[BC_STATE_DS])
        .value(snapshot.state.value[BC_STATE_PSI])
        .value(snapshot.state.value[BC_STATE_DPSI])
        .value(snapshot.state_reference.value[BC_STATE_PSI])
        .value(snapshot.state_reference.value[BC_STATE_DPSI])
        .value(snapshot.state.value[BC_STATE_THETA_L])
        .value(snapshot.state.value[BC_STATE_DTHETA_L])
        .value(snapshot.state.value[BC_STATE_THETA_R])
        .value(snapshot.state.value[BC_STATE_DTHETA_R])
        .value(snapshot.state.value[BC_STATE_THETA_B])
        .value(snapshot.state.value[BC_STATE_DTHETA_B])
        .value(snapshot.mapped_forward_velocity)
        .value(snapshot.heading_error)
        .value(velocity_truth.velocity_x)
        .value(velocity_truth.velocity_y)
        .value(feedback.imu.specific_force_x)
        .value(feedback.imu.specific_force_y)
        .value(feedback.imu.specific_force_z)
        .value(feedback.imu.roll)
        .value(feedback.imu.pitch)
        .value(feedback.imu.yaw_rate)
        .value(velocity.linear_acceleration_x)
        .value(velocity.linear_acceleration_y)
        .value(velocity.prior_velocity_x)
        .value(velocity.prior_velocity_y)
        .value(velocity.velocity_x)
        .value(velocity.velocity_y)
        .value(velocity.velocity_x - velocity_truth.velocity_x)
        .value(velocity.velocity_y - velocity_truth.velocity_y)
        .value(velocity.acceleration_bias_x)
        .value(velocity.acceleration_bias_y)
        .value(velocity.wheel_velocity_measurement)
        .value(velocity.innovation)
        .value(velocity.innovation_variance)
        .value(velocity.nis)
        .value(velocity.velocity_variance_x)
        .value(velocity.rejection_elapsed_seconds)
        .value(velocity.recovery_elapsed_seconds)
        .value(velocity.reacquisition_elapsed_seconds)
        .value(static_cast<int>(velocity.reacquisition_active))
        .value(static_cast<int>(velocity.measurement_accepted))
        .value(static_cast<int>(velocity.wheel_velocity_reliable))
        .value(snapshot.forward_velocity.wheel_odometry)
        .value(snapshot.forward_velocity.estimated_axle)
        .value(feedback.wheel[BC_L].angular_velocity)
        .value(feedback.wheel[BC_R].angular_velocity)
        .value(sample.wheel.forward_velocity[BC_L])
        .value(sample.wheel.forward_velocity[BC_R])
        .value(snapshot.actuation_request.wheel_torque[BC_L])
        .value(snapshot.actuation_request.wheel_torque[BC_R])
        .value(snapshot.actuation.wheel_torque[BC_L])
        .value(snapshot.actuation.wheel_torque[BC_R])
        .value(static_cast<int>(sample.contact.wheel[BC_L]))
        .value(static_cast<int>(sample.contact.wheel[BC_R]))
        .value(static_cast<int>(sample.contact.other))
        .value(sample.contact.wheel_normal_force[BC_L])
        .value(sample.contact.wheel_normal_force[BC_R])
        .value(snapshot.leg[BC_L].length)
        .value(snapshot.leg[BC_R].length)
        .value(snapshot.leg[BC_L].angle_body)
        .value(snapshot.leg[BC_R].angle_body)
        .value(snapshot.support_force[BC_L].vertical_force)
        .value(snapshot.support_force[BC_R].vertical_force)
        .value(snapshot.support_force[BC_L].filtered_vertical_force)
        .value(snapshot.support_force[BC_R].filtered_vertical_force)
        .value(bc_contact_state_name(snapshot.support_force[BC_L].state))
        .value(bc_contact_state_name(snapshot.support_force[BC_R].state))
        .value(snapshot.roll)
        .value(snapshot.roll_rate);
    const bc_impact_observer_output_t &impact = snapshot.impact_observer;
    csv_.value(impact.forward_acceleration)
        .value(impact.vertical_acceleration);
    for (int window = 0; window < BC_IMPACT_WINDOW_NUM; ++window) {
        const bc_impact_window_output_t &output = impact.window[window];
        csv_.value(static_cast<int>(output.valid))
            .value(output.forward_delta_velocity)
            .value(output.vertical_delta_velocity)
            .value(output.pitch_rate_delta)
            .value(output.leg_rate_delta[BC_L])
            .value(output.leg_rate_delta[BC_R])
            .value(output.wheel_velocity_delta)
            .value(output.wheel_imu_delta_mismatch);
    }
    csv_.end_row();

    ++row_count_;
    if (state_changed || row_count_ % kFlushIntervalRows == 0U) {
        csv_.flush();
    }
    previous_system_ = snapshot.state_machine.system;
    previous_motion_ = snapshot.state_machine.motion;
    previous_forward_ = snapshot.state_machine.forward;
    previous_step_task_ = snapshot.state_machine.step_task;
    previous_support_ = snapshot.state_machine.support;
    state_initialized_ = true;
}

void InteractiveTraceWriter::flush() {
    csv_.flush();
}

} // namespace balance::sim
