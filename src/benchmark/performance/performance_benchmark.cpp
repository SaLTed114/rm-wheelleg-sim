#include "performance_benchmark.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "balance/math_utils.h"

namespace balance::benchmark {
namespace {

constexpr double kTimestepSeconds = 0.001;
constexpr double kMinimumLegLength = 0.13;
constexpr double kMaximumLegLength = 0.20;
constexpr double kForwardBaseTolerance = 0.10;
constexpr double kYawBaseTolerance = 0.20;
constexpr double kRelativeTrackingTolerance = 0.10;

bc_controller_config_t controller_config(
    const std::optional<double> leg_length,
    const std::optional<double> yaw_acceleration_feedforward_scale
) {
    bc_controller_config_t config{};
    bc_controller_default_config(&config);
    if (leg_length) {
        if (!std::isfinite(*leg_length) || *leg_length <= 0.0) {
            throw std::invalid_argument(
                "leg length must be finite and positive");
        }
        config.motion.leg_length = static_cast<float>(*leg_length);
    }
    if (yaw_acceleration_feedforward_scale) {
        if (!std::isfinite(*yaw_acceleration_feedforward_scale) ||
            *yaw_acceleration_feedforward_scale < 0.0) {
            throw std::invalid_argument(
                "yaw acceleration feedforward scale must be finite and non-negative");
        }
        config.control.lqr_compensation.yaw_acceleration_feedforward_scale =
            static_cast<float>(*yaw_acceleration_feedforward_scale);
    }
    return config;
}

double effective_yaw_acceleration_feedforward_scale(
    const std::optional<double> override
) {
    const bc_controller_config_t config =
        controller_config(std::nullopt, override);
    return config.control.lqr_compensation.
        yaw_acceleration_feedforward_scale;
}

} // namespace

PerformanceBenchmark::PerformanceBenchmark(
    const std::filesystem::path &model_path,
    const std::filesystem::path &output_directory,
    const PerformanceBenchmarkConfig &config
) : plant_(model_path, kTimestepSeconds),
    adapter_(plant_.model()),
    runner_(plant_, adapter_, controller_config(
        config.leg_length,
        config.yaw_acceleration_feedforward_scale)),
    sampler_(plant_.model()),
    roll_restraint_(plant_.model(), config.roll_restrained),
    forward_velocity_(
        config.forward_velocity_observation,
        controller_config(
            config.leg_length,
            config.yaw_acceleration_feedforward_scale).
                control.observer.wheel_radius),
    summary_(output_directory / "summary.csv", {
        "case", "axis", "target", "command_rate",
        "coupled_forward_velocity", "forward_lead_seconds",
        "target_hold_seconds",
        "stop_settle_seconds", "standing_seconds", "leg_length_target",
        "forward_velocity_observation", "roll_restrained",
        "yaw_acceleration_feedforward_scale",
        "completed", "balance_engaged",
        "leg_length_valid", "finite",
        "tracked", "settled", "issue", "issue_phase",
        "wheel_contact_ratio", "other_contact_steps", "max_pitch_deg",
        "max_roll_deg", "max_leg_common_deg", "max_leg_difference_deg",
        "peak_roll_force_request",
        "peak_roll_restraint_torque", "min_vertical_l", "min_vertical_r",
        "initial_s_error",
        "tracking_mean_error", "tracking_rmse", "settle_mean_ds",
        "settle_rmse_ds", "settle_mean_dpsi", "settle_rmse_dpsi",
        "max_heading_error", "heading_error_rmse",
        "stop_peak_abs_dpsi",
        "peak_raw_wheel_l", "peak_raw_wheel_r",
        "peak_raw_joint_l_front", "peak_raw_joint_l_rear",
        "peak_raw_joint_r_front", "peak_raw_joint_r_rear",
        "wheel_saturation_l", "wheel_saturation_r",
        "joint_saturation_l_front", "joint_saturation_l_rear",
        "joint_saturation_r_front", "joint_saturation_r_rear",
    }),
    trace_(output_directory / "trace.csv", {
        "case", "phase", "simulation_time", "command_rate",
        "leg_length_target", "forward_velocity_observation",
        "base_x", "base_y", "base_z", "base_forward_velocity",
        "base_vertical_velocity", "imu_specific_force_x",
        "imu_specific_force_y", "imu_specific_force_z",
        "imu_linear_acceleration_x", "imu_linear_acceleration_y",
        "velocity_prior_x", "velocity_prior_y",
        "velocity_estimate_x", "velocity_estimate_y",
        "velocity_truth_x", "velocity_truth_y",
        "velocity_prior_error_x", "velocity_prior_error_y",
        "velocity_estimate_error_x", "velocity_estimate_error_y",
        "acceleration_bias_x", "acceleration_bias_y",
        "wheel_velocity_measurement", "velocity_innovation",
        "velocity_innovation_variance", "velocity_nis",
        "velocity_measurement_accepted", "wheel_velocity_reliable",
        "wheel_odometry_velocity", "estimated_axle_velocity",
        "gimbal_yaw", "gimbal_yaw_rate",
        "gimbal_relative_yaw", "gimbal_relative_yaw_rate",
        "alignment", "command_forward", "mapped_forward",
        "heading_error", "ref_ddpsi",
        "system", "motion", "forward", "s", "ds", "psi", "dpsi",
        "theta_l",
        "dtheta_l", "theta_r", "dtheta_r", "theta_b", "dtheta_b",
        "ref_s", "ref_ds", "ref_psi", "ref_dpsi", "ref_theta_l",
        "ref_dtheta_l", "ref_theta_r", "ref_dtheta_r", "ref_theta_b",
        "ref_dtheta_b", "roll", "roll_rate", "roll_force_request",
        "roll_restraint_torque",
        "leg_l_length", "leg_l_angle", "leg_l_length_rate",
        "leg_l_angle_rate",
        "leg_r_length", "leg_r_angle", "leg_r_length_rate",
        "leg_r_angle_rate", "wheel_encoder_velocity_l",
        "wheel_encoder_velocity_r", "wheel_center_velocity_l",
        "wheel_center_velocity_r", "raw_wheel_l", "raw_wheel_r",
        "raw_joint_l_front", "raw_joint_l_rear", "raw_joint_r_front",
        "raw_joint_r_rear", "wheel_l", "wheel_r", "joint_l_front",
        "joint_l_rear", "joint_r_front", "joint_r_rear",
        "contact_wheel_l", "contact_wheel_r", "other_contact",
        "normal_force_l", "normal_force_r",
    }),
    yaw_acceleration_feedforward_scale_(
        effective_yaw_acceleration_feedforward_scale(
            config.yaw_acceleration_feedforward_scale)),
    trace_stride_(config.trace_stride) {
    if (trace_stride_ == 0U) {
        throw std::invalid_argument("trace stride must be positive");
    }
    const bc_controller_config_t controller =
        controller_config(
            config.leg_length,
            config.yaw_acceleration_feedforward_scale);
    leg_length_target_ = controller.motion.leg_length;
}

PerformanceResult PerformanceBenchmark::run(
    const PerformanceCaseSpec &spec
) {
    runner_.reset();
    roll_restraint_.reset();
    forward_velocity_.reset();
    sample_index_ = 0U;
    PerformanceResult result{};
    result.spec = spec;
    result.leg_length_target = leg_length_target_;
    result.forward_velocity_observation = forward_velocity_.observation();
    result.roll_restrained = roll_restraint_.enabled();
    result.yaw_acceleration_feedforward_scale =
        yaw_acceleration_feedforward_scale_;
    PerformanceScenario scenario(spec);
    scenario.reset(plant_.data().time);

    while (!scenario.finished()) {
        result.balance_engaged = result.balance_engaged ||
            runner_.snapshot().state_machine.motion ==
                BC_MOTION_ACTIVE;
        scenario.update(runner_.snapshot(), plant_.data().time);
        if (scenario.finished()) break;

        PerformanceResult *monitored =
            scenario.monitored() ? &result : nullptr;
        if (!step(
                spec, scenario.phase_name(), scenario.command(),
                scenario.gimbal(), monitored,
                scenario.tracking_evaluation(),
                scenario.settle_evaluation(),
                scenario.phase() == PerformancePhase::stop_ramp ||
                    scenario.phase() == PerformancePhase::stop_settle)) {
            finish_result(result);
            return result;
        }
    }

    result.completed = true;
    if (!result.balance_engaged) {
        result.issue = "balance_not_engaged";
        result.issue_phase = "engaging";
    }
    finish_result(result);
    return result;
}

void PerformanceBenchmark::write_summary(
    const PerformanceResult &result
) {
    summary_.begin_row();
    summary_.value(result.spec.name)
        .value(performance_axis_name(result.spec.axis))
        .value(result.spec.target)
        .value(result.spec.command_rate)
        .value(result.spec.coupled_forward_velocity)
        .value(result.spec.forward_lead_seconds)
        .value(result.spec.target_hold_seconds)
        .value(result.spec.stop_settle_seconds)
        .value(result.spec.standing_seconds)
        .value(result.leg_length_target)
        .value(forward_observation_name(
            result.forward_velocity_observation))
        .value(result.roll_restrained)
        .value(result.yaw_acceleration_feedforward_scale)
        .value(result.completed)
        .value(result.balance_engaged)
        .value(result.leg_length_valid)
        .value(result.common.finite())
        .value(result.tracked)
        .value(result.settled)
        .value(result.issue)
        .value(result.issue_phase)
        .value(result.common.wheel_contact_ratio())
        .value(result.common.other_contact_count())
        .value(result.maximum_pitch * 180.0 / BC_PI)
        .value(result.maximum_roll * 180.0 / BC_PI)
        .value(result.maximum_leg_common * 180.0 / BC_PI)
        .value(result.maximum_leg_difference * 180.0 / BC_PI)
        .value(result.maximum_roll_force_request)
        .value(result.maximum_roll_restraint_torque)
        .value(result.minimum_vertical_projection[BC_L])
        .value(result.minimum_vertical_projection[BC_R])
        .value(result.initial_position_error)
        .value(result.tracking_error.mean())
        .value(result.tracking_error.rms())
        .value(result.settle_forward.mean())
        .value(result.settle_forward.rms())
        .value(result.settle_yaw.mean())
        .value(result.settle_yaw.rms())
        .value(result.maximum_heading_error)
        .value(result.heading_error.rms())
        .value(result.stop_peak_yaw_rate);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        summary_.value(result.common.peak_wheel_torque(side));
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            summary_.value(result.common.peak_joint_torque(side, joint));
        }
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        summary_.value(result.common.wheel_saturation_ratio(side));
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            summary_.value(
                result.common.joint_saturation_ratio(side, joint));
        }
    }
    summary_.end_row();
    summary_.flush();
}

bool PerformanceBenchmark::step(
    const PerformanceCaseSpec &spec, const char *phase,
    const bc_operator_command_t &command,
    const sim::VirtualGimbalState &gimbal,
    PerformanceResult *result, const bool evaluate_tracking,
    const bool evaluate_settle, const bool stopping
) {
    roll_restraint_.apply(
        plant_.data(), runner_.snapshot().roll,
        runner_.snapshot().roll_rate);
    const ImuMotionState velocity_truth =
        sampler_.read_imu_motion(plant_.data());
    step_runner(command, gimbal);
    const SimulationSample sample = sampler_.read(
        plant_.data(), runner_.snapshot());
    if (sample_index_ % trace_stride_ == 0U) {
        write_trace(
            spec, phase, command, gimbal,
            sample, velocity_truth);
    }
    ++sample_index_;

    if (result == nullptr) return true;
    return collect(
        *result, phase, sample,
        evaluate_tracking, evaluate_settle, stopping);
}

void PerformanceBenchmark::step_runner(
    const bc_operator_command_t &command,
    const sim::VirtualGimbalState &gimbal
) {
    if (forward_velocity_.observation() ==
        ForwardVelocityObservation::wheel_odometry) {
        runner_.step_with_gimbal_heading(
            command, gimbal.world_yaw, gimbal.world_yaw_rate);
        return;
    }

    bc_sensor_feedback_t feedback{};
    adapter_.read(plant_.data(), feedback);
    feedback.gimbal = runner_.gimbal_feedback(
        gimbal.world_yaw, gimbal.world_yaw_rate, feedback.imu);
    forward_velocity_.apply(
        plant_.data(), sampler_,
        runner_.snapshot().state.value[BC_STATE_DS], feedback);
    runner_.step_with_feedback(command, feedback);
}

bool PerformanceBenchmark::collect(
    PerformanceResult &result, const char *phase,
    const SimulationSample &sample,
    const bool evaluate_tracking, const bool evaluate_settle,
    const bool stopping
) const {
    const bc_controller_snapshot_t &snapshot = sample.controller;
    result.common.observe(sample);

    const double pitch = std::abs(static_cast<double>(
        snapshot.state.value[BC_STATE_THETA_B]));
    const double roll = std::abs(static_cast<double>(snapshot.roll));
    result.maximum_pitch = std::max(result.maximum_pitch, pitch);
    result.maximum_roll = std::max(result.maximum_roll, roll);
    result.maximum_roll_force_request = std::max(
        result.maximum_roll_force_request,
        std::abs(static_cast<double>(snapshot.roll_force_request)));
    result.maximum_roll_restraint_torque = std::max(
        result.maximum_roll_restraint_torque,
        std::abs(roll_restraint_.torque()));
    result.maximum_heading_error = std::max(
        result.maximum_heading_error,
        std::abs(static_cast<double>(snapshot.heading_error)));
    result.heading_error.add(snapshot.heading_error);
    if (stopping) {
        result.stop_peak_yaw_rate = std::max(
            result.stop_peak_yaw_rate,
            std::abs(static_cast<double>(
                snapshot.state.value[BC_STATE_DPSI])));
    }

    if (!result.initial_position_error_captured) {
        result.initial_position_error =
            snapshot.state_reference.value[BC_STATE_S] -
            snapshot.state.value[BC_STATE_S];
        result.initial_position_error_captured = true;
    }
    const double theta_left = snapshot.state.value[BC_STATE_THETA_L];
    const double theta_right = snapshot.state.value[BC_STATE_THETA_R];
    result.maximum_leg_common = std::max(
        result.maximum_leg_common,
        std::abs(0.5 * (theta_left + theta_right)));
    result.maximum_leg_difference = std::max(
        result.maximum_leg_difference,
        std::abs(0.5 * (theta_left - theta_right)));

    const int theta_index[BC_SIDE_NUM] = {
        BC_STATE_THETA_L, BC_STATE_THETA_R,
    };
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const double length = snapshot.leg[side].length;
        result.leg_length_valid = result.leg_length_valid &&
            std::isfinite(length) &&
            length >= kMinimumLegLength && length <= kMaximumLegLength;
        result.minimum_vertical_projection[side] = std::min(
            result.minimum_vertical_projection[side],
            length * std::cos(snapshot.state.value[theta_index[side]]));
    }

    if (evaluate_tracking) {
        const int index = result.spec.axis == PerformanceAxis::forward ?
            BC_STATE_DS : BC_STATE_DPSI;
        result.tracking_error.add(
            snapshot.state.value[index] - result.spec.target);
    }
    if (evaluate_settle) {
        result.settle_forward.add(snapshot.state.value[BC_STATE_DS]);
        result.settle_yaw.add(snapshot.state.value[BC_STATE_DPSI]);
    }

    const std::string issue = common_diagnostic_issue(sample);
    if (!issue.empty() && result.issue == "none") {
        result.issue = issue;
        result.issue_phase = phase;
    }
    return issue != "non_finite_telemetry";
}

void PerformanceBenchmark::finish_result(
    PerformanceResult &result
) const {
    const double tracking_tolerance =
        result.spec.axis == PerformanceAxis::forward ?
            std::max(
                kForwardBaseTolerance,
                kRelativeTrackingTolerance * std::abs(result.spec.target)) :
            std::max(
                kYawBaseTolerance,
                kRelativeTrackingTolerance * std::abs(result.spec.target));

    result.tracked = result.completed && result.balance_engaged &&
        result.tracking_error.count() != 0U &&
        std::abs(result.tracking_error.mean()) <= tracking_tolerance &&
        result.tracking_error.rms() <= tracking_tolerance;
    result.settled = result.completed && result.balance_engaged &&
        result.settle_forward.count() != 0U &&
        std::abs(result.settle_forward.mean()) <= kForwardBaseTolerance &&
        result.settle_forward.rms() <= kForwardBaseTolerance &&
        std::abs(result.settle_yaw.mean()) <= kYawBaseTolerance &&
        result.settle_yaw.rms() <= kYawBaseTolerance;
}

void PerformanceBenchmark::write_trace(
    const PerformanceCaseSpec &spec, const char *phase,
    const bc_operator_command_t &command,
    const sim::VirtualGimbalState &gimbal,
    const SimulationSample &sample,
    const ImuMotionState &velocity_truth
) {
    const bc_controller_snapshot_t &snapshot = sample.controller;
    const bc_velocity_estimator_output_t &velocity =
        snapshot.velocity_estimator;
    trace_.begin_row();
    trace_.value(spec.name)
        .value(phase)
        .value(sample.time)
        .value(spec.command_rate)
        .value(leg_length_target_)
        .value(forward_observation_name(forward_velocity_.observation()))
        .value(sample.base.x)
        .value(sample.base.y)
        .value(sample.base.z)
        .value(sample.base.forward_velocity)
        .value(sample.base.vertical_velocity)
        .value(runner_.feedback().imu.specific_force_x)
        .value(runner_.feedback().imu.specific_force_y)
        .value(runner_.feedback().imu.specific_force_z)
        .value(velocity.linear_acceleration_x)
        .value(velocity.linear_acceleration_y)
        .value(velocity.prior_velocity_x)
        .value(velocity.prior_velocity_y)
        .value(velocity.velocity_x)
        .value(velocity.velocity_y)
        .value(velocity_truth.velocity_x)
        .value(velocity_truth.velocity_y)
        .value(velocity.prior_velocity_x - velocity_truth.velocity_x)
        .value(velocity.prior_velocity_y - velocity_truth.velocity_y)
        .value(velocity.velocity_x - velocity_truth.velocity_x)
        .value(velocity.velocity_y - velocity_truth.velocity_y)
        .value(velocity.acceleration_bias_x)
        .value(velocity.acceleration_bias_y)
        .value(velocity.wheel_velocity_measurement)
        .value(velocity.innovation)
        .value(velocity.innovation_variance)
        .value(velocity.nis)
        .value(static_cast<int>(velocity.measurement_accepted))
        .value(static_cast<int>(velocity.wheel_velocity_reliable))
        .value(snapshot.forward_velocity.wheel_odometry)
        .value(snapshot.forward_velocity.estimated_axle)
        .value(gimbal.world_yaw)
        .value(gimbal.world_yaw_rate)
        .value(snapshot.gimbal.relative_yaw)
        .value(snapshot.gimbal.relative_yaw_rate)
        .value(bc_chassis_alignment_name(
            snapshot.state_machine.alignment))
        .value(command.forward_velocity)
        .value(snapshot.mapped_forward_velocity)
        .value(snapshot.heading_error)
        .value(snapshot.yaw_acceleration_reference)
        .value(snapshot.state_machine.system)
        .value(snapshot.state_machine.motion)
        .value(bc_forward_state_name(snapshot.state_machine.forward));
    for (const float value : snapshot.state.value) trace_.value(value);
    for (const float value : snapshot.state_reference.value) {
        trace_.value(value);
    }
    trace_.value(snapshot.roll)
        .value(snapshot.roll_rate)
        .value(snapshot.roll_force_request)
        .value(roll_restraint_.torque());
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const bc_leg_kinematics_t &leg = snapshot.leg[side];
        trace_.value(leg.length)
            .value(leg.angle_body)
            .value(leg.length_velocity)
            .value(leg.angular_velocity);
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        trace_.value(
            forward_velocity_.wheel_radius() *
            runner_.feedback().wheel[side].angular_velocity);
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        trace_.value(sample.wheel.forward_velocity[side]);
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        trace_.value(snapshot.actuation_request.wheel_torque[side]);
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            trace_.value(
                snapshot.actuation_request.leg[side].joint_torque[joint]);
        }
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        trace_.value(snapshot.actuation.wheel_torque[side]);
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            trace_.value(snapshot.actuation.leg[side].joint_torque[joint]);
        }
    }
    trace_.value(sample.contact.wheel[BC_L])
        .value(sample.contact.wheel[BC_R])
        .value(sample.contact.other)
        .value(sample.contact.wheel_normal_force[BC_L])
        .value(sample.contact.wheel_normal_force[BC_R]);
    trace_.end_row();
}

} // namespace balance::benchmark
