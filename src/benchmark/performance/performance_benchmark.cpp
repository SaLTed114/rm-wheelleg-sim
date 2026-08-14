#include "performance_benchmark.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "balance/math_utils.h"
#include "performance_metrics.hpp"

namespace balance::benchmark {
namespace {

constexpr double kTimestepSeconds = 0.001;
constexpr double kMinimumLegLength = 0.13;
constexpr double kMaximumLegLength = 0.39;
constexpr double kForwardBaseTolerance = 0.10;
constexpr double kYawBaseTolerance = 0.20;
constexpr double kRelativeTrackingTolerance = 0.10;
constexpr double kFormalForwardT90Limit = 0.8;
constexpr double kFormalPitchLimit = 6.0 * BC_PI / 180.0;
constexpr double kFigureEightClosureTolerance = 0.15;

bc_controller_config_t controller_config(
    const std::optional<double> leg_length,
    const std::optional<double> forward_acceleration_rate,
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
    if (forward_acceleration_rate) {
        if (!std::isfinite(*forward_acceleration_rate) ||
            *forward_acceleration_rate <= 0.0) {
            throw std::invalid_argument(
                "forward acceleration rate must be finite and positive");
        }
        config.motion.forward_reference.velocity_ramp.rate_limit =
            static_cast<float>(*forward_acceleration_rate);
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
        controller_config(std::nullopt, std::nullopt, override);
    return config.control.lqr_compensation.
        yaw_acceleration_feedforward_scale;
}

} // namespace

bc_controller_config_t performance_controller_config(
    const PerformanceBenchmarkConfig &config
) {
    return controller_config(
        config.leg_length,
        config.forward_acceleration_rate,
        config.yaw_acceleration_feedforward_scale);
}

PerformanceBenchmark::PerformanceBenchmark(
    const std::filesystem::path &model_path,
    const std::filesystem::path &output_directory,
    const PerformanceBenchmarkConfig &config
) : plant_(model_path, kTimestepSeconds),
    adapter_(plant_.model()),
    runner_(plant_, adapter_, performance_controller_config(config)),
    sampler_(plant_.model()),
    roll_restraint_(plant_.model(), config.roll_restrained),
    forward_velocity_(
        config.forward_velocity_observation,
        performance_controller_config(config).
                control.observer.wheel_radius),
    summary_(output_directory / "summary.csv", {
        "case", "kind", "forward_target", "yaw_target",
        "forward_rate", "yaw_rate",
        "coupled_forward_velocity", "forward_lead_seconds",
        "target_hold_seconds",
        "stop_settle_seconds", "standing_seconds", "leg_length_target",
        "forward_velocity_observation", "roll_restrained",
        "yaw_acceleration_feedforward_scale",
        "completed", "balance_engaged", "leg_length_valid", "finite",
        "valid", "response_pass", "stop_pass", "contact_free",
        "unsaturated", "entry_ready", "entry_timed_out",
        "entry_wait_seconds", "issue", "issue_phase",
        "t10", "t50", "t90", "t10_t90_acceleration", "overshoot",
        "wheel_contact_ratio", "wheel_contact_ratio_l",
        "wheel_contact_ratio_r", "minimum_normal_force_l",
        "minimum_normal_force_r", "other_contact_steps", "max_pitch_deg",
        "max_roll_deg", "max_leg_common_deg", "max_leg_difference_deg",
        "peak_roll_force_request",
        "peak_roll_restraint_torque", "min_vertical_l", "min_vertical_r",
        "initial_s_error", "forward_mean_error", "forward_rmse",
        "yaw_mean_error", "yaw_rmse", "actual_forward_mean",
        "actual_yaw_mean", "lateral_acceleration_mean",
        "path_closure_error",
        "hold_wheel_contact_ratio", "hold_wheel_contact_ratio_l",
        "hold_wheel_contact_ratio_r", "hold_minimum_normal_force_l",
        "hold_minimum_normal_force_r", "settle_mean_ds",
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
        "case", "phase", "simulation_time", "forward_rate", "yaw_rate",
        "leg_length_target", "forward_velocity_observation",
        "base_x", "base_y", "base_z", "axle_x", "axle_y",
        "base_forward_velocity",
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
        "velocity_variance_x", "velocity_rejection_elapsed_seconds",
        "velocity_recovery_elapsed_seconds",
        "velocity_measurement_accepted", "wheel_velocity_reliable",
        "wheel_odometry_velocity", "estimated_axle_velocity",
        "gimbal_yaw", "gimbal_yaw_rate",
        "gimbal_relative_yaw", "gimbal_relative_yaw_rate",
        "alignment", "command_forward", "mapped_forward",
        "heading_error", "ref_ddpsi",
        "system", "motion", "forward", "support_phase", "s", "ds", "psi", "dpsi",
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
        "support_force_raw_l", "support_force_raw_r",
        "support_force_filtered_l", "support_force_filtered_r",
        "support_contact_l", "support_contact_r",
        "contact_wheel_l", "contact_wheel_r", "other_contact",
        "normal_force_l", "normal_force_r",
        "impact_forward_acceleration", "impact_vertical_acceleration",
        "impact_5ms_valid", "impact_5ms_forward_dv",
        "impact_5ms_vertical_dv", "impact_5ms_pitch_rate_delta",
        "impact_5ms_leg_rate_delta_l", "impact_5ms_leg_rate_delta_r",
        "impact_5ms_wheel_velocity_delta",
        "impact_5ms_wheel_imu_mismatch",
        "impact_10ms_valid", "impact_10ms_forward_dv",
        "impact_10ms_vertical_dv", "impact_10ms_pitch_rate_delta",
        "impact_10ms_leg_rate_delta_l", "impact_10ms_leg_rate_delta_r",
        "impact_10ms_wheel_velocity_delta",
        "impact_10ms_wheel_imu_mismatch",
    }),
    yaw_acceleration_feedforward_scale_(
        effective_yaw_acceleration_feedforward_scale(
            config.yaw_acceleration_feedforward_scale)),
    trace_stride_(config.trace_stride) {
    if (trace_stride_ == 0U) {
        throw std::invalid_argument("trace stride must be positive");
    }
    const bc_controller_config_t controller =
        performance_controller_config(config);
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
        const GroundContactState contact =
            sampler_.read_contacts(plant_.data());
        scenario.update(
            runner_.snapshot(), plant_.data().time,
            contact.wheel[BC_L] && contact.wheel[BC_R]);
        result.entry_ready = scenario.entry_ready();
        result.entry_timed_out = scenario.entry_timed_out();
        result.entry_wait_seconds = scenario.entry_wait_seconds();
        if (result.entry_timed_out && result.issue == "none") {
            result.issue = "entry_not_ready";
            result.issue_phase = "entry_wait";
        }
        if (scenario.finished()) break;

        PerformanceResult *monitored =
            scenario.monitored() ? &result : nullptr;
        if (!step(
                spec, scenario.phase_name(), scenario.command(),
                scenario.gimbal(), monitored,
                scenario.tracking_evaluation(),
                scenario.settle_evaluation(),
                scenario.phase() == PerformancePhase::yaw_stop_ramp ||
                    scenario.phase() == PerformancePhase::forward_stop_ramp ||
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
        .value(performance_case_kind_name(result.spec.kind))
        .value(result.spec.forward_target)
        .value(result.spec.yaw_target)
        .value(result.spec.forward_rate)
        .value(result.spec.yaw_rate)
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
        .value(result.valid)
        .value(result.response_pass)
        .value(result.stop_pass)
        .value(result.contact_free)
        .value(result.unsaturated)
        .value(result.entry_ready)
        .value(result.entry_timed_out)
        .value(result.entry_wait_seconds)
        .value(result.issue)
        .value(result.issue_phase)
        .value(result.t10)
        .value(result.t50)
        .value(result.t90)
        .value(result.t10_t90_acceleration)
        .value(result.overshoot)
        .value(result.common.wheel_contact_ratio())
        .value(result.common.wheel_contact_ratio(BC_L))
        .value(result.common.wheel_contact_ratio(BC_R))
        .value(result.common.minimum_wheel_normal_force(BC_L))
        .value(result.common.minimum_wheel_normal_force(BC_R))
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
        .value(result.forward_error.mean())
        .value(result.forward_error.rms())
        .value(result.yaw_error.mean())
        .value(result.yaw_error.rms())
        .value(result.actual_forward.mean())
        .value(result.actual_yaw.mean())
        .value(result.lateral_acceleration.mean())
        .value(result.path_closure_error)
        .value(result.hold.wheel_contact_ratio())
        .value(result.hold.wheel_contact_ratio(BC_L))
        .value(result.hold.wheel_contact_ratio(BC_R))
        .value(result.hold.minimum_wheel_normal_force(BC_L))
        .value(result.hold.minimum_wheel_normal_force(BC_R))
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
    if (evaluate_tracking) result.hold.observe(sample);

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

    collect_response_timing(result, sample, stopping);

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
        const double actual_forward = snapshot.state.value[BC_STATE_DS];
        const double actual_yaw = snapshot.state.value[BC_STATE_DPSI];
        const bool trajectory =
            result.spec.kind == PerformanceCaseKind::figure_eight;
        const double forward_target = trajectory ?
            snapshot.state_reference.value[BC_STATE_DS] :
            result.spec.forward_target;
        const double yaw_target = trajectory ?
            snapshot.state_reference.value[BC_STATE_DPSI] :
            result.spec.yaw_target;
        result.forward_error.add(
            actual_forward - forward_target);
        result.yaw_error.add(actual_yaw - yaw_target);
        result.actual_forward.add(actual_forward);
        result.actual_yaw.add(actual_yaw);
        result.lateral_acceleration.add(actual_forward * actual_yaw);
        if (trajectory) {
            if (!result.path_start_captured) {
                result.path_start_x = sample.axle.x;
                result.path_start_y = sample.axle.y;
                result.path_start_captured = true;
            }
            result.path_end_x = sample.axle.x;
            result.path_end_y = sample.axle.y;
        }
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
    if (issue == "attitude_termination") {
        result.attitude_terminated = true;
    }
    return issue != "non_finite_telemetry" &&
        issue != "attitude_termination";
}

void PerformanceBenchmark::collect_response_timing(
    PerformanceResult &result,
    const SimulationSample &sample,
    const bool stopping
) const {
    if (result.spec.kind != PerformanceCaseKind::forward_response ||
        stopping) {
        return;
    }
    const double target = result.spec.forward_target;
    const double actual = sample.controller.state.value[BC_STATE_DS];
    if (!result.response_started) {
        result.response_started = true;
        result.response_start_time = sample.time;
        result.response_initial_forward = actual;
    }
    const double elapsed = sample.time - result.response_start_time;
    const double progress = normalized_response_progress(
        actual, result.response_initial_forward, target);
    result.maximum_forward_progress = std::max(
        result.maximum_forward_progress, progress);
    capture_response_crossing(progress, 0.1, elapsed, result.t10);
    capture_response_crossing(progress, 0.5, elapsed, result.t50);
    capture_response_crossing(progress, 0.9, elapsed, result.t90);
}

void PerformanceBenchmark::finish_result(
    PerformanceResult &result
) const {
    if (result.path_start_captured) {
        result.path_closure_error = std::hypot(
            result.path_end_x - result.path_start_x,
            result.path_end_y - result.path_start_y);
    }
    const double forward_tolerance = std::max(
        kForwardBaseTolerance,
        kRelativeTrackingTolerance * std::abs(result.spec.forward_target));
    const double yaw_tolerance = std::max(
        kYawBaseTolerance,
        kRelativeTrackingTolerance * std::abs(result.spec.yaw_target));
    result.valid = result.completed && result.balance_engaged &&
        result.leg_length_valid && result.common.finite() &&
        !result.attitude_terminated && !result.entry_timed_out;

    const bool forward_required =
        result.spec.kind != PerformanceCaseKind::heading_response;
    const bool yaw_required =
        result.spec.kind != PerformanceCaseKind::forward_response;
    const bool forward_response = !forward_required ||
        (result.forward_error.count() != 0U &&
         std::abs(result.forward_error.mean()) <= forward_tolerance &&
         result.forward_error.rms() <= forward_tolerance);
    const bool yaw_response = !yaw_required ||
        (result.yaw_error.count() != 0U &&
         std::abs(result.yaw_error.mean()) <= yaw_tolerance &&
         result.yaw_error.rms() <= yaw_tolerance);
    result.response_pass = result.valid && forward_response && yaw_response;
    if (result.spec.kind == PerformanceCaseKind::figure_eight) {
        result.response_pass = result.response_pass &&
            result.path_start_captured &&
            result.path_closure_error <= kFigureEightClosureTolerance;
    }

    if (result.spec.formal_acceptance &&
        result.spec.kind == PerformanceCaseKind::forward_response) {
        result.response_pass = result.response_pass &&
            std::isfinite(result.t90) &&
            result.t90 < kFormalForwardT90Limit &&
            result.maximum_pitch <= kFormalPitchLimit;
    }
    result.stop_pass = result.completed && result.balance_engaged &&
        result.settle_forward.count() != 0U &&
        std::abs(result.settle_forward.mean()) <= kForwardBaseTolerance &&
        result.settle_forward.rms() <= kForwardBaseTolerance &&
        std::abs(result.settle_yaw.mean()) <= kYawBaseTolerance &&
        result.settle_yaw.rms() <= kYawBaseTolerance;

    result.contact_free = result.common.other_contact_count() == 0U &&
        result.common.wheel_contact_ratio() >= 1.0 - 1.0e-12;
    result.unsaturated =
        result.common.any_wheel_saturation_ratio() == 0.0 &&
        result.common.any_joint_saturation_ratio() == 0.0;
    result.overshoot = response_overshoot(
        result.maximum_forward_progress,
        result.response_initial_forward, result.spec.forward_target);
    result.t10_t90_acceleration = t10_t90_acceleration(
        result.t10, result.t90,
        result.response_initial_forward, result.spec.forward_target);
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
        .value(spec.forward_rate)
        .value(spec.yaw_rate)
        .value(leg_length_target_)
        .value(forward_observation_name(forward_velocity_.observation()))
        .value(sample.base.x)
        .value(sample.base.y)
        .value(sample.base.z)
        .value(sample.axle.x)
        .value(sample.axle.y)
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
        .value(velocity.velocity_variance_x)
        .value(velocity.rejection_elapsed_seconds)
        .value(velocity.recovery_elapsed_seconds)
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
        .value(bc_forward_state_name(snapshot.state_machine.forward))
        .value(bc_support_phase_state_name(
            snapshot.state_machine.support));
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
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        trace_.value(snapshot.support_force[side].vertical_force);
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        trace_.value(snapshot.support_force[side].filtered_vertical_force);
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        trace_.value(bc_contact_state_name(
            snapshot.support_force[side].state));
    }
    trace_.value(sample.contact.wheel[BC_L])
        .value(sample.contact.wheel[BC_R])
        .value(sample.contact.other)
        .value(sample.contact.wheel_normal_force[BC_L])
        .value(sample.contact.wheel_normal_force[BC_R]);
    const bc_impact_observer_output_t &impact = snapshot.impact_observer;
    trace_.value(impact.forward_acceleration)
        .value(impact.vertical_acceleration);
    for (int window = 0; window < BC_IMPACT_WINDOW_NUM; ++window) {
        const bc_impact_window_output_t &output = impact.window[window];
        trace_.value(static_cast<int>(output.valid))
            .value(output.forward_delta_velocity)
            .value(output.vertical_delta_velocity)
            .value(output.pitch_rate_delta)
            .value(output.leg_rate_delta[BC_L])
            .value(output.leg_rate_delta[BC_R])
            .value(output.wheel_velocity_delta)
            .value(output.wheel_imu_delta_mismatch);
    }
    trace_.end_row();
}

} // namespace balance::benchmark
