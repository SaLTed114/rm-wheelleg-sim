#include "trim_scanner.hpp"

#include <algorithm>
#include <cmath>

#include "balance/math_utils.h"
#include "input/virtual_gimbal.hpp"

namespace balance::benchmark {
namespace {

constexpr double kTimestepSeconds = 0.001;
constexpr double kDisabledSeconds = 2.0;
constexpr double kEngagementTimeoutSeconds = 8.0;
constexpr double kBalanceSeconds = 8.0;
constexpr double kEvaluationSeconds = 3.0;

} // namespace

TrimScanner::TrimScanner(
    const std::filesystem::path &model_path,
    const std::filesystem::path &output_directory,
    const TrimScanConfig &config
) : plant_(model_path, kTimestepSeconds),
    adapter_(plant_.model()),
    sampler_(plant_.model()),
    config_(config),
    summary_(output_directory / "summary.csv", {
        "leg_length_target", "offset_deg", "engaged", "finite",
        "candidate", "issue", "wheel_contact_ratio",
        "other_contact_steps", "wheel_saturation_ratio",
        "joint_saturation_ratio", "max_position_error", "mean_pitch_deg",
        "rms_pitch_deg", "mean_ds", "rms_ds", "ds_slope",
        "mean_base_velocity", "rms_base_velocity", "forward_displacement",
        "mean_leg_common_deg", "mean_leg_difference_deg",
        "mean_wheel_common", "max_raw_wheel", "max_raw_joint",
        "mean_normal_force_l", "mean_normal_force_r",
    }),
    trace_(output_directory / "trace.csv", {
        "offset_deg", "time", "base_x", "base_z",
        "base_forward_velocity", "s", "ds", "theta_l", "dtheta_l",
        "theta_r", "dtheta_r", "theta_b", "dtheta_b", "ref_s",
        "ref_ds", "ref_theta_l", "ref_theta_r", "raw_wheel_l",
        "raw_wheel_r", "wheel_l", "wheel_r", "contact_wheel_l",
        "contact_wheel_r", "other_contact", "normal_force_l",
        "normal_force_r",
    }) {}

TrimResult TrimScanner::run(const double offset_deg) {
    bc_controller_config_t controller_config{};
    bc_controller_default_config(&controller_config);
    controller_config.motion.leg_length =
        static_cast<float>(config_.leg_length);
    controller_config.control.lqr_compensation.leg_angle_trim =
        static_cast<float>(offset_deg * BC_PI / 180.0);

    sim::SimulationRunner runner(plant_, adapter_, controller_config);
    runner.reset();
    bc_operator_command_t command{};
    sim::VirtualGimbal virtual_gimbal;
    bool gimbal_initialized = false;
    const auto step_runner = [&]() {
        if (runner.snapshot().state_machine.motion == BC_MOTION_ACTIVE) {
            if (!gimbal_initialized) {
                virtual_gimbal.reset(
                    runner.snapshot().state.value[BC_STATE_PSI]);
                gimbal_initialized = true;
            }
        } else {
            virtual_gimbal.reset(
                runner.snapshot().state.value[BC_STATE_PSI]);
            gimbal_initialized = false;
        }
        const sim::VirtualGimbalState &gimbal = virtual_gimbal.state();
        runner.step_with_gimbal_heading(
            command, gimbal.world_yaw, gimbal.world_yaw_rate);
    };

    TrimResult result{};
    result.offset_deg = offset_deg;
    while (plant_.data().time < kDisabledSeconds) step_runner();

    const double engagement_deadline =
        plant_.data().time + kEngagementTimeoutSeconds;
    command.system_enabled = 1U;
    while (plant_.data().time < engagement_deadline &&
           runner.snapshot().state_machine.motion !=
               BC_MOTION_ACTIVE) {
        command.balance_restart = static_cast<uint8_t>(
            runner.snapshot().state_machine.system == BC_SYSTEM_OFF);
        step_runner();
    }
    command.balance_restart = 0U;
    result.engaged = runner.snapshot().state_machine.motion ==
        BC_MOTION_ACTIVE;
    if (!result.engaged) {
        result.issue = "engagement_timeout";
        return result;
    }

    const double balance_start = plant_.data().time;
    const double evaluation_start =
        balance_start + kBalanceSeconds - kEvaluationSeconds;
    const double balance_end = balance_start + kBalanceSeconds;
    double evaluation_x = 0.0;
    double evaluation_y = 0.0;
    double evaluation_yaw = 0.0;
    bool evaluation_started = false;
    std::size_t sample_index = 0U;

    while (plant_.data().time < balance_end) {
        step_runner();
        const SimulationSample sample = sampler_.read(
            plant_.data(), runner.snapshot());
        if (sample_index % config_.trace_stride == 0U) {
            write_trace(sample, offset_deg, sample.time - balance_start);
        }
        ++sample_index;

        const std::string issue = common_diagnostic_issue(sample);
        if (issue == "non_finite_telemetry") {
            result.issue = issue;
            result.common.invalidate();
            break;
        }
        if (sample.time < evaluation_start) continue;
        if (!issue.empty() && result.issue == "none") result.issue = issue;

        if (!evaluation_started) {
            evaluation_x = sample.base.x;
            evaluation_y = sample.base.y;
            evaluation_yaw =
                sample.controller.state.value[BC_STATE_PSI];
            evaluation_started = true;
        }
        collect(result, sample, sample.time - evaluation_start);
    }

    if (evaluation_started) {
        const SimulationSample final_sample = sampler_.read(
            plant_.data(), runner.snapshot());
        const double delta_x = final_sample.base.x - evaluation_x;
        const double delta_y = final_sample.base.y - evaluation_y;
        result.forward_displacement =
            delta_x * std::cos(evaluation_yaw) +
            delta_y * std::sin(evaluation_yaw);
    }
    finish(result);
    return result;
}

void TrimScanner::write_summary(const TrimResult &result) {
    summary_.begin_row();
    summary_.value(config_.leg_length)
        .value(result.offset_deg)
        .value(result.engaged)
        .value(result.common.finite())
        .value(result.candidate)
        .value(result.issue)
        .value(result.common.wheel_contact_ratio())
        .value(result.common.other_contact_count())
        .value(result.common.any_wheel_saturation_ratio())
        .value(result.common.any_joint_saturation_ratio())
        .value(result.maximum_position_error)
        .value(result.pitch.mean() * 180.0 / BC_PI)
        .value(result.pitch.rms() * 180.0 / BC_PI)
        .value(result.velocity.mean())
        .value(result.velocity.rms())
        .value(result.velocity_trend.slope())
        .value(result.base_velocity.mean())
        .value(result.base_velocity.rms())
        .value(result.forward_displacement)
        .value(result.leg_common.mean() * 180.0 / BC_PI)
        .value(result.leg_difference.mean() * 180.0 / BC_PI)
        .value(result.wheel_common.mean())
        .value(result.common.maximum_wheel_torque())
        .value(result.common.maximum_joint_torque())
        .value(result.normal_force_left.mean())
        .value(result.normal_force_right.mean());
    summary_.end_row();
    summary_.flush();
}

void TrimScanner::collect(
    TrimResult &result, const SimulationSample &sample,
    const double evaluation_time
) const {
    const bc_controller_snapshot_t &snapshot = sample.controller;
    result.common.observe(sample);

    const double pitch = snapshot.state.value[BC_STATE_THETA_B];
    const double velocity = snapshot.state.value[BC_STATE_DS];
    const double theta_left = snapshot.state.value[BC_STATE_THETA_L];
    const double theta_right = snapshot.state.value[BC_STATE_THETA_R];
    const double raw_left =
        snapshot.actuation_request.wheel_torque[BC_L];
    const double raw_right =
        snapshot.actuation_request.wheel_torque[BC_R];

    result.pitch.add(pitch);
    result.velocity.add(velocity);
    result.base_velocity.add(sample.base.forward_velocity);
    result.velocity_trend.add(evaluation_time, velocity);
    result.leg_common.add(0.5 * (theta_left + theta_right));
    result.leg_difference.add(0.5 * (theta_left - theta_right));
    result.wheel_common.add(0.5 * (raw_left + raw_right));
    result.normal_force_left.add(
        sample.contact.wheel_normal_force[BC_L]);
    result.normal_force_right.add(
        sample.contact.wheel_normal_force[BC_R]);
    result.maximum_position_error = std::max(
        result.maximum_position_error,
        std::abs(static_cast<double>(
            snapshot.state_reference.value[BC_STATE_S] -
            snapshot.state.value[BC_STATE_S])));
    if (!std::isfinite(sample.base.forward_velocity)) {
        result.common.invalidate();
    }
}

void TrimScanner::finish(TrimResult &result) const {
    result.candidate = result.engaged && result.common.finite() &&
        result.issue == "none" && result.common.sample_count() != 0U &&
        result.common.wheel_contact_ratio() >= 0.99 &&
        result.common.other_contact_count() == 0U &&
        result.common.any_wheel_saturation_ratio() == 0.0 &&
        result.common.any_joint_saturation_ratio() == 0.0 &&
        std::abs(result.pitch.mean()) <= BC_PI / 180.0 &&
        std::abs(result.velocity.mean()) <= 0.02 &&
        std::abs(result.velocity_trend.slope()) <= 0.01;
}

void TrimScanner::write_trace(
    const SimulationSample &sample,
    const double offset_deg, const double time
) {
    const bc_controller_snapshot_t &snapshot = sample.controller;
    trace_.begin_row();
    trace_.value(offset_deg)
        .value(time)
        .value(sample.base.x)
        .value(sample.base.z)
        .value(sample.base.forward_velocity);
    for (const int index : {
             BC_STATE_S, BC_STATE_DS,
             BC_STATE_THETA_L, BC_STATE_DTHETA_L,
             BC_STATE_THETA_R, BC_STATE_DTHETA_R,
             BC_STATE_THETA_B, BC_STATE_DTHETA_B,
         }) {
        trace_.value(snapshot.state.value[index]);
    }
    for (const int index : {BC_STATE_S, BC_STATE_DS}) {
        trace_.value(snapshot.state_reference.value[index]);
    }
    const double offset = offset_deg * BC_PI / 180.0;
    trace_.value(
            snapshot.state_reference.value[BC_STATE_THETA_L] + offset)
        .value(snapshot.state_reference.value[BC_STATE_THETA_R] + offset)
        .value(snapshot.actuation_request.wheel_torque[BC_L])
        .value(snapshot.actuation_request.wheel_torque[BC_R])
        .value(snapshot.actuation.wheel_torque[BC_L])
        .value(snapshot.actuation.wheel_torque[BC_R])
        .value(sample.contact.wheel[BC_L])
        .value(sample.contact.wheel[BC_R])
        .value(sample.contact.other)
        .value(sample.contact.wheel_normal_force[BC_L])
        .value(sample.contact.wheel_normal_force[BC_R]);
    trace_.end_row();
}

} // namespace balance::benchmark
