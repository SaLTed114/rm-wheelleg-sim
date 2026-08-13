#include "ramp_climb.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "balance/math_utils.h"
#include "common/common_diagnostics.hpp"

namespace balance::benchmark {
namespace {

constexpr double kDisabledSettleSeconds = 2.0;
constexpr double kEngagementTimeoutSeconds = 10.0;
constexpr double kForwardVelocity = 2.0;
constexpr double kForwardCommandSeconds = 4.5;
constexpr double kPostReleaseObservationSeconds = 7.0;
constexpr double kGroundRecoveryHoldSeconds = 0.1;
constexpr double kDivergenceAngle = 60.0 * BC_PI / 180.0;

int require_id(const mjModel &model, const mjtObj type, const char *name) {
    const int id = mj_name2id(&model, type, name);
    if (id < 0) {
        throw std::runtime_error(
            "missing MuJoCo object '" + std::string(name) + "'");
    }
    return id;
}

bc_gimbal_feedback_t held_heading_feedback(
    const bc_controller_snapshot_t &snapshot,
    const float heading
) {
    return {
        bc_wrap_anglef(heading - snapshot.state.value[BC_STATE_PSI]),
        -snapshot.state.value[BC_STATE_DPSI],
    };
}

} // namespace

RampClimbBenchmark::RampClimbBenchmark(
    const std::filesystem::path &model_path,
    const std::filesystem::path &output_directory
) : output_directory_(output_directory),
    plant_(model_path, 0.001),
    adapter_(plant_.model()),
    runner_(plant_, adapter_),
    sampler_(plant_.model()) {
    ramp_ = require_id(
        plant_.model(), mjOBJ_GEOM, "benchmark_ramp_17deg");
    wheel_[BC_L] = require_id(
        plant_.model(), mjOBJ_GEOM, "Right_wheel_collision");
    wheel_[BC_R] = require_id(
        plant_.model(), mjOBJ_GEOM, "Left_wheel_collision");
}

RampClimbBenchmark::RampContact
RampClimbBenchmark::read_ramp_contact() const {
    RampContact result{};
    for (int index = 0; index < plant_.data().ncon; ++index) {
        const mjContact &contact = plant_.data().contact[index];
        if (contact.geom[0] != ramp_ && contact.geom[1] != ramp_) continue;
        const int other = contact.geom[0] == ramp_ ?
            contact.geom[1] : contact.geom[0];
        bool wheel = false;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            if (other == wheel_[side]) {
                result.wheel[side] = true;
                wheel = true;
            }
        }
        if (!wheel) {
            result.non_wheel = true;
            if (result.first_non_wheel == "none") {
                const char *name = mj_id2name(
                    &plant_.model(), mjOBJ_GEOM, other);
                if (name != nullptr) {
                    result.first_non_wheel = name;
                } else {
                    const int body = plant_.model().geom_bodyid[other];
                    const char *body_name = mj_id2name(
                        &plant_.model(), mjOBJ_BODY, body);
                    result.first_non_wheel = body_name != nullptr ?
                        body_name : "unnamed_robot_geom";
                }
            }
        }
    }
    return result;
}

RampClimbResult RampClimbBenchmark::run() {
    runner_.reset();
    plant_.configure_ramp_climb_benchmark();
    RampClimbResult result{};
    CsvWriter trace(output_directory_ / "trace.csv", {
        "time", "axle_x", "base_z", "truth_velocity",
        "target_velocity", "pitch", "pitch_rate",
        "support_phase", "specific_force_norm",
        "support_force_l", "support_force_r",
        "support_state_l", "support_state_r",
        "leg_length_l", "leg_length_r",
        "leg_length_velocity_l", "leg_length_velocity_r",
        "support_target_l", "support_target_r",
        "kf_velocity", "kf_wheel_measurement", "kf_innovation",
        "kf_nis", "kf_measurement_accepted", "kf_wheel_reliable",
        "kf_reacquisition_elapsed", "kf_reacquisition_active",
        "s", "ref_s", "ds", "ref_ds", "forward_mode",
        "ground_wheel_l", "ground_wheel_r",
        "ramp_wheel_l", "ramp_wheel_r",
        "truth_any_wheel_contact", "ramp_non_wheel_contact",
        "ramp_non_wheel_kind",
    });

    bc_operator_command_t command{};
    float held_heading{};
    double recovery_hold_start = -1.0;
    while (plant_.data().time < kDisabledSettleSeconds) {
        held_heading = runner_.snapshot().state.value[BC_STATE_PSI];
        runner_.step(
            command, held_heading_feedback(runner_.snapshot(), held_heading));
    }

    command.system_enabled = 1U;
    while (plant_.data().time < kEngagementTimeoutSeconds &&
           !result.balance_engaged) {
        const auto &snapshot = runner_.snapshot();
        held_heading = snapshot.state.value[BC_STATE_PSI];
        command.balance_restart = static_cast<uint8_t>(
            snapshot.state_machine.system == BC_SYSTEM_OFF);
        runner_.step(
            command, held_heading_feedback(snapshot, held_heading));
        if (runner_.snapshot().state_machine.motion == BC_MOTION_ACTIVE) {
            result.balance_engaged = true;
            result.active_time = plant_.data().time;
        }
    }
    command.balance_restart = 0U;
    command.forward_velocity = static_cast<float>(kForwardVelocity);

    const double forward_end_time =
        plant_.data().time + kForwardCommandSeconds;
    const double end_time =
        forward_end_time + kPostReleaseObservationSeconds;
    bool wheel_was_reliable =
        runner_.snapshot().velocity_estimator.wheel_velocity_reliable;
    while (plant_.data().time < end_time) {
        if (plant_.data().time >= forward_end_time &&
            command.forward_velocity != 0.0F) {
            command.forward_velocity = 0.0F;
            result.command_release_time = plant_.data().time;
        }
        runner_.step(
            command,
            held_heading_feedback(runner_.snapshot(), held_heading));
        const SimulationSample sample = sampler_.read(
            plant_.data(), runner_.snapshot());
        const RampContact ramp_contact = read_ramp_contact();
        write_trace(
            trace, sample, ramp_contact, command.forward_velocity);

        result.finite = result.finite &&
            controller_snapshot_is_finite(sample.controller);
        const double pitch = std::abs(static_cast<double>(
            sample.controller.state.value[BC_STATE_THETA_B]));
        result.maximum_pitch = std::max(result.maximum_pitch, pitch);
        result.attitude_diverged = result.attitude_diverged ||
            pitch > kDivergenceAngle ||
            std::abs(static_cast<double>(sample.controller.roll)) >
                kDivergenceAngle;

        if (!result.clearance_collision && ramp_contact.non_wheel) {
            result.clearance_collision = true;
            result.collision_time = sample.time;
            result.collision_axle_x = sample.axle.x;
            result.first_collision = ramp_contact.first_non_wheel;
        }

        const bool truth_any_wheel_contact =
            sample.contact.wheel[BC_L] || sample.contact.wheel[BC_R] ||
            ramp_contact.wheel[BC_L] || ramp_contact.wheel[BC_R];
        const bool support_airborne =
            sample.controller.state_machine.support == BC_SUPPORT_AIRBORNE;
        if (!result.airborne_after_collision &&
            result.clearance_collision && support_airborne) {
            result.airborne_after_collision = true;
            result.airborne_time = sample.time;
            result.wheel_contact_at_airborne = truth_any_wheel_contact;
        }

        if (result.airborne_after_collision &&
            !result.wheel_recontact_after_airborne &&
            truth_any_wheel_contact) {
            result.wheel_recontact_after_airborne = true;
            result.wheel_recontact_time = sample.time;
        }

        const bool support_estimator_all_ground =
            sample.controller.support_force[BC_L].valid &&
            sample.controller.support_force[BC_R].valid &&
            sample.controller.support_force[BC_L].state ==
                BC_CONTACT_GROUND &&
            sample.controller.support_force[BC_R].state ==
                BC_CONTACT_GROUND;
        if (result.wheel_recontact_after_airborne &&
            !result.support_estimator_recovered &&
            support_estimator_all_ground) {
            result.support_estimator_recovered = true;
            result.support_estimator_recovery_time = sample.time;
        }

        if (result.wheel_recontact_after_airborne &&
            !result.landing_recovery_started &&
            sample.controller.state_machine.support ==
                BC_SUPPORT_LANDING_RETRACT) {
            result.landing_recovery_started = true;
            result.landing_recovery_time = sample.time;
        }

        if (!result.support_phase_recovered &&
            result.wheel_recontact_after_airborne &&
            sample.controller.state_machine.support == BC_SUPPORT_GROUND &&
            truth_any_wheel_contact) {
            if (recovery_hold_start < 0.0) {
                recovery_hold_start = sample.time;
            } else if (sample.time - recovery_hold_start >=
                       kGroundRecoveryHoldSeconds) {
                result.support_phase_recovered = true;
                result.ground_recovery_time = sample.time;
            }
        } else {
            recovery_hold_start = -1.0;
        }

        const bool wheel_reliable =
            sample.controller.velocity_estimator.wheel_velocity_reliable;
        if (result.clearance_collision && wheel_was_reliable &&
            !wheel_reliable) {
            result.wheel_reliability_lost = true;
            result.wheel_reliability_recovered = false;
            result.wheel_reliability_recovery_time =
                std::numeric_limits<double>::quiet_NaN();
        } else if (result.wheel_reliability_lost &&
                   !result.wheel_reliability_recovered && wheel_reliable) {
            result.wheel_reliability_recovered = true;
            result.wheel_reliability_recovery_time = sample.time;
        }
        result.wheel_reacquisition_used =
            result.wheel_reacquisition_used ||
            sample.controller.velocity_estimator.reacquisition_active;
        wheel_was_reliable = wheel_reliable;
        result.forward_hold_recovered =
            std::isfinite(result.command_release_time) &&
            sample.controller.state_machine.forward == BC_FORWARD_HOLD;
        result.final_truth_velocity = 0.5 * (
            sample.wheel.forward_velocity[BC_L] +
            sample.wheel.forward_velocity[BC_R]);
        result.final_estimated_velocity =
            sample.controller.state.value[BC_STATE_DS];
        result.final_wheel_velocity =
            sample.controller.forward_velocity.wheel_odometry;
        result.final_position_error =
            sample.controller.state_reference.value[BC_STATE_S] -
            sample.controller.state.value[BC_STATE_S];
        if (!result.finite) break;
    }
    trace.flush();

    CsvWriter summary(output_directory_ / "summary.csv", {
        "balance_engaged", "finite", "clearance_collision",
        "first_collision", "collision_seconds_after_active",
        "collision_axle_x", "airborne_after_collision",
        "airborne_seconds_after_collision", "wheel_contact_at_airborne",
        "wheel_recontact_after_airborne",
        "wheel_recontact_seconds_after_airborne",
        "support_estimator_recovered",
        "support_estimator_recovery_seconds_after_recontact",
        "landing_recovery_started",
        "landing_recovery_seconds_after_recontact",
        "support_phase_recovered",
        "ground_recovery_seconds_after_recontact",
        "command_release_seconds_after_active",
        "wheel_reliability_lost", "wheel_reliability_recovered",
        "wheel_reliability_recovery_seconds_after_release",
        "wheel_reacquisition_used",
        "forward_hold_recovered", "final_truth_velocity",
        "final_estimated_velocity", "final_wheel_velocity",
        "final_position_error",
        "attitude_diverged", "maximum_pitch_deg",
    });
    summary.begin_row();
    summary.value(result.balance_engaged)
        .value(result.finite)
        .value(result.clearance_collision)
        .value(result.first_collision)
        .value(result.collision_time - result.active_time)
        .value(result.collision_axle_x)
        .value(result.airborne_after_collision)
        .value(result.airborne_time - result.collision_time)
        .value(result.wheel_contact_at_airborne)
        .value(result.wheel_recontact_after_airborne)
        .value(result.wheel_recontact_time - result.airborne_time)
        .value(result.support_estimator_recovered)
        .value(result.support_estimator_recovery_time -
            result.wheel_recontact_time)
        .value(result.landing_recovery_started)
        .value(result.landing_recovery_time - result.wheel_recontact_time)
        .value(result.support_phase_recovered)
        .value(result.ground_recovery_time - result.wheel_recontact_time)
        .value(result.command_release_time - result.active_time)
        .value(result.wheel_reliability_lost)
        .value(result.wheel_reliability_recovered)
        .value(result.wheel_reliability_recovery_time -
            result.command_release_time)
        .value(result.wheel_reacquisition_used)
        .value(result.forward_hold_recovered)
        .value(result.final_truth_velocity)
        .value(result.final_estimated_velocity)
        .value(result.final_wheel_velocity)
        .value(result.final_position_error)
        .value(result.attitude_diverged)
        .value(result.maximum_pitch * 180.0 / BC_PI);
    summary.end_row();
    summary.flush();
    return result;
}

void RampClimbBenchmark::write_trace(
    CsvWriter &trace,
    const SimulationSample &sample,
    const RampContact &ramp_contact,
    const double target_velocity
) const {
    const auto &snapshot = sample.controller;
    const double truth_velocity = 0.5 * (
        sample.wheel.forward_velocity[BC_L] +
        sample.wheel.forward_velocity[BC_R]);
    const bool any_wheel =
        sample.contact.wheel[BC_L] || sample.contact.wheel[BC_R] ||
        ramp_contact.wheel[BC_L] || ramp_contact.wheel[BC_R];
    trace.begin_row();
    trace.value(sample.time)
        .value(sample.axle.x)
        .value(sample.base.z)
        .value(truth_velocity)
        .value(target_velocity)
        .value(snapshot.state.value[BC_STATE_THETA_B])
        .value(snapshot.state.value[BC_STATE_DTHETA_B])
        .value(bc_support_phase_state_name(snapshot.state_machine.support))
        .value(snapshot.specific_force_norm)
        .value(snapshot.support_force[BC_L].filtered_vertical_force)
        .value(snapshot.support_force[BC_R].filtered_vertical_force)
        .value(bc_contact_state_name(snapshot.support_force[BC_L].state))
        .value(bc_contact_state_name(snapshot.support_force[BC_R].state))
        .value(snapshot.leg[BC_L].length)
        .value(snapshot.leg[BC_R].length)
        .value(snapshot.leg[BC_L].length_velocity)
        .value(snapshot.leg[BC_R].length_velocity)
        .value(snapshot.support_request.leg[BC_L].target)
        .value(snapshot.support_request.leg[BC_R].target)
        .value(snapshot.velocity_estimator.velocity_x)
        .value(snapshot.velocity_estimator.wheel_velocity_measurement)
        .value(snapshot.velocity_estimator.innovation)
        .value(snapshot.velocity_estimator.nis)
        .value(static_cast<int>(
            snapshot.velocity_estimator.measurement_accepted))
        .value(static_cast<int>(
            snapshot.velocity_estimator.wheel_velocity_reliable))
        .value(snapshot.velocity_estimator.reacquisition_elapsed_seconds)
        .value(static_cast<int>(
            snapshot.velocity_estimator.reacquisition_active))
        .value(snapshot.state.value[BC_STATE_S])
        .value(snapshot.state_reference.value[BC_STATE_S])
        .value(snapshot.state.value[BC_STATE_DS])
        .value(snapshot.state_reference.value[BC_STATE_DS])
        .value(bc_forward_state_name(snapshot.state_machine.forward))
        .value(sample.contact.wheel[BC_L])
        .value(sample.contact.wheel[BC_R])
        .value(ramp_contact.wheel[BC_L])
        .value(ramp_contact.wheel[BC_R])
        .value(any_wheel)
        .value(ramp_contact.non_wheel)
        .value(ramp_contact.first_non_wheel);
    trace.end_row();
}

} // namespace balance::benchmark
