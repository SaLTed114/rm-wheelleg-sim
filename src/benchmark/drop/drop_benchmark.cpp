#include "drop_benchmark.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "balance/math_utils.h"

namespace balance::benchmark {
namespace {

constexpr double kTimestepSeconds = 0.001;
constexpr double kStandingSeconds = 2.0;
constexpr double kDisabledSettleSeconds = 2.0;
constexpr double kEngagementTimeoutSeconds = 10.0;
constexpr double kDropTimeoutSeconds = 3.0;
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

std::string contact_kind(const GroundContactState &contact) {
    if (contact.wheel[BC_L] && contact.wheel[BC_R]) return "both_wheels";
    if (contact.wheel[BC_L]) return "wheel_l";
    if (contact.wheel[BC_R]) return "wheel_r";
    if (contact.other) return contact.unexpected.empty() ?
        "other" : "other:" + contact.unexpected;
    return "none";
}

double radians_to_degrees(const double value) {
    return value * 180.0 / BC_PI;
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

std::string drop_case_name(const DropCaseSpec &spec) {
    std::ostringstream name;
    name << drop_air_policy_name(spec.policy) << "_pitch_rate_";
    if (spec.initial_pitch_rate > 0.0) name << "pos_";
    else if (spec.initial_pitch_rate < 0.0) name << "neg_";
    else name << "zero_";
    name << std::fixed << std::setprecision(1)
         << std::abs(spec.initial_pitch_rate);
    std::string result = name.str();
    std::replace(result.begin(), result.end(), '.', 'p');
    return result;
}

const std::array<DropCaseSpec, 6> &drop_exploration_cases() {
    static const std::array<DropCaseSpec, 6> cases{{
        {DropAirPolicy::length_only,    0.2, -0.5},
        {DropAirPolicy::length_only,    0.2,  0.0},
        {DropAirPolicy::length_only,    0.2, +0.5},
        {DropAirPolicy::leg_lqr,        0.2, -0.5},
        {DropAirPolicy::leg_lqr,        0.2,  0.0},
        {DropAirPolicy::leg_lqr,        0.2, +0.5},
    }};
    return cases;
}

const DropCaseSpec *find_drop_case(const std::string_view name) noexcept {
    const auto &cases = drop_exploration_cases();
    const auto found = std::find_if(
        cases.begin(), cases.end(),
        [name](const DropCaseSpec &spec) {
            return drop_case_name(spec) == name;
        });
    return found == cases.end() ? nullptr : &*found;
}

DropBenchmark::DropBenchmark(
    const std::filesystem::path &model_path,
    const std::filesystem::path &output_directory
) : output_directory_(output_directory),
    plant_(model_path, kTimestepSeconds),
    adapter_(plant_.model()),
    runner_(plant_, adapter_),
    sampler_(plant_.model()),
    summary_(output_directory / "summary.csv", {
        "case", "policy", "wheel_clearance_target",
        "initial_pitch_rate", "completed", "finite", "attitude_diverged",
        "balance_engaged", "touchdown", "first_contact_kind",
        "release_clearance_l", "release_clearance_r",
        "first_contact_seconds", "touchdown_seconds",
        "touchdown_pitch_deg", "touchdown_pitch_rate",
        "touchdown_roll_deg", "touchdown_leg_angle_l_deg",
        "touchdown_leg_angle_r_deg", "touchdown_leg_length_l",
        "touchdown_leg_length_r", "airborne_max_pitch_deg",
        "airborne_max_pitch_rate", "airborne_max_leg_error_deg",
        "airborne_peak_wheel_torque", "airborne_peak_joint_torque",
        "airborne_wheel_saturation_ratio",
        "airborne_joint_saturation_ratio", "post_max_pitch_deg",
        "post_max_roll_deg", "post_rebound", "post_other_contact",
        "post_first_other_contact", "post_first_other_contact_seconds",
        "final_pitch_deg", "final_pitch_rate", "final_roll_deg",
        "support_air_l", "support_air_r",
        "support_air_delay_l", "support_air_delay_r",
        "support_ground_l", "support_ground_r",
        "support_touchdown_delay_l", "support_touchdown_delay_r",
    }) {
    const int base_joint = require_id(
        plant_.model(), mjOBJ_JOINT, "base_free_joint");
    base_qpos_ = plant_.model().jnt_qposadr[base_joint];
    base_dof_ = plant_.model().jnt_dofadr[base_joint];
    ground_ = require_id(plant_.model(), mjOBJ_GEOM, "ground");
    wheel_ = {{
        require_id(
            plant_.model(), mjOBJ_GEOM, "Right_wheel_collision"),
        require_id(
            plant_.model(), mjOBJ_GEOM, "Left_wheel_collision"),
    }};

    bc_control_config_t config{};
    bc_control_default_config(&config);
    leg_angle_trim_ = config.lqr_compensation.leg_angle_trim;
}

std::array<double, BC_SIDE_NUM>
DropBenchmark::wheel_ground_clearance() {
    std::array<double, BC_SIDE_NUM> clearance{};
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        clearance[side] = mj_geomDistance(
            &plant_.model(), &plant_.data(), ground_, wheel_[side],
            kDistanceSearchLimit, nullptr);
    }
    return clearance;
}

void DropBenchmark::prepare_release(
    const DropCaseSpec &spec, DropResult &result
) {
    const auto initial_clearance = wheel_ground_clearance();
    const double minimum_clearance = *std::min_element(
        initial_clearance.begin(), initial_clearance.end());
    plant_.data().qpos[base_qpos_ + 2] +=
        spec.wheel_clearance - minimum_clearance;
    plant_.data().qvel[base_dof_ + 4] = spec.initial_pitch_rate;
    mj_forward(&plant_.model(), &plant_.data());
    result.release_clearance = wheel_ground_clearance();
    result.release_time = plant_.data().time;
}

DropResult DropBenchmark::run(const DropCaseSpec &spec) {
    runner_.reset();
    DropResult result{};
    result.spec = spec;
    result.name = drop_case_name(spec);

    bc_operator_command_t command{};
    float held_heading{};
    bool heading_initialized = false;
    while (plant_.data().time < kDisabledSettleSeconds) {
        held_heading = runner_.snapshot().state.value[BC_STATE_PSI];
        runner_.step(
            command,
            held_heading_feedback(runner_.snapshot(), held_heading));
    }
    command.system_enabled = 1U;
    double active_time = std::numeric_limits<double>::quiet_NaN();
    while (plant_.data().time < kEngagementTimeoutSeconds) {
        const auto &snapshot = runner_.snapshot();
        if (snapshot.state_machine.motion == BC_MOTION_ACTIVE) {
            if (!heading_initialized) {
                held_heading = snapshot.state.value[BC_STATE_PSI];
                heading_initialized = true;
            }
        } else {
            held_heading = snapshot.state.value[BC_STATE_PSI];
        }
        command.balance_restart = static_cast<uint8_t>(
            snapshot.state_machine.system == BC_SYSTEM_OFF);
        runner_.step(
            command,
            held_heading_feedback(runner_.snapshot(), held_heading));
        if (runner_.snapshot().state_machine.motion == BC_MOTION_ACTIVE) {
            result.balance_engaged = true;
            if (!std::isfinite(active_time)) active_time = plant_.data().time;
            if (plant_.data().time - active_time >= kStandingSeconds) break;
        }
    }
    if (!result.balance_engaged ||
        plant_.data().time - active_time < kStandingSeconds) {
        return result;
    }

    command.balance_restart = 0U;
    command.forward_velocity = 0.0F;
    prepare_release(spec, result);
    CsvWriter trace(output_directory_ / result.name / "trace.csv", {
        "case", "policy", "phase", "simulation_time",
        "release_elapsed", "touchdown_latched", "wheel_clearance_l",
        "wheel_clearance_r", "base_z", "base_vertical_velocity",
        "pitch", "pitch_rate", "roll", "roll_rate", "theta_l",
        "dtheta_l", "theta_r", "dtheta_r", "ref_theta_l",
        "ref_theta_r", "leg_l_length", "leg_l_length_rate",
        "leg_r_length", "leg_r_length_rate", "raw_wheel_l",
        "raw_wheel_r", "wheel_l", "wheel_r", "raw_joint_l_front",
        "raw_joint_l_rear", "raw_joint_r_front", "raw_joint_r_rear",
        "joint_l_front", "joint_l_rear", "joint_r_front",
        "joint_r_rear", "wheel_rate_l", "wheel_rate_r",
        "support_raw_l", "support_raw_r",
        "support_filtered_l", "support_filtered_r",
        "support_state_l", "support_state_r",
        "support_valid_l", "support_valid_r",
        "contact_wheel_l", "contact_wheel_r",
        "other_contact", "other_contact_kind", "normal_force_l",
        "normal_force_r",
    });

    DropContactLatch touchdown_latch;
    bool was_airborne_after_touchdown = false;
    bc_gimbal_feedback_t gimbal = held_heading_feedback(
        runner_.snapshot(), held_heading);
    runner_.step_with_control_transform(
        command, gimbal,
        [policy = spec.policy](bc_control_command_t &control) {
            apply_drop_air_policy(policy, control);
        });
    const double deadline = result.release_time + kDropTimeoutSeconds;
    while (plant_.data().time <= deadline) {
        const SimulationSample sample = sampler_.read(
            plant_.data(), runner_.snapshot());
        const bool wheel_contact =
            sample.contact.wheel[BC_L] || sample.contact.wheel[BC_R];
        const bool any_contact = wheel_contact || sample.contact.other;
        const bool touchdown_latched_before = touchdown_latch.latched();

        if (any_contact && !result.first_contact) {
            result.first_contact = true;
            result.first_contact_time = sample.time;
            result.first_contact_kind = contact_kind(sample.contact);
        }
        const bool touchdown_latched = touchdown_latch.update(wheel_contact);
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            const auto &support = sample.controller.support_force[side];
            if (!result.support_air_detected[side] && support.valid &&
                support.state == BC_CONTACT_AIR) {
                result.support_air_detected[side] = true;
                result.support_air_time[side] = sample.time;
            }
            if (result.support_air_detected[side] &&
                !result.support_ground_detected[side] && support.valid &&
                support.state == BC_CONTACT_GROUND) {
                result.support_ground_detected[side] = true;
                result.support_ground_time[side] = sample.time;
            }
        }
        if (touchdown_latched && !touchdown_latched_before) {
            result.touchdown = true;
            result.touchdown_time = sample.time;
            result.touchdown_pitch =
                sample.controller.state.value[BC_STATE_THETA_B];
            result.touchdown_pitch_rate =
                sample.controller.state.value[BC_STATE_DTHETA_B];
            result.touchdown_roll = sample.controller.roll;
            result.touchdown_leg_angle = {{
                sample.controller.state.value[BC_STATE_THETA_L],
                sample.controller.state.value[BC_STATE_THETA_R],
            }};
            result.touchdown_leg_length = {{
                sample.controller.leg[BC_L].length,
                sample.controller.leg[BC_R].length,
            }};
        }

        const double pitch = std::abs(static_cast<double>(
            sample.controller.state.value[BC_STATE_THETA_B]));
        const double pitch_rate = std::abs(static_cast<double>(
            sample.controller.state.value[BC_STATE_DTHETA_B]));
        if (!touchdown_latched) {
            result.airborne_diagnostics.observe(sample);
            result.airborne_max_pitch = std::max(
                result.airborne_max_pitch, pitch);
            result.airborne_max_pitch_rate = std::max(
                result.airborne_max_pitch_rate, pitch_rate);
            for (const int state : {BC_STATE_THETA_L, BC_STATE_THETA_R}) {
                const double reference =
                    sample.controller.state_reference.value[state] +
                    leg_angle_trim_;
                result.airborne_max_leg_angle_error = std::max(
                    result.airborne_max_leg_angle_error,
                    std::abs(static_cast<double>(bc_wrap_anglef(
                        static_cast<float>(reference) -
                        sample.controller.state.value[state]))));
            }
        } else {
            result.post_diagnostics.observe(sample);
            result.post_max_pitch = std::max(result.post_max_pitch, pitch);
            result.post_max_roll = std::max(
                result.post_max_roll,
                std::abs(static_cast<double>(sample.controller.roll)));
            result.post_other_contact = result.post_other_contact ||
                sample.contact.other;
            if (sample.contact.other &&
                result.post_first_other_contact == "none") {
                result.post_first_other_contact =
                    sample.contact.unexpected.empty() ?
                        "other" : sample.contact.unexpected;
                result.post_first_other_contact_time = sample.time;
            }
            if (!wheel_contact) was_airborne_after_touchdown = true;
            result.post_rebound = was_airborne_after_touchdown;
        }

        result.finite = result.finite &&
            controller_snapshot_is_finite(sample.controller);
        write_trace(
            trace, result,
            touchdown_latched ? "post_touchdown" : "airborne",
            touchdown_latched, sample);

        if (touchdown_latched &&
            (pitch > BC_PI / 3.0 ||
             std::abs(static_cast<double>(sample.controller.roll)) >
                 BC_PI / 3.0)) {
            result.attitude_diverged = true;
            break;
        }
        if (touchdown_latched &&
            sample.time - result.touchdown_time >= kPostTouchdownSeconds) {
            result.completed = true;
            result.final_pitch =
                sample.controller.state.value[BC_STATE_THETA_B];
            result.final_pitch_rate =
                sample.controller.state.value[BC_STATE_DTHETA_B];
            result.final_roll = sample.controller.roll;
            break;
        }
        if (!result.finite) break;

        if (!touchdown_latched) {
            gimbal = held_heading_feedback(
                runner_.snapshot(), held_heading);
            runner_.step_with_control_transform(
                command, gimbal,
                [policy = spec.policy](bc_control_command_t &control) {
                    apply_drop_air_policy(policy, control);
                });
        } else {
            gimbal = held_heading_feedback(
                runner_.snapshot(), held_heading);
            runner_.step(command, gimbal);
        }
    }
    trace.flush();
    return result;
}

void DropBenchmark::write_summary(const DropResult &result) {
    summary_.begin_row();
    summary_.value(result.name)
        .value(drop_air_policy_name(result.spec.policy))
        .value(result.spec.wheel_clearance)
        .value(result.spec.initial_pitch_rate)
        .value(result.completed)
        .value(result.finite)
        .value(result.attitude_diverged)
        .value(result.balance_engaged)
        .value(result.touchdown)
        .value(result.first_contact_kind)
        .value(result.release_clearance[BC_L])
        .value(result.release_clearance[BC_R])
        .value(result.first_contact_time - result.release_time)
        .value(result.touchdown_time - result.release_time)
        .value(radians_to_degrees(result.touchdown_pitch))
        .value(result.touchdown_pitch_rate)
        .value(radians_to_degrees(result.touchdown_roll))
        .value(radians_to_degrees(result.touchdown_leg_angle[BC_L]))
        .value(radians_to_degrees(result.touchdown_leg_angle[BC_R]))
        .value(result.touchdown_leg_length[BC_L])
        .value(result.touchdown_leg_length[BC_R])
        .value(radians_to_degrees(result.airborne_max_pitch))
        .value(result.airborne_max_pitch_rate)
        .value(radians_to_degrees(result.airborne_max_leg_angle_error))
        .value(result.airborne_diagnostics.maximum_wheel_torque())
        .value(result.airborne_diagnostics.maximum_joint_torque())
        .value(result.airborne_diagnostics.any_wheel_saturation_ratio())
        .value(result.airborne_diagnostics.any_joint_saturation_ratio())
        .value(radians_to_degrees(result.post_max_pitch))
        .value(radians_to_degrees(result.post_max_roll))
        .value(result.post_rebound)
        .value(result.post_other_contact)
        .value(result.post_first_other_contact)
        .value(result.post_first_other_contact_time - result.release_time)
        .value(radians_to_degrees(result.final_pitch))
        .value(result.final_pitch_rate)
        .value(radians_to_degrees(result.final_roll))
        .value(result.support_air_detected[BC_L])
        .value(result.support_air_detected[BC_R])
        .value(result.support_air_time[BC_L] - result.release_time)
        .value(result.support_air_time[BC_R] - result.release_time)
        .value(result.support_ground_detected[BC_L])
        .value(result.support_ground_detected[BC_R])
        .value(result.support_ground_time[BC_L] - result.touchdown_time)
        .value(result.support_ground_time[BC_R] - result.touchdown_time);
    summary_.end_row();
    summary_.flush();
}

void DropBenchmark::write_trace(
    CsvWriter &trace,
    const DropResult &result,
    const char *phase,
    const bool touchdown_latched,
    const SimulationSample &sample
) {
    const auto clearance = wheel_ground_clearance();
    const auto &snapshot = sample.controller;
    trace.begin_row();
    trace.value(result.name)
        .value(drop_air_policy_name(result.spec.policy))
        .value(phase)
        .value(sample.time)
        .value(sample.time - result.release_time)
        .value(touchdown_latched)
        .value(clearance[BC_L])
        .value(clearance[BC_R])
        .value(sample.base.z)
        .value(sample.base.vertical_velocity)
        .value(snapshot.state.value[BC_STATE_THETA_B])
        .value(snapshot.state.value[BC_STATE_DTHETA_B])
        .value(snapshot.roll)
        .value(snapshot.roll_rate)
        .value(snapshot.state.value[BC_STATE_THETA_L])
        .value(snapshot.state.value[BC_STATE_DTHETA_L])
        .value(snapshot.state.value[BC_STATE_THETA_R])
        .value(snapshot.state.value[BC_STATE_DTHETA_R])
        .value(snapshot.state_reference.value[BC_STATE_THETA_L])
        .value(snapshot.state_reference.value[BC_STATE_THETA_R])
        .value(snapshot.leg[BC_L].length)
        .value(snapshot.leg[BC_L].length_velocity)
        .value(snapshot.leg[BC_R].length)
        .value(snapshot.leg[BC_R].length_velocity)
        .value(snapshot.actuation_request.wheel_torque[BC_L])
        .value(snapshot.actuation_request.wheel_torque[BC_R])
        .value(snapshot.actuation.wheel_torque[BC_L])
        .value(snapshot.actuation.wheel_torque[BC_R]);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            trace.value(
                snapshot.actuation_request.leg[side].joint_torque[joint]);
        }
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            trace.value(snapshot.actuation.leg[side].joint_torque[joint]);
        }
    }
    trace.value(sample.wheel.angular_velocity[BC_L])
        .value(sample.wheel.angular_velocity[BC_R]);
    trace.value(snapshot.support_force[BC_L].vertical_force)
        .value(snapshot.support_force[BC_R].vertical_force)
        .value(snapshot.support_force[BC_L].filtered_vertical_force)
        .value(snapshot.support_force[BC_R].filtered_vertical_force)
        .value(bc_contact_state_name(snapshot.support_force[BC_L].state))
        .value(bc_contact_state_name(snapshot.support_force[BC_R].state))
        .value(snapshot.support_force[BC_L].valid)
        .value(snapshot.support_force[BC_R].valid);
    trace.value(sample.contact.wheel[BC_L])
        .value(sample.contact.wheel[BC_R])
        .value(sample.contact.other)
        .value(sample.contact.unexpected)
        .value(sample.contact.wheel_normal_force[BC_L])
        .value(sample.contact.wheel_normal_force[BC_R]);
    trace.end_row();
}

} // namespace balance::benchmark
