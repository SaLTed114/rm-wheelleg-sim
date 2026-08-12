#include "platform_drop.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "balance/math_utils.h"
#include "common/common_diagnostics.hpp"

namespace balance::benchmark {
namespace {

constexpr double kDisabledSettleSeconds = 2.0;
constexpr double kStandingSeconds = 1.0;
constexpr double kEngagementTimeoutSeconds = 10.0;
constexpr double kSpeedStableSeconds = 0.25;
constexpr double kSpeedTolerance = 0.1;
constexpr double kPlatformEdgeX = 2.0;
constexpr double kScenarioTimeoutSeconds = 16.0;
constexpr double kPostTouchdownSeconds = 1.5;
constexpr double kKfRecoveredTolerance = 0.1;
constexpr double kRecoveryAttitudeLimit = 15.0 * BC_PI / 180.0;
constexpr double kDivergenceAttitude = 60.0 * BC_PI / 180.0;
constexpr double kTimestepSeconds = 0.001;

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

const char *platform_name(const double height) {
    return height < 0.3 ?
        "drop_platform_200mm" : "drop_platform_400mm";
}

} // namespace

std::string platform_drop_case_name(const PlatformDropSpec &spec) {
    std::ostringstream name;
    name << "platform_drop_" << std::llround(spec.height * 1000.0)
         << "mm_l" << std::fixed << std::setprecision(2)
         << spec.leg_length << "_v" << std::setprecision(1)
         << spec.target_velocity << '_'
         << drop_air_policy_name(spec.policy);
    if (spec.airborne_leg_length > 0.0) {
        name << "_air_extend_l" << std::setprecision(2)
             << spec.airborne_leg_length;
    }
    if (spec.airborne_pitch_rate != 0.0) {
        name << "_pitch_rate_"
             << (spec.airborne_pitch_rate > 0.0 ? "pos_" : "neg_")
             << std::setprecision(1) << std::abs(spec.airborne_pitch_rate);
    }
    std::string result = name.str();
    std::replace(result.begin(), result.end(), '.', 'p');
    return result;
}

const std::array<PlatformDropSpec, 36> &platform_drop_cases() {
    static const std::array<PlatformDropSpec, 36> cases{{
        {0.2, 0.5, 0.18, 0.0, DropAirPolicy::length_only},
        {0.2, 0.5, 0.18, 0.0, DropAirPolicy::leg_lqr},
        {0.2, 1.0, 0.18, 0.0, DropAirPolicy::length_only},
        {0.2, 1.0, 0.18, 0.0, DropAirPolicy::leg_lqr},
        {0.2, 2.0, 0.18, 0.0, DropAirPolicy::length_only},
        {0.2, 2.0, 0.18, 0.0, DropAirPolicy::leg_lqr},
        {0.4, 0.5, 0.18, 0.0, DropAirPolicy::length_only},
        {0.4, 0.5, 0.18, 0.0, DropAirPolicy::leg_lqr},
        {0.4, 1.0, 0.18, 0.0, DropAirPolicy::length_only},
        {0.4, 1.0, 0.18, 0.0, DropAirPolicy::leg_lqr},
        {0.4, 2.0, 0.18, 0.0, DropAirPolicy::length_only},
        {0.4, 2.0, 0.18, 0.0, DropAirPolicy::leg_lqr},
        {0.2, 0.5, 0.24, 0.0, DropAirPolicy::length_only},
        {0.2, 0.5, 0.24, 0.0, DropAirPolicy::leg_lqr},
        {0.2, 1.0, 0.24, 0.0, DropAirPolicy::length_only},
        {0.2, 1.0, 0.24, 0.0, DropAirPolicy::leg_lqr},
        {0.2, 2.0, 0.24, 0.0, DropAirPolicy::length_only},
        {0.2, 2.0, 0.24, 0.0, DropAirPolicy::leg_lqr},
        {0.4, 0.5, 0.24, 0.0, DropAirPolicy::length_only},
        {0.4, 0.5, 0.24, 0.0, DropAirPolicy::leg_lqr},
        {0.4, 1.0, 0.24, 0.0, DropAirPolicy::length_only},
        {0.4, 1.0, 0.24, 0.0, DropAirPolicy::leg_lqr},
        {0.4, 2.0, 0.24, 0.0, DropAirPolicy::length_only},
        {0.4, 2.0, 0.24, 0.0, DropAirPolicy::leg_lqr},
        {0.2, 1.5, 0.18, 0.0, DropAirPolicy::length_only},
        {0.2, 2.5, 0.18, 0.0, DropAirPolicy::length_only},
        {0.2, 1.5, 0.18, 0.38, DropAirPolicy::length_only},
        {0.2, 2.0, 0.18, 0.38, DropAirPolicy::length_only},
        {0.2, 2.5, 0.18, 0.38, DropAirPolicy::length_only},
        {0.2, 1.5, 0.18, 0.38, DropAirPolicy::leg_lqr},
        {0.2, 2.0, 0.18, 0.38, DropAirPolicy::leg_lqr},
        {0.2, 2.5, 0.18, 0.38, DropAirPolicy::leg_lqr},
        {0.2, 2.0, 0.18, 0.38, DropAirPolicy::length_only, -0.5},
        {0.2, 2.0, 0.18, 0.38, DropAirPolicy::length_only, +0.5},
        {0.2, 2.0, 0.18, 0.38, DropAirPolicy::leg_lqr, -0.5},
        {0.2, 2.0, 0.18, 0.38, DropAirPolicy::leg_lqr, +0.5},
    }};
    return cases;
}

const PlatformDropSpec *find_platform_drop_case(
    const std::string_view name
) noexcept {
    const auto &cases = platform_drop_cases();
    const auto found = std::find_if(
        cases.begin(), cases.end(),
        [name](const PlatformDropSpec &spec) {
            return platform_drop_case_name(spec) == name;
        });
    return found == cases.end() ? nullptr : &*found;
}

PlatformDropScenario::PlatformDropScenario(
    const PlatformDropSpec &spec, const mjModel &model
) : spec_(spec), name_(platform_drop_case_name(spec)) {
    const int base_joint = require_id(
        model, mjOBJ_JOINT, "base_free_joint");
    base_qpos_ = model.jnt_qposadr[base_joint];
    base_dof_ = model.jnt_dofadr[base_joint];
    wheel_axis_ = {{
        require_id(model, mjOBJ_SITE, "Right_wheel_axis_site"),
        require_id(model, mjOBJ_SITE, "Left_wheel_axis_site"),
    }};
}

void PlatformDropScenario::reset(sim::MujocoPlant &plant) {
    plant.place_mocap_surface(
        "drop_platform_200mm", 0.5, 0.0, -2.0, false);
    plant.place_mocap_surface(
        "drop_platform_400mm", 0.5, 0.0, -2.0, false);
    plant.place_mocap_surface(
        platform_name(spec_.height), 0.5, 0.0,
        -0.43 + 0.5 * spec_.height, true);
    plant.data().qpos[base_qpos_ + 2] += spec_.height;
    mj_forward(&plant.model(), &plant.data());
    const double initial_axle_x = 0.5 * (
        plant.data().site_xpos[3 * wheel_axis_[BC_L]] +
        plant.data().site_xpos[3 * wheel_axis_[BC_R]]);
    plant.data().qpos[base_qpos_] -= initial_axle_x;
    mj_forward(&plant.model(), &plant.data());

    phase_ = PlatformDropPhase::disabled_settle;
    command_ = {};
    settle_start_time_ = -1.0;
    active_start_time_ = -1.0;
    stable_hold_start_time_ = -1.0;
    stable_time_ = 0.0;
    departure_time_ = 0.0;
    touchdown_time_ = 0.0;
    edge_velocity_ = 0.0;
    edge_leg_length_ = {};
    edge_crossed_ = false;
    balance_engaged_ = false;
    speed_stable_ = false;
    platform_contact_seen_ = false;
    left_platform_ = false;
    touchdown_ = false;
    heading_initialized_ = false;
    held_heading_ = 0.0F;
    issue_ = "none";
}

const char *PlatformDropScenario::phase_name() const noexcept {
    switch (phase_) {
    case PlatformDropPhase::disabled_settle:
        return "platform_drop_disabled_settle";
    case PlatformDropPhase::standing: return "platform_drop_standing";
    case PlatformDropPhase::accelerating:
        return "platform_drop_accelerating";
    case PlatformDropPhase::approaching_edge:
        return "platform_drop_approaching_edge";
    case PlatformDropPhase::airborne: return "platform_drop_airborne";
    case PlatformDropPhase::post_touchdown:
        return "platform_drop_post_touchdown";
    case PlatformDropPhase::complete: return "platform_drop_complete";
    case PlatformDropPhase::failed: return "platform_drop_failed";
    }
    return "platform_drop_unknown";
}

double PlatformDropScenario::axle_velocity(
    const SimulationSample &sample
) const noexcept {
    return 0.5 * (
        sample.wheel.forward_velocity[BC_L] +
        sample.wheel.forward_velocity[BC_R]);
}

void PlatformDropScenario::fail(const char *issue) noexcept {
    issue_ = issue;
    phase_ = PlatformDropPhase::failed;
}

void PlatformDropScenario::step(
    sim::MujocoPlant &plant,
    sim::SimulationRunner &runner,
    const SimulationSampler &sampler
) {
    if (finished()) return;
    if (settle_start_time_ < 0.0) settle_start_time_ = plant.data().time;
    if (plant.data().time - settle_start_time_ > kScenarioTimeoutSeconds) {
        fail("platform_drop_timeout");
        return;
    }

    const SimulationSample sample = sampler.read(
        plant.data(), runner.snapshot());
    const bool on_platform =
        sample.contact.wheel_on_platform[BC_L] ||
        sample.contact.wheel_on_platform[BC_R];
    const bool on_lower_ground =
        sample.contact.wheel_on_lower_ground[BC_L] ||
        sample.contact.wheel_on_lower_ground[BC_R];
    platform_contact_seen_ = platform_contact_seen_ || on_platform;

    command_.system_enabled = static_cast<uint8_t>(
        phase_ != PlatformDropPhase::disabled_settle);
    command_.balance_restart = static_cast<uint8_t>(
        command_.system_enabled &&
        runner.snapshot().state_machine.system == BC_SYSTEM_OFF);
    command_.forward_velocity = static_cast<float>(
        phase_ == PlatformDropPhase::disabled_settle ||
        phase_ == PlatformDropPhase::standing ?
            0.0 : spec_.target_velocity);

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

    if (phase_ == PlatformDropPhase::disabled_settle) {
        runner.step(command_, gimbal);
        if (plant.data().time - settle_start_time_ >=
            kDisabledSettleSeconds) {
            phase_ = PlatformDropPhase::standing;
        }
        return;
    }

    if (phase_ == PlatformDropPhase::standing) {
        runner.step(command_, gimbal);
        if (runner.snapshot().state_machine.motion == BC_MOTION_ACTIVE) {
            balance_engaged_ = true;
            if (active_start_time_ < 0.0) active_start_time_ = plant.data().time;
            if (plant.data().time - active_start_time_ >= kStandingSeconds) {
                phase_ = PlatformDropPhase::accelerating;
            }
        }
        if (!balance_engaged_ &&
            plant.data().time - settle_start_time_ >=
                kEngagementTimeoutSeconds) {
            fail("balance_not_engaged");
        }
        return;
    }

    if (phase_ == PlatformDropPhase::accelerating) {
        const double velocity_error = std::abs(
            axle_velocity(sample) - spec_.target_velocity);
        if (velocity_error <= kSpeedTolerance) {
            if (stable_hold_start_time_ < 0.0) {
                stable_hold_start_time_ = sample.time;
            }
            if (sample.time - stable_hold_start_time_ >=
                kSpeedStableSeconds) {
                speed_stable_ = true;
                stable_time_ = sample.time;
                phase_ = PlatformDropPhase::approaching_edge;
            }
        } else {
            stable_hold_start_time_ = -1.0;
        }
        if (!speed_stable_ && sample.axle.x >= kPlatformEdgeX) {
            fail("speed_not_stable_before_edge");
            return;
        }
    }

    if (phase_ == PlatformDropPhase::approaching_edge &&
        !edge_crossed_ && sample.axle.x >= kPlatformEdgeX) {
        edge_crossed_ = true;
        edge_velocity_ = axle_velocity(sample);
        edge_leg_length_ = {{
            sample.controller.leg[BC_L].length,
            sample.controller.leg[BC_R].length,
        }};
    }

    if ((phase_ == PlatformDropPhase::accelerating ||
         phase_ == PlatformDropPhase::approaching_edge) &&
        platform_contact_seen_ && !on_platform && !on_lower_ground) {
        left_platform_ = true;
        departure_time_ = sample.time;
        phase_ = PlatformDropPhase::airborne;
        if (spec_.airborne_pitch_rate != 0.0) {
            plant.data().qvel[base_dof_ + 4] =
                spec_.airborne_pitch_rate;
            mj_forward(&plant.model(), &plant.data());
        }
    }

    if (phase_ == PlatformDropPhase::airborne &&
        sample.contact.lower_ground) {
        touchdown_ = true;
        touchdown_time_ = sample.time;
        phase_ = PlatformDropPhase::post_touchdown;
    }

    if (phase_ == PlatformDropPhase::post_touchdown &&
        sample.time - touchdown_time_ >= kPostTouchdownSeconds) {
        phase_ = PlatformDropPhase::complete;
        return;
    }

    if (phase_ == PlatformDropPhase::airborne) {
        runner.step_with_control_transform(
            command_, gimbal,
            [policy = spec_.policy,
             airborne_leg_length = spec_.airborne_leg_length](
                bc_control_command_t &control
            ) {
                apply_drop_air_policy(policy, control);
                if (airborne_leg_length > 0.0) {
                    for (int side = 0; side < BC_SIDE_NUM; ++side) {
                        control.leg[side].target.length =
                            static_cast<float>(airborne_leg_length);
                    }
                }
            });
    } else if (phase_ == PlatformDropPhase::post_touchdown &&
               spec_.airborne_leg_length > 0.0) {
        runner.step_with_control_transform(
            command_, gimbal,
            [airborne_leg_length = spec_.airborne_leg_length](
                bc_control_command_t &control
            ) {
                for (int side = 0; side < BC_SIDE_NUM; ++side) {
                    control.leg[side].target.length =
                        static_cast<float>(airborne_leg_length);
                }
            });
    } else {
        runner.step(command_, gimbal);
    }
}

PlatformDropBenchmark::PlatformDropBenchmark(
    const std::filesystem::path &model_path,
    const std::filesystem::path &output_directory
) : output_directory_(output_directory),
    plant_(model_path, kTimestepSeconds),
    adapter_(plant_.model()),
    sampler_(plant_.model()),
    summary_(output_directory / "platform_summary.csv", {
        "case", "height", "target_velocity", "leg_length",
        "airborne_leg_length", "policy", "airborne_pitch_rate",
        "completed",
        "finite", "balance_engaged", "speed_stable", "left_platform",
        "touchdown", "recovered", "diverged", "issue",
        "edge_velocity", "edge_leg_length_l", "edge_leg_length_r",
        "departure_time", "flight_time",
        "departure_base_z", "touchdown_base_z", "base_z_drop",
        "touchdown_leg_length_l", "touchdown_leg_length_r",
        "touchdown_leg_world_angle_l_deg",
        "touchdown_leg_world_angle_r_deg",
        "touchdown_leg_world_rate_l", "touchdown_leg_world_rate_r",
        "touchdown_leg_angle_difference_deg",
        "airborne_max_leg_angle_error_l_deg",
        "airborne_max_leg_angle_error_r_deg",
        "touchdown_pitch_deg", "touchdown_pitch_rate",
        "touchdown_roll_deg", "maximum_pitch_deg", "maximum_roll_deg",
        "airborne_maximum_pitch_deg",
        "airborne_max_joint_torque_request",
        "airborne_max_joint_torque", "airborne_joint_saturation_ratio",
        "touchdown_vertical_velocity", "post_max_support_force",
        "wheel_touchdown_delay_l", "wheel_touchdown_delay_r",
        "wheel_touchdown_time_difference",
        "airborne_max_velocity_error", "touchdown_velocity_error",
        "kf_recovery_seconds", "support_air_delay_l",
        "support_air_delay_r", "support_ground_delay_l",
        "support_ground_delay_r", "other_contact",
        "first_other_contact",
    }) {
    bc_control_config_t config{};
    bc_control_default_config(&config);
    leg_angle_trim_ = config.lqr_compensation.leg_angle_trim;
}

PlatformDropResult PlatformDropBenchmark::run(
    const PlatformDropSpec &spec
) {
    bc_controller_config_t controller_config{};
    bc_controller_default_config(&controller_config);
    controller_config.motion.leg_length =
        static_cast<float>(spec.leg_length);
    sim::SimulationRunner runner(plant_, adapter_, controller_config);
    runner.reset();
    PlatformDropScenario scenario(spec, plant_.model());
    scenario.reset(plant_);

    PlatformDropResult result{};
    result.spec = spec;
    result.name = scenario.name();
    CsvWriter trace(output_directory_ / result.name / "trace.csv", {
        "case", "phase", "simulation_time", "target_velocity",
        "axle_x", "base_z", "leg_length_l", "leg_length_r",
        "truth_axle_velocity",
        "truth_base_velocity", "estimated_axle_velocity",
        "wheel_odometry_velocity", "kf_velocity_x", "kf_variance_x",
        "kf_prior_velocity_x", "kf_wheel_measurement", "kf_innovation",
        "kf_innovation_variance", "kf_nis", "kf_measurement_accepted",
        "kf_wheel_velocity_reliable", "pitch", "pitch_rate", "roll",
        "roll_rate", "leg_world_angle_l", "leg_world_angle_r",
        "leg_world_rate_l", "leg_world_rate_r",
        "wheel_rate_l", "wheel_rate_r",
        "wheel_torque_l", "wheel_torque_r", "support_filtered_l",
        "support_filtered_r", "support_state_l", "support_state_r",
        "raw_joint_l_front", "raw_joint_l_rear",
        "raw_joint_r_front", "raw_joint_r_rear",
        "joint_l_front", "joint_l_rear",
        "joint_r_front", "joint_r_rear",
        "contact_platform_l", "contact_platform_r",
        "contact_ground_l", "contact_ground_r", "other_contact",
        "other_contact_kind",
    });

    bool departure_captured = false;
    bool touchdown_captured = false;
    bool kf_recovered = false;
    std::size_t airborne_samples = 0U;
    std::size_t airborne_saturated_samples = 0U;
    while (!scenario.finished()) {
        scenario.step(plant_, runner, sampler_);
        const SimulationSample sample = sampler_.read(
            plant_.data(), runner.snapshot());
        write_trace(trace, scenario, sample);

        result.finite = result.finite &&
            controller_snapshot_is_finite(sample.controller) &&
            std::isfinite(sample.base.forward_velocity) &&
            std::isfinite(sample.base.z);
        const double pitch = std::abs(static_cast<double>(
            sample.controller.state.value[BC_STATE_THETA_B]));
        const double roll = std::abs(static_cast<double>(
            sample.controller.roll));
        result.maximum_pitch = std::max(result.maximum_pitch, pitch);
        result.maximum_roll = std::max(result.maximum_roll, roll);
        result.diverged = result.diverged ||
            pitch > kDivergenceAttitude || roll > kDivergenceAttitude;
        if (scenario.left_platform() && sample.contact.other) {
            result.other_contact = true;
            if (result.first_other_contact == "none") {
                result.first_other_contact = sample.contact.unexpected.empty() ?
                    "other" : sample.contact.unexpected;
            }
        }

        const double truth_velocity = 0.5 * (
            sample.wheel.forward_velocity[BC_L] +
            sample.wheel.forward_velocity[BC_R]);
        const double velocity_error = std::abs(
            sample.controller.forward_velocity.estimated_axle -
            truth_velocity);
        if (scenario.left_platform() && !scenario.touchdown()) {
            ++airborne_samples;
            bool saturated = false;
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                    const double requested = std::abs(static_cast<double>(
                        sample.controller.actuation_request.leg[side].
                            joint_torque[joint]));
                    const double actual = std::abs(static_cast<double>(
                        sample.controller.actuation.leg[side].
                            joint_torque[joint]));
                    result.airborne_maximum_joint_torque_request = std::max(
                        result.airborne_maximum_joint_torque_request,
                        requested);
                    result.airborne_maximum_joint_torque = std::max(
                        result.airborne_maximum_joint_torque, actual);
                    saturated = saturated ||
                        std::abs(requested - actual) > 1.0e-4;
                }
            }
            if (saturated) ++airborne_saturated_samples;
            result.airborne_maximum_pitch = std::max(
                result.airborne_maximum_pitch, pitch);
            const int angle_state[BC_SIDE_NUM] = {
                BC_STATE_THETA_L, BC_STATE_THETA_R,
            };
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                result.airborne_maximum_leg_angle_error[side] = std::max(
                    result.airborne_maximum_leg_angle_error[side],
                    std::abs(static_cast<double>(bc_wrap_anglef(
                        static_cast<float>(leg_angle_trim_) -
                        sample.controller.state.value[angle_state[side]]))));
            }
            result.airborne_maximum_velocity_error = std::max(
                result.airborne_maximum_velocity_error, velocity_error);
        }
        if (scenario.left_platform() && !departure_captured) {
            departure_captured = true;
            result.departure_base_z = sample.base.z;
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                result.support_air_time[side] =
                    std::numeric_limits<double>::quiet_NaN();
            }
        }
        if (departure_captured) {
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                if (!std::isfinite(result.wheel_touchdown_time[side]) &&
                    sample.contact.wheel_on_lower_ground[side]) {
                    result.wheel_touchdown_time[side] = sample.time;
                }
                const auto &support = sample.controller.support_force[side];
                if (!std::isfinite(result.support_air_time[side]) &&
                    support.valid && support.state == BC_CONTACT_AIR) {
                    result.support_air_time[side] = sample.time;
                }
                if (scenario.touchdown() &&
                    !std::isfinite(result.support_ground_time[side]) &&
                    support.valid && support.state == BC_CONTACT_GROUND) {
                    result.support_ground_time[side] = sample.time;
                }
            }
        }
        if (scenario.touchdown() && !touchdown_captured) {
            touchdown_captured = true;
            result.touchdown_base_z = sample.base.z;
            result.touchdown_vertical_velocity = sample.base.vertical_velocity;
            result.touchdown_leg_length = {{
                sample.controller.leg[BC_L].length,
                sample.controller.leg[BC_R].length,
            }};
            result.touchdown_leg_world_angle = {{
                sample.controller.state.value[BC_STATE_THETA_L],
                sample.controller.state.value[BC_STATE_THETA_R],
            }};
            result.touchdown_leg_world_angle_rate = {{
                sample.controller.state.value[BC_STATE_DTHETA_L],
                sample.controller.state.value[BC_STATE_DTHETA_R],
            }};
            result.touchdown_leg_angle_difference = std::abs(
                static_cast<double>(bc_wrap_anglef(
                    static_cast<float>(
                        result.touchdown_leg_world_angle[BC_L] -
                        result.touchdown_leg_world_angle[BC_R]))));
            result.touchdown_pitch =
                sample.controller.state.value[BC_STATE_THETA_B];
            result.touchdown_pitch_rate =
                sample.controller.state.value[BC_STATE_DTHETA_B];
            result.touchdown_roll = sample.controller.roll;
            result.touchdown_velocity_error = velocity_error;
        }
        if (touchdown_captured) {
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                result.post_touchdown_maximum_support_force = std::max(
                    result.post_touchdown_maximum_support_force,
                    std::abs(static_cast<double>(
                        sample.controller.support_force[side].
                            filtered_vertical_force)));
            }
        }
        if (touchdown_captured && !kf_recovered &&
            velocity_error <= kKfRecoveredTolerance) {
            kf_recovered = true;
            result.kf_recovery_time =
                sample.time - scenario.touchdown_time();
        }
        if (!result.finite) break;
    }
    trace.flush();

    result.completed = scenario.phase() == PlatformDropPhase::complete;
    result.balance_engaged = scenario.balance_engaged();
    result.speed_stable = scenario.speed_stable();
    result.left_platform = scenario.left_platform();
    result.touchdown = scenario.touchdown();
    result.issue = scenario.issue();
    result.stable_time = scenario.stable_time();
    result.departure_time = scenario.departure_time();
    result.touchdown_time = scenario.touchdown_time();
    result.edge_velocity = scenario.edge_velocity();
    result.edge_leg_length = scenario.edge_leg_length();
    result.airborne_joint_saturation_ratio =
        airborne_samples == 0U ? 0.0 :
        static_cast<double>(airborne_saturated_samples) /
            static_cast<double>(airborne_samples);
    if (std::isfinite(result.wheel_touchdown_time[BC_L]) &&
        std::isfinite(result.wheel_touchdown_time[BC_R])) {
        result.wheel_touchdown_time_difference = std::abs(
            result.wheel_touchdown_time[BC_L] -
            result.wheel_touchdown_time[BC_R]);
    }
    const auto &final = runner.snapshot();
    result.recovered = result.completed && !result.diverged &&
        std::abs(static_cast<double>(
            final.state.value[BC_STATE_THETA_B])) <=
            kRecoveryAttitudeLimit &&
        std::abs(static_cast<double>(final.roll)) <=
            kRecoveryAttitudeLimit &&
        final.state_machine.motion == BC_MOTION_ACTIVE;
    return result;
}

void PlatformDropBenchmark::write_summary(
    const PlatformDropResult &result
) {
    const auto degrees = [](const double radians) {
        return radians * 180.0 / BC_PI;
    };
    summary_.begin_row();
    summary_.value(result.name)
        .value(result.spec.height)
        .value(result.spec.target_velocity)
        .value(result.spec.leg_length)
        .value(result.spec.airborne_leg_length)
        .value(drop_air_policy_name(result.spec.policy))
        .value(result.spec.airborne_pitch_rate)
        .value(result.completed)
        .value(result.finite)
        .value(result.balance_engaged)
        .value(result.speed_stable)
        .value(result.left_platform)
        .value(result.touchdown)
        .value(result.recovered)
        .value(result.diverged)
        .value(result.issue)
        .value(result.edge_velocity)
        .value(result.edge_leg_length[BC_L])
        .value(result.edge_leg_length[BC_R])
        .value(result.departure_time)
        .value(result.touchdown_time - result.departure_time)
        .value(result.departure_base_z)
        .value(result.touchdown_base_z)
        .value(result.departure_base_z - result.touchdown_base_z)
        .value(result.touchdown_leg_length[BC_L])
        .value(result.touchdown_leg_length[BC_R])
        .value(degrees(result.touchdown_leg_world_angle[BC_L]))
        .value(degrees(result.touchdown_leg_world_angle[BC_R]))
        .value(result.touchdown_leg_world_angle_rate[BC_L])
        .value(result.touchdown_leg_world_angle_rate[BC_R])
        .value(degrees(result.touchdown_leg_angle_difference))
        .value(degrees(result.airborne_maximum_leg_angle_error[BC_L]))
        .value(degrees(result.airborne_maximum_leg_angle_error[BC_R]))
        .value(degrees(result.touchdown_pitch))
        .value(result.touchdown_pitch_rate)
        .value(degrees(result.touchdown_roll))
        .value(degrees(result.maximum_pitch))
        .value(degrees(result.maximum_roll))
        .value(degrees(result.airborne_maximum_pitch))
        .value(result.airborne_maximum_joint_torque_request)
        .value(result.airborne_maximum_joint_torque)
        .value(result.airborne_joint_saturation_ratio)
        .value(result.touchdown_vertical_velocity)
        .value(result.post_touchdown_maximum_support_force)
        .value(result.wheel_touchdown_time[BC_L] - result.departure_time)
        .value(result.wheel_touchdown_time[BC_R] - result.departure_time)
        .value(result.wheel_touchdown_time_difference)
        .value(result.airborne_maximum_velocity_error)
        .value(result.touchdown_velocity_error)
        .value(result.kf_recovery_time)
        .value(result.support_air_time[BC_L] - result.departure_time)
        .value(result.support_air_time[BC_R] - result.departure_time)
        .value(result.support_ground_time[BC_L] - result.touchdown_time)
        .value(result.support_ground_time[BC_R] - result.touchdown_time)
        .value(result.other_contact)
        .value(result.first_other_contact);
    summary_.end_row();
    summary_.flush();
}

void PlatformDropBenchmark::write_trace(
    CsvWriter &trace,
    const PlatformDropScenario &scenario,
    const SimulationSample &sample
) {
    const auto &snapshot = sample.controller;
    const auto &kf = snapshot.velocity_estimator;
    const double truth_axle_velocity = 0.5 * (
        sample.wheel.forward_velocity[BC_L] +
        sample.wheel.forward_velocity[BC_R]);
    trace.begin_row();
    trace.value(scenario.name())
        .value(scenario.phase_name())
        .value(sample.time)
        .value(scenario.spec().target_velocity)
        .value(sample.axle.x)
        .value(sample.base.z)
        .value(snapshot.leg[BC_L].length)
        .value(snapshot.leg[BC_R].length)
        .value(truth_axle_velocity)
        .value(sample.base.forward_velocity)
        .value(snapshot.forward_velocity.estimated_axle)
        .value(snapshot.forward_velocity.wheel_odometry)
        .value(kf.velocity_x)
        .value(kf.velocity_variance_x)
        .value(kf.prior_velocity_x)
        .value(kf.wheel_velocity_measurement)
        .value(kf.innovation)
        .value(kf.innovation_variance)
        .value(kf.nis)
        .value(static_cast<int>(kf.measurement_accepted))
        .value(static_cast<int>(kf.wheel_velocity_reliable))
        .value(snapshot.state.value[BC_STATE_THETA_B])
        .value(snapshot.state.value[BC_STATE_DTHETA_B])
        .value(snapshot.roll)
        .value(snapshot.roll_rate)
        .value(snapshot.state.value[BC_STATE_THETA_L])
        .value(snapshot.state.value[BC_STATE_THETA_R])
        .value(snapshot.state.value[BC_STATE_DTHETA_L])
        .value(snapshot.state.value[BC_STATE_DTHETA_R])
        .value(sample.wheel.angular_velocity[BC_L])
        .value(sample.wheel.angular_velocity[BC_R])
        .value(snapshot.actuation.wheel_torque[BC_L])
        .value(snapshot.actuation.wheel_torque[BC_R])
        .value(snapshot.support_force[BC_L].filtered_vertical_force)
        .value(snapshot.support_force[BC_R].filtered_vertical_force)
        .value(bc_contact_state_name(snapshot.support_force[BC_L].state))
        .value(bc_contact_state_name(snapshot.support_force[BC_R].state))
        .value(snapshot.actuation_request.leg[BC_L].joint_torque[BC_FRONT])
        .value(snapshot.actuation_request.leg[BC_L].joint_torque[BC_REAR])
        .value(snapshot.actuation_request.leg[BC_R].joint_torque[BC_FRONT])
        .value(snapshot.actuation_request.leg[BC_R].joint_torque[BC_REAR])
        .value(snapshot.actuation.leg[BC_L].joint_torque[BC_FRONT])
        .value(snapshot.actuation.leg[BC_L].joint_torque[BC_REAR])
        .value(snapshot.actuation.leg[BC_R].joint_torque[BC_FRONT])
        .value(snapshot.actuation.leg[BC_R].joint_torque[BC_REAR])
        .value(sample.contact.wheel_on_platform[BC_L])
        .value(sample.contact.wheel_on_platform[BC_R])
        .value(sample.contact.wheel_on_lower_ground[BC_L])
        .value(sample.contact.wheel_on_lower_ground[BC_R])
        .value(sample.contact.other)
        .value(sample.contact.unexpected);
    trace.end_row();
}

} // namespace balance::benchmark
