#include "jump_impulse.hpp"

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
constexpr double kScenarioTimeoutSeconds = 10.0;
constexpr double kReadyHoldSeconds = 0.25;
constexpr double kRecoveryHoldSeconds = 0.50;
constexpr double kTakeoffConfirmSeconds = 0.002;
constexpr double kSupportWaitSeconds = 0.10;
constexpr double kMinimumMaximumThrustSeconds = 0.150;
constexpr double kForceRiseRate = 3000.0;
constexpr double kForceReleaseRate = 10000.0;
constexpr double kMaximumAxialForce = 240.0;
constexpr double kReleaseLegLength = 0.34;
constexpr double kVelocityTolerance = 0.05;
constexpr double kLegLengthTolerance = 0.012;
constexpr double kLegSpeedTolerance = 0.05;
constexpr double kPitchTolerance = 3.0 * BC_PI / 180.0;
constexpr double kPitchRateTolerance = 0.15;
constexpr double kRollTolerance = 3.0 * BC_PI / 180.0;
constexpr double kRollRateTolerance = 0.15;
constexpr double kYawTolerance = 1.0 * BC_PI / 180.0;
constexpr double kYawRateTolerance = 0.10;
constexpr double kDivergenceAngle = 60.0 * BC_PI / 180.0;
constexpr double kJointTorqueLimit = 40.0;
constexpr double kSaturationTolerance = 1.0e-4;

int require_id(const mjModel &model, const mjtObj type, const char *name) {
    const int id = mj_name2id(&model, type, name);
    if (id < 0) {
        throw std::runtime_error(
            "missing MuJoCo object '" + std::string(name) + "'");
    }
    return id;
}

std::string force_token(const double force) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(0) << force;
    return stream.str();
}

double move_toward(
    const double value,
    const double target,
    const double maximum_step
) {
    return value + std::clamp(
        target - value, -maximum_step, maximum_step);
}

} // namespace

std::string jump_impulse_case_name(const JumpImpulseSpec &spec) {
    std::string name = "jump_impulse_f" + force_token(spec.peak_force);
    if (spec.peak_force == 240.0 ||
        std::abs(spec.hold_seconds - 0.060) > 1.0e-9) {
        name += "_t" + force_token(spec.hold_seconds * 1000.0) + "ms";
    }
    return name;
}

const std::array<JumpImpulseSpec, 6> &jump_impulse_cases() {
    static const std::array<JumpImpulseSpec, 6> cases{{
        {140.0, 0.18, 0.060},
        {180.0, 0.18, 0.060},
        {220.0, 0.18, 0.060},
        {240.0, 0.18, 0.060},
        {240.0, 0.18, 0.090},
        {240.0, 0.18, 0.120},
    }};
    return cases;
}

const JumpImpulseSpec *find_jump_impulse_case(
    const std::string_view name
) noexcept {
    const auto &cases = jump_impulse_cases();
    const auto found = std::find_if(
        cases.begin(), cases.end(), [name](const JumpImpulseSpec &spec) {
            return jump_impulse_case_name(spec) == name;
        });
    return found == cases.end() ? nullptr : &*found;
}

JumpImpulseScenario::JumpImpulseScenario(
    const JumpImpulseSpec &spec,
    const mjModel &model
) : spec_(spec), name_(jump_impulse_case_name(spec)), model_(&model) {
    ground_ = require_id(model, mjOBJ_GEOM, "ground");
    base_body_ = require_id(model, mjOBJ_BODY, "base_link");
    wheel_ = {{
        require_id(model, mjOBJ_GEOM, "Right_wheel_collision"),
        require_id(model, mjOBJ_GEOM, "Left_wheel_collision"),
    }};
    wheel_axis_ = {{
        require_id(model, mjOBJ_SITE, "Right_wheel_axis_site"),
        require_id(model, mjOBJ_SITE, "Left_wheel_axis_site"),
    }};
}

void JumpImpulseScenario::reset() {
    phase_ = JumpImpulsePhase::disabled_settle;
    command_ = {};
    previous_contact_ = {{true, true}};
    departure_time_.fill(std::numeric_limits<double>::quiet_NaN());
    landing_time_.fill(std::numeric_limits<double>::quiet_NaN());
    commanded_force_.fill(0.0);
    reset_time_ = 0.0;
    ready_hold_start_ = -1.0;
    phase_start_ = 0.0;
    thrust_start_ = 0.0;
    all_air_candidate_start_ = -1.0;
    recovery_hold_start_ = -1.0;
    applied_hold_seconds_ = 0.0;
    initial_axle_height_ = 0.0;
    initial_com_height_ = 0.0;
    takeoff_time_ = std::numeric_limits<double>::quiet_NaN();
    takeoff_vertical_velocity_ = std::numeric_limits<double>::quiet_NaN();
    takeoff_com_vertical_velocity_ =
        std::numeric_limits<double>::quiet_NaN();
    takeoff_candidate_com_vertical_velocity_ =
        std::numeric_limits<double>::quiet_NaN();
    takeoff_com_height_ = std::numeric_limits<double>::quiet_NaN();
    support_airborne_time_ = std::numeric_limits<double>::quiet_NaN();
    balance_engaged_ = false;
    start_ready_ = false;
    pulse_completed_ = false;
    took_off_ = false;
    support_airborne_ = false;
    support_recovered_ = false;
    false_airborne_ = false;
    release_reason_ = "none";
    issue_ = "none";
}

const char *JumpImpulseScenario::phase_name() const noexcept {
    switch (phase_) {
    case JumpImpulsePhase::disabled_settle: return "jump_disabled_settle";
    case JumpImpulsePhase::standing: return "jump_standing";
    case JumpImpulsePhase::ramp_up: return "jump_ramp_up";
    case JumpImpulsePhase::hold: return "jump_hold";
    case JumpImpulsePhase::release: return "jump_release";
    case JumpImpulsePhase::wait_airborne: return "jump_wait_airborne";
    case JumpImpulsePhase::airborne: return "jump_airborne";
    case JumpImpulsePhase::post_landing: return "jump_post_landing";
    case JumpImpulsePhase::complete: return "jump_complete";
    case JumpImpulsePhase::failed: return "jump_failed";
    }
    return "jump_unknown";
}

std::array<bool, BC_SIDE_NUM> JumpImpulseScenario::wheel_contacts(
    const mjData &data
) const {
    std::array<bool, BC_SIDE_NUM> result{};
    for (int index = 0; index < data.ncon; ++index) {
        const mjContact &contact = data.contact[index];
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            result[side] = result[side] ||
                ((contact.geom[0] == ground_ &&
                  contact.geom[1] == wheel_[side]) ||
                 (contact.geom[1] == ground_ &&
                  contact.geom[0] == wheel_[side]));
        }
    }
    return result;
}

double JumpImpulseScenario::axle_height(const mjData &data) const {
    return 0.5 * (
        data.site_xpos[3 * wheel_axis_[BC_L] + 2] +
        data.site_xpos[3 * wheel_axis_[BC_R] + 2]);
}

double JumpImpulseScenario::axle_vertical_velocity(
    const mjData &data
) const {
    double velocity = 0.0;
    for (const int site : wheel_axis_) {
        mjtNum spatial[6]{};
        mj_objectVelocity(model_, &data, mjOBJ_SITE, site, spatial, 0);
        velocity += 0.5 * spatial[5];
    }
    return velocity;
}

double JumpImpulseScenario::com_height(const mjData &data) const noexcept {
    return data.subtree_com[3 * base_body_ + 2];
}

double JumpImpulseScenario::com_vertical_velocity(
    const mjData &data
) const noexcept {
    return data.subtree_linvel[3 * base_body_ + 2];
}

void JumpImpulseScenario::update_truth(
    mjData &data,
    const double time
) {
    const auto contact = wheel_contacts(data);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (previous_contact_[side] && !contact[side]) {
            departure_time_[side] = time;
        }
        if (took_off_ && !previous_contact_[side] && contact[side] &&
            !std::isfinite(landing_time_[side])) {
            landing_time_[side] = time;
        }
    }

    if (!contact[BC_L] && !contact[BC_R]) {
        if (all_air_candidate_start_ < 0.0) {
            all_air_candidate_start_ = time;
            mj_subtreeVel(model_, &data);
            takeoff_candidate_com_vertical_velocity_ =
                com_vertical_velocity(data);
            takeoff_com_height_ = com_height(data);
        } else if (!took_off_ &&
                   time - all_air_candidate_start_ >=
                       kTakeoffConfirmSeconds) {
            took_off_ = true;
            takeoff_time_ = all_air_candidate_start_;
            takeoff_vertical_velocity_ = axle_vertical_velocity(data);
            takeoff_com_vertical_velocity_ =
                takeoff_candidate_com_vertical_velocity_;
        }
    } else if (!took_off_) {
        all_air_candidate_start_ = -1.0;
    }
    previous_contact_ = contact;
}

bool JumpImpulseScenario::both_wheels_landed() const noexcept {
    return std::isfinite(landing_time_[BC_L]) &&
        std::isfinite(landing_time_[BC_R]);
}

double JumpImpulseScenario::first_landing_time() const noexcept {
    return both_wheels_landed() ?
        std::min(landing_time_[BC_L], landing_time_[BC_R]) :
        std::numeric_limits<double>::quiet_NaN();
}

void JumpImpulseScenario::start_release(
    const double time,
    const char *reason
) noexcept {
    applied_hold_seconds_ = phase_ == JumpImpulsePhase::hold ?
        time - phase_start_ : 0.0;
    release_reason_ = reason;
    phase_ = JumpImpulsePhase::release;
    phase_start_ = time;
}

void JumpImpulseScenario::apply_force_control(
    sim::SimulationRunner &runner
) {
    const auto force = commanded_force_;
    const auto &snapshot = runner.snapshot();
    const bc_gimbal_feedback_t gimbal{
        bc_wrap_anglef(-snapshot.state.value[BC_STATE_PSI]),
        -snapshot.state.value[BC_STATE_DPSI],
    };
    runner.step_with_control_transform(
        command_, gimbal, [force](bc_control_command_t &control) {
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                control.leg[side].length_strategy =
                    BC_LEG_LENGTH_AXIAL_FORCE;
                control.leg[side].target.axial_force =
                    static_cast<float>(force[side]);
            }
        });
}

void JumpImpulseScenario::finish_with_issue(const char *issue) noexcept {
    issue_ = issue;
    phase_ = JumpImpulsePhase::complete;
}

void JumpImpulseScenario::step(
    sim::MujocoPlant &plant,
    sim::SimulationRunner &runner,
    const SimulationSampler &sampler
) {
    if (finished()) return;
    const double time = plant.data().time;
    if (reset_time_ == 0.0) reset_time_ = time;
    if (time - reset_time_ > kScenarioTimeoutSeconds) {
        issue_ = "jump_impulse_timeout";
        phase_ = JumpImpulsePhase::failed;
        return;
    }

    const SimulationSample sample = sampler.read(
        plant.data(), runner.snapshot());
    if (start_ready_) update_truth(plant.data(), time);
    command_.system_enabled = static_cast<uint8_t>(
        phase_ != JumpImpulsePhase::disabled_settle);
    command_.balance_restart = static_cast<uint8_t>(
        command_.system_enabled &&
        runner.snapshot().state_machine.system == BC_SYSTEM_OFF);
    command_.forward_velocity = 0.0F;

    const auto support = runner.snapshot().state_machine.support;
    if (start_ready_ && support == BC_SUPPORT_AIRBORNE &&
        !support_airborne_) {
        support_airborne_ = true;
        support_airborne_time_ = time;
        if (!took_off_) {
            false_airborne_ = true;
            finish_with_issue("false_airborne");
            return;
        }
    }

    if (phase_ == JumpImpulsePhase::disabled_settle) {
        runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
        if (time - reset_time_ >= kDisabledSettleSeconds) {
            phase_ = JumpImpulsePhase::standing;
        }
        return;
    }

    if (phase_ == JumpImpulsePhase::standing) {
        const auto &snapshot = runner.snapshot();
        const bool active =
            snapshot.state_machine.motion == BC_MOTION_ACTIVE;
        balance_engaged_ = balance_engaged_ || active;
        bool ready = active &&
            std::abs(snapshot.state.value[BC_STATE_DS]) <=
                kVelocityTolerance &&
            std::abs(snapshot.state.value[BC_STATE_THETA_B]) <=
                kPitchTolerance &&
            std::abs(snapshot.state.value[BC_STATE_DTHETA_B]) <=
                kPitchRateTolerance &&
            std::abs(snapshot.roll) <= kRollTolerance &&
            std::abs(snapshot.roll_rate) <= kRollRateTolerance &&
            std::abs(snapshot.state.value[BC_STATE_PSI]) <= kYawTolerance &&
            std::abs(snapshot.state.value[BC_STATE_DPSI]) <=
                kYawRateTolerance;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            ready = ready &&
                std::abs(snapshot.leg[side].length - spec_.leg_length) <=
                    kLegLengthTolerance &&
                std::abs(snapshot.leg[side].length_velocity) <=
                    kLegSpeedTolerance;
        }
        if (ready) {
            if (ready_hold_start_ < 0.0) ready_hold_start_ = time;
            if (time - ready_hold_start_ >= kReadyHoldSeconds) {
                start_ready_ = true;
                initial_axle_height_ = axle_height(plant.data());
                initial_com_height_ = com_height(plant.data());
                previous_contact_ = wheel_contacts(plant.data());
                departure_time_.fill(
                    std::numeric_limits<double>::quiet_NaN());
                landing_time_.fill(
                    std::numeric_limits<double>::quiet_NaN());
                all_air_candidate_start_ = -1.0;
                thrust_start_ = time;
                phase_start_ = time;
                phase_ = JumpImpulsePhase::ramp_up;
                for (int side = 0; side < BC_SIDE_NUM; ++side) {
                    const double estimated =
                        snapshot.support_force[side].axial_force;
                    commanded_force_[side] =
                        snapshot.support_force[side].valid &&
                        std::isfinite(estimated) && estimated >= 0.0 &&
                        estimated <= kMaximumAxialForce ?
                        estimated : 76.204;
                }
            }
        } else {
            ready_hold_start_ = -1.0;
        }
        runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
        return;
    }

    if (phase_ == JumpImpulsePhase::ramp_up) {
        bool at_target = true;
        const double step = kForceRiseRate * plant.timestep();
        const double roll_force = runner.snapshot().roll_force_request;
        const std::array<double, BC_SIDE_NUM> roll_sign{{+1.0, -1.0}};
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            const double target = std::clamp(
                spec_.peak_force + roll_sign[side] * roll_force,
                0.0, kMaximumAxialForce);
            commanded_force_[side] = move_toward(
                commanded_force_[side], target, step);
            at_target = at_target &&
                std::abs(commanded_force_[side] - target) <= 1.0e-6;
        }
        if (at_target) {
            phase_ = JumpImpulsePhase::hold;
            phase_start_ = time;
        }
    } else if (phase_ == JumpImpulsePhase::hold) {
        const double roll_force = runner.snapshot().roll_force_request;
        commanded_force_[BC_L] = std::clamp(
            spec_.peak_force + roll_force, 0.0, kMaximumAxialForce);
        commanded_force_[BC_R] = std::clamp(
            spec_.peak_force - roll_force, 0.0, kMaximumAxialForce);
        if (time - phase_start_ >= spec_.hold_seconds) {
            start_release(time, "scheduled");
        }
    }

    if (phase_ == JumpImpulsePhase::ramp_up ||
        phase_ == JumpImpulsePhase::hold) {
        if (time - thrust_start_ >= std::max(
                kMinimumMaximumThrustSeconds,
                spec_.hold_seconds + 0.080)) {
            start_release(time, "maximum_thrust_time");
        } else if (sample.controller.leg[BC_L].length >=
                       kReleaseLegLength ||
                   sample.controller.leg[BC_R].length >=
                       kReleaseLegLength) {
            start_release(time, "leg_length_limit");
        }
    }

    if (phase_ == JumpImpulsePhase::release) {
        bool released = true;
        const double step = kForceReleaseRate * plant.timestep();
        for (double &force : commanded_force_) {
            force = move_toward(force, 0.0, step);
            released = released && force <= 1.0e-6;
        }
        if (released) {
            commanded_force_.fill(0.0);
            pulse_completed_ = true;
            phase_ = JumpImpulsePhase::wait_airborne;
            phase_start_ = time;
        }
    }

    if (phase_ == JumpImpulsePhase::wait_airborne) {
        if (support_airborne_) {
            phase_ = JumpImpulsePhase::airborne;
            runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
            return;
        }
        if (time - phase_start_ >= kSupportWaitSeconds) {
            finish_with_issue(took_off_ ? "support_timeout" : "no_takeoff");
            return;
        }
    }

    if (phase_ == JumpImpulsePhase::airborne && both_wheels_landed()) {
        phase_ = JumpImpulsePhase::post_landing;
    }
    if (phase_ == JumpImpulsePhase::post_landing) {
        const auto contacts = wheel_contacts(plant.data());
        const bool recovered =
            contacts[BC_L] && contacts[BC_R] &&
            support == BC_SUPPORT_GROUND;
        if (recovered) {
            if (recovery_hold_start_ < 0.0) recovery_hold_start_ = time;
            if (time - recovery_hold_start_ >= kRecoveryHoldSeconds) {
                support_recovered_ = true;
                phase_ = JumpImpulsePhase::complete;
                return;
            }
        } else {
            recovery_hold_start_ = -1.0;
        }
    }

    if (phase_ == JumpImpulsePhase::ramp_up ||
        phase_ == JumpImpulsePhase::hold ||
        phase_ == JumpImpulsePhase::release ||
        phase_ == JumpImpulsePhase::wait_airborne) {
        apply_force_control(runner);
    } else {
        runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
    }
}

JumpImpulseBenchmark::JumpImpulseBenchmark(
    const std::filesystem::path &model_path,
    const std::filesystem::path &output_directory
) : output_directory_(output_directory),
    plant_(model_path, 0.001),
    adapter_(plant_.model()),
    sampler_(plant_.model()),
    summary_(output_directory_ / "summary.csv", {
        "case", "peak_force", "leg_length", "hold_seconds",
        "applied_hold_seconds", "release_reason",
        "measurement_complete",
        "finite", "balance_engaged", "start_ready", "pulse_completed",
        "issue", "takeoff", "takeoff_vertical_velocity",
        "takeoff_com_vertical_velocity",
        "support_airborne", "support_airborne_delay", "flight_time",
        "departure_delta", "landing_delta", "maximum_wheel_clearance",
        "maximum_com_rise", "maximum_com_rise_after_takeoff",
        "both_wheels_landed",
        "support_recovered", "false_airborne", "non_wheel_contact",
        "maximum_pitch_deg", "maximum_roll_deg", "maximum_leg_length",
        "maximum_leg_speed", "maximum_joint_torque_request",
        "maximum_command_force", "maximum_estimated_axial_force",
        "ground_normal_impulse", "net_ground_impulse",
        "takeoff_com_momentum",
        "joint_saturated", "diverged",
    }) {}

JumpImpulseResult JumpImpulseBenchmark::run(const JumpImpulseSpec &spec) {
    plant_.reset();
    bc_controller_config_t config{};
    bc_controller_default_config(&config);
    config.motion.leg_length = static_cast<float>(spec.leg_length);
    sim::SimulationRunner runner(plant_, adapter_, config);
    JumpImpulseScenario scenario(spec, plant_.model());
    scenario.reset();

    JumpImpulseResult result{};
    result.spec = spec;
    result.name = scenario.name();
    CsvWriter trace(output_directory_ / result.name / "trace.csv", {
        "case", "phase", "time", "peak_force", "hold_seconds",
        "command_force_l",
        "command_force_r", "support_phase", "support_force_l",
        "support_force_r", "support_state_l", "support_state_r",
        "wheel_contact_l", "wheel_contact_r", "axle_height",
        "wheel_clearance", "axle_vertical_velocity",
        "com_height", "com_rise", "com_vertical_velocity",
        "ground_normal_force_l", "ground_normal_force_r",
        "base_vertical_velocity", "pitch", "pitch_rate", "roll",
        "roll_rate", "yaw", "yaw_rate", "leg_length_l", "leg_length_r",
        "leg_rate_l", "leg_rate_r", "non_wheel_contact",
        "non_wheel_pair", "wheel_torque_request_l",
        "wheel_torque_request_r", "joint_l_front", "joint_l_rear",
        "joint_r_front", "joint_r_rear", "joint_request_l_front",
        "joint_request_l_rear", "joint_request_r_front",
        "joint_request_r_rear",
    });

    while (!scenario.finished()) {
        scenario.step(plant_, runner, sampler_);
        mj_subtreeVel(&plant_.model(), &plant_.data());
        const SimulationSample sample = sampler_.read(
            plant_.data(), runner.snapshot());
        write_trace(trace, scenario, sample);

        result.finite = result.finite &&
            controller_snapshot_is_finite(sample.controller);
        if (scenario.start_ready()) {
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                result.maximum_command_force = std::max(
                    result.maximum_command_force,
                    scenario.commanded_force(side));
                result.maximum_estimated_axial_force = std::max(
                    result.maximum_estimated_axial_force,
                    static_cast<double>(
                        sample.controller.support_force[side].axial_force));
            }
            result.maximum_wheel_clearance = std::max(
                result.maximum_wheel_clearance,
                scenario.axle_height(plant_.data()) -
                    scenario.initial_axle_height());
            result.maximum_com_rise = std::max(
                result.maximum_com_rise,
                scenario.com_height(plant_.data()) -
                    scenario.initial_com_height());
            if (scenario.took_off()) {
                result.maximum_com_rise_after_takeoff = std::max(
                    result.maximum_com_rise_after_takeoff,
                    scenario.com_height(plant_.data()) -
                        scenario.takeoff_com_height());
            }
            if (!scenario.took_off() ||
                sample.time <= scenario.takeoff_time() +
                    plant_.timestep()) {
                result.ground_normal_impulse +=
                    (sample.contact.wheel_normal_force[BC_L] +
                     sample.contact.wheel_normal_force[BC_R]) *
                    plant_.timestep();
            }
            result.maximum_pitch = std::max(
                result.maximum_pitch, std::abs(static_cast<double>(
                    sample.controller.state.value[BC_STATE_THETA_B])));
            result.maximum_roll = std::max(
                result.maximum_roll,
                std::abs(static_cast<double>(sample.controller.roll)));
            result.non_wheel_contact = result.non_wheel_contact ||
                sample.contact.other;
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                result.maximum_leg_length = std::max(
                    result.maximum_leg_length,
                    static_cast<double>(sample.controller.leg[side].length));
                result.maximum_leg_speed = std::max(
                    result.maximum_leg_speed, std::abs(static_cast<double>(
                        sample.controller.leg[side].length_velocity)));
                for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                    const double requested = std::abs(static_cast<double>(
                        sample.controller.actuation_request.leg[side]
                            .joint_torque[joint]));
                    const double applied = std::abs(static_cast<double>(
                        sample.controller.actuation.leg[side]
                            .joint_torque[joint]));
                    result.maximum_joint_torque_request = std::max(
                        result.maximum_joint_torque_request, requested);
                    result.joint_saturated = result.joint_saturated ||
                        (requested > kJointTorqueLimit +
                            kSaturationTolerance &&
                         applied >= kJointTorqueLimit -
                            kSaturationTolerance);
                }
            }
        }
        result.diverged = result.diverged ||
            result.maximum_pitch > kDivergenceAngle ||
            result.maximum_roll > kDivergenceAngle;
        if (!result.finite) break;
    }

    result.measurement_complete = scenario.finished() && result.finite;
    result.balance_engaged = scenario.balance_engaged();
    result.start_ready = scenario.start_ready();
    result.pulse_completed = scenario.pulse_completed();
    result.issue = scenario.issue();
    result.took_off = scenario.took_off();
    result.support_airborne = scenario.support_airborne();
    result.both_wheels_landed = scenario.both_wheels_landed();
    result.support_recovered = scenario.support_recovered();
    result.false_airborne = scenario.false_airborne();
    result.takeoff_time = scenario.takeoff_time();
    result.takeoff_vertical_velocity =
        scenario.takeoff_vertical_velocity();
    result.takeoff_com_vertical_velocity =
        scenario.takeoff_com_vertical_velocity();
    result.applied_hold_seconds = scenario.applied_hold_seconds();
    result.release_reason = scenario.release_reason();
    if (result.took_off && result.support_airborne) {
        result.support_airborne_delay =
            scenario.support_airborne_time() - scenario.takeoff_time();
    }
    if (result.took_off && result.both_wheels_landed) {
        result.flight_time =
            scenario.first_landing_time() - scenario.takeoff_time();
    }
    if (result.took_off) {
        const double mass = mj_getTotalmass(&plant_.model());
        const double push_seconds =
            scenario.takeoff_time() - scenario.thrust_start_time();
        result.net_ground_impulse = result.ground_normal_impulse -
            mass * 9.81 * push_seconds;
        result.takeoff_com_momentum =
            mass * result.takeoff_com_vertical_velocity;
    }
    const auto &departure = scenario.departure_time();
    if (std::isfinite(departure[BC_L]) &&
        std::isfinite(departure[BC_R])) {
        result.departure_delta = std::abs(
            departure[BC_L] - departure[BC_R]);
    }
    const auto &landing = scenario.landing_time();
    if (std::isfinite(landing[BC_L]) && std::isfinite(landing[BC_R])) {
        result.landing_delta = std::abs(
            landing[BC_L] - landing[BC_R]);
    }
    if (!result.finite) result.issue = "non_finite_telemetry";
    if (result.diverged) result.issue = "attitude_diverged";
    write_summary(result);
    return result;
}

void JumpImpulseBenchmark::write_summary(
    const JumpImpulseResult &result
) {
    summary_.begin_row();
    summary_.value(result.name)
        .value(result.spec.peak_force)
        .value(result.spec.leg_length)
        .value(result.spec.hold_seconds)
        .value(result.applied_hold_seconds)
        .value(result.release_reason)
        .value(result.measurement_complete)
        .value(result.finite)
        .value(result.balance_engaged)
        .value(result.start_ready)
        .value(result.pulse_completed)
        .value(result.issue)
        .value(result.took_off)
        .value(result.takeoff_vertical_velocity)
        .value(result.takeoff_com_vertical_velocity)
        .value(result.support_airborne)
        .value(result.support_airborne_delay)
        .value(result.flight_time)
        .value(result.departure_delta)
        .value(result.landing_delta)
        .value(result.maximum_wheel_clearance)
        .value(result.maximum_com_rise)
        .value(result.maximum_com_rise_after_takeoff)
        .value(result.both_wheels_landed)
        .value(result.support_recovered)
        .value(result.false_airborne)
        .value(result.non_wheel_contact)
        .value(result.maximum_pitch * 180.0 / BC_PI)
        .value(result.maximum_roll * 180.0 / BC_PI)
        .value(result.maximum_leg_length)
        .value(result.maximum_leg_speed)
        .value(result.maximum_joint_torque_request)
        .value(result.maximum_command_force)
        .value(result.maximum_estimated_axial_force)
        .value(result.ground_normal_impulse)
        .value(result.net_ground_impulse)
        .value(result.takeoff_com_momentum)
        .value(result.joint_saturated)
        .value(result.diverged);
    summary_.end_row();
    summary_.flush();
}

void JumpImpulseBenchmark::write_trace(
    CsvWriter &trace,
    const JumpImpulseScenario &scenario,
    const SimulationSample &sample
) const {
    const auto &snapshot = sample.controller;
    trace.begin_row();
    trace.value(scenario.name())
        .value(scenario.phase_name())
        .value(sample.time)
        .value(scenario.spec().peak_force)
        .value(scenario.spec().hold_seconds)
        .value(scenario.commanded_force(BC_L))
        .value(scenario.commanded_force(BC_R))
        .value(bc_support_phase_state_name(snapshot.state_machine.support))
        .value(snapshot.support_force[BC_L].filtered_vertical_force)
        .value(snapshot.support_force[BC_R].filtered_vertical_force)
        .value(static_cast<int>(snapshot.support_force[BC_L].state))
        .value(static_cast<int>(snapshot.support_force[BC_R].state))
        .value(sample.contact.wheel[BC_L])
        .value(sample.contact.wheel[BC_R])
        .value(scenario.axle_height(plant_.data()))
        .value(scenario.axle_height(plant_.data()) -
            scenario.initial_axle_height())
        .value(scenario.axle_vertical_velocity(plant_.data()))
        .value(scenario.com_height(plant_.data()))
        .value(scenario.com_height(plant_.data()) -
            scenario.initial_com_height())
        .value(scenario.com_vertical_velocity(plant_.data()))
        .value(sample.contact.wheel_normal_force[BC_L])
        .value(sample.contact.wheel_normal_force[BC_R])
        .value(sample.base.vertical_velocity)
        .value(snapshot.state.value[BC_STATE_THETA_B])
        .value(snapshot.state.value[BC_STATE_DTHETA_B])
        .value(snapshot.roll)
        .value(snapshot.roll_rate)
        .value(snapshot.state.value[BC_STATE_PSI])
        .value(snapshot.state.value[BC_STATE_DPSI])
        .value(snapshot.leg[BC_L].length)
        .value(snapshot.leg[BC_R].length)
        .value(snapshot.leg[BC_L].length_velocity)
        .value(snapshot.leg[BC_R].length_velocity)
        .value(sample.contact.other)
        .value(sample.contact.unexpected)
        .value(snapshot.actuation_request.wheel_torque[BC_L])
        .value(snapshot.actuation_request.wheel_torque[BC_R])
        .value(snapshot.actuation.leg[BC_L].joint_torque[BC_FRONT])
        .value(snapshot.actuation.leg[BC_L].joint_torque[BC_REAR])
        .value(snapshot.actuation.leg[BC_R].joint_torque[BC_FRONT])
        .value(snapshot.actuation.leg[BC_R].joint_torque[BC_REAR])
        .value(snapshot.actuation_request.leg[BC_L]
            .joint_torque[BC_FRONT])
        .value(snapshot.actuation_request.leg[BC_L]
            .joint_torque[BC_REAR])
        .value(snapshot.actuation_request.leg[BC_R]
            .joint_torque[BC_FRONT])
        .value(snapshot.actuation_request.leg[BC_R]
            .joint_torque[BC_REAR]);
    trace.end_row();
}

} // namespace balance::benchmark
