#include "ramp_jump.hpp"

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
constexpr double kScenarioTimeoutSeconds = 15.0;
constexpr double kReadyHoldSeconds = 0.25;
constexpr double kSpeedHoldSeconds = 0.25;
constexpr double kRecoveryHoldSeconds = 0.50;
constexpr double kVelocityTolerance = 0.10;
constexpr double kLegLengthTolerance = 0.012;
constexpr double kLegSpeedTolerance = 0.05;
constexpr double kPitchTolerance = 3.0 * BC_PI / 180.0;
constexpr double kPitchRateTolerance = 0.15;
constexpr double kYawTolerance = 1.0 * BC_PI / 180.0;
constexpr double kYawRateTolerance = 0.10;
constexpr double kDivergenceAngle = 60.0 * BC_PI / 180.0;
constexpr double kTakeoffWindow = 0.15;
constexpr double kGravity = 9.81;
constexpr double kSaturationTolerance = 1.0e-4;

int require_id(const mjModel &model, const mjtObj type, const char *name) {
    const int id = mj_name2id(&model, type, name);
    if (id < 0) {
        throw std::runtime_error(
            "missing MuJoCo object '" + std::string(name) + "'");
    }
    return id;
}

bool is_descendant(const mjModel &model, int body, const int ancestor) {
    while (body > 0) {
        if (body == ancestor) return true;
        body = model.body_parentid[body];
    }
    return false;
}

std::string velocity_token(const double velocity) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << velocity;
    std::string token = stream.str();
    std::replace(token.begin(), token.end(), '.', 'p');
    while (!token.empty() && token.back() == '0') token.pop_back();
    if (!token.empty() && token.back() == 'p') token.pop_back();
    return token;
}

double truth_axle_velocity(const SimulationSample &sample) {
    return 0.5 * (
        sample.wheel.forward_velocity[BC_L] +
        sample.wheel.forward_velocity[BC_R]);
}

} // namespace

std::string ramp_jump_case_name(const RampJumpSpec &spec) {
    return "ramp_jump_v" + velocity_token(spec.target_velocity);
}

const std::array<RampJumpSpec, 5> &ramp_jump_cases() {
    static const std::array<RampJumpSpec, 5> cases{{
        {2.00, 0.24},
        {2.25, 0.24},
        {2.50, 0.24},
        {2.75, 0.24},
        {3.00, 0.24},
    }};
    return cases;
}

const RampJumpSpec *find_ramp_jump_case(const std::string_view name) noexcept {
    const auto &cases = ramp_jump_cases();
    const auto found = std::find_if(
        cases.begin(), cases.end(), [name](const RampJumpSpec &spec) {
            return ramp_jump_case_name(spec) == name;
        });
    return found == cases.end() ? nullptr : &*found;
}

RampJumpScenario::RampJumpScenario(
    const RampJumpSpec &spec, const mjModel &model
) : spec_(spec), name_(ramp_jump_case_name(spec)) {
    model_ = &model;
    ramp_ = require_id(model, mjOBJ_GEOM, "benchmark_ramp_17deg");
    ground_ = require_id(model, mjOBJ_GEOM, "ground");
    wheel_ = {{
        require_id(model, mjOBJ_GEOM, "Right_wheel_collision"),
        require_id(model, mjOBJ_GEOM, "Left_wheel_collision"),
    }};
    wheel_axis_ = {{
        require_id(model, mjOBJ_SITE, "Right_wheel_axis_site"),
        require_id(model, mjOBJ_SITE, "Left_wheel_axis_site"),
    }};
}

void RampJumpScenario::reset(sim::MujocoPlant &plant) {
    layout_ = plant.configure_ramp_jump_benchmark();
    phase_ = RampJumpPhase::disabled_settle;
    command_ = {};
    reset_time_ = plant.data().time;
    ready_hold_start_ = -1.0;
    speed_hold_start_ = -1.0;
    recovery_hold_start_ = -1.0;
    ramp_entry_velocity_ = std::numeric_limits<double>::quiet_NaN();
    takeoff_time_ = std::numeric_limits<double>::quiet_NaN();
    takeoff_x_ = std::numeric_limits<double>::quiet_NaN();
    takeoff_velocity_x_ = std::numeric_limits<double>::quiet_NaN();
    takeoff_velocity_z_ = std::numeric_limits<double>::quiet_NaN();
    wheel_departure_time_.fill(std::numeric_limits<double>::quiet_NaN());
    wheel_landing_time_.fill(std::numeric_limits<double>::quiet_NaN());
    wheel_landing_distance_.fill(std::numeric_limits<double>::quiet_NaN());
    previous_ramp_contact_.fill(false);
    current_contacts_ = {};
    ramp_contact_seen_ = false;
    balance_engaged_ = false;
    start_ready_ = false;
    entry_speed_stable_ = false;
    issue_ = "none";
}

const char *RampJumpScenario::phase_name() const noexcept {
    switch (phase_) {
    case RampJumpPhase::disabled_settle: return "jump_disabled_settle";
    case RampJumpPhase::standing: return "jump_standing";
    case RampJumpPhase::accelerating: return "jump_accelerating";
    case RampJumpPhase::approach: return "jump_approach";
    case RampJumpPhase::ascent: return "jump_ascent";
    case RampJumpPhase::airborne: return "jump_airborne";
    case RampJumpPhase::post_landing: return "jump_post_landing";
    case RampJumpPhase::complete: return "jump_complete";
    case RampJumpPhase::failed: return "jump_failed";
    }
    return "jump_unknown";
}

RampJumpScenario::ContactState
RampJumpScenario::read_contacts(const mjData &data) const {
    ContactState state{};
    for (int index = 0; index < data.ncon; ++index) {
        const mjContact &contact = data.contact[index];
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            state.ramp_wheel[side] = state.ramp_wheel[side] ||
                ((contact.geom[0] == ramp_ &&
                  contact.geom[1] == wheel_[side]) ||
                 (contact.geom[1] == ramp_ &&
                  contact.geom[0] == wheel_[side]));
            state.ground_wheel[side] = state.ground_wheel[side] ||
                ((contact.geom[0] == ground_ &&
                  contact.geom[1] == wheel_[side]) ||
                 (contact.geom[1] == ground_ &&
                  contact.geom[0] == wheel_[side]));
        }
    }
    return state;
}

std::array<double, 2> RampJumpScenario::axle_velocity(
    const mjData &data
) const {
    std::array<double, 2> velocity{};
    for (const int site : wheel_axis_) {
        mjtNum spatial[6]{};
        mj_objectVelocity(
            model_, &data, mjOBJ_SITE, site, spatial, 0);
        velocity[0] += 0.5 * spatial[3];
        velocity[1] += 0.5 * spatial[5];
    }
    return velocity;
}

double RampJumpScenario::wheel_axis_x(
    const mjData &data, const int side
) const {
    return data.site_xpos[3 * wheel_axis_[side]];
}

void RampJumpScenario::fail(const char *issue) noexcept {
    issue_ = issue;
    phase_ = RampJumpPhase::failed;
}

void RampJumpScenario::step(
    sim::MujocoPlant &plant,
    sim::SimulationRunner &runner,
    const SimulationSampler &sampler
) {
    if (finished()) return;
    if (plant.data().time - reset_time_ > kScenarioTimeoutSeconds) {
        fail("ramp_jump_timeout");
        return;
    }

    const SimulationSample sample = sampler.read(
        plant.data(), runner.snapshot());
    const ContactState contacts = read_contacts(plant.data());
    current_contacts_ = contacts;
    const double velocity = truth_axle_velocity(sample);

    command_.system_enabled = static_cast<uint8_t>(
        phase_ != RampJumpPhase::disabled_settle);
    command_.balance_restart = static_cast<uint8_t>(
        command_.system_enabled &&
        runner.snapshot().state_machine.system == BC_SYSTEM_OFF);
    command_.forward_velocity = 0.0F;

    if (phase_ == RampJumpPhase::disabled_settle) {
        runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
        if (plant.data().time - reset_time_ >= kDisabledSettleSeconds) {
            phase_ = RampJumpPhase::standing;
        }
        return;
    }

    if (phase_ == RampJumpPhase::standing) {
        const auto &snapshot = runner.snapshot();
        const bool active =
            snapshot.state_machine.motion == BC_MOTION_ACTIVE;
        balance_engaged_ = balance_engaged_ || active;
        bool ready = active;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            ready = ready &&
                std::abs(snapshot.leg[side].length - spec_.leg_length) <=
                    kLegLengthTolerance &&
                std::abs(snapshot.leg[side].length_velocity) <=
                    kLegSpeedTolerance;
        }
        ready = ready &&
            std::abs(snapshot.state.value[BC_STATE_THETA_B]) <=
                kPitchTolerance &&
            std::abs(snapshot.state.value[BC_STATE_DTHETA_B]) <=
                kPitchRateTolerance &&
            std::abs(snapshot.state.value[BC_STATE_PSI]) <= kYawTolerance &&
            std::abs(snapshot.state.value[BC_STATE_DPSI]) <=
                kYawRateTolerance;
        if (ready) {
            if (ready_hold_start_ < 0.0) {
                ready_hold_start_ = sample.time;
            } else if (sample.time - ready_hold_start_ >=
                       kReadyHoldSeconds) {
                start_ready_ = true;
                phase_ = RampJumpPhase::accelerating;
            }
        } else {
            ready_hold_start_ = -1.0;
        }
        runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
        return;
    }

    command_.forward_velocity = static_cast<float>(spec_.target_velocity);
    if (std::abs(velocity - spec_.target_velocity) <= kVelocityTolerance) {
        if (speed_hold_start_ < 0.0) speed_hold_start_ = sample.time;
        if (!entry_speed_stable_ &&
            sample.time - speed_hold_start_ >= kSpeedHoldSeconds) {
            entry_speed_stable_ = true;
            phase_ = RampJumpPhase::approach;
        }
    } else if (!entry_speed_stable_) {
        speed_hold_start_ = -1.0;
    }
    if (!entry_speed_stable_ && sample.axle.x >= layout_.start_x) {
        fail("entry_speed_not_stable");
        return;
    }
    const bool any_ramp_contact =
        contacts.ramp_wheel[BC_L] || contacts.ramp_wheel[BC_R];
    if ((phase_ == RampJumpPhase::accelerating ||
         phase_ == RampJumpPhase::approach) && any_ramp_contact) {
        ramp_entry_velocity_ = velocity;
        if (std::abs(ramp_entry_velocity_ - spec_.target_velocity) >
            kVelocityTolerance) {
            fail("entry_speed_not_stable");
            return;
        }
        phase_ = RampJumpPhase::ascent;
    }

    if (phase_ == RampJumpPhase::ascent) {
        ramp_contact_seen_ = ramp_contact_seen_ ||
            contacts.ramp_wheel[BC_L] || contacts.ramp_wheel[BC_R];
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            if (ramp_contact_seen_ && previous_ramp_contact_[side] &&
                !contacts.ramp_wheel[side] &&
                wheel_axis_x(plant.data(), side) >=
                    layout_.lip_x - kTakeoffWindow &&
                !std::isfinite(wheel_departure_time_[side])) {
                wheel_departure_time_[side] = sample.time;
            }
        }
        if (ramp_contact_seen_ &&
            !contacts.ramp_wheel[BC_L] &&
            !contacts.ramp_wheel[BC_R] &&
            std::isfinite(wheel_departure_time_[BC_L]) &&
            std::isfinite(wheel_departure_time_[BC_R]) &&
            sample.axle.x >= layout_.lip_x - 0.05) {
            const auto velocity_xz = axle_velocity(plant.data());
            takeoff_time_ = sample.time;
            takeoff_x_ = sample.axle.x;
            takeoff_velocity_x_ = velocity_xz[0];
            takeoff_velocity_z_ = velocity_xz[1];
            phase_ = RampJumpPhase::airborne;
        }
    }

    if (phase_ == RampJumpPhase::airborne ||
        phase_ == RampJumpPhase::post_landing) {
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            if (contacts.ground_wheel[side] &&
                !std::isfinite(wheel_landing_time_[side])) {
                wheel_landing_time_[side] = sample.time;
                wheel_landing_distance_[side] =
                    wheel_axis_x(plant.data(), side) - layout_.lip_x;
                phase_ = RampJumpPhase::post_landing;
            }
        }
        const bool both_landed =
            std::isfinite(wheel_landing_time_[BC_L]) &&
            std::isfinite(wheel_landing_time_[BC_R]);
        const bool support_ground =
            runner.snapshot().state_machine.support == BC_SUPPORT_GROUND;
        if (both_landed && support_ground &&
            contacts.ground_wheel[BC_L] && contacts.ground_wheel[BC_R]) {
            if (recovery_hold_start_ < 0.0) {
                recovery_hold_start_ = sample.time;
            } else if (sample.time - recovery_hold_start_ >=
                       kRecoveryHoldSeconds) {
                phase_ = RampJumpPhase::complete;
            }
        } else {
            recovery_hold_start_ = -1.0;
        }
    }

    previous_ramp_contact_ = contacts.ramp_wheel;
    runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
}

RampJumpBenchmark::RampJumpBenchmark(
    const std::filesystem::path &model_path,
    const std::filesystem::path &output_directory
) : output_directory_(output_directory),
    plant_(model_path, 0.001),
    adapter_(plant_.model()),
    sampler_(plant_.model()),
    summary_(output_directory_ / "summary.csv", {
        "case", "target_velocity", "leg_length", "completed", "finite",
        "balance_engaged", "start_ready", "entry_speed_stable", "issue",
        "ramp_entry_velocity", "takeoff", "takeoff_x",
        "takeoff_velocity_x", "takeoff_velocity_z", "flight_time",
        "departure_delta", "landing_distance_l", "landing_distance_r",
        "landing_delta", "first_landing_distance", "ballistic_distance",
        "support_recovered", "premature_airborne", "non_wheel_contact",
        "first_non_wheel_contact", "maximum_pitch_deg",
        "maximum_roll_deg", "maximum_yaw_error_deg",
        "maximum_wheel_torque_request", "maximum_joint_torque_request",
        "wheel_saturated", "joint_saturated", "diverged",
    }) {
    ramp_ = require_id(
        plant_.model(), mjOBJ_GEOM, "benchmark_ramp_17deg");
    ground_ = require_id(plant_.model(), mjOBJ_GEOM, "ground");
    wheel_ = {{
        require_id(
            plant_.model(), mjOBJ_GEOM, "Right_wheel_collision"),
        require_id(
            plant_.model(), mjOBJ_GEOM, "Left_wheel_collision"),
    }};
    const int base = require_id(
        plant_.model(), mjOBJ_BODY, "base_link");
    for (int geom = 0; geom < plant_.model().ngeom; ++geom) {
        if (is_descendant(
                plant_.model(), plant_.model().geom_bodyid[geom], base)) {
            robot_geoms_.push_back(geom);
        }
    }
}

RampJumpBenchmark::ContactObservation
RampJumpBenchmark::observe_contacts() const {
    ContactObservation result{};
    for (int index = 0; index < plant_.data().ncon; ++index) {
        const mjContact &contact = plant_.data().contact[index];
        int terrain = -1;
        int robot = -1;
        for (const int surface : {ramp_, ground_}) {
            if (contact.geom[0] == surface) {
                terrain = surface;
                robot = contact.geom[1];
            } else if (contact.geom[1] == surface) {
                terrain = surface;
                robot = contact.geom[0];
            }
        }
        if (terrain < 0 || std::find(
                robot_geoms_.begin(), robot_geoms_.end(), robot) ==
                robot_geoms_.end()) continue;
        bool wheel_contact = false;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            if (robot != wheel_[side]) continue;
            wheel_contact = true;
            result.ramp_wheel[side] = result.ramp_wheel[side] ||
                terrain == ramp_;
            result.ground_wheel[side] = result.ground_wheel[side] ||
                terrain == ground_;
        }
        if (wheel_contact) continue;
        result.non_wheel = true;
        mjtNum force[6]{};
        mj_contactForce(
            &plant_.model(), &plant_.data(), index, force);
        const double normal_force = std::abs(force[0]);
        result.non_wheel_normal_force += normal_force;
        if (normal_force > result.strongest_non_wheel_normal_force) {
            result.strongest_non_wheel_normal_force = normal_force;
            result.strongest_non_wheel_contact_x = contact.pos[0];
            result.strongest_non_wheel_contact_z = contact.pos[2];
        }
        if (result.first_non_wheel == "none") {
            const char *terrain_name = mj_id2name(
                &plant_.model(), mjOBJ_GEOM, terrain);
            const char *robot_name = mj_id2name(
                &plant_.model(), mjOBJ_GEOM, robot);
            const int robot_body = plant_.model().geom_bodyid[robot];
            const char *robot_body_name = mj_id2name(
                &plant_.model(), mjOBJ_BODY, robot_body);
            result.first_non_wheel =
                std::string(terrain_name != nullptr ? terrain_name : "terrain") +
                "+" + (robot_name != nullptr ? robot_name :
                    (robot_body_name != nullptr ? robot_body_name : "robot"));
        }
    }
    return result;
}

RampJumpResult RampJumpBenchmark::run(const RampJumpSpec &spec) {
    plant_.reset();
    bc_controller_config_t config{};
    bc_controller_default_config(&config);
    config.motion.leg_length = static_cast<float>(spec.leg_length);
    sim::SimulationRunner runner(plant_, adapter_, config);
    RampJumpScenario scenario(spec, plant_.model());
    scenario.reset(plant_);

    RampJumpResult result{};
    result.spec = spec;
    result.name = scenario.name();
    CsvWriter trace(output_directory_ / result.name / "trace.csv", {
        "case", "phase", "time", "target_velocity", "axle_x", "base_z",
        "wheel_speed_velocity", "axle_velocity_x", "axle_velocity_z",
        "base_vertical_velocity", "pitch", "pitch_rate", "roll",
        "roll_rate", "yaw", "yaw_rate", "leg_length_l", "leg_length_r",
        "leg_rate_l", "leg_rate_r", "support_phase", "support_force_l",
        "support_force_r", "support_state_l", "support_state_r",
        "ramp_wheel_l", "ramp_wheel_r", "ground_wheel_l",
        "ground_wheel_r", "non_wheel_contact", "non_wheel_pair",
        "non_wheel_normal_force", "non_wheel_contact_x",
        "non_wheel_contact_z", "wheel_torque_request_l",
        "wheel_torque_request_r", "wheel_torque_l", "wheel_torque_r",
        "joint_l_front", "joint_l_rear", "joint_r_front", "joint_r_rear",
    });

    while (!scenario.finished()) {
        scenario.step(plant_, runner, sampler_);
        const SimulationSample sample = sampler_.read(
            plant_.data(), runner.snapshot());
        const ContactObservation contact = observe_contacts();
        write_trace(trace, scenario, sample, contact);

        result.finite = result.finite &&
            controller_snapshot_is_finite(sample.controller);
        result.balance_engaged = result.balance_engaged ||
            scenario.balance_engaged();
        result.start_ready = result.start_ready || scenario.start_ready();
        result.entry_speed_stable = result.entry_speed_stable ||
            scenario.entry_speed_stable();
        result.premature_airborne = result.premature_airborne ||
            (scenario.phase() == RampJumpPhase::ascent &&
             sample.axle.x < scenario.layout().lip_x - 0.05 &&
             sample.controller.state_machine.support ==
                 BC_SUPPORT_AIRBORNE);
        if (scenario.start_ready()) {
            result.maximum_pitch = std::max(
                result.maximum_pitch, std::abs(static_cast<double>(
                    sample.controller.state.value[BC_STATE_THETA_B])));
            result.maximum_roll = std::max(
                result.maximum_roll,
                std::abs(static_cast<double>(sample.controller.roll)));
            result.maximum_yaw_error = std::max(
                result.maximum_yaw_error, std::abs(static_cast<double>(
                    sample.controller.state.value[BC_STATE_PSI])));
        }
        result.diverged = result.diverged ||
            result.maximum_pitch > kDivergenceAngle ||
            result.maximum_roll > kDivergenceAngle;
        result.non_wheel_contact = result.non_wheel_contact ||
            (scenario.start_ready() && contact.non_wheel);
        if (scenario.start_ready() && contact.non_wheel &&
            result.first_non_wheel_contact == "none") {
            result.first_non_wheel_contact = contact.first_non_wheel;
        }

        if (scenario.start_ready()) {
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                result.maximum_wheel_torque_request = std::max(
                    result.maximum_wheel_torque_request,
                    std::abs(static_cast<double>(sample.controller.
                        actuation_request.wheel_torque[side])));
                result.wheel_saturated = result.wheel_saturated ||
                    std::abs(sample.controller.actuation_request.
                        wheel_torque[side] -
                        sample.controller.actuation.wheel_torque[side]) >
                        kSaturationTolerance;
                for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                    result.maximum_joint_torque_request = std::max(
                        result.maximum_joint_torque_request,
                        std::abs(static_cast<double>(sample.controller.
                            actuation_request.leg[side].joint_torque[joint])));
                    result.joint_saturated = result.joint_saturated ||
                        std::abs(sample.controller.actuation_request.leg[side].
                            joint_torque[joint] -
                            sample.controller.actuation.leg[side].
                                joint_torque[joint]) > kSaturationTolerance;
                }
            }
        }
        if (!result.finite || result.diverged) break;
    }
    trace.flush();

    result.completed = scenario.phase() == RampJumpPhase::complete;
    result.issue = result.diverged ? "attitude_diverged" : scenario.issue();
    result.ramp_entry_velocity = scenario.ramp_entry_velocity();
    result.takeoff_time = scenario.takeoff_time();
    result.takeoff_x = scenario.takeoff_x();
    result.takeoff_velocity_x = scenario.takeoff_velocity_x();
    result.takeoff_velocity_z = scenario.takeoff_velocity_z();
    result.took_off = std::isfinite(result.takeoff_time);
    result.wheel_departure_time = scenario.wheel_departure_time();
    result.wheel_landing_time = scenario.wheel_landing_time();
    result.wheel_landing_distance = scenario.wheel_landing_distance();
    result.both_wheels_landed =
        std::isfinite(result.wheel_landing_time[BC_L]) &&
        std::isfinite(result.wheel_landing_time[BC_R]);
    if (result.both_wheels_landed) {
        const double first_landing_time = std::min(
            result.wheel_landing_time[BC_L],
            result.wheel_landing_time[BC_R]);
        result.flight_time = first_landing_time - result.takeoff_time;
        result.first_landing_distance = std::min(
            result.wheel_landing_distance[BC_L],
            result.wheel_landing_distance[BC_R]);
    }
    result.support_recovered = result.completed;
    if (result.took_off) {
        const double discriminant = result.takeoff_velocity_z *
                result.takeoff_velocity_z +
            2.0 * kGravity * scenario.layout().height;
        const double flight = (result.takeoff_velocity_z +
            std::sqrt(std::max(0.0, discriminant))) / kGravity;
        result.ballistic_distance = result.takeoff_velocity_x * flight;
    }
    write_summary(result);
    return result;
}

void RampJumpBenchmark::write_summary(const RampJumpResult &result) {
    const double departure_delta =
        std::abs(result.wheel_departure_time[BC_L] -
                 result.wheel_departure_time[BC_R]);
    const double landing_delta =
        std::abs(result.wheel_landing_time[BC_L] -
                 result.wheel_landing_time[BC_R]);
    summary_.begin_row();
    summary_.value(result.name)
        .value(result.spec.target_velocity)
        .value(result.spec.leg_length)
        .value(result.completed)
        .value(result.finite)
        .value(result.balance_engaged)
        .value(result.start_ready)
        .value(result.entry_speed_stable)
        .value(result.issue)
        .value(result.ramp_entry_velocity)
        .value(result.took_off)
        .value(result.takeoff_x)
        .value(result.takeoff_velocity_x)
        .value(result.takeoff_velocity_z)
        .value(result.flight_time)
        .value(departure_delta)
        .value(result.wheel_landing_distance[BC_L])
        .value(result.wheel_landing_distance[BC_R])
        .value(landing_delta)
        .value(result.first_landing_distance)
        .value(result.ballistic_distance)
        .value(result.support_recovered)
        .value(result.premature_airborne)
        .value(result.non_wheel_contact)
        .value(result.first_non_wheel_contact)
        .value(result.maximum_pitch * 180.0 / BC_PI)
        .value(result.maximum_roll * 180.0 / BC_PI)
        .value(result.maximum_yaw_error * 180.0 / BC_PI)
        .value(result.maximum_wheel_torque_request)
        .value(result.maximum_joint_torque_request)
        .value(result.wheel_saturated)
        .value(result.joint_saturated)
        .value(result.diverged);
    summary_.end_row();
    summary_.flush();
}

void RampJumpBenchmark::write_trace(
    CsvWriter &trace,
    const RampJumpScenario &scenario,
    const SimulationSample &sample,
    const ContactObservation &contact
) const {
    const auto &snapshot = sample.controller;
    const auto axle_velocity = scenario.axle_velocity_truth(plant_.data());
    trace.begin_row();
    trace.value(scenario.name())
        .value(scenario.phase_name())
        .value(sample.time)
        .value(scenario.commanded_velocity())
        .value(sample.axle.x)
        .value(sample.base.z)
        .value(truth_axle_velocity(sample))
        .value(axle_velocity[0])
        .value(axle_velocity[1])
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
        .value(bc_support_phase_state_name(snapshot.state_machine.support))
        .value(snapshot.support_force[BC_L].filtered_vertical_force)
        .value(snapshot.support_force[BC_R].filtered_vertical_force)
        .value(static_cast<int>(snapshot.support_force[BC_L].state))
        .value(static_cast<int>(snapshot.support_force[BC_R].state))
        .value(contact.ramp_wheel[BC_L])
        .value(contact.ramp_wheel[BC_R])
        .value(contact.ground_wheel[BC_L])
        .value(contact.ground_wheel[BC_R])
        .value(contact.non_wheel)
        .value(contact.first_non_wheel)
        .value(contact.non_wheel_normal_force)
        .value(contact.strongest_non_wheel_contact_x)
        .value(contact.strongest_non_wheel_contact_z)
        .value(snapshot.actuation_request.wheel_torque[BC_L])
        .value(snapshot.actuation_request.wheel_torque[BC_R])
        .value(snapshot.actuation.wheel_torque[BC_L])
        .value(snapshot.actuation.wheel_torque[BC_R])
        .value(snapshot.actuation.leg[BC_L].joint_torque[BC_FRONT])
        .value(snapshot.actuation.leg[BC_L].joint_torque[BC_REAR])
        .value(snapshot.actuation.leg[BC_R].joint_torque[BC_FRONT])
        .value(snapshot.actuation.leg[BC_R].joint_torque[BC_REAR]);
    trace.end_row();
}

} // namespace balance::benchmark
