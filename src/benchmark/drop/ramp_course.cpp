#include "ramp_course.hpp"

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
constexpr double kReadyHoldSeconds = 0.25;
constexpr double kStopHoldSeconds = 0.25;
constexpr double kScenarioTimeoutSeconds = 15.0;
constexpr double kLegLengthTolerance = 0.012;
constexpr double kLegSpeedTolerance = 0.05;
constexpr double kPitchTolerance = 3.0 * BC_PI / 180.0;
constexpr double kPitchRateTolerance = 0.15;
constexpr double kYawTolerance = 1.0 * BC_PI / 180.0;
constexpr double kYawRateTolerance = 0.10;
constexpr double kStopVelocityTolerance = 0.05;
constexpr double kAscentStopLead = 0.85;
constexpr double kExitCommandDistance = 0.40;
constexpr double kDistanceSearchLimit = 0.20;
constexpr double kDivergenceAngle = 60.0 * BC_PI / 180.0;
constexpr double kProgressDistance = 0.05;
constexpr double kProgressTimeoutSeconds = 1.5;
constexpr double kReverseVelocity = -0.20;
constexpr double kReverseHoldSeconds = 0.40;
constexpr double kStoppingReverseDistance = 0.50;
constexpr double kStoppingTimeoutSeconds = 4.0;
constexpr double kSaturationTolerance = 1.0e-4;

int require_id(const mjModel &model, const mjtObj type, const char *name) {
    const int id = mj_name2id(&model, type, name);
    if (id < 0) {
        throw std::runtime_error(
            "missing MuJoCo object '" + std::string(name) + "'");
    }
    return id;
}

const char *mode_name(const RampCourseMode mode) {
    switch (mode) {
    case RampCourseMode::ascent_stop: return "ascent_stop";
    case RampCourseMode::descent: return "descent";
    case RampCourseMode::traverse: return "traverse";
    }
    return "unknown";
}

std::string length_token(const double length) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << length;
    std::string token = stream.str();
    std::replace(token.begin(), token.end(), '.', 'p');
    return token;
}

double truth_axle_velocity(const SimulationSample &sample) {
    return 0.5 * (
        sample.wheel.forward_velocity[BC_L] +
        sample.wheel.forward_velocity[BC_R]);
}

bool is_descendant(
    const mjModel &model, int body, const int ancestor
) {
    while (body > 0) {
        if (body == ancestor) return true;
        body = model.body_parentid[body];
    }
    return false;
}

} // namespace

std::string ramp_course_case_name(const RampCourseSpec &spec) {
    return "ramp_course_" + std::string(mode_name(spec.mode)) +
        "_l" + length_token(spec.leg_length) +
        (spec.beveled_transition ? "_beveled" : "");
}

const std::array<RampCourseSpec, 4> &ramp_course_cases() {
    static const std::array<RampCourseSpec, 4> cases{{
        {RampCourseMode::ascent_stop, 0.18, 2.0},
        {RampCourseMode::ascent_stop, 0.24, 2.0},
        {RampCourseMode::descent, 0.24, 2.0},
        {RampCourseMode::traverse, 0.24, 2.0},
    }};
    return cases;
}

const RampCourseSpec *find_ramp_course_case(
    const std::string_view name
) noexcept {
    const auto &cases = ramp_course_cases();
    const auto found = std::find_if(
        cases.begin(), cases.end(), [name](const RampCourseSpec &spec) {
            return ramp_course_case_name(spec) == name;
        });
    return found == cases.end() ? nullptr : &*found;
}

RampCourseScenario::RampCourseScenario(
    const RampCourseSpec &spec, const mjModel &model
) : spec_(spec), name_(ramp_course_case_name(spec)) {
    base_qpos_ = model.jnt_qposadr[
        require_id(model, mjOBJ_JOINT, "base_free_joint")];
    wheel_axis_ = {{
        require_id(model, mjOBJ_SITE, "Right_wheel_axis_site"),
        require_id(model, mjOBJ_SITE, "Left_wheel_axis_site"),
    }};
}

void RampCourseScenario::set_initial_axle_x(
    sim::MujocoPlant &plant, const double target_x
) const {
    mj_forward(&plant.model(), &plant.data());
    const double axle_x = 0.5 * (
        plant.data().site_xpos[3 * wheel_axis_[BC_L]] +
        plant.data().site_xpos[3 * wheel_axis_[BC_R]]);
    plant.data().qpos[base_qpos_] += target_x - axle_x;
    mj_forward(&plant.model(), &plant.data());
}

void RampCourseScenario::reset(sim::MujocoPlant &plant) {
    layout_ = plant.configure_ramp_course_benchmark(
        spec_.beveled_transition);
    if (spec_.mode == RampCourseMode::descent) {
        plant.data().qpos[base_qpos_ + 2] += layout_.height;
        set_initial_axle_x(plant, layout_.ascent_end_x + 0.80);
    } else {
        set_initial_axle_x(plant, 0.0);
    }

    phase_ = RampCoursePhase::disabled_settle;
    command_ = {};
    reset_time_ = plant.data().time;
    active_time_ = -1.0;
    ready_hold_start_ = -1.0;
    stop_hold_start_ = -1.0;
    progress_reference_time_ = -1.0;
    progress_reference_x_ = 0.0;
    reverse_start_time_ = -1.0;
    stopping_start_x_ = 0.0;
    stopping_start_time_ = -1.0;
    balance_engaged_ = false;
    start_ready_ = false;
    issue_ = "none";
}

const char *RampCourseScenario::phase_name() const noexcept {
    switch (phase_) {
    case RampCoursePhase::disabled_settle: return "ramp_disabled_settle";
    case RampCoursePhase::standing: return "ramp_standing";
    case RampCoursePhase::approach: return "ramp_approach";
    case RampCoursePhase::ascent: return "ramp_ascent";
    case RampCoursePhase::platform: return "ramp_platform";
    case RampCoursePhase::descent: return "ramp_descent";
    case RampCoursePhase::exit: return "ramp_exit";
    case RampCoursePhase::stopping: return "ramp_stopping";
    case RampCoursePhase::complete: return "ramp_complete";
    case RampCoursePhase::failed: return "ramp_failed";
    }
    return "ramp_unknown";
}

void RampCourseScenario::fail(const char *issue) noexcept {
    issue_ = issue;
    phase_ = RampCoursePhase::failed;
    command_.forward_velocity = 0.0F;
}

void RampCourseScenario::step(
    sim::MujocoPlant &plant,
    sim::SimulationRunner &runner,
    const SimulationSampler &sampler
) {
    if (finished()) return;
    const SimulationSample sample = sampler.read(
        plant.data(), runner.snapshot());
    if (!controller_snapshot_is_finite(sample.controller)) {
        fail("non_finite_telemetry");
        return;
    }
    if (plant.data().time - reset_time_ > kScenarioTimeoutSeconds) {
        fail("ramp_course_timeout");
        return;
    }
    if (start_ready_ &&
        (std::abs(sample.controller.state.value[BC_STATE_THETA_B]) >
             kDivergenceAngle ||
         std::abs(sample.controller.roll) > kDivergenceAngle)) {
        fail("attitude_diverged");
        return;
    }

    command_.system_enabled = static_cast<uint8_t>(
        phase_ != RampCoursePhase::disabled_settle);
    command_.balance_restart = static_cast<uint8_t>(
        command_.system_enabled &&
        runner.snapshot().state_machine.system == BC_SYSTEM_OFF);
    command_.forward_velocity = 0.0F;

    if (phase_ == RampCoursePhase::disabled_settle) {
        runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
        if (plant.data().time - reset_time_ >= kDisabledSettleSeconds) {
            phase_ = RampCoursePhase::standing;
        }
        return;
    }

    if (phase_ == RampCoursePhase::standing) {
        const auto &snapshot = runner.snapshot();
        const bool active =
            snapshot.state_machine.motion == BC_MOTION_ACTIVE;
        if (active) {
            balance_engaged_ = true;
            if (active_time_ < 0.0) active_time_ = plant.data().time;
        }
        bool legs_ready = active;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            legs_ready = legs_ready &&
                std::abs(snapshot.leg[side].length - spec_.leg_length) <=
                    kLegLengthTolerance &&
                std::abs(snapshot.leg[side].length_velocity) <=
                    kLegSpeedTolerance;
        }
        const bool ready = legs_ready &&
            std::abs(snapshot.state.value[BC_STATE_THETA_B]) <=
                kPitchTolerance &&
            std::abs(snapshot.state.value[BC_STATE_DTHETA_B]) <=
                kPitchRateTolerance &&
            std::abs(snapshot.state.value[BC_STATE_PSI]) <= kYawTolerance &&
            std::abs(snapshot.state.value[BC_STATE_DPSI]) <=
                kYawRateTolerance;
        if (ready) {
            if (ready_hold_start_ < 0.0) {
                ready_hold_start_ = plant.data().time;
            } else if (plant.data().time - ready_hold_start_ >=
                       kReadyHoldSeconds) {
                start_ready_ = true;
                phase_ = spec_.mode == RampCourseMode::descent ?
                    RampCoursePhase::platform : RampCoursePhase::approach;
                progress_reference_time_ = sample.time;
                progress_reference_x_ = sample.axle.x;
            }
        } else {
            ready_hold_start_ = -1.0;
        }
        runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
        return;
    }

    command_.forward_velocity = static_cast<float>(spec_.target_velocity);
    const bool driving = phase_ != RampCoursePhase::stopping;
    const double velocity = truth_axle_velocity(sample);
    if (driving && sample.axle.x >=
        progress_reference_x_ + kProgressDistance) {
        progress_reference_x_ = sample.axle.x;
        progress_reference_time_ = sample.time;
    }
    if (driving && velocity <= kReverseVelocity) {
        if (reverse_start_time_ < 0.0) reverse_start_time_ = sample.time;
        if (sample.time - reverse_start_time_ >= kReverseHoldSeconds) {
            fail("reversed_during_course");
            return;
        }
    } else {
        reverse_start_time_ = -1.0;
    }
    if (driving && progress_reference_time_ >= 0.0 &&
        sample.time - progress_reference_time_ >= kProgressTimeoutSeconds) {
        fail("forward_progress_lost");
        return;
    }
    if (phase_ == RampCoursePhase::approach &&
        sample.axle.x >= layout_.ascent_start_x) {
        phase_ = RampCoursePhase::ascent;
    }
    if (phase_ == RampCoursePhase::ascent) {
        if (spec_.mode == RampCourseMode::ascent_stop &&
            sample.axle.x >= layout_.ascent_end_x - kAscentStopLead) {
            phase_ = RampCoursePhase::stopping;
            stopping_start_x_ = sample.axle.x;
            stopping_start_time_ = sample.time;
        } else if (sample.axle.x >= layout_.ascent_end_x) {
            phase_ = RampCoursePhase::platform;
        }
    }
    if (phase_ == RampCoursePhase::platform &&
        sample.axle.x >= layout_.platform_end_x) {
        phase_ = RampCoursePhase::descent;
    }
    if (phase_ == RampCoursePhase::descent &&
        sample.axle.x >= layout_.descent_end_x) {
        phase_ = RampCoursePhase::exit;
    }
    if (phase_ == RampCoursePhase::exit &&
        sample.axle.x >= layout_.descent_end_x + kExitCommandDistance) {
        phase_ = RampCoursePhase::stopping;
        stopping_start_x_ = sample.axle.x;
        stopping_start_time_ = sample.time;
    }

    if (phase_ == RampCoursePhase::stopping) {
        command_.forward_velocity = 0.0F;
        if (sample.axle.x <=
            stopping_start_x_ - kStoppingReverseDistance) {
            fail("reversed_while_stopping");
            return;
        }
        if (sample.time - stopping_start_time_ >=
            kStoppingTimeoutSeconds) {
            fail("stopping_not_settled");
            return;
        }
        const bool stopped =
            std::abs(truth_axle_velocity(sample)) <=
                kStopVelocityTolerance &&
            std::abs(sample.controller.state.value[BC_STATE_DS]) <=
                kStopVelocityTolerance &&
            sample.controller.velocity_estimator.wheel_velocity_reliable &&
            sample.controller.state_machine.forward == BC_FORWARD_HOLD;
        if (stopped) {
            if (stop_hold_start_ < 0.0) stop_hold_start_ = sample.time;
            if (sample.time - stop_hold_start_ >= kStopHoldSeconds) {
                phase_ = RampCoursePhase::complete;
            }
        } else {
            stop_hold_start_ = -1.0;
        }
    }
    runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
}

RampCourseBenchmark::RampCourseBenchmark(
    const std::filesystem::path &model_path,
    const std::filesystem::path &output_directory
) : output_directory_(output_directory),
    plant_(model_path, 0.001),
    adapter_(plant_.model()),
    sampler_(plant_.model()),
    summary_(output_directory_ / "summary.csv", {
        "case", "mode", "leg_length", "target_velocity",
        "completed", "finite", "balance_engaged", "start_ready",
        "issue", "non_wheel_collision", "first_collision_geom",
        "first_collision_phase", "collision_x", "collision_pitch_deg",
        "minimum_clearance", "minimum_clearance_geom",
        "minimum_clearance_x", "maximum_pitch_deg", "maximum_roll_deg",
        "maximum_yaw_error_deg", "ramp_wheel_contact_delta",
        "airborne_events", "maximum_airborne_duration",
        "first_airborne_time", "first_landing_time",
        "first_landing_wheel_delta", "wheel_velocity_lost",
        "wheel_velocity_recovered", "wheel_velocity_recovery_seconds",
        "forward_hold_recovered", "final_truth_velocity",
        "final_estimated_velocity", "final_position_error",
        "maximum_wheel_torque_request", "maximum_joint_torque_request",
        "wheel_saturated", "joint_saturated", "diverged",
    }) {
    wheel_ = {{
        require_id(plant_.model(), mjOBJ_GEOM, "Right_wheel_collision"),
        require_id(plant_.model(), mjOBJ_GEOM, "Left_wheel_collision"),
    }};
    const int base_body = require_id(
        plant_.model(), mjOBJ_BODY, "base_link");
    for (int geom = 0; geom < plant_.model().ngeom; ++geom) {
        if (!is_descendant(
                plant_.model(), plant_.model().geom_bodyid[geom],
                base_body)) continue;
        if (geom == wheel_[BC_L] || geom == wheel_[BC_R]) continue;
        non_wheel_robot_geoms_.push_back(geom);
    }
}

RampCourseBenchmark::TerrainObservation
RampCourseBenchmark::observe_terrain() {
    TerrainObservation observation{};
    for (int contact_index = 0;
         contact_index < plant_.data().ncon; ++contact_index) {
        const mjContact &contact = plant_.data().contact[contact_index];
        int terrain_index = -1;
        int other = -1;
        for (int index = 0; index < static_cast<int>(terrain_.size()); ++index) {
            if (contact.geom[0] == terrain_[index]) {
                terrain_index = index;
                other = contact.geom[1];
            } else if (contact.geom[1] == terrain_[index]) {
                terrain_index = index;
                other = contact.geom[0];
            }
        }
        if (terrain_index < 0) continue;
        bool wheel = false;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            if (other == wheel_[side]) {
                observation.wheel[side] = true;
                wheel = true;
            }
        }
        if (!wheel && std::find(
                non_wheel_robot_geoms_.begin(),
                non_wheel_robot_geoms_.end(), other) !=
                non_wheel_robot_geoms_.end()) {
            observation.non_wheel_collision = true;
            if (observation.collision_geom == "none") {
                const char *name = mj_id2name(
                    &plant_.model(), mjOBJ_GEOM, other);
                if (name != nullptr) {
                    observation.collision_geom = name;
                } else {
                    const int body = plant_.model().geom_bodyid[other];
                    const char *body_name = mj_id2name(
                        &plant_.model(), mjOBJ_BODY, body);
                    observation.collision_geom = body_name != nullptr ?
                        body_name : "unnamed_robot_geom";
                }
            }
        }
    }

    for (const int robot_geom : non_wheel_robot_geoms_) {
        for (const int terrain_geom : terrain_) {
            const double distance = mj_geomDistance(
                &plant_.model(), &plant_.data(), robot_geom, terrain_geom,
                kDistanceSearchLimit, nullptr);
            if (distance < observation.minimum_clearance) {
                observation.minimum_clearance = distance;
                const char *name = mj_id2name(
                    &plant_.model(), mjOBJ_GEOM, robot_geom);
                if (name != nullptr) {
                    observation.minimum_clearance_geom = name;
                } else {
                    const int body = plant_.model().geom_bodyid[robot_geom];
                    const char *body_name = mj_id2name(
                        &plant_.model(), mjOBJ_BODY, body);
                    observation.minimum_clearance_geom =
                        body_name != nullptr ? body_name :
                            "unnamed_robot_geom";
                }
            }
        }
    }
    return observation;
}

RampCourseResult RampCourseBenchmark::run(const RampCourseSpec &spec) {
    plant_.reset();
    bc_controller_config_t config{};
    bc_controller_default_config(&config);
    config.motion.leg_length = static_cast<float>(spec.leg_length);
    sim::SimulationRunner runner(plant_, adapter_, config);
    RampCourseScenario scenario(spec, plant_.model());
    scenario.reset(plant_);
    const char *normal_terrain[] = {
        "ramp_course_up", "ramp_course_platform", "ramp_course_down",
    };
    const char *beveled_terrain[] = {
        "ramp_course_beveled_up", "ramp_course_bevel",
        "ramp_course_beveled_platform", "ramp_course_beveled_down",
    };
    terrain_.clear();
    if (spec.beveled_transition) {
        for (const char *name : beveled_terrain) {
            terrain_.push_back(require_id(plant_.model(), mjOBJ_GEOM, name));
        }
    } else {
        for (const char *name : normal_terrain) {
            terrain_.push_back(require_id(plant_.model(), mjOBJ_GEOM, name));
        }
    }

    RampCourseResult result{};
    result.spec = spec;
    result.name = scenario.name();
    CsvWriter trace(output_directory_ / result.name / "trace.csv", {
        "case", "phase", "time", "axle_x", "axle_y", "base_z",
        "target_velocity", "truth_velocity", "pitch", "pitch_rate",
        "roll", "roll_rate", "yaw", "yaw_rate", "yaw_error",
        "leg_length_l", "leg_length_r", "leg_rate_l", "leg_rate_r",
        "support_phase", "support_force_l", "support_force_r",
        "kf_velocity", "wheel_velocity", "kf_nis", "wheel_reliable",
        "kf_reacquisition_active", "forward_mode", "ref_s", "s",
        "ref_ds", "ds", "terrain_wheel_l", "terrain_wheel_r",
        "ground_wheel_l", "ground_wheel_r", "non_wheel_collision",
        "collision_geom", "minimum_clearance", "clearance_geom",
    });

    bool previous_any_wheel = true;
    double airborne_start = -1.0;
    std::array<double, BC_SIDE_NUM> slope_contact_time{{-1.0, -1.0}};
    std::array<double, BC_SIDE_NUM> landing_time{{-1.0, -1.0}};
    bool wheel_was_reliable = false;
    double wheel_loss_time = -1.0;
    while (!scenario.finished()) {
        scenario.step(plant_, runner, sampler_);
        const SimulationSample sample = sampler_.read(
            plant_.data(), runner.snapshot());
        const TerrainObservation terrain = observe_terrain();
        const double velocity = truth_axle_velocity(sample);
        const double yaw_error = std::abs(static_cast<double>(
            sample.controller.state.value[BC_STATE_PSI]));
        write_trace(
            trace, scenario, sample, terrain, yaw_error, velocity);

        result.finite = result.finite &&
            controller_snapshot_is_finite(sample.controller);
        result.balance_engaged = result.balance_engaged ||
            scenario.balance_engaged();
        result.start_ready = result.start_ready || scenario.start_ready();
        if (scenario.start_ready()) {
            result.maximum_pitch = std::max(
                result.maximum_pitch, std::abs(static_cast<double>(
                    sample.controller.state.value[BC_STATE_THETA_B])));
            result.maximum_roll = std::max(
                result.maximum_roll,
                std::abs(static_cast<double>(sample.controller.roll)));
            result.maximum_yaw_error = std::max(
                result.maximum_yaw_error, yaw_error);
            result.diverged = result.diverged ||
                result.maximum_pitch > kDivergenceAngle ||
                result.maximum_roll > kDivergenceAngle;
        }

        if (scenario.start_ready() &&
            terrain.minimum_clearance < result.minimum_clearance) {
            result.minimum_clearance = terrain.minimum_clearance;
            result.minimum_clearance_geom = terrain.minimum_clearance_geom;
            result.minimum_clearance_x = sample.axle.x;
        }
        if (scenario.start_ready() && !result.non_wheel_collision &&
            terrain.non_wheel_collision) {
            result.non_wheel_collision = true;
            result.first_collision_geom = terrain.collision_geom;
            result.first_collision_phase = scenario.phase_name();
            result.collision_x = sample.axle.x;
            result.collision_pitch =
                sample.controller.state.value[BC_STATE_THETA_B];
        }

        if (scenario.start_ready()) {
            const bool any_wheel =
                terrain.wheel[BC_L] || terrain.wheel[BC_R] ||
                sample.contact.wheel[BC_L] || sample.contact.wheel[BC_R];
            if (previous_any_wheel && !any_wheel) {
                ++result.airborne_event_count;
                airborne_start = sample.time;
                landing_time = {{-1.0, -1.0}};
                if (!std::isfinite(result.first_airborne_time)) {
                    result.first_airborne_time = sample.time;
                }
            }
            if (!previous_any_wheel && any_wheel) {
                const double duration = sample.time - airborne_start;
                result.maximum_airborne_duration = std::max(
                    result.maximum_airborne_duration, duration);
                if (!std::isfinite(result.first_landing_time)) {
                    result.first_landing_time = sample.time;
                }
            }
            if (airborne_start >= 0.0) {
                for (int side = 0; side < BC_SIDE_NUM; ++side) {
                    const bool contact = terrain.wheel[side] ||
                        sample.contact.wheel[side];
                    if (contact && landing_time[side] < 0.0) {
                        landing_time[side] = sample.time;
                    }
                }
                if (!std::isfinite(
                        result.first_landing_wheel_time_difference) &&
                    landing_time[BC_L] >= 0.0 &&
                    landing_time[BC_R] >= 0.0) {
                    result.first_landing_wheel_time_difference = std::abs(
                        landing_time[BC_L] - landing_time[BC_R]);
                }
            }
            previous_any_wheel = any_wheel;

            const int target_surface = spec.mode == RampCourseMode::descent ?
                static_cast<int>(terrain_.size()) - 1 : 0;
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                bool target_contact = false;
                for (int contact_index = 0;
                     contact_index < plant_.data().ncon; ++contact_index) {
                    const mjContact &contact =
                        plant_.data().contact[contact_index];
                    target_contact = target_contact ||
                        ((contact.geom[0] == terrain_[target_surface] &&
                          contact.geom[1] == wheel_[side]) ||
                         (contact.geom[1] == terrain_[target_surface] &&
                          contact.geom[0] == wheel_[side]));
                }
                if (target_contact && slope_contact_time[side] < 0.0) {
                    slope_contact_time[side] = sample.time;
                }
            }
            if (!std::isfinite(result.ramp_wheel_contact_time_difference) &&
                slope_contact_time[BC_L] >= 0.0 &&
                slope_contact_time[BC_R] >= 0.0) {
                result.ramp_wheel_contact_time_difference = std::abs(
                    slope_contact_time[BC_L] - slope_contact_time[BC_R]);
            }
        }

        const bool reliable =
            sample.controller.velocity_estimator.wheel_velocity_reliable;
        if (scenario.start_ready() && wheel_was_reliable && !reliable) {
            result.wheel_velocity_lost = true;
            result.wheel_velocity_recovered = false;
            wheel_loss_time = sample.time;
        } else if (result.wheel_velocity_lost && !wheel_was_reliable &&
                   reliable) {
            result.wheel_velocity_recovered = true;
            result.wheel_velocity_recovery_seconds =
                sample.time - wheel_loss_time;
        }
        wheel_was_reliable = reliable;

        for (int side = 0; scenario.start_ready() && side < BC_SIDE_NUM;
             ++side) {
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
        result.final_truth_velocity = velocity;
        result.final_estimated_velocity =
            sample.controller.state.value[BC_STATE_DS];
        result.final_position_error =
            sample.controller.state_reference.value[BC_STATE_S] -
            sample.controller.state.value[BC_STATE_S];
        result.forward_hold_recovered =
            sample.controller.state_machine.forward == BC_FORWARD_HOLD;
    }
    trace.flush();
    result.completed = scenario.phase() == RampCoursePhase::complete;
    result.issue = scenario.issue();
    write_summary(result);
    return result;
}

void RampCourseBenchmark::write_summary(const RampCourseResult &result) {
    summary_.begin_row();
    summary_.value(result.name)
        .value(mode_name(result.spec.mode))
        .value(result.spec.leg_length)
        .value(result.spec.target_velocity)
        .value(result.completed)
        .value(result.finite)
        .value(result.balance_engaged)
        .value(result.start_ready)
        .value(result.issue)
        .value(result.non_wheel_collision)
        .value(result.first_collision_geom)
        .value(result.first_collision_phase)
        .value(result.collision_x)
        .value(result.collision_pitch * 180.0 / BC_PI)
        .value(result.minimum_clearance)
        .value(result.minimum_clearance_geom)
        .value(result.minimum_clearance_x)
        .value(result.maximum_pitch * 180.0 / BC_PI)
        .value(result.maximum_roll * 180.0 / BC_PI)
        .value(result.maximum_yaw_error * 180.0 / BC_PI)
        .value(result.ramp_wheel_contact_time_difference)
        .value(result.airborne_event_count)
        .value(result.maximum_airborne_duration)
        .value(result.first_airborne_time)
        .value(result.first_landing_time)
        .value(result.first_landing_wheel_time_difference)
        .value(result.wheel_velocity_lost)
        .value(result.wheel_velocity_recovered)
        .value(result.wheel_velocity_recovery_seconds)
        .value(result.forward_hold_recovered)
        .value(result.final_truth_velocity)
        .value(result.final_estimated_velocity)
        .value(result.final_position_error)
        .value(result.maximum_wheel_torque_request)
        .value(result.maximum_joint_torque_request)
        .value(result.wheel_saturated)
        .value(result.joint_saturated)
        .value(result.diverged);
    summary_.end_row();
    summary_.flush();
}

void RampCourseBenchmark::write_trace(
    CsvWriter &trace,
    const RampCourseScenario &scenario,
    const SimulationSample &sample,
    const TerrainObservation &terrain,
    const double yaw_error,
    const double truth_velocity
) const {
    const auto &snapshot = sample.controller;
    trace.begin_row();
    trace.value(scenario.name())
        .value(scenario.phase_name())
        .value(sample.time)
        .value(sample.axle.x)
        .value(sample.axle.y)
        .value(sample.base.z)
        .value(scenario.commanded_velocity())
        .value(truth_velocity)
        .value(snapshot.state.value[BC_STATE_THETA_B])
        .value(snapshot.state.value[BC_STATE_DTHETA_B])
        .value(snapshot.roll)
        .value(snapshot.roll_rate)
        .value(snapshot.state.value[BC_STATE_PSI])
        .value(snapshot.state.value[BC_STATE_DPSI])
        .value(yaw_error)
        .value(snapshot.leg[BC_L].length)
        .value(snapshot.leg[BC_R].length)
        .value(snapshot.leg[BC_L].length_velocity)
        .value(snapshot.leg[BC_R].length_velocity)
        .value(bc_support_phase_state_name(snapshot.state_machine.support))
        .value(snapshot.support_force[BC_L].filtered_vertical_force)
        .value(snapshot.support_force[BC_R].filtered_vertical_force)
        .value(snapshot.velocity_estimator.velocity_x)
        .value(snapshot.forward_velocity.wheel_odometry)
        .value(snapshot.velocity_estimator.nis)
        .value(static_cast<int>(
            snapshot.velocity_estimator.wheel_velocity_reliable))
        .value(static_cast<int>(
            snapshot.velocity_estimator.reacquisition_active))
        .value(bc_forward_state_name(snapshot.state_machine.forward))
        .value(snapshot.state_reference.value[BC_STATE_S])
        .value(snapshot.state.value[BC_STATE_S])
        .value(snapshot.state_reference.value[BC_STATE_DS])
        .value(snapshot.state.value[BC_STATE_DS])
        .value(terrain.wheel[BC_L])
        .value(terrain.wheel[BC_R])
        .value(sample.contact.wheel[BC_L])
        .value(sample.contact.wheel[BC_R])
        .value(terrain.non_wheel_collision)
        .value(terrain.collision_geom)
        .value(terrain.minimum_clearance)
        .value(terrain.minimum_clearance_geom);
    trace.end_row();
}

} // namespace balance::benchmark
