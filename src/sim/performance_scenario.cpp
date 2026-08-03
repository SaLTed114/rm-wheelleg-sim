#include "performance_scenario.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "balance/math_utils.h"

namespace balance::sim {
namespace {

constexpr double kDisabledSettleSeconds = 2.0;
constexpr double kEngagementTimeoutSeconds = 8.0;
constexpr double kStandingSeconds = 2.0;
constexpr double kTargetHoldSeconds = 3.0;
constexpr double kStopSettleSeconds = 2.0;
constexpr double kEvaluationSeconds = 1.0;
constexpr double kForwardRateLimit = 5.0;
constexpr double kYawRateLimit = 15.0;
constexpr double kTerminationAngle = 45.0 * BC_PI / 180.0;

constexpr std::array<PerformanceCaseSpec, 16> kCases{{
    {"forward_pos_1", PerformanceAxis::forward, 1.0, 5.0},
    {"forward_neg_1", PerformanceAxis::forward, -1.0, 5.0},
    {"forward_pos_2", PerformanceAxis::forward, 2.0, 5.0},
    {"forward_neg_2", PerformanceAxis::forward, -2.0, 5.0},
    {"forward_pos_2p5", PerformanceAxis::forward, 2.5, 5.0},
    {"forward_neg_2p5", PerformanceAxis::forward, -2.5, 5.0},
    {"forward_pos_3", PerformanceAxis::forward, 3.0, 5.0},
    {"forward_neg_3", PerformanceAxis::forward, -3.0, 5.0},
    {"yaw_pos_1pi", PerformanceAxis::yaw, BC_PI, 15.0},
    {"yaw_neg_1pi", PerformanceAxis::yaw, -BC_PI, 15.0},
    {"yaw_pos_2pi", PerformanceAxis::yaw, 2.0 * BC_PI, 15.0},
    {"yaw_neg_2pi", PerformanceAxis::yaw, -2.0 * BC_PI, 15.0},
    {"yaw_pos_3pi", PerformanceAxis::yaw, 3.0 * BC_PI, 15.0},
    {"yaw_neg_3pi", PerformanceAxis::yaw, -3.0 * BC_PI, 15.0},
    {"yaw_pos_4pi", PerformanceAxis::yaw, 4.0 * BC_PI, 15.0},
    {"yaw_neg_4pi", PerformanceAxis::yaw, -4.0 * BC_PI, 15.0},
}};

constexpr std::array<PerformanceCaseSpec, 10> kForwardAccelerationCases{{
    {"forward_pos_2_a0p5", PerformanceAxis::forward, 2.0, 0.5},
    {"forward_neg_2_a0p5", PerformanceAxis::forward, -2.0, 0.5},
    {"forward_pos_2_a1", PerformanceAxis::forward, 2.0, 1.0},
    {"forward_neg_2_a1", PerformanceAxis::forward, -2.0, 1.0},
    {"forward_pos_2_a2", PerformanceAxis::forward, 2.0, 2.0},
    {"forward_neg_2_a2", PerformanceAxis::forward, -2.0, 2.0},
    {"forward_pos_2_a3", PerformanceAxis::forward, 2.0, 3.0},
    {"forward_neg_2_a3", PerformanceAxis::forward, -2.0, 3.0},
    {"forward_pos_2_a5", PerformanceAxis::forward, 2.0, 5.0},
    {"forward_neg_2_a5", PerformanceAxis::forward, -2.0, 5.0},
}};

constexpr std::array<PerformanceCaseSpec, 14> kYawAccelerationCases{{
    {"yaw_pos_2pi_a1", PerformanceAxis::yaw, 2.0 * BC_PI, 1.0},
    {"yaw_neg_2pi_a1", PerformanceAxis::yaw, -2.0 * BC_PI, 1.0},
    {"yaw_pos_2pi_a2", PerformanceAxis::yaw, 2.0 * BC_PI, 2.0},
    {"yaw_neg_2pi_a2", PerformanceAxis::yaw, -2.0 * BC_PI, 2.0},
    {"yaw_pos_2pi_a3", PerformanceAxis::yaw, 2.0 * BC_PI, 3.0},
    {"yaw_neg_2pi_a3", PerformanceAxis::yaw, -2.0 * BC_PI, 3.0},
    {"yaw_pos_2pi_a5", PerformanceAxis::yaw, 2.0 * BC_PI, 5.0},
    {"yaw_neg_2pi_a5", PerformanceAxis::yaw, -2.0 * BC_PI, 5.0},
    {"yaw_pos_2pi_a7p5", PerformanceAxis::yaw, 2.0 * BC_PI, 7.5},
    {"yaw_neg_2pi_a7p5", PerformanceAxis::yaw, -2.0 * BC_PI, 7.5},
    {"yaw_pos_2pi_a10", PerformanceAxis::yaw, 2.0 * BC_PI, 10.0},
    {"yaw_neg_2pi_a10", PerformanceAxis::yaw, -2.0 * BC_PI, 10.0},
    {"yaw_pos_2pi_a15", PerformanceAxis::yaw, 2.0 * BC_PI, 15.0},
    {"yaw_neg_2pi_a15", PerformanceAxis::yaw, -2.0 * BC_PI, 15.0},
}};

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

bool finite_actuation(const bc_actuation_t &actuation) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (!std::isfinite(actuation.wheel_torque[side])) return false;
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            if (!std::isfinite(
                    actuation.leg[side].joint_torque[joint])) {
                return false;
            }
        }
    }
    return true;
}

} // namespace

const std::array<PerformanceCaseSpec, 16> &
performance_cases() noexcept {
    return kCases;
}

const std::array<PerformanceCaseSpec, 10> &
forward_acceleration_cases() noexcept {
    return kForwardAccelerationCases;
}

const std::array<PerformanceCaseSpec, 14> &
yaw_acceleration_cases() noexcept {
    return kYawAccelerationCases;
}

const PerformanceCaseSpec *find_performance_case(
    const std::string_view name
) noexcept {
    const auto found = std::find_if(
        kCases.begin(), kCases.end(),
        [name](const PerformanceCaseSpec &spec) {
            return spec.name == name;
        });
    if (found != kCases.end()) return &*found;

    const auto acceleration_found = std::find_if(
        kForwardAccelerationCases.begin(),
        kForwardAccelerationCases.end(),
        [name](const PerformanceCaseSpec &spec) {
            return spec.name == name;
        });
    if (acceleration_found != kForwardAccelerationCases.end()) {
        return &*acceleration_found;
    }

    const auto yaw_acceleration_found = std::find_if(
        kYawAccelerationCases.begin(), kYawAccelerationCases.end(),
        [name](const PerformanceCaseSpec &spec) {
            return spec.name == name;
        });
    return yaw_acceleration_found == kYawAccelerationCases.end() ?
        nullptr : &*yaw_acceleration_found;
}

const char *performance_axis_name(const PerformanceAxis axis) noexcept {
    return axis == PerformanceAxis::forward ? "forward" : "yaw";
}

const char *performance_phase_name(const PerformancePhase phase) noexcept {
    switch (phase) {
    case PerformancePhase::disabled_settle: return "disabled_settle";
    case PerformancePhase::engaging: return "engaging";
    case PerformancePhase::standing: return "standing";
    case PerformancePhase::target_ramp: return "target_ramp";
    case PerformancePhase::target_hold: return "target_hold";
    case PerformancePhase::stop_ramp: return "stop_ramp";
    case PerformancePhase::stop_settle: return "stop_settle";
    case PerformancePhase::complete: return "complete";
    }
    return "unknown";
}

PerformanceScenario::PerformanceScenario(
    const PerformanceCaseSpec &spec
) : spec_(spec) {
    if (!std::isfinite(spec_.target) ||
        !std::isfinite(spec_.command_rate) ||
        spec_.command_rate <= 0.0) {
        throw std::invalid_argument(
            "performance case target and command rate must be valid");
    }
    reset();
}

void PerformanceScenario::reset(const double simulation_time) noexcept {
    phase_ = PerformancePhase::disabled_settle;
    command_ = {};
    phase_start_time_ = simulation_time;
    simulation_time_ = simulation_time;
}

void PerformanceScenario::enter(
    const PerformancePhase phase, const double simulation_time
) noexcept {
    phase_ = phase;
    phase_start_time_ = simulation_time;
}

double PerformanceScenario::phase_elapsed() const noexcept {
    return simulation_time_ - phase_start_time_;
}

void PerformanceScenario::update(
    const bc_controller_snapshot_t &snapshot,
    const double simulation_time
) noexcept {
    simulation_time_ = simulation_time;

    switch (phase_) {
    case PerformancePhase::disabled_settle:
        if (phase_elapsed() >= kDisabledSettleSeconds) {
            enter(PerformancePhase::engaging, simulation_time);
        }
        break;

    case PerformancePhase::engaging:
        if (snapshot.state_machine.motion == BC_MOTION_BALANCE_ENGAGING ||
            phase_elapsed() >= kEngagementTimeoutSeconds) {
            enter(PerformancePhase::standing, simulation_time);
        }
        break;

    case PerformancePhase::standing:
        if (phase_elapsed() >= kStandingSeconds) {
            enter(PerformancePhase::target_ramp, simulation_time);
        }
        break;

    case PerformancePhase::target_ramp: {
        if (phase_elapsed() >=
            std::abs(spec_.target) / spec_.command_rate) {
            enter(PerformancePhase::target_hold, simulation_time);
        }
        break;
    }

    case PerformancePhase::target_hold:
        if (phase_elapsed() >= kTargetHoldSeconds) {
            enter(PerformancePhase::stop_ramp, simulation_time);
        }
        break;

    case PerformancePhase::stop_ramp: {
        if (phase_elapsed() >=
            std::abs(spec_.target) / spec_.command_rate) {
            enter(PerformancePhase::stop_settle, simulation_time);
        }
        break;
    }

    case PerformancePhase::stop_settle:
        if (phase_elapsed() >= kStopSettleSeconds) {
            enter(PerformancePhase::complete, simulation_time);
        }
        break;

    case PerformancePhase::complete:
        break;
    }

    command_ = {};
    if (phase_ == PerformancePhase::disabled_settle || finished()) return;

    command_.system_enabled = 1U;
    if (phase_ == PerformancePhase::engaging) {
        command_.balance_restart = static_cast<uint8_t>(
            snapshot.state_machine.system == BC_SYSTEM_OFF);
    }
    if (phase_ == PerformancePhase::target_ramp ||
        phase_ == PerformancePhase::target_hold) {
        double target = spec_.target;
        const double controller_rate_limit =
            spec_.axis == PerformanceAxis::forward ?
                kForwardRateLimit : kYawRateLimit;
        if (phase_ == PerformancePhase::target_ramp &&
            spec_.command_rate < controller_rate_limit) {
            target = std::copysign(
                std::min(
                    std::abs(spec_.target),
                    spec_.command_rate * phase_elapsed()),
                spec_.target);
        }
        if (spec_.axis == PerformanceAxis::forward) {
            command_.forward_velocity = static_cast<float>(target);
        } else {
            command_.yaw_rate = static_cast<float>(target);
        }
    } else if (phase_ == PerformancePhase::stop_ramp &&
               spec_.command_rate <
                   (spec_.axis == PerformanceAxis::forward ?
                       kForwardRateLimit : kYawRateLimit)) {
        const double target = std::copysign(
            std::max(
                0.0,
                std::abs(spec_.target) -
                    spec_.command_rate * phase_elapsed()),
            spec_.target);
        if (spec_.axis == PerformanceAxis::forward) {
            command_.forward_velocity = static_cast<float>(target);
        } else {
            command_.yaw_rate = static_cast<float>(target);
        }
    }
}

bool PerformanceScenario::monitored() const noexcept {
    return phase_ == PerformancePhase::target_ramp ||
        phase_ == PerformancePhase::target_hold ||
        phase_ == PerformancePhase::stop_ramp ||
        phase_ == PerformancePhase::stop_settle;
}

bool PerformanceScenario::tracking_evaluation() const noexcept {
    return phase_ == PerformancePhase::target_hold &&
        phase_elapsed() >= kTargetHoldSeconds - kEvaluationSeconds;
}

bool PerformanceScenario::settle_evaluation() const noexcept {
    return phase_ == PerformancePhase::stop_settle &&
        phase_elapsed() >= kStopSettleSeconds - kEvaluationSeconds;
}

bool PerformanceScenario::finished() const noexcept {
    return phase_ == PerformancePhase::complete;
}

PerformanceContactMonitor::PerformanceContactMonitor(const mjModel &model)
    : model_(model) {
    ground_ = require_id(model_, mjOBJ_GEOM, "ground");
    wheel_ = {{
        require_id(model_, mjOBJ_GEOM, "Right_wheel_collision"),
        require_id(model_, mjOBJ_GEOM, "Left_wheel_collision"),
    }};
}

PerformanceContactState PerformanceContactMonitor::read(
    const mjData &data
) const {
    PerformanceContactState state{};
    for (int index = 0; index < data.ncon; ++index) {
        const mjContact &contact = data.contact[index];
        const bool has_ground =
            contact.geom[0] == ground_ || contact.geom[1] == ground_;
        if (!has_ground) {
            state.other = true;
            if (state.unexpected.empty()) {
                state.unexpected = contact_name(contact);
            }
            continue;
        }

        bool wheel_contact = false;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            const bool pair =
                (contact.geom[0] == ground_ &&
                 contact.geom[1] == wheel_[side]) ||
                (contact.geom[1] == ground_ &&
                 contact.geom[0] == wheel_[side]);
            state.wheel[side] = state.wheel[side] || pair;
            if (pair) {
                mjtNum force[6] = {};
                mj_contactForce(&model_, &data, index, force);
                state.wheel_normal_force[side] += std::max(0.0, force[0]);
            }
            wheel_contact = wheel_contact || pair;
        }
        state.other = state.other || !wheel_contact;
        if (!wheel_contact && state.unexpected.empty()) {
            state.unexpected = contact_name(contact);
        }
    }
    return state;
}

std::string PerformanceContactMonitor::contact_name(
    const mjContact &contact
) const {
    std::string description;
    for (int pair = 0; pair < 2; ++pair) {
        if (!description.empty()) description += '+';
        const char *name = mj_id2name(
            &model_, mjOBJ_GEOM, contact.geom[pair]);
        if (name != nullptr) {
            description += name;
        } else {
            const int body = model_.geom_bodyid[contact.geom[pair]];
            const char *body_name = mj_id2name(
                &model_, mjOBJ_BODY, body);
            description += body_name != nullptr ? body_name :
                "geom_" + std::to_string(contact.geom[pair]);
        }
    }
    return description;
}

std::string performance_diagnostic_issue(
    const bc_controller_snapshot_t &snapshot,
    const PerformanceContactState &contact
) {
    bool finite = std::isfinite(snapshot.roll) &&
        std::isfinite(snapshot.roll_rate) &&
        finite_actuation(snapshot.actuation_request) &&
        finite_actuation(snapshot.actuation);
    for (int index = 0; index < BC_STATE_NUM; ++index) {
        finite = finite && std::isfinite(snapshot.state.value[index]) &&
            std::isfinite(snapshot.state_reference.value[index]);
    }
    if (!finite) return "non_finite_telemetry";
    if (contact.other) {
        return "non_wheel_contact:" + contact.unexpected;
    }

    const double pitch = std::abs(static_cast<double>(
        snapshot.state.value[BC_STATE_THETA_B]));
    const double roll = std::abs(static_cast<double>(snapshot.roll));
    if (pitch > kTerminationAngle || roll > kTerminationAngle) {
        return "attitude_termination";
    }
    return {};
}

} // namespace balance::sim
