#include "performance_scenario.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "balance/math_utils.h"

namespace balance::benchmark {
namespace {

constexpr double kDisabledSettleSeconds = 2.0;
constexpr double kEngagementTimeoutSeconds = 8.0;
constexpr double kEvaluationSeconds = 1.0;
constexpr double kForwardRateLimit = 5.0;
constexpr double kYawRateLimit = 15.0;

constexpr std::array<PerformanceCaseSpec, 16> kCases{{
    {"forward_pos_1", PerformanceAxis::forward, 1.0, 5.0},
    {"forward_neg_1", PerformanceAxis::forward, -1.0, 5.0},
    {"forward_pos_2", PerformanceAxis::forward, 2.0, 5.0},
    {"forward_neg_2", PerformanceAxis::forward, -2.0, 5.0},
    {"forward_pos_2p5", PerformanceAxis::forward, 2.5, 5.0},
    {"forward_neg_2p5", PerformanceAxis::forward, -2.5, 5.0},
    {"forward_pos_3", PerformanceAxis::forward, 3.0, 5.0},
    {"forward_neg_3", PerformanceAxis::forward, -3.0, 5.0},
    {"yaw_pos_1pi", PerformanceAxis::yaw, BC_PI, 5.0},
    {"yaw_neg_1pi", PerformanceAxis::yaw, -BC_PI, 5.0},
    {"yaw_pos_2pi", PerformanceAxis::yaw, 2.0 * BC_PI, 5.0},
    {"yaw_neg_2pi", PerformanceAxis::yaw, -2.0 * BC_PI, 5.0},
    {"yaw_pos_3pi", PerformanceAxis::yaw, 3.0 * BC_PI, 5.0},
    {"yaw_neg_3pi", PerformanceAxis::yaw, -3.0 * BC_PI, 5.0},
    {"yaw_pos_4pi", PerformanceAxis::yaw, 4.0 * BC_PI, 5.0},
    {"yaw_neg_4pi", PerformanceAxis::yaw, -4.0 * BC_PI, 5.0},
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
        spec_.command_rate <= 0.0 ||
        !std::isfinite(spec_.target_hold_seconds) ||
        spec_.target_hold_seconds < 0.0 ||
        !std::isfinite(spec_.stop_settle_seconds) ||
        spec_.stop_settle_seconds < 0.0 ||
        !std::isfinite(spec_.standing_seconds) ||
        spec_.standing_seconds < 0.0) {
        throw std::invalid_argument(
            "performance case values and durations must be valid");
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
        if (phase_elapsed() >= spec_.standing_seconds) {
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
        if (phase_elapsed() >= spec_.target_hold_seconds) {
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
        if (phase_elapsed() >= spec_.stop_settle_seconds) {
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
        phase_elapsed() >= std::max(
            0.0, spec_.target_hold_seconds - kEvaluationSeconds);
}

bool PerformanceScenario::settle_evaluation() const noexcept {
    return phase_ == PerformancePhase::stop_settle &&
        phase_elapsed() >= std::max(
            0.0, spec_.stop_settle_seconds - kEvaluationSeconds);
}

bool PerformanceScenario::finished() const noexcept {
    return phase_ == PerformancePhase::complete;
}

} // namespace balance::benchmark
