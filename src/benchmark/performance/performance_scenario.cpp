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
constexpr double kDefaultGimbalAcceleration = 10.0;

constexpr std::array<PerformanceCaseSpec, 12> kCases{{
    {"forward_pos_1", PerformanceAxis::forward, 1.0, 5.0},
    {"forward_neg_1", PerformanceAxis::forward, -1.0, 5.0},
    {"forward_pos_2", PerformanceAxis::forward, 2.0, 5.0},
    {"forward_neg_2", PerformanceAxis::forward, -2.0, 5.0},
    {"forward_pos_2p5", PerformanceAxis::forward, 2.5, 5.0},
    {"forward_neg_2p5", PerformanceAxis::forward, -2.5, 5.0},
    {"forward_pos_3", PerformanceAxis::forward, 3.0, 5.0},
    {"forward_neg_3", PerformanceAxis::forward, -3.0, 5.0},
    {"heading_pos_1pi", PerformanceAxis::heading, BC_PI, 10.0},
    {"heading_neg_1pi", PerformanceAxis::heading, -BC_PI, 10.0},
    {"heading_pos_1p5pi", PerformanceAxis::heading, 1.5 * BC_PI, 10.0},
    {"heading_neg_1p5pi", PerformanceAxis::heading, -1.5 * BC_PI, 10.0},
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

} // namespace

const std::array<PerformanceCaseSpec, 12> &
performance_cases() noexcept {
    return kCases;
}

const std::array<PerformanceCaseSpec, 10> &
forward_acceleration_cases() noexcept {
    return kForwardAccelerationCases;
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

    return nullptr;
}

const char *performance_axis_name(const PerformanceAxis axis) noexcept {
    return axis == PerformanceAxis::forward ? "forward" : "heading";
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
) : spec_(spec),
    virtual_gimbal_(sim::VirtualGimbalConfig{
        std::max(
            1.5F * BC_PI_F,
            static_cast<float>(std::abs(spec.target))),
        static_cast<float>(
            spec.axis == PerformanceAxis::heading ?
                spec.command_rate : kDefaultGimbalAcceleration),
    }) {
    if (!std::isfinite(spec_.target) ||
        !std::isfinite(spec_.command_rate) ||
        spec_.command_rate <= 0.0 ||
        !std::isfinite(spec_.target_hold_seconds) ||
        spec_.target_hold_seconds < 0.0 ||
        !std::isfinite(spec_.stop_settle_seconds) ||
        spec_.stop_settle_seconds < 0.0 ||
        !std::isfinite(spec_.standing_seconds) ||
        spec_.standing_seconds < 0.0 ||
        !std::isfinite(spec_.coupled_forward_velocity) ||
        std::abs(spec_.coupled_forward_velocity) > 3.0 ||
        !std::isfinite(spec_.forward_lead_seconds) ||
        spec_.forward_lead_seconds < 0.0 ||
        spec_.forward_lead_seconds > spec_.standing_seconds ||
        (spec_.axis != PerformanceAxis::heading &&
         (spec_.coupled_forward_velocity != 0.0 ||
          spec_.forward_lead_seconds != 0.0)) ||
        (spec_.coupled_forward_velocity == 0.0 &&
         spec_.forward_lead_seconds != 0.0) ||
        (spec_.axis == PerformanceAxis::heading &&
         std::abs(spec_.target) > 2.0 * BC_PI)) {
        throw std::invalid_argument(
            "performance case values and durations must be valid");
    }
    reset();
}

void PerformanceScenario::reset(const double simulation_time) noexcept {
    phase_ = PerformancePhase::disabled_settle;
    command_ = {};
    virtual_gimbal_.reset();
    gimbal_initialized_ = false;
    phase_start_time_ = simulation_time;
    simulation_time_ = simulation_time;
    previous_update_time_ = simulation_time;
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
    const float timestep_seconds = static_cast<float>(
        std::max(0.0, simulation_time - previous_update_time_));
    previous_update_time_ = simulation_time;
    simulation_time_ = simulation_time;

    if (snapshot.state_machine.motion == BC_MOTION_ACTIVE) {
        if (!gimbal_initialized_) {
            virtual_gimbal_.reset(
                snapshot.state.value[BC_STATE_PSI]);
            gimbal_initialized_ = true;
        }
    } else {
        virtual_gimbal_.reset(snapshot.state.value[BC_STATE_PSI]);
        gimbal_initialized_ = false;
    }

    switch (phase_) {
    case PerformancePhase::disabled_settle:
        if (phase_elapsed() >= kDisabledSettleSeconds) {
            enter(PerformancePhase::engaging, simulation_time);
        }
        break;

    case PerformancePhase::engaging:
        if (snapshot.state_machine.motion == BC_MOTION_ACTIVE ||
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
    float gimbal_forward_velocity = 0.0F;
    float target_gimbal_yaw_rate = 0.0F;
    const bool coupled_forward_active =
        spec_.axis == PerformanceAxis::heading &&
        spec_.coupled_forward_velocity != 0.0 &&
        ((phase_ == PerformancePhase::standing &&
          spec_.forward_lead_seconds > 0.0 &&
          phase_elapsed() >=
              spec_.standing_seconds - spec_.forward_lead_seconds) ||
         phase_ == PerformancePhase::target_ramp ||
         phase_ == PerformancePhase::target_hold ||
         phase_ == PerformancePhase::stop_ramp);
    if (coupled_forward_active) {
        gimbal_forward_velocity =
            static_cast<float>(spec_.coupled_forward_velocity);
    }
    if (phase_ == PerformancePhase::target_ramp ||
        phase_ == PerformancePhase::target_hold) {
        double target = spec_.target;
        if (spec_.axis == PerformanceAxis::forward &&
            phase_ == PerformancePhase::target_ramp &&
            spec_.command_rate < kForwardRateLimit) {
            target = std::copysign(
                std::min(
                    std::abs(spec_.target),
                    spec_.command_rate * phase_elapsed()),
                spec_.target);
        }
        if (spec_.axis == PerformanceAxis::forward) {
            gimbal_forward_velocity = static_cast<float>(target);
        } else {
            target_gimbal_yaw_rate = static_cast<float>(target);
        }
    } else if (phase_ == PerformancePhase::stop_ramp &&
               spec_.axis == PerformanceAxis::forward &&
               spec_.command_rate < kForwardRateLimit) {
        const double target = std::copysign(
            std::max(
                0.0,
                std::abs(spec_.target) -
                    spec_.command_rate * phase_elapsed()),
            spec_.target);
        gimbal_forward_velocity = static_cast<float>(target);
    }

    if (gimbal_initialized_) {
        virtual_gimbal_.update(
            target_gimbal_yaw_rate, timestep_seconds);
        command_.forward_velocity = gimbal_forward_velocity;
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
