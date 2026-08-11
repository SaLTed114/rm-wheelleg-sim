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

// This fixed geometry closes after two mirrored 270-degree turns. The entry
// speed makes the 0.35 m curvature transitions use exactly 10 rad/s^2.
constexpr double kFigureEightCurvature = 0.5 * BC_PI;
constexpr double kFigureEightTransitionLength = 0.35;
constexpr double kFigureEightCircleLength = 2.65;
constexpr double kFigureEightStraightLength = 0.9401112857459557;
constexpr double kFigureEightEntryVelocity = 1.4927053303604616;
constexpr double kFigureEightEntryYawRate = 2.3447360499173757;
constexpr double kFigureEightMaximumVelocity = 3.0;
constexpr double kFigureEightCircleDecelerationDistance =
    (kFigureEightMaximumVelocity * kFigureEightMaximumVelocity -
     kFigureEightEntryVelocity * kFigureEightEntryVelocity) /
    (2.0 * kForwardRateLimit);

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

constexpr std::array<PerformanceCaseSpec, 1> kMotionCases{{
    {
        "figure_eight_cross", PerformanceAxis::combined, 0.0,
        kDefaultGimbalAcceleration, 0.0, 2.0, 2.0, 0.0, 0.0,
        PerformanceCaseKind::cross_figure_eight,
    },
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

const std::array<PerformanceCaseSpec, 1> &motion_cases() noexcept {
    return kMotionCases;
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

    const auto motion_found = std::find_if(
        kMotionCases.begin(), kMotionCases.end(),
        [name](const PerformanceCaseSpec &spec) {
            return spec.name == name;
        });
    if (motion_found != kMotionCases.end()) return &*motion_found;

    return nullptr;
}

const char *performance_axis_name(const PerformanceAxis axis) noexcept {
    switch (axis) {
    case PerformanceAxis::forward: return "forward";
    case PerformanceAxis::heading: return "heading";
    case PerformanceAxis::combined: return "combined";
    }
    return "unknown";
}

const char *performance_phase_name(const PerformancePhase phase) noexcept {
    switch (phase) {
    case PerformancePhase::disabled_settle: return "disabled_settle";
    case PerformancePhase::engaging: return "engaging";
    case PerformancePhase::standing: return "standing";
    case PerformancePhase::target_ramp: return "target_ramp";
    case PerformancePhase::target_hold: return "target_hold";
    case PerformancePhase::trajectory: return "trajectory";
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
    const bool axis_response =
        spec_.kind == PerformanceCaseKind::axis_response;
    if ((axis_response && spec_.axis == PerformanceAxis::combined) ||
        (!axis_response && spec_.axis != PerformanceAxis::combined) ||
        !std::isfinite(spec_.target) ||
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
        (axis_response && spec_.axis != PerformanceAxis::heading &&
         (spec_.coupled_forward_velocity != 0.0 ||
          spec_.forward_lead_seconds != 0.0)) ||
        (spec_.coupled_forward_velocity == 0.0 &&
         spec_.forward_lead_seconds != 0.0) ||
        (axis_response && spec_.axis == PerformanceAxis::heading &&
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
    figure_eight_phase_ = FigureEightPhase::lead_in;
    figure_eight_phase_distance_ = 0.0;
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

void PerformanceScenario::enter_figure_eight(
    const FigureEightPhase phase
) noexcept {
    figure_eight_phase_ = phase;
    figure_eight_phase_distance_ = 0.0;
}

const char *PerformanceScenario::phase_name() const noexcept {
    if (phase_ != PerformancePhase::trajectory) {
        return performance_phase_name(phase_);
    }
    switch (figure_eight_phase_) {
    case FigureEightPhase::lead_in: return "figure_eight_lead_in";
    case FigureEightPhase::straight_one:
        return "figure_eight_straight_one";
    case FigureEightPhase::left_entry: return "figure_eight_left_entry";
    case FigureEightPhase::left_arc: return "figure_eight_left_arc";
    case FigureEightPhase::left_exit: return "figure_eight_left_exit";
    case FigureEightPhase::straight_two:
        return "figure_eight_straight_two";
    case FigureEightPhase::right_entry: return "figure_eight_right_entry";
    case FigureEightPhase::right_arc: return "figure_eight_right_arc";
    case FigureEightPhase::right_exit: return "figure_eight_right_exit";
    }
    return "figure_eight_unknown";
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
            if (spec_.kind == PerformanceCaseKind::cross_figure_eight) {
                enter_figure_eight(FigureEightPhase::lead_in);
                enter(PerformancePhase::trajectory, simulation_time);
            } else {
                enter(PerformancePhase::target_ramp, simulation_time);
            }
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

    case PerformancePhase::trajectory:
        advance_figure_eight(snapshot, timestep_seconds);
        break;

    case PerformancePhase::stop_ramp: {
        const bool stopped_figure_eight =
            spec_.kind == PerformanceCaseKind::cross_figure_eight &&
            std::abs(snapshot.state_reference.value[BC_STATE_DS]) <=
                1.0e-4F;
        const bool stopped_axis_response =
            spec_.kind == PerformanceCaseKind::axis_response &&
            phase_elapsed() >= std::abs(spec_.target) / spec_.command_rate;
        if (stopped_figure_eight || stopped_axis_response) {
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
    if (phase_ == PerformancePhase::trajectory) {
        update_figure_eight_command(
            snapshot, gimbal_forward_velocity, target_gimbal_yaw_rate);
    }
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
        phase_ == PerformancePhase::trajectory ||
        phase_ == PerformancePhase::stop_ramp ||
        phase_ == PerformancePhase::stop_settle;
}

void PerformanceScenario::advance_figure_eight(
    const bc_controller_snapshot_t &snapshot,
    const float timestep_seconds
) noexcept {
    const double velocity = std::max(
        0.0, static_cast<double>(
            snapshot.state_reference.value[BC_STATE_DS]));

    if (figure_eight_phase_ == FigureEightPhase::lead_in) {
        if (velocity >= kFigureEightEntryVelocity - 1.0e-4) {
            enter_figure_eight(FigureEightPhase::straight_one);
        }
        return;
    }

    figure_eight_phase_distance_ +=
        velocity * static_cast<double>(timestep_seconds);
    const auto reached = [this](const double distance) {
        return figure_eight_phase_distance_ >= distance;
    };

    switch (figure_eight_phase_) {
    case FigureEightPhase::lead_in:
        break;
    case FigureEightPhase::straight_one:
        if (reached(kFigureEightStraightLength)) {
            enter_figure_eight(FigureEightPhase::left_entry);
        }
        break;
    case FigureEightPhase::left_entry:
        if (reached(kFigureEightTransitionLength)) {
            enter_figure_eight(FigureEightPhase::left_arc);
        }
        break;
    case FigureEightPhase::left_arc:
        if (reached(kFigureEightCircleLength)) {
            enter_figure_eight(FigureEightPhase::left_exit);
        }
        break;
    case FigureEightPhase::left_exit:
        if (reached(kFigureEightTransitionLength)) {
            enter_figure_eight(FigureEightPhase::straight_two);
        }
        break;
    case FigureEightPhase::straight_two:
        if (reached(kFigureEightStraightLength)) {
            enter_figure_eight(FigureEightPhase::right_entry);
        }
        break;
    case FigureEightPhase::right_entry:
        if (reached(kFigureEightTransitionLength)) {
            enter_figure_eight(FigureEightPhase::right_arc);
        }
        break;
    case FigureEightPhase::right_arc:
        if (reached(kFigureEightCircleLength)) {
            enter_figure_eight(FigureEightPhase::right_exit);
        }
        break;
    case FigureEightPhase::right_exit:
        if (reached(kFigureEightTransitionLength)) {
            enter(PerformancePhase::stop_ramp, simulation_time_);
        }
        break;
    }
}

void PerformanceScenario::update_figure_eight_command(
    const bc_controller_snapshot_t &snapshot,
    float &forward_velocity, float &target_yaw_rate
) const noexcept {
    forward_velocity = static_cast<float>(kFigureEightEntryVelocity);
    const float planned_velocity = std::max(
        0.0F, snapshot.state_reference.value[BC_STATE_DS]);

    switch (figure_eight_phase_) {
    case FigureEightPhase::lead_in:
    case FigureEightPhase::straight_one:
    case FigureEightPhase::straight_two:
        target_yaw_rate = 0.0F;
        break;
    case FigureEightPhase::left_entry:
        target_yaw_rate = static_cast<float>(kFigureEightEntryYawRate);
        break;
    case FigureEightPhase::left_arc:
        forward_velocity = static_cast<float>(
            figure_eight_phase_distance_ <
                    kFigureEightCircleLength -
                        kFigureEightCircleDecelerationDistance ?
                kFigureEightMaximumVelocity : kFigureEightEntryVelocity);
        target_yaw_rate = static_cast<float>(
            kFigureEightCurvature * planned_velocity);
        break;
    case FigureEightPhase::left_exit:
        target_yaw_rate = 0.0F;
        break;
    case FigureEightPhase::right_entry:
        target_yaw_rate = static_cast<float>(-kFigureEightEntryYawRate);
        break;
    case FigureEightPhase::right_arc:
        forward_velocity = static_cast<float>(
            figure_eight_phase_distance_ <
                    kFigureEightCircleLength -
                        kFigureEightCircleDecelerationDistance ?
                kFigureEightMaximumVelocity : kFigureEightEntryVelocity);
        target_yaw_rate = static_cast<float>(
            -kFigureEightCurvature * planned_velocity);
        break;
    case FigureEightPhase::right_exit:
        target_yaw_rate = 0.0F;
        break;
    }
}

bool PerformanceScenario::tracking_evaluation() const noexcept {
    if (spec_.kind == PerformanceCaseKind::cross_figure_eight) {
        return phase_ == PerformancePhase::trajectory;
    }
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
