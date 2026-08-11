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
constexpr double kEntryStableSeconds = 0.5;
constexpr double kEntryTimeoutSeconds = 5.0;
constexpr double kEntryForwardAbsoluteTolerance = 0.1;
constexpr double kEntryForwardRelativeTolerance = 0.05;
constexpr double kEntryPitchLimit = 3.0 * BC_PI / 180.0;
constexpr double kEntryPitchRateLimit = 5.0 * BC_PI / 180.0;
constexpr double kFigureEightStraightSeconds = 0.350608486431;
constexpr double kFigureEightTurnDriveSeconds = 1.5;
constexpr double kFigureEightTurnExitSeconds = BC_PI / 10.0;

constexpr std::array<PerformanceCaseSpec, 4> kFormalCases{{
    {"forward_pos_3", PerformanceCaseKind::forward_response,
     3.0, 0.0, 5.0, 10.0, 3.0, 2.0, 2.0, 0.0, 0.0, true},
    {"forward_neg_3", PerformanceCaseKind::forward_response,
     -3.0, 0.0, 5.0, 10.0, 3.0, 2.0, 2.0, 0.0, 0.0, true},
    {"heading_pos_1p5pi", PerformanceCaseKind::heading_response,
     0.0, 1.5 * BC_PI, 5.0, 10.0, 3.0, 2.0, 2.0,
     0.0, 0.0, true},
    {"heading_neg_1p5pi", PerformanceCaseKind::heading_response,
     0.0, -1.5 * BC_PI, 5.0, 10.0, 3.0, 2.0, 2.0,
     0.0, 0.0, true},
}};

constexpr std::array<PerformanceCaseSpec, 1> kTrajectoryCases{{
    {"figure_eight_open_loop", PerformanceCaseKind::figure_eight,
     2.0, BC_PI, 5.0, 10.0, 0.0, 3.0, 2.0},
}};

bool finite_non_negative(const double value) {
    return std::isfinite(value) && value >= 0.0;
}

} // namespace

const std::array<PerformanceCaseSpec, 4> &
formal_performance_cases() noexcept {
    return kFormalCases;
}

const std::array<PerformanceCaseSpec, 1> &
trajectory_performance_cases() noexcept {
    return kTrajectoryCases;
}

const PerformanceCaseSpec *find_performance_case(
    const std::string_view name
) noexcept {
    const auto found = std::find_if(
        kFormalCases.begin(), kFormalCases.end(),
        [name](const PerformanceCaseSpec &spec) {
            return spec.name == name;
        });
    if (found != kFormalCases.end()) return &*found;
    const auto trajectory = std::find_if(
        kTrajectoryCases.begin(), kTrajectoryCases.end(),
        [name](const PerformanceCaseSpec &spec) {
            return spec.name == name;
        });
    return trajectory == kTrajectoryCases.end() ? nullptr : &*trajectory;
}

const char *performance_case_kind_name(
    const PerformanceCaseKind kind
) noexcept {
    switch (kind) {
    case PerformanceCaseKind::forward_response: return "forward";
    case PerformanceCaseKind::heading_response: return "heading";
    case PerformanceCaseKind::steady_turn: return "turn";
    case PerformanceCaseKind::figure_eight: return "figure_eight";
    }
    return "unknown";
}

const char *performance_phase_name(const PerformancePhase phase) noexcept {
    switch (phase) {
    case PerformancePhase::disabled_settle: return "disabled_settle";
    case PerformancePhase::engaging: return "engaging";
    case PerformancePhase::standing: return "standing";
    case PerformancePhase::target_ramp: return "target_ramp";
    case PerformancePhase::entry_wait: return "entry_wait";
    case PerformancePhase::yaw_ramp: return "yaw_ramp";
    case PerformancePhase::target_hold: return "target_hold";
    case PerformancePhase::trajectory: return "trajectory";
    case PerformancePhase::yaw_stop_ramp: return "yaw_stop_ramp";
    case PerformancePhase::forward_stop_ramp: return "forward_stop_ramp";
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
            static_cast<float>(std::abs(spec.yaw_target))),
        static_cast<float>(spec.yaw_rate),
    }) {
    const bool forward_case =
        spec_.kind == PerformanceCaseKind::forward_response;
    const bool heading_case =
        spec_.kind == PerformanceCaseKind::heading_response;
    const bool turn_case = spec_.kind == PerformanceCaseKind::steady_turn;
    const bool trajectory_case =
        spec_.kind == PerformanceCaseKind::figure_eight;
    if (spec_.name.empty() ||
        !std::isfinite(spec_.forward_target) ||
        !std::isfinite(spec_.yaw_target) ||
        !std::isfinite(spec_.forward_rate) || spec_.forward_rate <= 0.0 ||
        !std::isfinite(spec_.yaw_rate) || spec_.yaw_rate <= 0.0 ||
        !finite_non_negative(spec_.target_hold_seconds) ||
        !finite_non_negative(spec_.stop_settle_seconds) ||
        !finite_non_negative(spec_.standing_seconds) ||
        !std::isfinite(spec_.coupled_forward_velocity) ||
        std::abs(spec_.coupled_forward_velocity) > 3.0 ||
        !finite_non_negative(spec_.forward_lead_seconds) ||
        spec_.forward_lead_seconds > spec_.standing_seconds ||
        (forward_case && (spec_.forward_target == 0.0 ||
                          spec_.yaw_target != 0.0)) ||
        (heading_case && (spec_.yaw_target == 0.0 ||
                          spec_.forward_target != 0.0)) ||
        (turn_case && (spec_.forward_target <= 0.0 ||
                       spec_.yaw_target == 0.0)) ||
        (trajectory_case && (spec_.forward_target <= 0.0 ||
                             spec_.yaw_target <= 0.0)) ||
        (!heading_case && (spec_.coupled_forward_velocity != 0.0 ||
                           spec_.forward_lead_seconds != 0.0)) ||
        (spec_.coupled_forward_velocity == 0.0 &&
         spec_.forward_lead_seconds != 0.0) ||
        std::abs(spec_.yaw_target) > 2.0 * BC_PI) {
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
    entry_stability_active_ = false;
    entry_ready_ = false;
    entry_timed_out_ = false;
    entry_stability_start_ = simulation_time;
    entry_wait_seconds_ = 0.0;
    figure_eight_phase_ = FigureEightPhase::straight_one;
    figure_eight_phase_start_time_ = simulation_time;
    phase_start_time_ = simulation_time;
    simulation_time_ = simulation_time;
    previous_update_time_ = simulation_time;
}

void PerformanceScenario::enter_figure_eight(
    const FigureEightPhase phase
) noexcept {
    figure_eight_phase_ = phase;
    figure_eight_phase_start_time_ = simulation_time_;
}

void PerformanceScenario::enter(
    const PerformancePhase phase, const double simulation_time
) noexcept {
    phase_ = phase;
    phase_start_time_ = simulation_time;
    if (phase == PerformancePhase::entry_wait) {
        entry_stability_active_ = false;
    }
}

const char *PerformanceScenario::phase_name() const noexcept {
    if (phase_ == PerformancePhase::trajectory) {
        switch (figure_eight_phase_) {
        case FigureEightPhase::straight_one:
            return "figure_eight_straight_one";
        case FigureEightPhase::left_drive:
            return "figure_eight_left_drive";
        case FigureEightPhase::left_exit:
            return "figure_eight_left_exit";
        case FigureEightPhase::straight_two:
            return "figure_eight_straight_two";
        case FigureEightPhase::right_drive:
            return "figure_eight_right_drive";
        case FigureEightPhase::right_exit:
            return "figure_eight_right_exit";
        }
    }
    return performance_phase_name(phase_);
}

double PerformanceScenario::phase_elapsed() const noexcept {
    return simulation_time_ - phase_start_time_;
}

bool PerformanceScenario::turn_entry_stable(
    const bc_controller_snapshot_t &snapshot,
    const bool both_wheels_contact
) const noexcept {
    const double velocity_tolerance = std::max(
        kEntryForwardAbsoluteTolerance,
        kEntryForwardRelativeTolerance * std::abs(spec_.forward_target));
    return both_wheels_contact &&
        std::abs(snapshot.state.value[BC_STATE_DS] -
                 spec_.forward_target) <= velocity_tolerance &&
        std::abs(snapshot.state.value[BC_STATE_THETA_B]) <= kEntryPitchLimit &&
        std::abs(snapshot.state.value[BC_STATE_DTHETA_B]) <=
            kEntryPitchRateLimit;
}

void PerformanceScenario::update(
    const bc_controller_snapshot_t &snapshot,
    const double simulation_time,
    const bool both_wheels_contact
) noexcept {
    const float timestep_seconds = static_cast<float>(
        std::max(0.0, simulation_time - previous_update_time_));
    previous_update_time_ = simulation_time;
    simulation_time_ = simulation_time;

    if (snapshot.state_machine.motion == BC_MOTION_ACTIVE) {
        if (!gimbal_initialized_) {
            virtual_gimbal_.reset(snapshot.state.value[BC_STATE_PSI]);
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
        const double target =
            spec_.kind == PerformanceCaseKind::heading_response ?
                spec_.yaw_target : spec_.forward_target;
        const double rate =
            spec_.kind == PerformanceCaseKind::heading_response ?
                spec_.yaw_rate : spec_.forward_rate;
        if (phase_elapsed() >= std::abs(target) / rate) {
            enter(
                (spec_.kind == PerformanceCaseKind::steady_turn ||
                 spec_.kind == PerformanceCaseKind::figure_eight) ?
                    PerformancePhase::entry_wait :
                    PerformancePhase::target_hold,
                simulation_time);
        }
        break;
    }
    case PerformancePhase::entry_wait:
        entry_wait_seconds_ = phase_elapsed();
        if (turn_entry_stable(snapshot, both_wheels_contact)) {
            if (!entry_stability_active_) {
                entry_stability_active_ = true;
                entry_stability_start_ = simulation_time;
            } else if (simulation_time - entry_stability_start_ >=
                       kEntryStableSeconds) {
                entry_ready_ = true;
                entry_wait_seconds_ = phase_elapsed();
                if (spec_.kind == PerformanceCaseKind::figure_eight) {
                    enter_figure_eight(FigureEightPhase::straight_one);
                    enter(PerformancePhase::trajectory, simulation_time);
                } else {
                    enter(PerformancePhase::yaw_ramp, simulation_time);
                }
            }
        } else {
            entry_stability_active_ = false;
        }
        if (!entry_ready_ && phase_elapsed() >= kEntryTimeoutSeconds) {
            entry_timed_out_ = true;
            entry_wait_seconds_ = phase_elapsed();
            enter(PerformancePhase::forward_stop_ramp, simulation_time);
        }
        break;
    case PerformancePhase::yaw_ramp:
        if (phase_elapsed() >= std::abs(spec_.yaw_target) / spec_.yaw_rate) {
            enter(PerformancePhase::target_hold, simulation_time);
        }
        break;
    case PerformancePhase::target_hold:
        if (phase_elapsed() >= spec_.target_hold_seconds) {
            enter(
                spec_.kind == PerformanceCaseKind::forward_response ?
                    PerformancePhase::forward_stop_ramp :
                    PerformancePhase::yaw_stop_ramp,
                simulation_time);
        }
        break;
    case PerformancePhase::trajectory:
        advance_figure_eight();
        break;
    case PerformancePhase::yaw_stop_ramp:
        if (phase_elapsed() >= std::abs(spec_.yaw_target) / spec_.yaw_rate) {
            enter(
                spec_.kind == PerformanceCaseKind::steady_turn ?
                    PerformancePhase::forward_stop_ramp :
                    PerformancePhase::stop_settle,
                simulation_time);
        }
        break;
    case PerformancePhase::forward_stop_ramp:
        if (phase_elapsed() >=
            std::abs(spec_.forward_target) / spec_.forward_rate) {
            enter(PerformancePhase::stop_settle, simulation_time);
        }
        break;
    case PerformancePhase::stop_settle:
        if (phase_elapsed() >= spec_.stop_settle_seconds) {
            enter(PerformancePhase::complete, simulation_time);
        }
        break;
    case PerformancePhase::complete:
        break;
    }

    update_command(snapshot, timestep_seconds);
}

void PerformanceScenario::advance_figure_eight() noexcept {
    const double elapsed = simulation_time_ - figure_eight_phase_start_time_;
    switch (figure_eight_phase_) {
    case FigureEightPhase::straight_one:
        if (elapsed >= kFigureEightStraightSeconds) {
            enter_figure_eight(FigureEightPhase::left_drive);
        }
        break;
    case FigureEightPhase::left_drive:
        if (elapsed >= kFigureEightTurnDriveSeconds) {
            enter_figure_eight(FigureEightPhase::left_exit);
        }
        break;
    case FigureEightPhase::left_exit:
        if (elapsed >= kFigureEightTurnExitSeconds) {
            enter_figure_eight(FigureEightPhase::straight_two);
        }
        break;
    case FigureEightPhase::straight_two:
        if (elapsed >= kFigureEightStraightSeconds) {
            enter_figure_eight(FigureEightPhase::right_drive);
        }
        break;
    case FigureEightPhase::right_drive:
        if (elapsed >= kFigureEightTurnDriveSeconds) {
            enter_figure_eight(FigureEightPhase::right_exit);
        }
        break;
    case FigureEightPhase::right_exit:
        if (elapsed >= kFigureEightTurnExitSeconds) {
            enter(PerformancePhase::forward_stop_ramp, simulation_time_);
        }
        break;
    }
}

void PerformanceScenario::update_command(
    const bc_controller_snapshot_t &snapshot,
    const float timestep_seconds
) noexcept {
    command_ = {};
    if (phase_ == PerformancePhase::disabled_settle || finished()) return;

    command_.system_enabled = 1U;
    if (phase_ == PerformancePhase::engaging) {
        command_.balance_restart = static_cast<uint8_t>(
            snapshot.state_machine.system == BC_SYSTEM_OFF);
    }

    float forward = 0.0F;
    float yaw = 0.0F;
    const bool heading_case =
        spec_.kind == PerformanceCaseKind::heading_response;
    const bool turn_case = spec_.kind == PerformanceCaseKind::steady_turn;
    const bool trajectory_case =
        spec_.kind == PerformanceCaseKind::figure_eight;

    const bool coupled_forward_active = heading_case &&
        spec_.coupled_forward_velocity != 0.0 &&
        ((phase_ == PerformancePhase::standing &&
          spec_.forward_lead_seconds > 0.0 &&
          phase_elapsed() >=
              spec_.standing_seconds - spec_.forward_lead_seconds) ||
         phase_ == PerformancePhase::target_ramp ||
         phase_ == PerformancePhase::target_hold ||
         phase_ == PerformancePhase::yaw_stop_ramp);
    if (coupled_forward_active) {
        forward = static_cast<float>(spec_.coupled_forward_velocity);
    }

    if (!heading_case &&
        (phase_ == PerformancePhase::target_ramp ||
         phase_ == PerformancePhase::entry_wait ||
         phase_ == PerformancePhase::yaw_ramp ||
         phase_ == PerformancePhase::target_hold ||
         phase_ == PerformancePhase::yaw_stop_ramp ||
         phase_ == PerformancePhase::trajectory)) {
        forward = static_cast<float>(spec_.forward_target);
    }
    if ((heading_case && (phase_ == PerformancePhase::target_ramp ||
                          phase_ == PerformancePhase::target_hold)) ||
        (turn_case && (phase_ == PerformancePhase::yaw_ramp ||
                       phase_ == PerformancePhase::target_hold))) {
        yaw = static_cast<float>(spec_.yaw_target);
    }
    if (trajectory_case && phase_ == PerformancePhase::trajectory) {
        if (figure_eight_phase_ == FigureEightPhase::left_drive) {
            yaw = static_cast<float>(spec_.yaw_target);
        } else if (figure_eight_phase_ == FigureEightPhase::right_drive) {
            yaw = static_cast<float>(-spec_.yaw_target);
        }
    }

    if (gimbal_initialized_) {
        virtual_gimbal_.update(yaw, timestep_seconds);
        command_.forward_velocity = forward;
    }
}

bool PerformanceScenario::monitored() const noexcept {
    return phase_ == PerformancePhase::target_ramp ||
        phase_ == PerformancePhase::entry_wait ||
        phase_ == PerformancePhase::yaw_ramp ||
        phase_ == PerformancePhase::target_hold ||
        phase_ == PerformancePhase::trajectory ||
        phase_ == PerformancePhase::yaw_stop_ramp ||
        phase_ == PerformancePhase::forward_stop_ramp ||
        phase_ == PerformancePhase::stop_settle;
}

bool PerformanceScenario::tracking_evaluation() const noexcept {
    if (spec_.kind == PerformanceCaseKind::figure_eight) {
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
