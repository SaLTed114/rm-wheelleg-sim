#ifndef BALANCE_BENCHMARK_PERFORMANCE_SCENARIO_HPP
#define BALANCE_BENCHMARK_PERFORMANCE_SCENARIO_HPP

#include <array>
#include <string_view>

#include "balance/controller_snapshot.h"
#include "input/virtual_gimbal.hpp"

namespace balance::benchmark {

enum class PerformanceCaseKind {
    forward_response,
    heading_response,
    steady_turn,
    figure_eight,
};

enum class PerformancePhase {
    disabled_settle,
    engaging,
    standing,
    target_ramp,
    entry_wait,
    yaw_ramp,
    target_hold,
    trajectory,
    yaw_stop_ramp,
    forward_stop_ramp,
    stop_settle,
    complete,
};

struct PerformanceCaseSpec {
    std::string_view name;
    PerformanceCaseKind kind{PerformanceCaseKind::forward_response};
    double forward_target{};
    double yaw_target{};
    double forward_rate{5.0};
    double yaw_rate{10.0};
    double target_hold_seconds{3.0};
    double stop_settle_seconds{2.0};
    double standing_seconds{2.0};
    double coupled_forward_velocity{};
    double forward_lead_seconds{};
    bool formal_acceptance{};
};

[[nodiscard]] const std::array<PerformanceCaseSpec, 4> &
formal_performance_cases() noexcept;
[[nodiscard]] const std::array<PerformanceCaseSpec, 1> &
trajectory_performance_cases() noexcept;
[[nodiscard]] const PerformanceCaseSpec *find_performance_case(
    std::string_view name) noexcept;
[[nodiscard]] const char *performance_case_kind_name(
    PerformanceCaseKind kind) noexcept;
[[nodiscard]] const char *performance_phase_name(
    PerformancePhase phase) noexcept;

class PerformanceScenario {
public:
    explicit PerformanceScenario(const PerformanceCaseSpec &spec);

    void reset(double simulation_time = 0.0) noexcept;
    void update(
        const bc_controller_snapshot_t &snapshot,
        double simulation_time, bool both_wheels_contact = true) noexcept;

    [[nodiscard]] const PerformanceCaseSpec &spec() const noexcept {
        return spec_;
    }
    [[nodiscard]] PerformancePhase phase() const noexcept { return phase_; }
    [[nodiscard]] const char *phase_name() const noexcept;
    [[nodiscard]] const bc_operator_command_t &command() const noexcept {
        return command_;
    }
    [[nodiscard]] const sim::VirtualGimbalState &gimbal() const noexcept {
        return virtual_gimbal_.state();
    }
    [[nodiscard]] bool monitored() const noexcept;
    [[nodiscard]] bool tracking_evaluation() const noexcept;
    [[nodiscard]] bool settle_evaluation() const noexcept;
    [[nodiscard]] bool entry_ready() const noexcept { return entry_ready_; }
    [[nodiscard]] bool entry_timed_out() const noexcept {
        return entry_timed_out_;
    }
    [[nodiscard]] double entry_wait_seconds() const noexcept {
        return entry_wait_seconds_;
    }
    [[nodiscard]] bool finished() const noexcept;

private:
    enum class FigureEightPhase {
        straight_one,
        left_drive,
        left_exit,
        straight_two,
        right_drive,
        right_exit,
    };

    void enter(PerformancePhase phase, double simulation_time) noexcept;
    void enter_figure_eight(FigureEightPhase phase) noexcept;
    void advance_figure_eight() noexcept;
    void update_command(
        const bc_controller_snapshot_t &snapshot,
        float timestep_seconds) noexcept;
    [[nodiscard]] bool turn_entry_stable(
        const bc_controller_snapshot_t &snapshot,
        bool both_wheels_contact) const noexcept;
    [[nodiscard]] double phase_elapsed() const noexcept;

    PerformanceCaseSpec spec_;
    PerformancePhase phase_{PerformancePhase::disabled_settle};
    bc_operator_command_t command_{};
    sim::VirtualGimbal virtual_gimbal_;
    bool gimbal_initialized_{};
    bool entry_stability_active_{};
    bool entry_ready_{};
    bool entry_timed_out_{};
    double entry_stability_start_{};
    double entry_wait_seconds_{};
    FigureEightPhase figure_eight_phase_{FigureEightPhase::straight_one};
    double figure_eight_phase_start_time_{};
    double phase_start_time_{};
    double simulation_time_{};
    double previous_update_time_{};
};

} // namespace balance::benchmark

#endif
