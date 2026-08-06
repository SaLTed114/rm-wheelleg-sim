#ifndef BALANCE_BENCHMARK_PERFORMANCE_SCENARIO_HPP
#define BALANCE_BENCHMARK_PERFORMANCE_SCENARIO_HPP

#include <array>
#include <string_view>

#include "balance/controller_snapshot.h"

namespace balance::benchmark {

enum class PerformanceAxis {
    forward,
    yaw,
};

enum class PerformancePhase {
    disabled_settle,
    engaging,
    standing,
    target_ramp,
    target_hold,
    stop_ramp,
    stop_settle,
    complete,
};

struct PerformanceCaseSpec {
    std::string_view name;
    PerformanceAxis axis;
    double target;
    double command_rate;
    double target_hold_seconds{3.0};
    double stop_settle_seconds{2.0};
    double standing_seconds{2.0};
};

[[nodiscard]] const std::array<PerformanceCaseSpec, 16> &
performance_cases() noexcept;
[[nodiscard]] const std::array<PerformanceCaseSpec, 10> &
forward_acceleration_cases() noexcept;
[[nodiscard]] const std::array<PerformanceCaseSpec, 14> &
yaw_acceleration_cases() noexcept;
[[nodiscard]] const PerformanceCaseSpec *find_performance_case(
    std::string_view name) noexcept;
[[nodiscard]] const char *performance_axis_name(
    PerformanceAxis axis) noexcept;
[[nodiscard]] const char *performance_phase_name(
    PerformancePhase phase) noexcept;

class PerformanceScenario {
public:
    explicit PerformanceScenario(const PerformanceCaseSpec &spec);

    void reset(double simulation_time = 0.0) noexcept;
    void update(
        const bc_controller_snapshot_t &snapshot,
        double simulation_time) noexcept;

    [[nodiscard]] const PerformanceCaseSpec &spec() const noexcept {
        return spec_;
    }
    [[nodiscard]] PerformancePhase phase() const noexcept { return phase_; }
    [[nodiscard]] const char *phase_name() const noexcept {
        return performance_phase_name(phase_);
    }
    [[nodiscard]] const bc_operator_command_t &command() const noexcept {
        return command_;
    }
    [[nodiscard]] bool monitored() const noexcept;
    [[nodiscard]] bool tracking_evaluation() const noexcept;
    [[nodiscard]] bool settle_evaluation() const noexcept;
    [[nodiscard]] bool finished() const noexcept;

private:
    void enter(PerformancePhase phase, double simulation_time) noexcept;
    [[nodiscard]] double phase_elapsed() const noexcept;

    PerformanceCaseSpec spec_;
    PerformancePhase phase_{PerformancePhase::disabled_settle};
    bc_operator_command_t command_{};
    double phase_start_time_{};
    double simulation_time_{};
};

} // namespace balance::benchmark

#endif
