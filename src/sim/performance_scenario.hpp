#ifndef BALANCE_SIM_PERFORMANCE_SCENARIO_HPP
#define BALANCE_SIM_PERFORMANCE_SCENARIO_HPP

#include <array>
#include <string>
#include <string_view>

#include <mujoco/mujoco.h>

#include "balance/controller_snapshot.h"

namespace balance::sim {

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
    failed,
};

struct PerformanceCaseSpec {
    std::string_view name;
    PerformanceAxis axis;
    double target;
};

[[nodiscard]] const std::array<PerformanceCaseSpec, 14> &
performance_cases() noexcept;
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
    [[nodiscard]] bool failed() const noexcept {
        return phase_ == PerformancePhase::failed;
    }
    [[nodiscard]] const char *failure_reason() const noexcept {
        return failure_reason_;
    }

private:
    void enter(PerformancePhase phase, double simulation_time) noexcept;
    [[nodiscard]] double phase_elapsed() const noexcept;

    PerformanceCaseSpec spec_;
    PerformancePhase phase_{PerformancePhase::disabled_settle};
    bc_operator_command_t command_{};
    double phase_start_time_{};
    double simulation_time_{};
    const char *failure_reason_{"none"};
};

struct PerformanceContactState {
    std::array<bool, BC_SIDE_NUM> wheel{};
    bool other{};
    std::string unexpected;
};

class PerformanceContactMonitor {
public:
    explicit PerformanceContactMonitor(const mjModel &model);

    [[nodiscard]] PerformanceContactState read(const mjData &data) const;

private:
    [[nodiscard]] std::string contact_name(
        const mjContact &contact) const;

    const mjModel &model_;
    int ground_{};
    std::array<int, BC_SIDE_NUM> wheel_{};
};

[[nodiscard]] std::string performance_termination_reason(
    const bc_controller_snapshot_t &snapshot,
    const PerformanceContactState &contact);

} // namespace balance::sim

#endif
