#ifndef BALANCE_BENCHMARK_PERFORMANCE_BENCHMARK_HPP
#define BALANCE_BENCHMARK_PERFORMANCE_BENCHMARK_HPP

#include <array>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>

#include "common/common_diagnostics.hpp"
#include "common/csv_writer.hpp"
#include "common/simulation_sample.hpp"
#include "common/statistics.hpp"
#include "base_roll_restraint.hpp"
#include "forward_velocity_diagnostic.hpp"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "performance_scenario.hpp"
#include "simulation_runner.hpp"

namespace balance::benchmark {

struct PerformanceBenchmarkConfig {
    std::optional<double> leg_length;
    std::size_t trace_stride{10U};
    ForwardVelocityObservation forward_velocity_observation{
        ForwardVelocityObservation::wheel_odometry};
    bool roll_restrained{};
};

struct PerformanceResult {
    PerformanceCaseSpec spec;
    double leg_length_target{};
    ForwardVelocityObservation forward_velocity_observation{};
    bool roll_restrained{};
    bool completed{};
    bool balance_engaged{};
    bool leg_length_valid{true};
    bool tracked{};
    bool settled{};
    std::string issue{"none"};
    std::string issue_phase{"none"};
    double maximum_pitch{};
    double maximum_roll{};
    double maximum_leg_common{};
    double maximum_leg_difference{};
    double maximum_roll_restraint_torque{};
    std::array<double, BC_SIDE_NUM> minimum_vertical_projection{{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
    }};
    double initial_position_error{};
    bool initial_position_error_captured{};
    SampleStatistics tracking_error;
    SampleStatistics settle_forward;
    SampleStatistics settle_yaw;
    CommonDiagnostics common;
};

class PerformanceBenchmark {
public:
    PerformanceBenchmark(
        const std::filesystem::path &model_path,
        const std::filesystem::path &output_directory,
        const PerformanceBenchmarkConfig &config);

    [[nodiscard]] PerformanceResult run(const PerformanceCaseSpec &spec);
    void write_summary(const PerformanceResult &result);

private:
    bool step(
        const PerformanceCaseSpec &spec, const char *phase,
        const bc_operator_command_t &command, PerformanceResult *result,
        bool evaluate_tracking, bool evaluate_settle);
    bool collect(
        PerformanceResult &result, const char *phase,
        const SimulationSample &sample,
        bool evaluate_tracking, bool evaluate_settle) const;
    void step_runner(const bc_operator_command_t &command);
    void finish_result(PerformanceResult &result) const;
    void write_trace(
        const PerformanceCaseSpec &spec, const char *phase,
        const bc_operator_command_t &command,
        const SimulationSample &sample,
        const ImuMotionState &velocity_truth);

    sim::MujocoPlant plant_;
    sim::MujocoAdapter adapter_;
    sim::SimulationRunner runner_;
    SimulationSampler sampler_;
    BaseRollRestraint roll_restraint_;
    ForwardVelocityDiagnostic forward_velocity_;
    CsvWriter summary_;
    CsvWriter trace_;
    double leg_length_target_{};
    std::size_t trace_stride_{};
    std::size_t sample_index_{};
};

} // namespace balance::benchmark

#endif
