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
    std::optional<double> forward_acceleration_rate;
    std::size_t trace_stride{10U};
    ForwardVelocityObservation forward_velocity_observation{
        ForwardVelocityObservation::wheel_odometry};
    bool roll_restrained{};
    std::optional<double> yaw_acceleration_feedforward_scale;
};

[[nodiscard]] bc_controller_config_t performance_controller_config(
    const PerformanceBenchmarkConfig &config);

struct PerformanceResult {
    PerformanceCaseSpec spec;
    double leg_length_target{};
    ForwardVelocityObservation forward_velocity_observation{};
    bool roll_restrained{};
    double yaw_acceleration_feedforward_scale{};
    bool completed{};
    bool balance_engaged{};
    bool leg_length_valid{true};
    bool valid{};
    bool response_pass{};
    bool stop_pass{};
    bool contact_free{};
    bool unsaturated{};
    bool entry_ready{};
    bool entry_timed_out{};
    bool attitude_terminated{};
    std::string issue{"none"};
    std::string issue_phase{"none"};
    double entry_wait_seconds{};
    double t10{std::numeric_limits<double>::quiet_NaN()};
    double t50{std::numeric_limits<double>::quiet_NaN()};
    double t90{std::numeric_limits<double>::quiet_NaN()};
    double t10_t90_acceleration{
        std::numeric_limits<double>::quiet_NaN()};
    double overshoot{};
    double maximum_pitch{};
    double maximum_roll{};
    double maximum_leg_common{};
    double maximum_leg_difference{};
    double maximum_roll_force_request{};
    double maximum_roll_restraint_torque{};
    std::array<double, BC_SIDE_NUM> minimum_vertical_projection{{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
    }};
    double initial_position_error{};
    bool initial_position_error_captured{};
    double maximum_heading_error{};
    double stop_peak_yaw_rate{};
    bool path_start_captured{};
    double path_start_x{};
    double path_start_y{};
    double path_end_x{};
    double path_end_y{};
    double path_closure_error{};
    bool response_started{};
    double response_start_time{};
    double response_initial_forward{};
    double maximum_forward_progress{};
    SampleStatistics forward_error;
    SampleStatistics yaw_error;
    SampleStatistics actual_forward;
    SampleStatistics actual_yaw;
    SampleStatistics lateral_acceleration;
    SampleStatistics heading_error;
    SampleStatistics settle_forward;
    SampleStatistics settle_yaw;
    CommonDiagnostics common;
    CommonDiagnostics hold;
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
        const bc_operator_command_t &command,
        const sim::VirtualGimbalState &gimbal,
        PerformanceResult *result, bool evaluate_tracking,
        bool evaluate_settle, bool stopping);
    bool collect(
        PerformanceResult &result, const char *phase,
        const SimulationSample &sample,
        bool evaluate_tracking, bool evaluate_settle,
        bool stopping) const;
    void collect_response_timing(
        PerformanceResult &result,
        const SimulationSample &sample,
        bool stopping) const;
    void step_runner(
        const bc_operator_command_t &command,
        const sim::VirtualGimbalState &gimbal);
    void finish_result(PerformanceResult &result) const;
    void write_trace(
        const PerformanceCaseSpec &spec, const char *phase,
        const bc_operator_command_t &command,
        const sim::VirtualGimbalState &gimbal,
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
    double yaw_acceleration_feedforward_scale_{};
    std::size_t trace_stride_{};
    std::size_t sample_index_{};
};

} // namespace balance::benchmark

#endif
