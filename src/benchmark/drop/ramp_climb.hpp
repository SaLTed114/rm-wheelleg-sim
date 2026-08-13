#ifndef BALANCE_BENCHMARK_RAMP_CLIMB_HPP
#define BALANCE_BENCHMARK_RAMP_CLIMB_HPP

#include <filesystem>
#include <limits>
#include <string>

#include "common/csv_writer.hpp"
#include "common/simulation_sample.hpp"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

namespace balance::benchmark {

struct RampClimbResult {
    bool finite{true};
    bool balance_engaged{};
    bool clearance_collision{};
    bool airborne_after_collision{};
    bool wheel_contact_at_airborne{};
    bool wheel_recontact_after_airborne{};
    bool support_estimator_recovered{};
    bool landing_recovery_started{};
    bool support_phase_recovered{};
    bool wheel_reliability_lost{};
    bool wheel_reliability_recovered{};
    bool wheel_reacquisition_used{};
    bool forward_hold_recovered{};
    bool attitude_diverged{};
    std::string first_collision{"none"};
    double active_time{std::numeric_limits<double>::quiet_NaN()};
    double collision_time{std::numeric_limits<double>::quiet_NaN()};
    double airborne_time{std::numeric_limits<double>::quiet_NaN()};
    double wheel_recontact_time{std::numeric_limits<double>::quiet_NaN()};
    double support_estimator_recovery_time{
        std::numeric_limits<double>::quiet_NaN()};
    double landing_recovery_time{std::numeric_limits<double>::quiet_NaN()};
    double ground_recovery_time{std::numeric_limits<double>::quiet_NaN()};
    double command_release_time{std::numeric_limits<double>::quiet_NaN()};
    double wheel_reliability_recovery_time{
        std::numeric_limits<double>::quiet_NaN()};
    double collision_axle_x{std::numeric_limits<double>::quiet_NaN()};
    double maximum_pitch{};
    double final_truth_velocity{};
    double final_estimated_velocity{};
    double final_wheel_velocity{};
    double final_position_error{};
};

class RampClimbBenchmark {
public:
    RampClimbBenchmark(
        const std::filesystem::path &model_path,
        const std::filesystem::path &output_directory);

    [[nodiscard]] RampClimbResult run();

private:
    struct RampContact {
        bool wheel[BC_SIDE_NUM]{};
        bool non_wheel{};
        std::string first_non_wheel{"none"};
    };

    [[nodiscard]] RampContact read_ramp_contact() const;
    void write_trace(
        CsvWriter &trace,
        const SimulationSample &sample,
        const RampContact &ramp_contact,
        double target_velocity) const;

    std::filesystem::path output_directory_;
    sim::MujocoPlant plant_;
    sim::MujocoAdapter adapter_;
    sim::SimulationRunner runner_;
    SimulationSampler sampler_;
    int ramp_{};
    int wheel_[BC_SIDE_NUM]{};
};

} // namespace balance::benchmark

#endif
