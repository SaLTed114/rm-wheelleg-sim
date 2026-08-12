#ifndef BALANCE_BENCHMARK_DROP_BENCHMARK_HPP
#define BALANCE_BENCHMARK_DROP_BENCHMARK_HPP

#include <array>
#include <filesystem>
#include <limits>
#include <string>
#include <string_view>

#include "common/common_diagnostics.hpp"
#include "common/csv_writer.hpp"
#include "common/simulation_sample.hpp"
#include "drop_policy.hpp"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

namespace balance::benchmark {

struct DropCaseSpec {
    DropAirPolicy policy{DropAirPolicy::length_only};
    double wheel_clearance{0.2};
    double initial_pitch_rate{};
};

struct DropResult {
    DropCaseSpec spec;
    std::string name;
    bool completed{};
    bool finite{true};
    bool attitude_diverged{};
    bool balance_engaged{};
    bool touchdown{};
    bool first_contact{};
    std::string first_contact_kind{"none"};
    double release_time{};
    double first_contact_time{std::numeric_limits<double>::quiet_NaN()};
    double touchdown_time{std::numeric_limits<double>::quiet_NaN()};
    std::array<double, BC_SIDE_NUM> release_clearance{};
    double touchdown_pitch{};
    double touchdown_pitch_rate{};
    double touchdown_roll{};
    std::array<double, BC_SIDE_NUM> touchdown_leg_angle{};
    std::array<double, BC_SIDE_NUM> touchdown_leg_length{};
    double airborne_max_pitch{};
    double airborne_max_pitch_rate{};
    double airborne_max_leg_angle_error{};
    double post_max_pitch{};
    double post_max_roll{};
    bool post_rebound{};
    bool post_other_contact{};
    std::string post_first_other_contact{"none"};
    double post_first_other_contact_time{
        std::numeric_limits<double>::quiet_NaN()};
    double final_pitch{};
    double final_pitch_rate{};
    double final_roll{};
    std::array<bool, BC_SIDE_NUM> support_air_detected{};
    std::array<bool, BC_SIDE_NUM> support_ground_detected{};
    std::array<double, BC_SIDE_NUM> support_air_time{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
    std::array<double, BC_SIDE_NUM> support_ground_time{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
    CommonDiagnostics airborne_diagnostics;
    CommonDiagnostics post_diagnostics;
};

[[nodiscard]] std::string drop_case_name(const DropCaseSpec &spec);
[[nodiscard]] const std::array<DropCaseSpec, 6> &drop_exploration_cases();
[[nodiscard]] const DropCaseSpec *find_drop_case(
    std::string_view name) noexcept;

class DropBenchmark {
public:
    DropBenchmark(
        const std::filesystem::path &model_path,
        const std::filesystem::path &output_directory);

    [[nodiscard]] DropResult run(const DropCaseSpec &spec);
    void write_summary(const DropResult &result);

private:
    [[nodiscard]] std::array<double, BC_SIDE_NUM>
    wheel_ground_clearance();
    void prepare_release(const DropCaseSpec &spec, DropResult &result);
    void write_trace(
        CsvWriter &trace,
        const DropResult &result,
        const char *phase,
        bool touchdown_latched,
        const SimulationSample &sample);

    std::filesystem::path output_directory_;
    sim::MujocoPlant plant_;
    sim::MujocoAdapter adapter_;
    sim::SimulationRunner runner_;
    SimulationSampler sampler_;
    CsvWriter summary_;
    int base_qpos_{};
    int base_dof_{};
    int ground_{};
    std::array<int, BC_SIDE_NUM> wheel_{};
    double leg_angle_trim_{};
};

} // namespace balance::benchmark

#endif
