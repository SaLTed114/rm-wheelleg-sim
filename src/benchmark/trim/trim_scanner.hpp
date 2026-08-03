#ifndef BALANCE_BENCHMARK_TRIM_SCANNER_HPP
#define BALANCE_BENCHMARK_TRIM_SCANNER_HPP

#include <filesystem>
#include <string>

#include "common/common_diagnostics.hpp"
#include "common/csv_writer.hpp"
#include "common/simulation_sample.hpp"
#include "common/statistics.hpp"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

namespace balance::benchmark {

struct TrimScanConfig {
    double leg_length{0.18};
    double offset_minimum_deg{-5.0};
    double offset_maximum_deg{15.0};
    double offset_step_deg{1.0};
    std::size_t trace_stride{10U};
};

struct TrimResult {
    double offset_deg{};
    bool engaged{};
    bool candidate{};
    std::string issue{"none"};
    double maximum_position_error{};
    double forward_displacement{};
    SampleStatistics pitch;
    SampleStatistics velocity;
    SampleStatistics base_velocity;
    SampleStatistics leg_common;
    SampleStatistics leg_difference;
    SampleStatistics wheel_common;
    SampleStatistics normal_force_left;
    SampleStatistics normal_force_right;
    LinearTrend velocity_trend;
    CommonDiagnostics common;
};

class TrimScanner {
public:
    TrimScanner(
        const std::filesystem::path &model_path,
        const std::filesystem::path &output_directory,
        const TrimScanConfig &config);

    [[nodiscard]] TrimResult run(double offset_deg);
    void write_summary(const TrimResult &result);

private:
    void collect(
        TrimResult &result, const SimulationSample &sample,
        double evaluation_time) const;
    void finish(TrimResult &result) const;
    void write_trace(
        const SimulationSample &sample,
        double offset_deg, double time);

    sim::MujocoPlant plant_;
    sim::MujocoAdapter adapter_;
    SimulationSampler sampler_;
    TrimScanConfig config_;
    CsvWriter summary_;
    CsvWriter trace_;
};

} // namespace balance::benchmark

#endif
