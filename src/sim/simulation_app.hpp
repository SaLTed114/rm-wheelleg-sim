#ifndef BALANCE_SIM_SIMULATION_APP_HPP
#define BALANCE_SIM_SIMULATION_APP_HPP

#include <filesystem>
#include <optional>

namespace balance::benchmark {
struct PerformanceCaseSpec;
}

namespace balance::sim {

struct SimulationAppOptions {
    std::filesystem::path model_path;
    const benchmark::PerformanceCaseSpec *performance_case{};
    std::optional<float> leg_length;
    bool keyboard_drive{};
};

void run_simulation_app(const SimulationAppOptions &options);

} // namespace balance::sim

#endif
