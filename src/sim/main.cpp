#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

namespace {

double parse_duration(const char *text) {
    std::size_t parsed = 0;
    const std::string value(text);
    const double duration = std::stod(value, &parsed);
    if (parsed != value.size()) {
        throw std::invalid_argument("duration contains trailing characters");
    }
    return duration;
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 2 || argc > 3) {
        std::cerr << "usage: rm_balance_sim <model.xml> [duration_seconds]\n";
        return EXIT_FAILURE;
    }

    try {
        constexpr double kTimestepSeconds = 0.001;
        const double duration_seconds =
            argc == 3 ? parse_duration(argv[2]) : 1.0;

        balance::sim::MujocoPlant plant(
            std::filesystem::path(argv[1]), kTimestepSeconds);
        balance::sim::MujocoAdapter adapter(plant.model());
        balance::sim::SimulationRunner runner(plant, adapter);
        runner.reset();
        const auto stats = runner.run_for(duration_seconds);

        std::cout << "loaded model: nq=" << plant.model().nq
                  << " nv=" << plant.model().nv
                  << " nu=" << plant.model().nu << '\n';
        std::cout << "completed: physics_steps=" << stats.physics_steps
                  << " control_ticks=" << stats.control_ticks
                  << " final_time=" << std::fixed << std::setprecision(6)
                  << stats.final_time << " s\n";
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_sim: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
