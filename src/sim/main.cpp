#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>

#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "mujoco_viewer.hpp"
#include "simulation_runner.hpp"

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: rm_balance_sim <model.xml>\n";
        return EXIT_FAILURE;
    }

    try {
        constexpr double kTimestepSeconds = 0.001;
        constexpr double kMaxFrameTimeSeconds = 0.1;

        balance::sim::MujocoPlant plant(
            std::filesystem::path(argv[1]), kTimestepSeconds);
        balance::sim::MujocoAdapter adapter(plant.model());
        balance::sim::SimulationRunner runner(plant, adapter);
        balance::sim::MujocoViewer viewer(plant.model());
        runner.reset();

        using Clock = std::chrono::steady_clock;
        auto previous_time = Clock::now();
        double accumulated_time = 0.0;

        while (!viewer.should_close()) {
            const auto current_time = Clock::now();
            const std::chrono::duration<double> frame_time =
                current_time - previous_time;
            previous_time = current_time;

            if (viewer.consume_reset_request()) {
                runner.reset();
                accumulated_time = 0.0;
            }

            if (viewer.paused()) {
                accumulated_time = 0.0;
            } else {
                accumulated_time += std::clamp(
                    frame_time.count(), 0.0, kMaxFrameTimeSeconds);
                while (accumulated_time >= plant.timestep()) {
                    runner.step();
                    accumulated_time -= plant.timestep();
                }
            }

            viewer.render(plant.data());
            viewer.poll_events();
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_sim: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
