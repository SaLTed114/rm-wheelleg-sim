#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>

#include "cpp/simulation_runner.hpp"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "mujoco_viewer.hpp"

namespace {

constexpr double timestep_seconds = 0.001;
constexpr double max_frame_time_seconds = 0.1;

} // namespace

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: rm_balance_cpp_viewer <model.xml>\n";
        return EXIT_FAILURE;
    }

    try {
        balance::sim::MujocoPlant plant(
            std::filesystem::path(argv[1]), timestep_seconds);
        balance::sim::MujocoAdapter adapter(plant.model());
        balance::sim::cpp::SimulationRunner runner(plant, adapter);
        balance::sim::MujocoViewer viewer(
            plant.model(), "rm-balance-cpp-viewer");
        runner.reset();

        using Clock = std::chrono::steady_clock;
        auto previous_time = Clock::now();
        double accumulated_time = 0.0;
        while (!viewer.should_close()) {
            viewer.poll_events();
            const auto current_time = Clock::now();
            const std::chrono::duration<double> frame_time =
                current_time - previous_time;
            previous_time = current_time;
            accumulated_time += std::clamp(
                frame_time.count(), 0.0, max_frame_time_seconds);

            while (accumulated_time >= plant.timestep()) {
                balance::control::OperatorCommand command{};
                command.system_enabled = plant.data().time >= 2.0;
                command.balance_restart = command.system_enabled &&
                    runner.snapshot().motion_state ==
                        balance::control::MotionState::idle;
                runner.step(command);
                accumulated_time -= plant.timestep();
            }

            viewer.render_scene(plant.data());
            viewer.present();
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_cpp_viewer: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
