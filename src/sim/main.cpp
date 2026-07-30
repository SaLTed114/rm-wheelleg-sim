#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>

#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "mujoco_viewer.hpp"
#include "simulation_runner.hpp"

namespace {

bc_operator_command_t make_demo_command(double simulation_time) {
    constexpr double kPhaseDuration = 4.0;
    constexpr float kPi = 3.14159265358979323846F;
    constexpr float kLengthCenter = 0.30F;
    constexpr float kLengthAmplitude = 0.04F;
    constexpr float kAngleCenter = -0.5F * kPi;
    constexpr float kAngleAmplitude = 15.0F * kPi / 180.0F;

    const double cycle_time = std::fmod(
        simulation_time, 2.0 * kPhaseDuration);
    const bool angle_phase = cycle_time < kPhaseDuration;
    const double phase_time = angle_phase
        ? cycle_time : cycle_time - kPhaseDuration;
    const float wave = static_cast<float>(std::sin(
        2.0 * kPi * phase_time / kPhaseDuration));

    bc_operator_command_t command{};
    command.enabled = 1U;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        command.leg[side].length = kLengthCenter;
        command.leg[side].angle_body = kAngleCenter;

        if (angle_phase) {
            command.leg[side].angle_body += kAngleAmplitude * wave;
        } else {
            command.leg[side].length += kLengthAmplitude * wave;
        }
    }
    return command;
}

} // namespace

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
        runner.set_command(make_demo_command(plant.data().time));

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
                runner.set_command(make_demo_command(plant.data().time));
                accumulated_time = 0.0;
            }

            if (viewer.paused()) {
                accumulated_time = 0.0;
            } else {
                accumulated_time += std::clamp(
                    frame_time.count(), 0.0, kMaxFrameTimeSeconds);
                while (accumulated_time >= plant.timestep()) {
                    runner.set_command(make_demo_command(plant.data().time));
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
