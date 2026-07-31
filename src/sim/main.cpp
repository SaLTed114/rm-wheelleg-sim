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
#include "static_stand_scenario.hpp"

namespace {

struct MotionTarget {
    float forward_velocity;
    float yaw_rate;
    const char *phase;
};

MotionTarget make_motion_target(
    const balance::sim::StaticStandScenario &scenario,
    const double simulation_time
) {
    if (scenario.phase() != balance::sim::StaticStandPhase::Balancing) {
        return {0.0F, 0.0F, scenario.phase_name()};
    }

    constexpr double kCycleDuration = 23.0;
    const double time = std::fmod(
        simulation_time - scenario.release_time(), kCycleDuration);
    if (time < 3.0) return {0.0F, 0.0F, "standing"};
    if (time < 6.0) return {0.25F, 0.0F, "forward"};
    if (time < 8.0) return {0.0F, 0.0F, "stopping"};
    if (time < 11.0) return {-0.25F, 0.0F, "reverse"};
    if (time < 13.0) return {0.0F, 0.0F, "stopping"};
    if (time < 16.0) return {0.0F, 1.57F, "yaw left"};
    if (time < 18.0) return {0.0F, 0.0F, "stopping"};
    if (time < 21.0) return {0.0F, -1.57F, "yaw right"};
    return {0.0F, 0.0F, "stopping"};
}

void print_state(
    const char *phase, const bc_state_vector_t &state
) {
    std::cout << phase << ':';
    for (int index = 0; index < BC_STATE_NUM; ++index) {
        std::cout << ' ' << state.value[index];
    }
    std::cout << '\n';
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
        balance::sim::StaticStandScenario scenario(plant, runner);
        balance::sim::MujocoViewer viewer(plant.model());
        scenario.reset();

        using Clock = std::chrono::steady_clock;
        auto previous_time = Clock::now();
        double accumulated_time = 0.0;
        double next_state_print = 0.0;
        MotionTarget motion{0.0F, 0.0F, scenario.phase_name()};

        while (!viewer.should_close()) {
            const auto current_time = Clock::now();
            const std::chrono::duration<double> frame_time =
                current_time - previous_time;
            previous_time = current_time;

            if (viewer.consume_reset_request()) {
                scenario.reset();
                motion = {0.0F, 0.0F, scenario.phase_name()};
                accumulated_time = 0.0;
                next_state_print = 0.0;
            }

            if (viewer.paused()) {
                accumulated_time = 0.0;
            } else {
                accumulated_time += std::clamp(
                    frame_time.count(), 0.0, kMaxFrameTimeSeconds);
                while (accumulated_time >= plant.timestep()) {
                    motion = make_motion_target(
                        scenario, plant.data().time);
                    scenario.set_motion_target(
                        motion.forward_velocity, motion.yaw_rate);
                    scenario.step();
                    accumulated_time -= plant.timestep();
                }
            }

            if (plant.data().time >= next_state_print) {
                print_state(motion.phase, runner.state());
                next_state_print += 0.5;
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
