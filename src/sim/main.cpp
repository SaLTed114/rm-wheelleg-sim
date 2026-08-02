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

struct MotionTarget {
    float forward_velocity;
    float yaw_rate;
    const char *phase;
};

MotionTarget make_motion_target(
    const bc_controller_snapshot_t &snapshot,
    const double balance_start_time,
    const double simulation_time
) {
    if (snapshot.state_machine.system == BC_SYSTEM_OFF) {
        return {
            0.0F, 0.0F,
            bc_system_state_name(snapshot.state_machine.system),
        };
    }
    if (snapshot.state_machine.motion != BC_MOTION_BALANCE_ENGAGING) {
        return {
            0.0F, 0.0F,
            bc_motion_state_name(snapshot.state_machine.motion),
        };
    }
    if (balance_start_time < 0.0) return {0.0F, 0.0F, "standing"};

    constexpr double kCycleDuration = 23.0;
    const double time = std::fmod(
        simulation_time - balance_start_time, kCycleDuration);
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
        balance::sim::MujocoViewer viewer(plant.model());
        runner.reset();

        using Clock = std::chrono::steady_clock;
        auto previous_time = Clock::now();
        double accumulated_time = 0.0;
        double next_state_print = 0.0;
        double balance_start_time = -1.0;
        bc_motion_state_t previous_motion = BC_MOTION_IDLE;
        MotionTarget motion{
            0.0F, 0.0F,
            bc_system_state_name(runner.snapshot().state_machine.system),
        };

        while (!viewer.should_close()) {
            const auto current_time = Clock::now();
            const std::chrono::duration<double> frame_time =
                current_time - previous_time;
            previous_time = current_time;

            if (viewer.consume_reset_request()) {
                runner.reset();
                motion = {
                    0.0F, 0.0F,
                    bc_system_state_name(
                        runner.snapshot().state_machine.system),
                };
                accumulated_time = 0.0;
                next_state_print = 0.0;
                balance_start_time = -1.0;
                previous_motion = BC_MOTION_IDLE;
            }

            if (viewer.paused()) {
                accumulated_time = 0.0;
            } else {
                accumulated_time += std::clamp(
                    frame_time.count(), 0.0, kMaxFrameTimeSeconds);
                while (accumulated_time >= plant.timestep()) {
                    const auto &snapshot = runner.snapshot();
                    if (previous_motion != BC_MOTION_BALANCE_ENGAGING &&
                        snapshot.state_machine.motion ==
                            BC_MOTION_BALANCE_ENGAGING) {
                        balance_start_time = plant.data().time;
                    }
                    previous_motion = snapshot.state_machine.motion;
                    motion = make_motion_target(
                        snapshot, balance_start_time, plant.data().time);
                    bc_operator_command_t command{};
                    command.system_enabled = static_cast<uint8_t>(
                        plant.data().time >= 2.0);
                    command.balance_restart =
                        command.system_enabled &&
                        snapshot.state_machine.system == BC_SYSTEM_OFF;
                    command.forward_velocity = motion.forward_velocity;
                    command.yaw_rate = motion.yaw_rate;
                    runner.step(command);
                    accumulated_time -= plant.timestep();
                }
            }

            if (plant.data().time >= next_state_print) {
                print_state(motion.phase, runner.snapshot().state);
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
