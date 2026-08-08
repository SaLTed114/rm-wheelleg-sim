#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

#include "balance/math_utils.h"
#include "common/common_diagnostics.hpp"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "mujoco_viewer.hpp"
#include "performance/performance_scenario.hpp"
#include "common/simulation_sample.hpp"
#include "input/virtual_gimbal.hpp"
#include "simulation_runner.hpp"

namespace {

struct MotionTarget {
    float forward_velocity;
    float gimbal_yaw_rate;
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
    if (snapshot.state_machine.motion != BC_MOTION_ACTIVE) {
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

MotionTarget make_keyboard_motion_target(
    const bc_controller_snapshot_t &snapshot,
    const balance::sim::KeyboardDriveInput &input
) {
    if (snapshot.state_machine.system == BC_SYSTEM_OFF) {
        return {
            0.0F, 0.0F,
            bc_system_state_name(snapshot.state_machine.system),
        };
    }
    if (snapshot.state_machine.motion != BC_MOTION_ACTIVE) {
        return {
            0.0F, 0.0F,
            bc_motion_state_name(snapshot.state_machine.motion),
        };
    }

    constexpr float kKeyboardForwardVelocity = 2.0F;
    constexpr float kKeyboardBoostForwardVelocity = 3.0F;
    constexpr float kKeyboardYawRate = BC_PI_F;
    return {
        input.forward_axis * (input.boost ?
            kKeyboardBoostForwardVelocity :
            kKeyboardForwardVelocity),
        input.yaw_axis * kKeyboardYawRate,
        bc_forward_state_name(snapshot.state_machine.forward),
    };
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

void print_usage() {
    std::cerr
        << "usage: rm_balance_sim <model.xml> [--keyboard] "
           "[--case <case-name>] "
           "[--leg-length <metres>]\n"
        << "keyboard mode: W/S forward/reverse, A/D left/right, "
           "Shift boosts forward speed, "
           "Space pause, R reset, Esc quit\n"
        << "available performance cases:\n";
    for (const auto &spec : balance::benchmark::performance_cases()) {
        std::cerr << "  " << spec.name << '\n';
    }
    for (const auto &spec :
         balance::benchmark::forward_acceleration_cases()) {
        std::cerr << "  " << spec.name << '\n';
    }
}

} // namespace

int main(int argc, char **argv) {
    const balance::benchmark::PerformanceCaseSpec *selected_case = nullptr;
    std::optional<float> selected_leg_length;
    bool keyboard_drive = false;
    bool arguments_valid = argc >= 2;
    for (int index = 2; arguments_valid && index < argc;) {
        const std::string option = argv[index];
        if (option == "--keyboard" && !keyboard_drive) {
            keyboard_drive = true;
            ++index;
            continue;
        }
        if (index + 1 >= argc) {
            arguments_valid = false;
            break;
        }

        const std::string value = argv[index + 1];
        if (option == "--case" && selected_case == nullptr) {
            selected_case =
                balance::benchmark::find_performance_case(value);
            if (selected_case == nullptr) {
                std::cerr << "unknown performance case: " << value << '\n';
                arguments_valid = false;
            }
        } else if (option == "--leg-length" && !selected_leg_length) {
            std::size_t consumed = 0U;
            try {
                selected_leg_length = std::stof(value, &consumed);
            } catch (const std::exception &) {
                arguments_valid = false;
                break;
            }
            arguments_valid = consumed == value.size() &&
                std::isfinite(*selected_leg_length) &&
                *selected_leg_length > 0.0F;
        } else {
            arguments_valid = false;
        }
        index += 2;
    }
    arguments_valid = arguments_valid &&
        !(keyboard_drive && selected_case != nullptr);
    if (!arguments_valid) {
        print_usage();
        return EXIT_FAILURE;
    }

    try {
        constexpr double kTimestepSeconds = 0.001;
        constexpr double kMaxFrameTimeSeconds = 0.1;

        balance::sim::MujocoPlant plant(
            std::filesystem::path(argv[1]), kTimestepSeconds);
        balance::sim::MujocoAdapter adapter(plant.model());
        bc_controller_config_t controller_config{};
        bc_controller_default_config(&controller_config);
        if (selected_leg_length) {
            controller_config.motion.leg_length = *selected_leg_length;
        }
        balance::sim::SimulationRunner runner(
            plant, adapter, controller_config);
        balance::sim::VirtualGimbal virtual_gimbal;
        balance::sim::MujocoViewer viewer(plant.model());
        balance::benchmark::SimulationSampler sampler(plant.model());
        std::optional<balance::benchmark::PerformanceScenario>
            performance_scenario;
        if (selected_case != nullptr) {
            performance_scenario.emplace(*selected_case);
        }
        runner.reset();
        if (performance_scenario) {
            performance_scenario->reset(plant.data().time);
        }

        using Clock = std::chrono::steady_clock;
        auto previous_time = Clock::now();
        double accumulated_time = 0.0;
        double next_state_print = 0.0;
        double balance_start_time = -1.0;
        bool case_finished = false;
        bool case_balance_engaged = false;
        std::string case_issue{"none"};
        bc_motion_state_t previous_motion = BC_MOTION_IDLE;
        bool virtual_gimbal_initialized = false;
        balance::sim::VirtualGimbalState displayed_gimbal{};
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
                if (performance_scenario) {
                    performance_scenario->reset(plant.data().time);
                }
                motion = {
                    0.0F, 0.0F,
                    bc_system_state_name(
                        runner.snapshot().state_machine.system),
                };
                accumulated_time = 0.0;
                next_state_print = 0.0;
                balance_start_time = -1.0;
                case_finished = false;
                case_balance_engaged = false;
                case_issue = "none";
                previous_motion = BC_MOTION_IDLE;
                virtual_gimbal.reset();
                virtual_gimbal_initialized = false;
                displayed_gimbal = {};
            }

            if (viewer.paused() || case_finished) {
                accumulated_time = 0.0;
            } else {
                accumulated_time += std::clamp(
                    frame_time.count(), 0.0, kMaxFrameTimeSeconds);
                while (accumulated_time >= plant.timestep()) {
                    const auto &snapshot = runner.snapshot();
                    bc_operator_command_t command{};

                    if (performance_scenario) {
                        case_balance_engaged = case_balance_engaged ||
                            snapshot.state_machine.motion ==
                                BC_MOTION_ACTIVE;
                        performance_scenario->update(
                            snapshot, plant.data().time);
                        motion = {
                            performance_scenario->command()
                                .forward_velocity,
                            performance_scenario->gimbal()
                                .world_yaw_rate,
                            performance_scenario->phase_name(),
                        };
                        displayed_gimbal = performance_scenario->gimbal();
                        if (performance_scenario->finished()) {
                            case_finished = true;
                            if (!case_balance_engaged) {
                                case_issue = "balance_not_engaged";
                            }
                            std::cout << "case " << selected_case->name
                                      << " complete";
                            if (case_issue != "none") {
                                std::cout << "; issue=" << case_issue;
                            }
                            std::cout
                                      << "; press R to replay\n";
                            accumulated_time = 0.0;
                            break;
                        }
                        command = performance_scenario->command();
                        const bool monitored =
                            performance_scenario->monitored();
                        runner.step(
                            command,
                            performance_scenario->gimbal_feedback());
                        accumulated_time -= plant.timestep();

                        if (monitored) {
                            const auto sample = sampler.read(
                                plant.data(), runner.snapshot());
                            const std::string issue =
                                balance::benchmark::common_diagnostic_issue(
                                    sample);
                            if (!issue.empty() && case_issue == "none") {
                                case_issue = issue;
                                std::cout
                                    << "case " << selected_case->name
                                    << " noted in " << motion.phase
                                    << " at " << plant.data().time
                                    << " s: " << issue << '\n';
                            }
                            if (issue == "non_finite_telemetry") {
                                case_finished = true;
                                accumulated_time = 0.0;
                                break;
                            }
                        }
                        continue;
                    }

                    if (previous_motion != BC_MOTION_ACTIVE &&
                        snapshot.state_machine.motion ==
                            BC_MOTION_ACTIVE) {
                        balance_start_time = plant.data().time;
                    }
                    previous_motion = snapshot.state_machine.motion;
                    motion = keyboard_drive ?
                        make_keyboard_motion_target(
                            snapshot, viewer.keyboard_drive_input()) :
                        make_motion_target(
                            snapshot, balance_start_time,
                            plant.data().time);
                    command.system_enabled = static_cast<uint8_t>(
                        plant.data().time >= 2.0);
                    command.balance_restart =
                        command.system_enabled &&
                        snapshot.state_machine.system == BC_SYSTEM_OFF;
                    if (snapshot.state_machine.motion == BC_MOTION_ACTIVE) {
                        if (!virtual_gimbal_initialized) {
                            virtual_gimbal.reset(
                                snapshot.state.value[BC_STATE_PSI]);
                            virtual_gimbal_initialized = true;
                        }
                        virtual_gimbal.update(
                            motion.gimbal_yaw_rate,
                            static_cast<float>(plant.timestep()));
                        command.forward_velocity = motion.forward_velocity;
                    } else {
                        virtual_gimbal.reset(
                            snapshot.state.value[BC_STATE_PSI]);
                        virtual_gimbal_initialized = false;
                    }
                    displayed_gimbal = virtual_gimbal.state();
                    const bc_gimbal_feedback_t gimbal_feedback =
                        virtual_gimbal_initialized ?
                            virtual_gimbal.feedback(
                                snapshot.state.value[BC_STATE_PSI],
                                snapshot.state.value[BC_STATE_DPSI]) :
                            bc_gimbal_feedback_t{};
                    runner.step(command, gimbal_feedback);
                    accumulated_time -= plant.timestep();
                }
            }

            if (plant.data().time >= next_state_print) {
                print_state(motion.phase, runner.snapshot().state);
                next_state_print += 0.5;
            }

            if (selected_case != nullptr) {
                std::string title = "rm-balance-sim | ";
                title += selected_case->name;
                title += " | ";
                title += motion.phase;
                if (selected_leg_length) {
                    title += " | L=";
                    title += std::to_string(*selected_leg_length);
                    title += " m";
                }
                if (case_issue != "none") {
                    title += " | issue: ";
                    title += case_issue;
                }
                if (case_finished) title += " | complete - R to replay";
                title += " | ";
                title += bc_chassis_alignment_name(
                    runner.snapshot().state_machine.alignment);
                title += " | error=";
                title += std::to_string(runner.snapshot().heading_error);
                title += " rad | gimbal=";
                title += std::to_string(displayed_gimbal.world_yaw_rate);
                title += " rad/s";
                viewer.set_title(title);
            } else if (keyboard_drive) {
                std::string title = "rm-balance-sim | keyboard | ";
                title += motion.phase;
                title += " | v=";
                title += std::to_string(
                    runner.snapshot().mapped_forward_velocity);
                title += " m/s | gimbal=";
                title += std::to_string(displayed_gimbal.world_yaw_rate);
                title += " rad/s | ";
                title += bc_chassis_alignment_name(
                    runner.snapshot().state_machine.alignment);
                title += " | error=";
                title += std::to_string(runner.snapshot().heading_error);
                title += " rad | WASD/arrows, Shift boost, Space, R, Esc";
                viewer.set_title(title);
            }

            viewer.set_virtual_gimbal_heading(
                displayed_gimbal.world_yaw,
                runner.snapshot().state_machine.motion == BC_MOTION_ACTIVE);
            viewer.render(plant.data());
            viewer.poll_events();
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_sim: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
