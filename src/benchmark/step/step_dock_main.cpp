#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>

#include "balance/math_utils.h"
#include "step_dock.hpp"

int main(int argc, char **argv) {
    if (argc != 3 && argc != 4) {
        std::cerr << "usage: rm_balance_step_dock "
                     "<model.xml> <output-directory> [case]\n";
        return EXIT_FAILURE;
    }

    try {
        balance::benchmark::StepDockBenchmark benchmark(argv[1], argv[2]);
        auto run = [&benchmark](
            const balance::benchmark::StepDockSpec &spec) {
            const auto result = benchmark.run(spec);
            std::cout << std::left << std::setw(24) << result.name
                      << " collision_v=" << result.collision_velocity
                      << " collision_yaw_deg="
                      << result.collision_world_heading * 180.0 / BC_PI
                      << " clearance=" << result.collision_clearance
                      << " min_clearance="
                      << result.minimum_approach_clearance
                      << " contact=" << result.first_contact_pair
                      << " trigger_force=" << result.trigger_contact_force
                      << " cut_delay_ms="
                      << 1000.0 * result.control_cut_delay
                      << " horizontal_residual_peak="
                      << result.delay_peak_horizontal_residual
                      << " detect_ms="
                      << 1000.0 * result.horizontal_force_detection_latency
                      << " delay_pitch_deg="
                      << result.maximum_delay_pitch * 180.0 / BC_PI
                      << " retained=" << result.retained_on_platform
                      << " settled=" << result.final_settled
                      << " top_contact="
                      << result.final_base_top_contact_ratio
                      << " hold_margin="
                      << result.hold_window_minimum_wheel_edge_margin
                      << " hold_advance="
                      << result.hold_window_maximum_base_advance
                      << " hold_pitch_deg="
                      << result.hold_window_maximum_abs_pitch * 180.0 / BC_PI
                      << " hold_wheel_contact="
                      << result.hold_window_both_wheel_top_contact_ratio
                      << " leg=" << result.final_leg_length[BC_L]
                      << '/' << result.final_leg_length[BC_R]
                      << " max_joint_request="
                      << result.maximum_post_impact_joint_request
                      << " max_recovery_wheel_request="
                      << result.maximum_recovery_wheel_request
                      << " issue=" << result.issue << '\n';
        };
        if (argc == 4) {
            const auto *spec = balance::benchmark::find_step_dock_case(
                argv[3]);
            if (spec == nullptr) {
                std::cerr << "unknown step dock case: " << argv[3] << '\n';
                return EXIT_FAILURE;
            }
            run(*spec);
        } else {
            for (const auto &spec : balance::benchmark::step_dock_cases()) {
                run(spec);
            }
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_step_dock: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
