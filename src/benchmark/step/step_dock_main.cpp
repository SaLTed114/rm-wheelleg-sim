#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>

#include "balance/math_utils.h"
#include "step_dock.hpp"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: rm_balance_step_dock "
                     "<model.xml> <output-directory>\n";
        return EXIT_FAILURE;
    }

    try {
        balance::benchmark::StepDockBenchmark benchmark(argv[1], argv[2]);
        for (const auto &spec : balance::benchmark::step_dock_cases()) {
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
                      << " settled=" << result.passively_supported
                      << " top_contact="
                      << result.final_base_top_contact_ratio
                      << " leg=" << result.final_leg_length[BC_L]
                      << '/' << result.final_leg_length[BC_R]
                      << " zero_peak="
                      << result.maximum_post_cut_actuation
                      << " issue=" << result.issue << '\n';
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_step_dock: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
