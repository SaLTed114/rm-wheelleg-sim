#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>

#include "balance/math_utils.h"
#include "ramp_jump.hpp"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: rm_balance_ramp_jump "
                     "<model.xml> <output-directory>\n";
        return EXIT_FAILURE;
    }

    try {
        balance::benchmark::RampJumpBenchmark benchmark(argv[1], argv[2]);
        for (const auto &spec : balance::benchmark::ramp_jump_cases()) {
            const auto result = benchmark.run(spec);
            std::cout << std::left << std::setw(20) << result.name
                      << " entry_v=" << result.ramp_entry_velocity
                      << " takeoff_vx=" << result.takeoff_velocity_x
                      << " takeoff_vz=" << result.takeoff_velocity_z
                      << " landing_l=" << result.wheel_landing_distance[BC_L]
                      << " landing_r=" << result.wheel_landing_distance[BC_R]
                      << " ballistic=" << result.ballistic_distance
                      << " pitch_deg="
                      << result.maximum_pitch * 180.0 / BC_PI
                      << " complete=" << result.completed
                      << " issue=" << result.issue << '\n';
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_ramp_jump: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
