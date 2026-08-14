#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>

#include "balance/math_utils.h"
#include "jump_impulse.hpp"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: rm_balance_jump_impulse "
                     "<model.xml> <output-directory>\n";
        return EXIT_FAILURE;
    }

    try {
        balance::benchmark::JumpImpulseBenchmark benchmark(argv[1], argv[2]);
        for (const auto &spec : balance::benchmark::jump_impulse_cases()) {
            const auto result = benchmark.run(spec);
            std::cout << std::left << std::setw(22) << result.name
                      << " hold=" << result.applied_hold_seconds
                      << " release=" << result.release_reason
                      << " takeoff=" << result.took_off
                      << " com_vz="
                      << result.takeoff_com_vertical_velocity
                      << " impulse=" << result.net_ground_impulse
                      << " clearance=" << result.maximum_wheel_clearance
                      << " support_delay=" << result.support_airborne_delay
                      << " landed=" << result.both_wheels_landed
                      << " recovered=" << result.support_recovered
                      << " joint_peak="
                      << result.maximum_joint_torque_request
                      << " issue=" << result.issue << '\n';
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_jump_impulse: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
