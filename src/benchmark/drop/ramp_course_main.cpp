#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>

#include "balance/math_utils.h"
#include "ramp_course.hpp"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: rm_balance_ramp_course "
                     "<model.xml> <output-directory>\n";
        return EXIT_FAILURE;
    }

    try {
        balance::benchmark::RampCourseBenchmark benchmark(argv[1], argv[2]);
        for (const auto &spec : balance::benchmark::ramp_course_cases()) {
            const auto result = benchmark.run(spec);
            std::cout << std::left << std::setw(31) << result.name
                      << " complete=" << result.completed
                      << " collision=" << result.non_wheel_collision
                      << " clearance=" << result.minimum_clearance
                      << " geom=" << result.minimum_clearance_geom
                      << " pitch_deg="
                      << result.maximum_pitch * 180.0 / BC_PI
                      << " yaw_deg="
                      << result.maximum_yaw_error * 180.0 / BC_PI
                      << " airborne=" << result.airborne_event_count
                      << " issue=" << result.issue << '\n';
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_ramp_course: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
