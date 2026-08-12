#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "drop/drop_benchmark.hpp"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: mujoco_drop_test <model.xml> <output-directory>\n";
        return EXIT_FAILURE;
    }

    balance::benchmark::DropBenchmark benchmark(argv[1], argv[2]);
    for (const auto &spec : balance::benchmark::drop_exploration_cases()) {
        const auto result = benchmark.run(spec);
        benchmark.write_summary(result);
        if (!result.completed || !result.finite ||
            result.attitude_diverged ||
            !result.balance_engaged || !result.touchdown ||
            !result.support_air_detected[BC_L] ||
            !result.support_air_detected[BC_R] ||
            !result.support_ground_detected[BC_L] ||
            !result.support_ground_detected[BC_R]) {
            std::cerr << result.name << " did not complete a finite drop\n";
            return EXIT_FAILURE;
        }
        for (const double clearance : result.release_clearance) {
            if (std::abs(clearance - spec.wheel_clearance) > 2.0e-3) {
                std::cerr << result.name
                          << " has incorrect release clearance: "
                          << clearance << '\n';
                return EXIT_FAILURE;
            }
        }
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            const double air_delay =
                result.support_air_time[side] - result.release_time;
            const double touchdown_delay =
                result.support_ground_time[side] - result.touchdown_time;
            if (air_delay < 0.020 || air_delay > 0.050 ||
                touchdown_delay < 0.0 || touchdown_delay > 0.025) {
                std::cerr << result.name
                          << " has excessive support-force delay: air="
                          << air_delay << ", touchdown="
                          << touchdown_delay << '\n';
                return EXIT_FAILURE;
            }
        }
        if (!std::filesystem::exists(
                std::filesystem::path(argv[2]) /
                result.name / "trace.csv")) {
            std::cerr << result.name << " did not produce a trace\n";
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}
