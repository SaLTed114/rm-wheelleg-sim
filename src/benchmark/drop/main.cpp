#include <cstdlib>
#include <cmath>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>

#include "balance/math_utils.h"
#include "drop_benchmark.hpp"
#include "platform_drop.hpp"

namespace {

void print_result(const balance::benchmark::DropResult &result) {
    std::cout << std::left << std::setw(34) << result.name
              << " complete=" << result.completed
              << " finite=" << result.finite
              << " diverged=" << result.attitude_diverged
              << " first=" << result.first_contact_kind
              << " touchdown=" << result.touchdown
              << " t=" << result.touchdown_time - result.release_time
              << " pitch_deg="
              << result.touchdown_pitch * 180.0 / BC_PI
              << " pitch_rate=" << result.touchdown_pitch_rate
              << " post_pitch_deg="
              << result.post_max_pitch * 180.0 / BC_PI
              << " rebound=" << result.post_rebound
              << " other_contact=" << result.post_other_contact
              << '\n';
}

void print_result(const balance::benchmark::PlatformDropResult &result) {
    std::cout << std::left << std::setw(44) << result.name
              << " complete=" << result.completed
              << " recovered=" << result.recovered
              << " diverged=" << result.diverged
              << " edge_velocity=" << result.edge_velocity
              << " flight_time="
              << result.touchdown_time - result.departure_time
              << " kf_air_error="
              << result.airborne_maximum_velocity_error
              << '\n';
}

} // namespace

int main(int argc, char **argv) {
    if (argc != 3 && argc != 5) {
        std::cerr << "usage: rm_balance_drop <model.xml> <output-directory> "
                     "[--wheel-clearance <metres>]\n";
        return EXIT_FAILURE;
    }

    std::optional<double> wheel_clearance;
    if (argc == 5) {
        if (std::string(argv[3]) != "--wheel-clearance") {
            return EXIT_FAILURE;
        }
        std::size_t consumed = 0U;
        try {
            wheel_clearance = std::stod(argv[4], &consumed);
        } catch (const std::exception &) {
            return EXIT_FAILURE;
        }
        if (consumed != std::string(argv[4]).size() ||
            !std::isfinite(*wheel_clearance) || *wheel_clearance <= 0.0) {
            return EXIT_FAILURE;
        }
    }

    try {
        balance::benchmark::DropBenchmark benchmark(argv[1], argv[2]);
        for (const auto &configured_spec :
             balance::benchmark::drop_exploration_cases()) {
            auto spec = configured_spec;
            if (wheel_clearance) spec.wheel_clearance = *wheel_clearance;
            const auto result = benchmark.run(spec);
            benchmark.write_summary(result);
            print_result(result);
        }
        if (!wheel_clearance) {
            balance::benchmark::PlatformDropBenchmark platform_benchmark(
                argv[1], argv[2]);
            for (const auto &spec :
                 balance::benchmark::platform_drop_cases()) {
                const auto result = platform_benchmark.run(spec);
                platform_benchmark.write_summary(result);
                print_result(result);
            }
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_drop: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
