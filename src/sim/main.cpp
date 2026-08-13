#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

#include "drop/drop_benchmark.hpp"
#include "drop/platform_drop.hpp"
#include "performance/performance_scenario.hpp"
#include "simulation_app.hpp"

namespace {

void print_usage() {
    std::cerr
        << "usage: rm_balance_sim <model.xml> [--keyboard] "
           "[--trace <csv-path>] "
           "[--case <case-name>] "
           "[--wheel-clearance <metres>] "
           "[--leg-length <metres>] "
           "[--yaw-acceleration-feedforward <scale>]\n"
        << "keyboard mode: W/S forward/reverse, A/D left/right, "
           "Shift boosts forward speed, "
           "Space pause, R reset, Esc quit\n"
        << "available performance cases:\n";
    for (const auto &spec :
         balance::benchmark::formal_performance_cases()) {
        std::cerr << "  " << spec.name << '\n';
    }
    for (const auto &spec :
         balance::benchmark::trajectory_performance_cases()) {
        std::cerr << "  " << spec.name << '\n';
    }
    std::cerr << "available drop cases:\n";
    for (const auto &spec : balance::benchmark::drop_exploration_cases()) {
        std::cerr << "  " << balance::benchmark::drop_case_name(spec) << '\n';
    }
    std::cerr << "available platform drop cases:\n";
    for (const auto &spec : balance::benchmark::platform_drop_cases()) {
        std::cerr << "  "
                  << balance::benchmark::platform_drop_case_name(spec)
                  << '\n';
    }
    std::cerr << "available active landing cases:\n";
    for (const auto &spec :
         balance::benchmark::platform_active_landing_cases()) {
        std::cerr << "  "
                  << balance::benchmark::platform_drop_case_name(spec)
                  << '\n';
    }
}

bool parse_arguments(
    const int argc,
    char **argv,
    balance::sim::SimulationAppOptions &options) {
    if (argc < 2) return false;
    options.model_path = argv[1];

    for (int index = 2; index < argc;) {
        const std::string option = argv[index];
        if (option == "--keyboard" && !options.keyboard_drive) {
            options.keyboard_drive = true;
            ++index;
            continue;
        }
        if (index + 1 >= argc) return false;

        const std::string value = argv[index + 1];
        if (option == "--case" && options.performance_case == nullptr &&
            options.drop_case == nullptr &&
            options.platform_drop_case == nullptr) {
            options.performance_case =
                balance::benchmark::find_performance_case(value);
            if (options.performance_case == nullptr) {
                options.drop_case =
                    balance::benchmark::find_drop_case(value);
            }
            if (options.performance_case == nullptr &&
                options.drop_case == nullptr) {
                options.platform_drop_case =
                    balance::benchmark::find_platform_drop_case(value);
            }
            if (options.performance_case == nullptr &&
                options.drop_case == nullptr &&
                options.platform_drop_case == nullptr) {
                std::cerr << "unknown case: " << value << '\n';
                return false;
            }
        } else if (option == "--trace" && !options.trace_path &&
                   !value.empty()) {
            options.trace_path = std::filesystem::path(value);
        } else if (option == "--wheel-clearance" &&
                   !options.drop_wheel_clearance) {
            std::size_t consumed = 0U;
            try {
                options.drop_wheel_clearance = std::stod(value, &consumed);
            } catch (const std::exception &) {
                return false;
            }
            if (consumed != value.size() ||
                !std::isfinite(*options.drop_wheel_clearance) ||
                *options.drop_wheel_clearance <= 0.0) {
                return false;
            }
        } else if (option == "--leg-length" && !options.leg_length) {
            std::size_t consumed = 0U;
            try {
                options.leg_length = std::stof(value, &consumed);
            } catch (const std::exception &) {
                return false;
            }
            if (consumed != value.size() ||
                !std::isfinite(*options.leg_length) ||
                *options.leg_length <= 0.0F) {
                return false;
            }
        } else if (option == "--yaw-acceleration-feedforward" &&
                   !options.yaw_acceleration_feedforward_scale) {
            std::size_t consumed = 0U;
            try {
                options.yaw_acceleration_feedforward_scale =
                    std::stof(value, &consumed);
            } catch (const std::exception &) {
                return false;
            }
            if (consumed != value.size() ||
                !std::isfinite(
                    *options.yaw_acceleration_feedforward_scale) ||
                *options.yaw_acceleration_feedforward_scale < 0.0F) {
                return false;
            }
        } else {
            return false;
        }
        index += 2;
    }
    const bool case_selected = options.performance_case != nullptr ||
        options.drop_case != nullptr ||
        options.platform_drop_case != nullptr;
    return !(options.keyboard_drive && case_selected) &&
        !(options.trace_path && case_selected) &&
        (!options.drop_wheel_clearance ||
         (options.drop_case != nullptr &&
          options.platform_drop_case == nullptr));
}

} // namespace

int main(int argc, char **argv) {
    balance::sim::SimulationAppOptions options;
    if (!parse_arguments(argc, argv, options)) {
        print_usage();
        return EXIT_FAILURE;
    }

    try {
        balance::sim::run_simulation_app(options);
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_sim: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
