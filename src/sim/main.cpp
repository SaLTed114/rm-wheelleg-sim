#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

#include "performance/performance_scenario.hpp"
#include "simulation_app.hpp"

namespace {

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
        if (option == "--case" && options.performance_case == nullptr) {
            options.performance_case =
                balance::benchmark::find_performance_case(value);
            if (options.performance_case == nullptr) {
                std::cerr << "unknown performance case: " << value << '\n';
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
        } else {
            return false;
        }
        index += 2;
    }
    return !(options.keyboard_drive && options.performance_case != nullptr);
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
