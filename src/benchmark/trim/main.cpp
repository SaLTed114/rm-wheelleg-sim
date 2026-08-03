#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

#include "balance/math_utils.h"
#include "trim/trim_scanner.hpp"

namespace {

using balance::benchmark::TrimResult;
using balance::benchmark::TrimScanConfig;
using balance::benchmark::TrimScanner;

bool parse_double(const std::string &value, double &output) {
    std::size_t consumed = 0U;
    try {
        output = std::stod(value, &consumed);
    } catch (const std::exception &) {
        return false;
    }
    return consumed == value.size() && std::isfinite(output);
}

bool parse_size(const std::string &value, std::size_t &output) {
    std::size_t consumed = 0U;
    try {
        output = std::stoul(value, &consumed);
    } catch (const std::exception &) {
        return false;
    }
    return consumed == value.size() && output > 0U;
}

bool parse_arguments(
    const int argc, char **argv, TrimScanConfig &config
) {
    if (argc < 3 || (argc - 3) % 2 != 0) return false;

    for (int index = 3; index < argc; index += 2) {
        const std::string option = argv[index];
        const std::string value = argv[index + 1];
        if (option == "--leg-length") {
            if (!parse_double(value, config.leg_length)) return false;
        } else if (option == "--offset-min-deg") {
            if (!parse_double(value, config.offset_minimum_deg)) return false;
        } else if (option == "--offset-max-deg") {
            if (!parse_double(value, config.offset_maximum_deg)) return false;
        } else if (option == "--offset-step-deg") {
            if (!parse_double(value, config.offset_step_deg)) return false;
        } else if (option == "--trace-stride") {
            if (!parse_size(value, config.trace_stride)) return false;
        } else {
            return false;
        }
    }

    return config.leg_length > 0.0 && config.offset_step_deg > 0.0 &&
        config.offset_minimum_deg <= config.offset_maximum_deg;
}

void print_result(const TrimResult &result) {
    std::cout << std::fixed << std::setprecision(3)
              << "offset=" << std::setw(7) << result.offset_deg
              << " deg pitch=" << std::setw(7)
              << result.pitch.mean() * 180.0 / BC_PI
              << " deg ds=" << std::setw(8) << result.velocity.mean()
              << " m/s slope=" << std::setw(8)
              << result.velocity_trend.slope()
              << " contact=" << result.common.wheel_contact_ratio()
              << " candidate=" << result.candidate;
    if (result.issue != "none") std::cout << " issue=" << result.issue;
    std::cout << '\n';
}

} // namespace

int main(int argc, char **argv) {
    TrimScanConfig config;
    if (!parse_arguments(argc, argv, config)) {
        std::cerr
            << "usage: rm_balance_trim_scan <model.xml> <output-directory> "
               "[--leg-length <metres>] "
               "[--offset-min-deg <degrees>] "
               "[--offset-max-deg <degrees>] "
               "[--offset-step-deg <degrees>] "
               "[--trace-stride <steps>]\n";
        return EXIT_FAILURE;
    }

    try {
        TrimScanner scanner(argv[1], argv[2], config);
        const int case_count = static_cast<int>(std::floor(
            (config.offset_maximum_deg - config.offset_minimum_deg) /
                config.offset_step_deg + 1.0e-9)) + 1;
        for (int index = 0; index < case_count; ++index) {
            const double offset = config.offset_minimum_deg +
                static_cast<double>(index) * config.offset_step_deg;
            TrimResult result = scanner.run(offset);
            scanner.write_summary(result);
            print_result(result);
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_trim_scan: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
