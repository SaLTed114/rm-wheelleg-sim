#include <algorithm>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "balance/math_utils.h"
#include "performance/performance_benchmark.hpp"

namespace {

constexpr std::size_t kDefaultTraceStride = 10U;

using balance::benchmark::PerformanceBenchmark;
using balance::benchmark::PerformanceBenchmarkConfig;
using balance::benchmark::PerformanceCaseSpec;
using balance::benchmark::PerformanceResult;
using balance::benchmark::ForwardVelocityObservation;
using balance::benchmark::PerformanceAxis;

enum class Suite {
    baseline,
    forward_acceleration,
    yaw_acceleration,
};

void print_result(const PerformanceResult &result) {
    double maximum_wheel_saturation = 0.0;
    double maximum_joint_saturation = 0.0;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        maximum_wheel_saturation = std::max(
            maximum_wheel_saturation,
            result.common.wheel_saturation_ratio(side));
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            maximum_joint_saturation = std::max(
                maximum_joint_saturation,
                result.common.joint_saturation_ratio(side, joint));
        }
    }

    std::cout << std::left << std::setw(22) << result.spec.name
              << " complete=" << result.completed
              << " engaged=" << result.balance_engaged
              << " leg_range=" << result.leg_length_valid
              << " finite=" << result.common.finite()
              << " tracked=" << result.tracked
              << " settled=" << result.settled
              << " error=" << std::setw(10)
              << result.tracking_error.mean()
              << " rmse=" << std::setw(10)
              << result.tracking_error.rms()
              << " pitch=" << std::setw(8)
              << result.maximum_pitch * 180.0 / BC_PI
              << " roll=" << std::setw(8)
              << result.maximum_roll * 180.0 / BC_PI
              << " wheel_sat=" << maximum_wheel_saturation
              << " joint_sat=" << maximum_joint_saturation;
    if (result.issue != "none") {
        std::cout << " issue=" << result.issue
                  << '@' << result.issue_phase;
    }
    std::cout << '\n';
}

} // namespace

int main(int argc, char **argv) {
    Suite suite = Suite::baseline;
    std::optional<double> leg_length;
    std::optional<std::size_t> trace_stride;
    bool roll_restrained = false;
    ForwardVelocityObservation forward_observation =
        ForwardVelocityObservation::wheel_odometry;
    const PerformanceCaseSpec *selected_case = nullptr;
    std::optional<std::string> custom_name;
    std::optional<PerformanceAxis> custom_axis;
    std::optional<double> custom_target;
    std::optional<double> custom_command_rate;
    std::optional<double> custom_target_hold;
    std::optional<double> custom_stop_settle;
    std::optional<double> custom_standing;
    bool arguments_valid = argc >= 3 && (argc - 3) % 2 == 0;
    for (int index = 3; arguments_valid && index < argc; index += 2) {
        const std::string option = argv[index];
        const std::string value = argv[index + 1];
        if (option == "--suite" && suite == Suite::baseline &&
            value == "forward-acceleration") {
            suite = Suite::forward_acceleration;
        } else if (option == "--suite" && suite == Suite::baseline &&
                   value == "yaw-acceleration") {
            suite = Suite::yaw_acceleration;
        } else if (option == "--leg-length" && !leg_length) {
            std::size_t consumed = 0U;
            try {
                leg_length = std::stod(value, &consumed);
            } catch (const std::exception &) {
                arguments_valid = false;
                break;
            }
            arguments_valid = consumed == value.size();
        } else if (option == "--trace-stride" && !trace_stride) {
            std::size_t consumed = 0U;
            try {
                trace_stride = std::stoul(value, &consumed);
            } catch (const std::exception &) {
                arguments_valid = false;
                break;
            }
            arguments_valid = consumed == value.size() && *trace_stride > 0U;
        } else if (option == "--case" && selected_case == nullptr) {
            selected_case = balance::benchmark::find_performance_case(value);
            arguments_valid = selected_case != nullptr;
        } else if (option == "--name" && !custom_name && !value.empty()) {
            custom_name = value;
        } else if (option == "--axis" && !custom_axis) {
            if (value == "forward") {
                custom_axis = PerformanceAxis::forward;
            } else if (value == "yaw") {
                custom_axis = PerformanceAxis::yaw;
            } else {
                arguments_valid = false;
            }
        } else if (option == "--target" && !custom_target) {
            std::size_t consumed = 0U;
            try {
                custom_target = std::stod(value, &consumed);
            } catch (const std::exception &) {
                arguments_valid = false;
                break;
            }
            arguments_valid = consumed == value.size();
        } else if (option == "--command-rate" && !custom_command_rate) {
            std::size_t consumed = 0U;
            try {
                custom_command_rate = std::stod(value, &consumed);
            } catch (const std::exception &) {
                arguments_valid = false;
                break;
            }
            arguments_valid = consumed == value.size();
        } else if (option == "--target-hold-seconds" &&
                   !custom_target_hold) {
            std::size_t consumed = 0U;
            try {
                custom_target_hold = std::stod(value, &consumed);
            } catch (const std::exception &) {
                arguments_valid = false;
                break;
            }
            arguments_valid = consumed == value.size();
        } else if (option == "--stop-settle-seconds" &&
                   !custom_stop_settle) {
            std::size_t consumed = 0U;
            try {
                custom_stop_settle = std::stod(value, &consumed);
            } catch (const std::exception &) {
                arguments_valid = false;
                break;
            }
            arguments_valid = consumed == value.size();
        } else if (option == "--standing-seconds" && !custom_standing) {
            std::size_t consumed = 0U;
            try {
                custom_standing = std::stod(value, &consumed);
            } catch (const std::exception &) {
                arguments_valid = false;
                break;
            }
            arguments_valid = consumed == value.size();
        } else if (option == "--roll-restraint" && !roll_restrained) {
            roll_restrained = value == "on";
            arguments_valid = roll_restrained;
        } else if (option == "--forward-observation" &&
                   forward_observation ==
                       ForwardVelocityObservation::wheel_odometry) {
            if (value == "base-truth") {
                forward_observation =
                    ForwardVelocityObservation::base_truth;
            } else if (value == "contact-gated") {
                forward_observation =
                    ForwardVelocityObservation::contact_gated;
            } else {
                arguments_valid = false;
            }
        } else {
            arguments_valid = false;
        }
    }
    const bool any_custom = custom_name || custom_axis || custom_target ||
        custom_command_rate || custom_target_hold || custom_stop_settle;
    const bool any_custom_timing = custom_standing.has_value();
    const bool complete_custom = custom_name && custom_axis && custom_target &&
        custom_command_rate;
    arguments_valid = arguments_valid &&
        !(suite != Suite::baseline && selected_case != nullptr) &&
        (!(any_custom || any_custom_timing) || complete_custom) &&
        !(any_custom && (suite != Suite::baseline || selected_case != nullptr));
    if (!arguments_valid) {
        std::cerr
            << "usage: rm_balance_performance <model.xml> <output-directory> "
               "[--suite forward-acceleration|yaw-acceleration] "
               "[--case <case-name>] "
               "[--name <case-name> --axis forward|yaw "
               "--target <value> --command-rate <value> "
               "--standing-seconds <seconds> "
               "--target-hold-seconds <seconds> "
               "--stop-settle-seconds <seconds>] "
               "[--leg-length <metres>] [--trace-stride <steps>] "
               "[--roll-restraint on] "
               "[--forward-observation base-truth|contact-gated]\n";
        return EXIT_FAILURE;
    }

    try {
        const PerformanceBenchmarkConfig config{
            leg_length,
            trace_stride.value_or(kDefaultTraceStride),
            forward_observation,
            roll_restrained,
        };
        PerformanceBenchmark benchmark(argv[1], argv[2], config);
        std::vector<PerformanceCaseSpec> cases;
        if (complete_custom) {
            PerformanceCaseSpec spec{
                *custom_name, *custom_axis, *custom_target,
                *custom_command_rate};
            if (custom_target_hold) {
                spec.target_hold_seconds = *custom_target_hold;
            }
            if (custom_stop_settle) {
                spec.stop_settle_seconds = *custom_stop_settle;
            }
            if (custom_standing) {
                spec.standing_seconds = *custom_standing;
            }
            cases.push_back(spec);
        } else if (selected_case != nullptr) {
            cases.push_back(*selected_case);
        } else if (suite == Suite::forward_acceleration) {
            const auto &acceleration_cases =
                balance::benchmark::forward_acceleration_cases();
            cases.assign(
                acceleration_cases.begin(), acceleration_cases.end());
        } else if (suite == Suite::yaw_acceleration) {
            const auto &acceleration_cases =
                balance::benchmark::yaw_acceleration_cases();
            cases.assign(
                acceleration_cases.begin(), acceleration_cases.end());
        } else {
            const auto &baseline_cases =
                balance::benchmark::performance_cases();
            cases.assign(baseline_cases.begin(), baseline_cases.end());
        }
        for (const PerformanceCaseSpec &spec : cases) {
            PerformanceResult result = benchmark.run(spec);
            benchmark.write_summary(result);
            print_result(result);
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_performance: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
