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
    bool position_feedback_enabled = true;
    bool velocity_feedback_enabled = true;
    const PerformanceCaseSpec *selected_case = nullptr;
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
        } else if (option == "--position-feedback" &&
                   position_feedback_enabled) {
            position_feedback_enabled = value != "off";
            arguments_valid = !position_feedback_enabled;
        } else if (option == "--velocity-feedback" &&
                   velocity_feedback_enabled) {
            velocity_feedback_enabled = value != "off";
            arguments_valid = !velocity_feedback_enabled;
        } else {
            arguments_valid = false;
        }
    }
    arguments_valid = arguments_valid &&
        !(suite != Suite::baseline && selected_case != nullptr);
    if (!arguments_valid) {
        std::cerr
            << "usage: rm_balance_performance <model.xml> <output-directory> "
               "[--suite forward-acceleration|yaw-acceleration] "
               "[--case <case-name>] "
               "[--leg-length <metres>] [--trace-stride <steps>] "
               "[--roll-restraint on] "
               "[--forward-observation base-truth|contact-gated] "
               "[--position-feedback off] [--velocity-feedback off]\n";
        return EXIT_FAILURE;
    }

    try {
        const PerformanceBenchmarkConfig config{
            leg_length,
            trace_stride.value_or(kDefaultTraceStride),
            forward_observation,
            roll_restrained,
            position_feedback_enabled,
            velocity_feedback_enabled,
        };
        PerformanceBenchmark benchmark(argv[1], argv[2], config);
        std::vector<PerformanceCaseSpec> cases;
        if (selected_case != nullptr) {
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
