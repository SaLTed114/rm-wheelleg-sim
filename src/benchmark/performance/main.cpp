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

using balance::benchmark::ForwardVelocityObservation;
using balance::benchmark::PerformanceBenchmark;
using balance::benchmark::PerformanceBenchmarkConfig;
using balance::benchmark::PerformanceCaseKind;
using balance::benchmark::PerformanceCaseSpec;
using balance::benchmark::PerformanceResult;

bool parse_double(const std::string &text, std::optional<double> &output) {
    if (output) return false;
    std::size_t consumed = 0U;
    try {
        output = std::stod(text, &consumed);
    } catch (const std::exception &) {
        return false;
    }
    return consumed == text.size();
}

void print_result(const PerformanceResult &result) {
    std::cout << std::left << std::setw(24) << result.spec.name
              << " kind="
              << balance::benchmark::performance_case_kind_name(
                     result.spec.kind)
              << " valid=" << result.valid
              << " response=" << result.response_pass
              << " stop=" << result.stop_pass
              << " contact_free=" << result.contact_free
              << " unsaturated=" << result.unsaturated
              << " t90=" << result.t90
              << " pitch_deg=" << result.maximum_pitch * 180.0 / BC_PI
              << " v=" << result.actual_forward.mean()
              << " yaw=" << result.actual_yaw.mean()
              << " ay=" << result.lateral_acceleration.mean();
    if (result.spec.kind == PerformanceCaseKind::steady_turn) {
        std::cout << " entry_ready=" << result.entry_ready
                  << " entry_wait=" << result.entry_wait_seconds
                  << " hold_contact="
                  << result.hold.wheel_contact_ratio();
    } else if (result.spec.kind == PerformanceCaseKind::figure_eight) {
        std::cout << " closure=" << result.path_closure_error
                  << " path_contact="
                  << result.hold.wheel_contact_ratio();
    }
    if (result.issue != "none") {
        std::cout << " issue=" << result.issue << '@' << result.issue_phase;
    }
    std::cout << '\n';
}

} // namespace

int main(int argc, char **argv) {
    std::optional<double> leg_length;
    std::optional<std::size_t> trace_stride;
    bool roll_restrained = false;
    std::optional<double> yaw_acceleration_feedforward_scale;
    ForwardVelocityObservation forward_observation =
        ForwardVelocityObservation::wheel_odometry;
    const PerformanceCaseSpec *selected_case = nullptr;
    std::optional<std::string> custom_name;
    std::optional<PerformanceCaseKind> custom_kind;
    std::optional<double> custom_target;
    std::optional<double> custom_command_rate;
    std::optional<double> forward_target;
    std::optional<double> yaw_target;
    std::optional<double> forward_rate;
    std::optional<double> yaw_rate;
    std::optional<double> target_hold;
    std::optional<double> stop_settle;
    std::optional<double> standing;
    std::optional<double> coupled_forward_velocity;
    std::optional<double> forward_lead_seconds;
    bool suite_seen = false;

    bool arguments_valid = argc >= 3 && (argc - 3) % 2 == 0;
    for (int index = 3; arguments_valid && index < argc; index += 2) {
        const std::string option = argv[index];
        const std::string value = argv[index + 1];
        if (option == "--suite" && !suite_seen) {
            suite_seen = true;
            arguments_valid = value == "formal" || value == "baseline";
        } else if (option == "--leg-length") {
            arguments_valid = parse_double(value, leg_length);
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
        } else if ((option == "--kind" || option == "--axis") &&
                   !custom_kind) {
            if (value == "forward") {
                custom_kind = PerformanceCaseKind::forward_response;
            } else if (value == "heading") {
                custom_kind = PerformanceCaseKind::heading_response;
            } else if (option == "--kind" && value == "turn") {
                custom_kind = PerformanceCaseKind::steady_turn;
            } else {
                arguments_valid = false;
            }
        } else if (option == "--target") {
            arguments_valid = parse_double(value, custom_target);
        } else if (option == "--command-rate") {
            arguments_valid = parse_double(value, custom_command_rate);
        } else if (option == "--forward-target") {
            arguments_valid = parse_double(value, forward_target);
        } else if (option == "--yaw-target") {
            arguments_valid = parse_double(value, yaw_target);
        } else if (option == "--forward-rate") {
            arguments_valid = parse_double(value, forward_rate);
        } else if (option == "--yaw-rate") {
            arguments_valid = parse_double(value, yaw_rate);
        } else if (option == "--target-hold-seconds") {
            arguments_valid = parse_double(value, target_hold);
        } else if (option == "--stop-settle-seconds") {
            arguments_valid = parse_double(value, stop_settle);
        } else if (option == "--standing-seconds") {
            arguments_valid = parse_double(value, standing);
        } else if (option == "--coupled-forward") {
            arguments_valid = parse_double(value, coupled_forward_velocity);
        } else if (option == "--forward-lead-seconds") {
            arguments_valid = parse_double(value, forward_lead_seconds);
        } else if (option == "--roll-restraint" && !roll_restrained) {
            roll_restrained = value == "on";
            arguments_valid = roll_restrained;
        } else if (option == "--yaw-acceleration-feedforward") {
            arguments_valid = parse_double(
                value, yaw_acceleration_feedforward_scale);
        } else if (option == "--forward-observation" &&
                   forward_observation ==
                       ForwardVelocityObservation::wheel_odometry) {
            if (value == "base-truth") {
                forward_observation = ForwardVelocityObservation::base_truth;
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

    const bool any_custom = custom_name || custom_kind || custom_target ||
        custom_command_rate || forward_target || yaw_target || forward_rate ||
        yaw_rate || target_hold || stop_settle || standing ||
        coupled_forward_velocity || forward_lead_seconds;
    bool complete_custom = false;
    if (custom_name && custom_kind) {
        if (*custom_kind == PerformanceCaseKind::steady_turn) {
            complete_custom = forward_target && yaw_target &&
                forward_rate && yaw_rate;
        } else {
            complete_custom = custom_target && custom_command_rate;
        }
    }
    arguments_valid = arguments_valid &&
        (!any_custom || complete_custom) &&
        !(any_custom && selected_case != nullptr) &&
        !(suite_seen && (any_custom || selected_case != nullptr));

    if (!arguments_valid) {
        std::cerr
            << "usage: rm_balance_performance <model.xml> <output-directory> "
               "[--suite formal|baseline] [--case <formal-case>] "
               "[--name <name> --kind forward|heading "
               "--target <value> --command-rate <value>] "
               "[--name <name> --kind turn --forward-target <m/s> "
               "--yaw-target <rad/s> --forward-rate <m/s^2> "
               "--yaw-rate <rad/s^2>] "
               "[--standing-seconds <s>] [--target-hold-seconds <s>] "
               "[--stop-settle-seconds <s>] [--leg-length <m>] "
               "[--trace-stride <steps>] [--roll-restraint on] "
               "[--yaw-acceleration-feedforward <scale>] "
               "[--forward-observation base-truth|contact-gated]\n";
        return EXIT_FAILURE;
    }

    try {
        std::optional<double> controller_forward_rate;
        if (complete_custom &&
            *custom_kind != PerformanceCaseKind::heading_response) {
            controller_forward_rate =
                *custom_kind == PerformanceCaseKind::steady_turn ?
                    *forward_rate : *custom_command_rate;
        }
        const PerformanceBenchmarkConfig config{
            leg_length,
            controller_forward_rate,
            trace_stride.value_or(kDefaultTraceStride),
            forward_observation,
            roll_restrained,
            yaw_acceleration_feedforward_scale,
        };
        PerformanceBenchmark benchmark(argv[1], argv[2], config);
        std::vector<PerformanceCaseSpec> cases;
        if (complete_custom) {
            PerformanceCaseSpec spec{};
            spec.name = *custom_name;
            spec.kind = *custom_kind;
            if (*custom_kind == PerformanceCaseKind::forward_response) {
                spec.forward_target = *custom_target;
                spec.forward_rate = *custom_command_rate;
            } else if (*custom_kind == PerformanceCaseKind::heading_response) {
                spec.yaw_target = *custom_target;
                spec.yaw_rate = *custom_command_rate;
            } else {
                spec.forward_target = *forward_target;
                spec.yaw_target = *yaw_target;
                spec.forward_rate = *forward_rate;
                spec.yaw_rate = *yaw_rate;
            }
            if (target_hold) spec.target_hold_seconds = *target_hold;
            if (stop_settle) spec.stop_settle_seconds = *stop_settle;
            if (standing) spec.standing_seconds = *standing;
            if (coupled_forward_velocity) {
                spec.coupled_forward_velocity = *coupled_forward_velocity;
            }
            if (forward_lead_seconds) {
                spec.forward_lead_seconds = *forward_lead_seconds;
            }
            cases.push_back(spec);
        } else if (selected_case != nullptr) {
            cases.push_back(*selected_case);
        } else {
            const auto &formal =
                balance::benchmark::formal_performance_cases();
            cases.assign(formal.begin(), formal.end());
        }

        bool formal_pass = true;
        for (const PerformanceCaseSpec &spec : cases) {
            PerformanceResult result = benchmark.run(spec);
            benchmark.write_summary(result);
            print_result(result);
            if (spec.formal_acceptance) {
                formal_pass = formal_pass && result.valid &&
                    result.response_pass && result.stop_pass;
            }
        }
        return formal_pass ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_performance: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
