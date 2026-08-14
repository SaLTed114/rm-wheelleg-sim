#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "performance/performance_benchmark.hpp"

namespace {

using balance::benchmark::PerformanceBenchmark;
using balance::benchmark::PerformanceBenchmarkConfig;
using balance::benchmark::PerformanceCaseKind;
using balance::benchmark::PerformanceCaseSpec;

bool run_case(
    PerformanceBenchmark &benchmark,
    const PerformanceCaseSpec &spec
) {
    const auto result = benchmark.run(spec);
    std::cout << spec.name
              << " valid=" << result.valid
              << " response=" << result.response_pass
              << " stop=" << result.stop_pass
              << " contact_free=" << result.contact_free
              << " unsaturated=" << result.unsaturated
              << " issue=" << result.issue << '\n';
    return result.completed && result.valid && result.response_pass &&
        result.stop_pass && result.contact_free && result.unsaturated;
}

} // namespace

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: mujoco_impact_observer_matrix_test "
                     "<model.xml> <output-directory>\n";
        return EXIT_FAILURE;
    }

    PerformanceBenchmarkConfig config{};
    config.leg_length = 0.38;
    config.forward_acceleration_rate = 3.0;
    config.trace_stride = 1U;
    PerformanceBenchmark benchmark(argv[1], argv[2], config);

    PerformanceCaseSpec forward{};
    forward.name = "impact_negative_forward";
    forward.kind = PerformanceCaseKind::forward_response;
    forward.forward_target = 2.0;
    forward.forward_rate = 3.0;
    forward.target_hold_seconds = 2.0;

    PerformanceCaseSpec turn_positive{};
    turn_positive.name = "impact_negative_turn_pos";
    turn_positive.kind = PerformanceCaseKind::steady_turn;
    turn_positive.forward_target = 2.0;
    turn_positive.yaw_target = 0.2;
    turn_positive.forward_rate = 3.0;
    turn_positive.yaw_rate = 1.0;
    turn_positive.target_hold_seconds = 2.0;

    PerformanceCaseSpec turn_negative = turn_positive;
    turn_negative.name = "impact_negative_turn_neg";
    turn_negative.yaw_target = -0.2;

    bool passed = run_case(benchmark, forward);
    passed = run_case(benchmark, turn_positive) && passed;
    passed = run_case(benchmark, turn_negative) && passed;
    if (!passed) {
        std::cerr << "impact observer negative matrix failed\n";
        return EXIT_FAILURE;
    }
    if (!std::filesystem::exists(
            std::filesystem::path(argv[2]) / "trace.csv")) {
        std::cerr << "impact observer negative trace is missing\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
