#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "balance/math_utils.h"
#include "step/step_dock.hpp"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: mujoco_step_dock_test "
                     "<model.xml> <output-directory>\n";
        return EXIT_FAILURE;
    }

    const auto &cases = balance::benchmark::step_dock_cases();
    if (cases.size() != 1U ||
        balance::benchmark::find_step_dock_case(
            "step_dock_complete") == nullptr ||
        balance::benchmark::find_step_dock_case(
            "step_dock_passive") != nullptr ||
        balance::benchmark::find_step_dock_case(
            "step_dock_delay_10ms") != nullptr ||
        balance::benchmark::find_step_dock_case(
            "step_dock_transfer_preview") != nullptr ||
        balance::benchmark::find_step_dock_case(
            "step_dock_transfer_low_friction_preview") != nullptr ||
        balance::benchmark::find_step_dock_case(
            "step_dock_rebalance_preview") != nullptr) {
        std::cerr << "step dock registry is incorrect\n";
        return EXIT_FAILURE;
    }

    const std::filesystem::path output_directory = argv[2];
    balance::benchmark::StepDockBenchmark benchmark(argv[1], output_directory);
    const auto result = benchmark.run(cases.front());
    std::cout << result.name
              << " collision_v=" << result.collision_velocity
              << " collision_yaw_deg="
              << result.collision_world_heading * 180.0 / BC_PI
              << " hold_margin="
              << result.hold_window_minimum_wheel_edge_margin
              << " hold_contact="
              << result.hold_window_both_wheel_top_contact_ratio
              << " max_joint_request="
              << result.maximum_post_impact_joint_request
              << " max_recovery_wheel_request="
              << result.maximum_recovery_wheel_request
              << " retained=" << result.retained_on_platform
              << " settled=" << result.final_settled
              << " issue=" << result.issue << '\n';

    const bool passed = result.name == "step_dock_complete" &&
        result.spec.production_task && result.measurement_complete &&
        result.finite && result.balance_engaged && result.start_ready &&
        result.contact_detected && !result.body_contact_before_trigger &&
        result.control_cut && result.control_cut_delay >= 0.0 &&
        result.control_cut_delay <= 0.010 &&
        result.collision_velocity >= 1.0 &&
        std::abs(result.collision_world_heading) <=
            5.0 * BC_PI / 180.0 &&
        result.hold_window_minimum_wheel_edge_margin > 0.0 &&
        result.hold_window_both_wheel_top_contact_ratio >= 0.99 &&
        result.final_wheel_top_contact_ratio >= 0.99 &&
        result.retained_on_platform && result.final_settled &&
        result.recovery_reference_captured &&
        !result.post_impact_joint_saturated &&
        result.maximum_post_impact_joint_request < 40.0 &&
        result.maximum_recovery_wheel_request > 6.32 &&
        result.issue == "none" &&
        std::filesystem::exists(
            output_directory / result.name / "trace.csv");
    if (!passed) {
        std::cerr << "production step dock completion failed\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
