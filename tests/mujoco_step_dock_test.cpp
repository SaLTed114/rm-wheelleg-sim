#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include "balance/math_utils.h"
#include "step/step_dock.hpp"

namespace {

bool complete_result(
    const balance::benchmark::StepDockResult &result,
    const std::filesystem::path &output_directory
) {
    const bool cut_timing_valid = result.spec.production_task ?
        result.control_cut_delay >= 0.0 &&
            result.control_cut_delay <= 0.010 :
        std::abs(result.control_cut_delay -
                 result.spec.cut_delay_seconds) <= 1.0e-9;
    const bool collision_complete = result.measurement_complete &&
        result.finite &&
        result.balance_engaged && result.start_ready &&
        result.contact_detected &&
        !result.body_contact_before_trigger && result.control_cut &&
        result.maximum_post_cut_actuation <= 1.0e-8 &&
        std::isfinite(result.collision_clearance) &&
        cut_timing_valid &&
        std::filesystem::exists(
            output_directory / result.name / "trace.csv");
    return collision_complete &&
        (!result.spec.require_speed_stable || result.speed_stable) &&
        (!result.spec.production_task || result.retained_on_platform) &&
        result.collision_velocity >= 1.0;
}

bool complete_acceleration_collision(
    const balance::benchmark::StepDockResult &result,
    const std::filesystem::path &output_directory
) {
    return result.measurement_complete && result.finite &&
        result.balance_engaged && result.start_ready &&
        result.contact_detected && !result.body_contact_before_trigger &&
        result.control_cut && result.maximum_post_cut_actuation <= 1.0e-8 &&
        std::isfinite(result.collision_clearance) &&
        std::isfinite(result.collision_travel) &&
        result.issue == "none" &&
        std::abs(result.control_cut_delay -
                 result.spec.cut_delay_seconds) <= 1.0e-9 &&
        std::filesystem::exists(
            output_directory / result.name / "trace.csv");
}

void print_result(const balance::benchmark::StepDockResult &result) {
    std::cout << result.name
              << " collision_v=" << result.collision_velocity
              << " collision_yaw_deg="
              << result.collision_world_heading * 180.0 / BC_PI
              << " travel=" << result.collision_travel
              << " cut_delay=" << result.control_cut_delay
              << " contact=" << result.first_contact_pair
              << " passive=" << result.passively_supported
              << " issue=" << result.issue << '\n';
}

} // namespace

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: mujoco_step_dock_test "
                     "<model.xml> <output-directory>\n";
        return EXIT_FAILURE;
    }

    const auto &cases = balance::benchmark::step_dock_cases();
    if (cases.size() != 2U ||
        balance::benchmark::find_step_dock_case(
            "step_dock_passive") == nullptr ||
        balance::benchmark::find_step_dock_case(
            "step_dock_delay_10ms") == nullptr ||
        balance::benchmark::find_step_dock_case("missing") != nullptr) {
        std::cerr << "step dock registry is incorrect\n";
        return EXIT_FAILURE;
    }

    const std::filesystem::path output_directory = argv[2];
    balance::benchmark::StepDockBenchmark benchmark(argv[1], output_directory);
    const auto production = benchmark.run(cases.front());
    print_result(production);
    if (!complete_result(production, output_directory) ||
        std::abs(production.collision_world_heading) >
            5.0 * BC_PI / 180.0) {
        std::cerr << "production step dock task failed\n";
        return EXIT_FAILURE;
    }

    const auto calibration = benchmark.run(cases.back());
    print_result(calibration);
    if (!complete_result(calibration, output_directory)) {
        std::cerr << "step dock heading calibration failed\n";
        return EXIT_FAILURE;
    }

    constexpr double target_heading_degrees[] = {2.0, -2.0};
    for (const double target_degrees : target_heading_degrees) {
        auto spec = cases.back();
        spec.approach_heading_radians = target_degrees * BC_PI / 180.0;
        spec.initial_heading_radians = bc_wrap_angle(
            spec.approach_heading_radians -
            calibration.collision_world_heading);
        const auto result = benchmark.run(spec);
        print_result(result);
        const double heading_error = bc_wrap_angle(
            result.collision_world_heading -
            spec.approach_heading_radians);
        if (!complete_result(result, output_directory) ||
            std::abs(heading_error) > 0.10 * BC_PI / 180.0) {
            std::cerr << "step dock did not reach its calibrated collision "
                         "heading\n";
            return EXIT_FAILURE;
        }
    }

    constexpr double acceleration_gaps[] = {0.3, 0.6, 0.9, 1.2, 1.5};
    for (const double target_travel : acceleration_gaps) {
        auto spec = cases.back();
        spec.platform_gap_at_acceleration = target_travel;
        spec.require_speed_stable = false;
        const auto travel_millimeters = static_cast<long>(
            std::lround(1000.0 * target_travel));
        const std::filesystem::path calibration_directory =
            output_directory / "acceleration-calibration" /
            (std::to_string(travel_millimeters) + "mm");
        balance::benchmark::StepDockBenchmark distance_calibration(
            argv[1], calibration_directory);
        const auto distance_calibration_result =
            distance_calibration.run(spec);
        print_result(distance_calibration_result);
        if (!complete_acceleration_collision(
                distance_calibration_result, calibration_directory)) {
            std::cerr << "step dock acceleration calibration failed\n";
            return EXIT_FAILURE;
        }
        spec.platform_gap_at_acceleration += target_travel -
            distance_calibration_result.collision_travel;
        spec.initial_heading_radians = bc_wrap_angle(
            -distance_calibration_result.collision_world_heading);
        spec.target_collision_travel = target_travel;
        const auto result = benchmark.run(spec);
        print_result(result);
        const double heading_error = bc_wrap_angle(
            result.collision_world_heading);
        if (!complete_acceleration_collision(result, output_directory) ||
            std::abs(result.collision_travel - target_travel) > 0.01 ||
            std::abs(heading_error) > 0.10 * BC_PI / 180.0) {
            std::cerr << "step dock acceleration collision sample failed\n";
            return EXIT_FAILURE;
        }
    }
    if (!std::filesystem::exists(
            std::filesystem::path(argv[2]) / "summary.csv")) {
        std::cerr << "step dock summary is missing\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
