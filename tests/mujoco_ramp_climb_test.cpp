#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "balance/math_utils.h"
#include "drop/ramp_climb.hpp"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: mujoco_ramp_climb_test "
                     "<model.xml> <output-directory>\n";
        return EXIT_FAILURE;
    }

    balance::benchmark::RampClimbBenchmark benchmark(argv[1], argv[2]);
    const auto result = benchmark.run();
    std::cout << "collision=" << result.clearance_collision
              << " kind=" << result.first_collision
              << " collision_x=" << result.collision_axle_x
              << " airborne_after_collision="
              << result.airborne_after_collision
              << " airborne_delay="
              << result.airborne_time - result.collision_time
              << " wheel_contact_at_airborne="
              << result.wheel_contact_at_airborne
              << " wheel_recontact="
              << result.wheel_recontact_after_airborne
              << " estimator_recovered="
              << result.support_estimator_recovered
              << " landing_recovery="
              << result.landing_recovery_started
              << " phase_recovered="
              << result.support_phase_recovered
              << " wheel_lost=" << result.wheel_reliability_lost
              << " wheel_recovered="
              << result.wheel_reliability_recovered
              << " reacquisition_used="
              << result.wheel_reacquisition_used
              << " hold_recovered=" << result.forward_hold_recovered
              << " final_velocity=" << result.final_truth_velocity
              << " final_position_error=" << result.final_position_error
              << " max_pitch_deg="
              << result.maximum_pitch * 180.0 / BC_PI << '\n';

    if (!result.balance_engaged || !result.finite ||
        !result.clearance_collision || !result.airborne_after_collision ||
        result.airborne_time < result.collision_time ||
        result.wheel_contact_at_airborne ||
        !result.wheel_recontact_after_airborne ||
        result.wheel_recontact_time < result.airborne_time ||
        !result.support_estimator_recovered ||
        !result.landing_recovery_started || result.attitude_diverged ||
        !result.wheel_reliability_lost ||
        !result.wheel_reliability_recovered ||
        !result.wheel_reacquisition_used ||
        !result.forward_hold_recovered ||
        std::abs(result.final_truth_velocity) > 0.05 ||
        std::abs(result.final_estimated_velocity) > 0.05 ||
        std::abs(result.final_wheel_velocity) > 0.05 ||
        std::abs(result.final_position_error) > 0.05 ||
        !std::filesystem::exists(
            std::filesystem::path(argv[2]) / "trace.csv") ||
        !std::filesystem::exists(
            std::filesystem::path(argv[2]) / "summary.csv")) {
        std::cerr << "17 degree ramp collision did not recover support "
                     "and wheel velocity estimation safely\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
