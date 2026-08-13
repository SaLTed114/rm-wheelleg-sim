#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "drop/platform_drop.hpp"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: mujoco_active_landing_test "
                     "<model.xml> <output-dir>\n";
        return EXIT_FAILURE;
    }

    const auto *spec = balance::benchmark::find_platform_drop_case(
        "platform_drop_200mm_l0p18_v2p0_leg_lqr_landing_controller");
    if (spec == nullptr) {
        std::cerr << "active landing smoke case is missing\n";
        return EXIT_FAILURE;
    }

    balance::benchmark::PlatformDropBenchmark benchmark(argv[1], argv[2]);
    const auto result = benchmark.run(*spec);
    benchmark.write_summary(result);
    if (!result.completed || !result.finite || !result.recovered ||
        result.diverged || result.rebound ||
        !result.landing_recovery_started ||
        result.landing_recovery_seconds <= 0.0 ||
        result.shadow_airborne_delay <= 0.0 ||
        result.shadow_airborne_delay > 0.045 ||
        result.shadow_landing_delay <= 0.0 ||
        result.shadow_landing_delay > 0.015 ||
        result.shadow_recover_delay <= result.shadow_landing_delay ||
        result.shadow_ground_delay <= result.shadow_recover_delay ||
        result.maximum_applied_axial_force[BC_L] <= 0.0 ||
        result.maximum_applied_axial_force[BC_R] <= 0.0 ||
        result.post_touchdown_joint_saturation_ratio != 0.0) {
        std::cerr << "active landing smoke case did not recover\n";
        return EXIT_FAILURE;
    }
    if (!std::filesystem::exists(
            std::filesystem::path(argv[2]) /
            result.name / "trace.csv")) {
        std::cerr << "active landing smoke trace is missing\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
