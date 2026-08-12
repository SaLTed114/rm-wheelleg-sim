#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "drop/platform_drop.hpp"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: mujoco_landing_suspension_test "
                     "<model.xml> <output-dir>\n";
        return EXIT_FAILURE;
    }

    const auto *spec = balance::benchmark::find_platform_drop_case(
        "platform_drop_200mm_l0p18_v2p0_leg_lqr_air_extend_l0p38_"
        "landing_suspension_k800_d120");
    if (spec == nullptr) {
        std::cerr << "landing suspension smoke case is missing\n";
        return EXIT_FAILURE;
    }

    balance::benchmark::PlatformDropBenchmark benchmark(argv[1], argv[2]);
    const auto result = benchmark.run(*spec);
    benchmark.write_summary(result);
    if (!result.completed || !result.finite || !result.recovered ||
        result.diverged || result.rebound ||
        !result.landing_recovery_started ||
        result.landing_recovery_seconds <= 0.0 ||
        result.other_contact ||
        result.maximum_applied_axial_force[BC_L] <= 0.0 ||
        result.maximum_applied_axial_force[BC_R] <= 0.0 ||
        result.post_touchdown_joint_saturation_ratio != 0.0) {
        std::cerr << "landing suspension smoke case did not recover\n";
        return EXIT_FAILURE;
    }
    if (!std::filesystem::exists(
            std::filesystem::path(argv[2]) /
            result.name / "trace.csv")) {
        std::cerr << "landing suspension smoke trace is missing\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
