#include <cstdlib>
#include <array>
#include <filesystem>
#include <iostream>

#include "drop/platform_drop.hpp"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr
            << "usage: mujoco_platform_drop_test <model.xml> <output-dir>\n";
        return EXIT_FAILURE;
    }

    const auto &cases = balance::benchmark::platform_drop_cases();
    if (cases.size() != 2U ||
        balance::benchmark::find_platform_drop_case(
            "platform_drop_200mm_l0p18_v2p0_length_only_"
            "air_extend_l0p38") == nullptr ||
        balance::benchmark::find_platform_drop_case(
            "platform_drop_200mm_l0p18_v2p0_leg_lqr_"
            "air_extend_l0p38") == nullptr ||
        balance::benchmark::platform_active_landing_cases().size() != 4U ||
        balance::benchmark::find_platform_drop_case(
            "platform_drop_400mm_l0p18_v-2p5_leg_lqr_"
            "landing_controller") == nullptr) {
        std::cerr << "platform drop case registry is incorrect\n";
        return EXIT_FAILURE;
    }

    balance::benchmark::PlatformDropBenchmark benchmark(argv[1], argv[2]);
    std::array<balance::benchmark::PlatformDropResult, 2> results{};
    for (std::size_t index = 0; index < cases.size(); ++index) {
        results[index] = benchmark.run(cases[index]);
        const auto &result = results[index];
        benchmark.write_summary(results[index]);
        std::cout << result.name
                  << " complete=" << result.completed
                  << " recovered=" << result.recovered
                  << " diverged=" << result.diverged
                  << " issue=" << result.issue << '\n';
        if (!result.completed || !result.finite ||
            !result.balance_engaged || !result.speed_stable ||
            !result.left_platform || !result.touchdown ||
            !result.recovered || result.diverged || result.other_contact) {
            std::cerr << result.name
                      << " did not complete the platform drop sequence\n";
            return EXIT_FAILURE;
        }
        if (!std::filesystem::exists(
                std::filesystem::path(argv[2]) /
                result.name / "trace.csv")) {
            std::cerr << result.name << " did not produce a trace\n";
            return EXIT_FAILURE;
        }
    }
    const auto &length_only = results[0];
    const auto &leg_lqr = results[1];
    if (leg_lqr.touchdown_leg_angle_difference >=
            length_only.touchdown_leg_angle_difference ||
        leg_lqr.airborne_maximum_leg_angle_error[BC_L] >=
            length_only.airborne_maximum_leg_angle_error[BC_L] ||
        leg_lqr.airborne_maximum_leg_angle_error[BC_R] >=
            length_only.airborne_maximum_leg_angle_error[BC_R]) {
        std::cerr << "leg LQR did not improve airborne leg alignment\n";
        return EXIT_FAILURE;
    }
    if (!std::filesystem::exists(
            std::filesystem::path(argv[2]) / "platform_summary.csv")) {
        std::cerr << "platform drop benchmark did not produce a summary\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
