#include <cstdlib>
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
    if (cases.size() != 36U ||
        balance::benchmark::find_platform_drop_case(
            "platform_drop_200mm_l0p18_v0p5_length_only") == nullptr ||
        balance::benchmark::find_platform_drop_case(
            "platform_drop_400mm_l0p24_v2p0_leg_lqr") == nullptr ||
        balance::benchmark::find_platform_drop_case(
            "platform_drop_200mm_l0p18_v2p0_length_only_"
            "air_extend_l0p38") == nullptr ||
        balance::benchmark::find_platform_drop_case(
            "platform_drop_200mm_l0p18_v2p0_leg_lqr_"
            "air_extend_l0p38") == nullptr) {
        std::cerr << "platform drop case registry is incorrect\n";
        return EXIT_FAILURE;
    }

    balance::benchmark::PlatformDropBenchmark benchmark(argv[1], argv[2]);
    for (const auto &spec : cases) {
        const auto result = benchmark.run(spec);
        benchmark.write_summary(result);
        std::cout << result.name
                  << " complete=" << result.completed
                  << " recovered=" << result.recovered
                  << " diverged=" << result.diverged
                  << " issue=" << result.issue << '\n';
        if (!result.completed || !result.finite ||
            !result.balance_engaged || !result.speed_stable ||
            !result.left_platform || !result.touchdown) {
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
    if (!std::filesystem::exists(
            std::filesystem::path(argv[2]) / "platform_summary.csv")) {
        std::cerr << "platform drop benchmark did not produce a summary\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
