#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "drop/ramp_jump.hpp"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: mujoco_ramp_jump_test "
                     "<model.xml> <output-directory>\n";
        return EXIT_FAILURE;
    }

    const auto &cases = balance::benchmark::ramp_jump_cases();
    if (cases.size() != 5U ||
        balance::benchmark::find_ramp_jump_case("ramp_jump_v2") == nullptr ||
        balance::benchmark::find_ramp_jump_case("ramp_jump_v2p5") == nullptr ||
        balance::benchmark::find_ramp_jump_case("ramp_jump_v3") == nullptr ||
        balance::benchmark::find_ramp_jump_case("missing") != nullptr) {
        std::cerr << "ramp jump registry is incorrect\n";
        return EXIT_FAILURE;
    }

    balance::benchmark::RampJumpBenchmark benchmark(argv[1], argv[2]);
    bool passed = true;
    for (const auto &spec : cases) {
        const auto result = benchmark.run(spec);
        std::cout << result.name
                  << " entry_v=" << result.ramp_entry_velocity
                  << " takeoff_vx=" << result.takeoff_velocity_x
                  << " landing=" << result.first_landing_distance
                  << " complete=" << result.completed
                  << " issue=" << result.issue << '\n';
        passed = passed && result.finite && result.balance_engaged &&
            result.start_ready && result.entry_speed_stable &&
            result.took_off && result.both_wheels_landed &&
            result.completed && result.support_recovered &&
            !result.diverged &&
            std::filesystem::exists(
                std::filesystem::path(argv[2]) /
                result.name / "trace.csv");
    }
    passed = passed && std::filesystem::exists(
        std::filesystem::path(argv[2]) / "summary.csv");
    if (!passed) {
        std::cerr << "ramp jump sweep did not produce complete measurements\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
