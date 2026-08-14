#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <iostream>

#include "drop/jump_impulse.hpp"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: mujoco_jump_impulse_test "
                     "<model.xml> <output-directory>\n";
        return EXIT_FAILURE;
    }

    const auto &cases = balance::benchmark::jump_impulse_cases();
    if (cases.size() != 6U ||
        balance::benchmark::find_jump_impulse_case(
            "jump_impulse_f140") == nullptr ||
        balance::benchmark::find_jump_impulse_case(
            "jump_impulse_f180") == nullptr ||
        balance::benchmark::find_jump_impulse_case(
            "jump_impulse_f220") == nullptr ||
        balance::benchmark::find_jump_impulse_case(
            "jump_impulse_f240_t60ms") == nullptr ||
        balance::benchmark::find_jump_impulse_case(
            "jump_impulse_f240_t90ms") == nullptr ||
        balance::benchmark::find_jump_impulse_case(
            "jump_impulse_f240_t120ms") == nullptr ||
        balance::benchmark::find_jump_impulse_case("missing") != nullptr) {
        std::cerr << "jump impulse registry is incorrect\n";
        return EXIT_FAILURE;
    }

    balance::benchmark::JumpImpulseBenchmark benchmark(argv[1], argv[2]);
    bool passed = true;
    int takeoff_count = 0;
    int false_airborne_count = 0;
    for (const auto &spec : cases) {
        const auto result = benchmark.run(spec);
        std::cout << result.name
                  << " takeoff=" << result.took_off
                  << " clearance=" << result.maximum_wheel_clearance
                  << " recovered=" << result.support_recovered
                  << " issue=" << result.issue << '\n';
        passed = passed && result.measurement_complete && result.finite &&
            result.balance_engaged && result.start_ready &&
            result.pulse_completed && !result.diverged &&
            !result.joint_saturated &&
            result.maximum_command_force >= spec.peak_force - 1.0 &&
            result.maximum_command_force <= 240.0 + 1.0e-6 &&
            result.maximum_leg_length < 0.44 &&
            std::filesystem::exists(
                std::filesystem::path(argv[2]) /
                result.name / "trace.csv");
        if (result.took_off) {
            ++takeoff_count;
            passed = passed && result.support_airborne &&
                result.both_wheels_landed && result.support_recovered &&
                result.issue == "none" &&
                std::isfinite(result.net_ground_impulse) &&
                std::isfinite(result.takeoff_com_momentum) &&
                std::abs(result.net_ground_impulse -
                    result.takeoff_com_momentum) < 0.5 &&
                result.maximum_com_rise_after_takeoff > 0.0;
        }
        if (result.false_airborne) ++false_airborne_count;
    }
    passed = passed && takeoff_count == 4 && false_airborne_count == 2;
    passed = passed && std::filesystem::exists(
        std::filesystem::path(argv[2]) / "summary.csv");
    if (!passed) {
        std::cerr << "jump impulse sweep did not produce complete traces\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
