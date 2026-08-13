#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "drop/ramp_course.hpp"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: mujoco_ramp_course_test "
                     "<model.xml> <output-directory>\n";
        return EXIT_FAILURE;
    }

    const auto &cases = balance::benchmark::ramp_course_cases();
    if (cases.size() != 4U ||
        balance::benchmark::find_ramp_course_case(
            "ramp_course_ascent_stop_l0p18") == nullptr ||
        balance::benchmark::find_ramp_course_case(
            "ramp_course_ascent_stop_l0p24") == nullptr ||
        balance::benchmark::find_ramp_course_case(
            "ramp_course_descent_l0p24") == nullptr ||
        balance::benchmark::find_ramp_course_case(
            "ramp_course_traverse_l0p24") == nullptr ||
        balance::benchmark::find_ramp_course_case(
            "ramp_course_ascent_stop_l0p22") != nullptr ||
        balance::benchmark::find_ramp_course_case(
            "ramp_course_traverse_l0p24_beveled") != nullptr ||
        balance::benchmark::find_ramp_course_case("missing") != nullptr) {
        std::cerr << "ramp course registry is incorrect\n";
        return EXIT_FAILURE;
    }

    balance::benchmark::RampCourseBenchmark benchmark(argv[1], argv[2]);
    bool passed = true;
    for (const auto &spec : cases) {
        const auto result = benchmark.run(spec);
        std::cout << result.name
                  << " complete=" << result.completed
                  << " finite=" << result.finite
                  << " ready=" << result.start_ready
                  << " collision=" << result.non_wheel_collision
                  << " clearance=" << result.minimum_clearance
                  << " issue=" << result.issue << '\n';
        passed = passed && result.finite && result.balance_engaged &&
            result.start_ready &&
            result.minimum_clearance_geom != "none" &&
            std::filesystem::exists(
                std::filesystem::path(argv[2]) / result.name / "trace.csv");
    }
    passed = passed && std::filesystem::exists(
        std::filesystem::path(argv[2]) / "summary.csv");
    if (!passed) {
        std::cerr << "ramp course matrix did not produce complete diagnostics\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
