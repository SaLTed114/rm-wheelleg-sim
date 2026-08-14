#ifndef BALANCE_SIM_SIMULATION_APP_HPP
#define BALANCE_SIM_SIMULATION_APP_HPP

#include <filesystem>
#include <optional>

namespace balance::benchmark {
struct DropCaseSpec;
struct PlatformDropSpec;
struct PerformanceCaseSpec;
struct RampCourseSpec;
struct RampJumpSpec;
}

namespace balance::sim {

struct SimulationAppOptions {
    std::filesystem::path model_path;
    std::optional<std::filesystem::path> trace_path;
    const benchmark::DropCaseSpec *drop_case{};
    const benchmark::PlatformDropSpec *platform_drop_case{};
    const benchmark::PerformanceCaseSpec *performance_case{};
    const benchmark::RampCourseSpec *ramp_course_case{};
    const benchmark::RampJumpSpec *ramp_jump_case{};
    std::optional<double> drop_wheel_clearance;
    std::optional<float> leg_length;
    std::optional<float> yaw_acceleration_feedforward_scale;
    bool keyboard_drive{};
};

void run_simulation_app(const SimulationAppOptions &options);

} // namespace balance::sim

#endif
