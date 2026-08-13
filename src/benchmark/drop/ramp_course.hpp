#ifndef BALANCE_BENCHMARK_RAMP_COURSE_HPP
#define BALANCE_BENCHMARK_RAMP_COURSE_HPP

#include <array>
#include <filesystem>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "common/csv_writer.hpp"
#include "common/simulation_sample.hpp"
#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

namespace balance::benchmark {

enum class RampCourseMode {
    ascent_stop,
    descent,
    traverse,
};

struct RampCourseSpec {
    RampCourseMode mode{};
    double leg_length{};
    double target_velocity{2.0};
    bool beveled_transition{};
};

[[nodiscard]] std::string ramp_course_case_name(const RampCourseSpec &spec);
[[nodiscard]] const std::array<RampCourseSpec, 4> &ramp_course_cases();
[[nodiscard]] const RampCourseSpec *find_ramp_course_case(
    std::string_view name) noexcept;

enum class RampCoursePhase {
    disabled_settle,
    standing,
    approach,
    ascent,
    platform,
    descent,
    exit,
    stopping,
    complete,
    failed,
};

struct RampCourseResult {
    RampCourseSpec spec;
    std::string name;
    bool completed{};
    bool finite{true};
    bool balance_engaged{};
    bool start_ready{};
    bool non_wheel_collision{};
    bool wheel_velocity_lost{};
    bool wheel_velocity_recovered{};
    bool forward_hold_recovered{};
    bool diverged{};
    std::string issue{"none"};
    std::string minimum_clearance_geom{"none"};
    std::string first_collision_geom{"none"};
    std::string first_collision_phase{"none"};
    double minimum_clearance{std::numeric_limits<double>::infinity()};
    double minimum_clearance_x{};
    double collision_x{std::numeric_limits<double>::quiet_NaN()};
    double collision_pitch{std::numeric_limits<double>::quiet_NaN()};
    double maximum_pitch{};
    double maximum_roll{};
    double maximum_yaw_error{};
    double ramp_wheel_contact_time_difference{
        std::numeric_limits<double>::quiet_NaN()};
    int airborne_event_count{};
    double maximum_airborne_duration{};
    double first_airborne_time{std::numeric_limits<double>::quiet_NaN()};
    double first_landing_time{std::numeric_limits<double>::quiet_NaN()};
    double first_landing_wheel_time_difference{
        std::numeric_limits<double>::quiet_NaN()};
    double wheel_velocity_recovery_seconds{
        std::numeric_limits<double>::quiet_NaN()};
    double final_truth_velocity{};
    double final_estimated_velocity{};
    double final_position_error{};
    double maximum_wheel_torque_request{};
    double maximum_joint_torque_request{};
    bool wheel_saturated{};
    bool joint_saturated{};
};

class RampCourseScenario {
public:
    RampCourseScenario(const RampCourseSpec &spec, const mjModel &model);

    void reset(sim::MujocoPlant &plant);
    void step(
        sim::MujocoPlant &plant,
        sim::SimulationRunner &runner,
        const SimulationSampler &sampler);

    [[nodiscard]] const RampCourseSpec &spec() const noexcept { return spec_; }
    [[nodiscard]] const std::string &name() const noexcept { return name_; }
    [[nodiscard]] RampCoursePhase phase() const noexcept { return phase_; }
    [[nodiscard]] const char *phase_name() const noexcept;
    [[nodiscard]] const char *issue() const noexcept { return issue_; }
    [[nodiscard]] bool finished() const noexcept {
        return phase_ == RampCoursePhase::complete ||
            phase_ == RampCoursePhase::failed;
    }
    [[nodiscard]] bool balance_engaged() const noexcept {
        return balance_engaged_;
    }
    [[nodiscard]] bool start_ready() const noexcept { return start_ready_; }
    [[nodiscard]] float held_heading() const noexcept { return 0.0F; }
    [[nodiscard]] float commanded_velocity() const noexcept {
        return command_.forward_velocity;
    }

private:
    void fail(const char *issue) noexcept;
    void set_initial_axle_x(sim::MujocoPlant &plant, double target_x) const;

    RampCourseSpec spec_;
    std::string name_;
    sim::RampCourseLayout layout_{};
    RampCoursePhase phase_{RampCoursePhase::disabled_settle};
    bc_operator_command_t command_{};
    double reset_time_{};
    double active_time_{-1.0};
    double ready_hold_start_{-1.0};
    double stop_hold_start_{-1.0};
    double progress_reference_time_{-1.0};
    double progress_reference_x_{};
    double reverse_start_time_{-1.0};
    double stopping_start_x_{};
    double stopping_start_time_{-1.0};
    bool balance_engaged_{};
    bool start_ready_{};
    const char *issue_{"none"};
    int base_qpos_{};
    std::array<int, BC_SIDE_NUM> wheel_axis_{};
};

class RampCourseBenchmark {
public:
    RampCourseBenchmark(
        const std::filesystem::path &model_path,
        const std::filesystem::path &output_directory);

    [[nodiscard]] RampCourseResult run(const RampCourseSpec &spec);
    void write_summary(const RampCourseResult &result);

private:
    struct TerrainObservation {
        bool wheel[BC_SIDE_NUM]{};
        bool non_wheel_collision{};
        std::string collision_geom{"none"};
        double minimum_clearance{std::numeric_limits<double>::infinity()};
        std::string minimum_clearance_geom{"none"};
    };

    [[nodiscard]] TerrainObservation observe_terrain();
    void write_trace(
        CsvWriter &trace,
        const RampCourseScenario &scenario,
        const SimulationSample &sample,
        const TerrainObservation &terrain,
        double yaw_error,
        double truth_velocity) const;

    std::filesystem::path output_directory_;
    sim::MujocoPlant plant_;
    sim::MujocoAdapter adapter_;
    SimulationSampler sampler_;
    CsvWriter summary_;
    std::vector<int> terrain_;
    std::array<int, BC_SIDE_NUM> wheel_{};
    std::vector<int> non_wheel_robot_geoms_;
};

} // namespace balance::benchmark

#endif
