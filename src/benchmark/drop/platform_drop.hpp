#ifndef BALANCE_BENCHMARK_PLATFORM_DROP_HPP
#define BALANCE_BENCHMARK_PLATFORM_DROP_HPP

#include <array>
#include <filesystem>
#include <limits>
#include <string>
#include <string_view>

#include "common/simulation_sample.hpp"
#include "common/csv_writer.hpp"
#include "drop_policy.hpp"
#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

namespace balance::benchmark {

struct PlatformDropSpec {
    double height{};
    double target_velocity{};
    double leg_length{0.18};
    double airborne_leg_length{};
    DropAirPolicy policy{DropAirPolicy::length_only};
    double airborne_pitch_rate{};
    bool active_landing{};
};

[[nodiscard]] std::string platform_drop_case_name(
    const PlatformDropSpec &spec);
[[nodiscard]] const std::array<PlatformDropSpec, 2> &
platform_drop_cases();
[[nodiscard]] const std::array<PlatformDropSpec, 4> &
platform_active_landing_cases();
[[nodiscard]] const PlatformDropSpec *find_platform_drop_case(
    std::string_view name) noexcept;

enum class PlatformDropPhase {
    disabled_settle,
    standing,
    accelerating,
    approaching_edge,
    airborne,
    post_touchdown,
    complete,
    failed,
};

class PlatformDropScenario {
public:
    PlatformDropScenario(const PlatformDropSpec &spec, const mjModel &model);

    void reset(sim::MujocoPlant &plant);
    void step(
        sim::MujocoPlant &plant,
        sim::SimulationRunner &runner,
        const SimulationSampler &sampler);

    [[nodiscard]] const PlatformDropSpec &spec() const noexcept {
        return spec_;
    }
    [[nodiscard]] const std::string &name() const noexcept { return name_; }
    [[nodiscard]] PlatformDropPhase phase() const noexcept { return phase_; }
    [[nodiscard]] const char *phase_name() const noexcept;
    [[nodiscard]] const char *issue() const noexcept { return issue_; }
    [[nodiscard]] bool finished() const noexcept {
        return phase_ == PlatformDropPhase::complete ||
            phase_ == PlatformDropPhase::failed;
    }
    [[nodiscard]] bool balance_engaged() const noexcept {
        return balance_engaged_;
    }
    [[nodiscard]] bool speed_stable() const noexcept { return speed_stable_; }
    [[nodiscard]] bool left_platform() const noexcept {
        return left_platform_;
    }
    [[nodiscard]] bool touchdown() const noexcept { return touchdown_; }
    [[nodiscard]] double stable_time() const noexcept { return stable_time_; }
    [[nodiscard]] double departure_time() const noexcept {
        return departure_time_;
    }
    [[nodiscard]] double touchdown_time() const noexcept {
        return touchdown_time_;
    }
    [[nodiscard]] double edge_velocity() const noexcept {
        return edge_velocity_;
    }
    [[nodiscard]] const std::array<double, BC_SIDE_NUM> &
    edge_leg_length() const noexcept {
        return edge_leg_length_;
    }
    [[nodiscard]] float held_heading() const noexcept {
        return held_heading_;
    }

private:
    [[nodiscard]] double axle_velocity(
        const SimulationSample &sample) const noexcept;
    void fail(const char *issue) noexcept;

    PlatformDropSpec spec_;
    std::string name_;
    PlatformDropPhase phase_{PlatformDropPhase::disabled_settle};
    bc_operator_command_t command_{};
    double settle_start_time_{-1.0};
    double active_start_time_{-1.0};
    double stable_hold_start_time_{-1.0};
    double stable_time_{};
    double departure_time_{};
    double touchdown_time_{};
    double edge_velocity_{};
    std::array<double, BC_SIDE_NUM> edge_leg_length_{};
    bool edge_crossed_{};
    bool balance_engaged_{};
    bool speed_stable_{};
    bool platform_contact_seen_{};
    bool left_platform_{};
    bool touchdown_{};
    bool heading_initialized_{};
    float held_heading_{};
    const char *issue_{"none"};
    int base_qpos_{};
    int base_dof_{};
    std::array<int, BC_SIDE_NUM> wheel_axis_{};
};

struct PlatformDropResult {
    PlatformDropSpec spec;
    std::string name;
    bool completed{};
    bool finite{true};
    bool balance_engaged{};
    bool speed_stable{};
    bool left_platform{};
    bool touchdown{};
    bool recovered{};
    bool diverged{};
    bool other_contact{};
    std::string issue{"none"};
    std::string first_other_contact{"none"};
    double stable_time{};
    double departure_time{};
    double touchdown_time{};
    double edge_velocity{};
    std::array<double, BC_SIDE_NUM> edge_leg_length{};
    double departure_base_z{};
    double touchdown_base_z{};
    std::array<double, BC_SIDE_NUM> touchdown_leg_length{};
    std::array<double, BC_SIDE_NUM> touchdown_leg_world_angle{};
    std::array<double, BC_SIDE_NUM> touchdown_leg_world_angle_rate{};
    std::array<double, BC_SIDE_NUM> airborne_maximum_leg_angle_error{};
    double touchdown_leg_angle_difference{};
    double touchdown_pitch{};
    double touchdown_pitch_rate{};
    double touchdown_roll{};
    double maximum_pitch{};
    double maximum_roll{};
    double airborne_maximum_pitch{};
    double airborne_maximum_joint_torque_request{};
    double airborne_maximum_joint_torque{};
    double airborne_joint_saturation_ratio{};
    double touchdown_vertical_velocity{};
    double post_touchdown_maximum_support_force{};
    double post_touchdown_minimum_base_z{
        std::numeric_limits<double>::infinity()};
    double post_touchdown_maximum_pitch{};
    double post_touchdown_maximum_roll{};
    std::array<double, BC_SIDE_NUM> maximum_leg_compression{};
    std::array<double, BC_SIDE_NUM> maximum_ground_normal_force{};
    std::array<double, BC_SIDE_NUM> maximum_requested_axial_force{};
    std::array<double, BC_SIDE_NUM> maximum_applied_axial_force{};
    double force_rate_limited_ratio{};
    double post_touchdown_joint_saturation_ratio{};
    bool landing_recovery_started{};
    double landing_recovery_seconds{
        std::numeric_limits<double>::quiet_NaN()};
    double shadow_airborne_delay{
        std::numeric_limits<double>::quiet_NaN()};
    double shadow_landing_delay{
        std::numeric_limits<double>::quiet_NaN()};
    double shadow_recover_delay{
        std::numeric_limits<double>::quiet_NaN()};
    double shadow_ground_delay{
        std::numeric_limits<double>::quiet_NaN()};
    bool rebound{};
    double landing_stable_time{
        std::numeric_limits<double>::quiet_NaN()};
    std::array<double, BC_SIDE_NUM> wheel_touchdown_time{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
    double wheel_touchdown_time_difference{
        std::numeric_limits<double>::quiet_NaN()};
    double airborne_maximum_velocity_error{};
    double touchdown_velocity_error{
        std::numeric_limits<double>::quiet_NaN()};
    double kf_recovery_time{
        std::numeric_limits<double>::quiet_NaN()};
    std::array<double, BC_SIDE_NUM> support_air_time{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
    std::array<double, BC_SIDE_NUM> support_ground_time{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
};

class PlatformDropBenchmark {
public:
    PlatformDropBenchmark(
        const std::filesystem::path &model_path,
        const std::filesystem::path &output_directory);

    [[nodiscard]] PlatformDropResult run(const PlatformDropSpec &spec);
    void write_summary(const PlatformDropResult &result);

private:
    void write_trace(
        CsvWriter &trace,
        const PlatformDropScenario &scenario,
        const SimulationSample &sample);

    std::filesystem::path output_directory_;
    sim::MujocoPlant plant_;
    sim::MujocoAdapter adapter_;
    SimulationSampler sampler_;
    CsvWriter summary_;
    double leg_angle_trim_{};
};

} // namespace balance::benchmark

#endif
