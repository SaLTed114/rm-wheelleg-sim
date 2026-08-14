#ifndef BALANCE_BENCHMARK_STEP_DOCK_HPP
#define BALANCE_BENCHMARK_STEP_DOCK_HPP

#include <array>
#include <cmath>
#include <deque>
#include <filesystem>
#include <limits>
#include <string>
#include <string_view>
#include <utility>

#include "common/csv_writer.hpp"
#include "common/simulation_sample.hpp"
#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

namespace balance::benchmark {

struct StepDockSpec {
    double target_velocity{2.0};
    double leg_length{0.38};
    double platform_height{0.20};
    double cut_delay_seconds{};
    bool production_task{};
    bool transfer_preview{};
    double forward_acceleration_rate{3.0};
    double initial_heading_radians{};
    double approach_heading_radians{};
    double platform_gap_at_acceleration{
        std::numeric_limits<double>::quiet_NaN()};
    double target_collision_travel{
        std::numeric_limits<double>::quiet_NaN()};
    bool require_speed_stable{true};
};

[[nodiscard]] std::string step_dock_case_name(const StepDockSpec &spec);
[[nodiscard]] const std::array<StepDockSpec, 2> &step_dock_cases();
[[nodiscard]] const StepDockSpec &step_dock_transfer_preview_case();
[[nodiscard]] const StepDockSpec *find_step_dock_case(
    std::string_view name) noexcept;

enum class StepDockPhase {
    disabled_settle,
    standing,
    accelerating,
    approach,
    impact_delay,
    passive,
    transfer,
    transfer_hold,
    complete,
    failed,
};

struct StepDockContact {
    bool leg_face{};
    bool wheel_face{};
    bool other_leg_face{};
    bool base_face{};
    bool base_top{};
    std::array<bool, BC_SIDE_NUM> side_face{};
    std::array<bool, BC_SIDE_NUM> side_wheel_top{};
    double total_normal_force{};
    double strongest_normal_force{};
    double strongest_x{std::numeric_limits<double>::quiet_NaN()};
    double strongest_z{std::numeric_limits<double>::quiet_NaN()};
    std::string strongest_pair{"none"};
};

struct StepDockResult {
    StepDockSpec spec;
    std::string name;
    std::string issue{"none"};
    std::string first_contact_pair{"none"};
    bool measurement_complete{};
    bool finite{true};
    bool balance_engaged{};
    bool start_ready{};
    bool speed_stable{};
    bool contact_detected{};
    bool body_contact_before_trigger{};
    bool control_cut{};
    bool retained_on_platform{};
    bool passively_supported{};
    double collision_time{std::numeric_limits<double>::quiet_NaN()};
    double collision_velocity{std::numeric_limits<double>::quiet_NaN()};
    double collision_clearance{std::numeric_limits<double>::quiet_NaN()};
    double collision_world_heading{
        std::numeric_limits<double>::quiet_NaN()};
    double acceleration_start_x{std::numeric_limits<double>::quiet_NaN()};
    double collision_travel{std::numeric_limits<double>::quiet_NaN()};
    double control_cut_delay{std::numeric_limits<double>::quiet_NaN()};
    double minimum_approach_clearance{
        std::numeric_limits<double>::infinity()};
    double trigger_contact_force{};
    double strongest_contact_force{};
    double platform_normal_impulse{};
    double maximum_post_cut_actuation{};
    std::array<double, BC_SIDE_NUM> approach_horizontal_force{};
    std::array<double, BC_SIDE_NUM> approach_horizontal_range{};
    double delay_peak_horizontal_residual{};
    double horizontal_force_detection_latency{
        std::numeric_limits<double>::quiet_NaN()};
    double maximum_delay_pitch{};
    double maximum_delay_pitch_rate{};
    bool base_contact_before_cut{};
    double cut_base_velocity{std::numeric_limits<double>::quiet_NaN()};
    double cut_pitch{std::numeric_limits<double>::quiet_NaN()};
    std::array<double, BC_SIDE_NUM> cut_leg_length{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
    double final_base_x{};
    double final_base_z{};
    double final_pitch{};
    double final_roll{};
    std::array<double, BC_SIDE_NUM> final_leg_length{};
    std::array<double, BC_SIDE_NUM> final_leg_angle{};
    std::array<double, BC_SIDE_NUM> final_wheel_x{};
    std::array<double, BC_SIDE_NUM> final_wheel_z{};
    double final_base_top_contact_ratio{};
    double final_wheel_top_contact_ratio{};
    double final_maximum_forward_speed{};
    double final_maximum_vertical_speed{};
    double final_maximum_pitch_rate{};
    double final_maximum_roll_rate{};
};

class StepDockScenario {
public:
    StepDockScenario(const StepDockSpec &spec, const mjModel &model);

    void reset(sim::MujocoPlant &plant);
    void step(
        sim::MujocoPlant &plant,
        sim::SimulationRunner &runner,
        const SimulationSampler &sampler);

    [[nodiscard]] const StepDockSpec &spec() const noexcept { return spec_; }
    [[nodiscard]] const std::string &name() const noexcept { return name_; }
    [[nodiscard]] StepDockPhase phase() const noexcept { return phase_; }
    [[nodiscard]] const char *phase_name() const noexcept;
    [[nodiscard]] const char *issue() const noexcept { return issue_; }
    [[nodiscard]] bool finished() const noexcept {
        return observation_complete_ || phase_ == StepDockPhase::complete ||
            phase_ == StepDockPhase::failed;
    }
    [[nodiscard]] bool observation_complete() const noexcept {
        return observation_complete_;
    }
    [[nodiscard]] bool balance_engaged() const noexcept {
        return balance_engaged_;
    }
    [[nodiscard]] bool start_ready() const noexcept { return start_ready_; }
    [[nodiscard]] bool speed_stable() const noexcept { return speed_stable_; }
    [[nodiscard]] bool contact_detected() const noexcept {
        return contact_detected_;
    }
    [[nodiscard]] bool body_contact_before_trigger() const noexcept {
        return body_contact_before_trigger_;
    }
    [[nodiscard]] double collision_time() const noexcept {
        return collision_time_;
    }
    [[nodiscard]] double collision_velocity() const noexcept {
        return collision_velocity_;
    }
    [[nodiscard]] double collision_clearance() const noexcept {
        return collision_clearance_;
    }
    [[nodiscard]] double collision_world_heading() const noexcept {
        return collision_world_heading_;
    }
    [[nodiscard]] double acceleration_start_x() const noexcept {
        return acceleration_start_x_;
    }
    [[nodiscard]] double collision_travel() const noexcept {
        return collision_x_ - acceleration_start_x_;
    }
    [[nodiscard]] const std::string &first_contact_pair() const noexcept {
        return first_contact_pair_;
    }
    [[nodiscard]] double trigger_contact_force() const noexcept {
        return trigger_contact_force_;
    }
    [[nodiscard]] double control_cut_time() const noexcept {
        return control_cut_time_;
    }
    [[nodiscard]] bool control_cut() const noexcept {
        return std::isfinite(control_cut_time_);
    }
    [[nodiscard]] double collision_elapsed(double time) const noexcept;
    [[nodiscard]] double passive_elapsed(double time) const noexcept;
    [[nodiscard]] const std::array<double, BC_SIDE_NUM> &
    transfer_length_reference() const noexcept {
        return transfer_length_reference_;
    }
    [[nodiscard]] const std::array<double, BC_SIDE_NUM> &
    transfer_angle_reference() const noexcept {
        return transfer_angle_reference_;
    }
    [[nodiscard]] double commanded_velocity() const noexcept {
        return command_.forward_velocity;
    }
    [[nodiscard]] double base_clearance(const mjData &data) const noexcept;
    [[nodiscard]] double base_world_heading(
        const mjData &data) const noexcept;
    [[nodiscard]] double wheel_axis_x(const mjData &data, int side) const;
    [[nodiscard]] double wheel_axis_z(const mjData &data, int side) const;
    [[nodiscard]] StepDockContact observe_contacts(
        const mjData &data) const;
    [[nodiscard]] const sim::StepDockLayout &layout() const noexcept {
        return layout_;
    }

private:
    [[nodiscard]] int leg_side_for_body(int body) const noexcept;
    [[nodiscard]] std::string contact_pair(const mjContact &contact) const;
    void update_speed_window(double time, double velocity);
    void record_collision(
        double time, double base_x, double velocity, double clearance,
        double world_heading,
        const StepDockContact &contact, bool body_first) noexcept;
    void enter_passive(
        double time,
        const bc_controller_snapshot_t &snapshot) noexcept;
    void step_passive(
        sim::SimulationRunner &runner,
        const bc_gimbal_feedback_t &gimbal);
    void step_transfer_preview(
        sim::SimulationRunner &runner,
        const bc_gimbal_feedback_t &gimbal,
        double time);
    void step_transfer_hold(
        sim::SimulationRunner &runner,
        const bc_gimbal_feedback_t &gimbal);
    void step_transfer_control(
        sim::SimulationRunner &runner,
        const bc_gimbal_feedback_t &gimbal);
    void fail(const char *issue) noexcept;

    StepDockSpec spec_;
    std::string name_;
    const mjModel *model_{};
    sim::StepDockLayout layout_{};
    StepDockPhase phase_{StepDockPhase::disabled_settle};
    bc_operator_command_t command_{};
    int platform_{};
    int base_body_{};
    int base_geom_{};
    std::array<int, BC_SIDE_NUM> leg_root_front_{};
    std::array<int, BC_SIDE_NUM> leg_root_rear_{};
    std::array<int, BC_SIDE_NUM> wheel_collision_{};
    std::array<int, BC_SIDE_NUM> wheel_axis_{};
    double wheel_radius_{};
    std::array<double, 3> base_bounds_center_{};
    std::array<double, 3> base_bounds_half_{};
    std::deque<std::pair<double, double>> speed_window_;
    double reset_time_{};
    double ready_hold_start_{-1.0};
    double passive_start_{std::numeric_limits<double>::quiet_NaN()};
    double control_cut_time_{std::numeric_limits<double>::quiet_NaN()};
    double collision_time_{std::numeric_limits<double>::quiet_NaN()};
    double collision_velocity_{std::numeric_limits<double>::quiet_NaN()};
    double collision_clearance_{std::numeric_limits<double>::quiet_NaN()};
    double collision_world_heading_{
        std::numeric_limits<double>::quiet_NaN()};
    double acceleration_start_x_{
        std::numeric_limits<double>::quiet_NaN()};
    double collision_x_{std::numeric_limits<double>::quiet_NaN()};
    double trigger_contact_force_{};
    std::array<double, BC_SIDE_NUM> transfer_start_length_{};
    std::array<double, BC_SIDE_NUM> transfer_start_angle_{};
    std::array<double, BC_SIDE_NUM> transfer_length_reference_{};
    std::array<double, BC_SIDE_NUM> transfer_angle_reference_{};
    std::string first_contact_pair_{"none"};
    bool balance_engaged_{};
    bool start_ready_{};
    bool speed_stable_{};
    bool contact_detected_{};
    bool body_contact_before_trigger_{};
    bool observation_complete_{};
    const char *issue_{"none"};
};

class StepDockBenchmark {
public:
    StepDockBenchmark(
        const std::filesystem::path &model_path,
        const std::filesystem::path &output_directory);

    [[nodiscard]] StepDockResult run(const StepDockSpec &spec);

private:
    void write_summary(const StepDockResult &result);
    void write_trace(
        CsvWriter &trace,
        const StepDockScenario &scenario,
        const SimulationSample &sample,
        const StepDockContact &contact,
        const bc_sensor_feedback_t &feedback) const;

    std::filesystem::path output_directory_;
    sim::MujocoPlant plant_;
    sim::MujocoAdapter adapter_;
    SimulationSampler sampler_;
    CsvWriter summary_;
};

} // namespace balance::benchmark

#endif
