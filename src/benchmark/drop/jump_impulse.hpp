#ifndef BALANCE_BENCHMARK_JUMP_IMPULSE_HPP
#define BALANCE_BENCHMARK_JUMP_IMPULSE_HPP

#include <array>
#include <filesystem>
#include <limits>
#include <string>
#include <string_view>

#include "common/csv_writer.hpp"
#include "common/simulation_sample.hpp"
#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

namespace balance::benchmark {

struct JumpImpulseSpec {
    double peak_force{};
    double leg_length{0.18};
    double hold_seconds{0.060};
};

[[nodiscard]] std::string jump_impulse_case_name(
    const JumpImpulseSpec &spec);
[[nodiscard]] const std::array<JumpImpulseSpec, 6> &jump_impulse_cases();
[[nodiscard]] const JumpImpulseSpec *find_jump_impulse_case(
    std::string_view name) noexcept;

enum class JumpImpulsePhase {
    disabled_settle,
    standing,
    ramp_up,
    hold,
    release,
    wait_airborne,
    airborne,
    post_landing,
    complete,
    failed,
};

struct JumpImpulseResult {
    JumpImpulseSpec spec;
    std::string name;
    std::string issue{"none"};
    bool measurement_complete{};
    bool finite{true};
    bool balance_engaged{};
    bool start_ready{};
    bool pulse_completed{};
    bool took_off{};
    bool support_airborne{};
    bool both_wheels_landed{};
    bool support_recovered{};
    bool false_airborne{};
    bool non_wheel_contact{};
    bool joint_saturated{};
    bool diverged{};
    double takeoff_time{std::numeric_limits<double>::quiet_NaN()};
    double takeoff_vertical_velocity{
        std::numeric_limits<double>::quiet_NaN()};
    double takeoff_com_vertical_velocity{
        std::numeric_limits<double>::quiet_NaN()};
    double support_airborne_delay{
        std::numeric_limits<double>::quiet_NaN()};
    double flight_time{std::numeric_limits<double>::quiet_NaN()};
    double departure_delta{std::numeric_limits<double>::quiet_NaN()};
    double landing_delta{std::numeric_limits<double>::quiet_NaN()};
    double maximum_wheel_clearance{};
    double maximum_com_rise{};
    double maximum_com_rise_after_takeoff{};
    double maximum_pitch{};
    double maximum_roll{};
    double maximum_leg_length{};
    double maximum_leg_speed{};
    double maximum_joint_torque_request{};
    double maximum_command_force{};
    double maximum_estimated_axial_force{};
    double ground_normal_impulse{};
    double net_ground_impulse{std::numeric_limits<double>::quiet_NaN()};
    double takeoff_com_momentum{
        std::numeric_limits<double>::quiet_NaN()};
    double applied_hold_seconds{};
    std::string release_reason{"none"};
};

class JumpImpulseScenario {
public:
    JumpImpulseScenario(const JumpImpulseSpec &spec, const mjModel &model);

    void reset();
    void step(
        sim::MujocoPlant &plant,
        sim::SimulationRunner &runner,
        const SimulationSampler &sampler);

    [[nodiscard]] const JumpImpulseSpec &spec() const noexcept {
        return spec_;
    }
    [[nodiscard]] const std::string &name() const noexcept { return name_; }
    [[nodiscard]] JumpImpulsePhase phase() const noexcept { return phase_; }
    [[nodiscard]] const char *phase_name() const noexcept;
    [[nodiscard]] const char *issue() const noexcept { return issue_; }
    [[nodiscard]] bool finished() const noexcept {
        return phase_ == JumpImpulsePhase::complete ||
            phase_ == JumpImpulsePhase::failed;
    }
    [[nodiscard]] bool balance_engaged() const noexcept {
        return balance_engaged_;
    }
    [[nodiscard]] bool start_ready() const noexcept { return start_ready_; }
    [[nodiscard]] bool pulse_completed() const noexcept {
        return pulse_completed_;
    }
    [[nodiscard]] bool took_off() const noexcept { return took_off_; }
    [[nodiscard]] bool support_airborne() const noexcept {
        return support_airborne_;
    }
    [[nodiscard]] bool both_wheels_landed() const noexcept;
    [[nodiscard]] bool support_recovered() const noexcept {
        return support_recovered_;
    }
    [[nodiscard]] bool false_airborne() const noexcept {
        return false_airborne_;
    }
    [[nodiscard]] double commanded_force(int side) const noexcept {
        return commanded_force_[side];
    }
    [[nodiscard]] double takeoff_time() const noexcept {
        return takeoff_time_;
    }
    [[nodiscard]] double takeoff_vertical_velocity() const noexcept {
        return takeoff_vertical_velocity_;
    }
    [[nodiscard]] double takeoff_com_vertical_velocity() const noexcept {
        return takeoff_com_vertical_velocity_;
    }
    [[nodiscard]] double support_airborne_time() const noexcept {
        return support_airborne_time_;
    }
    [[nodiscard]] double first_landing_time() const noexcept;
    [[nodiscard]] const std::array<double, BC_SIDE_NUM> &departure_time()
        const noexcept { return departure_time_; }
    [[nodiscard]] const std::array<double, BC_SIDE_NUM> &landing_time()
        const noexcept { return landing_time_; }
    [[nodiscard]] double axle_height(const mjData &data) const;
    [[nodiscard]] double axle_vertical_velocity(const mjData &data) const;
    [[nodiscard]] double initial_axle_height() const noexcept {
        return initial_axle_height_;
    }
    [[nodiscard]] double com_height(const mjData &data) const noexcept;
    [[nodiscard]] double com_vertical_velocity(
        const mjData &data) const noexcept;
    [[nodiscard]] double initial_com_height() const noexcept {
        return initial_com_height_;
    }
    [[nodiscard]] double thrust_start_time() const noexcept {
        return thrust_start_;
    }
    [[nodiscard]] double takeoff_com_height() const noexcept {
        return takeoff_com_height_;
    }
    [[nodiscard]] double applied_hold_seconds() const noexcept {
        return applied_hold_seconds_;
    }
    [[nodiscard]] const char *release_reason() const noexcept {
        return release_reason_;
    }

private:
    [[nodiscard]] std::array<bool, BC_SIDE_NUM> wheel_contacts(
        const mjData &data) const;
    void update_truth(mjData &data, double time);
    void start_release(double time, const char *reason) noexcept;
    void apply_force_control(sim::SimulationRunner &runner);
    void finish_with_issue(const char *issue) noexcept;

    JumpImpulseSpec spec_;
    std::string name_;
    JumpImpulsePhase phase_{JumpImpulsePhase::disabled_settle};
    bc_operator_command_t command_{};
    const mjModel *model_{};
    int ground_{};
    int base_body_{};
    std::array<int, BC_SIDE_NUM> wheel_{};
    std::array<int, BC_SIDE_NUM> wheel_axis_{};
    std::array<bool, BC_SIDE_NUM> previous_contact_{{true, true}};
    std::array<double, BC_SIDE_NUM> departure_time_{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
    std::array<double, BC_SIDE_NUM> landing_time_{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
    std::array<double, BC_SIDE_NUM> commanded_force_{};
    double reset_time_{};
    double ready_hold_start_{-1.0};
    double phase_start_{};
    double thrust_start_{};
    double all_air_candidate_start_{-1.0};
    double recovery_hold_start_{-1.0};
    double applied_hold_seconds_{};
    double initial_axle_height_{};
    double initial_com_height_{};
    double takeoff_time_{std::numeric_limits<double>::quiet_NaN()};
    double takeoff_vertical_velocity_{
        std::numeric_limits<double>::quiet_NaN()};
    double takeoff_com_vertical_velocity_{
        std::numeric_limits<double>::quiet_NaN()};
    double takeoff_candidate_com_vertical_velocity_{
        std::numeric_limits<double>::quiet_NaN()};
    double takeoff_com_height_{std::numeric_limits<double>::quiet_NaN()};
    double support_airborne_time_{
        std::numeric_limits<double>::quiet_NaN()};
    bool balance_engaged_{};
    bool start_ready_{};
    bool pulse_completed_{};
    bool took_off_{};
    bool support_airborne_{};
    bool support_recovered_{};
    bool false_airborne_{};
    const char *release_reason_{"none"};
    const char *issue_{"none"};
};

class JumpImpulseBenchmark {
public:
    JumpImpulseBenchmark(
        const std::filesystem::path &model_path,
        const std::filesystem::path &output_directory);

    [[nodiscard]] JumpImpulseResult run(const JumpImpulseSpec &spec);

private:
    void write_summary(const JumpImpulseResult &result);
    void write_trace(
        CsvWriter &trace,
        const JumpImpulseScenario &scenario,
        const SimulationSample &sample) const;

    std::filesystem::path output_directory_;
    sim::MujocoPlant plant_;
    sim::MujocoAdapter adapter_;
    SimulationSampler sampler_;
    CsvWriter summary_;
};

} // namespace balance::benchmark

#endif
