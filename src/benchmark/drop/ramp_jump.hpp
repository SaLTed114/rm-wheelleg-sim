#ifndef BALANCE_BENCHMARK_RAMP_JUMP_HPP
#define BALANCE_BENCHMARK_RAMP_JUMP_HPP

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

struct RampJumpSpec {
    double target_velocity{};
    double leg_length{0.24};
};

[[nodiscard]] std::string ramp_jump_case_name(const RampJumpSpec &spec);
[[nodiscard]] const std::array<RampJumpSpec, 5> &ramp_jump_cases();
[[nodiscard]] const RampJumpSpec *find_ramp_jump_case(
    std::string_view name) noexcept;

enum class RampJumpPhase {
    disabled_settle,
    standing,
    accelerating,
    approach,
    ascent,
    airborne,
    post_landing,
    complete,
    failed,
};

struct RampJumpResult {
    RampJumpSpec spec;
    std::string name;
    bool completed{};
    bool finite{true};
    bool balance_engaged{};
    bool start_ready{};
    bool entry_speed_stable{};
    bool took_off{};
    bool both_wheels_landed{};
    bool support_recovered{};
    bool premature_airborne{};
    bool diverged{};
    bool non_wheel_contact{};
    bool wheel_saturated{};
    bool joint_saturated{};
    std::string issue{"none"};
    std::string first_non_wheel_contact{"none"};
    double ramp_entry_velocity{std::numeric_limits<double>::quiet_NaN()};
    double takeoff_time{std::numeric_limits<double>::quiet_NaN()};
    double takeoff_x{std::numeric_limits<double>::quiet_NaN()};
    double takeoff_velocity_x{std::numeric_limits<double>::quiet_NaN()};
    double takeoff_velocity_z{std::numeric_limits<double>::quiet_NaN()};
    double flight_time{std::numeric_limits<double>::quiet_NaN()};
    std::array<double, BC_SIDE_NUM> wheel_departure_time{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
    std::array<double, BC_SIDE_NUM> wheel_landing_time{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
    std::array<double, BC_SIDE_NUM> wheel_landing_distance{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
    double first_landing_distance{std::numeric_limits<double>::quiet_NaN()};
    double ballistic_distance{std::numeric_limits<double>::quiet_NaN()};
    double maximum_pitch{};
    double maximum_roll{};
    double maximum_yaw_error{};
    double maximum_wheel_torque_request{};
    double maximum_joint_torque_request{};
};

class RampJumpScenario {
public:
    RampJumpScenario(const RampJumpSpec &spec, const mjModel &model);

    void reset(sim::MujocoPlant &plant);
    void step(
        sim::MujocoPlant &plant,
        sim::SimulationRunner &runner,
        const SimulationSampler &sampler);

    [[nodiscard]] const RampJumpSpec &spec() const noexcept { return spec_; }
    [[nodiscard]] const std::string &name() const noexcept { return name_; }
    [[nodiscard]] RampJumpPhase phase() const noexcept { return phase_; }
    [[nodiscard]] const char *phase_name() const noexcept;
    [[nodiscard]] const char *issue() const noexcept { return issue_; }
    [[nodiscard]] bool finished() const noexcept {
        return phase_ == RampJumpPhase::complete ||
            phase_ == RampJumpPhase::failed;
    }
    [[nodiscard]] bool balance_engaged() const noexcept {
        return balance_engaged_;
    }
    [[nodiscard]] bool start_ready() const noexcept { return start_ready_; }
    [[nodiscard]] bool entry_speed_stable() const noexcept {
        return entry_speed_stable_;
    }
    [[nodiscard]] float held_heading() const noexcept { return 0.0F; }
    [[nodiscard]] float commanded_velocity() const noexcept {
        return command_.forward_velocity;
    }
    [[nodiscard]] const sim::RampJumpLayout &layout() const noexcept {
        return layout_;
    }
    [[nodiscard]] double ramp_entry_velocity() const noexcept {
        return ramp_entry_velocity_;
    }
    [[nodiscard]] double takeoff_time() const noexcept { return takeoff_time_; }
    [[nodiscard]] double takeoff_x() const noexcept { return takeoff_x_; }
    [[nodiscard]] double takeoff_velocity_x() const noexcept {
        return takeoff_velocity_x_;
    }
    [[nodiscard]] double takeoff_velocity_z() const noexcept {
        return takeoff_velocity_z_;
    }
    [[nodiscard]] const std::array<double, BC_SIDE_NUM> &
    wheel_departure_time() const noexcept { return wheel_departure_time_; }
    [[nodiscard]] const std::array<double, BC_SIDE_NUM> &
    wheel_landing_time() const noexcept { return wheel_landing_time_; }
    [[nodiscard]] const std::array<double, BC_SIDE_NUM> &
    wheel_landing_distance() const noexcept { return wheel_landing_distance_; }
    [[nodiscard]] const std::array<bool, BC_SIDE_NUM> &
    ramp_wheel_contact() const noexcept { return current_contacts_.ramp_wheel; }
    [[nodiscard]] std::array<double, 2> axle_velocity_truth(
        const mjData &data) const { return axle_velocity(data); }

private:
    struct ContactState {
        std::array<bool, BC_SIDE_NUM> ramp_wheel{};
        std::array<bool, BC_SIDE_NUM> ground_wheel{};
    };

    [[nodiscard]] ContactState read_contacts(const mjData &data) const;
    [[nodiscard]] std::array<double, 2> axle_velocity(
        const mjData &data) const;
    [[nodiscard]] double wheel_axis_x(const mjData &data, int side) const;
    void fail(const char *issue) noexcept;

    RampJumpSpec spec_;
    std::string name_;
    sim::RampJumpLayout layout_{};
    RampJumpPhase phase_{RampJumpPhase::disabled_settle};
    bc_operator_command_t command_{};
    double reset_time_{};
    double ready_hold_start_{-1.0};
    double speed_hold_start_{-1.0};
    double recovery_hold_start_{-1.0};
    double ramp_entry_velocity_{std::numeric_limits<double>::quiet_NaN()};
    double takeoff_time_{std::numeric_limits<double>::quiet_NaN()};
    double takeoff_x_{std::numeric_limits<double>::quiet_NaN()};
    double takeoff_velocity_x_{std::numeric_limits<double>::quiet_NaN()};
    double takeoff_velocity_z_{std::numeric_limits<double>::quiet_NaN()};
    std::array<double, BC_SIDE_NUM> wheel_departure_time_{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
    std::array<double, BC_SIDE_NUM> wheel_landing_time_{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
    std::array<double, BC_SIDE_NUM> wheel_landing_distance_{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    }};
    std::array<bool, BC_SIDE_NUM> previous_ramp_contact_{};
    ContactState current_contacts_{};
    bool ramp_contact_seen_{};
    bool balance_engaged_{};
    bool start_ready_{};
    bool entry_speed_stable_{};
    const char *issue_{"none"};
    int ramp_{};
    int ground_{};
    const mjModel *model_{};
    std::array<int, BC_SIDE_NUM> wheel_{};
    std::array<int, BC_SIDE_NUM> wheel_axis_{};
};

class RampJumpBenchmark {
public:
    RampJumpBenchmark(
        const std::filesystem::path &model_path,
        const std::filesystem::path &output_directory);

    [[nodiscard]] RampJumpResult run(const RampJumpSpec &spec);
    void write_summary(const RampJumpResult &result);

private:
    struct ContactObservation {
        bool non_wheel{};
        std::string first_non_wheel{"none"};
        double non_wheel_normal_force{};
        double strongest_non_wheel_normal_force{};
        double strongest_non_wheel_contact_x{};
        double strongest_non_wheel_contact_z{};
        std::array<bool, BC_SIDE_NUM> ramp_wheel{};
        std::array<bool, BC_SIDE_NUM> ground_wheel{};
    };

    [[nodiscard]] ContactObservation observe_contacts() const;
    void write_trace(
        CsvWriter &trace,
        const RampJumpScenario &scenario,
        const SimulationSample &sample,
        const ContactObservation &contact) const;

    std::filesystem::path output_directory_;
    sim::MujocoPlant plant_;
    sim::MujocoAdapter adapter_;
    SimulationSampler sampler_;
    CsvWriter summary_;
    int ramp_{};
    int ground_{};
    std::array<int, BC_SIDE_NUM> wheel_{};
    std::vector<int> robot_geoms_;
};

} // namespace balance::benchmark

#endif
