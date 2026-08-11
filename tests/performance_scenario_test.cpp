#include "performance/performance_benchmark.hpp"
#include "performance/performance_metrics.hpp"
#include "performance/performance_scenario.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

bool near(const float value, const float expected) {
    return std::abs(value - expected) < 1.0e-5F;
}

} // namespace

int main() {
    using balance::benchmark::PerformanceCaseKind;
    using balance::benchmark::PerformanceCaseSpec;
    using balance::benchmark::PerformancePhase;
    using balance::benchmark::PerformanceScenario;

    double positive_crossing = std::numeric_limits<double>::quiet_NaN();
    double negative_crossing = std::numeric_limits<double>::quiet_NaN();
    const double positive_progress =
        balance::benchmark::normalized_response_progress(2.7, 0.0, 3.0);
    const double negative_progress =
        balance::benchmark::normalized_response_progress(-2.7, 0.0, -3.0);
    balance::benchmark::capture_response_crossing(
        positive_progress, 0.9, 0.7, positive_crossing);
    balance::benchmark::capture_response_crossing(
        negative_progress, 0.9, 0.7, negative_crossing);
    if (std::abs(positive_crossing - 0.7) > 1.0e-12 ||
        std::abs(negative_crossing - 0.7) > 1.0e-12 ||
        std::abs(balance::benchmark::response_overshoot(
                     1.1, 0.0, -3.0) - 0.3) > 1.0e-12 ||
        std::abs(balance::benchmark::t10_t90_acceleration(
                     0.2, 0.8, 0.0, -3.0) - 4.0) > 1.0e-12) {
        std::cerr << "signed response metrics are inconsistent\n";
        return 1;
    }

    balance::benchmark::PerformanceBenchmarkConfig benchmark_config{};
    benchmark_config.forward_acceleration_rate = 7.5;
    const bc_controller_config_t controller_config =
        balance::benchmark::performance_controller_config(benchmark_config);
    if (!near(
            controller_config.motion.forward_reference.velocity_ramp.
                rate_limit,
            7.5F)) {
        std::cerr << "forward acceleration override was not configured\n";
        return 1;
    }

    PerformanceCaseSpec forward_spec{};
    forward_spec.name = "custom";
    forward_spec.kind = PerformanceCaseKind::forward_response;
    forward_spec.forward_target = 2.0;
    forward_spec.forward_rate = 1.0;
    forward_spec.target_hold_seconds = 0.2;
    forward_spec.stop_settle_seconds = 0.3;
    forward_spec.standing_seconds = 0.4;
    PerformanceScenario forward(forward_spec);
    bc_controller_snapshot_t snapshot{};

    forward.update(snapshot, 2.0);
    if (forward.phase() != PerformancePhase::engaging ||
        !forward.command().balance_restart) {
        std::cerr << "scenario did not begin engagement\n";
        return 1;
    }
    snapshot.state_machine.motion = BC_MOTION_ACTIVE;
    forward.update(snapshot, 2.01);
    forward.update(snapshot, 2.41);
    if (forward.phase() != PerformancePhase::target_ramp ||
        !near(forward.command().forward_velocity, 2.0F)) {
        std::cerr << "forward target was not applied\n";
        return 1;
    }
    forward.update(snapshot, 4.41);
    if (forward.phase() != PerformancePhase::target_hold) {
        std::cerr << "forward ramp duration was ignored\n";
        return 1;
    }
    forward.update(snapshot, 4.61);
    forward.update(snapshot, 6.61);
    if (forward.phase() != PerformancePhase::stop_settle ||
        forward.command().forward_velocity != 0.0F) {
        std::cerr << "forward stop ramp did not finish\n";
        return 1;
    }
    forward.update(snapshot, 6.911);
    if (!forward.finished()) {
        std::cerr << "stop settle duration was ignored\n";
        return 1;
    }

    PerformanceCaseSpec heading_spec{};
    heading_spec.name = "heading";
    heading_spec.kind = PerformanceCaseKind::heading_response;
    heading_spec.yaw_target = BC_PI;
    heading_spec.yaw_rate = 10.0;
    heading_spec.target_hold_seconds = 0.2;
    heading_spec.stop_settle_seconds = 0.3;
    heading_spec.standing_seconds = 0.4;
    PerformanceScenario heading(heading_spec);
    snapshot = {};
    heading.update(snapshot, 2.0);
    snapshot.state_machine.motion = BC_MOTION_ACTIVE;
    heading.update(snapshot, 2.01);
    heading.update(snapshot, 2.41);
    heading.update(snapshot, 2.42);
    if (heading.phase() != PerformancePhase::target_ramp ||
        heading.gimbal().world_yaw <= 0.0F ||
        heading.gimbal().world_yaw_rate <= 0.0F) {
        std::cerr << "heading scenario did not drive virtual gimbal\n";
        return 1;
    }

    PerformanceCaseSpec turn_spec{};
    turn_spec.name = "turn";
    turn_spec.kind = PerformanceCaseKind::steady_turn;
    turn_spec.forward_target = 2.0;
    turn_spec.yaw_target = BC_PI;
    turn_spec.forward_rate = 5.0;
    turn_spec.yaw_rate = 10.0;
    turn_spec.target_hold_seconds = 2.0;
    turn_spec.stop_settle_seconds = 0.2;
    turn_spec.standing_seconds = 0.2;
    PerformanceScenario turn(turn_spec);
    snapshot = {};
    turn.update(snapshot, 2.0);
    snapshot.state_machine.motion = BC_MOTION_ACTIVE;
    turn.update(snapshot, 2.01);
    turn.update(snapshot, 2.21);
    turn.update(snapshot, 2.611);
    if (turn.phase() != PerformancePhase::entry_wait ||
        turn.entry_ready()) {
        std::cerr << "turn did not wait for its actual entry state\n";
        return 1;
    }
    snapshot.state.value[BC_STATE_DS] = 2.0F;
    turn.update(snapshot, 2.62, false);
    turn.update(snapshot, 3.0, true);
    turn.update(snapshot, 3.49, true);
    if (turn.entry_ready()) {
        std::cerr << "turn readiness did not require a full stable window\n";
        return 1;
    }
    turn.update(snapshot, 3.501, true);
    if (!turn.entry_ready() || turn.phase() != PerformancePhase::yaw_ramp) {
        std::cerr << "stable turn entry was not accepted\n";
        return 1;
    }
    turn.update(snapshot, 3.511, true);
    if (turn.gimbal().world_yaw_rate <= 0.0F ||
        !near(turn.command().forward_velocity, 2.0F)) {
        std::cerr << "turn did not combine forward and yaw commands\n";
        return 1;
    }

    PerformanceScenario timeout(turn_spec);
    snapshot = {};
    timeout.update(snapshot, 2.0);
    snapshot.state_machine.motion = BC_MOTION_ACTIVE;
    timeout.update(snapshot, 2.01);
    timeout.update(snapshot, 2.21);
    timeout.update(snapshot, 2.611);
    timeout.update(snapshot, 7.612, true);
    if (!timeout.entry_timed_out() ||
        timeout.phase() != PerformancePhase::forward_stop_ramp) {
        std::cerr << "turn entry timeout did not abort the turn\n";
        return 1;
    }

    const auto &formal = balance::benchmark::formal_performance_cases();
    const auto &trajectories =
        balance::benchmark::trajectory_performance_cases();
    if (formal.size() != 4U ||
        balance::benchmark::find_performance_case("forward_pos_3") ==
            nullptr ||
        trajectories.size() != 1U ||
        balance::benchmark::find_performance_case(
            "figure_eight_open_loop") == nullptr ||
        balance::benchmark::find_performance_case("figure_eight_cross") !=
            nullptr) {
        std::cerr << "formal case registry was not reduced as intended\n";
        return 1;
    }

    PerformanceScenario figure_eight(trajectories.front());
    snapshot = {};
    constexpr float timestep = 0.001F;
    float previous_forward_command = 0.0F;
    float maximum_yaw_rate = 0.0F;
    float minimum_yaw_rate = 0.0F;
    bool saw_left = false;
    bool saw_right = false;
    for (int step = 0; step < 15000 && !figure_eight.finished(); ++step) {
        const double time = static_cast<double>(step) * timestep;
        if (time >= 2.001) snapshot.state_machine.motion = BC_MOTION_ACTIVE;
        const float velocity_step = 5.0F * timestep;
        snapshot.state_reference.value[BC_STATE_DS] += std::clamp(
            previous_forward_command -
                snapshot.state_reference.value[BC_STATE_DS],
            -velocity_step, velocity_step);
        snapshot.state.value[BC_STATE_DS] =
            snapshot.state_reference.value[BC_STATE_DS];
        figure_eight.update(snapshot, time, true);
        previous_forward_command =
            figure_eight.command().forward_velocity;
        maximum_yaw_rate = std::max(
            maximum_yaw_rate, figure_eight.gimbal().world_yaw_rate);
        minimum_yaw_rate = std::min(
            minimum_yaw_rate, figure_eight.gimbal().world_yaw_rate);
        saw_left = saw_left ||
            std::string(figure_eight.phase_name()) ==
                "figure_eight_left_drive";
        saw_right = saw_right ||
            std::string(figure_eight.phase_name()) ==
                "figure_eight_right_drive";
    }
    if (!figure_eight.finished() || !saw_left || !saw_right ||
        maximum_yaw_rate < BC_PI_F - 0.02F ||
        minimum_yaw_rate > -BC_PI_F + 0.02F ||
        std::abs(figure_eight.gimbal().world_yaw_rate) > 0.02F) {
        std::cerr << "measured figure-eight command is incomplete\n";
        return 1;
    }

    bool invalid_rejected = false;
    try {
        PerformanceCaseSpec invalid = turn_spec;
        invalid.yaw_target = 2.01 * BC_PI;
        PerformanceScenario ignored(invalid);
    } catch (const std::invalid_argument &) {
        invalid_rejected = true;
    }
    if (!invalid_rejected) {
        std::cerr << "out-of-range yaw target was accepted\n";
        return 1;
    }
    return 0;
}
