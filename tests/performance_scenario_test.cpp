#include "performance/performance_scenario.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace {

bool near(const float value, const float expected) {
    return std::abs(value - expected) < 1.0e-5F;
}

} // namespace

int main() {
    using balance::benchmark::PerformanceAxis;
    using balance::benchmark::PerformanceCaseSpec;
    using balance::benchmark::PerformancePhase;
    using balance::benchmark::PerformanceScenario;

    const PerformanceCaseSpec spec{
        "custom", PerformanceAxis::forward, 2.0, 1.0, 0.2, 0.3, 0.4};
    PerformanceScenario scenario(spec);
    bc_controller_snapshot_t snapshot{};

    scenario.update(snapshot, 2.0);
    if (scenario.phase() != PerformancePhase::engaging ||
        !scenario.command().balance_restart) {
        std::cerr << "scenario did not begin engagement\n";
        return 1;
    }

    snapshot.state_machine.motion = BC_MOTION_ACTIVE;
    scenario.update(snapshot, 2.01);
    scenario.update(snapshot, 2.41);
    scenario.update(snapshot, 2.91);
    if (scenario.phase() != PerformancePhase::target_ramp ||
        !near(scenario.command().forward_velocity, 0.5F)) {
        std::cerr << "custom command ramp was not applied\n";
        return 1;
    }

    scenario.update(snapshot, 4.41);
    if (scenario.phase() != PerformancePhase::target_hold ||
        !near(scenario.command().forward_velocity, 2.0F)) {
        std::cerr << "scenario did not reach custom target\n";
        return 1;
    }

    scenario.update(snapshot, 4.61);
    if (scenario.phase() != PerformancePhase::stop_ramp ||
        !near(scenario.command().forward_velocity, 2.0F)) {
        std::cerr << "custom target hold duration was ignored\n";
        return 1;
    }

    scenario.update(snapshot, 6.61);
    if (scenario.phase() != PerformancePhase::stop_settle ||
        !near(scenario.command().forward_velocity, 0.0F)) {
        std::cerr << "scenario did not finish its stop ramp\n";
        return 1;
    }

    scenario.update(snapshot, 6.911);
    if (!scenario.finished()) {
        std::cerr << "custom settle duration was ignored\n";
        return 1;
    }

    const PerformanceCaseSpec heading_spec{
        "heading", PerformanceAxis::heading, BC_PI, 10.0,
        0.2, 0.3, 0.4};
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

    const PerformanceCaseSpec coupled_spec{
        "coupled", PerformanceAxis::heading, BC_PI, 10.0,
        0.2, 0.3, 1.0, 2.3, 0.5};
    PerformanceScenario coupled(coupled_spec);
    snapshot = {};
    coupled.update(snapshot, 2.0);
    snapshot.state_machine.motion = BC_MOTION_ACTIVE;
    coupled.update(snapshot, 2.01);
    coupled.update(snapshot, 2.50);
    if (coupled.command().forward_velocity != 0.0F) {
        std::cerr << "coupled forward command started before its lead\n";
        return 1;
    }
    coupled.update(snapshot, 2.51);
    if (!near(coupled.command().forward_velocity, 2.3F) ||
        coupled.gimbal().world_yaw_rate != 0.0F) {
        std::cerr << "coupled forward lead was not applied independently\n";
        return 1;
    }
    coupled.update(snapshot, 3.01);
    coupled.update(snapshot, 3.02);
    if (!near(coupled.command().forward_velocity, 2.3F) ||
        coupled.gimbal().world_yaw_rate <= 0.0F) {
        std::cerr << "coupled heading command did not follow its lead\n";
        return 1;
    }

    const PerformanceCaseSpec two_pi{
        "two_pi", PerformanceAxis::heading,
        2.0 * BC_PI, 3.0, 1.0, 1.0, 1.0};
    PerformanceScenario maximum_heading(two_pi);
    (void)maximum_heading;

    bool invalid_rejected = false;
    try {
        const PerformanceCaseSpec invalid{
            "invalid", PerformanceAxis::heading,
            2.01 * BC_PI, 1.0, 1.0, 1.0, 1.0};
        PerformanceScenario ignored(invalid);
    } catch (const std::invalid_argument &) {
        invalid_rejected = true;
    }
    if (!invalid_rejected) {
        std::cerr << "out-of-range heading target was accepted\n";
        return 1;
    }

    const PerformanceCaseSpec &figure_eight_spec =
        balance::benchmark::motion_cases().front();
    if (balance::benchmark::find_performance_case(
            figure_eight_spec.name) != &figure_eight_spec) {
        std::cerr << "figure-eight case was not registered\n";
        return 1;
    }

    PerformanceScenario figure_eight(figure_eight_spec);
    snapshot = {};
    constexpr float timestep = 0.001F;
    float previous_forward_command = 0.0F;
    float maximum_forward_command = 0.0F;
    float maximum_yaw_rate = 0.0F;
    float minimum_yaw_rate = 0.0F;
    bool saw_left_arc = false;
    bool saw_right_arc = false;
    for (int step = 0; step < 15000 && !figure_eight.finished(); ++step) {
        const double time = static_cast<double>(step) * timestep;
        if (time >= 2.001) snapshot.state_machine.motion = BC_MOTION_ACTIVE;

        const float velocity_step = 5.0F * timestep;
        const float velocity_error = previous_forward_command -
            snapshot.state_reference.value[BC_STATE_DS];
        snapshot.state_reference.value[BC_STATE_DS] += std::clamp(
            velocity_error, -velocity_step, velocity_step);
        snapshot.state_reference.value[BC_STATE_S] +=
            snapshot.state_reference.value[BC_STATE_DS] * timestep;

        figure_eight.update(snapshot, time);
        previous_forward_command =
            figure_eight.command().forward_velocity;
        maximum_forward_command = std::max(
            maximum_forward_command, previous_forward_command);
        maximum_yaw_rate = std::max(
            maximum_yaw_rate, figure_eight.gimbal().world_yaw_rate);
        minimum_yaw_rate = std::min(
            minimum_yaw_rate, figure_eight.gimbal().world_yaw_rate);
        saw_left_arc = saw_left_arc ||
            std::strcmp(
                figure_eight.phase_name(), "figure_eight_left_arc") == 0;
        saw_right_arc = saw_right_arc ||
            std::strcmp(
                figure_eight.phase_name(), "figure_eight_right_arc") == 0;
    }
    if (!figure_eight.finished() || !saw_left_arc || !saw_right_arc ||
        !near(maximum_forward_command, 3.0F) ||
        maximum_yaw_rate < 1.49F * BC_PI_F ||
        minimum_yaw_rate > -1.49F * BC_PI_F ||
        std::abs(figure_eight.gimbal().world_yaw) > 0.02F) {
        std::cerr << "figure-eight command trajectory is incomplete\n";
        return 1;
    }
    return 0;
}
