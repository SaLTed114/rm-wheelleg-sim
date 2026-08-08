#include "performance/performance_scenario.hpp"

#include <cmath>
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

    bool invalid_rejected = false;
    try {
        const PerformanceCaseSpec invalid{
            "invalid", PerformanceAxis::heading,
            2.0 * BC_PI, 1.0, 1.0, 1.0, 1.0};
        PerformanceScenario ignored(invalid);
    } catch (const std::invalid_argument &) {
        invalid_rejected = true;
    }
    if (!invalid_rejected) {
        std::cerr << "invalid custom timing was accepted\n";
        return 1;
    }
    return 0;
}
