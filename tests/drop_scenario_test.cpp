#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "common/simulation_sample.hpp"
#include "drop/drop_scenario.hpp"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: drop_scenario_test <model.xml>\n";
        return EXIT_FAILURE;
    }

    balance::sim::MujocoPlant plant(argv[1], 0.001);
    balance::sim::MujocoAdapter adapter(plant.model());
    balance::sim::SimulationRunner runner(plant, adapter);
    balance::benchmark::SimulationSampler sampler(plant.model());
    const auto *spec = balance::benchmark::find_drop_case(
        "leg_lqr_pitch_rate_pos_0p5");
    if (spec == nullptr) {
        std::cerr << "drop case lookup failed\n";
        return EXIT_FAILURE;
    }

    balance::benchmark::DropScenario scenario(*spec, plant.model());
    bool standing_heading_captured = false;
    double standing_heading = 0.0;
    double maximum_standing_yaw_error = 0.0;
    double maximum_standing_yaw_rate = 0.0;
    for (int step = 0; step < 15000 && !scenario.finished(); ++step) {
        scenario.step(plant, runner, sampler);
        if (scenario.phase() == balance::benchmark::DropPhase::standing &&
            runner.snapshot().state_machine.motion == BC_MOTION_ACTIVE) {
            if (!standing_heading_captured) {
                standing_heading =
                    runner.snapshot().state.value[BC_STATE_PSI];
                standing_heading_captured = true;
            }
            maximum_standing_yaw_error = std::max(
                maximum_standing_yaw_error,
                std::abs(static_cast<double>(
                    runner.snapshot().state.value[BC_STATE_PSI]) -
                    standing_heading));
            maximum_standing_yaw_rate = std::max(
                maximum_standing_yaw_rate,
                std::abs(static_cast<double>(
                    runner.snapshot().state.value[BC_STATE_DPSI])));
        }
    }
    if (!standing_heading_captured ||
        maximum_standing_yaw_error > 0.1 ||
        maximum_standing_yaw_rate > 0.3) {
        std::cerr << "drop startup did not hold heading: yaw_error="
                  << maximum_standing_yaw_error
                  << ", yaw_rate=" << maximum_standing_yaw_rate << '\n';
        return EXIT_FAILURE;
    }
    if (!scenario.finished() ||
        scenario.phase() != balance::benchmark::DropPhase::complete ||
        !scenario.balance_engaged() || !scenario.touchdown_latched()) {
        std::cerr << "drop GUI scenario did not complete\n";
        return EXIT_FAILURE;
    }
    for (const double clearance : scenario.release_clearance()) {
        if (std::abs(clearance - spec->wheel_clearance) > 2.0e-3) {
            std::cerr << "drop GUI scenario clearance is incorrect\n";
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}
