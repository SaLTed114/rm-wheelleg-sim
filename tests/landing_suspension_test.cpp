#include <cmath>
#include <cstdlib>
#include <iostream>

#include "drop/landing_suspension.hpp"

namespace {

bool near(const double actual, const double expected) {
    return std::abs(actual - expected) <= 1.0e-5;
}

} // namespace

int main() {
    balance::benchmark::LandingSuspensionSpec spec;
    spec.stiffness = 800.0;
    spec.damping = 80.0;
    spec.force_rate_limit = 3000.0;
    spec.retraction_speed = 0.8;
    balance::benchmark::LandingSuspensionController suspension(spec);
    bc_controller_snapshot_t snapshot{};
    snapshot.leg[BC_L].length = 0.34F;
    snapshot.leg[BC_R].length = 0.32F;
    snapshot.support_force[BC_L].valid = 1U;
    snapshot.support_force[BC_L].axial_force = -10.0F;

    suspension.update(snapshot, {{true, false}}, 0.001);
    const auto first = suspension.output();
    if (!first.contact_latched[BC_L] || first.contact_latched[BC_R] ||
        !near(first.captured_length[BC_L], 0.34F) ||
        !near(first.equilibrium_length[BC_L], 0.34F) ||
        !near(first.applied_force[BC_L], spec.support_force)) {
        std::cerr << "single-leg touchdown was not captured\n";
        return EXIT_FAILURE;
    }

    snapshot.leg[BC_L].length = 0.33F;
    snapshot.leg[BC_L].length_velocity = -0.2F;
    suspension.update(snapshot, {{false, false}}, 0.001);
    const auto compressed = suspension.output();
    const double equilibrium = 0.34 - 0.8 * 0.001;
    const double requested =
        76.204 + 800.0 * (equilibrium - 0.33) + 80.0 * 0.2;
    if (!near(compressed.requested_force[BC_L], requested) ||
        !near(compressed.equilibrium_length[BC_L], equilibrium) ||
        !compressed.force_rate_limited[BC_L] ||
        !near(compressed.applied_force[BC_L], 79.204) ||
        compressed.contact_latched[BC_R]) {
        std::cerr << "impedance or force slew limit is incorrect\n";
        return EXIT_FAILURE;
    }

    for (int step = 0; step < 250; ++step) {
        suspension.update(snapshot, {{false, false}}, 0.001);
    }
    if (!near(suspension.output().equilibrium_length[BC_L],
              spec.retracted_length)) {
        std::cerr << "moving equilibrium did not stop at working length\n";
        return EXIT_FAILURE;
    }

    snapshot.roll = 1.0F;
    snapshot.roll_rate = 0.0F;
    snapshot.leg[BC_R].length = 0.32F;
    suspension.update(snapshot, {{false, true}}, 0.001);
    const auto both = suspension.output();
    if (!both.contact_latched[BC_R] ||
        !near(both.captured_length[BC_R], 0.32F) ||
        both.requested_force[BC_L] >= both.requested_force[BC_R]) {
        std::cerr << "roll differential or right touchdown is incorrect\n";
        return EXIT_FAILURE;
    }

    balance::benchmark::LandingSuspensionSpec limit_spec;
    limit_spec.stiffness = 10000.0;
    limit_spec.force_rate_limit = 1.0e9;
    balance::benchmark::LandingSuspensionController limited(limit_spec);
    bc_controller_snapshot_t limit_snapshot{};
    limit_snapshot.leg[BC_L].length = 0.34F;
    limited.update(limit_snapshot, {{true, false}}, 0.001);
    limit_snapshot.leg[BC_L].length = 0.30F;
    limited.update(limit_snapshot, {{false, false}}, 0.001);
    if (limited.output().requested_force[BC_L] <= limit_spec.maximum_force ||
        !near(limited.output().applied_force[BC_L],
              limit_spec.maximum_force)) {
        std::cerr << "axial force did not respect the 240 N limit\n";
        return EXIT_FAILURE;
    }

    suspension.reset();
    if (suspension.output().contact_latched[BC_L] ||
        suspension.output().contact_latched[BC_R]) {
        std::cerr << "reset did not clear touchdown latches\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
