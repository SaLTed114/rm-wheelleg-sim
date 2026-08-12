#include "landing_suspension.hpp"

#include <algorithm>
#include <cmath>

namespace balance::benchmark {

LandingSuspensionController::LandingSuspensionController(
    const LandingSuspensionSpec &spec
) : spec_(spec) {
    reset();
}

void LandingSuspensionController::reset() noexcept {
    output_ = {};
}

void LandingSuspensionController::update(
    const bc_controller_snapshot_t &snapshot,
    const std::array<bool, BC_SIDE_NUM> &wheel_contact,
    const double timestep_seconds
) noexcept {
    const double raw_roll_force =
        -spec_.roll_kp * snapshot.roll -
        spec_.roll_kd * snapshot.roll_rate;
    const double roll_force = std::clamp(
        raw_roll_force, -spec_.roll_force_limit, spec_.roll_force_limit);
    const std::array<double, BC_SIDE_NUM> roll_sign{{+1.0, -1.0}};

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        bool captured_now = false;
        if (!output_.contact_latched[side] && wheel_contact[side]) {
            output_.contact_latched[side] = true;
            output_.captured_length[side] = snapshot.leg[side].length;
            output_.equilibrium_length[side] =
                output_.captured_length[side];
            captured_now = true;
            const auto &support = snapshot.support_force[side];
            const double initial_force =
                support.valid && std::isfinite(support.axial_force) &&
                    support.axial_force >= spec_.minimum_force &&
                    support.axial_force <= spec_.maximum_force ?
                support.axial_force : spec_.support_force;
            output_.applied_force[side] = initial_force;
        }
        if (!output_.contact_latched[side]) continue;

        if (!captured_now) {
            output_.equilibrium_length[side] = std::max(
                spec_.retracted_length,
                output_.equilibrium_length[side] -
                    spec_.retraction_speed * timestep_seconds);
        }

        const double requested =
            spec_.support_force +
            spec_.stiffness * (
                output_.equilibrium_length[side] -
                snapshot.leg[side].length) -
            spec_.damping * snapshot.leg[side].length_velocity +
            roll_sign[side] * roll_force;
        output_.requested_force[side] = requested;
        const double limited = std::clamp(
            requested, spec_.minimum_force, spec_.maximum_force);
        const double maximum_step = std::max(
            0.0, spec_.force_rate_limit * timestep_seconds);
        const double delta = std::clamp(
            limited - output_.applied_force[side],
            -maximum_step, maximum_step);
        output_.force_rate_limited[side] =
            std::abs(limited - output_.applied_force[side]) >
                maximum_step + 1.0e-12;
        output_.applied_force[side] += delta;
    }
}

} // namespace balance::benchmark
