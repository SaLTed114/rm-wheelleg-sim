#include "drop_policy.hpp"

#include <stdexcept>
#include <string>

namespace balance::benchmark {

const char *drop_air_policy_name(const DropAirPolicy policy) noexcept {
    switch (policy) {
    case DropAirPolicy::length_only: return "length_only";
    case DropAirPolicy::leg_lqr: return "leg_lqr";
    }
    return "unknown";
}

DropAirPolicy parse_drop_air_policy(const std::string_view name) {
    if (name == "length_only") return DropAirPolicy::length_only;
    if (name == "leg_lqr") return DropAirPolicy::leg_lqr;
    throw std::invalid_argument(
        "unknown drop air policy '" + std::string(name) + "'");
}

void apply_drop_air_policy(
    const DropAirPolicy policy,
    bc_control_command_t &command
) noexcept {
    command.wheel_strategy = BC_WHEEL_DISABLED;
    command.yaw_acceleration_reference = 0.0F;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        command.leg[side].length_strategy = BC_LEG_LENGTH_POSITION;
        command.leg[side].angle_strategy =
            policy == DropAirPolicy::leg_lqr ?
                BC_LEG_ANGLE_LQR : BC_LEG_ANGLE_DISABLED;
    }

    if (policy == DropAirPolicy::leg_lqr) {
        command.disabled_state_feedback =
            BC_STATE_FEEDBACK_MASK(BC_STATE_S) |
            BC_STATE_FEEDBACK_MASK(BC_STATE_DS) |
            BC_STATE_FEEDBACK_MASK(BC_STATE_PSI) |
            BC_STATE_FEEDBACK_MASK(BC_STATE_DPSI) |
            BC_STATE_FEEDBACK_MASK(BC_STATE_THETA_B) |
            BC_STATE_FEEDBACK_MASK(BC_STATE_DTHETA_B);
    } else {
        command.disabled_state_feedback = 0U;
    }
}

} // namespace balance::benchmark
