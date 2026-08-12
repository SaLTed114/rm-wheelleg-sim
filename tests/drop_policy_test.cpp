#include <cstdlib>
#include <iostream>

#include "drop/drop_policy.hpp"

namespace {

bc_control_command_t ground_command() {
    bc_control_command_t command{};
    command.wheel_strategy = BC_WHEEL_LQR;
    command.yaw_acceleration_reference = 3.0F;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        command.leg[side].length_strategy =
            BC_LEG_LENGTH_POSITION_SUPPORT;
        command.leg[side].angle_strategy = BC_LEG_ANGLE_LQR;
    }
    return command;
}

bool verify_length_only() {
    auto command = ground_command();
    balance::benchmark::apply_drop_air_policy(
        balance::benchmark::DropAirPolicy::length_only, command);
    if (command.wheel_strategy != BC_WHEEL_DISABLED ||
        command.yaw_acceleration_reference != 0.0F ||
        command.disabled_state_feedback != 0U) return false;
    for (const auto &leg : command.leg) {
        if (leg.length_strategy != BC_LEG_LENGTH_POSITION ||
            leg.angle_strategy != BC_LEG_ANGLE_DISABLED) return false;
    }
    return true;
}

bool verify_leg_lqr() {
    auto command = ground_command();
    balance::benchmark::apply_drop_air_policy(
        balance::benchmark::DropAirPolicy::leg_lqr, command);
    const uint16_t expected =
        BC_STATE_FEEDBACK_MASK(BC_STATE_S) |
        BC_STATE_FEEDBACK_MASK(BC_STATE_DS) |
        BC_STATE_FEEDBACK_MASK(BC_STATE_PSI) |
        BC_STATE_FEEDBACK_MASK(BC_STATE_DPSI) |
        BC_STATE_FEEDBACK_MASK(BC_STATE_THETA_B) |
        BC_STATE_FEEDBACK_MASK(BC_STATE_DTHETA_B);
    if (command.wheel_strategy != BC_WHEEL_DISABLED ||
        command.yaw_acceleration_reference != 0.0F ||
        command.disabled_state_feedback != expected) return false;
    for (const auto &leg : command.leg) {
        if (leg.length_strategy != BC_LEG_LENGTH_POSITION ||
            leg.angle_strategy != BC_LEG_ANGLE_LQR) return false;
    }
    return true;
}

} // namespace

int main() {
    if (!verify_length_only() || !verify_leg_lqr()) {
        std::cerr << "drop air policy selection is incorrect\n";
        return EXIT_FAILURE;
    }
    balance::benchmark::DropContactLatch latch;
    if (latch.update(false) || !latch.update(true) ||
        !latch.update(false) || !latch.latched()) {
        std::cerr << "drop touchdown latch reset after contact loss\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
