#include "balance/state_machine/system.h"

#include <math.h>
#include <stdio.h>

static int control_uses_strategies(
    const bc_control_command_t *command,
    const bc_leg_length_strategy_t length_strategy,
    const bc_leg_angle_strategy_t angle_strategy,
    const bc_wheel_strategy_t wheel_strategy
) {
    if (command->wheel_strategy != wheel_strategy) return 0;

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (command->leg[side].length_strategy != length_strategy ||
            command->leg[side].angle_strategy != angle_strategy) return 0;
    }
    return 1;
}

int main() {
    bc_motion_config_t config;
    bc_system_t system;
    bc_operator_command_t operator_command = {0};
    bc_state_vector_t state = {0};
    bc_leg_kinematics_t leg[BC_SIDE_NUM] = {0};
    bc_control_command_t command;
    const bc_state_machine_input_t input = {
        .operator_command = &operator_command,
        .state = &state,
        .leg = leg,
        .timestep_seconds = 0.1F,
    };

    bc_motion_default_config(&config);
    if (config.leg_length != 0.18F ||
        config.engage_duration != 0.1F) {
        fputs("default motion config is incorrect\n", stderr);
        return 1;
    }
    bc_system_init(&system, &config);

    operator_command.balance_restart = 1U;
    bc_system_update(&system, &input, &command);
    if (system.state != BC_SYSTEM_OFF ||
        system.motion.state != BC_MOTION_IDLE ||
        !control_uses_strategies(
            &command,
            BC_LEG_LENGTH_DISABLED, BC_LEG_ANGLE_DISABLED,
            BC_WHEEL_DISABLED)) {
        fputs("disabled state machine did not remain off and idle\n", stderr);
        return 1;
    }

    operator_command.system_enabled = 1U;
    operator_command.balance_restart = 0U;
    bc_system_update(&system, &input, &command);
    if (system.state != BC_SYSTEM_ON ||
        system.motion.state != BC_MOTION_IDLE ||
        !control_uses_strategies(
            &command,
            BC_LEG_LENGTH_DISABLED, BC_LEG_ANGLE_DISABLED,
            BC_WHEEL_DISABLED)) {
        fputs("system enable did not wait for balance restart\n", stderr);
        return 1;
    }

    operator_command.balance_restart = 1U;
    bc_system_update(&system, &input, &command);
    if (system.motion.state != BC_MOTION_LEG_POSITIONING ||
        !control_uses_strategies(
            &command,
            BC_LEG_LENGTH_POSITION, BC_LEG_ANGLE_POSITION,
            BC_WHEEL_DISABLED)) {
        fputs("balance restart did not start leg positioning\n", stderr);
        return 1;
    }
    operator_command.balance_restart = 0U;

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        leg[side].length = config.leg_length;
        leg[side].angle_body = config.leg_angle_body;
    }

    bc_system_update(&system, &input, &command);
    leg[BC_L].length = 0.0F;
    bc_system_update(&system, &input, &command);
    leg[BC_L].length = config.leg_length;

    state.value[BC_STATE_S] = 1.0F;
    state.value[BC_STATE_PSI] = -0.5F;
    for (int index = BC_STATE_THETA_L; index < BC_STATE_NUM; ++index) {
        state.value[index] = 0.1F * (float)index;
    }
    operator_command.forward_velocity = 0.2F;
    operator_command.yaw_rate = -0.3F;
    for (int step = 0; step < 3; ++step) {
        bc_system_update(&system, &input, &command);
    }

    if (system.motion.state != BC_MOTION_BALANCE_ENGAGING ||
        !control_uses_strategies(
            &command,
            BC_LEG_LENGTH_POSITION_SUPPORT, BC_LEG_ANGLE_LQR,
            BC_WHEEL_LQR)) {
        fputs("stable legs did not engage balance control\n", stderr);
        return 1;
    }
    if (command.state_reference.value[BC_STATE_S] != 1.0F ||
        command.state_reference.value[BC_STATE_DS] != 0.0F ||
        command.state_reference.value[BC_STATE_PSI] != -0.5F ||
        command.state_reference.value[BC_STATE_DPSI] != 0.0F ||
        command.disabled_state_feedback != (
            BC_STATE_FEEDBACK_MASK(BC_STATE_S) |
            BC_STATE_FEEDBACK_MASK(BC_STATE_PSI))) {
        fputs("engaging balance accepted a motion command\n", stderr);
        return 1;
    }
    for (int index = BC_STATE_THETA_L; index < BC_STATE_NUM; ++index) {
        if (command.state_reference.value[index] != 0.0F) {
            fprintf(
                stderr, "attitude reference %d was not cleared\n", index);
            return 1;
        }
    }

    operator_command.forward_velocity = 10.0F;
    operator_command.yaw_rate = 20.0F;
    bc_system_update(&system, &input, &command);
    if (system.motion.state != BC_MOTION_ACTIVE ||
        fabsf(command.state_reference.value[BC_STATE_S] - 1.05F) > 1.0e-6F ||
        fabsf(command.state_reference.value[BC_STATE_DS] - 0.5F) > 1.0e-6F ||
        fabsf(command.state_reference.value[BC_STATE_PSI] + 0.35F) > 1.0e-6F ||
        fabsf(command.state_reference.value[BC_STATE_DPSI] - 1.5F) > 1.0e-6F ||
        command.disabled_state_feedback != 0U) {
        fputs("active balance did not use rate-limited targets\n", stderr);
        return 1;
    }

    state.value[BC_STATE_S] = 2.0F;
    state.value[BC_STATE_DS] = -0.4F;
    state.value[BC_STATE_PSI] = 0.75F;
    operator_command.forward_velocity = 0.2F;
    operator_command.yaw_rate = 0.0F;
    bc_system_update(&system, &input, &command);
    if (fabsf(command.state_reference.value[BC_STATE_S] - 1.07F) > 1.0e-6F ||
        fabsf(command.state_reference.value[BC_STATE_DS] - 0.2F) > 1.0e-6F ||
        fabsf(command.state_reference.value[BC_STATE_PSI] + 0.35F) > 1.0e-6F ||
        command.state_reference.value[BC_STATE_DPSI] != 0.0F ||
        command.disabled_state_feedback != 0U ||
        command.state_reference.value[BC_STATE_THETA_L] != 0.0F ||
        command.state_reference.value[BC_STATE_THETA_R] != 0.0F ||
        command.state_reference.value[BC_STATE_DTHETA_L] != 0.0F ||
        command.state_reference.value[BC_STATE_DTHETA_R] != 0.0F ||
        command.state_reference.value[BC_STATE_THETA_B] != 0.0F ||
        command.state_reference.value[BC_STATE_DTHETA_B] != 0.0F) {
        fputs("active reference followed the measured state\n", stderr);
        return 1;
    }

    operator_command.balance_restart = 1U;
    bc_system_update(&system, &input, &command);
    if (system.motion.state != BC_MOTION_LEG_POSITIONING ||
        system.motion.state_reference.value[BC_STATE_S] != 0.0F ||
        system.motion.state_reference.value[BC_STATE_PSI] != 0.0F ||
        system.motion.forward_velocity_ramp.value != 0.0F ||
        system.motion.yaw_rate_ramp.value != 0.0F ||
        system.motion.engage_hold.elapsed_seconds != 0.0F ||
        !control_uses_strategies(
            &command,
            BC_LEG_LENGTH_POSITION, BC_LEG_ANGLE_POSITION,
            BC_WHEEL_DISABLED)) {
        fputs("balance restart did not return to leg positioning\n", stderr);
        return 1;
    }

    operator_command.system_enabled = 0U;
    bc_system_update(&system, &input, &command);
    if (system.state != BC_SYSTEM_OFF ||
        system.motion.state != BC_MOTION_IDLE ||
        !control_uses_strategies(
            &command,
            BC_LEG_LENGTH_DISABLED, BC_LEG_ANGLE_DISABLED,
            BC_WHEEL_DISABLED)) {
        fputs("system disable did not reset the state machine\n", stderr);
        return 1;
    }

    system.state = BC_SYSTEM_FAULT;
    system.motion.state = BC_MOTION_BALANCE_ENGAGING;
    operator_command.system_enabled = 1U;
    bc_system_update(&system, &input, &command);
    if (system.state != BC_SYSTEM_OFF ||
        system.motion.state != BC_MOTION_IDLE ||
        !control_uses_strategies(
            &command,
            BC_LEG_LENGTH_DISABLED, BC_LEG_ANGLE_DISABLED,
            BC_WHEEL_DISABLED)) {
        fputs("fault placeholder did not fall back to off\n", stderr);
        return 1;
    }

    return 0;
}
