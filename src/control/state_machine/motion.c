#include "balance/state_machine/motion.h"

#include "balance/math_utils.h"

#include <math.h>
#include <string.h>

static void bc_motion_set_leg_control(
    const bc_motion_t *motion,
    const bc_leg_length_strategy_t length_strategy,
    const bc_leg_angle_strategy_t angle_strategy,
    bc_control_command_t *command
) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        command->leg[side].length_strategy = length_strategy;
        command->leg[side].angle_strategy = angle_strategy;
        command->leg[side].target.length = motion->config.leg_length;
        command->leg[side].target.angle_body =
            motion->config.leg_angle_body;
    }
}

static uint8_t bc_motion_legs_are_stable(
    const bc_motion_t *motion,
    const bc_leg_kinematics_t leg[BC_SIDE_NUM]
) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const uint8_t stable =
            fabsf(leg[side].length - motion->config.leg_length) <=
                motion->config.length_tolerance &&
            fabsf(leg[side].length_velocity) <=
                motion->config.length_velocity_tolerance &&
            fabsf(bc_wrap_anglef(
                leg[side].angle_body - motion->config.leg_angle_body)) <=
                motion->config.angle_tolerance &&
            fabsf(leg[side].angular_velocity) <=
                motion->config.angular_velocity_tolerance;
        if (!stable) return 0U;
    }
    return 1U;
}

static void bc_motion_transition(
    bc_motion_t *motion,
    const bc_state_machine_input_t *input
) {
    switch (motion->state) {
    case BC_MOTION_IDLE:
        break;

    case BC_MOTION_SELF_RIGHTING:
        motion->state = BC_MOTION_LEG_POSITIONING;
        break;

    case BC_MOTION_LEG_POSITIONING:
        if (!bc_condition_hold_update(
            &motion->leg_stable_hold,
            bc_motion_legs_are_stable(motion, input->leg),
            motion->config.stable_duration,
            input->timestep_seconds)) break;

        motion->state = BC_MOTION_BALANCE_ENGAGING;
        memset(
            &motion->state_reference, 0,
            sizeof(motion->state_reference));
        motion->state_reference.value[BC_STATE_S] =
            input->state->value[BC_STATE_S];
        motion->state_reference.value[BC_STATE_PSI] =
            input->state->value[BC_STATE_PSI];
        break;

    case BC_MOTION_BALANCE_ENGAGING:
    case BC_MOTION_ACTIVE:
        break;
    }
}

static void bc_motion_action(
    bc_motion_t *motion,
    const bc_state_machine_input_t *input,
    bc_control_command_t *output
) {
    switch (motion->state) {
    case BC_MOTION_IDLE:
    case BC_MOTION_SELF_RIGHTING:
        break;

    case BC_MOTION_LEG_POSITIONING:
        bc_motion_set_leg_control(
            motion,
            BC_LEG_LENGTH_POSITION, BC_LEG_ANGLE_POSITION,
            output);
        break;

    case BC_MOTION_BALANCE_ENGAGING: {
        bc_motion_set_leg_control(
            motion,
            BC_LEG_LENGTH_POSITION_SUPPORT, BC_LEG_ANGLE_LQR,
            output);
        output->wheel_strategy = BC_WHEEL_LQR;
        const float forward_velocity = bc_reference_ramp_update(
            &motion->forward_velocity_ramp,
            &motion->config.forward_velocity_ramp,
            input->operator_command->forward_velocity,
            input->timestep_seconds);
        const float yaw_rate = bc_reference_ramp_update(
            &motion->yaw_rate_ramp,
            &motion->config.yaw_rate_ramp,
            input->operator_command->yaw_rate,
            input->timestep_seconds);
        if (motion->config.position_feedback_enabled) {
            motion->state_reference.value[BC_STATE_S] +=
                forward_velocity * input->timestep_seconds;
        } else {
            motion->state_reference.value[BC_STATE_S] =
                input->state->value[BC_STATE_S];
        }
        motion->state_reference.value[BC_STATE_DS] =
            motion->config.velocity_feedback_enabled ?
                forward_velocity : input->state->value[BC_STATE_DS];
        motion->state_reference.value[BC_STATE_PSI] +=
            yaw_rate * input->timestep_seconds;
        motion->state_reference.value[BC_STATE_DPSI] =
            yaw_rate;
        output->state_reference = motion->state_reference;
        break;
    }

    case BC_MOTION_ACTIVE:
        break;
    }
}

void bc_motion_default_config(bc_motion_config_t *config) {
    *config = (bc_motion_config_t){
        .leg_length                 = 0.18F,
        .leg_angle_body             = -0.5F * BC_PI_F,
        .length_tolerance           = 0.025F,
        .length_velocity_tolerance  = 0.03F,
        .angle_tolerance            = 8.0F * BC_PI_F / 180.0F,
        .angular_velocity_tolerance = 0.15F,
        .stable_duration            = 0.25F,
        .position_feedback_enabled  = 1U,
        .velocity_feedback_enabled  = 1U,
        .forward_velocity_ramp = {
            .value_limit = 3.0F,
            .rate_limit = 5.0F,
        },
        .yaw_rate_ramp = {
            .value_limit = 4.0F * BC_PI_F,
            .rate_limit = 15.0F,
        },
    };
}

void bc_motion_init(
    bc_motion_t *motion,
    const bc_motion_config_t *config
) {
    motion->config = *config;
    bc_motion_reset(motion);
}

void bc_motion_reset(bc_motion_t *motion) {
    motion->state = BC_MOTION_IDLE;
    memset(
        &motion->state_reference, 0,
        sizeof(motion->state_reference));
    bc_reference_ramp_reset(&motion->forward_velocity_ramp);
    bc_reference_ramp_reset(&motion->yaw_rate_ramp);
    bc_condition_hold_reset(&motion->leg_stable_hold);
}

void bc_motion_start(bc_motion_t *motion) {
    bc_motion_reset(motion);
    motion->state = BC_MOTION_SELF_RIGHTING;
}

void bc_motion_update(
    bc_motion_t *motion,
    const bc_state_machine_input_t *input,
    bc_control_command_t *output
) {
    memset(output, 0, sizeof(*output));
    bc_motion_transition(motion, input);
    bc_motion_action(motion, input, output);
}

const char *bc_motion_state_name(const bc_motion_state_t state) {
    switch (state) {
    case BC_MOTION_IDLE:
        return "idle";
    case BC_MOTION_SELF_RIGHTING:
        return "self righting";
    case BC_MOTION_LEG_POSITIONING:
        return "positioning legs";
    case BC_MOTION_BALANCE_ENGAGING:
        return "engaging balance";
    case BC_MOTION_ACTIVE:
        return "active";
    }
    return "unknown";
}
