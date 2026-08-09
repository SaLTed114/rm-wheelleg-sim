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

static void bc_motion_set_balance_control(
    const bc_motion_t *motion,
    bc_control_command_t *command
) {
    bc_motion_set_leg_control(
        motion,
        BC_LEG_LENGTH_POSITION_SUPPORT, BC_LEG_ANGLE_LQR,
        command);
    command->wheel_strategy = BC_WHEEL_LQR;
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
        bc_condition_hold_reset(&motion->engage_hold);
        break;

    case BC_MOTION_BALANCE_ENGAGING:
        if (bc_condition_hold_update(
                &motion->engage_hold, 1U,
                motion->config.engage_duration,
                input->timestep_seconds)) {
            motion->state = BC_MOTION_ACTIVE;
            bc_forward_mode_start(&motion->forward);
            bc_forward_reference_start(
                &motion->forward_reference,
                input->state->value[BC_STATE_S],
                &motion->state_reference);
            bc_yaw_reference_start(
                &motion->yaw_reference,
                input->state->value[BC_STATE_PSI],
                input->state->value[BC_STATE_DPSI],
                &motion->state_reference);
        }
        break;

    case BC_MOTION_ACTIVE:
        break;
    }
}

static void bc_motion_map_normal_command(
    bc_motion_t *motion,
    const bc_state_machine_input_t *input
) {
    const float front_error = bc_wrap_anglef(
        input->gimbal_feedback->relative_yaw);
    const float rear_error = bc_wrap_anglef(
        front_error + BC_PI_F);

    if (motion->alignment == BC_CHASSIS_FRONT) {
        if (fabsf(rear_error) + motion->config.alignment_hysteresis <
            fabsf(front_error)) {
            motion->alignment = BC_CHASSIS_REAR;
        }
    } else if (
        fabsf(front_error) + motion->config.alignment_hysteresis <
        fabsf(rear_error)
    ) {
        motion->alignment = BC_CHASSIS_FRONT;
    }

    const uint8_t rear = motion->alignment == BC_CHASSIS_REAR;
    motion->mapped_forward_velocity = rear ?
        -input->operator_command->forward_velocity :
        input->operator_command->forward_velocity;
    motion->heading_error = rear ? rear_error : front_error;
}

static void bc_motion_update_forward(
    bc_motion_t *motion,
    const bc_state_machine_input_t *input,
    const float forward_velocity,
    bc_control_command_t *output
) {
    const bc_forward_state_t previous_state = motion->forward.state;
    bc_forward_mode_update(
        &motion->forward,
        bc_forward_reference_requested(
            &motion->forward_reference,
            forward_velocity),
        motion->forward_reference.velocity_ramp.value,
        input->state->value[BC_STATE_DS],
        input->timestep_seconds,
        output);

    if (motion->forward.state == BC_FORWARD_HOLD &&
        previous_state != BC_FORWARD_HOLD) {
        bc_forward_reference_start(
            &motion->forward_reference,
            input->state->value[BC_STATE_S],
            &motion->state_reference);
    } else if (motion->forward.state == BC_FORWARD_VELOCITY) {
        bc_forward_reference_update(
            &motion->forward_reference,
            forward_velocity,
            input->timestep_seconds,
            &motion->state_reference);
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

    case BC_MOTION_BALANCE_ENGAGING:
        bc_motion_set_balance_control(motion, output);
        output->disabled_state_feedback =
            BC_STATE_FEEDBACK_MASK(BC_STATE_S) |
            BC_STATE_FEEDBACK_MASK(BC_STATE_PSI);
        output->state_reference = motion->state_reference;
        break;

    case BC_MOTION_ACTIVE:
        bc_motion_set_balance_control(motion, output);
        bc_motion_map_normal_command(motion, input);
        bc_motion_update_forward(
            motion, input, motion->mapped_forward_velocity, output);
        bc_yaw_reference_update(
            &motion->yaw_reference,
            input->state,
            motion->heading_error,
            input->gimbal_feedback->relative_yaw_rate,
            input->timestep_seconds,
            &motion->state_reference);
        output->state_reference = motion->state_reference;
        output->yaw_acceleration_reference =
            motion->yaw_reference.acceleration_reference;
        break;
    }
}

void bc_motion_default_config(bc_motion_config_t *config) {
    bc_forward_mode_config_t forward;
    bc_forward_reference_config_t forward_reference;
    bc_yaw_reference_config_t yaw_reference;
    bc_forward_mode_default_config(&forward);
    bc_forward_reference_default_config(&forward_reference);
    bc_yaw_reference_default_config(&yaw_reference);
    *config = (bc_motion_config_t){
        .leg_length                 = 0.18F,
        .leg_angle_body             = -0.5F * BC_PI_F,
        .length_tolerance           = 0.035F,
        .length_velocity_tolerance  = 0.03F,
        .angle_tolerance            = 8.0F * BC_PI_F / 180.0F,
        .angular_velocity_tolerance = 0.15F,
        .stable_duration            = 0.25F,
        .engage_duration            = 0.1F,
        .alignment_hysteresis       = 5.0F * BC_PI_F / 180.0F,
        .forward                    = forward,
        .forward_reference          = forward_reference,
        .yaw_reference              = yaw_reference,
    };
}

void bc_motion_init(
    bc_motion_t *motion,
    const bc_motion_config_t *config
) {
    motion->config = *config;
    bc_forward_mode_init(&motion->forward, &config->forward);
    bc_forward_reference_init(
        &motion->forward_reference,
        &config->forward_reference);
    bc_yaw_reference_init(
        &motion->yaw_reference, &config->yaw_reference);
    bc_motion_reset(motion);
}

void bc_motion_reset(bc_motion_t *motion) {
    motion->state = BC_MOTION_IDLE;
    memset(
        &motion->state_reference, 0,
        sizeof(motion->state_reference));
    motion->alignment = BC_CHASSIS_FRONT;
    motion->mapped_forward_velocity = 0.0F;
    motion->heading_error = 0.0F;
    bc_forward_mode_reset(&motion->forward);
    bc_forward_reference_reset(&motion->forward_reference);
    bc_yaw_reference_reset(&motion->yaw_reference);
    bc_condition_hold_reset(&motion->leg_stable_hold);
    bc_condition_hold_reset(&motion->engage_hold);
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

const char *bc_chassis_alignment_name(
    const bc_chassis_alignment_t alignment
) {
    switch (alignment) {
    case BC_CHASSIS_FRONT:
        return "front";
    case BC_CHASSIS_REAR:
        return "rear";
    }
    return "unknown";
}
