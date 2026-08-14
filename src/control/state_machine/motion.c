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
        command->leg[side].target.length =
            motion->leg_length_reference.value;
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
            fabsf(
                leg[side].length - motion->config.startup_leg_length) <=
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

static void bc_motion_map_command(
    bc_motion_t *motion,
    const bc_state_machine_input_t *input
) {
    const float front_error = bc_wrap_anglef(
        input->gimbal_feedback->relative_yaw);
    const float rear_error = bc_wrap_anglef(
        front_error + BC_PI_F);

    if (motion->step_task.request.force_front_alignment) {
        motion->alignment = BC_CHASSIS_FRONT;
        motion->heading_error = front_error;
        motion->mapped_forward_velocity =
            motion->step_task.request.suppress_forward ? 0.0F :
            fabsf(front_error) <=
                motion->step_task.config.alignment_tolerance ?
            input->operator_command->forward_velocity : 0.0F;
        return;
    }

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

static void bc_motion_reanchor_hold_reference(
    bc_motion_t *motion,
    const bc_state_machine_input_t *input
) {
    memset(
        &motion->state_reference, 0,
        sizeof(motion->state_reference));
    bc_forward_mode_start(&motion->forward);
    bc_forward_reference_start(
        &motion->forward_reference,
        input->state->value[BC_STATE_S],
        &motion->state_reference);
    bc_yaw_reference_start(
        &motion->yaw_reference,
        input->state->value[BC_STATE_PSI],
        0.0F,
        &motion->state_reference);
}

static void bc_motion_start_step_recovery_catch(
    bc_motion_t *motion
) {
    memset(
        &motion->state_reference, 0,
        sizeof(motion->state_reference));
    bc_forward_mode_start(&motion->forward);
    bc_forward_reference_reset(&motion->forward_reference);
    bc_yaw_reference_reset(&motion->yaw_reference);
}

static void bc_motion_set_step_transfer_control(
    const bc_step_task_request_t *request,
    bc_control_command_t *output
) {
    output->wheel_strategy = BC_WHEEL_DISABLED;
    output->yaw_acceleration_reference = 0.0F;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        output->leg[side].length_strategy =
            BC_LEG_LENGTH_POSITION_SUPPORT;
        output->leg[side].angle_strategy = BC_LEG_ANGLE_POSITION;
        output->leg[side].target.length = request->leg_length[side];
        output->leg[side].target.angle_body =
            request->leg_angle_body[side];
    }
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
        input->wheel_odometry_velocity,
        input->wheel_velocity_reliable,
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

static void bc_motion_apply_support_request(
    const bc_support_phase_t *phase,
    bc_control_command_t *output
) {
    const bc_support_phase_request_t *request = &phase->request;
    if (request->disable_wheels) {
        output->wheel_strategy = BC_WHEEL_DISABLED;
        output->yaw_acceleration_reference = 0.0F;
    }
    output->disabled_state_feedback |=
        request->disabled_state_feedback;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const bc_support_leg_request_t *leg = &request->leg[side];
        if (!leg->override_length) continue;
        output->leg[side].length_strategy = leg->length_strategy;
        if (leg->length_strategy == BC_LEG_LENGTH_AXIAL_FORCE) {
            output->leg[side].target.axial_force = leg->target;
        } else {
            output->leg[side].target.length = leg->target;
        }
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
        const float front_error = bc_wrap_anglef(
            input->gimbal_feedback->relative_yaw);
        const uint8_t step_command_eligible =
            input->operator_command->task == BC_OPERATOR_TASK_STEP_DOCK &&
            !motion->step_task.command_rearm_required;
        const float support_working_leg_length =
            motion->step_task.request.active ?
                motion->step_task.request.working_leg_length :
            step_command_eligible ?
                motion->step_task.config.prepare_leg_length :
                motion->config.leg_length;
        bc_support_phase_update(
            &motion->support_phase, input, support_working_leg_length);
        bc_step_task_update(
            &motion->step_task, input,
            motion->support_phase.state, front_error);
        bc_motion_map_command(motion, input);

        const bc_step_task_request_t *step_request =
            &motion->step_task.request;
        if (step_request->recovery_entered) {
            bc_motion_start_step_recovery_catch(motion);
            motion->leg_length_reference.value =
                step_request->working_leg_length;
        }
        if (step_request->recovery_reference_capture) {
            bc_motion_reanchor_hold_reference(motion, input);
            motion->leg_length_reference.value =
                step_request->working_leg_length;
        }
        if (step_request->control_mode == BC_STEP_CONTROL_PASSIVE) {
            motion->mapped_forward_velocity = 0.0F;
            break;
        }
        if (step_request->control_mode == BC_STEP_CONTROL_TRANSFER) {
            motion->mapped_forward_velocity = 0.0F;
            bc_motion_set_step_transfer_control(step_request, output);
            break;
        }
        if (step_request->control_mode == BC_STEP_CONTROL_RECOVER) {
            motion->mapped_forward_velocity = 0.0F;
            motion->leg_length_reference.value =
                step_request->working_leg_length;
            bc_motion_set_balance_control(motion, output);
            output->state_reference = motion->state_reference;
            output->yaw_acceleration_reference = 0.0F;
            if (step_request->suppress_position_heading_feedback) {
                output->disabled_state_feedback |=
                    BC_STATE_FEEDBACK_MASK(BC_STATE_S) |
                    BC_STATE_FEEDBACK_MASK(BC_STATE_PSI);
            }
            if (motion->step_task.state == BC_STEP_TASK_COMPLETE) {
                bc_support_phase_reset(&motion->support_phase);
            }
            break;
        }

        const float working_leg_length = step_request->active ?
            step_request->working_leg_length : motion->config.leg_length;
        bc_reference_ramp_update(
            &motion->leg_length_reference,
            &motion->config.leg_length_ramp,
            working_leg_length,
            input->timestep_seconds);
        bc_motion_set_balance_control(motion, output);
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
        bc_motion_apply_support_request(&motion->support_phase, output);
        break;
    }
}

void bc_motion_default_config(bc_motion_config_t *config) {
    bc_forward_mode_config_t forward;
    bc_forward_reference_config_t forward_reference;
    bc_yaw_reference_config_t yaw_reference;
    bc_step_task_config_t step_task;
    bc_support_phase_config_t support_phase;
    bc_forward_mode_default_config(&forward);
    bc_forward_reference_default_config(&forward_reference);
    bc_yaw_reference_default_config(&yaw_reference);
    bc_step_task_default_config(&step_task);
    bc_support_phase_default_config(&support_phase);
    *config = (bc_motion_config_t){
        .startup_leg_length         = 0.18F,
        .leg_length                 = 0.18F,
        .leg_length_ramp            = {
            .value_limit = 0.39F,
            .rate_limit = 0.40F,
        },
        .leg_angle_body             = -0.5F * BC_PI_F,
        .length_tolerance           = 0.035F,
        .length_velocity_tolerance  = 0.03F,
        .angle_tolerance            = 8.0F * BC_PI_F / 180.0F,
        .angular_velocity_tolerance = 0.15F,
        .stable_duration            = 0.10F,
        .engage_duration            = 0.05F,
        .alignment_hysteresis       = 5.0F * BC_PI_F / 180.0F,
        .forward                    = forward,
        .forward_reference          = forward_reference,
        .yaw_reference              = yaw_reference,
        .step_task                  = step_task,
        .support_phase              = support_phase,
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
    bc_step_task_init(&motion->step_task, &config->step_task);
    bc_support_phase_init(
        &motion->support_phase, &config->support_phase);
    bc_motion_reset(motion);
}

void bc_motion_reset(bc_motion_t *motion) {
    motion->state = BC_MOTION_IDLE;
    memset(
        &motion->state_reference, 0,
        sizeof(motion->state_reference));
    motion->alignment = BC_CHASSIS_FRONT;
    bc_reference_ramp_reset(&motion->leg_length_reference);
    motion->leg_length_reference.value =
        motion->config.startup_leg_length;
    motion->mapped_forward_velocity = 0.0F;
    motion->heading_error = 0.0F;
    bc_forward_mode_reset(&motion->forward);
    bc_forward_reference_reset(&motion->forward_reference);
    bc_yaw_reference_reset(&motion->yaw_reference);
    bc_step_task_reset(&motion->step_task);
    bc_support_phase_reset(&motion->support_phase);
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
