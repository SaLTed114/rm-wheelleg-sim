#include "balance/state_machine/drive.h"

#include "balance/math_utils.h"

#include <math.h>

static float bc_drive_apply_deadband(
    const float value, const float deadband
) {
    return fabsf(value) <= deadband ? 0.0F : value;
}

static float bc_drive_forward_target(
    const bc_drive_t *drive,
    const bc_state_machine_input_t *input
) {
    return bc_drive_apply_deadband(
        input->operator_command->forward_velocity,
        drive->config.forward_command_deadband);
}

static float bc_drive_yaw_target(
    const bc_drive_t *drive,
    const bc_state_machine_input_t *input
) {
    return bc_drive_apply_deadband(
        input->operator_command->yaw_rate,
        drive->config.yaw_command_deadband);
}

static void bc_drive_capture_position(
    const bc_state_machine_input_t *input,
    bc_state_vector_t *state_reference
) {
    state_reference->value[BC_STATE_S] =
        input->state->value[BC_STATE_S];
    state_reference->value[BC_STATE_DS] = 0.0F;
    state_reference->value[BC_STATE_PSI] =
        input->state->value[BC_STATE_PSI];
    state_reference->value[BC_STATE_DPSI] = 0.0F;
}

static uint8_t bc_drive_motion_requested(
    const bc_drive_t *drive,
    const bc_state_machine_input_t *input
) {
    return bc_drive_forward_target(drive, input) != 0.0F ||
        bc_drive_yaw_target(drive, input) != 0.0F;
}

static uint8_t bc_drive_is_stopped(
    const bc_drive_t *drive,
    const bc_state_machine_input_t *input
) {
    return !bc_drive_motion_requested(drive, input) &&
        drive->forward_velocity_ramp.value == 0.0F &&
        drive->yaw_rate_ramp.value == 0.0F &&
        fabsf(input->wheel_odometry_velocity) <
            drive->config.stop_wheel_velocity_tolerance &&
        fabsf(input->state->value[BC_STATE_DS]) <
            drive->config.stop_forward_velocity_tolerance &&
        fabsf(input->state->value[BC_STATE_DPSI]) <
            drive->config.stop_yaw_rate_tolerance;
}

static void bc_drive_transition(
    bc_drive_t *drive,
    const bc_state_machine_input_t *input,
    bc_state_vector_t *state_reference
) {
    switch (drive->state) {
    case BC_DRIVE_IDLE:
        break;

    case BC_DRIVE_PARKED:
        if (bc_drive_motion_requested(drive, input)) {
            drive->state = BC_DRIVE_DRIVING;
            bc_condition_hold_reset(&drive->stopped_hold);
        }
        break;

    case BC_DRIVE_DRIVING:
        if (!bc_condition_hold_update(
                &drive->stopped_hold,
                bc_drive_is_stopped(drive, input),
                drive->config.stop_duration,
                input->timestep_seconds)) break;

        drive->state = BC_DRIVE_PARKED;
        bc_drive_capture_position(input, state_reference);
        break;
    }
}

static void bc_drive_action(
    bc_drive_t *drive,
    const bc_state_machine_input_t *input,
    bc_state_vector_t *state_reference,
    bc_control_command_t *output
) {
    switch (drive->state) {
    case BC_DRIVE_IDLE:
        break;

    case BC_DRIVE_PARKED:
        output->state_reference = *state_reference;
        break;

    case BC_DRIVE_DRIVING: {
        const float forward_velocity = bc_reference_ramp_update(
            &drive->forward_velocity_ramp,
            &drive->config.forward_velocity_ramp,
            bc_drive_forward_target(drive, input),
            input->timestep_seconds);
        const float yaw_rate = bc_reference_ramp_update(
            &drive->yaw_rate_ramp,
            &drive->config.yaw_rate_ramp,
            bc_drive_yaw_target(drive, input),
            input->timestep_seconds);
        state_reference->value[BC_STATE_S] +=
            forward_velocity * input->timestep_seconds;
        state_reference->value[BC_STATE_DS] = forward_velocity;
        state_reference->value[BC_STATE_PSI] +=
            yaw_rate * input->timestep_seconds;
        state_reference->value[BC_STATE_DPSI] = yaw_rate;
        output->disabled_state_feedback |=
            BC_STATE_FEEDBACK_MASK(BC_STATE_S);
        output->state_reference = *state_reference;
        break;
    }
    }
}

void bc_drive_default_config(bc_drive_config_t *config) {
    *config = (bc_drive_config_t){
        .forward_command_deadband         = 0.01F,
        .yaw_command_deadband             = 0.02F,
        .stop_wheel_velocity_tolerance    = 0.05F,
        .stop_forward_velocity_tolerance  = 0.05F,
        .stop_yaw_rate_tolerance          = 0.10F,
        .stop_duration                    = 0.25F,
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

void bc_drive_init(
    bc_drive_t *drive,
    const bc_drive_config_t *config
) {
    drive->config = *config;
    bc_drive_reset(drive);
}

void bc_drive_reset(bc_drive_t *drive) {
    drive->state = BC_DRIVE_IDLE;
    bc_reference_ramp_reset(&drive->forward_velocity_ramp);
    bc_reference_ramp_reset(&drive->yaw_rate_ramp);
    bc_condition_hold_reset(&drive->stopped_hold);
}

void bc_drive_start(
    bc_drive_t *drive,
    const bc_state_machine_input_t *input,
    bc_state_vector_t *state_reference
) {
    bc_drive_reset(drive);
    drive->state = BC_DRIVE_PARKED;
    bc_drive_capture_position(input, state_reference);
}

void bc_drive_update(
    bc_drive_t *drive,
    const bc_state_machine_input_t *input,
    bc_state_vector_t *state_reference,
    bc_control_command_t *output
) {
    bc_drive_transition(drive, input, state_reference);
    bc_drive_action(drive, input, state_reference, output);
}

const char *bc_drive_state_name(const bc_drive_state_t state) {
    switch (state) {
    case BC_DRIVE_IDLE:
        return "idle";
    case BC_DRIVE_PARKED:
        return "parked";
    case BC_DRIVE_DRIVING:
        return "driving";
    }
    return "unknown";
}
