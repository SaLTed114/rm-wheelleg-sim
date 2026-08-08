#include "balance/state_machine/drive.h"

#include <math.h>

static uint8_t bc_drive_is_stopped(
    const bc_drive_t *drive,
    const uint8_t forward_motion_requested,
    const float reference_velocity,
    const float measured_velocity
) {
    return !forward_motion_requested &&
        reference_velocity == 0.0F &&
        fabsf(measured_velocity) <
            drive->config.stop_forward_velocity_tolerance;
}

static void bc_drive_transition(
    bc_drive_t *drive,
    const uint8_t forward_motion_requested,
    const float reference_velocity,
    const float measured_velocity,
    const float timestep_seconds
) {
    switch (drive->state) {
    case BC_DRIVE_IDLE:
        break;

    case BC_DRIVE_HOLD:
        if (forward_motion_requested) {
            drive->state = BC_DRIVE_DRIVING;
            bc_condition_hold_reset(&drive->stopped_hold);
        }
        break;

    case BC_DRIVE_DRIVING:
        if (!bc_condition_hold_update(
                &drive->stopped_hold,
                bc_drive_is_stopped(
                    drive, forward_motion_requested,
                    reference_velocity, measured_velocity),
                drive->config.stop_duration,
                timestep_seconds)) break;

        drive->state = BC_DRIVE_HOLD;
        break;

    case BC_DRIVE_SPIN:
        bc_condition_hold_reset(&drive->stopped_hold);
        drive->state = BC_DRIVE_HOLD;
        break;
    }
}

static void bc_drive_action(
    const bc_drive_t *drive,
    bc_control_command_t *output
) {
    switch (drive->state) {
    case BC_DRIVE_IDLE:
    case BC_DRIVE_HOLD:
    case BC_DRIVE_SPIN:
        break;

    case BC_DRIVE_DRIVING:
        output->disabled_state_feedback |=
            BC_STATE_FEEDBACK_MASK(BC_STATE_S);
        break;
    }
}

void bc_drive_default_config(bc_drive_config_t *config) {
    *config = (bc_drive_config_t){
        .stop_forward_velocity_tolerance = 0.05F,
        .stop_duration = 0.25F,
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
    bc_condition_hold_reset(&drive->stopped_hold);
}

void bc_drive_start(bc_drive_t *drive) {
    bc_drive_reset(drive);
    drive->state = BC_DRIVE_HOLD;
}

void bc_drive_update(
    bc_drive_t *drive,
    const uint8_t forward_motion_requested,
    const float reference_velocity,
    const float measured_velocity,
    const float timestep_seconds,
    bc_control_command_t *output
) {
    bc_drive_transition(
        drive, forward_motion_requested,
        reference_velocity, measured_velocity,
        timestep_seconds);
    bc_drive_action(drive, output);
}

const char *bc_drive_state_name(const bc_drive_state_t state) {
    switch (state) {
    case BC_DRIVE_IDLE:
        return "idle";
    case BC_DRIVE_HOLD:
        return "hold";
    case BC_DRIVE_DRIVING:
        return "drive";
    case BC_DRIVE_SPIN:
        return "spin";
    }
    return "unknown";
}
