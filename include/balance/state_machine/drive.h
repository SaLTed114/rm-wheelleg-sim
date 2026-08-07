#ifndef BALANCE_STATE_MACHINE_DRIVE_H
#define BALANCE_STATE_MACHINE_DRIVE_H

#include "balance/reference_ramp.h"
#include "balance/state_machine/condition_hold.h"
#include "balance/state_machine/input.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BC_DRIVE_IDLE,
    BC_DRIVE_PARKED,
    BC_DRIVE_DRIVING
} bc_drive_state_t;

typedef struct {
    float forward_command_deadband;
    float yaw_command_deadband;
    float stop_wheel_velocity_tolerance;
    float stop_forward_velocity_tolerance;
    float stop_yaw_rate_tolerance;
    float stop_duration;
    bc_reference_ramp_config_t forward_velocity_ramp;
    bc_reference_ramp_config_t yaw_rate_ramp;
} bc_drive_config_t;

typedef struct {
    bc_drive_config_t config;
    bc_drive_state_t state;
    bc_reference_ramp_t forward_velocity_ramp;
    bc_reference_ramp_t yaw_rate_ramp;
    bc_condition_hold_t stopped_hold;
} bc_drive_t;

void bc_drive_default_config(bc_drive_config_t *config);
void bc_drive_init(
    bc_drive_t *drive,
    const bc_drive_config_t *config);
void bc_drive_reset(bc_drive_t *drive);
void bc_drive_start(
    bc_drive_t *drive,
    const bc_state_machine_input_t *input,
    bc_state_vector_t *state_reference);

void bc_drive_update(
    bc_drive_t *drive,
    const bc_state_machine_input_t *input,
    bc_state_vector_t *state_reference,
    bc_control_command_t *output);

const char *bc_drive_state_name(bc_drive_state_t state);

#ifdef __cplusplus
}
#endif

#endif
