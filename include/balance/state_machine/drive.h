#ifndef BALANCE_STATE_MACHINE_DRIVE_H
#define BALANCE_STATE_MACHINE_DRIVE_H

#include "balance/state_machine/condition_hold.h"
#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BC_DRIVE_IDLE,
    BC_DRIVE_HOLD,
    BC_DRIVE_DRIVING,
    BC_DRIVE_SPIN
} bc_drive_state_t;

typedef struct {
    float stop_forward_velocity_tolerance;
    float stop_duration;
} bc_drive_config_t;

typedef struct {
    bc_drive_config_t config;
    bc_drive_state_t state;
    bc_condition_hold_t stopped_hold;
} bc_drive_t;

void bc_drive_default_config(bc_drive_config_t *config);
void bc_drive_init(
    bc_drive_t *drive,
    const bc_drive_config_t *config);
void bc_drive_reset(bc_drive_t *drive);
void bc_drive_start(bc_drive_t *drive);

void bc_drive_update(
    bc_drive_t *drive,
    uint8_t forward_motion_requested,
    float reference_velocity,
    float measured_velocity,
    float timestep_seconds,
    bc_control_command_t *output);

const char *bc_drive_state_name(bc_drive_state_t state);

#ifdef __cplusplus
}
#endif

#endif
