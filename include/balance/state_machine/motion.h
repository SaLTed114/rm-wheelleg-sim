#ifndef BALANCE_STATE_MACHINE_MOTION_H
#define BALANCE_STATE_MACHINE_MOTION_H

#include "balance/reference/forward.h"
#include "balance/state_machine/condition_hold.h"
#include "balance/state_machine/drive.h"
#include "balance/state_machine/input.h"
#include "balance/reference/yaw.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BC_MOTION_IDLE,
    BC_MOTION_SELF_RIGHTING,
    BC_MOTION_LEG_POSITIONING,
    BC_MOTION_BALANCE_ENGAGING,
    BC_MOTION_ACTIVE
} bc_motion_state_t;

typedef struct {
    float leg_length;
    float leg_angle_body;
    float length_tolerance;
    float length_velocity_tolerance;
    float angle_tolerance;
    float angular_velocity_tolerance;
    float stable_duration;
    float engage_duration;
    bc_drive_config_t drive;
    bc_forward_reference_config_t forward_reference;
    bc_yaw_reference_config_t yaw_reference;
} bc_motion_config_t;

typedef struct {
    bc_motion_config_t config;
    bc_motion_state_t state;
    bc_state_vector_t state_reference;
    bc_drive_t drive;
    bc_forward_reference_t forward_reference;
    bc_yaw_reference_t yaw_reference;
    bc_condition_hold_t leg_stable_hold;
    bc_condition_hold_t engage_hold;
} bc_motion_t;

void bc_motion_default_config(bc_motion_config_t *config);
void bc_motion_init(
    bc_motion_t *motion,
    const bc_motion_config_t *config);
void bc_motion_reset(bc_motion_t *motion);
void bc_motion_start(bc_motion_t *motion);

void bc_motion_update(
    bc_motion_t *motion,
    const bc_state_machine_input_t *input,
    bc_control_command_t *output);

const char *bc_motion_state_name(bc_motion_state_t state);

#ifdef __cplusplus
}
#endif

#endif
