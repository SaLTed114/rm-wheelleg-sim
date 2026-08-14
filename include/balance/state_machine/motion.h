#ifndef BALANCE_STATE_MACHINE_MOTION_H
#define BALANCE_STATE_MACHINE_MOTION_H

#include "balance/reference/forward.h"
#include "balance/reference/ramp.h"
#include "balance/reference/yaw.h"
#include "balance/state_machine/condition_hold.h"
#include "balance/state_machine/forward_mode.h"
#include "balance/state_machine/input.h"
#include "balance/state_machine/step_task.h"
#include "balance/state_machine/support_phase.h"

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

typedef enum {
    BC_CHASSIS_FRONT,
    BC_CHASSIS_REAR
} bc_chassis_alignment_t;

typedef struct {
    float startup_leg_length;
    float leg_length; /* ACTIVE working target. */
    bc_reference_ramp_config_t leg_length_ramp;
    float leg_angle_body;
    float length_tolerance;
    float length_velocity_tolerance;
    float angle_tolerance;
    float angular_velocity_tolerance;
    float stable_duration;
    float engage_duration;
    float alignment_hysteresis;
    bc_forward_mode_config_t forward;
    bc_forward_reference_config_t forward_reference;
    bc_yaw_reference_config_t yaw_reference;
    bc_step_task_config_t step_task;
    bc_support_phase_config_t support_phase;
} bc_motion_config_t;

typedef struct {
    bc_motion_config_t config;
    bc_motion_state_t state;
    bc_reference_ramp_t leg_length_reference;
    bc_state_vector_t state_reference;
    bc_chassis_alignment_t alignment;
    float mapped_forward_velocity;
    float heading_error;
    bc_forward_mode_t forward;
    bc_forward_reference_t forward_reference;
    bc_yaw_reference_t yaw_reference;
    bc_step_task_t step_task;
    bc_support_phase_t support_phase;
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
const char *bc_chassis_alignment_name(bc_chassis_alignment_t alignment);

#ifdef __cplusplus
}
#endif

#endif
