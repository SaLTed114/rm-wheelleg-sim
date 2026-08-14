#ifndef BALANCE_STATE_MACHINE_STEP_TASK_H
#define BALANCE_STATE_MACHINE_STEP_TASK_H

#include "balance/state_machine/condition_hold.h"
#include "balance/state_machine/input.h"
#include "balance/state_machine/support_phase.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BC_STEP_TASK_INACTIVE,
    BC_STEP_TASK_PREPARE,
    BC_STEP_TASK_IMPACT_PASSIVE
} bc_step_task_state_t;

typedef struct {
    float prepare_leg_length;
    float leg_length_tolerance;
    float alignment_tolerance;
    float minimum_forward_velocity;
    float leg_rate_delta_threshold;
    float wheel_imu_mismatch_threshold;
    float impact_confirm_duration;
} bc_step_task_config_t;

typedef struct {
    bc_step_task_config_t config;
    bc_step_task_state_t state;
    bc_condition_hold_t impact_hold;
    uint8_t impact_armed;
} bc_step_task_t;

void bc_step_task_default_config(bc_step_task_config_t *config);
void bc_step_task_init(
    bc_step_task_t *task,
    const bc_step_task_config_t *config);
void bc_step_task_reset(bc_step_task_t *task);

void bc_step_task_update(
    bc_step_task_t *task,
    const bc_state_machine_input_t *input,
    bc_support_phase_state_t support_state,
    float heading_error);

const char *bc_step_task_state_name(bc_step_task_state_t state);

#ifdef __cplusplus
}
#endif

#endif
