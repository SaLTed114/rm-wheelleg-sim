#ifndef BALANCE_STATE_MACHINE_FORWARD_MODE_H
#define BALANCE_STATE_MACHINE_FORWARD_MODE_H

#include "balance/state_machine/condition_hold.h"
#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BC_FORWARD_IDLE,
    BC_FORWARD_HOLD,
    BC_FORWARD_VELOCITY
} bc_forward_state_t;

typedef struct {
    float stop_forward_velocity_tolerance;
    float stop_wheel_velocity_tolerance;
    float stop_duration;
} bc_forward_mode_config_t;

typedef struct {
    bc_forward_mode_config_t config;
    bc_forward_state_t state;
    bc_condition_hold_t stopped_hold;
} bc_forward_mode_t;

void bc_forward_mode_default_config(bc_forward_mode_config_t *config);
void bc_forward_mode_init(
    bc_forward_mode_t *forward,
    const bc_forward_mode_config_t *config);
void bc_forward_mode_reset(bc_forward_mode_t *forward);
void bc_forward_mode_start(bc_forward_mode_t *forward);

void bc_forward_mode_update(
    bc_forward_mode_t *forward,
    uint8_t forward_motion_requested,
    float reference_velocity,
    float measured_velocity,
    float wheel_velocity,
    uint8_t wheel_velocity_reliable,
    float timestep_seconds,
    bc_control_command_t *output);

const char *bc_forward_state_name(bc_forward_state_t state);

#ifdef __cplusplus
}
#endif

#endif
