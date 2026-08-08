#ifndef BALANCE_FORWARD_REFERENCE_H
#define BALANCE_FORWARD_REFERENCE_H

#include "balance/reference/ramp.h"
#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float command_deadband;
    bc_reference_ramp_config_t velocity_ramp;
} bc_forward_reference_config_t;

typedef struct {
    bc_forward_reference_config_t config;
    bc_reference_ramp_t velocity_ramp;
} bc_forward_reference_t;

void bc_forward_reference_default_config(
    bc_forward_reference_config_t *config);
void bc_forward_reference_init(
    bc_forward_reference_t *forward,
    const bc_forward_reference_config_t *config);
void bc_forward_reference_reset(bc_forward_reference_t *forward);
void bc_forward_reference_start(
    bc_forward_reference_t *forward,
    float current_position,
    bc_state_vector_t *reference);
uint8_t bc_forward_reference_requested(
    const bc_forward_reference_t *forward,
    float target_velocity);
void bc_forward_reference_update(
    bc_forward_reference_t *forward,
    float target_velocity,
    float timestep_seconds,
    bc_state_vector_t *reference);

#ifdef __cplusplus
}
#endif

#endif
