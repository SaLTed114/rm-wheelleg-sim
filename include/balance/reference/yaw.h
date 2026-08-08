#ifndef BALANCE_YAW_REFERENCE_H
#define BALANCE_YAW_REFERENCE_H

#include "balance/reference/ramp.h"
#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float command_deadband;
    bc_reference_ramp_config_t rate_ramp;
} bc_yaw_reference_config_t;

typedef struct {
    bc_yaw_reference_config_t config;
    bc_reference_ramp_t rate_ramp;
} bc_yaw_reference_t;

void bc_yaw_reference_default_config(
    bc_yaw_reference_config_t *config);
void bc_yaw_reference_init(
    bc_yaw_reference_t *yaw,
    const bc_yaw_reference_config_t *config);
void bc_yaw_reference_reset(bc_yaw_reference_t *yaw);
void bc_yaw_reference_start(
    bc_yaw_reference_t *yaw,
    float current_yaw,
    bc_state_vector_t *reference);
void bc_yaw_reference_update(
    bc_yaw_reference_t *yaw,
    float target_rate,
    float timestep_seconds,
    bc_state_vector_t *reference);

#ifdef __cplusplus
}
#endif

#endif
