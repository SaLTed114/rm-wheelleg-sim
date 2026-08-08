#ifndef BALANCE_YAW_REFERENCE_H
#define BALANCE_YAW_REFERENCE_H

#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float rate_limit;
} bc_yaw_reference_config_t;

typedef struct {
    bc_yaw_reference_config_t config;
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
    float current_yaw_rate,
    bc_state_vector_t *reference);
void bc_yaw_reference_update(
    const bc_yaw_reference_t *yaw,
    const bc_state_vector_t *state,
    float heading_error,
    float relative_yaw_rate,
    bc_state_vector_t *reference);

#ifdef __cplusplus
}
#endif

#endif
