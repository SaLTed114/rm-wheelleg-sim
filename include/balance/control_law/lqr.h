#ifndef BALANCE_CONTROL_LAW_LQR_H
#define BALANCE_CONTROL_LAW_LQR_H

#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float wheel_torque[BC_SIDE_NUM];
    float leg_torque[BC_SIDE_NUM];
} bc_lqr_output_t;

void bc_lqr_calculate(
    float leg_length,
    const bc_state_vector_t *state_error,
    float yaw_acceleration_reference,
    bc_lqr_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
