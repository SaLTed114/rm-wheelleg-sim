#ifndef BALANCE_LQR_CONTROLLER_H
#define BALANCE_LQR_CONTROLLER_H

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
    const bc_state_vector_t *state,
    const bc_state_vector_t *reference,
    bc_lqr_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
