#ifndef BALANCE_STATE_MACHINE_INPUT_H
#define BALANCE_STATE_MACHINE_INPUT_H

#include "balance/leg_kinematics.h"
#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const bc_operator_command_t *operator_command;
    const bc_state_vector_t *state;
    const bc_leg_kinematics_t *leg;
    float timestep_seconds;
} bc_state_machine_input_t;

#ifdef __cplusplus
}
#endif

#endif
