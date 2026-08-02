#ifndef BALANCE_STATE_MACHINE_SYSTEM_H
#define BALANCE_STATE_MACHINE_SYSTEM_H

#include "balance/state_machine/motion.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BC_SYSTEM_OFF,
    BC_SYSTEM_ON,
    BC_SYSTEM_FAULT
} bc_system_state_t;

typedef struct {
    bc_system_state_t state;
    bc_motion_t motion;
} bc_system_t;

void bc_system_init(
    bc_system_t *system,
    const bc_motion_config_t *motion_config);
void bc_system_reset(bc_system_t *system);

void bc_system_update(
    bc_system_t *system,
    const bc_state_machine_input_t *input,
    bc_control_command_t *output);

const char *bc_system_state_name(bc_system_state_t state);

#ifdef __cplusplus
}
#endif

#endif
