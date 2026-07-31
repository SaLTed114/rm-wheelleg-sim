#include "balance/state_machine/system.h"

#include <string.h>

static void bc_system_transition(
    bc_system_t *system,
    const bc_state_machine_input_t *input
) {
    switch (system->state) {
    case BC_SYSTEM_OFF:
        if (input->operator_command->system_enabled) {
            system->state = BC_SYSTEM_ON;
        }
        break;

    case BC_SYSTEM_ON:
        if (!input->operator_command->system_enabled) {
            system->state = BC_SYSTEM_OFF;
        }
        break;
    }
}

static void bc_system_action(
    bc_system_t *system,
    const bc_state_machine_input_t *input,
    bc_control_command_t *output
) {
    switch (system->state) {
    case BC_SYSTEM_OFF:
        bc_motion_reset(&system->motion);
        break;

    case BC_SYSTEM_ON:
        if (input->operator_command->balance_restart) {
            bc_motion_start(&system->motion);
        }
        bc_motion_update(&system->motion, input, output);
        break;
    }
}

void bc_system_init(
    bc_system_t *system,
    const bc_motion_config_t *motion_config
) {
    bc_motion_init(&system->motion, motion_config);
    bc_system_reset(system);
}

void bc_system_reset(bc_system_t *system) {
    system->state = BC_SYSTEM_OFF;
    bc_motion_reset(&system->motion);
}

void bc_system_update(
    bc_system_t *system,
    const bc_state_machine_input_t *input,
    bc_control_command_t *output
) {
    memset(output, 0, sizeof(*output));

    bc_system_transition(system, input);
    bc_system_action(system, input, output);
}

const char *bc_system_state_name(const bc_system_state_t state) {
    switch (state) {
    case BC_SYSTEM_OFF:
        return "off";
    case BC_SYSTEM_ON:
        return "on";
    }
    return "unknown";
}
