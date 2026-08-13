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

    case BC_SYSTEM_FAULT:
        system->state = BC_SYSTEM_OFF;
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
    case BC_SYSTEM_FAULT:
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

bc_observation_context_t bc_system_observation_context(
    const bc_system_t *system
) {
    bc_observation_context_t context = {
        .wheel_velocity = BC_WHEEL_OBSERVATION_DISABLED,
    };
    if (system->state != BC_SYSTEM_ON ||
        (system->motion.state != BC_MOTION_BALANCE_ENGAGING &&
         system->motion.state != BC_MOTION_ACTIVE)) {
        return context;
    }
    if (system->motion.state == BC_MOTION_BALANCE_ENGAGING) {
        context.wheel_velocity = BC_WHEEL_OBSERVATION_GROUND;
        return context;
    }

    switch (system->motion.support_phase.state) {
    case BC_SUPPORT_GROUND:
        context.wheel_velocity = BC_WHEEL_OBSERVATION_GROUND;
        break;
    case BC_SUPPORT_AIRBORNE:
        context.wheel_velocity = BC_WHEEL_OBSERVATION_AIRBORNE;
        break;
    case BC_SUPPORT_LANDING_RETRACT:
    case BC_SUPPORT_GROUND_RECOVER:
        context.wheel_velocity = BC_WHEEL_OBSERVATION_CONTACT_TRANSIENT;
        break;
    }
    return context;
}

const char *bc_system_state_name(const bc_system_state_t state) {
    switch (state) {
    case BC_SYSTEM_OFF:
        return "off";
    case BC_SYSTEM_ON:
        return "on";
    case BC_SYSTEM_FAULT:
        return "fault";
    }
    return "unknown";
}
