#include "balance/control_core.h"

#include <string.h>

void bc_control_core_init(bc_control_core_t *core) {
    bc_control_core_reset(core);
}

void bc_control_core_reset(bc_control_core_t *core) {
    core->tick_count = 0U;
}

void bc_control_core_step(
    bc_control_core_t *core,
    const bc_observation_t *observation,
    const bc_operator_command_t *command,
    bc_actuation_t *actuation
) {
    (void)observation;
    (void)command;

    memset(actuation, 0, sizeof(*actuation));
    core->tick_count += 1U;
}
