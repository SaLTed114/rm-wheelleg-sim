#include "balance/controller_snapshot.h"

#include <string.h>

void bc_controller_capture_snapshot(
    const bc_controller_t *controller,
    bc_controller_snapshot_t *snapshot
) {
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->state_machine.system = controller->system.state;
    snapshot->state_machine.motion = controller->system.motion.state;
    snapshot->state = controller->control_core.observer.state;
    memcpy(
        snapshot->leg,
        controller->control_core.observer.leg,
        sizeof(snapshot->leg));
    snapshot->actuation = controller->last_actuation;
    snapshot->tick_count = controller->control_core.tick_count;
}
