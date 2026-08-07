#include "balance/controller_snapshot.h"

#include <string.h>

void bc_controller_capture_snapshot(
    const bc_controller_t *controller,
    bc_controller_snapshot_t *snapshot
) {
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->state_machine.system = controller->system.state;
    snapshot->state_machine.motion = controller->system.motion.state;
    snapshot->state_machine.drive = controller->system.motion.drive.state;
    snapshot->state = controller->control_core.observer.state;
    snapshot->state_reference = controller->system.motion.state_reference;
    snapshot->roll = controller->control_core.observer.roll;
    snapshot->roll_rate = controller->control_core.observer.roll_rate;
    snapshot->forward_velocity =
        controller->control_core.observer.forward_velocity;
    snapshot->velocity_estimator =
        controller->control_core.observer.velocity_estimator.output;
    memcpy(
        snapshot->leg,
        controller->control_core.observer.leg,
        sizeof(snapshot->leg));
    snapshot->actuation_request =
        controller->control_core.actuation_request;
    snapshot->actuation = controller->last_actuation;
    snapshot->tick_count = controller->control_core.tick_count;
}
