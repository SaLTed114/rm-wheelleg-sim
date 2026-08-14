#include "balance/controller_snapshot.h"

#include <string.h>

void bc_controller_capture_snapshot(
    const bc_controller_t *controller,
    bc_controller_snapshot_t *snapshot
) {
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->state_machine.system = controller->system.state;
    snapshot->state_machine.motion = controller->system.motion.state;
    snapshot->state_machine.forward =
        controller->system.motion.forward.state;
    snapshot->state_machine.support =
        controller->system.motion.support_phase.state;
    snapshot->state_machine.alignment =
        controller->system.motion.alignment;
    snapshot->state = controller->control_core.observer.state;
    snapshot->state_reference = controller->system.motion.state_reference;
    snapshot->yaw_acceleration_reference =
        controller->system.motion.yaw_reference.acceleration_reference;
    snapshot->roll = controller->control_core.observer.roll;
    snapshot->roll_rate = controller->control_core.observer.roll_rate;
    snapshot->roll_force_request =
        controller->control_core.roll_force_request;
    snapshot->specific_force_norm = controller->specific_force_norm;
    snapshot->gimbal = controller->gimbal_feedback;
    snapshot->mapped_forward_velocity =
        controller->system.motion.mapped_forward_velocity;
    snapshot->heading_error = controller->system.motion.heading_error;
    snapshot->forward_velocity =
        controller->control_core.observer.forward_velocity;
    snapshot->impact_observer =
        controller->control_core.observer.impact_observer.output;
    snapshot->velocity_estimator =
        controller->control_core.observer.velocity_estimator.output;
    memcpy(
        snapshot->leg,
        controller->control_core.observer.leg,
        sizeof(snapshot->leg));
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        snapshot->support_force[side] =
            controller->control_core.support_force[side].output;
    }
    snapshot->support_request =
        controller->system.motion.support_phase.request;
    snapshot->actuation_request =
        controller->control_core.actuation_request;
    snapshot->actuation = controller->last_actuation;
    snapshot->tick_count = controller->control_core.tick_count;
}
