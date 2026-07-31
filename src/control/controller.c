#include "balance/controller.h"

#include <string.h>

static void bc_controller_update_observer_status(
    bc_controller_t *controller
) {
    controller->status.state = controller->control_core.observer.state;
    memcpy(
        controller->status.leg,
        controller->control_core.observer.leg,
        sizeof(controller->status.leg));
}

void bc_controller_default_config(bc_controller_config_t *config) {
    bc_control_default_config(&config->control);
    bc_motion_default_config(&config->motion);
}

void bc_controller_init(
    bc_controller_t *controller,
    const bc_controller_config_t *config
) {
    bc_control_core_init(&controller->control_core, &config->control);
    bc_system_init(&controller->system, &config->motion);
    bc_controller_reset(controller);
}

void bc_controller_reset(bc_controller_t *controller) {
    bc_control_core_reset(&controller->control_core);
    bc_system_reset(&controller->system);
    memset(
        &controller->operator_command, 0,
        sizeof(controller->operator_command));
    memset(&controller->status, 0, sizeof(controller->status));
    controller->status.system_state = controller->system.state;
    controller->status.motion_state = controller->system.motion.state;
    controller->timestep_seconds = 0.0F;
    bc_controller_update_observer_status(controller);
}

void bc_controller_update(
    bc_controller_t *controller,
    const bc_sensor_feedback_t *feedback,
    const float timestep_seconds
) {
    controller->timestep_seconds = timestep_seconds;
    bc_control_core_update(
        &controller->control_core, feedback, timestep_seconds);
    bc_controller_update_observer_status(controller);
}

void bc_controller_set_command(
    bc_controller_t *controller,
    const bc_operator_command_t *command
) {
    controller->operator_command = *command;
}

void bc_controller_calculate(bc_controller_t *controller) {
    bc_control_command_t control_command;

    const bc_state_machine_input_t input = {
        .operator_command = &controller->operator_command,
        .state = &controller->control_core.observer.state,
        .leg = controller->control_core.observer.leg,
        .timestep_seconds = controller->timestep_seconds,
    };
    bc_system_update(&controller->system, &input, &control_command);
    bc_control_core_calculate(&controller->control_core, &control_command);
    controller->status.system_state = controller->system.state;
    controller->status.motion_state = controller->system.motion.state;
    controller->status.tick_count = controller->control_core.tick_count;
}

void bc_controller_execute(
    bc_controller_t *controller,
    bc_actuation_t *actuation
) {
    bc_control_core_execute(
        &controller->control_core,
        controller->system.state == BC_SYSTEM_ON,
        actuation);
    controller->status.actuation = *actuation;
}

const bc_controller_status_t *bc_controller_get_status(
    const bc_controller_t *controller
) {
    return &controller->status;
}
