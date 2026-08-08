#include "balance/controller.h"

#include <string.h>

void bc_controller_default_config(bc_controller_config_t *config) {
    bc_control_default_config(&config->control);
    bc_motion_default_config(&config->motion);
    config->velocity_estimator_update_delay = 0.5F;
}

void bc_controller_init(
    bc_controller_t *controller,
    const bc_controller_config_t *config
) {
    bc_control_core_init(&controller->control_core, &config->control);
    bc_system_init(&controller->system, &config->motion);
    controller->velocity_estimator_update_delay =
        config->velocity_estimator_update_delay;
    bc_controller_reset(controller);
}

void bc_controller_reset(bc_controller_t *controller) {
    bc_control_core_reset(&controller->control_core);
    bc_system_reset(&controller->system);
    memset(
        &controller->operator_command, 0,
        sizeof(controller->operator_command));
    memset(
        &controller->gimbal_feedback, 0,
        sizeof(controller->gimbal_feedback));
    memset(
        &controller->last_actuation, 0,
        sizeof(controller->last_actuation));
    bc_condition_hold_reset(&controller->velocity_estimator_hold);
    controller->timestep_seconds = 0.0F;
}

void bc_controller_update(
    bc_controller_t *controller,
    const bc_sensor_feedback_t *feedback,
    const float timestep_seconds
) {
    controller->timestep_seconds = timestep_seconds;
    controller->gimbal_feedback = feedback->gimbal;
    const uint8_t balance_motion =
        controller->system.motion.state == BC_MOTION_BALANCE_ENGAGING ||
        controller->system.motion.state == BC_MOTION_ACTIVE;
    const uint8_t wheel_velocity_update_enabled =
        bc_condition_hold_update(
            &controller->velocity_estimator_hold,
            controller->system.state == BC_SYSTEM_ON && balance_motion,
            controller->velocity_estimator_update_delay,
            timestep_seconds);
    bc_control_core_update(
        &controller->control_core, feedback, timestep_seconds,
        wheel_velocity_update_enabled);
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
        .gimbal_feedback = &controller->gimbal_feedback,
        .state = &controller->control_core.observer.state,
        .leg = controller->control_core.observer.leg,
        .timestep_seconds = controller->timestep_seconds,
    };
    bc_system_update(&controller->system, &input, &control_command);
    bc_control_core_calculate(&controller->control_core, &control_command);
}

void bc_controller_execute(
    bc_controller_t *controller,
    bc_actuation_t *actuation
) {
    bc_control_core_execute(
        &controller->control_core,
        controller->system.state == BC_SYSTEM_ON,
        actuation);
    controller->last_actuation = *actuation;
}
