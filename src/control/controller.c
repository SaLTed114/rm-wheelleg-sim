#include "balance/controller.h"

#include <math.h>
#include <string.h>

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
    memset(
        &controller->gimbal_feedback, 0,
        sizeof(controller->gimbal_feedback));
    memset(
        &controller->last_actuation, 0,
        sizeof(controller->last_actuation));
    controller->specific_force_norm = 0.0F;
    controller->timestep_seconds = 0.0F;
}

void bc_controller_update(
    bc_controller_t *controller,
    const bc_sensor_feedback_t *feedback,
    const float timestep_seconds
) {
    controller->timestep_seconds = timestep_seconds;
    controller->gimbal_feedback = feedback->gimbal;
    controller->specific_force_norm = sqrtf(
        feedback->imu.specific_force_x * feedback->imu.specific_force_x +
        feedback->imu.specific_force_y * feedback->imu.specific_force_y +
        feedback->imu.specific_force_z * feedback->imu.specific_force_z);
    const bc_observation_context_t observation_context =
        bc_system_observation_context(&controller->system);
    bc_control_core_update(
        &controller->control_core, feedback, &observation_context,
        timestep_seconds);
}

void bc_controller_set_command(
    bc_controller_t *controller,
    const bc_operator_command_t *command
) {
    controller->operator_command = *command;
}

void bc_controller_calculate(bc_controller_t *controller) {
    bc_control_command_t control_command;
    bc_support_force_output_t support_force[BC_SIDE_NUM];
    float nominal_axial_force[BC_SIDE_NUM];
    const float roll_force = bc_control_core_roll_force(
        &controller->control_core);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        support_force[side] =
            controller->control_core.support_force[side].output;
        nominal_axial_force[side] =
            controller->control_core.config.support_force +
            controller->control_core.config.roll_force_sign[side] *
                roll_force;
    }

    const bc_state_machine_input_t input = {
        .operator_command = &controller->operator_command,
        .gimbal_feedback = &controller->gimbal_feedback,
        .state = &controller->control_core.observer.state,
        .leg = controller->control_core.observer.leg,
        .support_force = support_force,
        .impact_observer =
            &controller->control_core.observer.impact_observer.output,
        .nominal_axial_force = {
            nominal_axial_force[BC_L], nominal_axial_force[BC_R],
        },
        .length_position_kp =
            controller->control_core.config.length_controller.kp,
        .length_position_kd =
            controller->control_core.config.length_controller.kd,
        .specific_force_norm = controller->specific_force_norm,
        .wheel_odometry_velocity =
            controller->control_core.observer.forward_velocity.
                wheel_odometry,
        .wheel_velocity_reliable =
            controller->control_core.observer.velocity_estimator.output.
                wheel_velocity_reliable,
        .timestep_seconds = controller->timestep_seconds,
    };
    bc_system_update(&controller->system, &input, &control_command);
    if (controller->system.motion.support_phase.state ==
        BC_SUPPORT_AIRBORNE) {
        bc_control_core_reject_wheel_velocity(&controller->control_core);
    }
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
