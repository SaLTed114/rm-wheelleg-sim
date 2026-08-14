#include "balance/controller_snapshot.h"

#include <math.h>
#include <stdio.h>

static int actuation_is_zero(const bc_actuation_t *actuation) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (actuation->wheel_torque[side] != 0.0F) return 0;
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            if (actuation->leg[side].joint_torque[joint] != 0.0F) return 0;
        }
    }
    return 1;
}

int main() {
    bc_controller_config_t config;
    bc_controller_t controller;
    bc_controller_snapshot_t snapshot;
    bc_sensor_feedback_t feedback = {0};
    bc_operator_command_t command = {0};
    bc_actuation_t actuation;

    bc_controller_default_config(&config);
    config.motion.stable_duration = 0.002F;
    config.motion.engage_duration = 0.002F;
    config.control.observer.wheel_velocity_startup_delay = 0.002F;
    bc_controller_init(&controller, &config);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.system != BC_SYSTEM_OFF ||
        snapshot.state_machine.motion != BC_MOTION_IDLE ||
        snapshot.state_machine.forward != BC_FORWARD_IDLE ||
        snapshot.state_machine.alignment != BC_CHASSIS_FRONT ||
        snapshot.tick_count != 0U ||
        snapshot.roll_force_request != 0.0F ||
        !actuation_is_zero(&snapshot.actuation_request) ||
        !actuation_is_zero(&snapshot.actuation)) {
        fputs("reset controller snapshot is incorrect\n", stderr);
        return 1;
    }

    feedback.imu.pitch = 0.1F;
    feedback.imu.roll = 0.25F;
    feedback.imu.roll_rate = -0.4F;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.system != BC_SYSTEM_OFF ||
        snapshot.state_machine.motion != BC_MOTION_IDLE ||
        snapshot.state.value[BC_STATE_THETA_B] != 0.1F ||
        snapshot.roll != 0.25F ||
        snapshot.roll_rate != -0.4F ||
        snapshot.tick_count != 1U ||
        !actuation_is_zero(&snapshot.actuation)) {
        fputs("disabled controller snapshot or output is incorrect\n", stderr);
        return 1;
    }
    const bc_controller_snapshot_t disabled_snapshot = snapshot;

    command.system_enabled = 1U;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_set_command(&controller, &command);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.system != BC_SYSTEM_OFF ||
        snapshot.state_machine.motion != BC_MOTION_IDLE ||
        snapshot.tick_count != 1U) {
        fputs("set_command changed controller state\n", stderr);
        return 1;
    }
    bc_controller_calculate(&controller);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.system != BC_SYSTEM_ON ||
        snapshot.state_machine.motion != BC_MOTION_IDLE ||
        disabled_snapshot.state_machine.system != BC_SYSTEM_OFF ||
        disabled_snapshot.tick_count != 1U) {
        fputs("system enable or snapshot ownership is incorrect\n", stderr);
        return 1;
    }

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        controller.control_core.actuation_request.wheel_torque[side] =
            2.0F * config.control.wheel_torque_limit;
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            controller.control_core.actuation_request.leg[side]
                .joint_torque[joint] =
                    -2.0F * config.control.joint_torque_limit;
        }
    }
    bc_controller_execute(&controller, &actuation);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (!actuation_is_zero(&disabled_snapshot.actuation_request)) {
        fputs("new request modified an old snapshot\n", stderr);
        return 1;
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (snapshot.actuation_request.wheel_torque[side] !=
                2.0F * config.control.wheel_torque_limit ||
            snapshot.actuation.wheel_torque[side] !=
                config.control.wheel_torque_limit ||
            snapshot.actuation.wheel_torque[side] ==
                controller.control_core.actuation_request
                    .wheel_torque[side]) {
            fputs("snapshot did not capture limited wheel output\n", stderr);
            return 1;
        }
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            if (snapshot.actuation_request.leg[side]
                    .joint_torque[joint] !=
                    -2.0F * config.control.joint_torque_limit ||
                snapshot.actuation.leg[side].joint_torque[joint] !=
                -config.control.joint_torque_limit) {
                fputs("snapshot did not capture limited joint output\n", stderr);
                return 1;
            }
        }
    }

    command.balance_restart = 1U;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.motion != BC_MOTION_LEG_POSITIONING) {
        fputs("balance restart did not start leg positioning\n", stderr);
        return 1;
    }
    command.balance_restart = 0U;
    command.forward_velocity = 10.0F;
    feedback.gimbal.relative_yaw = 0.1F;
    feedback.gimbal.relative_yaw_rate = 0.2F;

    for (int step = 0; step < 2; ++step) {
        bc_controller_update(&controller, &feedback, 0.001F);
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            controller.control_core.observer.leg[side].length =
                config.motion.leg_length;
            controller.control_core.observer.leg[side].angle_body =
                config.motion.leg_angle_body;
        }
        bc_controller_set_command(&controller, &command);
        bc_controller_calculate(&controller);
        bc_controller_execute(&controller, &actuation);
    }
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.motion != BC_MOTION_BALANCE_ENGAGING ||
        snapshot.state_machine.forward != BC_FORWARD_IDLE ||
        snapshot.state_reference.value[BC_STATE_DS] != 0.0F ||
        snapshot.state_reference.value[BC_STATE_DPSI] != 0.0F ||
        snapshot.tick_count != 5U ||
        fabsf(snapshot.roll_force_request - (-176.0F)) > 1.0e-5F) {
        fputs("stable legs did not engage balance control\n", stderr);
        return 1;
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (snapshot.actuation.wheel_torque[side] !=
            actuation.wheel_torque[side]) {
            fputs("snapshot did not capture the latest actuation\n", stderr);
            return 1;
        }
    }

    command.forward_velocity = 0.0F;

    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.velocity_estimator.measurement_accepted) {
        fputs("wheel update started before its balance delay\n", stderr);
        return 1;
    }
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);

    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (!snapshot.velocity_estimator.measurement_accepted) {
        fputs("wheel update did not start after its balance delay\n", stderr);
        return 1;
    }
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.motion != BC_MOTION_ACTIVE ||
        snapshot.state_machine.forward != BC_FORWARD_HOLD ||
        snapshot.state_machine.alignment != BC_CHASSIS_FRONT ||
        snapshot.gimbal.relative_yaw != 0.1F ||
        snapshot.gimbal.relative_yaw_rate != 0.2F ||
        snapshot.mapped_forward_velocity != 0.0F ||
        snapshot.heading_error != 0.1F ||
        snapshot.state_reference.value[BC_STATE_DS] != 0.0F ||
        fabsf(snapshot.state_reference.value[BC_STATE_PSI] - 0.1F) >
            1.0e-7F ||
        fabsf(snapshot.state_reference.value[BC_STATE_DPSI] - 0.2F) >
            1.0e-7F ||
        snapshot.yaw_acceleration_reference != 10.0F) {
        fputs("pure yaw did not activate in forward hold\n", stderr);
        return 1;
    }

    command.forward_velocity = 10.0F;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.forward != BC_FORWARD_VELOCITY ||
        snapshot.mapped_forward_velocity != 10.0F ||
        fabsf(snapshot.state_reference.value[BC_STATE_DS] - 0.005F) >
            1.0e-7F ||
        fabsf(snapshot.state_reference.value[BC_STATE_PSI] - 0.1F) >
            1.0e-7F ||
        fabsf(snapshot.state_reference.value[BC_STATE_DPSI] - 0.2F) >
            1.0e-7F ||
        snapshot.yaw_acceleration_reference != 0.0F) {
        fputs("forward command did not leave hold\n", stderr);
        return 1;
    }

    controller.system.motion.support_phase.state = BC_SUPPORT_AIRBORNE;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.velocity_estimator.measurement_accepted ||
        snapshot.velocity_estimator.wheel_velocity_reliable ||
        !controller.control_core.observer.velocity_estimator.
            measurement_initialized) {
        fputs("airborne controller accepted wheel velocity\n", stderr);
        return 1;
    }
    controller.system.motion.support_phase.state = BC_SUPPORT_GROUND;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.velocity_estimator.measurement_accepted ||
        snapshot.velocity_estimator.wheel_velocity_reliable) {
        fputs("wheel velocity recovered on the first ground sample\n", stderr);
        return 1;
    }
    for (int step = 0; step < 25; ++step) {
        bc_controller_update(&controller, &feedback, 0.001F);
    }
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (!snapshot.velocity_estimator.measurement_accepted ||
        !snapshot.velocity_estimator.wheel_velocity_reliable) {
        fputs("wheel velocity did not recover after the ground hold\n", stderr);
        return 1;
    }

    command.task = BC_OPERATOR_TASK_STEP_DOCK;
    command.forward_velocity = 0.5F;
    controller.gimbal_feedback.relative_yaw = 0.0F;
    controller.system.motion.support_phase.state = BC_SUPPORT_GROUND;
    controller.control_core.observer.forward_velocity.wheel_odometry = 0.5F;
    controller.control_core.observer.velocity_estimator.output.
        wheel_velocity_reliable = 1U;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        controller.control_core.observer.leg[side].length =
            config.motion.step_task.prepare_leg_length;
        controller.control_core.support_force[side].output.state =
            BC_CONTACT_GROUND;
        controller.control_core.support_force[side].output.
            filtered_vertical_force = 70.0F;
    }
    controller.control_core.observer.impact_observer.output.valid = 1U;
    controller.control_core.observer.impact_observer.output.
        window[BC_IMPACT_WINDOW_SHORT].valid = 1U;
    controller.control_core.observer.impact_observer.output.
        window[BC_IMPACT_WINDOW_SHORT].leg_rate_delta[BC_L] = 0.6F;
    controller.control_core.observer.impact_observer.output.
        window[BC_IMPACT_WINDOW_SHORT].wheel_imu_delta_mismatch = 0.13F;
    bc_controller_set_command(&controller, &command);
    for (int step = 0; step < 3; ++step) {
        bc_controller_calculate(&controller);
    }
    bc_controller_execute(&controller, &actuation);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.step_task !=
            BC_STEP_TASK_IMPACT_PASSIVE ||
        snapshot.step_impact_armed ||
        !actuation_is_zero(&snapshot.actuation_request) ||
        !actuation_is_zero(&snapshot.actuation)) {
        fputs("step passive did not clear all controller actuation\n", stderr);
        return 1;
    }
    command.task = BC_OPERATOR_TASK_NORMAL;
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.step_task != BC_STEP_TASK_TRANSFER ||
        snapshot.step_request.control_mode != BC_STEP_CONTROL_TRANSFER ||
        snapshot.actuation_request.wheel_torque[BC_L] != 0.0F ||
        snapshot.actuation_request.wheel_torque[BC_R] != 0.0F) {
        fputs("step passive did not enter controller transfer\n", stderr);
        return 1;
    }

    controller.system.motion.step_task.state =
        BC_STEP_TASK_TRANSFER_HOLD;
    controller.system.motion.step_task.state_elapsed_seconds =
        config.motion.step_task.transfer_hold_duration - 0.001F;
    controller.control_core.observer.state.value[BC_STATE_S] = 2.0F;
    controller.control_core.observer.state.value[BC_STATE_DS] = 0.2F;
    controller.control_core.observer.state.value[BC_STATE_PSI] = 1.15F;
    controller.control_core.observer.state.value[BC_STATE_DPSI] = -0.1F;
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.step_task != BC_STEP_TASK_RECOVER ||
        snapshot.state_machine.forward != BC_FORWARD_HOLD ||
        snapshot.state_reference.value[BC_STATE_S] != 0.0F ||
        snapshot.state_reference.value[BC_STATE_DS] != 0.0F ||
        snapshot.state_reference.value[BC_STATE_PSI] != 0.0F ||
        snapshot.state_reference.value[BC_STATE_DPSI] != 0.0F ||
        snapshot.yaw_acceleration_reference != 0.0F ||
        snapshot.step_request.control_mode != BC_STEP_CONTROL_RECOVER ||
        !snapshot.step_request.suppress_position_heading_feedback) {
        fputs("controller step recovery catch retained position feedback\n",
              stderr);
        return 1;
    }

    controller.control_core.observer.state.value[BC_STATE_DS] = 0.0F;
    controller.control_core.observer.state.value[BC_STATE_DPSI] = 0.0F;
    controller.control_core.observer.state.value[BC_STATE_THETA_B] = 0.0F;
    controller.control_core.observer.state.value[BC_STATE_DTHETA_B] = 0.0F;
    controller.control_core.observer.state.value[BC_STATE_DTHETA_L] = 0.0F;
    controller.control_core.observer.state.value[BC_STATE_DTHETA_R] = 0.0F;
    controller.control_core.observer.roll = 0.0F;
    controller.control_core.observer.roll_rate = 0.0F;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        controller.control_core.observer.leg[side].length =
            config.motion.step_task.transfer_final_length;
        controller.control_core.observer.leg[side].length_velocity = 0.0F;
        controller.control_core.observer.leg[side].angle_body =
            config.motion.step_task.transfer_final_angle_body;
        controller.control_core.observer.leg[side].angular_velocity = 0.0F;
        controller.control_core.support_force[side].output.valid = 1U;
        controller.control_core.support_force[side].output.state =
            BC_CONTACT_GROUND;
    }
    controller.control_core.observer.velocity_estimator.output.
        wheel_velocity_reliable = 1U;
    controller.system.motion.step_task.recovery_hold.elapsed_seconds =
        config.motion.step_task.recovery_stable_duration - 0.001F;
    command.task = BC_OPERATOR_TASK_STEP_DOCK;
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.step_task != BC_STEP_TASK_RECOVER_LOCK ||
        snapshot.state_reference.value[BC_STATE_S] != 2.0F ||
        snapshot.state_reference.value[BC_STATE_DS] != 0.0F ||
        snapshot.state_reference.value[BC_STATE_PSI] != 1.15F ||
        snapshot.state_reference.value[BC_STATE_DPSI] != 0.0F ||
        snapshot.step_request.suppress_position_heading_feedback ||
        !snapshot.step_request.recovery_reference_capture ||
        !controller.system.motion.step_task.recovery_reference_captured) {
        fputs("controller recovery catch did not capture references\n",
              stderr);
        return 1;
    }
    controller.system.motion.step_task.recovery_hold.elapsed_seconds =
        config.motion.step_task.recovery_stable_duration - 0.001F;
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.step_task != BC_STEP_TASK_COMPLETE ||
        !snapshot.step_command_rearm_required) {
        fputs("controller locked recovery did not complete\n", stderr);
        return 1;
    }
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.step_task != BC_STEP_TASK_INACTIVE ||
        !snapshot.step_command_rearm_required) {
        fputs("controller step completion retriggered without NORMAL\n",
              stderr);
        return 1;
    }

    controller.system.motion.state = BC_MOTION_IDLE;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.system != BC_SYSTEM_ON ||
        snapshot.state_machine.motion != BC_MOTION_IDLE ||
        !actuation_is_zero(&snapshot.actuation)) {
        fputs("balance idle did not disable control output\n", stderr);
        return 1;
    }

    command.balance_restart = 1U;
    command.task = BC_OPERATOR_TASK_NORMAL;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.motion != BC_MOTION_LEG_POSITIONING) {
        fputs("balance restart did not return to leg positioning\n", stderr);
        return 1;
    }

    command.system_enabled = 0U;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.state_machine.system != BC_SYSTEM_OFF ||
        snapshot.state_machine.motion != BC_MOTION_IDLE ||
        snapshot.state_machine.forward != BC_FORWARD_IDLE ||
        snapshot.state_machine.alignment != BC_CHASSIS_FRONT ||
        !actuation_is_zero(&snapshot.actuation)) {
        fputs("system disable did not reset and clear the controller\n", stderr);
        return 1;
    }

    bc_controller_reset(&controller);
    bc_controller_capture_snapshot(&controller, &snapshot);
    if (snapshot.tick_count != 0U ||
        snapshot.state_reference.value[BC_STATE_S] != 0.0F ||
        snapshot.state_reference.value[BC_STATE_PSI] != 0.0F ||
        snapshot.roll != 0.0F ||
        snapshot.roll_rate != 0.0F ||
        snapshot.roll_force_request != 0.0F ||
        snapshot.gimbal.relative_yaw != 0.0F ||
        snapshot.gimbal.relative_yaw_rate != 0.0F ||
        snapshot.mapped_forward_velocity != 0.0F ||
        snapshot.heading_error != 0.0F ||
        snapshot.yaw_acceleration_reference != 0.0F ||
        !actuation_is_zero(&snapshot.actuation_request) ||
        !actuation_is_zero(&snapshot.actuation)) {
        fputs("reset did not clear the captured output\n", stderr);
        return 1;
    }

    return 0;
}
