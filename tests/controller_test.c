#include "balance/controller.h"

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
    bc_sensor_feedback_t feedback = {0};
    bc_operator_command_t command = {0};
    bc_actuation_t actuation;

    bc_controller_default_config(&config);
    config.motion.stable_duration = 0.002F;
    bc_controller_init(&controller, &config);

    feedback.imu.pitch = 0.1F;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    const bc_controller_status_t *status = bc_controller_get_status(
        &controller);
    if (status->system_state != BC_SYSTEM_OFF ||
        status->motion_state != BC_MOTION_IDLE ||
        status->state.value[BC_STATE_THETA_B] != 0.1F ||
        !actuation_is_zero(&actuation)) {
        fputs("disabled controller status or output is incorrect\n", stderr);
        return 1;
    }

    command.system_enabled = 1U;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_set_command(&controller, &command);
    status = bc_controller_get_status(&controller);
    if (status->system_state != BC_SYSTEM_OFF ||
        status->motion_state != BC_MOTION_IDLE ||
        status->tick_count != 1U) {
        fputs("set_command changed controller state\n", stderr);
        return 1;
    }
    bc_controller_calculate(&controller);
    status = bc_controller_get_status(&controller);
    if (status->system_state != BC_SYSTEM_ON ||
        status->motion_state != BC_MOTION_IDLE) {
        fputs("system enable did not wait for balance restart\n", stderr);
        return 1;
    }

    command.balance_restart = 1U;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    status = bc_controller_get_status(&controller);
    if (status->motion_state != BC_MOTION_LEG_POSITIONING) {
        fputs("balance restart did not start leg positioning\n", stderr);
        return 1;
    }
    command.balance_restart = 0U;

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
    status = bc_controller_get_status(&controller);
    if (status->motion_state != BC_MOTION_BALANCE_ENGAGING ||
        status->tick_count != 5U) {
        fputs("stable legs did not engage balance control\n", stderr);
        return 1;
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (status->actuation.wheel_torque[side] !=
            actuation.wheel_torque[side]) {
            fputs("status did not retain the latest actuation\n", stderr);
            return 1;
        }
    }

    controller.system.motion.state = BC_MOTION_IDLE;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    status = bc_controller_get_status(&controller);
    if (status->system_state != BC_SYSTEM_ON ||
        status->motion_state != BC_MOTION_IDLE ||
        !actuation_is_zero(&actuation)) {
        fputs("balance idle did not disable control output\n", stderr);
        return 1;
    }

    command.balance_restart = 1U;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    status = bc_controller_get_status(&controller);
    if (status->motion_state != BC_MOTION_LEG_POSITIONING) {
        fputs("balance restart did not return to leg positioning\n", stderr);
        return 1;
    }

    command.system_enabled = 0U;
    bc_controller_update(&controller, &feedback, 0.001F);
    bc_controller_set_command(&controller, &command);
    bc_controller_calculate(&controller);
    bc_controller_execute(&controller, &actuation);
    status = bc_controller_get_status(&controller);
    if (status->system_state != BC_SYSTEM_OFF ||
        status->motion_state != BC_MOTION_IDLE ||
        !actuation_is_zero(&actuation)) {
        fputs("system disable did not reset and clear the controller\n", stderr);
        return 1;
    }

    return 0;
}
