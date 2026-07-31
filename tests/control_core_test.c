#include "balance/control_core.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

int main() {
    bc_control_config_t config;
    bc_control_core_t core;
    bc_sensor_feedback_t feedback = {0};
    bc_operator_command_t command = {0};
    bc_actuation_t actuation;

    bc_control_default_config(&config);
    bc_control_core_init(&core, &config);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        actuation.wheel_torque[side] = 42.0F;
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            actuation.leg[side].joint_torque[joint] = 42.0F;
        }
    }

    bc_control_core_update(&core, &feedback, 0.001F);
    if (core.tick_count != 0U) {
        fputs("update advanced the control tick\n", stderr);
        return 1;
    }
    bc_control_core_set_command(&core, &command);
    if (core.tick_count != 0U) {
        fputs("set_command advanced the control tick\n", stderr);
        return 1;
    }
    bc_control_core_execute(&core, &actuation);
    if (core.tick_count != 1U) {
        fprintf(
            stderr, "unexpected tick count: %" PRIu32 "\n",
            core.tick_count);
        return 1;
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (actuation.wheel_torque[side] != 0.0F) {
            fprintf(stderr, "wheel %d was not cleared\n", side);
            return 1;
        }
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            if (actuation.leg[side].joint_torque[joint] != 0.0F) {
                fprintf(
                    stderr, "leg %d joint %d was not cleared\n", side, joint);
                return 1;
            }
        }
    }

    command.enabled = 1U;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        feedback.leg[side].joint[BC_FRONT].angle = 3.10F;
        feedback.leg[side].joint[BC_REAR].angle = 3.10F;
        command.leg[side].length =
            config.observer.leg_geometry.hip_link_length +
            config.observer.leg_geometry.wheel_link_length;
        command.leg[side].angle_body = -3.10F;
    }
    bc_control_core_update(&core, &feedback, 0.001F);
    bc_control_core_set_command(&core, &command);
    bc_control_core_execute(&core, &actuation);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const float torque = actuation.leg[side].joint_torque[joint];
            if (torque <= 0.0F || torque >= 2.0F) {
                fprintf(
                    stderr, "leg %d joint %d did not use wrapped angle error\n",
                    side, joint);
                return 1;
            }
        }
    }

    bc_control_config_t supported_config = config;
    bc_control_config_t unsupported_config = config;
    supported_config.joint_torque_limit = 1000.0F;
    supported_config.wheel_torque_limit = 0.5F;
    unsupported_config.joint_torque_limit = 1000.0F;
    unsupported_config.wheel_torque_limit = 0.5F;
    unsupported_config.support_force = 0.0F;

    bc_control_core_t supported_core;
    bc_control_core_t unsupported_core;
    bc_control_core_init(&supported_core, &supported_config);
    bc_control_core_init(&unsupported_core, &unsupported_config);
    bc_control_core_update(&supported_core, &feedback, 0.001F);
    bc_control_core_update(&unsupported_core, &feedback, 0.001F);

    command = (bc_operator_command_t){
        .enabled = 1U,
        .balance_enabled = 1U,
        .state_reference = supported_core.observer.state,
    };
    command.state_reference.value[BC_STATE_S] += 100.0F;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        command.leg[side].length = supported_core.observer.leg[side].length;
    }
    bc_control_core_set_command(&supported_core, &command);
    bc_control_core_set_command(&unsupported_core, &command);

    bc_actuation_t supported_actuation;
    bc_actuation_t unsupported_actuation;
    bc_control_core_execute(&supported_core, &supported_actuation);
    bc_control_core_execute(&unsupported_core, &unsupported_actuation);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (fabsf(supported_actuation.wheel_torque[side]) != 0.5F) {
            fprintf(stderr, "wheel %d did not use the torque limit\n", side);
            return 1;
        }
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const float expected_difference = config.support_force *
                supported_core.observer.leg[side]
                    .jacobian[BC_LEG_LENGTH][joint];
            const float actual_difference =
                supported_actuation.leg[side].joint_torque[joint] -
                unsupported_actuation.leg[side].joint_torque[joint];
            if (fabsf(actual_difference - expected_difference) > 1.0e-4F) {
                fprintf(
                    stderr,
                    "leg %d joint %d did not add support feedforward\n",
                    side, joint);
                return 1;
            }
        }
    }

    bc_control_core_reset(&core);
    if (core.tick_count != 0U) {
        fputs("reset did not clear tick count\n", stderr);
        return 1;
    }

    return 0;
}
