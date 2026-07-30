#include "balance/control_core.h"

#include <inttypes.h>
#include <stdio.h>

int main() {
    bc_control_config_t config;
    bc_control_core_t core;
    bc_observation_t observation = {0};
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

    bc_control_core_step(&core, &observation, &command, &actuation);
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
        observation.leg[side].joint[BC_FRONT].angle = 3.10F;
        observation.leg[side].joint[BC_REAR].angle = 3.10F;
        command.leg[side].length =
            config.leg_geometry.hip_link_length +
            config.leg_geometry.wheel_link_length;
        command.leg[side].angle_body = -3.10F;
    }
    bc_control_core_step(&core, &observation, &command, &actuation);
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

    bc_control_core_reset(&core);
    if (core.tick_count != 0U) {
        fputs("reset did not clear tick count\n", stderr);
        return 1;
    }

    return 0;
}
