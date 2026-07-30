#include "balance/control_core.h"
#include "balance/math_utils.h"
#include "balance/observer.h"

#include <string.h>

void bc_control_default_config(bc_control_config_t *config) {
    *config = (bc_control_config_t){
        .observer = {
            .leg_geometry = {
                .hip_link_length   = 0.215F,
                .wheel_link_length = 0.254F,
            },
            .wheel_radius = 0.05806F,
        },
        .length_controller = {
            .kp           = 1600.0F,
            .kd           = 75.0F,
            .output_limit = 220.0F,
        },
        .angle_controller = {
            .kp           = 40.0F,
            .kd           = 6.0F,
            .output_limit = 30.0F,
        },
        .joint_torque_limit = 40.0F,
    };
}

void bc_control_core_init(
    bc_control_core_t *core,
    const bc_control_config_t *config
) {
    core->config = *config;
    bc_observer_init(&core->observer, &config->observer);
    bc_control_core_reset(core);
}

void bc_control_core_reset(bc_control_core_t *core) {
    bc_observer_reset(&core->observer);
    memset(&core->command, 0, sizeof(core->command));
    core->tick_count = 0U;
}

void bc_control_core_update(
    bc_control_core_t *core,
    const bc_sensor_feedback_t *feedback,
    const float timestep_seconds
) {
    bc_observer_update(&core->observer, feedback, timestep_seconds);
}

void bc_control_core_set_command(
    bc_control_core_t *core,
    const bc_operator_command_t *command
) {
    core->command = *command;
}

void bc_control_core_execute(
    bc_control_core_t *core,
    bc_actuation_t *actuation
) {
    memset(actuation, 0, sizeof(*actuation));

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (!core->command.enabled) continue;

        const bc_leg_kinematics_t *leg = &core->observer.leg[side];
        const bc_leg_target_t *target  = &core->command.leg[side];

        const float axial_force = bc_pd_calculate(
            &core->config.length_controller,
            target->length - leg->length, -leg->length_velocity);
        const float leg_torque = bc_pd_calculate(
            &core->config.angle_controller,
            bc_wrap_anglef(target->angle_body - leg->angle_body),
            -leg->angular_velocity);

        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const float joint_torque =
                axial_force * leg->jacobian[BC_LEG_LENGTH][joint] +
                leg_torque  * leg->jacobian[BC_LEG_ANGLE ][joint];

            actuation->leg[side].joint_torque[joint] = bc_clampf(
                joint_torque,
                -core->config.joint_torque_limit,
                +core->config.joint_torque_limit);
        }
    }

    core->tick_count += 1U;
}
