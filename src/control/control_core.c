#include "balance/control_core.h"
#include "balance/leg_kinematics.h"

#include <math.h>
#include <string.h>

static float clamp(float value, float limit) {
    if (value > +limit) return +limit;
    if (value < -limit) return -limit;
    return value;
}

static float wrap_angle(float angle) {
    const float pi = 3.14159265358979323846F;
    return remainderf(angle, 2.0F * pi);
}

void bc_control_default_config(bc_control_config_t *config) {
    *config = (bc_control_config_t){
        .leg_geometry = {
            .hip_link_length   = 0.215F,
            .wheel_link_length = 0.254F,
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
    bc_control_core_reset(core);
}

void bc_control_core_reset(bc_control_core_t *core) {
    memset(core->leg, 0, sizeof(core->leg));
    core->tick_count = 0U;
}

void bc_control_core_step(
    bc_control_core_t *core,
    const bc_observation_t *observation,
    const bc_operator_command_t *command,
    bc_actuation_t *actuation
) {
    memset(actuation, 0, sizeof(*actuation));

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        bc_leg_kinematics_calculate(
            &core->config.leg_geometry, &observation->leg[side],
            &core->leg[side]);

        if (!command->enabled) continue;

        const bc_leg_kinematics_t *leg = &core->leg[side];
        const bc_leg_target_t *target  = &command->leg[side];

        const float axial_force = bc_pd_calculate(
            &core->config.length_controller,
            target->length - leg->length, -leg->length_velocity);
        const float leg_torque = bc_pd_calculate(
            &core->config.angle_controller,
            wrap_angle(target->angle_body - leg->angle_body),
            -leg->angular_velocity);

        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const float joint_torque =
                axial_force * leg->jacobian[BC_LEG_LENGTH][joint] +
                leg_torque  * leg->jacobian[BC_LEG_ANGLE ][joint];

            actuation->leg[side].joint_torque[joint] = clamp(
                joint_torque, core->config.joint_torque_limit);
        }
    }

    core->tick_count += 1U;
}
