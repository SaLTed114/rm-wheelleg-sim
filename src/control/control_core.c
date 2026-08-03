#include "balance/control_core.h"
#include "balance/control_law/lqr.h"
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
        .lqr_compensation = {
            .leg_angle_trim = 5.5F * BC_PI_F / 180.0F,
        },
        .support_force      = 54.0F,
        .wheel_torque_limit = 6.32F,
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
    memset(&core->actuation_request, 0, sizeof(core->actuation_request));
    core->tick_count = 0U;
}

void bc_control_core_update(
    bc_control_core_t *core,
    const bc_sensor_feedback_t *feedback,
    const float timestep_seconds
) {
    bc_observer_update(&core->observer, feedback, timestep_seconds);
}

void bc_control_core_calculate(
    bc_control_core_t *core,
    const bc_control_command_t *command
) {
    memset(&core->actuation_request, 0, sizeof(core->actuation_request));
    core->tick_count += 1U;

    uint8_t lqr_required =
        command->wheel_strategy == BC_WHEEL_LQR;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        lqr_required = lqr_required ||
            command->leg[side].angle_strategy == BC_LEG_ANGLE_LQR;
    }

    bc_lqr_output_t lqr_output = {0};
    if (lqr_required) {
        const float average_length = 0.5F * (
            core->observer.leg[BC_L].length +
            core->observer.leg[BC_R].length);
        bc_state_vector_t effective_reference = command->state_reference;
        effective_reference.value[BC_STATE_THETA_L] +=
            core->config.lqr_compensation.leg_angle_trim;
        effective_reference.value[BC_STATE_THETA_R] +=
            core->config.lqr_compensation.leg_angle_trim;

        bc_lqr_calculate(
            average_length, &core->observer.state,
            &effective_reference, &lqr_output);
    }

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const bc_leg_kinematics_t *leg = &core->observer.leg[side];
        const bc_leg_control_command_t *leg_command =
            &command->leg[side];
        const bc_leg_target_t *target = &leg_command->target;
        bc_leg_request_t *request = &core->actuation_request.leg[side];

        float axial_force = 0.0F;
        switch (leg_command->length_strategy) {
        case BC_LEG_LENGTH_DISABLED:
            break;

        case BC_LEG_LENGTH_POSITION:
        case BC_LEG_LENGTH_POSITION_SUPPORT:
            axial_force = bc_pd_calculate(
                &core->config.length_controller,
                target->length - leg->length, -leg->length_velocity);
            if (leg_command->length_strategy ==
                BC_LEG_LENGTH_POSITION_SUPPORT) {
                axial_force += core->config.support_force;
            }
            break;
        }

        float leg_torque = 0.0F;
        switch (leg_command->angle_strategy) {
        case BC_LEG_ANGLE_DISABLED:
            break;

        case BC_LEG_ANGLE_POSITION:
            leg_torque = bc_pd_calculate(
                &core->config.angle_controller,
                bc_wrap_anglef(target->angle_body - leg->angle_body),
                -leg->angular_velocity);
            break;

        case BC_LEG_ANGLE_LQR:
            leg_torque = lqr_output.leg_torque[side];
            break;
        }

        switch (command->wheel_strategy) {
        case BC_WHEEL_DISABLED:
            break;

        case BC_WHEEL_LQR:
            core->actuation_request.wheel_torque[side] =
                lqr_output.wheel_torque[side];
            break;
        }

        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const float joint_torque =
                axial_force * leg->jacobian[BC_LEG_LENGTH][joint] +
                leg_torque  * leg->jacobian[BC_LEG_ANGLE ][joint];

            request->joint_torque[joint] = joint_torque;
        }
    }
}

void bc_control_core_execute(
    bc_control_core_t *core,
    const uint8_t output_enabled,
    bc_actuation_t *actuation
) {
    memset(actuation, 0, sizeof(*actuation));

    if (!output_enabled) return;

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const bc_leg_request_t *request =
            &core->actuation_request.leg[side];
        actuation->wheel_torque[side] = bc_clampf(
            core->actuation_request.wheel_torque[side],
            -core->config.wheel_torque_limit,
            +core->config.wheel_torque_limit);

        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            actuation->leg[side].joint_torque[joint] = bc_clampf(
                request->joint_torque[joint],
                -core->config.joint_torque_limit,
                +core->config.joint_torque_limit);
        }
    }
}
