#include "balance/control_core.h"
#include "balance/control_law/lqr.h"
#include "balance/math_utils.h"
#include "balance/observer.h"

#include <string.h>

_Static_assert(
    BC_STATE_NUM <= 16,
    "state feedback mask is too small for the state vector");

void bc_control_default_config(bc_control_config_t *config) {
    bc_support_force_config_t support_force_estimator;
    bc_impact_observer_config_t impact_observer;
    bc_support_force_default_config(&support_force_estimator);
    bc_impact_observer_default_config(&impact_observer);
    *config = (bc_control_config_t){
        .observer = {
            .leg_geometry = {
                .hip_link_length   = 0.175F,
                .wheel_link_length = 0.208F,
            },
            .impact_observer = impact_observer,
            .velocity_estimator = {
                .gravity                   = 9.81F,
                .initial_velocity_variance = 0.0004F,
                .initial_bias_variance     = 0.000001F,
                .acceleration_variance     = 0.02F,
                .bias_walk_variance        = 0.00000001F,
                .wheel_velocity_variance   = 0.0004F,
                .recovery_velocity_variance = 0.0008F,
                .nis_gate                  = 9.0F,
                .wheel_rejection_duration  = 0.02F,
                .wheel_recovery_duration   = 0.02F,
                .reacquisition_stable_duration = 0.10F,
                .reacquisition_max_wheel_speed = 0.5F,
                .reacquisition_max_wheel_acceleration = 25.0F,
                .reacquisition_velocity_rate = 2.0F,
            },
            .imu_position = {
                .x = 0.0F,
                .y = 0.0F,
                .z = 0.0F,
            },
            .hip_center_position = {
                .x = 0.0F,
                .y = 0.0F,
                .z = 0.08725F,
            },
            .wheel_radius = 0.06F,
            .wheel_velocity_startup_delay = 0.5F,
        },
        .length_controller = {
            .kp           = 1600.0F,
            .kd           = 75.0F,
            .output_limit = 220.0F,
        },
        .angle_controller = {
            .kp           = 50.0F,
            .kd           = 6.0F,
            .output_limit = 30.0F,
        },
        .roll_controller = {
            .kp           = 800.0F,
            .kd           = 60.0F,
            .output_limit = 200.0F,
        },
        .roll_force_sign = {+1.0F, -1.0F},
        .lqr_compensation = {
            .leg_angle_trim = BC_PI_F / 180.0F,
            .yaw_acceleration_feedforward_scale = 0.9F,
        },
        .support_force_estimator = support_force_estimator,
        .support_force      = 103.27294F,
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
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        bc_support_force_init(
            &core->support_force[side],
            &config->support_force_estimator);
    }
    bc_control_core_reset(core);
}

void bc_control_core_reset(bc_control_core_t *core) {
    bc_observer_reset(&core->observer);
    memset(&core->actuation_request, 0, sizeof(core->actuation_request));
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        bc_support_force_reset(&core->support_force[side]);
    }
    core->roll_force_request = 0.0F;
    core->tick_count = 0U;
}

void bc_control_core_update(
    bc_control_core_t *core,
    const bc_sensor_feedback_t *feedback,
    const bc_observation_context_t *context,
    const float timestep_seconds
) {
    bc_observer_update(
        &core->observer, feedback, context, timestep_seconds);
    const int angle_state[BC_SIDE_NUM] = {
        BC_STATE_THETA_L, BC_STATE_THETA_R,
    };
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        bc_support_force_update(
            &core->support_force[side],
            &core->observer.leg[side],
            &feedback->leg[side],
            core->observer.state.value[angle_state[side]],
            timestep_seconds);
    }
}

void bc_control_core_reject_wheel_velocity(bc_control_core_t *core) {
    bc_observer_reject_wheel_velocity(&core->observer);
}

void bc_control_core_calculate(
    bc_control_core_t *core,
    const bc_control_command_t *command
) {
    memset(&core->actuation_request, 0, sizeof(core->actuation_request));
    core->roll_force_request = 0.0F;
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

        bc_state_vector_t state_error = {0};
        for (int state = 0; state < BC_STATE_NUM; ++state) {
            if (command->disabled_state_feedback &
                BC_STATE_FEEDBACK_MASK(state)) continue;

            state_error.value[state] =
                effective_reference.value[state] -
                core->observer.state.value[state];
        }

        bc_lqr_calculate(
            average_length, &state_error,
            core->config.lqr_compensation.
                yaw_acceleration_feedforward_scale *
                command->yaw_acceleration_reference,
            &lqr_output);
    }

    const float roll_force = bc_control_core_roll_force(core);

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
                core->roll_force_request = roll_force;
                axial_force += core->config.support_force;
                axial_force +=
                    core->config.roll_force_sign[side] * roll_force;
            }
            break;

        case BC_LEG_LENGTH_AXIAL_FORCE:
            axial_force = target->axial_force;
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

float bc_control_core_roll_force(const bc_control_core_t *core) {
    return bc_pd_calculate(
        &core->config.roll_controller,
        -core->observer.roll,
        -core->observer.roll_rate);
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
