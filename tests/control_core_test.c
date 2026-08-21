#include "balance/control_core.h"
#include "balance/control_law/lqr.h"
#include "balance/math_utils.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

int main() {
    bc_control_config_t config;
    bc_control_core_t core;
    bc_sensor_feedback_t feedback = {0};
    bc_control_command_t command = {0};
    bc_actuation_t actuation;
    const bc_observation_context_t observation_context = {
        .wheel_velocity = BC_WHEEL_OBSERVATION_DISABLED,
    };

    bc_control_default_config(&config);
    if (fabsf(
        config.lqr_compensation.leg_angle_trim -
            BC_PI_F / 180.0F) > 1.0e-7F ||
        config.lqr_compensation.yaw_acceleration_feedforward_scale != 0.9F ||
        config.roll_controller.kp != 800.0F ||
        config.roll_controller.kd != 60.0F ||
        config.roll_controller.output_limit != 200.0F ||
        config.roll_force_sign[BC_L] != +1.0F ||
        config.roll_force_sign[BC_R] != -1.0F ||
        fabsf(config.support_force - 103.27294F) > 1.0e-4F) {
        fputs("default LQR compensation is incorrect\n", stderr);
        return 1;
    }
    bc_control_core_init(&core, &config);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        actuation.wheel_torque[side] = 42.0F;
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            actuation.leg[side].joint_torque[joint] = 42.0F;
        }
    }

    bc_control_core_update(&core, &feedback, &observation_context, 0.001F);
    if (core.tick_count != 0U) {
        fputs("update advanced the control tick\n", stderr);
        return 1;
    }
    bc_control_core_calculate(&core, &command);
    if (core.tick_count != 1U) {
        fprintf(
            stderr, "unexpected tick count: %" PRIu32 "\n",
            core.tick_count);
        return 1;
    }
    bc_control_core_execute(&core, 1U, &actuation);
    if (core.tick_count != 1U) {
        fputs("execute advanced the control tick\n", stderr);
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

    bc_control_config_t feedback_strategy_config = config;
    feedback_strategy_config.lqr_compensation.leg_angle_trim = 0.0F;
    bc_control_core_t feedback_strategy_core;
    bc_control_core_init(
        &feedback_strategy_core, &feedback_strategy_config);
    bc_control_core_update(
        &feedback_strategy_core, &feedback, &observation_context, 0.001F);
    bc_control_command_t feedback_strategy_command = {
        .wheel_strategy = BC_WHEEL_LQR,
        .disabled_state_feedback =
            BC_STATE_FEEDBACK_MASK(BC_STATE_S),
    };
    feedback_strategy_command.state_reference.value[BC_STATE_S] = 100.0F;
    feedback_strategy_command.state_reference.value[BC_STATE_THETA_B] = 0.1F;
    bc_control_core_calculate(
        &feedback_strategy_core, &feedback_strategy_command);
    float masked_wheel[BC_SIDE_NUM];
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        masked_wheel[side] = feedback_strategy_core.actuation_request
            .wheel_torque[side];
    }
    feedback_strategy_command.state_reference.value[BC_STATE_S] = 0.0F;
    bc_control_core_calculate(
        &feedback_strategy_core, &feedback_strategy_command);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (fabsf(
                feedback_strategy_core.actuation_request
                    .wheel_torque[side] - masked_wheel[side]) > 1.0e-6F) {
            fputs("masked state changed the LQR output\n", stderr);
            return 1;
        }
    }
    feedback_strategy_command.disabled_state_feedback = 0U;
    feedback_strategy_command.state_reference.value[BC_STATE_S] = 100.0F;
    bc_control_core_calculate(
        &feedback_strategy_core, &feedback_strategy_command);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (fabsf(
                feedback_strategy_core.actuation_request
                    .wheel_torque[side] - masked_wheel[side]) <= 1.0e-3F) {
            fputs("unmasked state did not change the LQR output\n", stderr);
            return 1;
        }
    }

    bc_control_config_t feedforward_config = config;
    feedforward_config.lqr_compensation.leg_angle_trim = 0.0F;
    feedforward_config.lqr_compensation.
        yaw_acceleration_feedforward_scale = 0.5F;
    bc_control_core_t feedforward_core;
    bc_control_core_init(&feedforward_core, &feedforward_config);
    bc_control_core_update(
        &feedforward_core, &feedback, &observation_context, 0.001F);
    bc_control_command_t feedforward_command = {
        .wheel_strategy = BC_WHEEL_LQR,
        .state_reference = feedforward_core.observer.state,
        .yaw_acceleration_reference = 2.0F,
    };
    bc_control_core_calculate(&feedforward_core, &feedforward_command);
    const float feedforward_length = 0.5F * (
        feedforward_core.observer.leg[BC_L].length +
        feedforward_core.observer.leg[BC_R].length);
    bc_state_vector_t zero_error = {0};
    bc_lqr_output_t expected_feedforward = {0};
    bc_lqr_calculate(
        feedforward_length, &zero_error, 1.0F,
        &expected_feedforward);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (fabsf(
                feedforward_core.actuation_request.wheel_torque[side] -
                expected_feedforward.wheel_torque[side]) > 1.0e-6F) {
            fprintf(
                stderr,
                "wheel %d did not apply yaw acceleration feedforward\n",
                side);
            return 1;
        }
    }

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        feedback.leg[side].joint[BC_FRONT].angle = 3.10F;
        feedback.leg[side].joint[BC_REAR].angle = 3.10F;
        command.leg[side].length_strategy = BC_LEG_LENGTH_POSITION;
        command.leg[side].angle_strategy = BC_LEG_ANGLE_POSITION;
        command.leg[side].target.length =
            config.observer.leg_geometry.hip_link_length +
            config.observer.leg_geometry.wheel_link_length;
        command.leg[side].target.angle_body = -3.10F;
    }
    bc_control_core_update(&core, &feedback, &observation_context, 0.001F);
    bc_control_core_calculate(&core, &command);
    bc_control_core_execute(&core, 1U, &actuation);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const float torque = actuation.leg[side].joint_torque[joint];
            if (torque <= 0.0F || torque >= 3.0F) {
                fprintf(
                    stderr, "leg %d joint %d did not use wrapped angle error\n",
                    side, joint);
                return 1;
            }
        }
    }

    bc_control_core_execute(&core, 0U, &actuation);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (actuation.wheel_torque[side] != 0.0F) {
            fprintf(stderr, "locked wheel %d was not cleared\n", side);
            return 1;
        }
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            if (actuation.leg[side].joint_torque[joint] != 0.0F) {
                fprintf(
                    stderr, "locked leg %d joint %d was not cleared\n",
                    side, joint);
                return 1;
            }
        }
    }

    bc_control_config_t trim_config = config;
    trim_config.lqr_compensation.leg_angle_trim = 0.12F;
    bc_control_core_t trim_core;
    bc_control_core_init(&trim_core, &trim_config);
    bc_control_core_update(
        &trim_core, &feedback, &observation_context, 0.001F);

    bc_control_command_t trim_command = {
        .wheel_strategy = BC_WHEEL_LQR,
    };
    bc_control_core_calculate(&trim_core, &trim_command);

    bc_state_vector_t effective_reference = {0};
    effective_reference.value[BC_STATE_THETA_L] = 0.12F;
    effective_reference.value[BC_STATE_THETA_R] = 0.12F;
    bc_lqr_output_t expected_lqr = {0};
    const float trim_length = 0.5F * (
        trim_core.observer.leg[BC_L].length +
        trim_core.observer.leg[BC_R].length);
    bc_state_vector_t state_error = {0};
    for (int state = 0; state < BC_STATE_NUM; ++state) {
        state_error.value[state] = effective_reference.value[state] -
            trim_core.observer.state.value[state];
    }
    bc_lqr_calculate(trim_length, &state_error, 0.0F, &expected_lqr);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (fabsf(
                trim_core.actuation_request.wheel_torque[side] -
                expected_lqr.wheel_torque[side]) > 1.0e-6F) {
            fprintf(
                stderr, "wheel %d did not apply the LQR compensation\n",
                side);
            return 1;
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
    bc_control_core_update(
        &supported_core, &feedback, &observation_context, 0.001F);
    bc_control_core_update(
        &unsupported_core, &feedback, &observation_context, 0.001F);

    command = (bc_control_command_t){
        .wheel_strategy = BC_WHEEL_LQR,
        .state_reference = supported_core.observer.state,
    };
    command.state_reference.value[BC_STATE_S] += 100.0F;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        command.leg[side].length_strategy =
            BC_LEG_LENGTH_POSITION_SUPPORT;
        command.leg[side].angle_strategy = BC_LEG_ANGLE_LQR;
        command.leg[side].target.length =
            supported_core.observer.leg[side].length;
    }
    bc_control_core_calculate(&supported_core, &command);
    bc_control_core_calculate(&unsupported_core, &command);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (fabsf(supported_core.actuation_request.wheel_torque[side]) <=
            supported_config.wheel_torque_limit) {
            fprintf(
                stderr, "wheel %d request was limited before execute\n", side);
            return 1;
        }
    }

    bc_actuation_t supported_actuation;
    bc_actuation_t unsupported_actuation;
    bc_control_core_execute(&supported_core, 1U, &supported_actuation);
    bc_control_core_execute(&unsupported_core, 1U, &unsupported_actuation);
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

    bc_control_config_t roll_config = config;
    roll_config.lqr_compensation.leg_angle_trim = 0.0F;
    roll_config.lqr_compensation.yaw_acceleration_feedforward_scale = 0.0F;
    roll_config.support_force = 0.0F;
    roll_config.joint_torque_limit = 1000.0F;
    bc_control_core_t roll_core;
    bc_sensor_feedback_t roll_feedback = {0};
    bc_control_core_init(&roll_core, &roll_config);
    bc_control_core_update(
        &roll_core, &roll_feedback, &observation_context, 0.001F);
    roll_core.observer.roll = 0.1F;
    roll_core.observer.roll_rate = 0.2F;
    bc_control_command_t roll_command = {
        .wheel_strategy = BC_WHEEL_LQR,
        .state_reference = roll_core.observer.state,
    };
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        roll_command.leg[side].length_strategy =
            BC_LEG_LENGTH_POSITION_SUPPORT;
        roll_command.leg[side].angle_strategy = BC_LEG_ANGLE_LQR;
        roll_command.leg[side].target.length =
            roll_core.observer.leg[side].length;
    }
    bc_control_core_calculate(&roll_core, &roll_command);
    const float expected_roll_force = -92.0F;
    if (fabsf(
            roll_core.roll_force_request - expected_roll_force) > 1.0e-5F) {
        fputs("roll PD output is incorrect\n", stderr);
        return 1;
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const float sign = side == BC_L ? +1.0F : -1.0F;
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const float expected = sign * expected_roll_force *
                roll_core.observer.leg[side]
                    .jacobian[BC_LEG_LENGTH][joint];
            if (fabsf(
                    roll_core.actuation_request.leg[side]
                        .joint_torque[joint] - expected) > 1.0e-5F) {
                fprintf(
                    stderr,
                    "leg %d joint %d did not apply roll differential force\n",
                    side, joint);
                return 1;
            }
        }
    }
    roll_command.wheel_strategy = BC_WHEEL_DISABLED;
    bc_control_core_calculate(&roll_core, &roll_command);
    if (fabsf(
            roll_core.roll_force_request - expected_roll_force) > 1.0e-5F) {
        fputs("wheel strategy unexpectedly disabled roll PD\n", stderr);
        return 1;
    }
    roll_core.config.roll_force_sign[BC_L] = -1.0F;
    roll_core.config.roll_force_sign[BC_R] = +1.0F;
    bc_control_core_calculate(&roll_core, &roll_command);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const float sign = side == BC_L ? -1.0F : +1.0F;
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const float expected = sign * expected_roll_force *
                roll_core.observer.leg[side]
                    .jacobian[BC_LEG_LENGTH][joint];
            if (fabsf(
                    roll_core.actuation_request.leg[side]
                        .joint_torque[joint] - expected) > 1.0e-5F) {
                fputs("roll force polarity did not reverse both legs\n", stderr);
                return 1;
            }
        }
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        roll_command.leg[side].length_strategy = BC_LEG_LENGTH_POSITION;
    }
    bc_control_core_calculate(&roll_core, &roll_command);
    if (roll_core.roll_force_request != 0.0F) {
        fputs("position-only leg control did not disable roll PD\n", stderr);
        return 1;
    }

    bc_control_config_t force_config = config;
    force_config.joint_torque_limit = 1000.0F;
    bc_control_core_t force_core;
    bc_control_core_init(&force_core, &force_config);
    bc_control_core_update(
        &force_core, &feedback, &observation_context, 0.001F);
    bc_control_command_t force_command = {0};
    force_command.leg[BC_L].length_strategy = BC_LEG_LENGTH_AXIAL_FORCE;
    force_command.leg[BC_L].target.axial_force = 123.0F;
    force_command.leg[BC_R].length_strategy = BC_LEG_LENGTH_DISABLED;
    bc_control_core_calculate(&force_core, &force_command);
    for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
        const float expected = 123.0F * force_core.observer.leg[BC_L]
            .jacobian[BC_LEG_LENGTH][joint];
        if (fabsf(
                force_core.actuation_request.leg[BC_L]
                    .joint_torque[joint] - expected) > 1.0e-5F ||
            force_core.actuation_request.leg[BC_R]
                    .joint_torque[joint] != 0.0F) {
            fputs("direct axial force mapping is incorrect\n", stderr);
            return 1;
        }
    }
    force_core.config.joint_torque_limit = 0.5F;
    bc_control_core_execute(&force_core, 1U, &actuation);
    for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
        if (fabsf(actuation.leg[BC_L].joint_torque[joint]) > 0.5F) {
            fputs("direct axial force bypassed joint torque limiting\n", stderr);
            return 1;
        }
    }

    bc_control_core_reset(&core);
    if (core.tick_count != 0U) {
        fputs("reset did not clear tick count\n", stderr);
        return 1;
    }

    return 0;
}
