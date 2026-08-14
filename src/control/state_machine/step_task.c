#include "balance/state_machine/step_task.h"

#include "balance/math_utils.h"

#include <math.h>
#include <stddef.h>

static float bc_step_interpolate(
    const float elapsed,
    const float end_time,
    const float start,
    const float end
) {
    const float ratio = end_time > 0.0F ?
        bc_clampf(elapsed / end_time, 0.0F, 1.0F) : 1.0F;
    return start + ratio * (end - start);
}

static float bc_step_transfer_reference(
    const bc_step_task_config_t *config,
    const float elapsed,
    const float start,
    const float first,
    const float second,
    const float third,
    const float final
) {
    if (elapsed < config->transfer_first_time) {
        return bc_step_interpolate(
            elapsed, config->transfer_first_time, start, first);
    }
    if (elapsed < config->transfer_second_time) {
        return bc_step_interpolate(
            elapsed - config->transfer_first_time,
            config->transfer_second_time - config->transfer_first_time,
            first, second);
    }
    if (elapsed < config->transfer_third_time) {
        return bc_step_interpolate(
            elapsed - config->transfer_second_time,
            config->transfer_third_time - config->transfer_second_time,
            second, third);
    }
    return bc_step_interpolate(
        elapsed - config->transfer_third_time,
        config->transfer_end_time - config->transfer_third_time,
        third, final);
}

static void bc_step_task_set_state(
    bc_step_task_t *task,
    const bc_step_task_state_t state
) {
    task->state = state;
    task->state_elapsed_seconds = 0.0F;
}

static uint8_t bc_step_task_duration_elapsed(
    const bc_step_task_t *task,
    const bc_state_machine_input_t *input,
    const float duration
) {
    return task->state_elapsed_seconds +
        0.5F * fmaxf(input->timestep_seconds, 0.0F) >= duration;
}

static uint8_t bc_step_task_recovery_timed_out(
    const bc_step_task_t *task,
    const bc_state_machine_input_t *input
) {
    return task->recovery_elapsed_seconds +
        0.5F * fmaxf(input->timestep_seconds, 0.0F) >=
        task->config.recovery_timeout;
}

static void bc_step_task_capture_transfer_start(
    bc_step_task_t *task,
    const bc_state_machine_input_t *input
) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        task->transfer_start_length[side] = input->leg[side].length;
        task->transfer_start_angle_body[side] =
            input->leg[side].angle_body;
        task->transfer_length_reference[side] =
            task->transfer_start_length[side];
        task->transfer_angle_reference[side] =
            task->transfer_start_angle_body[side];
    }
}

static void bc_step_task_update_transfer_reference(
    bc_step_task_t *task
) {
    const bc_step_task_config_t *config = &task->config;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        task->transfer_length_reference[side] =
            bc_step_transfer_reference(
                config, task->state_elapsed_seconds,
                task->transfer_start_length[side],
                config->transfer_first_length,
                config->transfer_second_length,
                config->transfer_third_length,
                config->transfer_final_length);
        task->transfer_angle_reference[side] =
            bc_step_transfer_reference(
                config, task->state_elapsed_seconds,
                task->transfer_start_angle_body[side],
                config->transfer_first_angle_body,
                config->transfer_second_angle_body,
                config->transfer_third_angle_body,
                config->transfer_final_angle_body);
    }
}

static uint8_t bc_step_task_recovery_stable(
    const bc_step_task_t *task,
    const bc_state_machine_input_t *input
) {
    const bc_step_task_config_t *config = &task->config;
    const int leg_angular_velocity_state[BC_SIDE_NUM] = {
        BC_STATE_DTHETA_L, BC_STATE_DTHETA_R,
    };
    if (input->state == NULL || input->support_force == NULL ||
        !input->wheel_velocity_reliable ||
        fabsf(input->state->value[BC_STATE_THETA_B]) >
            config->recovery_pitch_tolerance ||
        fabsf(input->state->value[BC_STATE_DTHETA_B]) >
            config->recovery_pitch_rate_tolerance ||
        fabsf(input->roll) > config->recovery_roll_tolerance ||
        fabsf(input->roll_rate) > config->recovery_roll_rate_tolerance ||
        fabsf(input->state->value[BC_STATE_DS]) >
            config->recovery_velocity_tolerance ||
        fabsf(input->state->value[BC_STATE_DPSI]) >
            config->recovery_yaw_rate_tolerance) {
        return 0U;
    }

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (!input->support_force[side].valid ||
            input->support_force[side].state != BC_CONTACT_GROUND ||
            fabsf(input->leg[side].length -
                config->transfer_final_length) >
                config->recovery_leg_length_tolerance ||
            fabsf(input->leg[side].length_velocity) >
                config->recovery_leg_speed_tolerance ||
            fabsf(bc_wrap_anglef(
                input->leg[side].angle_body -
                config->transfer_final_angle_body)) >
                config->recovery_leg_angle_tolerance ||
            fabsf(input->state->value[
                leg_angular_velocity_state[side]]) >
                config->recovery_leg_angular_velocity_tolerance) {
            return 0U;
        }
    }
    return 1U;
}

static void bc_step_task_prepare_request(bc_step_task_t *task) {
    bc_step_task_request_t request = {
        .control_mode = BC_STEP_CONTROL_NORMAL,
        .working_leg_length = task->config.transfer_final_length,
    };
    switch (task->state) {
    case BC_STEP_TASK_INACTIVE:
        break;
    case BC_STEP_TASK_PREPARE:
        request.active = 1U;
        request.force_front_alignment = 1U;
        request.working_leg_length = task->config.prepare_leg_length;
        break;
    case BC_STEP_TASK_IMPACT_PASSIVE:
        request.control_mode = BC_STEP_CONTROL_PASSIVE;
        request.active = 1U;
        request.force_front_alignment = 1U;
        request.suppress_forward = 1U;
        break;
    case BC_STEP_TASK_TRANSFER:
    case BC_STEP_TASK_TRANSFER_HOLD:
        request.control_mode = BC_STEP_CONTROL_TRANSFER;
        request.active = 1U;
        request.force_front_alignment = 1U;
        request.suppress_forward = 1U;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            request.leg_length[side] =
                task->transfer_length_reference[side];
            request.leg_angle_body[side] =
                task->transfer_angle_reference[side];
        }
        break;
    case BC_STEP_TASK_RECOVER:
        request.control_mode = BC_STEP_CONTROL_RECOVER;
        request.active = 1U;
        request.force_front_alignment = 1U;
        request.suppress_forward = 1U;
        request.suppress_position_heading_feedback = 1U;
        break;
    case BC_STEP_TASK_RECOVER_LOCK:
    case BC_STEP_TASK_COMPLETE:
        request.control_mode = BC_STEP_CONTROL_RECOVER;
        request.active = 1U;
        request.force_front_alignment = 1U;
        request.suppress_forward = 1U;
        break;
    case BC_STEP_TASK_RECOVERY_FAILED:
        request.control_mode = BC_STEP_CONTROL_RECOVER;
        request.active = 1U;
        request.force_front_alignment = 1U;
        request.suppress_forward = 1U;
        request.suppress_position_heading_feedback =
            !task->recovery_reference_captured;
        break;
    }
    task->request = request;
}

static uint8_t bc_step_task_prepared(
    const bc_step_task_t *task,
    const bc_state_machine_input_t *input,
    const bc_support_phase_state_t support_state,
    const float heading_error
) {
    if (support_state != BC_SUPPORT_GROUND ||
        !input->wheel_velocity_reliable ||
        input->wheel_odometry_velocity <
            task->config.minimum_forward_velocity ||
        fabsf(heading_error) > task->config.alignment_tolerance) {
        return 0U;
    }

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (fabsf(
                input->leg[side].length -
                task->config.prepare_leg_length) >
            task->config.leg_length_tolerance) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t bc_step_task_impact_candidate(
    const bc_step_task_t *task,
    const bc_impact_observer_output_t *impact
) {
    if (impact == NULL || !impact->valid) return 0U;
    const bc_impact_window_output_t *window =
        &impact->window[BC_IMPACT_WINDOW_SHORT];
    if (!window->valid) return 0U;

    const float leg_rate_delta = fmaxf(
        fabsf(window->leg_rate_delta[BC_L]),
        fabsf(window->leg_rate_delta[BC_R]));
    return leg_rate_delta > task->config.leg_rate_delta_threshold &&
        fabsf(window->wheel_imu_delta_mismatch) >
            task->config.wheel_imu_mismatch_threshold;
}

void bc_step_task_default_config(bc_step_task_config_t *config) {
    *config = (bc_step_task_config_t){
        .prepare_leg_length = 0.38F,
        .leg_length_tolerance = 0.020F,
        .alignment_tolerance = 5.0F * BC_PI_F / 180.0F,
        .minimum_forward_velocity = 0.3F,
        .leg_rate_delta_threshold = 0.5F,
        .wheel_imu_mismatch_threshold = 0.12F,
        .impact_confirm_duration = 0.002F,
        .impact_passive_duration = 0.001F,
        .transfer_first_time = 0.08F,
        .transfer_second_time = 0.16F,
        .transfer_third_time = 0.34F,
        .transfer_end_time = 0.50F,
        .transfer_hold_duration = 0.10F,
        .transfer_first_length = 0.24F,
        .transfer_second_length = 0.16F,
        .transfer_third_length = 0.17F,
        .transfer_final_length = 0.18F,
        .transfer_first_angle_body = -50.0F * BC_PI_F / 180.0F,
        .transfer_second_angle_body = -30.0F * BC_PI_F / 180.0F,
        .transfer_third_angle_body = -125.0F * BC_PI_F / 180.0F,
        .transfer_final_angle_body = -90.0F * BC_PI_F / 180.0F,
        .recovery_pitch_tolerance = 5.0F * BC_PI_F / 180.0F,
        .recovery_pitch_rate_tolerance = 0.50F,
        .recovery_roll_tolerance = 3.0F * BC_PI_F / 180.0F,
        .recovery_roll_rate_tolerance = 0.15F,
        .recovery_velocity_tolerance = 0.10F,
        .recovery_yaw_rate_tolerance = 0.10F,
        .recovery_leg_length_tolerance = 0.020F,
        .recovery_leg_speed_tolerance = 0.05F,
        .recovery_leg_angle_tolerance = 8.0F * BC_PI_F / 180.0F,
        .recovery_leg_angular_velocity_tolerance = 0.15F,
        .recovery_stable_duration = 0.05F,
        .recovery_timeout = 4.0F,
    };
}

void bc_step_task_init(
    bc_step_task_t *task,
    const bc_step_task_config_t *config
) {
    task->config = *config;
    bc_step_task_reset(task);
}

void bc_step_task_reset(bc_step_task_t *task) {
    task->state = BC_STEP_TASK_INACTIVE;
    task->state_elapsed_seconds = 0.0F;
    task->recovery_elapsed_seconds = 0.0F;
    task->impact_armed = 0U;
    task->command_rearm_required = 0U;
    task->recovery_timed_out = 0U;
    task->recovery_reference_captured = 0U;
    task->request = (bc_step_task_request_t){0};
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        task->transfer_start_length[side] = 0.0F;
        task->transfer_start_angle_body[side] = 0.0F;
        task->transfer_length_reference[side] = 0.0F;
        task->transfer_angle_reference[side] = 0.0F;
    }
    bc_condition_hold_reset(&task->impact_hold);
    bc_condition_hold_reset(&task->recovery_hold);
}

void bc_step_task_update(
    bc_step_task_t *task,
    const bc_state_machine_input_t *input,
    const bc_support_phase_state_t support_state,
    const float heading_error
) {
    task->request.recovery_entered = 0U;
    task->request.recovery_reference_capture = 0U;
    const float timestep_seconds = fmaxf(
        0.0F, input->timestep_seconds);
    task->state_elapsed_seconds += timestep_seconds;
    if (task->state == BC_STEP_TASK_RECOVER ||
        task->state == BC_STEP_TASK_RECOVER_LOCK) {
        task->recovery_elapsed_seconds += timestep_seconds;
    }
    switch (task->state) {
    case BC_STEP_TASK_INACTIVE:
        if (input->operator_command->task == BC_OPERATOR_TASK_NORMAL) {
            task->command_rearm_required = 0U;
        } else if (!task->command_rearm_required) {
            bc_step_task_set_state(task, BC_STEP_TASK_PREPARE);
        }
        break;

    case BC_STEP_TASK_PREPARE:
        if (input->operator_command->task != BC_OPERATOR_TASK_STEP_DOCK) {
            bc_step_task_set_state(task, BC_STEP_TASK_INACTIVE);
            task->impact_armed = 0U;
            bc_condition_hold_reset(&task->impact_hold);
            break;
        }

        task->impact_armed = bc_step_task_prepared(
            task, input, support_state, heading_error);
        if (bc_condition_hold_update(
                &task->impact_hold,
                task->impact_armed && bc_step_task_impact_candidate(
                    task, input->impact_observer),
                task->config.impact_confirm_duration,
                input->timestep_seconds)) {
            bc_step_task_set_state(task, BC_STEP_TASK_IMPACT_PASSIVE);
            task->impact_armed = 0U;
        }
        break;

    case BC_STEP_TASK_IMPACT_PASSIVE:
        if (bc_step_task_duration_elapsed(
                task, input, task->config.impact_passive_duration)) {
            bc_step_task_capture_transfer_start(task, input);
            bc_step_task_set_state(task, BC_STEP_TASK_TRANSFER);
        }
        break;

    case BC_STEP_TASK_TRANSFER:
        bc_step_task_update_transfer_reference(task);
        if (bc_step_task_duration_elapsed(
                task, input, task->config.transfer_end_time)) {
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                task->transfer_length_reference[side] =
                    task->config.transfer_final_length;
                task->transfer_angle_reference[side] =
                    task->config.transfer_final_angle_body;
            }
            bc_step_task_set_state(task, BC_STEP_TASK_TRANSFER_HOLD);
        }
        break;

    case BC_STEP_TASK_TRANSFER_HOLD:
        if (bc_step_task_duration_elapsed(
                task, input, task->config.transfer_hold_duration)) {
            bc_condition_hold_reset(&task->recovery_hold);
            task->recovery_elapsed_seconds = 0.0F;
            task->recovery_reference_captured = 0U;
            bc_step_task_set_state(task, BC_STEP_TASK_RECOVER);
            task->request.recovery_entered = 1U;
        }
        break;

    case BC_STEP_TASK_RECOVER:
        if (bc_condition_hold_update(
                &task->recovery_hold,
                bc_step_task_recovery_stable(task, input),
                task->config.recovery_stable_duration,
                input->timestep_seconds)) {
            bc_condition_hold_reset(&task->recovery_hold);
            task->recovery_reference_captured = 1U;
            bc_step_task_set_state(task, BC_STEP_TASK_RECOVER_LOCK);
            task->request.recovery_reference_capture = 1U;
        } else if (bc_step_task_recovery_timed_out(task, input)) {
            task->recovery_timed_out = 1U;
            bc_step_task_set_state(task, BC_STEP_TASK_RECOVERY_FAILED);
        }
        break;

    case BC_STEP_TASK_RECOVER_LOCK:
        if (bc_condition_hold_update(
                &task->recovery_hold,
                bc_step_task_recovery_stable(task, input),
                task->config.recovery_stable_duration,
                input->timestep_seconds)) {
            task->command_rearm_required = 1U;
            bc_step_task_set_state(task, BC_STEP_TASK_COMPLETE);
        } else if (bc_step_task_recovery_timed_out(task, input)) {
            task->recovery_timed_out = 1U;
            bc_step_task_set_state(task, BC_STEP_TASK_RECOVERY_FAILED);
        }
        break;

    case BC_STEP_TASK_COMPLETE:
        if (bc_step_task_duration_elapsed(
                task, input,
                fmaxf(input->timestep_seconds, 0.0F))) {
            bc_step_task_set_state(task, BC_STEP_TASK_INACTIVE);
        }
        break;

    case BC_STEP_TASK_RECOVERY_FAILED:
        break;
    }
    const uint8_t recovery_entered = task->request.recovery_entered;
    const uint8_t recovery_reference_capture =
        task->request.recovery_reference_capture;
    bc_step_task_prepare_request(task);
    task->request.recovery_entered = recovery_entered;
    task->request.recovery_reference_capture =
        recovery_reference_capture;
}

const char *bc_step_task_state_name(const bc_step_task_state_t state) {
    switch (state) {
    case BC_STEP_TASK_INACTIVE:
        return "inactive";
    case BC_STEP_TASK_PREPARE:
        return "prepare";
    case BC_STEP_TASK_IMPACT_PASSIVE:
        return "impact passive";
    case BC_STEP_TASK_TRANSFER:
        return "transfer";
    case BC_STEP_TASK_TRANSFER_HOLD:
        return "transfer hold";
    case BC_STEP_TASK_RECOVER:
        return "recover catch";
    case BC_STEP_TASK_RECOVER_LOCK:
        return "recover lock";
    case BC_STEP_TASK_COMPLETE:
        return "complete";
    case BC_STEP_TASK_RECOVERY_FAILED:
        return "recovery failed";
    }
    return "unknown";
}
