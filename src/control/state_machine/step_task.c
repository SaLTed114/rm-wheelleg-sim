#include "balance/state_machine/step_task.h"

#include "balance/math_utils.h"

#include <math.h>
#include <stddef.h>

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
    task->impact_armed = 0U;
    bc_condition_hold_reset(&task->impact_hold);
}

void bc_step_task_update(
    bc_step_task_t *task,
    const bc_state_machine_input_t *input,
    const bc_support_phase_state_t support_state,
    const float heading_error
) {
    switch (task->state) {
    case BC_STEP_TASK_INACTIVE:
        if (input->operator_command->task == BC_OPERATOR_TASK_STEP_DOCK) {
            task->state = BC_STEP_TASK_PREPARE;
        }
        break;

    case BC_STEP_TASK_PREPARE:
        if (input->operator_command->task != BC_OPERATOR_TASK_STEP_DOCK) {
            bc_step_task_reset(task);
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
            task->state = BC_STEP_TASK_IMPACT_PASSIVE;
            task->impact_armed = 0U;
        }
        break;

    case BC_STEP_TASK_IMPACT_PASSIVE:
        break;
    }
}

const char *bc_step_task_state_name(const bc_step_task_state_t state) {
    switch (state) {
    case BC_STEP_TASK_INACTIVE:
        return "inactive";
    case BC_STEP_TASK_PREPARE:
        return "prepare";
    case BC_STEP_TASK_IMPACT_PASSIVE:
        return "impact passive";
    }
    return "unknown";
}
