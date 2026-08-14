#include "balance/state_machine/step_task.h"
#include "balance/math_utils.h"

#include <math.h>
#include <stdio.h>

int main(void) {
    bc_step_task_config_t config;
    bc_step_task_t task;
    bc_operator_command_t command = {0};
    bc_leg_kinematics_t leg[BC_SIDE_NUM] = {0};
    bc_impact_observer_output_t impact = {0};
    bc_state_machine_input_t input = {
        .operator_command = &command,
        .leg = leg,
        .impact_observer = &impact,
        .wheel_odometry_velocity = 0.5F,
        .wheel_velocity_reliable = 1U,
        .timestep_seconds = 0.001F,
    };

    bc_step_task_default_config(&config);
    if (config.prepare_leg_length != 0.38F ||
        config.leg_length_tolerance != 0.020F ||
        fabsf(config.alignment_tolerance -
              5.0F * BC_PI_F / 180.0F) > 1.0e-6F ||
        config.minimum_forward_velocity != 0.3F ||
        config.leg_rate_delta_threshold != 0.5F ||
        config.wheel_imu_mismatch_threshold != 0.12F ||
        config.impact_confirm_duration != 0.002F) {
        fputs("default step task config is incorrect\n", stderr);
        return 1;
    }
    bc_step_task_init(&task, &config);
    command.task = BC_OPERATOR_TASK_STEP_DOCK;
    bc_step_task_update(
        &task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_PREPARE || task.impact_armed) {
        fputs("step task did not enter prepare\n", stderr);
        return 1;
    }

    command.task = BC_OPERATOR_TASK_NORMAL;
    bc_step_task_update(
        &task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_INACTIVE) {
        fputs("prepare did not allow cancellation\n", stderr);
        return 1;
    }

    command.task = BC_OPERATOR_TASK_STEP_DOCK;
    bc_step_task_update(
        &task, &input, BC_SUPPORT_GROUND, 0.0F);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        leg[side].length = config.prepare_leg_length;
    }
    impact.valid = 1U;
    impact.window[BC_IMPACT_WINDOW_SHORT].valid = 1U;
    impact.window[BC_IMPACT_WINDOW_SHORT].leg_rate_delta[BC_L] = 0.6F;
    impact.window[BC_IMPACT_WINDOW_SHORT].wheel_imu_delta_mismatch = 0.0F;
    bc_step_task_update(
        &task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (!task.impact_armed ||
        task.state != BC_STEP_TASK_PREPARE ||
        task.impact_hold.elapsed_seconds != 0.0F) {
        fputs("single impact feature triggered the task\n", stderr);
        return 1;
    }

    impact.window[BC_IMPACT_WINDOW_SHORT].wheel_imu_delta_mismatch = 0.13F;
    bc_step_task_update(
        &task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_PREPARE ||
        fabsf(task.impact_hold.elapsed_seconds - 0.001F) > 1.0e-7F) {
        fputs("impact confirmation timing started incorrectly\n", stderr);
        return 1;
    }
    bc_step_task_update(
        &task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_IMPACT_PASSIVE) {
        fputs("fused impact did not enter passive state\n", stderr);
        return 1;
    }

    command.task = BC_OPERATOR_TASK_NORMAL;
    bc_step_task_update(
        &task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_IMPACT_PASSIVE) {
        fputs("passive state was not latched\n", stderr);
        return 1;
    }
    bc_step_task_reset(&task);
    if (task.state != BC_STEP_TASK_INACTIVE ||
        task.impact_hold.elapsed_seconds != 0.0F) {
        fputs("step task reset is incorrect\n", stderr);
        return 1;
    }
    return 0;
}
