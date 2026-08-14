#include "balance/state_machine/step_task.h"
#include "balance/math_utils.h"

#include <math.h>
#include <stdio.h>

static int close_to(
    const float actual,
    const float expected,
    const float tolerance
) {
    return fabsf(actual - expected) <= tolerance;
}

static void advance(
    bc_step_task_t *task,
    const bc_state_machine_input_t *input,
    const int count
) {
    for (int index = 0; index < count; ++index) {
        bc_step_task_update(task, input, BC_SUPPORT_GROUND, 0.0F);
    }
}

static int advance_until_state_changes(
    bc_step_task_t *task,
    const bc_state_machine_input_t *input,
    const bc_step_task_state_t initial_state,
    const int maximum_count
) {
    for (int count = 1; count <= maximum_count; ++count) {
        bc_step_task_update(task, input, BC_SUPPORT_GROUND, 0.0F);
        if (task->state != initial_state) return count;
    }
    return 0;
}

int main(void) {
    bc_step_task_config_t config;
    bc_step_task_t task;
    bc_operator_command_t command = {0};
    bc_state_vector_t state = {0};
    bc_leg_kinematics_t leg[BC_SIDE_NUM] = {0};
    bc_support_force_output_t support[BC_SIDE_NUM] = {0};
    bc_impact_observer_output_t impact = {0};
    bc_state_machine_input_t input = {
        .operator_command = &command,
        .state = &state,
        .leg = leg,
        .support_force = support,
        .impact_observer = &impact,
        .wheel_odometry_velocity = 0.5F,
        .wheel_velocity_reliable = 1U,
        .timestep_seconds = 0.001F,
    };

    bc_step_task_default_config(&config);
    if (config.prepare_leg_length != 0.38F ||
        config.leg_length_tolerance != 0.020F ||
        !close_to(
            config.alignment_tolerance,
            5.0F * BC_PI_F / 180.0F, 1.0e-6F) ||
        config.minimum_forward_velocity != 0.3F ||
        config.leg_rate_delta_threshold != 0.5F ||
        config.wheel_imu_mismatch_threshold != 0.12F ||
        config.impact_confirm_duration != 0.002F ||
        config.impact_passive_duration != 0.001F ||
        config.transfer_end_time != 0.50F ||
        config.transfer_hold_duration != 0.10F ||
        config.transfer_final_length != 0.18F ||
        !close_to(
            config.transfer_final_angle_body,
            -0.5F * BC_PI_F, 1.0e-6F) ||
        !close_to(
            config.recovery_pitch_tolerance,
            5.0F * BC_PI_F / 180.0F, 1.0e-6F) ||
        config.recovery_pitch_rate_tolerance != 0.50F ||
        config.recovery_stable_duration != 0.05F ||
        config.recovery_timeout != 4.0F) {
        fputs("default step task config is incorrect\n", stderr);
        return 1;
    }
    const int transfer_hold_steps = (int)lroundf(
        config.transfer_hold_duration / input.timestep_seconds);
    const int recovery_stable_maximum_steps = (int)ceilf(
        config.recovery_stable_duration / input.timestep_seconds) + 1;

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        support[side].valid = 1U;
        support[side].state = BC_CONTACT_GROUND;
        leg[side].length = config.prepare_leg_length;
        leg[side].angle_body = -60.0F * BC_PI_F / 180.0F;
    }
    bc_step_task_init(&task, &config);
    command.task = BC_OPERATOR_TASK_STEP_DOCK;
    bc_step_task_update(&task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_PREPARE || task.impact_armed ||
        !task.request.active || !task.request.force_front_alignment ||
        task.request.control_mode != BC_STEP_CONTROL_NORMAL ||
        task.request.working_leg_length != config.prepare_leg_length) {
        fputs("step task did not enter prepare\n", stderr);
        return 1;
    }

    command.task = BC_OPERATOR_TASK_NORMAL;
    bc_step_task_update(&task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_INACTIVE || task.request.active) {
        fputs("prepare did not allow cancellation\n", stderr);
        return 1;
    }

    command.task = BC_OPERATOR_TASK_STEP_DOCK;
    bc_step_task_update(&task, &input, BC_SUPPORT_GROUND, 0.0F);
    impact.valid = 1U;
    impact.window[BC_IMPACT_WINDOW_SHORT].valid = 1U;
    impact.window[BC_IMPACT_WINDOW_SHORT].leg_rate_delta[BC_L] = 0.6F;
    impact.window[BC_IMPACT_WINDOW_SHORT].wheel_imu_delta_mismatch = 0.13F;
    bc_step_task_update(&task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_PREPARE ||
        !close_to(task.impact_hold.elapsed_seconds, 0.001F, 1.0e-7F)) {
        fputs("impact confirmation timing started incorrectly\n", stderr);
        return 1;
    }
    bc_step_task_update(&task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_IMPACT_PASSIVE ||
        task.request.control_mode != BC_STEP_CONTROL_PASSIVE) {
        fputs("fused impact did not enter passive control\n", stderr);
        return 1;
    }

    bc_step_task_update(&task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_TRANSFER ||
        task.request.control_mode != BC_STEP_CONTROL_TRANSFER ||
        !close_to(
            task.transfer_length_reference[BC_L],
            config.prepare_leg_length, 1.0e-6F)) {
        fputs("passive cycle did not enter transfer from measured pose\n",
              stderr);
        return 1;
    }

    advance(&task, &input, 80);
    if (task.state != BC_STEP_TASK_TRANSFER ||
        !close_to(
            task.transfer_length_reference[BC_L],
            config.transfer_first_length, 1.0e-5F) ||
        !close_to(
            task.transfer_angle_reference[BC_L],
            config.transfer_first_angle_body, 1.0e-5F)) {
        fputs("first transfer knot is incorrect\n", stderr);
        return 1;
    }
    advance(&task, &input, 80);
    if (!close_to(
            task.transfer_length_reference[BC_L],
            config.transfer_second_length, 1.0e-5F) ||
        !close_to(
            task.transfer_angle_reference[BC_L],
            config.transfer_second_angle_body, 1.0e-5F)) {
        fputs("second transfer knot is incorrect\n", stderr);
        return 1;
    }
    advance(&task, &input, 180);
    if (!close_to(
            task.transfer_length_reference[BC_L],
            config.transfer_third_length, 1.0e-5F) ||
        !close_to(
            task.transfer_angle_reference[BC_L],
            config.transfer_third_angle_body, 1.0e-5F)) {
        fputs("third transfer knot is incorrect\n", stderr);
        return 1;
    }
    advance(&task, &input, 160);
    if (task.state != BC_STEP_TASK_TRANSFER_HOLD ||
        task.request.control_mode != BC_STEP_CONTROL_TRANSFER ||
        task.transfer_length_reference[BC_L] !=
            config.transfer_final_length ||
        task.transfer_angle_reference[BC_L] !=
            config.transfer_final_angle_body) {
        fputs("transfer did not enter final hold\n", stderr);
        return 1;
    }

    advance(&task, &input, transfer_hold_steps);
    if (task.state != BC_STEP_TASK_RECOVER ||
        task.request.control_mode != BC_STEP_CONTROL_RECOVER ||
        !task.request.recovery_entered ||
        !task.request.suppress_position_heading_feedback ||
        task.recovery_reference_captured) {
        fputs("transfer hold did not enter recovery\n", stderr);
        return 1;
    }

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        leg[side].length = config.transfer_final_length;
        leg[side].length_velocity = 0.0F;
        leg[side].angle_body = config.transfer_final_angle_body;
        leg[side].angular_velocity = 1.0F;
    }
    state.value[BC_STATE_DTHETA_R] =
        config.recovery_leg_angular_velocity_tolerance + 0.01F;
    bc_step_task_update(&task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_RECOVER ||
        task.recovery_hold.elapsed_seconds != 0.0F) {
        fputs("world leg angular velocity did not gate recovery\n", stderr);
        return 1;
    }
    state.value[BC_STATE_DTHETA_R] = 0.0F;
    if (!advance_until_state_changes(
            &task, &input, BC_STEP_TASK_RECOVER,
            recovery_stable_maximum_steps)) {
        fputs("stable recovery did not leave catch state\n", stderr);
        return 1;
    }
    if (task.state != BC_STEP_TASK_RECOVER_LOCK ||
        !task.request.recovery_reference_capture ||
        task.request.suppress_position_heading_feedback ||
        !task.recovery_reference_captured ||
        task.command_rearm_required || task.recovery_timed_out) {
        fputs("stable recovery did not request reference capture\n", stderr);
        return 1;
    }
    if (!advance_until_state_changes(
            &task, &input, BC_STEP_TASK_RECOVER_LOCK,
            recovery_stable_maximum_steps)) {
        fputs("locked recovery did not leave lock state\n", stderr);
        return 1;
    }
    if (task.state != BC_STEP_TASK_COMPLETE ||
        !task.command_rearm_required || task.recovery_timed_out) {
        fputs("locked recovery did not complete\n", stderr);
        return 1;
    }
    bc_step_task_update(&task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_INACTIVE ||
        !task.command_rearm_required || task.request.active) {
        fputs("complete did not return to disarmed inactive\n", stderr);
        return 1;
    }
    bc_step_task_update(&task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_INACTIVE) {
        fputs("continuous STEP command retriggered the task\n", stderr);
        return 1;
    }
    command.task = BC_OPERATOR_TASK_NORMAL;
    bc_step_task_update(&task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.command_rearm_required) {
        fputs("NORMAL command did not rearm the task\n", stderr);
        return 1;
    }
    command.task = BC_OPERATOR_TASK_STEP_DOCK;
    bc_step_task_update(&task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_PREPARE) {
        fputs("rearmed STEP command did not start prepare\n", stderr);
        return 1;
    }

    bc_step_task_reset(&task);
    task.state = BC_STEP_TASK_RECOVER;
    task.recovery_elapsed_seconds = config.recovery_timeout - 0.001F;
    input.wheel_velocity_reliable = 0U;
    bc_step_task_update(&task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_RECOVERY_FAILED ||
        !task.recovery_timed_out ||
        task.request.control_mode != BC_STEP_CONTROL_RECOVER ||
        !task.request.suppress_position_heading_feedback) {
        fputs("catch timeout did not preserve unanchored recovery\n",
              stderr);
        return 1;
    }

    bc_step_task_reset(&task);
    task.state = BC_STEP_TASK_RECOVER_LOCK;
    task.recovery_elapsed_seconds = config.recovery_timeout - 0.001F;
    task.recovery_reference_captured = 1U;
    bc_step_task_update(&task, &input, BC_SUPPORT_GROUND, 0.0F);
    if (task.state != BC_STEP_TASK_RECOVERY_FAILED ||
        !task.recovery_timed_out ||
        task.request.suppress_position_heading_feedback) {
        fputs("lock timeout did not preserve captured recovery\n", stderr);
        return 1;
    }
    return 0;
}
