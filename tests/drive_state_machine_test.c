#include "balance/state_machine/drive.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int nearly_equal(const float actual, const float expected) {
    return fabsf(actual - expected) <= 1.0e-6F;
}

int main() {
    bc_drive_config_t config;
    bc_drive_t drive;
    bc_operator_command_t operator_command = {0};
    bc_state_vector_t state = {0};
    bc_state_vector_t reference = {0};
    bc_control_command_t output = {0};
    bc_state_machine_input_t input = {
        .operator_command = &operator_command,
        .state = &state,
        .wheel_odometry_velocity = 0.0F,
        .timestep_seconds = 0.1F,
    };

    bc_drive_default_config(&config);
    if (config.forward_command_deadband != 0.01F ||
        config.yaw_command_deadband != 0.02F ||
        config.stop_wheel_velocity_tolerance != 0.05F ||
        config.stop_forward_velocity_tolerance != 0.05F ||
        config.stop_yaw_rate_tolerance != 0.10F ||
        config.stop_duration != 0.25F) {
        fputs("default drive config is incorrect\n", stderr);
        return 1;
    }

    bc_drive_init(&drive, &config);
    if (drive.state != BC_DRIVE_IDLE ||
        drive.forward_velocity_ramp.value != 0.0F ||
        drive.yaw_rate_ramp.value != 0.0F) {
        fputs("drive did not initialize idle\n", stderr);
        return 1;
    }

    state.value[BC_STATE_S] = 1.25F;
    state.value[BC_STATE_DS] = 0.4F;
    state.value[BC_STATE_PSI] = -0.75F;
    state.value[BC_STATE_DPSI] = -0.2F;
    bc_drive_start(&drive, &input, &reference);
    bc_drive_update(&drive, &input, &reference, &output);
    if (drive.state != BC_DRIVE_PARKED ||
        reference.value[BC_STATE_S] != 1.25F ||
        reference.value[BC_STATE_DS] != 0.0F ||
        reference.value[BC_STATE_PSI] != -0.75F ||
        reference.value[BC_STATE_DPSI] != 0.0F ||
        output.disabled_state_feedback != 0U) {
        fputs("parked drive did not capture its pose\n", stderr);
        return 1;
    }

    operator_command.forward_velocity = config.forward_command_deadband;
    operator_command.yaw_rate = -config.yaw_command_deadband;
    memset(&output, 0, sizeof(output));
    bc_drive_update(&drive, &input, &reference, &output);
    if (drive.state != BC_DRIVE_PARKED) {
        fputs("deadband command left parked state\n", stderr);
        return 1;
    }

    operator_command.forward_velocity = 10.0F;
    operator_command.yaw_rate = 20.0F;
    memset(&output, 0, sizeof(output));
    bc_drive_update(&drive, &input, &reference, &output);
    if (drive.state != BC_DRIVE_DRIVING ||
        !nearly_equal(reference.value[BC_STATE_S], 1.30F) ||
        !nearly_equal(reference.value[BC_STATE_DS], 0.5F) ||
        !nearly_equal(reference.value[BC_STATE_PSI], -0.60F) ||
        !nearly_equal(reference.value[BC_STATE_DPSI], 1.5F) ||
        output.disabled_state_feedback !=
            BC_STATE_FEEDBACK_MASK(BC_STATE_S)) {
        fputs("motion command did not enter driving with ramped targets\n", stderr);
        return 1;
    }

    operator_command.forward_velocity = 0.0F;
    operator_command.yaw_rate = 0.0F;
    memset(&output, 0, sizeof(output));
    bc_drive_update(&drive, &input, &reference, &output);
    if (drive.state != BC_DRIVE_DRIVING ||
        drive.forward_velocity_ramp.value != 0.0F ||
        drive.yaw_rate_ramp.value != 0.0F) {
        fputs("driving ramps did not return to zero\n", stderr);
        return 1;
    }

    input.wheel_odometry_velocity = 0.05F;
    bc_drive_update(&drive, &input, &reference, &output);
    bc_drive_update(&drive, &input, &reference, &output);
    input.wheel_odometry_velocity = 0.0F;
    state.value[BC_STATE_DS] = 0.05F;
    bc_drive_update(&drive, &input, &reference, &output);
    state.value[BC_STATE_DS] = 0.0F;
    state.value[BC_STATE_DPSI] = 0.10F;
    bc_drive_update(&drive, &input, &reference, &output);
    if (drive.state != BC_DRIVE_DRIVING ||
        drive.stopped_hold.elapsed_seconds != 0.0F) {
        fputs("stop detection ignored measured motion\n", stderr);
        return 1;
    }

    state.value[BC_STATE_DPSI] = 0.0F;
    state.value[BC_STATE_S] = 2.0F;
    state.value[BC_STATE_PSI] = 0.4F;
    memset(&output, 0, sizeof(output));
    bc_drive_update(&drive, &input, &reference, &output);
    memset(&output, 0, sizeof(output));
    bc_drive_update(&drive, &input, &reference, &output);
    memset(&output, 0, sizeof(output));
    bc_drive_update(&drive, &input, &reference, &output);
    if (drive.state != BC_DRIVE_PARKED ||
        reference.value[BC_STATE_S] != 2.0F ||
        reference.value[BC_STATE_PSI] != 0.4F ||
        reference.value[BC_STATE_DS] != 0.0F ||
        reference.value[BC_STATE_DPSI] != 0.0F ||
        output.disabled_state_feedback != 0U) {
        fputs("stable stop did not park and capture the pose\n", stderr);
        return 1;
    }

    bc_drive_reset(&drive);
    if (drive.state != BC_DRIVE_IDLE ||
        drive.stopped_hold.elapsed_seconds != 0.0F ||
        strcmp(bc_drive_state_name(BC_DRIVE_IDLE), "idle") != 0 ||
        strcmp(bc_drive_state_name(BC_DRIVE_PARKED), "parked") != 0 ||
        strcmp(bc_drive_state_name(BC_DRIVE_DRIVING), "driving") != 0) {
        fputs("drive reset or state names are incorrect\n", stderr);
        return 1;
    }

    return 0;
}
