#include "balance/state_machine/drive.h"

#include <stdio.h>
#include <string.h>

int main() {
    bc_drive_config_t config;
    bc_drive_t drive;
    bc_control_command_t output = {0};

    bc_drive_default_config(&config);
    if (config.stop_forward_velocity_tolerance != 0.05F ||
        config.stop_duration != 0.25F) {
        fputs("default drive config is incorrect\n", stderr);
        return 1;
    }

    bc_drive_init(&drive, &config);
    if (drive.state != BC_DRIVE_IDLE ||
        drive.stopped_hold.elapsed_seconds != 0.0F) {
        fputs("drive did not initialize idle\n", stderr);
        return 1;
    }

    bc_drive_start(&drive);
    bc_drive_update(&drive, 0U, 0.0F, 1.0F, 0.1F, &output);
    if (drive.state != BC_DRIVE_HOLD ||
        output.disabled_state_feedback != 0U) {
        fputs("drive did not start in hold\n", stderr);
        return 1;
    }

    memset(&output, 0, sizeof(output));
    bc_drive_update(&drive, 1U, 0.0F, 0.0F, 0.1F, &output);
    if (drive.state != BC_DRIVE_DRIVING ||
        output.disabled_state_feedback !=
            BC_STATE_FEEDBACK_MASK(BC_STATE_S)) {
        fputs("forward request did not enter driving\n", stderr);
        return 1;
    }

    memset(&output, 0, sizeof(output));
    bc_drive_update(&drive, 0U, 0.5F, 0.0F, 0.1F, &output);
    if (drive.state != BC_DRIVE_DRIVING ||
        drive.stopped_hold.elapsed_seconds != 0.0F) {
        fputs("nonzero reference velocity was treated as stopped\n", stderr);
        return 1;
    }

    bc_drive_update(
        &drive, 0U, 0.0F,
        config.stop_forward_velocity_tolerance,
        0.1F, &output);
    if (drive.stopped_hold.elapsed_seconds != 0.0F) {
        fputs("measured threshold was treated as stopped\n", stderr);
        return 1;
    }

    bc_drive_update(&drive, 0U, 0.0F, 0.0F, 0.1F, &output);
    bc_drive_update(&drive, 1U, 0.0F, 0.0F, 0.1F, &output);
    if (drive.state != BC_DRIVE_DRIVING ||
        drive.stopped_hold.elapsed_seconds != 0.0F) {
        fputs("new forward request did not cancel stop hold\n", stderr);
        return 1;
    }

    for (int step = 0; step < 3; ++step) {
        memset(&output, 0, sizeof(output));
        bc_drive_update(&drive, 0U, 0.0F, 0.0F, 0.1F, &output);
    }
    if (drive.state != BC_DRIVE_HOLD ||
        output.disabled_state_feedback != 0U) {
        fputs("stable forward stop did not return to hold\n", stderr);
        return 1;
    }

    drive.state = BC_DRIVE_SPIN;
    memset(&output, 0, sizeof(output));
    bc_drive_update(&drive, 0U, 0.0F, 0.0F, 0.1F, &output);
    if (drive.state != BC_DRIVE_HOLD ||
        output.disabled_state_feedback != 0U) {
        fputs("spin placeholder did not fall back safely\n", stderr);
        return 1;
    }

    bc_drive_reset(&drive);
    if (drive.state != BC_DRIVE_IDLE ||
        drive.stopped_hold.elapsed_seconds != 0.0F ||
        strcmp(bc_drive_state_name(BC_DRIVE_IDLE), "idle") != 0 ||
        strcmp(bc_drive_state_name(BC_DRIVE_HOLD), "hold") != 0 ||
        strcmp(bc_drive_state_name(BC_DRIVE_DRIVING), "drive") != 0 ||
        strcmp(bc_drive_state_name(BC_DRIVE_SPIN), "spin") != 0) {
        fputs("drive reset or state names are incorrect\n", stderr);
        return 1;
    }

    return 0;
}
