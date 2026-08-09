#include "balance/state_machine/forward_mode.h"

#include <stdio.h>
#include <string.h>

int main() {
    bc_forward_mode_config_t config;
    bc_forward_mode_t forward;
    bc_control_command_t output = {0};

    bc_forward_mode_default_config(&config);
    if (config.stop_forward_velocity_tolerance != 0.05F ||
        config.stop_wheel_velocity_tolerance != 0.05F ||
        config.stop_duration != 0.25F) {
        fputs("default forward mode config is incorrect\n", stderr);
        return 1;
    }

    bc_forward_mode_init(&forward, &config);
    if (forward.state != BC_FORWARD_IDLE ||
        forward.stopped_hold.elapsed_seconds != 0.0F) {
        fputs("forward mode did not initialize idle\n", stderr);
        return 1;
    }

    bc_forward_mode_start(&forward);
    bc_forward_mode_update(
        &forward, 0U, 0.0F, 1.0F, 1.0F, 1U, 0.1F, &output);
    if (forward.state != BC_FORWARD_HOLD ||
        output.disabled_state_feedback != 0U) {
        fputs("forward mode did not start in hold\n", stderr);
        return 1;
    }

    memset(&output, 0, sizeof(output));
    bc_forward_mode_update(
        &forward, 1U, 0.0F, 0.0F, 0.0F, 1U, 0.1F, &output);
    if (forward.state != BC_FORWARD_VELOCITY ||
        output.disabled_state_feedback !=
            BC_STATE_FEEDBACK_MASK(BC_STATE_S)) {
        fputs("forward request did not enter velocity mode\n", stderr);
        return 1;
    }

    memset(&output, 0, sizeof(output));
    bc_forward_mode_update(
        &forward, 0U, 0.5F, 0.0F, 0.0F, 1U, 0.1F, &output);
    if (forward.state != BC_FORWARD_VELOCITY ||
        forward.stopped_hold.elapsed_seconds != 0.0F) {
        fputs("nonzero reference velocity was treated as stopped\n", stderr);
        return 1;
    }

    bc_forward_mode_update(
        &forward, 0U, 0.0F,
        config.stop_forward_velocity_tolerance,
        0.0F, 1U, 0.1F, &output);
    if (forward.stopped_hold.elapsed_seconds != 0.0F) {
        fputs("measured threshold was treated as stopped\n", stderr);
        return 1;
    }

    bc_forward_mode_update(
        &forward, 0U, 0.0F, 0.0F, 0.0F, 0U, 0.1F, &output);
    if (forward.stopped_hold.elapsed_seconds != 0.0F) {
        fputs("unreliable wheel velocity was treated as stopped\n", stderr);
        return 1;
    }

    bc_forward_mode_update(
        &forward, 0U, 0.0F, 0.0F,
        config.stop_wheel_velocity_tolerance,
        1U, 0.1F, &output);
    if (forward.stopped_hold.elapsed_seconds != 0.0F) {
        fputs("moving wheel velocity was treated as stopped\n", stderr);
        return 1;
    }

    bc_forward_mode_update(
        &forward, 0U, 0.0F, 0.0F, 0.0F, 1U, 0.1F, &output);
    bc_forward_mode_update(
        &forward, 1U, 0.0F, 0.0F, 0.0F, 1U, 0.1F, &output);
    if (forward.state != BC_FORWARD_VELOCITY ||
        forward.stopped_hold.elapsed_seconds != 0.0F) {
        fputs("new forward request did not cancel stop hold\n", stderr);
        return 1;
    }

    for (int step = 0; step < 3; ++step) {
        memset(&output, 0, sizeof(output));
        bc_forward_mode_update(
            &forward, 0U, 0.0F, 0.0F, 0.0F, 1U, 0.1F, &output);
    }
    if (forward.state != BC_FORWARD_HOLD ||
        output.disabled_state_feedback != 0U) {
        fputs("stable forward stop did not return to hold\n", stderr);
        return 1;
    }

    bc_forward_mode_reset(&forward);
    if (forward.state != BC_FORWARD_IDLE ||
        forward.stopped_hold.elapsed_seconds != 0.0F ||
        strcmp(bc_forward_state_name(BC_FORWARD_IDLE), "idle") != 0 ||
        strcmp(bc_forward_state_name(BC_FORWARD_HOLD), "hold") != 0 ||
        strcmp(
            bc_forward_state_name(BC_FORWARD_VELOCITY), "velocity") != 0) {
        fputs("forward reset or state names are incorrect\n", stderr);
        return 1;
    }

    return 0;
}
