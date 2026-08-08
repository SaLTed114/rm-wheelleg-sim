#include "balance/state_machine/forward_mode.h"

#include <math.h>

static uint8_t bc_forward_mode_is_stopped(
    const bc_forward_mode_t *forward,
    const uint8_t forward_motion_requested,
    const float reference_velocity,
    const float measured_velocity
) {
    return !forward_motion_requested &&
        reference_velocity == 0.0F &&
        fabsf(measured_velocity) <
            forward->config.stop_forward_velocity_tolerance;
}

static void bc_forward_mode_transition(
    bc_forward_mode_t *forward,
    const uint8_t forward_motion_requested,
    const float reference_velocity,
    const float measured_velocity,
    const float timestep_seconds
) {
    switch (forward->state) {
    case BC_FORWARD_IDLE:
        break;

    case BC_FORWARD_HOLD:
        if (forward_motion_requested) {
            forward->state = BC_FORWARD_VELOCITY;
            bc_condition_hold_reset(&forward->stopped_hold);
        }
        break;

    case BC_FORWARD_VELOCITY:
        if (!bc_condition_hold_update(
                &forward->stopped_hold,
                bc_forward_mode_is_stopped(
                    forward, forward_motion_requested,
                    reference_velocity, measured_velocity),
                forward->config.stop_duration,
                timestep_seconds)) break;

        forward->state = BC_FORWARD_HOLD;
        break;
    }
}

static void bc_forward_mode_action(
    const bc_forward_mode_t *forward,
    bc_control_command_t *output
) {
    switch (forward->state) {
    case BC_FORWARD_IDLE:
    case BC_FORWARD_HOLD:
        break;

    case BC_FORWARD_VELOCITY:
        output->disabled_state_feedback |=
            BC_STATE_FEEDBACK_MASK(BC_STATE_S);
        break;
    }
}

void bc_forward_mode_default_config(bc_forward_mode_config_t *config) {
    *config = (bc_forward_mode_config_t){
        .stop_forward_velocity_tolerance = 0.05F,
        .stop_duration = 0.25F,
    };
}

void bc_forward_mode_init(
    bc_forward_mode_t *forward,
    const bc_forward_mode_config_t *config
) {
    forward->config = *config;
    bc_forward_mode_reset(forward);
}

void bc_forward_mode_reset(bc_forward_mode_t *forward) {
    forward->state = BC_FORWARD_IDLE;
    bc_condition_hold_reset(&forward->stopped_hold);
}

void bc_forward_mode_start(bc_forward_mode_t *forward) {
    bc_forward_mode_reset(forward);
    forward->state = BC_FORWARD_HOLD;
}

void bc_forward_mode_update(
    bc_forward_mode_t *forward,
    const uint8_t forward_motion_requested,
    const float reference_velocity,
    const float measured_velocity,
    const float timestep_seconds,
    bc_control_command_t *output
) {
    bc_forward_mode_transition(
        forward, forward_motion_requested,
        reference_velocity, measured_velocity,
        timestep_seconds);
    bc_forward_mode_action(forward, output);
}

const char *bc_forward_state_name(const bc_forward_state_t state) {
    switch (state) {
    case BC_FORWARD_IDLE:
        return "idle";
    case BC_FORWARD_HOLD:
        return "hold";
    case BC_FORWARD_VELOCITY:
        return "velocity";
    }
    return "unknown";
}
