#include "balance/reference/forward.h"

#include <math.h>

static float bc_forward_reference_apply_deadband(
    const float value, const float deadband
) {
    return fabsf(value) <= deadband ? 0.0F : value;
}

void bc_forward_reference_default_config(
    bc_forward_reference_config_t *config
) {
    *config = (bc_forward_reference_config_t){
        .command_deadband = 0.01F,
        .velocity_ramp = {
            .value_limit = 3.0F,
            .rate_limit = 5.0F,
        },
    };
}

void bc_forward_reference_init(
    bc_forward_reference_t *forward,
    const bc_forward_reference_config_t *config
) {
    forward->config = *config;
    bc_forward_reference_reset(forward);
}

void bc_forward_reference_reset(bc_forward_reference_t *forward) {
    bc_reference_ramp_reset(&forward->velocity_ramp);
}

void bc_forward_reference_start(
    bc_forward_reference_t *forward,
    const float current_position,
    bc_state_vector_t *reference
) {
    bc_forward_reference_reset(forward);
    reference->value[BC_STATE_S] = current_position;
    reference->value[BC_STATE_DS] = 0.0F;
}

uint8_t bc_forward_reference_requested(
    const bc_forward_reference_t *forward,
    const float target_velocity
) {
    return bc_forward_reference_apply_deadband(
        target_velocity, forward->config.command_deadband) != 0.0F;
}

void bc_forward_reference_update(
    bc_forward_reference_t *forward,
    const float target_velocity,
    const float timestep_seconds,
    bc_state_vector_t *reference
) {
    const float velocity = bc_reference_ramp_update(
        &forward->velocity_ramp,
        &forward->config.velocity_ramp,
        bc_forward_reference_apply_deadband(
            target_velocity, forward->config.command_deadband),
        timestep_seconds);

    if (timestep_seconds > 0.0F) {
        reference->value[BC_STATE_S] += velocity * timestep_seconds;
    }
    reference->value[BC_STATE_DS] = velocity;
}
