#include "balance/reference/yaw.h"

#include "balance/math_utils.h"

#include <math.h>

static float bc_yaw_reference_apply_deadband(
    const float value, const float deadband
) {
    return fabsf(value) <= deadband ? 0.0F : value;
}

void bc_yaw_reference_default_config(
    bc_yaw_reference_config_t *config
) {
    *config = (bc_yaw_reference_config_t){
        .command_deadband = 0.02F,
        .rate_ramp = {
            .value_limit = 4.0F * BC_PI_F,
            .rate_limit = 10.0F,
        },
    };
}

void bc_yaw_reference_init(
    bc_yaw_reference_t *yaw,
    const bc_yaw_reference_config_t *config
) {
    yaw->config = *config;
    bc_yaw_reference_reset(yaw);
}

void bc_yaw_reference_reset(bc_yaw_reference_t *yaw) {
    bc_reference_ramp_reset(&yaw->rate_ramp);
}

void bc_yaw_reference_start(
    bc_yaw_reference_t *yaw,
    const float current_yaw,
    bc_state_vector_t *reference
) {
    bc_yaw_reference_reset(yaw);
    reference->value[BC_STATE_PSI] = current_yaw;
    reference->value[BC_STATE_DPSI] = 0.0F;
}

void bc_yaw_reference_update(
    bc_yaw_reference_t *yaw,
    const float target_rate,
    const float timestep_seconds,
    bc_state_vector_t *reference
) {
    const float rate = bc_reference_ramp_update(
        &yaw->rate_ramp,
        &yaw->config.rate_ramp,
        bc_yaw_reference_apply_deadband(
            target_rate, yaw->config.command_deadband),
        timestep_seconds);

    if (timestep_seconds > 0.0F) {
        reference->value[BC_STATE_PSI] += rate * timestep_seconds;
    }
    reference->value[BC_STATE_DPSI] = rate;
}
