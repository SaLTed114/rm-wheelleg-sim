#include "balance/reference/yaw.h"

#include "balance/math_utils.h"

void bc_yaw_reference_default_config(
    bc_yaw_reference_config_t *config
) {
    *config = (bc_yaw_reference_config_t){
        .rate_limit = 1.5F * BC_PI_F,
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
    (void)yaw;
}

void bc_yaw_reference_start(
    bc_yaw_reference_t *yaw,
    const float current_yaw,
    const float current_yaw_rate,
    bc_state_vector_t *reference
) {
    bc_yaw_reference_reset(yaw);
    reference->value[BC_STATE_PSI] = current_yaw;
    reference->value[BC_STATE_DPSI] = current_yaw_rate;
}

void bc_yaw_reference_update(
    const bc_yaw_reference_t *yaw,
    const bc_state_vector_t *state,
    const float heading_error,
    const float relative_yaw_rate,
    bc_state_vector_t *reference
) {
    reference->value[BC_STATE_PSI] =
        state->value[BC_STATE_PSI] + bc_wrap_anglef(heading_error);
    reference->value[BC_STATE_DPSI] = bc_clampf(
        state->value[BC_STATE_DPSI] + relative_yaw_rate,
        -yaw->config.rate_limit, +yaw->config.rate_limit);
}
