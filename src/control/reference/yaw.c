#include "balance/reference/yaw.h"

#include "balance/math_utils.h"

void bc_yaw_reference_default_config(
    bc_yaw_reference_config_t *config
) {
    *config = (bc_yaw_reference_config_t){
        .rate_limit         = 1.5F * BC_PI_F,
        .acceleration_limit = 10.0F,
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
    yaw->previous_rate = 0.0F;
    yaw->acceleration_reference = 0.0F;
}

void bc_yaw_reference_start(
    bc_yaw_reference_t *yaw,
    const float current_yaw,
    const float current_yaw_rate,
    bc_state_vector_t *reference
) {
    bc_yaw_reference_reset(yaw);
    yaw->previous_rate = current_yaw_rate;
    reference->value[BC_STATE_PSI] = current_yaw;
    reference->value[BC_STATE_DPSI] = current_yaw_rate;
}

void bc_yaw_reference_update(
    bc_yaw_reference_t *yaw,
    const bc_state_vector_t *state,
    const float heading_error,
    const float relative_yaw_rate,
    const float timestep_seconds,
    bc_state_vector_t *reference
) {
    reference->value[BC_STATE_PSI] =
        state->value[BC_STATE_PSI] + bc_wrap_anglef(heading_error);
    const float target_rate = bc_clampf(
        state->value[BC_STATE_DPSI] + relative_yaw_rate,
        -yaw->config.rate_limit, +yaw->config.rate_limit);
    reference->value[BC_STATE_DPSI] = target_rate;

    yaw->acceleration_reference = 0.0F;
    if (timestep_seconds > 0.0F) {
        yaw->acceleration_reference = bc_clampf(
            (target_rate - yaw->previous_rate) / timestep_seconds,
            -yaw->config.acceleration_limit,
            +yaw->config.acceleration_limit);
    }
    yaw->previous_rate = target_rate;
}
