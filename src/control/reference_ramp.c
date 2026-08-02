#include "balance/reference_ramp.h"

#include "balance/math_utils.h"

void bc_reference_ramp_reset(bc_reference_ramp_t *ramp) {
    ramp->value = 0.0F;
}

float bc_reference_ramp_update(
    bc_reference_ramp_t *ramp,
    const bc_reference_ramp_config_t *config,
    const float target,
    const float timestep_seconds
) {
    if (timestep_seconds <= 0.0F) return ramp->value;

    const float limited_target = bc_clampf(
        target, -config->value_limit, +config->value_limit);
    const float maximum_delta = config->rate_limit * timestep_seconds;
    ramp->value += bc_clampf(
        limited_target - ramp->value,
        -maximum_delta, +maximum_delta);
    return ramp->value;
}
