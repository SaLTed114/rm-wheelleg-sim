#include "balance/reference/yaw.h"

#include "balance/math_utils.h"

#include <math.h>
#include <stdio.h>

static int nearly_equal(const float actual, const float expected) {
    return fabsf(actual - expected) <= 1.0e-5F;
}

int main() {
    bc_yaw_reference_config_t config;
    bc_yaw_reference_t yaw;
    bc_state_vector_t state = {0};
    bc_state_vector_t reference = {0};

    bc_yaw_reference_default_config(&config);
    if (!nearly_equal(config.rate_limit, 1.5F * BC_PI_F)) {
        fputs("default yaw reference config is incorrect\n", stderr);
        return 1;
    }

    bc_yaw_reference_init(&yaw, &config);
    bc_yaw_reference_start(&yaw, -0.75F, 0.25F, &reference);
    if (reference.value[BC_STATE_PSI] != -0.75F ||
        reference.value[BC_STATE_DPSI] != 0.25F) {
        fputs("yaw reference start did not capture heading\n", stderr);
        return 1;
    }

    state.value[BC_STATE_PSI] = 7.0F;
    state.value[BC_STATE_DPSI] = 0.5F;
    bc_yaw_reference_update(
        &yaw, &state, 0.3F, 0.2F, &reference);
    if (!nearly_equal(reference.value[BC_STATE_PSI], 7.3F) ||
        !nearly_equal(reference.value[BC_STATE_DPSI], 0.7F)) {
        fputs("relative heading target was not reconstructed\n", stderr);
        return 1;
    }

    bc_yaw_reference_update(
        &yaw, &state, 2.0F * BC_PI_F + 0.25F,
        100.0F, &reference);
    if (!nearly_equal(reference.value[BC_STATE_PSI], 7.25F) ||
        !nearly_equal(
            reference.value[BC_STATE_DPSI], 1.5F * BC_PI_F)) {
        fputs("positive rate limit or angle wrap is incorrect\n", stderr);
        return 1;
    }

    bc_yaw_reference_update(
        &yaw, &state, -0.4F, -100.0F, &reference);
    if (!nearly_equal(reference.value[BC_STATE_PSI], 6.6F) ||
        !nearly_equal(
            reference.value[BC_STATE_DPSI], -1.5F * BC_PI_F)) {
        fputs("negative heading reference limit is incorrect\n", stderr);
        return 1;
    }

    bc_yaw_reference_reset(&yaw);
    if (!nearly_equal(yaw.config.rate_limit, 1.5F * BC_PI_F)) {
        fputs("yaw reference reset changed configuration\n", stderr);
        return 1;
    }

    return 0;
}
