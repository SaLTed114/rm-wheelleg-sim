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
    bc_state_vector_t reference = {0};

    bc_yaw_reference_default_config(&config);
    if (config.command_deadband != 0.02F ||
        !nearly_equal(config.rate_ramp.value_limit, 4.0F * BC_PI_F) ||
        config.rate_ramp.rate_limit != 10.0F) {
        fputs("default yaw reference config is incorrect\n", stderr);
        return 1;
    }

    bc_yaw_reference_init(&yaw, &config);
    if (yaw.rate_ramp.value != 0.0F) {
        fputs("yaw reference did not initialize at rest\n", stderr);
        return 1;
    }

    reference.value[BC_STATE_DPSI] = 3.0F;
    bc_yaw_reference_start(&yaw, -0.75F, &reference);
    if (reference.value[BC_STATE_PSI] != -0.75F ||
        reference.value[BC_STATE_DPSI] != 0.0F) {
        fputs("yaw reference start did not capture heading\n", stderr);
        return 1;
    }

    bc_yaw_reference_update(
        &yaw, -config.command_deadband, 0.1F, &reference);
    if (reference.value[BC_STATE_PSI] != -0.75F ||
        reference.value[BC_STATE_DPSI] != 0.0F) {
        fputs("yaw command deadband was not applied\n", stderr);
        return 1;
    }

    bc_yaw_reference_update(&yaw, 100.0F, 0.1F, &reference);
    if (!nearly_equal(reference.value[BC_STATE_PSI], -0.65F) ||
        !nearly_equal(reference.value[BC_STATE_DPSI], 1.0F)) {
        fputs("positive yaw reference ramp is incorrect\n", stderr);
        return 1;
    }

    const float held_heading = reference.value[BC_STATE_PSI];
    bc_yaw_reference_update(&yaw, -100.0F, 0.0F, &reference);
    bc_yaw_reference_update(&yaw, -100.0F, -0.1F, &reference);
    if (reference.value[BC_STATE_PSI] != held_heading ||
        reference.value[BC_STATE_DPSI] != 1.0F) {
        fputs("non-positive timestep advanced yaw reference\n", stderr);
        return 1;
    }

    for (int step = 0; step < 20; ++step) {
        bc_yaw_reference_update(&yaw, 100.0F, 0.1F, &reference);
    }
    if (!nearly_equal(
            reference.value[BC_STATE_DPSI], 4.0F * BC_PI_F)) {
        fputs("yaw reference did not respect its positive limit\n", stderr);
        return 1;
    }

    for (int step = 0; step < 30; ++step) {
        bc_yaw_reference_update(&yaw, -100.0F, 0.1F, &reference);
    }
    if (!(reference.value[BC_STATE_DPSI] < 0.0F)) {
        fputs("yaw reference did not ramp through zero\n", stderr);
        return 1;
    }

    bc_yaw_reference_reset(&yaw);
    if (yaw.rate_ramp.value != 0.0F) {
        fputs("yaw reference reset did not clear the ramp\n", stderr);
        return 1;
    }

    return 0;
}
