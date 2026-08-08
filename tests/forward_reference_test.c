#include "balance/reference/forward.h"

#include <math.h>
#include <stdio.h>

static int nearly_equal(const float actual, const float expected) {
    return fabsf(actual - expected) <= 1.0e-6F;
}

int main() {
    bc_forward_reference_config_t config;
    bc_forward_reference_t forward;
    bc_state_vector_t reference = {0};

    bc_forward_reference_default_config(&config);
    if (config.command_deadband != 0.01F ||
        config.velocity_ramp.value_limit != 3.0F ||
        config.velocity_ramp.rate_limit != 5.0F) {
        fputs("default forward reference config is incorrect\n", stderr);
        return 1;
    }

    bc_forward_reference_init(&forward, &config);
    if (forward.velocity_ramp.value != 0.0F) {
        fputs("forward reference did not initialize at rest\n", stderr);
        return 1;
    }

    reference.value[BC_STATE_DS] = 2.0F;
    bc_forward_reference_start(&forward, 1.25F, &reference);
    if (reference.value[BC_STATE_S] != 1.25F ||
        reference.value[BC_STATE_DS] != 0.0F) {
        fputs("forward reference start did not capture position\n", stderr);
        return 1;
    }

    if (bc_forward_reference_requested(
            &forward, config.command_deadband) ||
        !bc_forward_reference_requested(
            &forward, 2.0F * config.command_deadband)) {
        fputs("forward request deadband is incorrect\n", stderr);
        return 1;
    }

    bc_forward_reference_update(
        &forward, config.command_deadband, 0.1F, &reference);
    if (reference.value[BC_STATE_S] != 1.25F ||
        reference.value[BC_STATE_DS] != 0.0F) {
        fputs("forward command deadband was not applied\n", stderr);
        return 1;
    }

    bc_forward_reference_update(&forward, 10.0F, 0.1F, &reference);
    if (!nearly_equal(reference.value[BC_STATE_S], 1.30F) ||
        !nearly_equal(reference.value[BC_STATE_DS], 0.5F)) {
        fputs("positive forward reference ramp is incorrect\n", stderr);
        return 1;
    }

    const float held_position = reference.value[BC_STATE_S];
    bc_forward_reference_update(&forward, -10.0F, 0.0F, &reference);
    bc_forward_reference_update(&forward, -10.0F, -0.1F, &reference);
    if (reference.value[BC_STATE_S] != held_position ||
        reference.value[BC_STATE_DS] != 0.5F) {
        fputs("non-positive timestep advanced forward reference\n", stderr);
        return 1;
    }

    for (int step = 0; step < 10; ++step) {
        bc_forward_reference_update(&forward, 10.0F, 0.1F, &reference);
    }
    if (reference.value[BC_STATE_DS] != 3.0F) {
        fputs("forward reference did not respect its positive limit\n", stderr);
        return 1;
    }

    for (int step = 0; step < 10; ++step) {
        bc_forward_reference_update(&forward, -10.0F, 0.1F, &reference);
    }
    if (!(reference.value[BC_STATE_DS] < 0.0F)) {
        fputs("forward reference did not ramp through zero\n", stderr);
        return 1;
    }

    bc_forward_reference_reset(&forward);
    if (forward.velocity_ramp.value != 0.0F) {
        fputs("forward reference reset did not clear the ramp\n", stderr);
        return 1;
    }

    return 0;
}
