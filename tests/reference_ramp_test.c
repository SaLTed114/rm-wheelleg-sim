#include "balance/math_utils.h"
#include "balance/reference_ramp.h"

#include <math.h>
#include <stdio.h>

static int expect_near(
    const char *name,
    const float actual,
    const float expected,
    const float tolerance
) {
    if (fabsf(actual - expected) <= tolerance) return 0;

    fprintf(
        stderr, "%s: expected %.7f, got %.7f\n",
        name, expected, actual);
    return 1;
}

int main() {
    const bc_reference_ramp_config_t forward_config = {
        .value_limit = 3.0F,
        .rate_limit = 5.0F,
    };
    const bc_reference_ramp_config_t yaw_config = {
        .value_limit = 4.0F * BC_PI_F,
        .rate_limit = 15.0F,
    };
    bc_reference_ramp_t ramp;

    bc_reference_ramp_reset(&ramp);
    if (ramp.value != 0.0F) {
        fputs("reference ramp reset was not zero\n", stderr);
        return 1;
    }
    if (bc_reference_ramp_update(
            &ramp, &forward_config, 3.0F, 0.0F) != 0.0F ||
        bc_reference_ramp_update(
            &ramp, &forward_config, 3.0F, -0.001F) != 0.0F) {
        fputs("non-positive timestep advanced reference ramp\n", stderr);
        return 1;
    }

    for (int tick = 0; tick < 600; ++tick) {
        bc_reference_ramp_update(
            &ramp, &forward_config, 10.0F, 0.001F);
    }
    if (expect_near("forward limit", ramp.value, 3.0F, 1.0e-6F)) {
        return 1;
    }
    if (bc_reference_ramp_update(
            &ramp, &forward_config, 10.0F, 0.001F) != 3.0F) {
        fputs("reference ramp overshot positive limit\n", stderr);
        return 1;
    }

    for (int tick = 0; tick < 1200; ++tick) {
        bc_reference_ramp_update(
            &ramp, &forward_config, -10.0F, 0.001F);
    }
    if (expect_near("reverse limit", ramp.value, -3.0F, 1.0e-6F)) {
        return 1;
    }

    bc_reference_ramp_reset(&ramp);
    for (int tick = 0; tick < 837; ++tick) {
        bc_reference_ramp_update(
            &ramp, &yaw_config, 5.0F * BC_PI_F, 0.001F);
    }
    if (!(ramp.value < 4.0F * BC_PI_F)) {
        fputs("yaw reference reached its limit too early\n", stderr);
        return 1;
    }
    bc_reference_ramp_update(
        &ramp, &yaw_config, 5.0F * BC_PI_F, 0.001F);
    if (expect_near(
            "yaw limit", ramp.value,
            4.0F * BC_PI_F, 1.0e-6F)) {
        return 1;
    }

    return 0;
}
