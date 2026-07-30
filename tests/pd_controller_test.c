#include "balance/pd_controller.h"

#include <math.h>
#include <stdio.h>

static int expect_near(const char *name, float actual, float expected) {
    if (fabsf(actual - expected) <= 1.0e-6F) return 0;

    fprintf(
        stderr, "%s: expected %.6f, got %.6f\n",
        name, expected, actual);
    return 1;
}

int main() {
    const bc_pd_controller_t controller = {
        .kp           = 2.0F,
        .kd           = 3.0F,
        .output_limit = 10.0F,
    };
    int failures = 0;

    failures += expect_near(
        "unsaturated output", bc_pd_calculate(&controller, 1.0F, -0.5F),
        0.5F);
    failures += expect_near(
        "positive saturation", bc_pd_calculate(&controller, 10.0F, 0.0F),
        10.0F);
    failures += expect_near(
        "negative saturation", bc_pd_calculate(&controller, -10.0F, 0.0F),
        -10.0F);

    return failures == 0 ? 0 : 1;
}
