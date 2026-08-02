#include "balance/control_law/lqr.h"

#include <math.h>
#include <stdio.h>

static int expect_near(
    const char *name, const float actual,
    const float expected, const float tolerance
) {
    if (fabsf(actual - expected) <= tolerance) return 0;

    fprintf(
        stderr, "%s: expected %.9f, got %.9f\n",
        name, expected, actual);
    return 1;
}

int main() {
    bc_state_vector_t state = {0};
    bc_state_vector_t reference = {0};
    bc_lqr_output_t output = {0};
    state.value[BC_STATE_THETA_B] = 0.1F;

    bc_lqr_calculate(0.20F, &state, &reference, &output);
    if (expect_near(
            "left wheel", output.wheel_torque[BC_L],
            1.1269755F, 2.0e-6F) ||
        expect_near(
            "right wheel", output.wheel_torque[BC_R],
            1.1256603F, 2.0e-6F) ||
        expect_near(
            "left leg", output.leg_torque[BC_L],
            1.6505593F, 2.0e-6F) ||
        expect_near(
            "right leg", output.leg_torque[BC_R],
            1.6548813F, 2.0e-6F)) {
        return 1;
    }

    bc_lqr_output_t below_range = {0};
    bc_lqr_output_t at_minimum = {0};
    bc_lqr_calculate(0.10F, &state, &reference, &below_range);
    bc_lqr_calculate(0.186F, &state, &reference, &at_minimum);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (expect_near(
                "clamped wheel", below_range.wheel_torque[side],
                at_minimum.wheel_torque[side], 2.0e-7F) ||
            expect_near(
                "clamped leg", below_range.leg_torque[side],
                at_minimum.leg_torque[side], 2.0e-7F)) {
            return 1;
        }
    }

    return 0;
}
