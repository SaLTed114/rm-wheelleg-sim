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
            0.8971487F, 2.0e-6F) ||
        expect_near(
            "right wheel", output.wheel_torque[BC_R],
            0.8974479F, 2.0e-6F) ||
        expect_near(
            "left leg", output.leg_torque[BC_L],
            1.0302552F, 2.0e-6F) ||
        expect_near(
            "right leg", output.leg_torque[BC_R],
            1.0339374F, 2.0e-6F)) {
        return 1;
    }

    bc_lqr_output_t below_range = {0};
    bc_lqr_output_t at_minimum = {0};
    bc_lqr_output_t at_previous_minimum = {0};
    bc_lqr_calculate(0.10F, &state, &reference, &below_range);
    bc_lqr_calculate(0.16F, &state, &reference, &at_minimum);
    bc_lqr_calculate(
        0.186F, &state, &reference, &at_previous_minimum);
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
    if (fabsf(
            at_minimum.wheel_torque[BC_L] -
            at_previous_minimum.wheel_torque[BC_L]) < 1.0e-4F) {
        fprintf(stderr, "0.16 m still uses the former 0.186 m gain\n");
        return 1;
    }

    return 0;
}
