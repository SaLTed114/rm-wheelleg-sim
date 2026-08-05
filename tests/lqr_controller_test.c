#include "balance/control_law/lqr.h"

#include "current_model_schedule.h"

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

static float expected_gain(
    const int input, const int state, const float normalized_length
) {
    const float *coefficient =
        bc_lqr_generated_coefficients[input][state];
    float gain = coefficient[0];

    for (int index = 1;
         index < BC_LQR_GENERATED_COEFFICIENT_COUNT; ++index) {
        gain = gain * normalized_length + coefficient[index];
    }
    return gain;
}

int main() {
    bc_state_vector_t state = {0};
    bc_state_vector_t reference = {0};
    bc_lqr_output_t output = {0};
    state.value[BC_STATE_THETA_B] = 0.1F;

    bc_lqr_calculate(0.20F, &state, &reference, &output);
    const float normalized_length =
        (0.20F - bc_lqr_generated_length_midpoint) /
            bc_lqr_generated_length_scale;
    const float expected[BC_LQR_GENERATED_INPUT_COUNT] = {
        -0.1F * expected_gain(0, BC_STATE_THETA_B, normalized_length),
        -0.1F * expected_gain(1, BC_STATE_THETA_B, normalized_length),
        -0.1F * expected_gain(2, BC_STATE_THETA_B, normalized_length),
        -0.1F * expected_gain(3, BC_STATE_THETA_B, normalized_length),
    };
    if (expect_near(
            "left wheel", output.wheel_torque[BC_L],
            expected[0], 2.0e-6F) ||
        expect_near(
            "right wheel", output.wheel_torque[BC_R],
            expected[1], 2.0e-6F) ||
        expect_near(
            "left leg", output.leg_torque[BC_L],
            expected[2], 2.0e-6F) ||
        expect_near(
            "right leg", output.leg_torque[BC_R],
            expected[3], 2.0e-6F)) {
        return 1;
    }

#ifdef BALANCE_DEFAULT_LQR_SCHEDULE
    if (expect_near(
            "default left wheel", output.wheel_torque[BC_L],
            0.8964367F, 2.0e-6F) ||
        expect_near(
            "default right wheel", output.wheel_torque[BC_R],
            0.8981590F, 2.0e-6F) ||
        expect_near(
            "default left leg", output.leg_torque[BC_L],
            1.0312429F, 2.0e-6F) ||
        expect_near(
            "default right leg", output.leg_torque[BC_R],
            1.0329542F, 2.0e-6F)) {
        return 1;
    }
#endif

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
