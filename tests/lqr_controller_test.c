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

static float expected_yaw_acceleration_feedforward(
    const int input, const float normalized_length
) {
    const float *coefficient =
        bc_lqr_generated_yaw_acceleration_feedforward_coefficients[input];
    float feedforward = coefficient[0];

    for (int index = 1;
         index < BC_LQR_GENERATED_COEFFICIENT_COUNT; ++index) {
        feedforward = feedforward * normalized_length + coefficient[index];
    }
    return feedforward;
}

int main() {
    bc_state_vector_t state_error = {0};
    bc_lqr_output_t output = {0};
    state_error.value[BC_STATE_THETA_B] = -0.1F;

    bc_lqr_calculate(0.20F, &state_error, 0.0F, &output);
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
            0.7438450F, 2.0e-6F) ||
        expect_near(
            "default right wheel", output.wheel_torque[BC_R],
            0.7440303F, 2.0e-6F) ||
        expect_near(
            "default left leg", output.leg_torque[BC_L],
            2.3533027F, 2.0e-6F) ||
        expect_near(
            "default right leg", output.leg_torque[BC_R],
            2.3542850F, 2.0e-6F)) {
        return 1;
    }
#endif

    bc_state_vector_t zero_error = {0};
    bc_lqr_output_t feedforward = {0};
    bc_lqr_calculate(0.20F, &zero_error, 1.0F, &feedforward);
    const float expected_feedforward[BC_LQR_GENERATED_INPUT_COUNT] = {
        expected_yaw_acceleration_feedforward(0, normalized_length),
        expected_yaw_acceleration_feedforward(1, normalized_length),
        expected_yaw_acceleration_feedforward(2, normalized_length),
        expected_yaw_acceleration_feedforward(3, normalized_length),
    };
    if (expect_near(
            "left wheel feedforward",
            feedforward.wheel_torque[BC_L],
            expected_feedforward[0], 2.0e-6F) ||
        expect_near(
            "right wheel feedforward",
            feedforward.wheel_torque[BC_R],
            expected_feedforward[1], 2.0e-6F) ||
        expect_near(
            "left leg feedforward",
            feedforward.leg_torque[BC_L],
            expected_feedforward[2], 2.0e-6F) ||
        expect_near(
            "right leg feedforward",
            feedforward.leg_torque[BC_R],
            expected_feedforward[3], 2.0e-6F)) {
        return 1;
    }

    bc_lqr_output_t below_range = {0};
    bc_lqr_output_t at_minimum = {0};
    bc_lqr_output_t at_previous_minimum = {0};
    bc_lqr_calculate(0.10F, &state_error, 0.0F, &below_range);
    bc_lqr_calculate(0.16F, &state_error, 0.0F, &at_minimum);
    bc_lqr_calculate(
        0.186F, &state_error, 0.0F, &at_previous_minimum);
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
