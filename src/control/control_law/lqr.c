#include "balance/control_law/lqr.h"
#include "balance/math_utils.h"

#include "current_model_schedule.h"

enum {
    BC_LQR_WHEEL_L,
    BC_LQR_WHEEL_R,
    BC_LQR_LEG_L,
    BC_LQR_LEG_R
};

_Static_assert(
    BC_LQR_GENERATED_INPUT_COUNT == 2 * BC_SIDE_NUM,
    "LQR input count does not match the control interface");
_Static_assert(
    BC_LQR_GENERATED_STATE_COUNT == BC_STATE_NUM,
    "LQR state count does not match the observer");

static float bc_lqr_gain(
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

void bc_lqr_calculate(
    const float leg_length,
    const bc_state_vector_t *state_error,
    bc_lqr_output_t *output
) {
    const float normalized_length = bc_clampf(
        (leg_length - bc_lqr_generated_length_midpoint) /
            bc_lqr_generated_length_scale,
        -1.0F, 1.0F);
    float input[BC_LQR_GENERATED_INPUT_COUNT] = {0.0F};

    for (int input_index = 0;
         input_index < BC_LQR_GENERATED_INPUT_COUNT; ++input_index) {
        for (int state_index = 0;
             state_index < BC_LQR_GENERATED_STATE_COUNT; ++state_index) {
            input[input_index] += bc_lqr_gain(
                input_index, state_index, normalized_length) *
                state_error->value[state_index];
        }
    }

    output->wheel_torque[BC_L] = input[BC_LQR_WHEEL_L];
    output->wheel_torque[BC_R] = input[BC_LQR_WHEEL_R];
    output->leg_torque[BC_L]   = input[BC_LQR_LEG_L];
    output->leg_torque[BC_R]   = input[BC_LQR_LEG_R];
}
