#include "balance_cpp/control_law/lqr.hpp"

#include <algorithm>
#include <array>

#include "current_model_schedule.h"

namespace balance::control {
namespace {

float polynomial(
    const float coefficient[BC_LQR_GENERATED_COEFFICIENT_COUNT],
    const float value
) {
    float result = coefficient[0];
    for (int index = 1; index < BC_LQR_GENERATED_COEFFICIENT_COUNT; ++index) {
        result = result * value + coefficient[index];
    }
    return result;
}

} // namespace

LqrOutput LqrController::calculate(
    const float leg_length,
    const StateVector &state_error
) const {
    const float normalized = std::clamp(
        (leg_length - bc_lqr_generated_length_midpoint) /
            bc_lqr_generated_length_scale,
        -1.0F, 1.0F);
    std::array<float, BC_LQR_GENERATED_INPUT_COUNT> input{};
    for (int row = 0; row < BC_LQR_GENERATED_INPUT_COUNT; ++row) {
        for (int column = 0; column < BC_LQR_GENERATED_STATE_COUNT; ++column) {
            input[static_cast<std::size_t>(row)] += polynomial(
                bc_lqr_generated_coefficients[row][column], normalized) *
                state_error.value[static_cast<std::size_t>(column)];
        }
    }
    return {{{input[0], input[1]}}, {{input[2], input[3]}}};
}

} // namespace balance::control
