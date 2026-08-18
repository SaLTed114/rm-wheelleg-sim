#include "balance_cpp/control_law/pd.hpp"

#include <algorithm>

namespace balance::control {

PdController::PdController(PdConfig config)
    : config_(config) {}

float PdController::calculate(
    const float position_error,
    const float velocity_error
) const {
    return std::clamp(
        config_.kp * position_error + config_.kd * velocity_error,
        -config_.output_limit, config_.output_limit);
}

} // namespace balance::control
