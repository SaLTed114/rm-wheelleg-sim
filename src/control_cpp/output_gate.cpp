#include "balance_cpp/output_gate.hpp"

#include <algorithm>
#include <cmath>

namespace balance::control {
namespace {

bool all_finite(const Actuation &request) {
    for (std::size_t side = 0; side < side_count; ++side) {
        if (!std::isfinite(request.wheel_torque[side])) return false;
        for (std::size_t joint = 0; joint < joint_count; ++joint) {
            if (!std::isfinite(request.leg[side].joint_torque[joint])) {
                return false;
            }
        }
    }
    return true;
}

} // namespace

OutputGate::OutputGate(OutputGateConfig config)
    : wheel_torque_limit_(config.wheel_torque_limit),
      joint_torque_limit_(config.joint_torque_limit) {}

Actuation OutputGate::apply(
    const Actuation &request,
    const bool enabled
) const {
    Actuation applied{};
    if (!enabled || !all_finite(request)) return applied;
    for (std::size_t side = 0; side < side_count; ++side) {
        applied.wheel_torque[side] = std::clamp(
            request.wheel_torque[side],
            -wheel_torque_limit_, wheel_torque_limit_);
        for (std::size_t joint = 0; joint < joint_count; ++joint) {
            applied.leg[side].joint_torque[joint] = std::clamp(
                request.leg[side].joint_torque[joint],
                -joint_torque_limit_, joint_torque_limit_);
        }
    }
    return applied;
}

} // namespace balance::control
