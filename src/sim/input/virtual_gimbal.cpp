#include "input/virtual_gimbal.hpp"

namespace balance::sim {

VirtualGimbal::VirtualGimbal(const VirtualGimbalConfig &config)
    : config_(config),
      rate_config_{
          config.yaw_rate_limit,
          config.yaw_acceleration_limit,
      } {
    reset();
}

void VirtualGimbal::reset(const float world_yaw) noexcept {
    bc_reference_ramp_reset(&rate_ramp_);
    state_ = {world_yaw, 0.0F};
}

void VirtualGimbal::update(
    const float target_yaw_rate,
    const float timestep_seconds
) noexcept {
    const float rate = bc_reference_ramp_update(
        &rate_ramp_, &rate_config_, target_yaw_rate,
        timestep_seconds);
    if (timestep_seconds > 0.0F) {
        state_.world_yaw += rate * timestep_seconds;
    }
    state_.world_yaw_rate = rate;
}

} // namespace balance::sim
