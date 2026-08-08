#ifndef BALANCE_SIM_INPUT_VIRTUAL_GIMBAL_HPP
#define BALANCE_SIM_INPUT_VIRTUAL_GIMBAL_HPP

#include "balance/math_utils.h"
#include "balance/reference/ramp.h"
#include "balance/types.h"

namespace balance::sim {

struct VirtualGimbalConfig {
    float yaw_rate_limit{1.5F * BC_PI_F};
    float yaw_acceleration_limit{10.0F};
};

struct VirtualGimbalState {
    float world_yaw{};
    float world_yaw_rate{};
};

class VirtualGimbal {
public:
    explicit VirtualGimbal(
        const VirtualGimbalConfig &config = VirtualGimbalConfig{});

    void reset(float world_yaw = 0.0F) noexcept;
    void update(float target_yaw_rate, float timestep_seconds) noexcept;

    [[nodiscard]] const VirtualGimbalConfig &config() const noexcept {
        return config_;
    }
    [[nodiscard]] const VirtualGimbalState &state() const noexcept {
        return state_;
    }
private:
    VirtualGimbalConfig config_;
    bc_reference_ramp_config_t rate_config_{};
    bc_reference_ramp_t rate_ramp_{};
    VirtualGimbalState state_{};
};

} // namespace balance::sim

#endif
