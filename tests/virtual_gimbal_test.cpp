#include "input/virtual_gimbal.hpp"

#include <cmath>
#include <iostream>

namespace {

bool near(const float value, const float expected) {
    return std::abs(value - expected) <= 1.0e-5F;
}

} // namespace

int main() {
    balance::sim::VirtualGimbal gimbal;
    if (!near(gimbal.config().yaw_rate_limit, 1.5F * BC_PI_F) ||
        !near(gimbal.config().yaw_acceleration_limit, 10.0F)) {
        std::cerr << "default virtual gimbal config is incorrect\n";
        return 1;
    }

    gimbal.reset(1.2F);
    if (!near(gimbal.state().world_yaw, 1.2F) ||
        gimbal.state().world_yaw_rate != 0.0F) {
        std::cerr << "virtual gimbal reset is incorrect\n";
        return 1;
    }

    gimbal.update(100.0F, 0.1F);
    if (!near(gimbal.state().world_yaw_rate, 1.0F) ||
        !near(gimbal.state().world_yaw, 1.3F)) {
        std::cerr << "virtual gimbal acceleration ramp is incorrect\n";
        return 1;
    }

    const auto held = gimbal.state();
    gimbal.update(-100.0F, 0.0F);
    gimbal.update(-100.0F, -0.1F);
    if (!near(gimbal.state().world_yaw, held.world_yaw) ||
        !near(gimbal.state().world_yaw_rate, held.world_yaw_rate)) {
        std::cerr << "non-positive timestep advanced virtual gimbal\n";
        return 1;
    }

    for (int step = 0; step < 10; ++step) {
        gimbal.update(100.0F, 0.1F);
    }
    if (!near(gimbal.state().world_yaw_rate, 1.5F * BC_PI_F)) {
        std::cerr << "virtual gimbal did not respect rate limit\n";
        return 1;
    }

    for (int step = 0; step < 10; ++step) {
        gimbal.update(-100.0F, 0.1F);
    }
    if (!(gimbal.state().world_yaw_rate < 0.0F)) {
        std::cerr << "virtual gimbal did not ramp through zero\n";
        return 1;
    }

    while (gimbal.state().world_yaw_rate < 0.0F) {
        gimbal.update(0.0F, 0.1F);
    }
    const float stopped_yaw = gimbal.state().world_yaw;
    gimbal.update(0.0F, 0.1F);
    if (!near(gimbal.state().world_yaw_rate, 0.0F) ||
        !near(gimbal.state().world_yaw, stopped_yaw)) {
        std::cerr << "stopped virtual gimbal did not hold heading\n";
        return 1;
    }

    gimbal.reset(2.0F);
    gimbal.update(1.0F, 0.1F);
    const auto feedback = gimbal.feedback(1.8F, -0.3F);
    if (!near(feedback.relative_yaw, 0.3F) ||
        !near(feedback.relative_yaw_rate, 1.3F)) {
        std::cerr << "virtual gimbal feedback mapping is incorrect\n";
        return 1;
    }

    return 0;
}
