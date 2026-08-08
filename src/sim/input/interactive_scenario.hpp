#ifndef BALANCE_SIM_INPUT_INTERACTIVE_SCENARIO_HPP
#define BALANCE_SIM_INPUT_INTERACTIVE_SCENARIO_HPP

#include "balance/controller_snapshot.h"
#include "input/keyboard_drive.hpp"
#include "input/virtual_gimbal.hpp"

namespace balance::sim {

enum class InteractiveMode {
    demo,
    keyboard,
};

struct InteractiveScenarioFrame {
    bc_operator_command_t command{};
    VirtualGimbalState gimbal{};
    const char *phase{"off"};
};

class InteractiveScenario {
public:
    explicit InteractiveScenario(InteractiveMode mode) noexcept;

    void reset(const bc_controller_snapshot_t &snapshot) noexcept;
    [[nodiscard]] const InteractiveScenarioFrame &update(
        const bc_controller_snapshot_t &snapshot,
        const KeyboardDriveInput &keyboard,
        double simulation_time,
        float timestep_seconds) noexcept;
    [[nodiscard]] const InteractiveScenarioFrame &frame() const noexcept {
        return frame_;
    }

private:
    struct MotionTarget {
        float forward_velocity;
        float gimbal_yaw_rate;
        const char *phase;
    };

    [[nodiscard]] MotionTarget demo_target(
        const bc_controller_snapshot_t &snapshot,
        double simulation_time) const noexcept;
    [[nodiscard]] static MotionTarget keyboard_target(
        const bc_controller_snapshot_t &snapshot,
        const KeyboardDriveInput &keyboard) noexcept;

    InteractiveMode mode_;
    VirtualGimbal virtual_gimbal_;
    InteractiveScenarioFrame frame_{};
    bc_motion_state_t previous_motion_{BC_MOTION_IDLE};
    double balance_start_time_{-1.0};
    bool virtual_gimbal_initialized_{};
};

} // namespace balance::sim

#endif
