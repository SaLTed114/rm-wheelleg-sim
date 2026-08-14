#include "input/interactive_scenario.hpp"

#include <cmath>
#include <iostream>

namespace {

bool near(const float value, const float expected) {
    return std::abs(value - expected) <= 1.0e-5F;
}

bc_controller_snapshot_t active_snapshot() {
    bc_controller_snapshot_t snapshot{};
    snapshot.state_machine.system = BC_SYSTEM_ON;
    snapshot.state_machine.motion = BC_MOTION_ACTIVE;
    snapshot.state_machine.forward = BC_FORWARD_VELOCITY;
    snapshot.state.value[BC_STATE_PSI] = 0.5F;
    snapshot.state.value[BC_STATE_DPSI] = -0.2F;
    return snapshot;
}

} // namespace

int main() {
    using balance::sim::InteractiveMode;
    using balance::sim::InteractiveScenario;
    using balance::sim::KeyboardDriveInput;

    bc_controller_snapshot_t snapshot{};
    InteractiveScenario keyboard(InteractiveMode::keyboard);
    keyboard.reset(snapshot);

    const auto &disabled = keyboard.update(
        snapshot, KeyboardDriveInput{}, 1.0, 0.001F);
    if (disabled.command.system_enabled ||
        disabled.command.balance_restart) {
        std::cerr << "disabled keyboard scenario produced a command\n";
        return 1;
    }

    const auto &engaging = keyboard.update(
        snapshot, KeyboardDriveInput{}, 2.0, 0.001F);
    if (!engaging.command.system_enabled ||
        !engaging.command.balance_restart) {
        std::cerr << "keyboard scenario did not request engagement\n";
        return 1;
    }

    snapshot = active_snapshot();
    const KeyboardDriveInput input{1.0F, 1.0F, false, true};
    const auto &moving = keyboard.update(snapshot, input, 3.0, 0.001F);
    if (!near(moving.command.forward_velocity, 2.0F) ||
        !near(moving.gimbal.world_yaw_rate, 0.01F) ||
        moving.gimbal.world_yaw <= 0.5F ||
        moving.command.task != BC_OPERATOR_TASK_STEP_DOCK) {
        std::cerr << "keyboard motion was not mapped through the gimbal\n";
        return 1;
    }

    snapshot.step_command_rearm_required = 1U;
    const auto &completed = keyboard.update(snapshot, input, 3.001, 0.001F);
    if (completed.command.task != BC_OPERATOR_TASK_NORMAL ||
        !completed.reset_step_task_latch) {
        std::cerr << "completed step did not reset the keyboard latch\n";
        return 1;
    }
    snapshot.step_command_rearm_required = 0U;

    const KeyboardDriveInput boosted_input{1.0F, 0.0F, true, false};
    const auto &boosted = keyboard.update(
        snapshot, boosted_input, 3.01, 0.001F);
    if (!near(boosted.command.forward_velocity, 2.7F)) {
        std::cerr << "keyboard boost exceeded its interactive limit\n";
        return 1;
    }

    snapshot.state_machine.motion = BC_MOTION_LEG_POSITIONING;
    const auto &inactive = keyboard.update(snapshot, input, 3.1, 0.001F);
    if (inactive.command.forward_velocity != 0.0F ||
        inactive.gimbal.world_yaw_rate != 0.0F) {
        std::cerr << "inactive motion did not reset interactive input\n";
        return 1;
    }

    InteractiveScenario demo(InteractiveMode::demo);
    snapshot = active_snapshot();
    demo.reset(snapshot);
    const auto &standing = demo.update(
        snapshot, KeyboardDriveInput{}, 3.0, 0.001F);
    if (standing.command.forward_velocity != 0.0F) {
        std::cerr << "demo did not begin with a standing phase\n";
        return 1;
    }
    const auto &forward = demo.update(
        snapshot, KeyboardDriveInput{}, 6.1, 0.001F);
    if (!near(forward.command.forward_velocity, 0.25F)) {
        std::cerr << "demo timeline did not enter forward phase\n";
        return 1;
    }

    return 0;
}
