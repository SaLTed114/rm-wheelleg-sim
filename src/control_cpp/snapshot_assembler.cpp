#include "balance_cpp/snapshot_assembler.hpp"

namespace balance::control {

Snapshot SnapshotAssembler::assemble(
    const Estimate &estimate,
    const StateMachineStatus &status,
    const ControlCommand &command,
    const ControlOutput &control_output,
    const Actuation &applied,
    const std::uint32_t tick_count
) const {
    Snapshot snapshot{};
    snapshot.system_state = status.system;
    snapshot.motion_state = status.motion.state;
    snapshot.state = estimate.state;
    snapshot.state_reference = command.state_reference;
    snapshot.leg = estimate.leg;
    snapshot.velocity_estimate = estimate.velocity;
    snapshot.roll = estimate.roll;
    snapshot.roll_rate = estimate.roll_rate;
    snapshot.roll_force_request = control_output.roll_force_request;
    snapshot.leg_stable_elapsed = status.motion.leg_stable_elapsed;
    snapshot.engage_elapsed = status.motion.engage_elapsed;
    snapshot.actuation_request = control_output.actuation;
    snapshot.actuation = applied;
    snapshot.tick_count = tick_count;
    return snapshot;
}

} // namespace balance::control
