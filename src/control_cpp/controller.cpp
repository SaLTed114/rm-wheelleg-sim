#include "balance_cpp/controller.hpp"

namespace balance::control {

Controller::Controller(const ControllerConfig &config)
    : observer_(config.observer), state_machine_(config.motion),
      control_core_(config.control), gate_(config.output) {
    reset();
}

void Controller::reset() {
    observer_.reset();
    state_machine_.reset();
    tick_count_ = 0;
    snapshot_ = {};
}

ControllerOutput Controller::tick(
    const SensorFrame &sensor,
    const OperatorCommand &command,
    const float timestep_seconds
) noexcept {
    const Estimate estimate = observer_.update(
        sensor, state_machine_.observation_context(), timestep_seconds);
    const ControlCommand control_command = state_machine_.update(
        command, estimate, timestep_seconds);
    const ControlOutput control_output = control_core_.calculate(
        estimate, control_command);
    const StateMachineStatus status = state_machine_.status();
    const Actuation applied = gate_.apply(
        control_output.actuation, status.system == SystemState::on);
    ++tick_count_;
    snapshot_ = snapshot_assembler_.assemble(
        estimate, status, control_command, control_output,
        applied, tick_count_);
    return {control_output.actuation, applied, snapshot_};
}

} // namespace balance::control
