#include "balance_cpp/state_machine/system.hpp"

namespace balance::control {

SystemStateMachine::SystemStateMachine(MotionConfig config)
    : motion_(config) {
    reset();
}

void SystemStateMachine::reset() {
    system_state_ = SystemState::off;
    motion_.reset();
}

ObservationContext SystemStateMachine::observation_context() const {
    return {system_state_ == SystemState::on &&
            motion_.wheel_observation_enabled()};
}

ControlCommand SystemStateMachine::update(
    const OperatorCommand &command,
    const Estimate &estimate,
    const float timestep
) {
    transition(command);
    return action(command, estimate, timestep);
}

void SystemStateMachine::transition(const OperatorCommand &command) {
    if (system_state_ == SystemState::off && command.system_enabled) {
        system_state_ = SystemState::on;
    } else if (system_state_ == SystemState::on && !command.system_enabled) {
        system_state_ = SystemState::off;
    }
}

ControlCommand SystemStateMachine::action(
    const OperatorCommand &command,
    const Estimate &estimate,
    const float timestep
) {
    if (system_state_ == SystemState::off) {
        motion_.reset();
        return {};
    }

    if (command.balance_restart) motion_.start();
    return motion_.update(estimate, timestep);
}

StateMachineStatus SystemStateMachine::status() const {
    return {system_state_, motion_.status()};
}

} // namespace balance::control
