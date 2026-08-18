#ifndef BALANCE_CPP_SYSTEM_STATE_MACHINE_HPP
#define BALANCE_CPP_SYSTEM_STATE_MACHINE_HPP

#include "balance_cpp/state_machine/motion.hpp"
#include "balance_cpp/types.hpp"

namespace balance::control {

class SystemStateMachine {
public:
    explicit SystemStateMachine(MotionConfig config = {});

    void reset();
    ObservationContext observation_context() const;
    [[nodiscard]] ControlCommand update(
        const OperatorCommand &command,
        const Estimate &estimate,
        float timestep
    );
    StateMachineStatus status() const;

private:
    void transition(const OperatorCommand &command);
    ControlCommand action(
        const OperatorCommand &command,
        const Estimate &estimate,
        float timestep);

    SystemState system_state_{SystemState::off};
    MotionStateMachine motion_{};
};

} // namespace balance::control

#endif
