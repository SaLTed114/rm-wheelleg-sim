#ifndef BALANCE_CPP_CONTROLLER_HPP
#define BALANCE_CPP_CONTROLLER_HPP

#include "balance_cpp/control_core.hpp"
#include "balance_cpp/observer.hpp"
#include "balance_cpp/output_gate.hpp"
#include "balance_cpp/snapshot_assembler.hpp"
#include "balance_cpp/state_machine/system.hpp"

namespace balance::control {

struct ControllerConfig {
    ObserverConfig observer{};
    MotionConfig motion{};
    ControlCoreConfig control{};
    OutputGateConfig output{};
};

class Controller {
public:
    explicit Controller(const ControllerConfig &config = {});

    [[nodiscard]] ControllerOutput tick(
        const SensorFrame &sensor,
        const OperatorCommand &command,
        float timestep_seconds
    ) noexcept;
    void reset();

    const Snapshot &snapshot() const {
        return snapshot_;
    }

private:
    Observer observer_{};
    SystemStateMachine state_machine_{};
    ControlCore control_core_{};
    OutputGate gate_{};
    SnapshotAssembler snapshot_assembler_{};
    std::uint32_t tick_count_{};
    Snapshot snapshot_{};
};

} // namespace balance::control

#endif
