#ifndef BALANCE_CPP_SNAPSHOT_ASSEMBLER_HPP
#define BALANCE_CPP_SNAPSHOT_ASSEMBLER_HPP

#include <cstdint>

#include "balance_cpp/types.hpp"

namespace balance::control {

class SnapshotAssembler {
public:
    [[nodiscard]] Snapshot assemble(
        const Estimate &estimate,
        const StateMachineStatus &status,
        const ControlCommand &command,
        const ControlOutput &control_output,
        const Actuation &applied,
        std::uint32_t tick_count
    ) const;
};

} // namespace balance::control

#endif
