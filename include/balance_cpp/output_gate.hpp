#ifndef BALANCE_CPP_OUTPUT_GATE_HPP
#define BALANCE_CPP_OUTPUT_GATE_HPP

#include "balance_cpp/types.hpp"

namespace balance::control {

struct OutputGateConfig {
    float wheel_torque_limit{6.32F};
    float joint_torque_limit{40.0F};
};

class OutputGate {
public:
    explicit OutputGate(OutputGateConfig config = {});

    [[nodiscard]] Actuation apply(
        const Actuation &request, bool enabled
    ) const;

private:
    float wheel_torque_limit_{};
    float joint_torque_limit_{};
};

} // namespace balance::control

#endif
