#ifndef BALANCE_CPP_LQR_CONTROLLER_HPP
#define BALANCE_CPP_LQR_CONTROLLER_HPP

#include "balance_cpp/types.hpp"

namespace balance::control {

struct LqrOutput {
    Sides<float> wheel_torque{};
    Sides<float> leg_torque{};
};

class LqrController {
public:
    [[nodiscard]] LqrOutput calculate(
        float leg_length, const StateVector &state_error) const;
};

} // namespace balance::control

#endif
