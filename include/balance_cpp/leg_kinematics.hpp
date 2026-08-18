#ifndef BALANCE_CPP_LEG_KINEMATICS_HPP
#define BALANCE_CPP_LEG_KINEMATICS_HPP

#include "balance_cpp/types.hpp"

namespace balance::control {

struct LegKinematicsConfig {
    float hip_link_length{0.215F};
    float wheel_link_length{0.254F};
};

class LegKinematicsSolver {
public:
    explicit LegKinematicsSolver(LegKinematicsConfig config = {});

    [[nodiscard]] LegKinematics calculate(
        const LegFeedback &feedback
    ) const;

private:
    float hip_link_length_{};
    float wheel_link_length_{};
};

} // namespace balance::control

#endif
