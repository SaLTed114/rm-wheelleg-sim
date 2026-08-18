#ifndef BALANCE_CPP_CONTROL_CORE_HPP
#define BALANCE_CPP_CONTROL_CORE_HPP

#include "balance_cpp/control_law/lqr.hpp"
#include "balance_cpp/control_law/pd.hpp"
#include "balance_cpp/math.hpp"
#include "balance_cpp/types.hpp"

namespace balance::control {

struct ControlCoreConfig {
    PdConfig length_controller{1600.0F, 75.0F, 220.0F};
    PdConfig angle_controller{50.0F, 6.0F, 30.0F};
    PdConfig roll_controller{800.0F, 60.0F, 200.0F};
    float support_force{76.204F};
    float leg_angle_trim{math::radians(2.42F)};
};

class ControlCore {
public:
    explicit ControlCore(ControlCoreConfig config = {});

    [[nodiscard]] ControlOutput calculate(
        const Estimate &estimate,
        const ControlCommand &command
    ) const;

private:
    PdController length_controller_{};
    PdController angle_controller_{};
    PdController roll_controller_{};
    LqrController lqr_controller_{};
    float leg_angle_trim_{};
    float support_force_{};
};

} // namespace balance::control

#endif
