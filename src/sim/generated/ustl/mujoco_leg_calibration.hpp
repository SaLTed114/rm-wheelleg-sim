#ifndef BALANCE_SIM_GENERATED_COD_MUJOCO_LEG_CALIBRATION_HPP
#define BALANCE_SIM_GENERATED_COD_MUJOCO_LEG_CALIBRATION_HPP

#include <array>

namespace balance::sim::cod_calibration {

// Retained from the COD adapter calibration before the Fudan migration.
inline constexpr std::array<std::array<double, 2>, 2> kJointScales{{
    {{-1.0, +1.0}},
    {{+1.0, -1.0}},
}};
inline constexpr std::array<std::array<double, 2>, 2> kJointOffsets{{
    {{-2.965142988559353, -0.067723581144335}},
    {{-2.936362292494494, -0.032802658978973}},
}};

} // namespace balance::sim::cod_calibration

#endif
