#ifndef BALANCE_SIM_INPUT_KEYBOARD_DRIVE_HPP
#define BALANCE_SIM_INPUT_KEYBOARD_DRIVE_HPP

namespace balance::sim {

struct KeyboardDriveInput {
    float forward_axis{};
    float yaw_axis{};
    bool boost{};
};

} // namespace balance::sim

#endif
