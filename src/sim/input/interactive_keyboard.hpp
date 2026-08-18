#ifndef BALANCE_SIM_INTERACTIVE_KEYBOARD_HPP
#define BALANCE_SIM_INTERACTIVE_KEYBOARD_HPP

#include "input/keyboard_drive.hpp"

struct GLFWwindow;

namespace balance::sim {

struct InteractiveKeyboardFrame {
    KeyboardDriveInput drive{};
    bool pause_pressed{};
    bool reset_pressed{};
    bool step_pressed{};
};

class InteractiveKeyboard {
public:
    explicit InteractiveKeyboard(GLFWwindow *window) : window_(window) {}

    [[nodiscard]] InteractiveKeyboardFrame sample(bool input_captured);

private:
    GLFWwindow *window_{};
    bool pause_pressed_{};
    bool reset_pressed_{};
    bool step_pressed_{};
};

} // namespace balance::sim

#endif
