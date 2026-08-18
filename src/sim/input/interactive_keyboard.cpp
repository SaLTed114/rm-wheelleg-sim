#include "input/interactive_keyboard.hpp"

#include <GLFW/glfw3.h>

namespace balance::sim {
namespace {

bool pressed(GLFWwindow *window, const int key) {
    return glfwGetKey(window, key) == GLFW_PRESS;
}

} // namespace

InteractiveKeyboardFrame InteractiveKeyboard::sample(
    const bool input_captured
) {
    const bool pause = pressed(window_, GLFW_KEY_SPACE);
    const bool reset = pressed(window_, GLFW_KEY_R) ||
        pressed(window_, GLFW_KEY_BACKSPACE);
    const bool step = pressed(window_, GLFW_KEY_T);

    InteractiveKeyboardFrame frame{};
    frame.pause_pressed = !input_captured && pause && !pause_pressed_;
    frame.reset_pressed = !input_captured && reset && !reset_pressed_;
    frame.step_pressed = !input_captured && step && !step_pressed_;

    pause_pressed_ = pause;
    reset_pressed_ = reset;
    step_pressed_ = step;
    if (!input_captured) {
        const bool forward = pressed(window_, GLFW_KEY_W) ||
            pressed(window_, GLFW_KEY_UP);
        const bool reverse = pressed(window_, GLFW_KEY_S) ||
            pressed(window_, GLFW_KEY_DOWN);
        const bool left = pressed(window_, GLFW_KEY_A) ||
            pressed(window_, GLFW_KEY_LEFT);
        const bool right = pressed(window_, GLFW_KEY_D) ||
            pressed(window_, GLFW_KEY_RIGHT);
        frame.drive = {
            static_cast<float>(forward) - static_cast<float>(reverse),
            static_cast<float>(left) - static_cast<float>(right),
            pressed(window_, GLFW_KEY_LEFT_SHIFT) ||
                pressed(window_, GLFW_KEY_RIGHT_SHIFT),
        };
    }
    return frame;
}

} // namespace balance::sim
