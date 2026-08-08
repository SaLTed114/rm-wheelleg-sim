#include "mujoco_viewer.hpp"

#include <stdexcept>

#include <GLFW/glfw3.h>

namespace balance::sim {

MujocoViewer::MujocoViewer(const mjModel &model)
    : model_(model) {
    if (!glfwInit()) {
        throw std::runtime_error("failed to initialize GLFW");
    }

    window_ = glfwCreateWindow(
        1280, 720, "rm-balance-sim", nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("failed to create GLFW window");
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, key_callback);
    glfwSetMouseButtonCallback(window_, mouse_button_callback);
    glfwSetCursorPosCallback(window_, cursor_position_callback);
    glfwSetScrollCallback(window_, scroll_callback);

    mjv_defaultFreeCamera(&model_, &camera_);
    const int tracked_body = mj_name2id(&model_, mjOBJ_BODY, "base_link");
    if (tracked_body >= 0) {
        camera_.type = mjCAMERA_TRACKING;
        camera_.trackbodyid = tracked_body;
        camera_.distance = 3.0;
        camera_.azimuth = 135.0;
        camera_.elevation = -20.0;
    }
    mjv_defaultOption(&option_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);
    mjv_makeScene(&model_, &scene_, 2000);
    mjr_makeContext(&model_, &context_, mjFONTSCALE_150);
}

MujocoViewer::~MujocoViewer() {
    mjv_freeScene(&scene_);
    mjr_freeContext(&context_);
    if (window_) {
        glfwDestroyWindow(window_);
    }
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
}

bool MujocoViewer::should_close() const {
    return glfwWindowShouldClose(window_) != 0;
}

KeyboardDriveInput MujocoViewer::keyboard_drive_input() const {
    const bool forward =
        glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS ||
        glfwGetKey(window_, GLFW_KEY_UP) == GLFW_PRESS;
    const bool reverse =
        glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS ||
        glfwGetKey(window_, GLFW_KEY_DOWN) == GLFW_PRESS;
    const bool left =
        glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS ||
        glfwGetKey(window_, GLFW_KEY_LEFT) == GLFW_PRESS;
    const bool right =
        glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS ||
        glfwGetKey(window_, GLFW_KEY_RIGHT) == GLFW_PRESS;
    const bool boost =
        glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    return {
        static_cast<float>(forward) - static_cast<float>(reverse),
        static_cast<float>(left) - static_cast<float>(right),
        boost,
    };
}

bool MujocoViewer::consume_reset_request() {
    const bool requested = reset_requested_;
    reset_requested_ = false;
    return requested;
}

void MujocoViewer::set_title(const std::string &title) {
    glfwSetWindowTitle(window_, title.c_str());
}

void MujocoViewer::render(mjData &data) {
    mjrRect viewport{};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    mjv_updateScene(
        &model_, &data, &option_, nullptr,
        &camera_, mjCAT_ALL, &scene_);
    mjr_render(viewport, &scene_, &context_);
    glfwSwapBuffers(window_);
}

void MujocoViewer::poll_events() {
    glfwPollEvents();
}

MujocoViewer &MujocoViewer::from_window(GLFWwindow *window) {
    return *static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
}

void MujocoViewer::key_callback(
    GLFWwindow *window, int key, int scancode, int action, int mods
) {
    (void)scancode;
    (void)mods;
    from_window(window).handle_key(key, action);
}

void MujocoViewer::mouse_button_callback(
    GLFWwindow *window, int button, int action, int mods
) {
    (void)button;
    (void)action;
    (void)mods;
    from_window(window).handle_mouse_button();
}

void MujocoViewer::cursor_position_callback(
    GLFWwindow *window, double x, double y
) {
    from_window(window).handle_cursor_position(x, y);
}

void MujocoViewer::scroll_callback(
    GLFWwindow *window, double x_offset, double y_offset
) {
    (void)x_offset;
    from_window(window).handle_scroll(y_offset);
}

void MujocoViewer::handle_key(int key, int action) {
    if (action != GLFW_PRESS) {
        return;
    }

    if (key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    } else if (key == GLFW_KEY_SPACE) {
        paused_ = !paused_;
    } else if (key == GLFW_KEY_BACKSPACE || key == GLFW_KEY_R) {
        reset_requested_ = true;
    }
}

void MujocoViewer::handle_mouse_button() {
    left_button_ =
        glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    middle_button_ =
        glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    right_button_ =
        glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(window_, &cursor_x_, &cursor_y_);
}

void MujocoViewer::handle_cursor_position(double x, double y) {
    if (!left_button_ && !middle_button_ && !right_button_) {
        return;
    }

    const double dx = x - cursor_x_;
    const double dy = y - cursor_y_;
    cursor_x_ = x;
    cursor_y_ = y;

    int width = 0;
    int height = 0;
    glfwGetWindowSize(window_, &width, &height);
    (void)width;
    if (height <= 0) {
        return;
    }

    const bool shift =
        glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    mjtMouse action;
    if (right_button_) {
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (left_button_) {
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(
        &model_, action, dx / height, dy / height, &scene_, &camera_);
}

void MujocoViewer::handle_scroll(double y_offset) {
    mjv_moveCamera(
        &model_, mjMOUSE_ZOOM, 0.0, -0.05 * y_offset,
        &scene_, &camera_);
}

} // namespace balance::sim
