#ifndef BALANCE_SIM_MUJOCO_VIEWER_HPP
#define BALANCE_SIM_MUJOCO_VIEWER_HPP

#include <mujoco/mujoco.h>

struct GLFWwindow;

namespace balance::sim {

class MujocoViewer {
public:
    explicit MujocoViewer(const mjModel &model);
    ~MujocoViewer();

    MujocoViewer(const MujocoViewer &) = delete;
    MujocoViewer &operator=(const MujocoViewer &) = delete;

    [[nodiscard]] bool should_close() const;
    [[nodiscard]] bool paused() const noexcept { return paused_; }
    bool consume_reset_request();

    void render(mjData &data);
    void poll_events();

private:
    static MujocoViewer &from_window(GLFWwindow *window);
    static void key_callback(
        GLFWwindow *window, int key, int scancode, int action, int mods);
    static void mouse_button_callback(
        GLFWwindow *window, int button, int action, int mods);
    static void cursor_position_callback(
        GLFWwindow *window, double x, double y);
    static void scroll_callback(
        GLFWwindow *window, double x_offset, double y_offset);

    void handle_key(int key, int action);
    void handle_mouse_button();
    void handle_cursor_position(double x, double y);
    void handle_scroll(double y_offset);

    const mjModel &model_;
    GLFWwindow *window_{};
    mjvCamera camera_{};
    mjvOption option_{};
    mjvScene scene_{};
    mjrContext context_{};

    bool left_button_{};
    bool middle_button_{};
    bool right_button_{};
    bool paused_{};
    bool reset_requested_{};
    double cursor_x_{};
    double cursor_y_{};
};

} // namespace balance::sim

#endif
