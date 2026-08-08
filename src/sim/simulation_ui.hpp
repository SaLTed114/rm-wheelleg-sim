#ifndef BALANCE_SIM_SIMULATION_UI_HPP
#define BALANCE_SIM_SIMULATION_UI_HPP

#include "balance/controller_snapshot.h"
#include "input/virtual_gimbal.hpp"

struct GLFWwindow;

namespace balance::sim {

struct SimulationUiActions {
    bool toggle_pause{};
    bool reset{};
};

struct SimulationUiFrame {
    const bc_controller_snapshot_t *snapshot{};
    VirtualGimbalState virtual_gimbal{};
    double simulation_time{};
    const char *phase{"unknown"};
    const char *case_name{};
    const char *case_issue{"none"};
    bool paused{};
    bool case_finished{};
};

class SimulationUi {
public:
    explicit SimulationUi(GLFWwindow *window);
    ~SimulationUi();

    SimulationUi(const SimulationUi &) = delete;
    SimulationUi &operator=(const SimulationUi &) = delete;

    [[nodiscard]] SimulationUiActions draw(
        const SimulationUiFrame &frame);
    void render();

    [[nodiscard]] bool wants_keyboard() const noexcept;
    [[nodiscard]] float sidebar_width() const noexcept {
        return sidebar_width_;
    }

private:
    static void draw_overview(
        const SimulationUiFrame &frame,
        SimulationUiActions &actions);
    static void draw_motion(const SimulationUiFrame &frame);
    static void draw_state(const bc_controller_snapshot_t &snapshot);
    static void draw_legs(const bc_controller_snapshot_t &snapshot);
    static void draw_actuation(const bc_controller_snapshot_t &snapshot);

    bool glfw_backend_initialized_{};
    bool opengl_backend_initialized_{};
    float sidebar_width_{};
};

} // namespace balance::sim

#endif
