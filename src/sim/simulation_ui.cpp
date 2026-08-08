#include "simulation_ui.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string_view>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

namespace balance::sim {
namespace {

constexpr float kNominalSidebarWidth = 420.0F;
constexpr float kMinimumSceneWidth = 320.0F;
constexpr float kActuationDifferenceTolerance = 1.0e-4F;
constexpr float kBaseFontSize = 18.0F;
constexpr float kStyleScale = 1.15F;

constexpr const char *kStateNames[BC_STATE_NUM] = {
    "S", "DS", "PSI", "DPSI",
    "THETA_L", "DTHETA_L", "THETA_R", "DTHETA_R",
    "THETA_B", "DTHETA_B",
};

void table_value(const char *name, const char *value) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted(name);
    ImGui::TableSetColumnIndex(1);
    ImGui::TextUnformatted(value);
}

void draw_torque_row(
    const char *name, const float requested, const float applied) {
    const bool limited =
        std::abs(requested - applied) > kActuationDifferenceTolerance;
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted(name);
    ImGui::TableSetColumnIndex(1);
    ImGui::Text("% .3f", requested);
    ImGui::TableSetColumnIndex(2);
    if (limited) {
        ImGui::PushStyleColor(
            ImGuiCol_Text, ImVec4(1.0F, 0.65F, 0.15F, 1.0F));
    }
    ImGui::Text("% .3f", applied);
    if (limited) ImGui::PopStyleColor();
}

} // namespace

SimulationUi::SimulationUi(GLFWwindow *window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.IniFilename = nullptr;
    ImFontConfig font_config{};
    font_config.SizePixels = kBaseFontSize;
    io.Fonts->AddFontDefault(&font_config);
    ImGui::StyleColorsDark();
    ImGuiStyle &style = ImGui::GetStyle();
    style.ScaleAllSizes(kStyleScale);
    style.WindowRounding = 0.0F;
    style.WindowBorderSize = 0.0F;

    if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) {
        ImGui::DestroyContext();
        throw std::runtime_error("failed to initialize Dear ImGui GLFW backend");
    }
    glfw_backend_initialized_ = true;
    if (!ImGui_ImplOpenGL3_Init()) {
        ImGui_ImplGlfw_Shutdown();
        glfw_backend_initialized_ = false;
        ImGui::DestroyContext();
        throw std::runtime_error(
            "failed to initialize Dear ImGui OpenGL backend");
    }
    opengl_backend_initialized_ = true;
}

SimulationUi::~SimulationUi() {
    if (opengl_backend_initialized_) ImGui_ImplOpenGL3_Shutdown();
    if (glfw_backend_initialized_) ImGui_ImplGlfw_Shutdown();
    if (ImGui::GetCurrentContext() != nullptr) ImGui::DestroyContext();
}

SimulationUiActions SimulationUi::draw(const SimulationUiFrame &frame) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    sidebar_width_ = std::clamp(
        viewport->WorkSize.x - kMinimumSceneWidth,
        0.0F,
        kNominalSidebarWidth);

    SimulationUiActions actions{};
    if (sidebar_width_ <= 0.0F) return actions;

    ImGui::SetNextWindowPos(ImVec2(
        viewport->WorkPos.x + viewport->WorkSize.x - sidebar_width_,
        viewport->WorkPos.y));
    ImGui::SetNextWindowSize(ImVec2(
        sidebar_width_, viewport->WorkSize.y));
    constexpr ImGuiWindowFlags kWindowFlags =
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::Begin("Simulation diagnostics", nullptr, kWindowFlags);
    draw_overview(frame, actions);
    if (frame.snapshot != nullptr) {
        draw_motion(frame);
        draw_state(*frame.snapshot);
        draw_legs(*frame.snapshot);
        draw_actuation(*frame.snapshot);
    }
    ImGui::End();
    return actions;
}

void SimulationUi::render() {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool SimulationUi::wants_keyboard() const noexcept {
    return ImGui::GetCurrentContext() != nullptr &&
        ImGui::GetIO().WantCaptureKeyboard;
}

void SimulationUi::draw_overview(
    const SimulationUiFrame &frame,
    SimulationUiActions &actions) {
    ImGui::SeparatorText("Run");
    if (ImGui::Button(frame.paused ? "Resume" : "Pause")) {
        actions.toggle_pause = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) actions.reset = true;

    if (ImGui::BeginTable("run status", 2, ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Field");
        ImGui::TableSetupColumn("Value");
        table_value("Status", frame.paused ? "paused" : "running");
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("Time");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.3f s", frame.simulation_time);
        table_value("Phase", frame.phase);
        if (frame.case_name != nullptr) {
            table_value("Case", frame.case_name);
            table_value("Result", frame.case_finished ? "complete" : "running");
            if (frame.case_issue != nullptr &&
                std::string_view(frame.case_issue) != "none") {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted("Issue");
                ImGui::TableSetColumnIndex(1);
                ImGui::TextColored(
                    ImVec4(1.0F, 0.45F, 0.35F, 1.0F),
                    "%s", frame.case_issue);
            }
        }
        ImGui::EndTable();
    }
}

void SimulationUi::draw_motion(const SimulationUiFrame &frame) {
    const auto &snapshot = *frame.snapshot;
    ImGui::SeparatorText("Motion");
    if (ImGui::BeginTable("motion status", 2, ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Field");
        ImGui::TableSetupColumn("Value");
        table_value("System", bc_system_state_name(
            snapshot.state_machine.system));
        table_value("Motion", bc_motion_state_name(
            snapshot.state_machine.motion));
        table_value("Forward", bc_forward_state_name(
            snapshot.state_machine.forward));
        table_value("Alignment", bc_chassis_alignment_name(
            snapshot.state_machine.alignment));
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("Tick");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%u", snapshot.tick_count);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("Mapped velocity");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("% .3f m/s", snapshot.mapped_forward_velocity);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("Heading error");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("% .3f rad", snapshot.heading_error);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("Yaw ref acceleration");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("% .3f rad/s^2", snapshot.yaw_acceleration_reference);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("Gimbal encoder");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("% .3f rad", snapshot.gimbal.relative_yaw);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("Gimbal encoder rate");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("% .3f rad/s", snapshot.gimbal.relative_yaw_rate);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("Gimbal world");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("% .3f rad", frame.virtual_gimbal.world_yaw);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("Gimbal world rate");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("% .3f rad/s", frame.virtual_gimbal.world_yaw_rate);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("Roll");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("% .3f rad", snapshot.roll);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("Roll rate");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("% .3f rad/s", snapshot.roll_rate);
        ImGui::EndTable();
    }
}

void SimulationUi::draw_state(const bc_controller_snapshot_t &snapshot) {
    if (!ImGui::CollapsingHeader(
            "State / Reference", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    constexpr ImGuiTableFlags kFlags =
        ImGuiTableFlags_BordersInnerV |
        ImGuiTableFlags_RowBg |
        ImGuiTableFlags_SizingStretchProp;
    if (!ImGui::BeginTable("state", 4, kFlags)) return;
    ImGui::TableSetupColumn("State");
    ImGui::TableSetupColumn("Value");
    ImGui::TableSetupColumn("Reference");
    ImGui::TableSetupColumn("Error");
    ImGui::TableHeadersRow();
    for (int index = 0; index < BC_STATE_NUM; ++index) {
        const float value = snapshot.state.value[index];
        const float reference = snapshot.state_reference.value[index];
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(kStateNames[index]);
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("% .3f", value);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("% .3f", reference);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("% .3f", reference - value);
    }
    ImGui::EndTable();
}

void SimulationUi::draw_legs(const bc_controller_snapshot_t &snapshot) {
    if (!ImGui::CollapsingHeader("Leg kinematics")) return;
    constexpr ImGuiTableFlags kFlags =
        ImGuiTableFlags_BordersInnerV |
        ImGuiTableFlags_RowBg |
        ImGuiTableFlags_SizingStretchProp;
    if (!ImGui::BeginTable("legs", 5, kFlags)) return;
    ImGui::TableSetupColumn("Side");
    ImGui::TableSetupColumn("L");
    ImGui::TableSetupColumn("dL");
    ImGui::TableSetupColumn("Angle");
    ImGui::TableSetupColumn("Rate");
    ImGui::TableHeadersRow();
    constexpr const char *kSideNames[BC_SIDE_NUM] = {"Left", "Right"};
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const auto &leg = snapshot.leg[side];
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(kSideNames[side]);
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.3f", leg.length);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("% .3f", leg.length_velocity);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("% .3f", leg.angle_body);
        ImGui::TableSetColumnIndex(4);
        ImGui::Text("% .3f", leg.angular_velocity);
    }
    ImGui::EndTable();
}

void SimulationUi::draw_actuation(
    const bc_controller_snapshot_t &snapshot) {
    if (!ImGui::CollapsingHeader("Actuation")) return;
    constexpr ImGuiTableFlags kFlags =
        ImGuiTableFlags_BordersInnerV |
        ImGuiTableFlags_RowBg |
        ImGuiTableFlags_SizingStretchProp;
    if (!ImGui::BeginTable("actuation", 3, kFlags)) return;
    ImGui::TableSetupColumn("Channel");
    ImGui::TableSetupColumn("Request");
    ImGui::TableSetupColumn("Applied");
    ImGui::TableHeadersRow();
    draw_torque_row(
        "L front",
        snapshot.actuation_request.leg[BC_L].joint_torque[BC_FRONT],
        snapshot.actuation.leg[BC_L].joint_torque[BC_FRONT]);
    draw_torque_row(
        "L rear",
        snapshot.actuation_request.leg[BC_L].joint_torque[BC_REAR],
        snapshot.actuation.leg[BC_L].joint_torque[BC_REAR]);
    draw_torque_row(
        "L wheel",
        snapshot.actuation_request.wheel_torque[BC_L],
        snapshot.actuation.wheel_torque[BC_L]);
    draw_torque_row(
        "R front",
        snapshot.actuation_request.leg[BC_R].joint_torque[BC_FRONT],
        snapshot.actuation.leg[BC_R].joint_torque[BC_FRONT]);
    draw_torque_row(
        "R rear",
        snapshot.actuation_request.leg[BC_R].joint_torque[BC_REAR],
        snapshot.actuation.leg[BC_R].joint_torque[BC_REAR]);
    draw_torque_row(
        "R wheel",
        snapshot.actuation_request.wheel_torque[BC_R],
        snapshot.actuation.wheel_torque[BC_R]);
    ImGui::EndTable();
}

} // namespace balance::sim
