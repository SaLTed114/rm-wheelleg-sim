#include "simulation_app.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include "common/common_diagnostics.hpp"
#include "common/simulation_sample.hpp"
#include "drop/drop_scenario.hpp"
#include "drop/platform_drop.hpp"
#include "input/interactive_scenario.hpp"
#include "interactive_trace.hpp"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "mujoco_viewer.hpp"
#include "performance/performance_scenario.hpp"
#include "simulation_runner.hpp"
#include "simulation_ui.hpp"

namespace balance::sim {
namespace {

constexpr double kTimestepSeconds = 0.001;
constexpr double kMaxFrameTimeSeconds = 0.1;

class SimulationApp {
public:
    explicit SimulationApp(const SimulationAppOptions &options)
        : options_(options),
          plant_(options.model_path, kTimestepSeconds),
          adapter_(plant_.model()),
          runner_(plant_, adapter_, controller_config(options)),
          viewer_(plant_.model()),
          ui_(viewer_.native_window()),
          sampler_(plant_.model()),
          interactive_(options.keyboard_drive ?
              InteractiveMode::keyboard : InteractiveMode::demo) {
        if (options_.performance_case != nullptr) {
            performance_.emplace(*options_.performance_case);
        }
        if (options_.drop_case != nullptr) {
            auto spec = *options_.drop_case;
            if (options_.drop_wheel_clearance) {
                spec.wheel_clearance = *options_.drop_wheel_clearance;
            }
            drop_.emplace(spec, plant_.model());
        }
        if (options_.platform_drop_case != nullptr) {
            platform_drop_.emplace(
                *options_.platform_drop_case, plant_.model());
        }
        if (options_.trace_path) {
            trace_ = std::make_unique<InteractiveTraceWriter>(
                *options_.trace_path);
            std::cout << "interactive trace: "
                      << std::filesystem::absolute(*options_.trace_path)
                      << '\n';
        }
        reset();
    }

    void run() {
        using Clock = std::chrono::steady_clock;
        auto previous_time = Clock::now();
        double accumulated_time = 0.0;

        while (!viewer_.should_close()) {
            viewer_.poll_events();
            const auto current_time = Clock::now();
            const std::chrono::duration<double> frame_time =
                current_time - previous_time;
            previous_time = current_time;

            const SimulationUiActions ui_actions = ui_.draw(ui_frame());
            if (ui_actions.toggle_pause ||
                viewer_.consume_pause_toggle()) {
                paused_ = !paused_;
            }
            if (ui_actions.reset || viewer_.consume_reset_request()) {
                reset();
                accumulated_time = 0.0;
            }

            if (paused_ || case_finished_) {
                accumulated_time = 0.0;
            } else {
                accumulated_time += std::clamp(
                    frame_time.count(), 0.0, kMaxFrameTimeSeconds);
                while (accumulated_time >= plant_.timestep()) {
                    const bool keep_stepping = platform_drop_ ?
                        step_platform_drop() : drop_ ?
                        step_drop() : performance_ ?
                            step_performance() : step_interactive();
                    if (!keep_stepping) {
                        accumulated_time = 0.0;
                        break;
                    }
                    accumulated_time -= plant_.timestep();
                }
            }

            viewer_.set_virtual_gimbal_heading(
                displayed_gimbal_.world_yaw,
                runner_.snapshot().state_machine.motion == BC_MOTION_ACTIVE);
            viewer_.render_scene(plant_.data(), ui_.sidebar_width());
            ui_.render();
            viewer_.present();
        }
    }

private:
    static bc_controller_config_t controller_config(
        const SimulationAppOptions &options) {
        bc_controller_config_t config{};
        bc_controller_default_config(&config);
        if (options.leg_length) {
            config.motion.leg_length = *options.leg_length;
        }
        if (options.yaw_acceleration_feedforward_scale) {
            config.control.lqr_compensation.
                yaw_acceleration_feedforward_scale =
                    *options.yaw_acceleration_feedforward_scale;
        }
        if (options.platform_drop_case != nullptr) {
            config.motion.leg_length = static_cast<float>(
                options.platform_drop_case->leg_length);
        }
        return config;
    }

    void reset() {
        if (trace_) trace_->flush();
        ++reset_index_;
        runner_.reset();
        if (options_.keyboard_drive) plant_.configure_keyboard_course();
        if (performance_) performance_->reset(plant_.data().time);
        if (drop_) drop_->reset();
        if (platform_drop_) platform_drop_->reset(plant_);
        interactive_.reset(runner_.snapshot());
        phase_ = bc_system_state_name(
            runner_.snapshot().state_machine.system);
        displayed_gimbal_ = {};
        case_finished_ = false;
        case_balance_engaged_ = false;
        case_issue_ = "none";
    }

    bool step_interactive() {
        const KeyboardDriveInput keyboard = ui_.wants_keyboard() ?
            KeyboardDriveInput{} : viewer_.keyboard_drive_input();
        const auto &frame = interactive_.update(
            runner_.snapshot(),
            keyboard,
            plant_.data().time,
            static_cast<float>(plant_.timestep()));
        phase_ = frame.phase;
        displayed_gimbal_ = frame.gimbal;
        runner_.step_with_gimbal_heading(
            frame.command,
            frame.gimbal.world_yaw,
            frame.gimbal.world_yaw_rate);
        if (trace_) {
            const auto sample = sampler_.read(
                plant_.data(), runner_.snapshot());
            trace_->write(
                reset_index_, phase_, keyboard, frame, sample,
                sampler_.read_imu_motion(plant_.data()), runner_.feedback());
        }
        return true;
    }

    bool step_performance() {
        const auto &snapshot = runner_.snapshot();
        case_balance_engaged_ = case_balance_engaged_ ||
            snapshot.state_machine.motion == BC_MOTION_ACTIVE;
        performance_->update(snapshot, plant_.data().time);
        phase_ = performance_->phase_name();
        displayed_gimbal_ = performance_->gimbal();

        if (performance_->finished()) {
            finish_performance_case();
            return false;
        }

        const bool monitored = performance_->monitored();
        runner_.step_with_gimbal_heading(
            performance_->command(),
            performance_->gimbal().world_yaw,
            performance_->gimbal().world_yaw_rate);
        if (!monitored) return true;

        const auto sample = sampler_.read(
            plant_.data(), runner_.snapshot());
        const std::string issue =
            benchmark::common_diagnostic_issue(sample);
        if (!issue.empty() && case_issue_ == "none") {
            case_issue_ = issue;
        }
        if (issue == "non_finite_telemetry") {
            case_finished_ = true;
            return false;
        }
        return true;
    }

    bool step_drop() {
        drop_->step(plant_, runner_, sampler_);
        phase_ = drop_->phase_name();
        case_balance_engaged_ = drop_->balance_engaged();
        if (std::string_view(drop_->issue()) != "none" &&
            case_issue_ == "none") {
            case_issue_ = drop_->issue();
        }

        if (drop_->phase() != benchmark::DropPhase::disabled_settle &&
            drop_->phase() != benchmark::DropPhase::standing) {
            const auto sample = sampler_.read(
                plant_.data(), runner_.snapshot());
            const std::string issue =
                benchmark::common_diagnostic_issue(sample);
            const bool tuning_only = issue.rfind(
                "non_wheel_contact:", 0) == 0;
            if (!issue.empty() && !tuning_only && case_issue_ == "none") {
                case_issue_ = issue;
            }
            if (issue == "non_finite_telemetry") {
                case_finished_ = true;
                return false;
            }
        }

        if (drop_->finished()) {
            case_finished_ = true;
            return false;
        }
        return true;
    }

    bool step_platform_drop() {
        platform_drop_->step(plant_, runner_, sampler_);
        phase_ = platform_drop_->phase_name();
        case_balance_engaged_ = platform_drop_->balance_engaged();
        if (std::string_view(platform_drop_->issue()) != "none" &&
            case_issue_ == "none") {
            case_issue_ = platform_drop_->issue();
        }

        const auto sample = sampler_.read(
            plant_.data(), runner_.snapshot());
        if (!benchmark::controller_snapshot_is_finite(sample.controller)) {
            case_issue_ = "non_finite_telemetry";
            case_finished_ = true;
            return false;
        }
        if (platform_drop_->finished()) {
            case_finished_ = true;
            return false;
        }
        return true;
    }

    void finish_performance_case() {
        case_finished_ = true;
        if (!case_balance_engaged_) case_issue_ = "balance_not_engaged";
    }

    [[nodiscard]] SimulationUiFrame ui_frame() const {
        const char *case_name = nullptr;
        if (platform_drop_) case_name = platform_drop_->name().c_str();
        else if (drop_) case_name = drop_->name().c_str();
        else if (performance_) {
            case_name = options_.performance_case->name.data();
        }
        return {
            &runner_.snapshot(),
            displayed_gimbal_,
            plant_.data().time,
            phase_.c_str(),
            case_name,
            case_issue_.c_str(),
            paused_,
            case_finished_,
        };
    }

    SimulationAppOptions options_;
    MujocoPlant plant_;
    MujocoAdapter adapter_;
    SimulationRunner runner_;
    MujocoViewer viewer_;
    SimulationUi ui_;
    benchmark::SimulationSampler sampler_;
    InteractiveScenario interactive_;
    std::optional<benchmark::DropScenario> drop_;
    std::optional<benchmark::PlatformDropScenario> platform_drop_;
    std::optional<benchmark::PerformanceScenario> performance_;
    std::unique_ptr<InteractiveTraceWriter> trace_;
    VirtualGimbalState displayed_gimbal_{};
    std::string phase_{"off"};
    std::string case_issue_{"none"};
    bool paused_{};
    bool case_finished_{};
    bool case_balance_engaged_{};
    std::uint32_t reset_index_{};
};

} // namespace

void run_simulation_app(const SimulationAppOptions &options) {
    SimulationApp app(options);
    app.run();
}

} // namespace balance::sim
