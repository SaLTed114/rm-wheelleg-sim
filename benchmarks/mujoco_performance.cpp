#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "balance/math_utils.h"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "performance_scenario.hpp"
#include "simulation_runner.hpp"

namespace {

constexpr double kTimestepSeconds = 0.001;
constexpr std::size_t kDefaultTraceStride = 10U;
constexpr double kMinimumLegLength = 0.13;
constexpr double kMaximumLegLength = 0.20;
constexpr double kForwardBaseTolerance = 0.10;
constexpr double kYawBaseTolerance = 0.20;
constexpr double kRelativeTrackingTolerance = 0.10;
constexpr double kSaturationTolerance = 1.0e-4;

using Axis = balance::sim::PerformanceAxis;
using CaseSpec = balance::sim::PerformanceCaseSpec;
using ContactState = balance::sim::PerformanceContactState;

struct ErrorAccumulator {
    double sum{};
    double squared_sum{};
    std::size_t count{};

    void add(const double value) {
        sum += value;
        squared_sum += value * value;
        ++count;
    }

    [[nodiscard]] double mean() const {
        return count == 0U ? 0.0 : sum / static_cast<double>(count);
    }

    [[nodiscard]] double rms() const {
        return count == 0U ? 0.0 :
            std::sqrt(squared_sum / static_cast<double>(count));
    }
};

struct CaseResult {
    CaseSpec spec;
    double leg_length_target{};
    bool completed{};
    bool balance_engaged{};
    bool finite{true};
    bool leg_length_valid{true};
    bool tracked{};
    bool settled{};
    std::string issue{"none"};
    std::string issue_phase{"none"};
    std::size_t monitored_steps{};
    std::size_t both_wheel_steps{};
    std::size_t other_contact_steps{};
    double maximum_pitch{};
    double maximum_roll{};
    double maximum_leg_common{};
    double maximum_leg_difference{};
    std::array<double, BC_SIDE_NUM> minimum_vertical_projection{{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
    }};
    double initial_position_error{};
    bool initial_position_error_captured{};
    ErrorAccumulator tracking_error;
    ErrorAccumulator settle_forward;
    ErrorAccumulator settle_yaw;
    std::array<double, BC_SIDE_NUM> peak_raw_wheel{};
    std::array<std::array<double, BC_JOINT_NUM>, BC_SIDE_NUM>
        peak_raw_joint{};
    std::array<std::size_t, BC_SIDE_NUM> wheel_saturation_steps{};
    std::array<std::array<std::size_t, BC_JOINT_NUM>, BC_SIDE_NUM>
        joint_saturation_steps{};

    [[nodiscard]] double contact_ratio() const {
        return monitored_steps == 0U ? 0.0 :
            static_cast<double>(both_wheel_steps) /
                static_cast<double>(monitored_steps);
    }

    [[nodiscard]] double wheel_saturation_ratio(const int side) const {
        return monitored_steps == 0U ? 0.0 :
            static_cast<double>(wheel_saturation_steps[side]) /
                static_cast<double>(monitored_steps);
    }

    [[nodiscard]] double joint_saturation_ratio(
        const int side, const int joint
    ) const {
        return monitored_steps == 0U ? 0.0 :
            static_cast<double>(joint_saturation_steps[side][joint]) /
                static_cast<double>(monitored_steps);
    }
};

bc_controller_config_t controller_config(
    const std::optional<double> leg_length
) {
    bc_controller_config_t config{};
    bc_controller_default_config(&config);
    if (leg_length) {
        if (!std::isfinite(*leg_length) || *leg_length <= 0.0) {
            throw std::invalid_argument(
                "leg length must be finite and positive");
        }
        config.motion.leg_length = static_cast<float>(*leg_length);
    }
    return config;
}

const char *axis_name(const Axis axis) {
    return balance::sim::performance_axis_name(axis);
}

bool finite_actuation(const bc_actuation_t &actuation) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (!std::isfinite(actuation.wheel_torque[side])) return false;
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            if (!std::isfinite(
                    actuation.leg[side].joint_torque[joint])) {
                return false;
            }
        }
    }
    return true;
}

class PerformanceBenchmark {
public:
    PerformanceBenchmark(
        const std::filesystem::path &model_path,
        const std::filesystem::path &output_directory,
        const std::optional<double> leg_length,
        const std::size_t trace_stride
    ) : plant_(model_path, kTimestepSeconds),
        adapter_(plant_.model()),
        runner_(plant_, adapter_, controller_config(leg_length)),
        contact_monitor_(plant_.model()),
        trace_stride_(trace_stride) {
        const bc_controller_config_t config = controller_config(leg_length);
        leg_length_target_ = config.motion.leg_length;
        std::filesystem::create_directories(output_directory);
        summary_.open(output_directory / "summary.csv", std::ios::trunc);
        trace_.open(output_directory / "trace.csv", std::ios::trunc);
        if (!summary_ || !trace_) {
            throw std::runtime_error(
                "failed to open performance benchmark output files");
        }

        const int base_joint = mj_name2id(
            &plant_.model(), mjOBJ_JOINT, "base_free_joint");
        if (base_joint < 0) {
            throw std::runtime_error(
                "missing MuJoCo object 'base_free_joint'");
        }
        base_qpos_ = plant_.model().jnt_qposadr[base_joint];
        base_dof_ = plant_.model().jnt_dofadr[base_joint];

        write_summary_header();
        write_trace_header();
    }

    CaseResult run(const CaseSpec &spec) {
        runner_.reset();
        sample_index_ = 0U;
        CaseResult result{};
        result.spec = spec;
        result.leg_length_target = leg_length_target_;
        balance::sim::PerformanceScenario scenario(spec);
        scenario.reset(plant_.data().time);

        while (!scenario.finished()) {
            result.balance_engaged = result.balance_engaged ||
                runner_.snapshot().state_machine.motion ==
                    BC_MOTION_BALANCE_ENGAGING;
            scenario.update(runner_.snapshot(), plant_.data().time);
            if (scenario.finished()) break;

            CaseResult *monitored = scenario.monitored() ? &result : nullptr;
            if (!step(
                    spec, scenario.phase_name(), scenario.command(),
                    monitored, scenario.tracking_evaluation(),
                    scenario.settle_evaluation())) {
                finish_result(result);
                return result;
            }
        }

        result.completed = true;
        if (!result.balance_engaged) {
            result.issue = "balance_not_engaged";
            result.issue_phase = "engaging";
        }
        finish_result(result);
        return result;
    }

    void write_summary(const CaseResult &result) {
        summary_ << std::setprecision(10)
                 << result.spec.name << ','
                 << axis_name(result.spec.axis) << ','
                 << result.spec.target << ','
                 << result.spec.command_rate << ','
                 << result.leg_length_target << ','
                 << result.completed << ','
                 << result.balance_engaged << ','
                 << result.leg_length_valid << ','
                 << result.finite << ','
                 << result.tracked << ','
                 << result.settled << ','
                 << result.issue << ','
                 << result.issue_phase << ','
                 << result.contact_ratio() << ','
                 << result.other_contact_steps << ','
                 << result.maximum_pitch * 180.0 / BC_PI << ','
                 << result.maximum_roll * 180.0 / BC_PI << ','
                 << result.maximum_leg_common * 180.0 / BC_PI << ','
                 << result.maximum_leg_difference * 180.0 / BC_PI << ','
                 << result.minimum_vertical_projection[BC_L] << ','
                 << result.minimum_vertical_projection[BC_R] << ','
                 << result.initial_position_error << ','
                 << result.tracking_error.mean() << ','
                 << result.tracking_error.rms() << ','
                 << result.settle_forward.mean() << ','
                 << result.settle_forward.rms() << ','
                 << result.settle_yaw.mean() << ','
                 << result.settle_yaw.rms();
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            summary_ << ',' << result.peak_raw_wheel[side];
        }
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                summary_ << ',' << result.peak_raw_joint[side][joint];
            }
        }
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            summary_ << ',' << result.wheel_saturation_ratio(side);
        }
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                summary_ << ','
                         << result.joint_saturation_ratio(side, joint);
            }
        }
        summary_ << '\n';
        summary_.flush();
    }

private:
    bool step(
        const CaseSpec &spec, const char *phase,
        const bc_operator_command_t &command, CaseResult *result,
        const bool evaluate_tracking, const bool evaluate_settle
    ) {
        runner_.step(command);
        const ContactState contact = contact_monitor_.read(plant_.data());
        if (sample_index_ % trace_stride_ == 0U) {
            write_trace(spec, phase, command, contact);
        }
        ++sample_index_;

        if (result == nullptr) return true;
        return collect(
            *result, phase, contact,
            evaluate_tracking, evaluate_settle);
    }

    bool collect(
        CaseResult &result, const char *phase,
        const ContactState &contact,
        const bool evaluate_tracking, const bool evaluate_settle
    ) const {
        const bc_controller_snapshot_t &snapshot = runner_.snapshot();
        ++result.monitored_steps;
        if (contact.wheel[BC_L] && contact.wheel[BC_R]) {
            ++result.both_wheel_steps;
        }
        if (contact.other) ++result.other_contact_steps;

        bool finite = std::isfinite(snapshot.roll) &&
            std::isfinite(snapshot.roll_rate) &&
            finite_actuation(snapshot.actuation_request) &&
            finite_actuation(snapshot.actuation);
        for (int index = 0; index < BC_STATE_NUM; ++index) {
            finite = finite && std::isfinite(snapshot.state.value[index]) &&
                std::isfinite(snapshot.state_reference.value[index]);
        }

        const double pitch = std::abs(static_cast<double>(
            snapshot.state.value[BC_STATE_THETA_B]));
        const double roll = std::abs(static_cast<double>(snapshot.roll));
        result.maximum_pitch = std::max(result.maximum_pitch, pitch);
        result.maximum_roll = std::max(result.maximum_roll, roll);
        result.finite = result.finite && finite;

        if (!result.initial_position_error_captured) {
            result.initial_position_error =
                snapshot.state_reference.value[BC_STATE_S] -
                snapshot.state.value[BC_STATE_S];
            result.initial_position_error_captured = true;
        }
        const double theta_left =
            snapshot.state.value[BC_STATE_THETA_L];
        const double theta_right =
            snapshot.state.value[BC_STATE_THETA_R];
        result.maximum_leg_common = std::max(
            result.maximum_leg_common,
            std::abs(0.5 * (theta_left + theta_right)));
        result.maximum_leg_difference = std::max(
            result.maximum_leg_difference,
            std::abs(0.5 * (theta_left - theta_right)));

        const int theta_index[BC_SIDE_NUM] = {
            BC_STATE_THETA_L, BC_STATE_THETA_R,
        };

        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            const double length = snapshot.leg[side].length;
            result.leg_length_valid = result.leg_length_valid &&
                std::isfinite(length) &&
                length >= kMinimumLegLength &&
                length <= kMaximumLegLength;
            result.minimum_vertical_projection[side] = std::min(
                result.minimum_vertical_projection[side],
                length * std::cos(
                    snapshot.state.value[theta_index[side]]));

            const double raw_wheel =
                snapshot.actuation_request.wheel_torque[side];
            const double final_wheel =
                snapshot.actuation.wheel_torque[side];
            result.peak_raw_wheel[side] = std::max(
                result.peak_raw_wheel[side], std::abs(raw_wheel));
            if (std::abs(raw_wheel - final_wheel) >
                    kSaturationTolerance) {
                ++result.wheel_saturation_steps[side];
            }

            for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                const double raw_joint = snapshot.actuation_request
                    .leg[side].joint_torque[joint];
                const double final_joint = snapshot.actuation
                    .leg[side].joint_torque[joint];
                result.peak_raw_joint[side][joint] = std::max(
                    result.peak_raw_joint[side][joint],
                    std::abs(raw_joint));
                if (std::abs(raw_joint - final_joint) >
                        kSaturationTolerance) {
                    ++result.joint_saturation_steps[side][joint];
                }
            }
        }

        if (evaluate_tracking) {
            const int index = result.spec.axis == Axis::forward ?
                BC_STATE_DS : BC_STATE_DPSI;
            result.tracking_error.add(
                snapshot.state.value[index] - result.spec.target);
        }
        if (evaluate_settle) {
            result.settle_forward.add(
                snapshot.state.value[BC_STATE_DS]);
            result.settle_yaw.add(
                snapshot.state.value[BC_STATE_DPSI]);
        }

        const std::string issue =
            balance::sim::performance_diagnostic_issue(snapshot, contact);
        if (!issue.empty() && result.issue == "none") {
            result.issue = issue;
            result.issue_phase = phase;
        }
        return issue != "non_finite_telemetry";
    }

    void finish_result(CaseResult &result) const {
        const double tracking_tolerance = result.spec.axis == Axis::forward ?
            std::max(
                kForwardBaseTolerance,
                kRelativeTrackingTolerance * std::abs(result.spec.target)) :
            std::max(
                kYawBaseTolerance,
                kRelativeTrackingTolerance * std::abs(result.spec.target));

        result.tracked = result.completed && result.balance_engaged &&
            result.tracking_error.count != 0U &&
            std::abs(result.tracking_error.mean()) <= tracking_tolerance &&
            result.tracking_error.rms() <= tracking_tolerance;
        result.settled = result.completed && result.balance_engaged &&
            result.settle_forward.count != 0U &&
            std::abs(result.settle_forward.mean()) <=
                kForwardBaseTolerance &&
            result.settle_forward.rms() <= kForwardBaseTolerance &&
            std::abs(result.settle_yaw.mean()) <= kYawBaseTolerance &&
            result.settle_yaw.rms() <= kYawBaseTolerance;
    }

    void write_summary_header() {
        summary_
            << "case,axis,target,command_rate,leg_length_target,completed,"
               "balance_engaged,leg_length_valid,finite,tracked,settled,"
               "issue,issue_phase,"
               "wheel_contact_ratio,other_contact_steps,max_pitch_deg,"
               "max_roll_deg,max_leg_common_deg,max_leg_difference_deg,"
               "min_vertical_l,min_vertical_r,initial_s_error,"
               "tracking_mean_error,tracking_rmse,"
               "settle_mean_ds,settle_rmse_ds,settle_mean_dpsi,"
               "settle_rmse_dpsi,peak_raw_wheel_l,peak_raw_wheel_r,"
               "peak_raw_joint_l_front,peak_raw_joint_l_rear,"
               "peak_raw_joint_r_front,peak_raw_joint_r_rear,"
               "wheel_saturation_l,wheel_saturation_r,"
               "joint_saturation_l_front,joint_saturation_l_rear,"
               "joint_saturation_r_front,joint_saturation_r_rear\n";
    }

    void write_trace_header() {
        trace_ << "case,phase,simulation_time,command_rate,leg_length_target,"
                  "base_x,base_z,"
                  "base_forward_velocity,base_vertical_velocity,"
                  "command_forward,command_yaw,"
                  "system,motion,s,ds,psi,dpsi,theta_l,dtheta_l,theta_r,"
                  "dtheta_r,theta_b,dtheta_b,ref_s,ref_ds,ref_psi,"
                  "ref_dpsi,ref_theta_l,ref_dtheta_l,ref_theta_r,"
                  "ref_dtheta_r,ref_theta_b,ref_dtheta_b,roll,roll_rate,"
                  "leg_l_length,leg_l_angle,leg_l_length_rate,"
                  "leg_l_angle_rate,leg_r_length,leg_r_angle,"
                  "leg_r_length_rate,leg_r_angle_rate,raw_wheel_l,"
                  "raw_wheel_r,raw_joint_l_front,raw_joint_l_rear,"
                  "raw_joint_r_front,raw_joint_r_rear,wheel_l,wheel_r,"
                  "joint_l_front,joint_l_rear,joint_r_front,joint_r_rear,"
                  "contact_wheel_l,contact_wheel_r,other_contact,"
                  "normal_force_l,normal_force_r\n";
    }

    void write_trace(
        const CaseSpec &spec, const char *phase,
        const bc_operator_command_t &command,
        const ContactState &contact
    ) {
        const bc_controller_snapshot_t &snapshot = runner_.snapshot();
        const double yaw = snapshot.state.value[BC_STATE_PSI];
        const double base_forward_velocity =
            plant_.data().qvel[base_dof_] * std::cos(yaw) +
            plant_.data().qvel[base_dof_ + 1] * std::sin(yaw);
        trace_ << std::setprecision(10)
               << spec.name << ',' << phase << ','
               << plant_.data().time << ','
               << spec.command_rate << ','
               << leg_length_target_ << ','
               << plant_.data().qpos[base_qpos_] << ','
               << plant_.data().qpos[base_qpos_ + 2] << ','
               << base_forward_velocity << ','
               << plant_.data().qvel[base_dof_ + 2] << ','
               << command.forward_velocity << ',' << command.yaw_rate << ','
               << snapshot.state_machine.system << ','
               << snapshot.state_machine.motion;
        for (const float value : snapshot.state.value) {
            trace_ << ',' << value;
        }
        for (const float value : snapshot.state_reference.value) {
            trace_ << ',' << value;
        }
        trace_ << ',' << snapshot.roll << ',' << snapshot.roll_rate;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            const bc_leg_kinematics_t &leg = snapshot.leg[side];
            trace_ << ',' << leg.length << ',' << leg.angle_body
                   << ',' << leg.length_velocity
                   << ',' << leg.angular_velocity;
        }
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            trace_ << ','
                   << snapshot.actuation_request.wheel_torque[side];
        }
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                trace_ << ',' << snapshot.actuation_request.leg[side]
                    .joint_torque[joint];
            }
        }
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            trace_ << ',' << snapshot.actuation.wheel_torque[side];
        }
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                trace_ << ',' << snapshot.actuation.leg[side]
                    .joint_torque[joint];
            }
        }
        trace_ << ',' << contact.wheel[BC_L]
               << ',' << contact.wheel[BC_R]
               << ',' << contact.other
               << ',' << contact.wheel_normal_force[BC_L]
               << ',' << contact.wheel_normal_force[BC_R] << '\n';
    }

    balance::sim::MujocoPlant plant_;
    balance::sim::MujocoAdapter adapter_;
    balance::sim::SimulationRunner runner_;
    balance::sim::PerformanceContactMonitor contact_monitor_;
    int base_qpos_{};
    int base_dof_{};
    double leg_length_target_{};
    std::ofstream summary_;
    std::ofstream trace_;
    std::size_t trace_stride_{};
    std::size_t sample_index_{};
};

void print_result(const CaseResult &result) {
    double maximum_wheel_saturation = 0.0;
    double maximum_joint_saturation = 0.0;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        maximum_wheel_saturation = std::max(
            maximum_wheel_saturation,
            result.wheel_saturation_ratio(side));
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            maximum_joint_saturation = std::max(
                maximum_joint_saturation,
                result.joint_saturation_ratio(side, joint));
        }
    }

    std::cout << std::left << std::setw(22) << result.spec.name
              << " complete=" << result.completed
              << " engaged=" << result.balance_engaged
              << " leg_range=" << result.leg_length_valid
              << " finite=" << result.finite
              << " tracked=" << result.tracked
              << " settled=" << result.settled
              << " error=" << std::setw(10)
              << result.tracking_error.mean()
              << " rmse=" << std::setw(10)
              << result.tracking_error.rms()
              << " pitch=" << std::setw(8)
              << result.maximum_pitch * 180.0 / BC_PI
              << " roll=" << std::setw(8)
              << result.maximum_roll * 180.0 / BC_PI
              << " wheel_sat=" << maximum_wheel_saturation
              << " joint_sat=" << maximum_joint_saturation;
    if (result.issue != "none") {
        std::cout << " issue=" << result.issue
                  << '@' << result.issue_phase;
    }
    std::cout << '\n';
}

} // namespace

int main(int argc, char **argv) {
    enum class Suite {
        baseline,
        forward_acceleration,
        yaw_acceleration,
    };

    Suite suite = Suite::baseline;
    std::optional<double> leg_length;
    std::optional<std::size_t> trace_stride;
    const CaseSpec *selected_case = nullptr;
    bool arguments_valid = argc >= 3 && (argc - 3) % 2 == 0;
    for (int index = 3; arguments_valid && index < argc; index += 2) {
        const std::string option = argv[index];
        const std::string value = argv[index + 1];
        if (option == "--suite" && suite == Suite::baseline &&
            value == "forward-acceleration") {
            suite = Suite::forward_acceleration;
        } else if (option == "--suite" && suite == Suite::baseline &&
                   value == "yaw-acceleration") {
            suite = Suite::yaw_acceleration;
        } else if (option == "--leg-length" && !leg_length) {
            std::size_t consumed = 0U;
            try {
                leg_length = std::stod(value, &consumed);
            } catch (const std::exception &) {
                arguments_valid = false;
                break;
            }
            arguments_valid = consumed == value.size();
        } else if (option == "--trace-stride" && !trace_stride) {
            std::size_t consumed = 0U;
            try {
                trace_stride = std::stoul(value, &consumed);
            } catch (const std::exception &) {
                arguments_valid = false;
                break;
            }
            arguments_valid = consumed == value.size() && *trace_stride > 0U;
        } else if (option == "--case" && selected_case == nullptr) {
            selected_case = balance::sim::find_performance_case(value);
            arguments_valid = selected_case != nullptr;
        } else {
            arguments_valid = false;
        }
    }
    arguments_valid = arguments_valid &&
        !(suite != Suite::baseline && selected_case != nullptr);
    if (!arguments_valid) {
        std::cerr
            << "usage: rm_balance_performance <model.xml> <output-directory> "
               "[--suite forward-acceleration|yaw-acceleration] "
               "[--case <case-name>] "
               "[--leg-length <metres>] [--trace-stride <steps>]\n";
        return EXIT_FAILURE;
    }

    try {
        PerformanceBenchmark benchmark(
            argv[1], argv[2], leg_length,
            trace_stride.value_or(kDefaultTraceStride));
        std::vector<CaseSpec> cases;
        if (selected_case != nullptr) {
            cases.push_back(*selected_case);
        } else if (suite == Suite::forward_acceleration) {
            const auto &acceleration_cases =
                balance::sim::forward_acceleration_cases();
            cases.assign(
                acceleration_cases.begin(), acceleration_cases.end());
        } else if (suite == Suite::yaw_acceleration) {
            const auto &acceleration_cases =
                balance::sim::yaw_acceleration_cases();
            cases.assign(
                acceleration_cases.begin(), acceleration_cases.end());
        } else {
            const auto &baseline_cases = balance::sim::performance_cases();
            cases.assign(baseline_cases.begin(), baseline_cases.end());
        }
        for (const CaseSpec &spec : cases) {
            CaseResult result = benchmark.run(spec);
            benchmark.write_summary(result);
            print_result(result);
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_performance: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
