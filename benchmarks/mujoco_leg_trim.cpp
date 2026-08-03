#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "balance/math_utils.h"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "performance_scenario.hpp"
#include "simulation_runner.hpp"

namespace {

constexpr double kTimestepSeconds = 0.001;
constexpr double kDisabledSeconds = 2.0;
constexpr double kEngagementTimeoutSeconds = 8.0;
constexpr double kBalanceSeconds = 8.0;
constexpr double kEvaluationSeconds = 3.0;
constexpr std::size_t kDefaultTraceStride = 10U;
constexpr double kSaturationTolerance = 1.0e-4;

using ContactState = balance::sim::PerformanceContactState;

struct Accumulator {
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

struct LinearTrend {
    double sum_time{};
    double sum_value{};
    double sum_time_squared{};
    double sum_time_value{};
    std::size_t count{};

    void add(const double time, const double value) {
        sum_time += time;
        sum_value += value;
        sum_time_squared += time * time;
        sum_time_value += time * value;
        ++count;
    }

    [[nodiscard]] double slope() const {
        const double denominator =
            static_cast<double>(count) * sum_time_squared -
            sum_time * sum_time;
        if (count < 2U || std::abs(denominator) < 1.0e-12) return 0.0;
        return (
            static_cast<double>(count) * sum_time_value -
            sum_time * sum_value) / denominator;
    }
};

struct ScanConfig {
    double leg_length{0.18};
    double offset_minimum_deg{-5.0};
    double offset_maximum_deg{15.0};
    double offset_step_deg{1.0};
    std::size_t trace_stride{kDefaultTraceStride};
};

struct CaseResult {
    double offset_deg{};
    bool engaged{};
    bool finite{true};
    bool candidate{};
    std::string issue{"none"};
    std::size_t evaluation_steps{};
    std::size_t both_wheel_steps{};
    std::size_t other_contact_steps{};
    std::size_t wheel_saturation_steps{};
    std::size_t joint_saturation_steps{};
    double maximum_position_error{};
    double maximum_raw_wheel{};
    double maximum_raw_joint{};
    double forward_displacement{};
    Accumulator pitch;
    Accumulator velocity;
    Accumulator base_velocity;
    Accumulator leg_common;
    Accumulator leg_difference;
    Accumulator wheel_common;
    Accumulator normal_force_left;
    Accumulator normal_force_right;
    LinearTrend velocity_trend;

    [[nodiscard]] double contact_ratio() const {
        return evaluation_steps == 0U ? 0.0 :
            static_cast<double>(both_wheel_steps) /
                static_cast<double>(evaluation_steps);
    }

    [[nodiscard]] double saturation_ratio() const {
        return evaluation_steps == 0U ? 0.0 :
            static_cast<double>(wheel_saturation_steps) /
                static_cast<double>(evaluation_steps);
    }

    [[nodiscard]] double joint_saturation_ratio() const {
        return evaluation_steps == 0U ? 0.0 :
            static_cast<double>(joint_saturation_steps) /
                static_cast<double>(evaluation_steps);
    }
};

bool finite_actuation(const bc_actuation_t &actuation) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (!std::isfinite(actuation.wheel_torque[side])) return false;
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            if (!std::isfinite(
                    actuation.leg[side].joint_torque[joint])) return false;
        }
    }
    return true;
}

class TrimScanner {
public:
    TrimScanner(
        const std::filesystem::path &model_path,
        const std::filesystem::path &output_directory,
        const ScanConfig &config
    ) : plant_(model_path, kTimestepSeconds),
        adapter_(plant_.model()),
        contact_monitor_(plant_.model()),
        config_(config) {
        std::filesystem::create_directories(output_directory);
        summary_.open(output_directory / "summary.csv", std::ios::trunc);
        trace_.open(output_directory / "trace.csv", std::ios::trunc);
        if (!summary_ || !trace_) {
            throw std::runtime_error("failed to open trim scan output files");
        }

        const int base_joint = mj_name2id(
            &plant_.model(), mjOBJ_JOINT, "base_free_joint");
        if (base_joint < 0) {
            throw std::runtime_error("missing MuJoCo object 'base_free_joint'");
        }
        base_qpos_ = plant_.model().jnt_qposadr[base_joint];
        base_dof_ = plant_.model().jnt_dofadr[base_joint];

        write_headers();
    }

    CaseResult run(const double offset_deg) {
        bc_controller_config_t controller_config{};
        bc_controller_default_config(&controller_config);
        controller_config.motion.leg_length =
            static_cast<float>(config_.leg_length);
        controller_config.control.lqr_compensation.leg_angle_trim =
            static_cast<float>(offset_deg * BC_PI / 180.0);
        controller_config.motion.position_feedback_enabled = 0U;

        balance::sim::SimulationRunner runner(
            plant_, adapter_, controller_config);
        runner.reset();

        CaseResult result{};
        result.offset_deg = offset_deg;
        bc_operator_command_t command{};
        while (plant_.data().time < kDisabledSeconds) {
            runner.step(command);
        }

        const double engagement_deadline =
            plant_.data().time + kEngagementTimeoutSeconds;
        command.system_enabled = 1U;
        while (plant_.data().time < engagement_deadline &&
               runner.snapshot().state_machine.motion !=
                   BC_MOTION_BALANCE_ENGAGING) {
            command.balance_restart = static_cast<uint8_t>(
                runner.snapshot().state_machine.system == BC_SYSTEM_OFF);
            runner.step(command);
        }
        command.balance_restart = 0U;
        result.engaged = runner.snapshot().state_machine.motion ==
            BC_MOTION_BALANCE_ENGAGING;
        if (!result.engaged) {
            result.issue = "engagement_timeout";
            return result;
        }

        const double balance_start = plant_.data().time;
        const double evaluation_start =
            balance_start + kBalanceSeconds - kEvaluationSeconds;
        const double balance_end = balance_start + kBalanceSeconds;
        double evaluation_x = 0.0;
        double evaluation_y = 0.0;
        double evaluation_yaw = 0.0;
        bool evaluation_started = false;
        std::size_t sample_index = 0U;

        while (plant_.data().time < balance_end) {
            runner.step(command);
            const ContactState contact =
                contact_monitor_.read(plant_.data());
            if (sample_index % config_.trace_stride == 0U) {
                write_trace(
                    runner.snapshot(), contact, offset_deg,
                    plant_.data().time - balance_start);
            }
            ++sample_index;

            const std::string issue =
                balance::sim::performance_diagnostic_issue(
                    runner.snapshot(), contact);
            if (issue == "non_finite_telemetry") {
                result.issue = issue;
                result.finite = false;
                break;
            }
            if (plant_.data().time < evaluation_start) continue;

            if (!issue.empty() && result.issue == "none") {
                result.issue = issue;
            }

            if (!evaluation_started) {
                evaluation_x = plant_.data().qpos[base_qpos_];
                evaluation_y = plant_.data().qpos[base_qpos_ + 1];
                evaluation_yaw =
                    runner.snapshot().state.value[BC_STATE_PSI];
                evaluation_started = true;
            }
            collect(
                result, runner.snapshot(), contact,
                plant_.data().time - evaluation_start);
        }

        if (evaluation_started) {
            const double delta_x =
                plant_.data().qpos[base_qpos_] - evaluation_x;
            const double delta_y =
                plant_.data().qpos[base_qpos_ + 1] - evaluation_y;
            result.forward_displacement =
                delta_x * std::cos(evaluation_yaw) +
                delta_y * std::sin(evaluation_yaw);
        }
        finish(result);
        return result;
    }

    void write_summary(const CaseResult &result) {
        summary_ << std::setprecision(10)
                 << config_.leg_length << ','
                 << result.offset_deg << ','
                 << result.engaged << ','
                 << result.finite << ','
                 << result.candidate << ','
                 << result.issue << ','
                 << result.contact_ratio() << ','
                 << result.other_contact_steps << ','
                 << result.saturation_ratio() << ','
                 << result.joint_saturation_ratio() << ','
                 << result.maximum_position_error << ','
                 << result.pitch.mean() * 180.0 / BC_PI << ','
                 << result.pitch.rms() * 180.0 / BC_PI << ','
                 << result.velocity.mean() << ','
                 << result.velocity.rms() << ','
                 << result.velocity_trend.slope() << ','
                 << result.base_velocity.mean() << ','
                 << result.base_velocity.rms() << ','
                 << result.forward_displacement << ','
                 << result.leg_common.mean() * 180.0 / BC_PI << ','
                 << result.leg_difference.mean() * 180.0 / BC_PI << ','
                 << result.wheel_common.mean() << ','
                 << result.maximum_raw_wheel << ','
                 << result.maximum_raw_joint << ','
                 << result.normal_force_left.mean() << ','
                 << result.normal_force_right.mean() << '\n';
        summary_.flush();
    }

private:
    [[nodiscard]] double base_forward_velocity(
        const bc_controller_snapshot_t &snapshot
    ) const {
        const double yaw = snapshot.state.value[BC_STATE_PSI];
        return plant_.data().qvel[base_dof_] * std::cos(yaw) +
            plant_.data().qvel[base_dof_ + 1] * std::sin(yaw);
    }

    void collect(
        CaseResult &result,
        const bc_controller_snapshot_t &snapshot,
        const ContactState &contact,
        const double evaluation_time
    ) const {
        ++result.evaluation_steps;
        if (contact.wheel[BC_L] && contact.wheel[BC_R]) {
            ++result.both_wheel_steps;
        }
        if (contact.other) ++result.other_contact_steps;

        const double pitch = snapshot.state.value[BC_STATE_THETA_B];
        const double velocity = snapshot.state.value[BC_STATE_DS];
        const double theta_left = snapshot.state.value[BC_STATE_THETA_L];
        const double theta_right = snapshot.state.value[BC_STATE_THETA_R];
        const double raw_left =
            snapshot.actuation_request.wheel_torque[BC_L];
        const double raw_right =
            snapshot.actuation_request.wheel_torque[BC_R];

        result.pitch.add(pitch);
        result.velocity.add(velocity);
        result.base_velocity.add(base_forward_velocity(snapshot));
        result.velocity_trend.add(evaluation_time, velocity);
        result.leg_common.add(0.5 * (theta_left + theta_right));
        result.leg_difference.add(0.5 * (theta_left - theta_right));
        result.wheel_common.add(0.5 * (raw_left + raw_right));
        result.normal_force_left.add(contact.wheel_normal_force[BC_L]);
        result.normal_force_right.add(contact.wheel_normal_force[BC_R]);
        result.maximum_position_error = std::max(
            result.maximum_position_error,
            std::abs(static_cast<double>(
                snapshot.state_reference.value[BC_STATE_S] -
                snapshot.state.value[BC_STATE_S])));
        result.maximum_raw_wheel = std::max({
            result.maximum_raw_wheel,
            std::abs(raw_left), std::abs(raw_right),
        });

        if (std::abs(raw_left - snapshot.actuation.wheel_torque[BC_L]) >
                kSaturationTolerance ||
            std::abs(raw_right - snapshot.actuation.wheel_torque[BC_R]) >
                kSaturationTolerance) {
            ++result.wheel_saturation_steps;
        }
        bool joint_saturated = false;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                const double raw = snapshot.actuation_request
                    .leg[side].joint_torque[joint];
                const double limited = snapshot.actuation
                    .leg[side].joint_torque[joint];
                result.maximum_raw_joint = std::max(
                    result.maximum_raw_joint, std::abs(raw));
                joint_saturated = joint_saturated ||
                    std::abs(raw - limited) > kSaturationTolerance;
            }
        }
        if (joint_saturated) ++result.joint_saturation_steps;
        result.finite = result.finite &&
            std::isfinite(pitch) && std::isfinite(velocity) &&
            std::isfinite(base_forward_velocity(snapshot)) &&
            finite_actuation(snapshot.actuation_request) &&
            finite_actuation(snapshot.actuation);
    }

    void finish(CaseResult &result) const {
        result.candidate = result.engaged && result.finite &&
            result.issue == "none" &&
            result.evaluation_steps != 0U &&
            result.contact_ratio() >= 0.99 &&
            result.other_contact_steps == 0U &&
            result.saturation_ratio() == 0.0 &&
            result.joint_saturation_ratio() == 0.0 &&
            std::abs(result.pitch.mean()) <= BC_PI / 180.0 &&
            std::abs(result.velocity.mean()) <= 0.02 &&
            std::abs(result.velocity_trend.slope()) <= 0.01;
    }

    void write_headers() {
        summary_
            << "leg_length_target,offset_deg,engaged,finite,candidate,issue,"
               "wheel_contact_ratio,other_contact_steps,"
               "wheel_saturation_ratio,joint_saturation_ratio,"
               "max_position_error,"
               "mean_pitch_deg,rms_pitch_deg,mean_ds,rms_ds,ds_slope,"
               "mean_base_velocity,rms_base_velocity,forward_displacement,"
               "mean_leg_common_deg,mean_leg_difference_deg,"
               "mean_wheel_common,max_raw_wheel,max_raw_joint,"
               "mean_normal_force_l,mean_normal_force_r\n";
        trace_
            << "offset_deg,time,base_x,base_z,base_forward_velocity,"
               "s,ds,theta_l,dtheta_l,theta_r,dtheta_r,theta_b,dtheta_b,"
               "ref_s,ref_ds,ref_theta_l,ref_theta_r,"
               "raw_wheel_l,raw_wheel_r,wheel_l,wheel_r,"
               "contact_wheel_l,contact_wheel_r,other_contact,"
               "normal_force_l,normal_force_r\n";
    }

    void write_trace(
        const bc_controller_snapshot_t &snapshot,
        const ContactState &contact,
        const double offset_deg,
        const double time
    ) {
        trace_ << std::setprecision(10)
               << offset_deg << ',' << time << ','
               << plant_.data().qpos[base_qpos_] << ','
               << plant_.data().qpos[base_qpos_ + 2] << ','
               << base_forward_velocity(snapshot);
        for (const int index : {
                 BC_STATE_S, BC_STATE_DS,
                 BC_STATE_THETA_L, BC_STATE_DTHETA_L,
                 BC_STATE_THETA_R, BC_STATE_DTHETA_R,
                 BC_STATE_THETA_B, BC_STATE_DTHETA_B,
             }) {
            trace_ << ',' << snapshot.state.value[index];
        }
        for (const int index : {
                 BC_STATE_S, BC_STATE_DS,
             }) {
            trace_ << ',' << snapshot.state_reference.value[index];
        }
        const double offset = offset_deg * BC_PI / 180.0;
        const double theta_left_reference =
            snapshot.state_reference.value[BC_STATE_THETA_L] + offset;
        const double theta_right_reference =
            snapshot.state_reference.value[BC_STATE_THETA_R] + offset;
        trace_ << ',' << theta_left_reference
               << ',' << theta_right_reference;
        trace_ << ',' << snapshot.actuation_request.wheel_torque[BC_L]
               << ',' << snapshot.actuation_request.wheel_torque[BC_R]
               << ',' << snapshot.actuation.wheel_torque[BC_L]
               << ',' << snapshot.actuation.wheel_torque[BC_R]
               << ',' << contact.wheel[BC_L]
               << ',' << contact.wheel[BC_R]
               << ',' << contact.other
               << ',' << contact.wheel_normal_force[BC_L]
               << ',' << contact.wheel_normal_force[BC_R] << '\n';
    }

    balance::sim::MujocoPlant plant_;
    balance::sim::MujocoAdapter adapter_;
    balance::sim::PerformanceContactMonitor contact_monitor_;
    ScanConfig config_;
    int base_qpos_{};
    int base_dof_{};
    std::ofstream summary_;
    std::ofstream trace_;
};

bool parse_double(const std::string &value, double &output) {
    std::size_t consumed = 0U;
    try {
        output = std::stod(value, &consumed);
    } catch (const std::exception &) {
        return false;
    }
    return consumed == value.size() && std::isfinite(output);
}

bool parse_size(const std::string &value, std::size_t &output) {
    std::size_t consumed = 0U;
    try {
        output = std::stoul(value, &consumed);
    } catch (const std::exception &) {
        return false;
    }
    return consumed == value.size() && output > 0U;
}

bool parse_arguments(
    const int argc, char **argv, ScanConfig &config
) {
    if (argc < 3 || (argc - 3) % 2 != 0) return false;

    for (int index = 3; index < argc; index += 2) {
        const std::string option = argv[index];
        const std::string value = argv[index + 1];
        if (option == "--leg-length") {
            if (!parse_double(value, config.leg_length)) return false;
        } else if (option == "--offset-min-deg") {
            if (!parse_double(value, config.offset_minimum_deg)) return false;
        } else if (option == "--offset-max-deg") {
            if (!parse_double(value, config.offset_maximum_deg)) return false;
        } else if (option == "--offset-step-deg") {
            if (!parse_double(value, config.offset_step_deg)) return false;
        } else if (option == "--trace-stride") {
            if (!parse_size(value, config.trace_stride)) return false;
        } else {
            return false;
        }
    }

    return config.leg_length > 0.0 && config.offset_step_deg > 0.0 &&
        config.offset_minimum_deg <= config.offset_maximum_deg;
}

void print_result(const CaseResult &result) {
    std::cout << std::fixed << std::setprecision(3)
              << "offset=" << std::setw(7) << result.offset_deg
              << " deg pitch=" << std::setw(7)
              << result.pitch.mean() * 180.0 / BC_PI
              << " deg ds=" << std::setw(8) << result.velocity.mean()
              << " m/s slope=" << std::setw(8)
              << result.velocity_trend.slope()
              << " contact=" << result.contact_ratio()
              << " candidate=" << result.candidate;
    if (result.issue != "none") std::cout << " issue=" << result.issue;
    std::cout << '\n';
}

} // namespace

int main(int argc, char **argv) {
    ScanConfig config;
    if (!parse_arguments(argc, argv, config)) {
        std::cerr
            << "usage: rm_balance_trim_scan <model.xml> <output-directory> "
               "[--leg-length <metres>] "
               "[--offset-min-deg <degrees>] "
               "[--offset-max-deg <degrees>] "
               "[--offset-step-deg <degrees>] "
               "[--trace-stride <steps>]\n";
        return EXIT_FAILURE;
    }

    try {
        TrimScanner scanner(argv[1], argv[2], config);
        const int case_count = static_cast<int>(std::floor(
            (config.offset_maximum_deg - config.offset_minimum_deg) /
                config.offset_step_deg + 1.0e-9)) + 1;
        for (int index = 0; index < case_count; ++index) {
            const double offset = config.offset_minimum_deg +
                static_cast<double>(index) * config.offset_step_deg;
            CaseResult result = scanner.run(offset);
            scanner.write_summary(result);
            print_result(result);
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_trim_scan: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
