#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
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
constexpr std::size_t kTraceStride = 10U;
constexpr double kStableAngle = 10.0 * BC_PI / 180.0;
constexpr double kMinimumLegLength = 0.13;
constexpr double kMaximumLegLength = 0.20;
constexpr double kMinimumWheelContactRatio = 0.99;
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
    bool completed{};
    bool finite{true};
    bool leg_length_valid{true};
    bool stable{};
    bool tracked{};
    bool settled{};
    std::string failure{"none"};
    std::size_t monitored_steps{};
    std::size_t both_wheel_steps{};
    std::size_t other_contact_steps{};
    double maximum_pitch{};
    double maximum_roll{};
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
        const std::filesystem::path &output_directory
    ) : plant_(model_path, kTimestepSeconds),
        adapter_(plant_.model()), runner_(plant_, adapter_),
        contact_monitor_(plant_.model()) {
        std::filesystem::create_directories(output_directory);
        summary_.open(output_directory / "summary.csv", std::ios::trunc);
        trace_.open(output_directory / "trace.csv", std::ios::trunc);
        if (!summary_ || !trace_) {
            throw std::runtime_error(
                "failed to open performance benchmark output files");
        }

        write_summary_header();
        write_trace_header();
    }

    CaseResult run(const CaseSpec &spec) {
        runner_.reset();
        sample_index_ = 0U;
        CaseResult result{spec};
        balance::sim::PerformanceScenario scenario(spec);
        scenario.reset(plant_.data().time);

        while (!scenario.finished()) {
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

        if (scenario.failed()) {
            result.failure = scenario.failure_reason();
        } else {
            result.completed = true;
        }
        finish_result(result);
        return result;
    }

    void write_summary(const CaseResult &result) {
        summary_ << std::setprecision(10)
                 << result.spec.name << ','
                 << axis_name(result.spec.axis) << ','
                 << result.spec.target << ','
                 << result.completed << ','
                 << result.stable << ','
                 << result.tracked << ','
                 << result.settled << ','
                 << result.failure << ','
                 << result.contact_ratio() << ','
                 << result.other_contact_steps << ','
                 << result.maximum_pitch * 180.0 / BC_PI << ','
                 << result.maximum_roll * 180.0 / BC_PI << ','
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
        if (sample_index_ % kTraceStride == 0U) {
            write_trace(spec, phase, command, contact);
        }
        ++sample_index_;

        if (result == nullptr) return true;
        return collect(
            *result, contact, evaluate_tracking, evaluate_settle);
    }

    bool collect(
        CaseResult &result, const ContactState &contact,
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

        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            const double length = snapshot.leg[side].length;
            result.leg_length_valid = result.leg_length_valid &&
                std::isfinite(length) &&
                length >= kMinimumLegLength &&
                length <= kMaximumLegLength;

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

        const std::string termination =
            balance::sim::performance_termination_reason(snapshot, contact);
        if (!termination.empty()) {
            result.failure = termination;
            return false;
        }
        return true;
    }

    void finish_result(CaseResult &result) const {
        const double tracking_tolerance = result.spec.axis == Axis::forward ?
            std::max(
                kForwardBaseTolerance,
                kRelativeTrackingTolerance * std::abs(result.spec.target)) :
            std::max(
                kYawBaseTolerance,
                kRelativeTrackingTolerance * std::abs(result.spec.target));

        result.stable = result.completed && result.finite &&
            result.leg_length_valid &&
            result.other_contact_steps == 0U &&
            result.contact_ratio() >= kMinimumWheelContactRatio &&
            result.maximum_pitch < kStableAngle &&
            result.maximum_roll < kStableAngle;
        result.tracked = result.completed &&
            result.tracking_error.count != 0U &&
            std::abs(result.tracking_error.mean()) <= tracking_tolerance &&
            result.tracking_error.rms() <= tracking_tolerance;
        result.settled = result.completed &&
            result.settle_forward.count != 0U &&
            std::abs(result.settle_forward.mean()) <=
                kForwardBaseTolerance &&
            result.settle_forward.rms() <= kForwardBaseTolerance &&
            std::abs(result.settle_yaw.mean()) <= kYawBaseTolerance &&
            result.settle_yaw.rms() <= kYawBaseTolerance;
    }

    void write_summary_header() {
        summary_
            << "case,axis,target,completed,stable,tracked,settled,failure,"
               "wheel_contact_ratio,other_contact_steps,max_pitch_deg,"
               "max_roll_deg,tracking_mean_error,tracking_rmse,"
               "settle_mean_ds,settle_rmse_ds,settle_mean_dpsi,"
               "settle_rmse_dpsi,peak_raw_wheel_l,peak_raw_wheel_r,"
               "peak_raw_joint_l_front,peak_raw_joint_l_rear,"
               "peak_raw_joint_r_front,peak_raw_joint_r_rear,"
               "wheel_saturation_l,wheel_saturation_r,"
               "joint_saturation_l_front,joint_saturation_l_rear,"
               "joint_saturation_r_front,joint_saturation_r_rear\n";
    }

    void write_trace_header() {
        trace_ << "case,phase,simulation_time,command_forward,command_yaw,"
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
                  "contact_wheel_l,contact_wheel_r,other_contact\n";
    }

    void write_trace(
        const CaseSpec &spec, const char *phase,
        const bc_operator_command_t &command,
        const ContactState &contact
    ) {
        const bc_controller_snapshot_t &snapshot = runner_.snapshot();
        trace_ << std::setprecision(10)
               << spec.name << ',' << phase << ','
               << plant_.data().time << ','
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
               << ',' << contact.other << '\n';
    }

    balance::sim::MujocoPlant plant_;
    balance::sim::MujocoAdapter adapter_;
    balance::sim::SimulationRunner runner_;
    balance::sim::PerformanceContactMonitor contact_monitor_;
    std::ofstream summary_;
    std::ofstream trace_;
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

    std::cout << std::left << std::setw(16) << result.spec.name
              << " stable=" << result.stable
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
    if (result.failure != "none") {
        std::cout << " failure=" << result.failure;
    }
    std::cout << '\n';
}

void print_boundaries(const std::vector<CaseResult> &results) {
    std::cout << "\nDirectional boundaries:\n";
    for (const Axis axis : {Axis::forward, Axis::yaw}) {
        for (const int sign : {1, -1}) {
            double stable = 0.0;
            double tracked = 0.0;
            double settled = 0.0;
            bool has_stable = false;
            bool has_tracked = false;
            bool has_settled = false;
            for (const CaseResult &result : results) {
                if (result.spec.axis != axis ||
                    (result.spec.target > 0.0 ? 1 : -1) != sign) {
                    continue;
                }
                const double magnitude = std::abs(result.spec.target);
                if (result.stable) {
                    stable = std::max(stable, magnitude);
                    has_stable = true;
                }
                if (result.stable && result.tracked) {
                    tracked = std::max(tracked, magnitude);
                    has_tracked = true;
                }
                if (result.stable && result.settled) {
                    settled = std::max(settled, magnitude);
                    has_settled = true;
                }
            }
            std::cout << "  " << axis_name(axis) << ' '
                      << (sign > 0 ? '+' : '-')
                      << ": stable=";
            if (has_stable) std::cout << stable;
            else std::cout << "none";
            std::cout << ", tracked=";
            if (has_tracked) std::cout << tracked;
            else std::cout << "none";
            std::cout << ", settled=";
            if (has_settled) std::cout << settled;
            else std::cout << "none";
            std::cout << '\n';
        }
    }
}

} // namespace

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr
            << "usage: rm_balance_performance <model.xml> <output-directory>\n";
        return EXIT_FAILURE;
    }

    try {
        PerformanceBenchmark benchmark(argv[1], argv[2]);
        std::vector<CaseResult> results;
        for (const CaseSpec &spec : balance::sim::performance_cases()) {
            CaseResult result = benchmark.run(spec);
            benchmark.write_summary(result);
            print_result(result);
            results.push_back(std::move(result));
        }
        print_boundaries(results);
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_performance: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
