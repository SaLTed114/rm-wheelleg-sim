#include "common_diagnostics.hpp"

#include <algorithm>
#include <cmath>

#include "balance/math_utils.h"

namespace balance::benchmark {
namespace {

constexpr double kSaturationTolerance = 1.0e-4;
constexpr double kTerminationAngle = 45.0 * BC_PI / 180.0;

double ratio(
    const std::size_t count, const std::size_t total
) {
    return total == 0U ? 0.0 :
        static_cast<double>(count) / static_cast<double>(total);
}

bool actuation_is_finite(const bc_actuation_t &actuation) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (!std::isfinite(actuation.wheel_torque[side])) return false;
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            if (!std::isfinite(
                    actuation.leg[side].joint_torque[joint])) return false;
        }
    }
    return true;
}

} // namespace

void CommonDiagnostics::observe(const SimulationSample &sample) {
    const bc_controller_snapshot_t &snapshot = sample.controller;
    ++sample_count_;
    if (sample.contact.wheel[BC_L] && sample.contact.wheel[BC_R]) {
        ++both_wheel_contact_count_;
    }
    if (sample.contact.other) ++other_contact_count_;
    finite_ = finite_ && controller_snapshot_is_finite(snapshot);

    bool any_wheel_saturated = false;
    bool any_joint_saturated = false;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const double raw_wheel =
            snapshot.actuation_request.wheel_torque[side];
        const double final_wheel = snapshot.actuation.wheel_torque[side];
        peak_wheel_torque_[side] = std::max(
            peak_wheel_torque_[side], std::abs(raw_wheel));
        const bool wheel_saturated =
            std::abs(raw_wheel - final_wheel) > kSaturationTolerance;
        if (wheel_saturated) ++wheel_saturation_count_[side];
        any_wheel_saturated = any_wheel_saturated || wheel_saturated;

        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            const double raw_joint = snapshot.actuation_request
                .leg[side].joint_torque[joint];
            const double final_joint = snapshot.actuation
                .leg[side].joint_torque[joint];
            peak_joint_torque_[side][joint] = std::max(
                peak_joint_torque_[side][joint], std::abs(raw_joint));
            const bool joint_saturated =
                std::abs(raw_joint - final_joint) > kSaturationTolerance;
            if (joint_saturated) ++joint_saturation_count_[side][joint];
            any_joint_saturated = any_joint_saturated || joint_saturated;
        }
    }
    if (any_wheel_saturated) ++any_wheel_saturation_count_;
    if (any_joint_saturated) ++any_joint_saturation_count_;
}

double CommonDiagnostics::wheel_contact_ratio() const {
    return ratio(both_wheel_contact_count_, sample_count_);
}

double CommonDiagnostics::wheel_saturation_ratio(const int side) const {
    return ratio(wheel_saturation_count_[side], sample_count_);
}

double CommonDiagnostics::joint_saturation_ratio(
    const int side, const int joint
) const {
    return ratio(joint_saturation_count_[side][joint], sample_count_);
}

double CommonDiagnostics::any_wheel_saturation_ratio() const {
    return ratio(any_wheel_saturation_count_, sample_count_);
}

double CommonDiagnostics::any_joint_saturation_ratio() const {
    return ratio(any_joint_saturation_count_, sample_count_);
}

double CommonDiagnostics::peak_wheel_torque(const int side) const {
    return peak_wheel_torque_[side];
}

double CommonDiagnostics::peak_joint_torque(
    const int side, const int joint
) const {
    return peak_joint_torque_[side][joint];
}

double CommonDiagnostics::maximum_wheel_torque() const {
    return *std::max_element(
        peak_wheel_torque_.begin(), peak_wheel_torque_.end());
}

double CommonDiagnostics::maximum_joint_torque() const {
    double maximum = 0.0;
    for (const auto &side : peak_joint_torque_) {
        maximum = std::max(
            maximum, *std::max_element(side.begin(), side.end()));
    }
    return maximum;
}

bool controller_snapshot_is_finite(
    const bc_controller_snapshot_t &snapshot
) {
    bool finite = std::isfinite(snapshot.roll) &&
        std::isfinite(snapshot.roll_rate) &&
        actuation_is_finite(snapshot.actuation_request) &&
        actuation_is_finite(snapshot.actuation);
    for (int index = 0; index < BC_STATE_NUM; ++index) {
        finite = finite && std::isfinite(snapshot.state.value[index]) &&
            std::isfinite(snapshot.state_reference.value[index]);
    }
    return finite;
}

std::string common_diagnostic_issue(const SimulationSample &sample) {
    if (!controller_snapshot_is_finite(sample.controller)) {
        return "non_finite_telemetry";
    }
    if (sample.contact.other) {
        return "non_wheel_contact:" + sample.contact.unexpected;
    }

    const double pitch = std::abs(static_cast<double>(
        sample.controller.state.value[BC_STATE_THETA_B]));
    const double roll = std::abs(static_cast<double>(
        sample.controller.roll));
    if (pitch > kTerminationAngle || roll > kTerminationAngle) {
        return "attitude_termination";
    }
    return {};
}

} // namespace balance::benchmark
