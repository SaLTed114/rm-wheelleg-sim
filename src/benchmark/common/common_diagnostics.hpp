#ifndef BALANCE_BENCHMARK_COMMON_DIAGNOSTICS_HPP
#define BALANCE_BENCHMARK_COMMON_DIAGNOSTICS_HPP

#include <array>
#include <cstddef>
#include <string>

#include "simulation_sample.hpp"

namespace balance::benchmark {

class CommonDiagnostics {
public:
    void observe(const SimulationSample &sample);
    void invalidate() { finite_ = false; }

    [[nodiscard]] bool finite() const { return finite_; }
    [[nodiscard]] std::size_t sample_count() const { return sample_count_; }
    [[nodiscard]] std::size_t other_contact_count() const {
        return other_contact_count_;
    }
    [[nodiscard]] double wheel_contact_ratio() const;
    [[nodiscard]] double wheel_saturation_ratio(int side) const;
    [[nodiscard]] double joint_saturation_ratio(
        int side, int joint) const;
    [[nodiscard]] double any_wheel_saturation_ratio() const;
    [[nodiscard]] double any_joint_saturation_ratio() const;
    [[nodiscard]] double peak_wheel_torque(int side) const;
    [[nodiscard]] double peak_joint_torque(int side, int joint) const;
    [[nodiscard]] double maximum_wheel_torque() const;
    [[nodiscard]] double maximum_joint_torque() const;

private:
    bool finite_{true};
    std::size_t sample_count_{};
    std::size_t both_wheel_contact_count_{};
    std::size_t other_contact_count_{};
    std::array<double, BC_SIDE_NUM> peak_wheel_torque_{};
    std::array<std::array<double, BC_JOINT_NUM>, BC_SIDE_NUM>
        peak_joint_torque_{};
    std::array<std::size_t, BC_SIDE_NUM> wheel_saturation_count_{};
    std::array<std::array<std::size_t, BC_JOINT_NUM>, BC_SIDE_NUM>
        joint_saturation_count_{};
    std::size_t any_wheel_saturation_count_{};
    std::size_t any_joint_saturation_count_{};
};

[[nodiscard]] bool controller_snapshot_is_finite(
    const bc_controller_snapshot_t &snapshot);
[[nodiscard]] std::string common_diagnostic_issue(
    const SimulationSample &sample);

} // namespace balance::benchmark

#endif
