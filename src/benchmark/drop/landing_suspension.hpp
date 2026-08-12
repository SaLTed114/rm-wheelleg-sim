#ifndef BALANCE_BENCHMARK_LANDING_SUSPENSION_HPP
#define BALANCE_BENCHMARK_LANDING_SUSPENSION_HPP

#include <array>

#include "balance/controller_snapshot.h"

namespace balance::benchmark {

struct LandingSuspensionSpec {
    double stiffness{};
    double damping{};
    double support_force{76.204};
    double minimum_force{};
    double maximum_force{240.0};
    double force_rate_limit{3000.0};
    double retracted_length{0.18};
    double retraction_speed{0.8};
    double recovery_length_kp{1600.0};
    double recovery_length_kd{75.0};
    double recovery_reference_speed{0.1};
    double roll_kp{800.0};
    double roll_kd{60.0};
    double roll_force_limit{200.0};
};

struct LandingSuspensionOutput {
    std::array<bool, BC_SIDE_NUM> contact_latched{};
    std::array<double, BC_SIDE_NUM> captured_length{};
    std::array<double, BC_SIDE_NUM> equilibrium_length{};
    std::array<double, BC_SIDE_NUM> requested_force{};
    std::array<double, BC_SIDE_NUM> applied_force{};
    std::array<bool, BC_SIDE_NUM> force_rate_limited{};
};

class LandingSuspensionController {
public:
    explicit LandingSuspensionController(const LandingSuspensionSpec &spec);

    void reset() noexcept;
    void update(
        const bc_controller_snapshot_t &snapshot,
        const std::array<bool, BC_SIDE_NUM> &wheel_contact,
        double timestep_seconds) noexcept;

    [[nodiscard]] const LandingSuspensionSpec &spec() const noexcept {
        return spec_;
    }
    [[nodiscard]] const LandingSuspensionOutput &output() const noexcept {
        return output_;
    }

private:
    LandingSuspensionSpec spec_;
    LandingSuspensionOutput output_{};
};

} // namespace balance::benchmark

#endif
