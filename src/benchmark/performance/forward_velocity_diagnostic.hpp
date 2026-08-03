#ifndef BALANCE_BENCHMARK_FORWARD_VELOCITY_DIAGNOSTIC_HPP
#define BALANCE_BENCHMARK_FORWARD_VELOCITY_DIAGNOSTIC_HPP

#include <mujoco/mujoco.h>

#include "balance/types.h"
#include "common/simulation_sample.hpp"

namespace balance::benchmark {

enum class ForwardVelocityObservation {
    wheel_odometry,
    base_truth,
    contact_gated,
};

[[nodiscard]] const char *forward_observation_name(
    ForwardVelocityObservation observation) noexcept;

class ForwardVelocityDiagnostic {
public:
    ForwardVelocityDiagnostic(
        ForwardVelocityObservation observation,
        double wheel_radius);

    void reset() noexcept;
    void apply(
        const mjData &data,
        const SimulationSampler &sampler,
        double previous_velocity,
        bc_sensor_feedback_t &feedback);

    [[nodiscard]] ForwardVelocityObservation observation() const noexcept {
        return observation_;
    }
    [[nodiscard]] double wheel_radius() const noexcept {
        return wheel_radius_;
    }

private:
    void replace_velocity(
        bc_sensor_feedback_t &feedback,
        double target_velocity) const;

    ForwardVelocityObservation observation_{};
    double wheel_radius_{};
    double contact_gate_until_{};
};

} // namespace balance::benchmark

#endif
