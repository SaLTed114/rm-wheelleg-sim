#include "forward_velocity_diagnostic.hpp"

#include <cmath>
#include <stdexcept>

namespace balance::benchmark {
namespace {

constexpr double kContactRecoverySeconds = 0.050;

} // namespace

const char *forward_observation_name(
    const ForwardVelocityObservation observation
) noexcept {
    switch (observation) {
    case ForwardVelocityObservation::wheel_odometry:
        return "wheel_odometry";
    case ForwardVelocityObservation::base_truth:
        return "base_truth";
    case ForwardVelocityObservation::contact_gated:
        return "contact_gated";
    }
    return "unknown";
}

ForwardVelocityDiagnostic::ForwardVelocityDiagnostic(
    const ForwardVelocityObservation observation,
    const double wheel_radius
) : observation_(observation), wheel_radius_(wheel_radius) {
    if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0) {
        throw std::invalid_argument(
            "diagnostic wheel radius must be finite and positive");
    }
}

void ForwardVelocityDiagnostic::reset() noexcept {
    contact_gate_until_ = 0.0;
}

void ForwardVelocityDiagnostic::apply(
    const mjData &data,
    const SimulationSampler &sampler,
    const double previous_velocity,
    bc_sensor_feedback_t &feedback
) {
    if (observation_ == ForwardVelocityObservation::wheel_odometry) return;

    if (observation_ == ForwardVelocityObservation::base_truth) {
        replace_velocity(feedback, sampler.read_base(data).forward_velocity);
        return;
    }

    const GroundContactState contact = sampler.read_contacts(data);
    if (!contact.wheel[BC_L] || !contact.wheel[BC_R]) {
        contact_gate_until_ = data.time + kContactRecoverySeconds;
    }
    if (data.time < contact_gate_until_) {
        replace_velocity(feedback, previous_velocity);
    }
}

void ForwardVelocityDiagnostic::replace_velocity(
    bc_sensor_feedback_t &feedback,
    const double target_velocity
) const {
    const double wheel_velocity = 0.5 * (
        feedback.wheel[BC_L].angular_velocity +
        feedback.wheel[BC_R].angular_velocity);
    const double correction =
        target_velocity / wheel_radius_ - wheel_velocity;

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        feedback.wheel[side].angular_velocity +=
            static_cast<float>(correction);
    }
}

} // namespace balance::benchmark
