#ifndef BALANCE_CPP_VELOCITY_ESTIMATOR_HPP
#define BALANCE_CPP_VELOCITY_ESTIMATOR_HPP

#include <array>

#include "balance_cpp/types.hpp"

namespace balance::control {

struct VelocityEstimatorConfig {
    float gravity{9.81F};
    float initial_velocity_variance{0.0004F};
    float initial_bias_variance{0.000001F};
    float acceleration_variance{0.02F};
    float bias_walk_variance{0.00000001F};
    float wheel_velocity_variance{0.0004F};
    float recovery_velocity_variance{0.0008F};
    float nis_gate{9.0F};
    float wheel_rejection_duration{0.02F};
    float wheel_recovery_duration{0.02F};
    float reacquisition_stable_duration{0.10F};
    float reacquisition_max_wheel_speed{0.5F};
    float reacquisition_max_wheel_acceleration{25.0F};
    float reacquisition_velocity_rate{2.0F};
};

class VelocityEstimator {
public:
    explicit VelocityEstimator(
        VelocityEstimatorConfig config = {}
    );

    void reset();
    void skip_update();
    void update(float measurement, float timestep);
    void predict(const ImuFeedback &imu, float timestep);

    const VelocityEstimate &estimate() const {
        return output_;
    }

private:
    VelocityEstimatorConfig config_{};
    std::array<float, 4> state_{};
    std::array<std::array<float, 4>, 4> covariance_{};
    VelocityEstimate output_{};
    float rejection_elapsed_{};
    float recovery_elapsed_{};
    float reacquisition_elapsed_{};
    float previous_wheel_measurement_{};
    bool measurement_initialized_{};
    bool wheel_velocity_reliable_{};
    bool previous_wheel_measurement_initialized_{};
};

} // namespace balance::control

#endif
