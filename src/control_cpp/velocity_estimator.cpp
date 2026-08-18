#include "balance_cpp/velocity_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace balance::control {

VelocityEstimator::VelocityEstimator(
    VelocityEstimatorConfig config
) : config_(config) {
    reset();
}

void VelocityEstimator::reset() {
    state_ = {};
    covariance_ = {};
    covariance_[0][0] = config_.initial_velocity_variance;
    covariance_[1][1] = config_.initial_velocity_variance;
    covariance_[2][2] = config_.initial_bias_variance;
    covariance_[3][3] = config_.initial_bias_variance;
    output_ = {};
    rejection_elapsed_ = 0.0F;
    recovery_elapsed_ = 0.0F;
    reacquisition_elapsed_ = 0.0F;
    previous_wheel_measurement_ = 0.0F;
    measurement_initialized_ = false;
    wheel_velocity_reliable_ = false;
    previous_wheel_measurement_initialized_ = false;
}

void VelocityEstimator::skip_update() {
    output_.measurement_accepted = false;
    rejection_elapsed_ = 0.0F;
    recovery_elapsed_ = 0.0F;
    reacquisition_elapsed_ = 0.0F;
    previous_wheel_measurement_initialized_ = false;
    measurement_initialized_ = false;
    wheel_velocity_reliable_ = false;
    output_.velocity_x = state_[0];
    output_.velocity_y = state_[1];
    output_.acceleration_bias_x = state_[2];
    output_.acceleration_bias_y = state_[3];
    output_.velocity_variance_x = covariance_[0][0];
    output_.wheel_velocity_reliable = false;
}

void VelocityEstimator::update(
    const float measurement,
    const float timestep
) {
    if (!measurement_initialized_) {
        covariance_ = {};
        covariance_[0][0] = config_.initial_velocity_variance;
        covariance_[1][1] = config_.initial_velocity_variance;
        covariance_[2][2] = config_.initial_bias_variance;
        covariance_[3][3] = config_.initial_bias_variance;
        rejection_elapsed_ = 0.0F;
        recovery_elapsed_ = 0.0F;
        measurement_initialized_ = true;
        wheel_velocity_reliable_ = true;
    }

    output_.measurement_accepted = false;
    const float innovation = measurement - state_[0];
    const float innovation_variance =
        covariance_[0][0] + config_.wheel_velocity_variance;
    const float nis = innovation * innovation / innovation_variance;
    const bool inlier = std::isfinite(measurement) &&
        std::isfinite(innovation_variance) && innovation_variance > 0.0F &&
        std::isfinite(nis) && nis <= config_.nis_gate;

    bool usable = false;
    if (inlier) {
        rejection_elapsed_ = 0.0F;
        if (wheel_velocity_reliable_) {
            recovery_elapsed_ = 0.0F;
            usable = true;
        } else {
            recovery_elapsed_ += timestep;
            if (recovery_elapsed_ >= config_.wheel_recovery_duration) {
                recovery_elapsed_ = 0.0F;
                wheel_velocity_reliable_ = true;
                usable = true;
            }
        }
    } else {
        recovery_elapsed_ = 0.0F;
        if (wheel_velocity_reliable_) {
            rejection_elapsed_ += timestep;
            if (rejection_elapsed_ >= config_.wheel_rejection_duration) {
                rejection_elapsed_ = 0.0F;
                wheel_velocity_reliable_ = false;
                covariance_[0][0] = std::max(
                    covariance_[0][0], config_.recovery_velocity_variance);
            }
        }
    }

    if (usable) {
        reacquisition_elapsed_ = 0.0F;
        previous_wheel_measurement_initialized_ = false;
        std::array<float, 4> gain{};
        for (std::size_t row = 0; row < gain.size(); ++row) {
            gain[row] = covariance_[row][0] / innovation_variance;
        }
        gain[1] = 0.0F;
        gain[3] = 0.0F;
        for (std::size_t row = 0; row < state_.size(); ++row) {
            state_[row] += gain[row] * innovation;
        }

        std::array<std::array<float, 4>, 4> left{};
        for (std::size_t row = 0; row < 4; ++row) {
            left[row][row] = 1.0F;
            left[row][0] -= gain[row];
        }
        std::array<std::array<float, 4>, 4> product{};
        std::array<std::array<float, 4>, 4> updated{};
        for (std::size_t row = 0; row < 4; ++row) {
            for (std::size_t column = 0; column < 4; ++column) {
                for (std::size_t inner = 0; inner < 4; ++inner) {
                    product[row][column] +=
                        left[row][inner] * covariance_[inner][column];
                }
            }
        }
        for (std::size_t row = 0; row < 4; ++row) {
            for (std::size_t column = 0; column < 4; ++column) {
                for (std::size_t inner = 0; inner < 4; ++inner) {
                    updated[row][column] +=
                        product[row][inner] * left[column][inner];
                }
                updated[row][column] +=
                    gain[row] * config_.wheel_velocity_variance * gain[column];
            }
        }
        for (std::size_t row = 0; row < 4; ++row) {
            for (std::size_t column = row + 1; column < 4; ++column) {
                const float average = 0.5F * (
                    updated[row][column] + updated[column][row]);
                updated[row][column] = average;
                updated[column][row] = average;
            }
        }
        covariance_ = updated;
        output_.measurement_accepted = true;
    } else if (!wheel_velocity_reliable_ && std::isfinite(measurement) &&
               std::abs(measurement) <=
                   config_.reacquisition_max_wheel_speed &&
               std::isfinite(timestep) && timestep > 0.0F) {
        bool stable = false;
        if (previous_wheel_measurement_initialized_) {
            const float acceleration = std::abs(
                measurement - previous_wheel_measurement_) / timestep;
            stable = std::isfinite(acceleration) &&
                acceleration <=
                    config_.reacquisition_max_wheel_acceleration;
        }
        previous_wheel_measurement_ = measurement;
        previous_wheel_measurement_initialized_ = true;
        if (!stable) {
            reacquisition_elapsed_ = 0.0F;
        } else {
            reacquisition_elapsed_ += timestep;
            if (reacquisition_elapsed_ >=
                config_.reacquisition_stable_duration) {
                const float maximum_step =
                    config_.reacquisition_velocity_rate * timestep;
                if (std::isfinite(maximum_step) && maximum_step > 0.0F) {
                    state_[0] += std::clamp(
                        measurement - state_[0],
                        -maximum_step, maximum_step);
                }
            }
        }
    } else {
        reacquisition_elapsed_ = 0.0F;
        previous_wheel_measurement_initialized_ = false;
    }

    output_.velocity_x = state_[0];
    output_.velocity_y = state_[1];
    output_.acceleration_bias_x = state_[2];
    output_.acceleration_bias_y = state_[3];
    output_.velocity_variance_x = covariance_[0][0];
    output_.wheel_velocity_reliable = wheel_velocity_reliable_;
}

void VelocityEstimator::predict(
    const ImuFeedback &imu,
    const float timestep
) {
    if (!std::isfinite(timestep) || timestep <= 0.0F) return;
    const float acceleration_x = imu.specific_force_x +
        config_.gravity * std::sin(imu.pitch) - state_[2];
    const float acceleration_y = imu.specific_force_y -
        config_.gravity * std::sin(imu.roll) * std::cos(imu.pitch) - state_[3];
    if (!std::isfinite(acceleration_x) || !std::isfinite(acceleration_y) ||
        !std::isfinite(imu.yaw_rate)) return;

    const float k1_x = acceleration_x + imu.yaw_rate * state_[1];
    const float k1_y = acceleration_y - imu.yaw_rate * state_[0];
    const float middle_x = state_[0] + 0.5F * timestep * k1_x;
    const float middle_y = state_[1] + 0.5F * timestep * k1_y;
    state_[0] += timestep * (acceleration_x + imu.yaw_rate * middle_y);
    state_[1] += timestep * (acceleration_y - imu.yaw_rate * middle_x);

    std::array<std::array<float, 4>, 4> dynamics{};
    dynamics[0][1] = imu.yaw_rate;
    dynamics[0][2] = -1.0F;
    dynamics[1][0] = -imu.yaw_rate;
    dynamics[1][3] = -1.0F;
    std::array<std::array<float, 4>, 4> transition{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            transition[row][column] = row == column ? 1.0F : 0.0F;
            transition[row][column] += timestep * dynamics[row][column];
            for (std::size_t inner = 0; inner < 4; ++inner) {
                transition[row][column] += 0.5F * timestep * timestep *
                    dynamics[row][inner] * dynamics[inner][column];
            }
        }
    }
    std::array<std::array<float, 4>, 4> product{};
    std::array<std::array<float, 4>, 4> updated{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            for (std::size_t inner = 0; inner < 4; ++inner) {
                product[row][column] +=
                    transition[row][inner] * covariance_[inner][column];
            }
        }
    }
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            for (std::size_t inner = 0; inner < 4; ++inner) {
                updated[row][column] +=
                    product[row][inner] * transition[column][inner];
            }
        }
    }
    updated[0][0] += config_.acceleration_variance * timestep * timestep;
    updated[1][1] += config_.acceleration_variance * timestep * timestep;
    updated[2][2] += config_.bias_walk_variance * timestep;
    updated[3][3] += config_.bias_walk_variance * timestep;
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = row + 1; column < 4; ++column) {
            const float average = 0.5F * (
                updated[row][column] + updated[column][row]);
            updated[row][column] = average;
            updated[column][row] = average;
        }
    }
    covariance_ = updated;
    output_.velocity_x = state_[0];
    output_.velocity_y = state_[1];
    output_.acceleration_bias_x = state_[2];
    output_.acceleration_bias_y = state_[3];
    output_.velocity_variance_x = covariance_[0][0];
    output_.wheel_velocity_reliable = wheel_velocity_reliable_;
}

} // namespace balance::control
