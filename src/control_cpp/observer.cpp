#include "balance_cpp/observer.hpp"

#include <algorithm>
#include <cmath>

#include "balance_cpp/math.hpp"

namespace balance::control {
namespace {

constexpr std::size_t left = 0;
constexpr std::size_t right = 1;
constexpr std::size_t front = 0;
constexpr std::size_t rear = 1;
constexpr std::size_t leg_length_coordinate = 0;
constexpr std::size_t leg_angle_coordinate = 1;

} // namespace

LegKinematicsSolver::LegKinematicsSolver(
    const LegKinematicsConfig config
)
    : hip_link_length_(config.hip_link_length),
      wheel_link_length_(config.wheel_link_length) {}

LegKinematics LegKinematicsSolver::calculate(
    const LegFeedback &feedback
) const {
    const float phi_front = feedback.joint[front].angle;
    const float phi_rear = feedback.joint[rear].angle;
    const float rate_front = feedback.joint[front].angular_velocity;
    const float rate_rear = feedback.joint[rear].angular_velocity;
    const float delta = 0.5F * (phi_front - phi_rear);
    const float sin_delta = std::sin(delta);
    const float cos_delta = std::cos(delta);
    const float l1_sq = hip_link_length_ * hip_link_length_;
    const float radicand = std::max(
        0.0F, wheel_link_length_ * wheel_link_length_ -
            l1_sq * sin_delta * sin_delta);
    const float root = std::sqrt(radicand);
    const float denominator = std::max(root, 1.0e-6F);
    const float derivative = 0.5F * (
        -hip_link_length_ * sin_delta -
        l1_sq * sin_delta * cos_delta / denominator);

    LegKinematics result{};
    result.length = hip_link_length_ * cos_delta + root;
    result.angle_body = 0.5F * (phi_front + phi_rear);
    result.length_velocity = (rate_front - rate_rear) * derivative;
    result.angular_velocity = 0.5F * (rate_front + rate_rear);
    result.jacobian[leg_length_coordinate] = {derivative, -derivative};
    result.jacobian[leg_angle_coordinate] = {0.5F, 0.5F};
    return result;
}

Observer::Observer(ObserverConfig config)
    : config_(config), leg_kinematics_(config.leg_kinematics),
      velocity_estimator_(config.velocity_estimator) {
    reset();
}

void Observer::reset() {
    velocity_estimator_.reset();
    wheel_startup_hold_.reset();
    estimate_ = {};
    previous_yaw_ = 0.0F;
    yaw_ = 0.0F;
    initialized_ = false;
}

Estimate Observer::update(
    const SensorFrame &sensor,
    const ObservationContext &context,
    const float timestep
) {
    if (!initialized_) {
        previous_yaw_ = sensor.imu.yaw;
        initialized_ = true;
    }
    yaw_ += math::wrap_angle(sensor.imu.yaw - previous_yaw_);
    previous_yaw_ = sensor.imu.yaw;
    estimate_.roll = math::wrap_angle(sensor.imu.roll);
    estimate_.roll_rate = sensor.imu.roll_rate;
    for (std::size_t side = 0; side < side_count; ++side) {
        estimate_.leg[side] = leg_kinematics_.calculate(sensor.leg[side]);
    }

    const float wheel_odometry = config_.wheel_radius * 0.5F * (
        sensor.wheel[left].angular_velocity +
        sensor.wheel[right].angular_velocity);
    float axle_z = config_.hip_center_position.z;
    float relative_velocity_x = 0.0F;
    for (const auto &leg : estimate_.leg) {
        axle_z += 0.5F * leg.length * std::sin(leg.angle_body);
        relative_velocity_x += 0.5F * (
            -leg.length_velocity * std::cos(leg.angle_body) +
            leg.length * leg.angular_velocity * std::sin(leg.angle_body));
    }
    const float velocity_offset =
        sensor.imu.pitch_rate * (axle_z - config_.imu_position.z) -
        sensor.imu.yaw_rate *
            (config_.hip_center_position.y - config_.imu_position.y) +
        relative_velocity_x;

    const bool wheel_started = wheel_startup_hold_.update(
        context.wheel_velocity_observation_enabled,
        config_.wheel_velocity_startup_delay,
        timestep);
    if (wheel_started) {
        velocity_estimator_.update(wheel_odometry - velocity_offset, timestep);
    } else {
        velocity_estimator_.skip_update();
    }
    velocity_estimator_.predict(sensor.imu, timestep);
    estimate_.velocity = velocity_estimator_.estimate();
    estimate_.velocity.wheel_odometry = wheel_odometry;
    estimate_.velocity.estimated_axle =
        estimate_.velocity.velocity_x + velocity_offset;

    auto &state = estimate_.state;
    state[StateIndex::velocity] = estimate_.velocity.estimated_axle;
    if (timestep > 0.0F) {
        state[StateIndex::position] +=
            state[StateIndex::velocity] * timestep;
    }
    state[StateIndex::heading] = yaw_;
    state[StateIndex::heading_rate] = sensor.imu.yaw_rate;
    state[StateIndex::pitch] = math::wrap_angle(sensor.imu.pitch);
    state[StateIndex::pitch_rate] = sensor.imu.pitch_rate;
    state[StateIndex::left_leg_angle] = math::wrap_angle(
        estimate_.leg[left].angle_body + 0.5F * math::pi + sensor.imu.pitch);
    state[StateIndex::left_leg_rate] =
        estimate_.leg[left].angular_velocity + sensor.imu.pitch_rate;
    state[StateIndex::right_leg_angle] = math::wrap_angle(
        estimate_.leg[right].angle_body + 0.5F * math::pi + sensor.imu.pitch);
    state[StateIndex::right_leg_rate] =
        estimate_.leg[right].angular_velocity + sensor.imu.pitch_rate;
    return estimate_;
}

} // namespace balance::control
