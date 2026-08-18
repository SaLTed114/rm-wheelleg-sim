#ifndef BALANCE_CPP_TYPES_HPP
#define BALANCE_CPP_TYPES_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace balance::control {

inline constexpr std::size_t side_count = 2;
inline constexpr std::size_t joint_count = 2;
inline constexpr std::size_t state_count = 10;

enum class Side : std::size_t { left, right };
enum class Joint : std::size_t { front, rear };
enum class StateIndex : std::size_t {
    position,
    velocity,
    heading,
    heading_rate,
    left_leg_angle,
    left_leg_rate,
    right_leg_angle,
    right_leg_rate,
    pitch,
    pitch_rate,
};

template <typename T>
using Sides = std::array<T, side_count>;

template <typename T>
using Joints = std::array<T, joint_count>;

struct JointFeedback {
    float angle{};
    float angular_velocity{};
    float torque{};
};

struct LegFeedback {
    Joints<JointFeedback> joint{};
};

struct WheelFeedback {
    float angle{};
    float angular_velocity{};
};

struct ImuFeedback {
    float roll{};
    float pitch{};
    float yaw{};
    float roll_rate{};
    float pitch_rate{};
    float yaw_rate{};
    float specific_force_x{};
    float specific_force_y{};
    float specific_force_z{};
};

struct SensorFrame {
    Sides<LegFeedback> leg{};
    Sides<WheelFeedback> wheel{};
    ImuFeedback imu{};
};

struct OperatorCommand {
    bool system_enabled{};
    bool balance_restart{};
};

struct LegActuation {
    Joints<float> joint_torque{};
};

struct Actuation {
    Sides<LegActuation> leg{};
    Sides<float> wheel_torque{};
};

struct StateVector {
    std::array<float, state_count> value{};

    float &operator[](StateIndex index) {
        return value[static_cast<std::size_t>(index)];
    }
    const float &operator[](StateIndex index) const {
        return value[static_cast<std::size_t>(index)];
    }
};

struct LegKinematics {
    float length{};
    float angle_body{};
    float length_velocity{};
    float angular_velocity{};
    std::array<Joints<float>, 2> jacobian{};
};

enum class SystemState {
    off,
    on,
};

enum class MotionState {
    idle,
    self_righting,
    leg_positioning,
    balance_engaging,
    active,
};

struct VelocityEstimate {
    float velocity_x{};
    float velocity_y{};
    float acceleration_bias_x{};
    float acceleration_bias_y{};
    float wheel_odometry{};
    float estimated_axle{};
    float velocity_variance_x{};
    bool measurement_accepted{};
    bool wheel_velocity_reliable{};
};

struct ObservationContext {
    bool wheel_velocity_observation_enabled{};
};

struct Estimate {
    StateVector state{};
    Sides<LegKinematics> leg{};
    VelocityEstimate velocity{};
    float roll{};
    float roll_rate{};
};

enum class LegLengthStrategy {
    disabled,
    position,
    position_support,
};

enum class LegAngleStrategy {
    disabled,
    position,
    lqr,
};

enum class WheelStrategy {
    disabled,
    lqr,
};

struct LegControlCommand {
    LegLengthStrategy length_strategy{LegLengthStrategy::disabled};
    LegAngleStrategy angle_strategy{LegAngleStrategy::disabled};
    float target_length{};
    float target_angle{};
};

struct ControlCommand {
    Sides<LegControlCommand> leg{};
    WheelStrategy wheel_strategy{WheelStrategy::disabled};
    StateVector state_reference{};
    bool suppress_position_feedback{};
    bool suppress_heading_feedback{};
};

struct MotionStatus {
    MotionState state{MotionState::idle};
    float leg_stable_elapsed{};
    float engage_elapsed{};
};

struct StateMachineStatus {
    SystemState system{SystemState::off};
    MotionStatus motion{};
};

struct ControlOutput {
    Actuation actuation{};
    float roll_force_request{};
};

struct Snapshot {
    SystemState system_state{SystemState::off};
    MotionState motion_state{MotionState::idle};
    StateVector state{};
    StateVector state_reference{};
    Sides<LegKinematics> leg{};
    VelocityEstimate velocity_estimate{};
    float roll{};
    float roll_rate{};
    float roll_force_request{};
    float leg_stable_elapsed{};
    float engage_elapsed{};
    Actuation actuation_request{};
    Actuation actuation{};
    std::uint32_t tick_count{};
};

struct ControllerOutput {
    Actuation requested{};
    Actuation applied{};
    Snapshot snapshot{};
};

} // namespace balance::control

#endif
