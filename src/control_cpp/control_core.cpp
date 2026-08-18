#include "balance_cpp/control_core.hpp"

#include "balance_cpp/math.hpp"

namespace balance::control {
namespace {

constexpr std::size_t left = 0;
constexpr std::size_t right = 1;
constexpr std::size_t leg_length_coordinate = 0;
constexpr std::size_t leg_angle_coordinate = 1;

} // namespace

ControlCore::ControlCore(ControlCoreConfig config)
    : length_controller_(config.length_controller),
      angle_controller_(config.angle_controller),
      roll_controller_(config.roll_controller),
      leg_angle_trim_(config.leg_angle_trim),
      support_force_(config.support_force) {}

ControlOutput ControlCore::calculate(
    const Estimate &estimate,
    const ControlCommand &command
) const {
    ControlOutput output{};
    bool lqr_required = command.wheel_strategy == WheelStrategy::lqr;
    for (const auto &leg : command.leg) {
        lqr_required = lqr_required ||
            leg.angle_strategy == LegAngleStrategy::lqr;
    }

    LqrOutput lqr{};
    if (lqr_required) {
        StateVector effective_reference = command.state_reference;
        effective_reference[StateIndex::left_leg_angle] += leg_angle_trim_;
        effective_reference[StateIndex::right_leg_angle] += leg_angle_trim_;
        StateVector error{};
        for (std::size_t index = 0; index < state_count; ++index) {
            if ((command.suppress_position_feedback &&
                 index == static_cast<std::size_t>(StateIndex::position)) ||
                (command.suppress_heading_feedback &&
                 index == static_cast<std::size_t>(StateIndex::heading))) {
                continue;
            }
            error.value[index] =
                effective_reference.value[index] - estimate.state.value[index];
        }
        const float average_length = 0.5F * (
            estimate.leg[left].length + estimate.leg[right].length);
        lqr = lqr_controller_.calculate(average_length, error);
    }

    const float roll_force = roll_controller_.calculate(
        -estimate.roll, -estimate.roll_rate);
    for (std::size_t side = 0; side < side_count; ++side) {
        const auto &leg = estimate.leg[side];
        const auto &leg_command = command.leg[side];
        float axial_force = 0.0F;
        switch (leg_command.length_strategy) {
        case LegLengthStrategy::disabled:
            break;
        case LegLengthStrategy::position:
        case LegLengthStrategy::position_support:
            axial_force = length_controller_.calculate(
                leg_command.target_length - leg.length,
                -leg.length_velocity);
            if (leg_command.length_strategy ==
                LegLengthStrategy::position_support) {
                output.roll_force_request = roll_force;
                const float roll_sign = side == left ? 1.0F : -1.0F;
                axial_force += support_force_ + roll_sign * roll_force;
            }
            break;
        }

        float leg_torque = 0.0F;
        switch (leg_command.angle_strategy) {
        case LegAngleStrategy::disabled:
            break;
        case LegAngleStrategy::position:
            leg_torque = angle_controller_.calculate(
                math::wrap_angle(leg_command.target_angle - leg.angle_body),
                -leg.angular_velocity);
            break;
        case LegAngleStrategy::lqr:
            leg_torque = lqr.leg_torque[side];
            break;
        }

        if (command.wheel_strategy == WheelStrategy::lqr) {
            output.actuation.wheel_torque[side] = lqr.wheel_torque[side];
        }
        for (std::size_t joint = 0; joint < joint_count; ++joint) {
            output.actuation.leg[side].joint_torque[joint] =
                axial_force * leg.jacobian[leg_length_coordinate][joint] +
                leg_torque * leg.jacobian[leg_angle_coordinate][joint];
        }
    }
    return output;
}

} // namespace balance::control
