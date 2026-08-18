#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "balance_cpp/control_core.hpp"
#include "balance_cpp/control_law/lqr.hpp"
#include "balance_cpp/control_law/pd.hpp"
#include "balance_cpp/leg_kinematics.hpp"
#include "balance_cpp/math.hpp"
#include "balance_cpp/observer.hpp"
#include "balance_cpp/output_gate.hpp"
#include "balance_cpp/snapshot_assembler.hpp"
#include "balance_cpp/state_machine/system.hpp"
#include "balance_cpp/velocity_estimator.hpp"

namespace {

using namespace balance::control;

bool near(const float left, const float right, const float tolerance) {
    return std::abs(left - right) <= tolerance;
}

bool test_math() {
    return near(math::radians(180.0F), math::pi, 1.0e-6F) &&
        near(math::wrap_angle(2.5F * math::pi), 0.5F * math::pi, 1.0e-6F);
}

bool all_zero(const Actuation &actuation) {
    for (std::size_t side = 0; side < side_count; ++side) {
        if (actuation.wheel_torque[side] != 0.0F) return false;
        for (const float torque : actuation.leg[side].joint_torque) {
            if (torque != 0.0F) return false;
        }
    }
    return true;
}

LegFeedback standing_leg() {
    float low = 0.0F;
    float high = 0.5F * math::pi;
    for (int iteration = 0; iteration < 50; ++iteration) {
        const float delta = 0.5F * (low + high);
        const float length = 0.215F * std::cos(delta) + std::sqrt(
            0.254F * 0.254F - 0.215F * 0.215F *
                std::sin(delta) * std::sin(delta));
        if (length > 0.18F) low = delta;
        else high = delta;
    }
    const float delta = 0.5F * (low + high);
    LegFeedback feedback{};
    feedback.joint[0].angle = -0.5F * math::pi + delta;
    feedback.joint[1].angle = -0.5F * math::pi - delta;
    return feedback;
}

bool test_leg_kinematics() {
    const LegKinematics result =
        LegKinematicsSolver{}.calculate(standing_leg());
    return near(result.length, 0.18F, 2.0e-6F) &&
        near(result.angle_body, -0.5F * math::pi, 2.0e-6F) &&
        near(result.jacobian[0][0], -result.jacobian[0][1], 1.0e-7F);
}

bool test_velocity_estimator() {
    VelocityEstimator estimator{};
    estimator.update(0.01F, 0.001F);
    const auto &estimate = estimator.estimate();
    if (!estimate.measurement_accepted ||
        !estimate.wheel_velocity_reliable ||
        !std::isfinite(estimate.velocity_x)) {
        return false;
    }
    estimator.skip_update();
    return !estimator.estimate().wheel_velocity_reliable &&
        !estimator.estimate().measurement_accepted;
}

bool test_observer_context() {
    ObserverConfig config{};
    config.wheel_velocity_startup_delay = 0.003F;
    Observer observer{config};
    SensorFrame sensor{};
    sensor.leg = {standing_leg(), standing_leg()};
    sensor.wheel[0].angular_velocity = 0.1F;
    sensor.wheel[1].angular_velocity = 0.1F;

    for (int tick = 0; tick < 5; ++tick) {
        const Estimate estimate = observer.update(sensor, {false}, 0.001F);
        if (estimate.velocity.wheel_velocity_reliable) return false;
    }
    for (int tick = 0; tick < 4; ++tick) {
        const Estimate estimate = observer.update(sensor, {true}, 0.001F);
        (void)estimate;
    }
    return observer.estimate().velocity.wheel_velocity_reliable &&
        observer.estimate().velocity.measurement_accepted;
}

bool test_system_state_machine() {
    MotionConfig config{};
    config.stable_duration = 0.003F;
    config.engage_duration = 0.002F;
    SystemStateMachine state_machine{config};
    Estimate estimate{};
    const auto leg = LegKinematicsSolver{}.calculate(standing_leg());
    estimate.leg = {leg, leg};
    estimate.state[StateIndex::position] = 1.25F;
    estimate.state[StateIndex::heading] = -0.3F;
    OperatorCommand command{};

    ControlCommand control_command =
        state_machine.update(command, estimate, 0.001F);
    StateMachineStatus status = state_machine.status();
    if (status.system != SystemState::off ||
        status.motion.state != MotionState::idle ||
        control_command.wheel_strategy != WheelStrategy::disabled) {
        return false;
    }
    command.system_enabled = true;
    control_command = state_machine.update(command, estimate, 0.001F);
    status = state_machine.status();
    if (status.system != SystemState::on ||
        status.motion.state != MotionState::idle) {
        return false;
    }
    command.balance_restart = true;
    control_command = state_machine.update(command, estimate, 0.001F);
    status = state_machine.status();
    if (status.motion.state != MotionState::leg_positioning ||
        control_command.leg[0].length_strategy !=
            LegLengthStrategy::position ||
        control_command.leg[0].angle_strategy !=
            LegAngleStrategy::position) {
        return false;
    }
    command.balance_restart = false;
    for (int tick = 0; tick < 4; ++tick) {
        control_command = state_machine.update(command, estimate, 0.001F);
        status = state_machine.status();
        if (status.motion.state == MotionState::balance_engaging) break;
    }
    if (status.motion.state != MotionState::balance_engaging ||
        control_command.wheel_strategy != WheelStrategy::lqr ||
        !control_command.suppress_position_feedback ||
        !control_command.suppress_heading_feedback ||
        !near(control_command.state_reference[StateIndex::position],
              1.25F, 1.0e-7F) ||
        !state_machine.observation_context()
             .wheel_velocity_observation_enabled) {
        return false;
    }
    for (int tick = 0; tick < 3; ++tick) {
        control_command = state_machine.update(command, estimate, 0.001F);
        status = state_machine.status();
        if (status.motion.state == MotionState::active) break;
    }
    if (status.motion.state != MotionState::active ||
        control_command.wheel_strategy != WheelStrategy::lqr ||
        control_command.suppress_position_feedback ||
        control_command.suppress_heading_feedback) {
        return false;
    }

    command.balance_restart = true;
    control_command = state_machine.update(command, estimate, 0.001F);
    status = state_machine.status();
    return status.system == SystemState::on &&
        status.motion.state == MotionState::leg_positioning &&
        control_command.leg[0].length_strategy ==
            LegLengthStrategy::position;
}

bool test_controllers() {
    const PdController pd{{2.0F, 3.0F, 4.0F}};
    if (!near(pd.calculate(1.0F, 1.0F), 4.0F, 1.0e-7F)) return false;

    StateVector error{};
    error[StateIndex::pitch] = 0.1F;
    const LqrOutput lqr = LqrController{}.calculate(0.18F, error);
    return std::isfinite(lqr.wheel_torque[0]) &&
        std::isfinite(lqr.leg_torque[0]);
}

bool test_control_core_and_gate() {
    const ControlCoreConfig control_config{};
    const MotionConfig motion_config{};
    const OutputGateConfig gate_config{};
    const auto leg = LegKinematicsSolver{}.calculate(standing_leg());
    Estimate estimate{};
    estimate.leg = {leg, leg};
    ControlCommand command{};
    ControlCore control_core{control_config};
    if (control_core.calculate(estimate, command)
            .actuation.wheel_torque[0] != 0.0F) {
        return false;
    }
    for (auto &leg_command : command.leg) {
        leg_command.length_strategy = LegLengthStrategy::position_support;
        leg_command.angle_strategy = LegAngleStrategy::lqr;
        leg_command.target_length = motion_config.startup_leg_length;
    }
    command.wheel_strategy = WheelStrategy::lqr;
    const ControlOutput request = control_core.calculate(estimate, command);
    if (request.actuation.leg[0].joint_torque[0] == 0.0F) return false;

    Actuation excessive = request.actuation;
    excessive.wheel_torque[0] = 100.0F;
    excessive.leg[0].joint_torque[0] = -100.0F;
    const OutputGate gate{gate_config};
    const Actuation applied = gate.apply(excessive, true);
    if (!near(
            applied.wheel_torque[0], gate_config.wheel_torque_limit,
            1.0e-7F) ||
        !near(applied.leg[0].joint_torque[0],
              -gate_config.joint_torque_limit, 1.0e-7F) ||
        gate.apply(excessive, false).wheel_torque[0] != 0.0F) {
        return false;
    }

    excessive.wheel_torque[1] = std::numeric_limits<float>::quiet_NaN();
    if (!all_zero(gate.apply(excessive, true))) return false;

    excessive.wheel_torque[1] = 0.0F;
    excessive.leg[1].joint_torque[1] =
        std::numeric_limits<float>::infinity();
    return all_zero(gate.apply(excessive, true));
}

bool test_snapshot_assembler() {
    Estimate estimate{};
    estimate.state[StateIndex::pitch] = 0.1F;
    StateMachineStatus status{};
    status.system = SystemState::on;
    status.motion.state = MotionState::active;
    ControlCommand command{};
    ControlOutput control_output{};
    control_output.roll_force_request = 3.0F;
    Actuation applied{};
    const Snapshot snapshot = SnapshotAssembler{}.assemble(
        estimate, status, command, control_output, applied, 42U);
    return snapshot.system_state == SystemState::on &&
        snapshot.motion_state == MotionState::active &&
        snapshot.tick_count == 42U && snapshot.roll_force_request == 3.0F &&
        snapshot.state[StateIndex::pitch] == 0.1F;
}

} // namespace

int main() {
    if (!test_math()) std::cerr << "math failed\n";
    else if (!test_leg_kinematics()) std::cerr << "leg kinematics failed\n";
    else if (!test_velocity_estimator()) std::cerr << "estimator failed\n";
    else if (!test_observer_context()) std::cerr << "observer failed\n";
    else if (!test_system_state_machine()) std::cerr << "system failed\n";
    else if (!test_controllers()) std::cerr << "controllers failed\n";
    else if (!test_control_core_and_gate()) std::cerr << "core/gate failed\n";
    else if (!test_snapshot_assembler()) std::cerr << "snapshot failed\n";
    else return EXIT_SUCCESS;
    return EXIT_FAILURE;
}
