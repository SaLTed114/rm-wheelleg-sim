#include "simulation_runner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "balance/math_utils.h"

namespace balance::sim {

namespace {

bc_controller_config_t default_controller_config() {
    bc_controller_config_t config{};
    bc_controller_default_config(&config);
    return config;
}

} // namespace

SimulationRunner::SimulationRunner(
    MujocoPlant &plant, const MujocoAdapter &adapter
) : SimulationRunner(plant, adapter, default_controller_config()) {}

SimulationRunner::SimulationRunner(
    MujocoPlant &plant,
    const MujocoAdapter &adapter,
    const bc_controller_config_t &config
) : plant_(plant), adapter_(adapter) {
    bc_controller_init(&controller_, &config);
    bc_controller_capture_snapshot(&controller_, &snapshot_);
    bc_sensor_feedback_t feedback{};
    adapter_.read(plant_.data(), feedback);
    yaw_frame_offset_ = feedback.imu.yaw -
        snapshot_.state.value[BC_STATE_PSI];
}

void SimulationRunner::reset() {
    plant_.reset();
    bc_controller_reset(&controller_);
    bc_controller_capture_snapshot(&controller_, &snapshot_);
    feedback_ = {};
    bc_sensor_feedback_t feedback{};
    adapter_.read(plant_.data(), feedback);
    yaw_frame_offset_ = feedback.imu.yaw -
        snapshot_.state.value[BC_STATE_PSI];
}

void SimulationRunner::step(const bc_operator_command_t &command) {
    step(command, bc_gimbal_feedback_t{});
}

void SimulationRunner::step(
    const bc_operator_command_t &command,
    const bc_gimbal_feedback_t &gimbal_feedback
) {
    bc_sensor_feedback_t feedback{};
    adapter_.read(plant_.data(), feedback);
    feedback.gimbal = gimbal_feedback;
    step_with_feedback(command, feedback);
}

void SimulationRunner::step_with_gimbal_heading(
    const bc_operator_command_t &command,
    const float world_yaw,
    const float world_yaw_rate
) {
    bc_sensor_feedback_t feedback{};
    adapter_.read(plant_.data(), feedback);
    feedback.gimbal = gimbal_feedback(
        world_yaw, world_yaw_rate, feedback.imu);
    step_with_feedback(command, feedback);
}

bc_gimbal_feedback_t SimulationRunner::gimbal_feedback(
    const float world_yaw,
    const float world_yaw_rate,
    const bc_imu_feedback_t &imu
) const noexcept {
    return {
        bc_wrap_anglef(world_yaw + yaw_frame_offset_ - imu.yaw),
        world_yaw_rate - imu.yaw_rate,
    };
}

void SimulationRunner::step_with_feedback(
    const bc_operator_command_t &command,
    const bc_sensor_feedback_t &feedback
) {
    step_with_feedback_and_transform(command, feedback, nullptr);
}

void SimulationRunner::step_with_control_transform(
    const bc_operator_command_t &command,
    const bc_gimbal_feedback_t &gimbal_feedback,
    const ControlCommandTransform &transform
) {
    bc_sensor_feedback_t feedback{};
    adapter_.read(plant_.data(), feedback);
    feedback.gimbal = gimbal_feedback;
    step_with_feedback_and_transform(command, feedback, &transform);
}

void SimulationRunner::step_with_feedback_and_transform(
    const bc_operator_command_t &command,
    const bc_sensor_feedback_t &feedback,
    const ControlCommandTransform *transform
) {
    bc_actuation_t actuation{};

    feedback_ = feedback;
    bc_controller_update(
        &controller_, &feedback_,
        static_cast<float>(plant_.timestep()));
    bc_controller_set_command(&controller_, &command);
    if (transform == nullptr) {
        bc_controller_calculate(&controller_);
    } else {
        bc_control_command_t control_command{};
        bc_state_machine_input_t input{};
        bc_support_force_output_t support_force[BC_SIDE_NUM]{};
        const float roll_force =
            bc_control_core_roll_force(&controller_.control_core);
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            support_force[side] =
                controller_.control_core.support_force[side].output;
            input.nominal_axial_force[side] =
                controller_.control_core.config.support_force +
                controller_.control_core.config.roll_force_sign[side] *
                    roll_force;
        }
        input.operator_command = &controller_.operator_command;
        input.gimbal_feedback = &controller_.gimbal_feedback;
        input.state = &controller_.control_core.observer.state;
        input.leg = controller_.control_core.observer.leg;
        input.support_force = support_force;
        input.length_position_kp =
            controller_.control_core.config.length_controller.kp;
        input.length_position_kd =
            controller_.control_core.config.length_controller.kd;
        input.specific_force_norm = controller_.specific_force_norm;
        input.wheel_odometry_velocity =
            controller_.control_core.observer.forward_velocity.
                wheel_odometry;
        input.wheel_velocity_reliable =
            controller_.control_core.observer.velocity_estimator.output.
                wheel_velocity_reliable;
        input.timestep_seconds = controller_.timestep_seconds;
        bc_system_update(&controller_.system, &input, &control_command);
        if (controller_.system.motion.support_phase.state ==
            BC_SUPPORT_AIRBORNE) {
            bc_control_core_reject_wheel_velocity(
                &controller_.control_core);
        }
        (*transform)(control_command);
        bc_control_core_calculate(
            &controller_.control_core, &control_command);
    }
    bc_controller_execute(&controller_, &actuation);
    bc_controller_capture_snapshot(&controller_, &snapshot_);
    adapter_.write(plant_.data(), actuation);
    plant_.step();
}

SimulationStats SimulationRunner::run_for(const double duration_seconds) {
    const bc_operator_command_t command{};
    return run_for(duration_seconds, command);
}

SimulationStats SimulationRunner::run_for(
    const double duration_seconds,
    const bc_operator_command_t &command
) {
    if (!std::isfinite(duration_seconds) || duration_seconds < 0.0) {
        throw std::invalid_argument(
            "simulation duration must be finite and non-negative");
    }

    const double exact_steps = duration_seconds / plant_.timestep();
    const auto rounded_steps =
        static_cast<std::size_t>(std::llround(exact_steps));
    const double tolerance = 16.0 * std::numeric_limits<double>::epsilon()
        * std::max(1.0, std::abs(exact_steps));
    const double step_error =
        std::abs(exact_steps - static_cast<double>(rounded_steps));
    if (step_error > tolerance) {
        throw std::invalid_argument(
            "simulation duration must be an integer multiple of the timestep");
    }

    for (std::size_t step = 0; step < rounded_steps; ++step) {
        this->step(command);
    }

    return SimulationStats{
        rounded_steps,
        static_cast<std::size_t>(snapshot().tick_count),
        plant_.data().time,
    };
}

} // namespace balance::sim
