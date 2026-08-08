#include "simulation_runner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

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
}

void SimulationRunner::reset() {
    plant_.reset();
    bc_controller_reset(&controller_);
    bc_controller_capture_snapshot(&controller_, &snapshot_);
    feedback_ = {};
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

void SimulationRunner::step_with_feedback(
    const bc_operator_command_t &command,
    const bc_sensor_feedback_t &feedback
) {
    bc_actuation_t actuation{};

    feedback_ = feedback;
    bc_controller_update(
        &controller_, &feedback_,
        static_cast<float>(plant_.timestep()));
    bc_controller_set_command(&controller_, &command);
    bc_controller_calculate(&controller_);
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
