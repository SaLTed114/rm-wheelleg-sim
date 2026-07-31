#include "simulation_runner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace balance::sim {

SimulationRunner::SimulationRunner(
    MujocoPlant &plant, const MujocoAdapter &adapter
) : plant_(plant), adapter_(adapter) {
    bc_control_config_t config{};
    bc_control_default_config(&config);
    bc_control_core_init(&control_core_, &config);
}

void SimulationRunner::reset() {
    plant_.reset();
    bc_control_core_reset(&control_core_);
    actuation_ = {};
}

void SimulationRunner::set_command(
    const bc_operator_command_t &command
) {
    bc_control_core_set_command(&control_core_, &command);
}

void SimulationRunner::step() {
    bc_sensor_feedback_t feedback{};

    adapter_.read(plant_.data(), feedback);
    bc_control_core_update(
        &control_core_, &feedback,
        static_cast<float>(plant_.timestep()));
    bc_control_core_execute(&control_core_, &actuation_);
    adapter_.write(plant_.data(), actuation_);
    plant_.step();
}

SimulationStats SimulationRunner::run_for(const double duration_seconds) {
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
        this->step();
    }

    return SimulationStats{
        rounded_steps,
        static_cast<std::size_t>(control_core_.tick_count),
        plant_.data().time,
    };
}

} // namespace balance::sim
