#include "simulation_runner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace balance::sim {

SimulationRunner::SimulationRunner(
    MujocoPlant &plant, const MujocoAdapter &adapter
) : plant_(plant), adapter_(adapter) {
    bc_control_core_init(&control_core_);
}

void SimulationRunner::reset() {
    plant_.reset();
    bc_control_core_reset(&control_core_);
    operator_command_ = {};
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

    bc_observation_t observation{};
    bc_actuation_t actuation{};
    for (std::size_t step = 0; step < rounded_steps; ++step) {
        adapter_.read(plant_.data(), observation);
        bc_control_core_step(
            &control_core_, &observation,
            &operator_command_, &actuation);
        adapter_.write(plant_.data(), actuation);
        plant_.step();
    }

    return SimulationStats{
        rounded_steps,
        static_cast<std::size_t>(control_core_.tick_count),
        plant_.data().time,
    };
}

} // namespace balance::sim
