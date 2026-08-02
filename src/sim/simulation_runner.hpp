#ifndef BALANCE_SIM_SIMULATION_RUNNER_HPP
#define BALANCE_SIM_SIMULATION_RUNNER_HPP

#include <cstddef>

#include "balance/controller.h"
#include "balance/controller_snapshot.h"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"

namespace balance::sim {

struct SimulationStats {
    std::size_t physics_steps;
    std::size_t control_ticks;
    double final_time;
};

class SimulationRunner {
public:
    SimulationRunner(MujocoPlant &plant, const MujocoAdapter &adapter);

    void reset();
    void step(const bc_operator_command_t &command);
    [[nodiscard]] SimulationStats run_for(double duration_seconds);
    [[nodiscard]] SimulationStats run_for(
        double duration_seconds,
        const bc_operator_command_t &command);
    [[nodiscard]] const bc_controller_snapshot_t &snapshot() const noexcept {
        return snapshot_;
    }

private:
    MujocoPlant &plant_;
    const MujocoAdapter &adapter_;
    bc_controller_t controller_{};
    bc_controller_snapshot_t snapshot_{};
};

} // namespace balance::sim

#endif
