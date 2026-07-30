#ifndef BALANCE_SIM_SIMULATION_RUNNER_HPP
#define BALANCE_SIM_SIMULATION_RUNNER_HPP

#include <cstddef>

#include "balance/control_core.h"
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
    void step();
    [[nodiscard]] SimulationStats run_for(double duration_seconds);

private:
    MujocoPlant &plant_;
    const MujocoAdapter &adapter_;
    bc_control_core_t control_core_{};
    bc_operator_command_t operator_command_{};
};

} // namespace balance::sim

#endif
