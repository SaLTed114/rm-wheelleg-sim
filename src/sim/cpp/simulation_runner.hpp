#ifndef BALANCE_SIM_CPP_SIMULATION_RUNNER_HPP
#define BALANCE_SIM_CPP_SIMULATION_RUNNER_HPP

#include "balance_cpp/controller.hpp"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"

namespace balance::sim::cpp {

class SimulationRunner {
public:
    SimulationRunner(MujocoPlant &plant, const MujocoAdapter &adapter);
    SimulationRunner(
        MujocoPlant &plant,
        const MujocoAdapter &adapter,
        const control::ControllerConfig &config
    );

    void reset();
    void step(const control::OperatorCommand &command);

    const control::Snapshot &snapshot() const {
        return output_.snapshot;
    }
    const control::ControllerOutput &output() const {
        return output_;
    }

private:
    static control::SensorFrame convert_sensor(
        const bc_sensor_feedback_t &feedback
    );
    static bc_actuation_t convert_actuation(
        const control::Actuation &actuation
    );

    MujocoPlant &plant_;
    const MujocoAdapter &adapter_;
    control::Controller controller_;
    control::ControllerOutput output_{};
};

} // namespace balance::sim::cpp

#endif
