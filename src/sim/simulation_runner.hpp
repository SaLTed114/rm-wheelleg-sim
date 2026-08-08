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
    SimulationRunner(
        MujocoPlant &plant,
        const MujocoAdapter &adapter,
        const bc_controller_config_t &config);

    void reset();
    void step(const bc_operator_command_t &command);
    void step(
        const bc_operator_command_t &command,
        const bc_gimbal_feedback_t &gimbal_feedback);
    void step_with_gimbal_heading(
        const bc_operator_command_t &command,
        float world_yaw,
        float world_yaw_rate);
    void step_with_feedback(
        const bc_operator_command_t &command,
        const bc_sensor_feedback_t &feedback);
    [[nodiscard]] bc_gimbal_feedback_t gimbal_feedback(
        float world_yaw,
        float world_yaw_rate,
        const bc_imu_feedback_t &imu) const noexcept;
    [[nodiscard]] SimulationStats run_for(double duration_seconds);
    [[nodiscard]] SimulationStats run_for(
        double duration_seconds,
        const bc_operator_command_t &command);
    [[nodiscard]] const bc_controller_snapshot_t &snapshot() const noexcept {
        return snapshot_;
    }
    [[nodiscard]] const bc_sensor_feedback_t &feedback() const noexcept {
        return feedback_;
    }

private:
    MujocoPlant &plant_;
    const MujocoAdapter &adapter_;
    bc_controller_t controller_{};
    bc_controller_snapshot_t snapshot_{};
    bc_sensor_feedback_t feedback_{};
    float yaw_frame_offset_{};
};

} // namespace balance::sim

#endif
