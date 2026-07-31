#ifndef BALANCE_SIM_STATIC_STAND_SCENARIO_HPP
#define BALANCE_SIM_STATIC_STAND_SCENARIO_HPP

#include <array>

#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

namespace balance::sim {

enum class StaticStandPhase {
    Preparing,
    Balancing,
    PreparationTimeout,
};

class StaticStandScenario {
public:
    StaticStandScenario(MujocoPlant &plant, SimulationRunner &runner);

    void reset();
    void step();
    void set_motion_target(
        float forward_velocity, float yaw_rate) noexcept;

    [[nodiscard]] StaticStandPhase phase() const noexcept { return phase_; }
    [[nodiscard]] double release_time() const noexcept { return release_time_; }
    [[nodiscard]] const char *phase_name() const noexcept;

private:
    [[nodiscard]] bc_operator_command_t make_posture_command() const;
    [[nodiscard]] bool posture_is_stable() const;
    void update_balance_reference();
    void place_wheels_on_ground();
    void release();

    MujocoPlant &plant_;
    SimulationRunner &runner_;
    StaticStandPhase phase_{StaticStandPhase::Preparing};
    double stable_duration_{};
    double release_time_{-1.0};
    float forward_velocity_target_{};
    float yaw_rate_target_{};
    bc_operator_command_t balance_command_{};
    int base_qpos_{};
    int ground_geom_{};
    std::array<int, BC_SIDE_NUM> wheel_axis_site_{};
};

} // namespace balance::sim

#endif
