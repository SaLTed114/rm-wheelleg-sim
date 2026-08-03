#ifndef BALANCE_BENCHMARK_SIMULATION_SAMPLE_HPP
#define BALANCE_BENCHMARK_SIMULATION_SAMPLE_HPP

#include <array>
#include <string>

#include <mujoco/mujoco.h>

#include "balance/controller_snapshot.h"

namespace balance::benchmark {

struct BaseState {
    double x{};
    double y{};
    double z{};
    double forward_velocity{};
    double vertical_velocity{};
};

struct GroundContactState {
    std::array<bool, BC_SIDE_NUM> wheel{};
    std::array<double, BC_SIDE_NUM> wheel_normal_force{};
    bool other{};
    std::string unexpected;
};

struct SimulationSample {
    double time;
    const bc_controller_snapshot_t &controller;
    BaseState base;
    GroundContactState contact;
};

class SimulationSampler {
public:
    explicit SimulationSampler(const mjModel &model);

    [[nodiscard]] SimulationSample read(
        const mjData &data,
        const bc_controller_snapshot_t &controller) const;

private:
    [[nodiscard]] GroundContactState read_contacts(
        const mjData &data) const;
    [[nodiscard]] std::string contact_name(
        const mjContact &contact) const;

    const mjModel &model_;
    int base_qpos_{};
    int base_dof_{};
    int ground_{};
    std::array<int, BC_SIDE_NUM> wheel_{};
};

} // namespace balance::benchmark

#endif
