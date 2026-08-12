#ifndef BALANCE_BENCHMARK_DROP_SCENARIO_HPP
#define BALANCE_BENCHMARK_DROP_SCENARIO_HPP

#include <array>
#include <string>

#include "common/simulation_sample.hpp"
#include "drop_benchmark.hpp"
#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

namespace balance::benchmark {

enum class DropPhase {
    disabled_settle,
    standing,
    airborne,
    post_touchdown,
    complete,
    failed,
};

class DropScenario {
public:
    DropScenario(const DropCaseSpec &spec, const mjModel &model);

    void reset() noexcept;
    void step(
        sim::MujocoPlant &plant,
        sim::SimulationRunner &runner,
        const SimulationSampler &sampler);

    [[nodiscard]] const DropCaseSpec &spec() const noexcept { return spec_; }
    [[nodiscard]] const std::string &name() const noexcept { return name_; }
    [[nodiscard]] DropPhase phase() const noexcept { return phase_; }
    [[nodiscard]] const char *phase_name() const noexcept;
    [[nodiscard]] const char *issue() const noexcept { return issue_; }
    [[nodiscard]] bool finished() const noexcept {
        return phase_ == DropPhase::complete || phase_ == DropPhase::failed;
    }
    [[nodiscard]] bool balance_engaged() const noexcept {
        return balance_engaged_;
    }
    [[nodiscard]] bool touchdown_latched() const noexcept {
        return touchdown_latch_.latched();
    }
    [[nodiscard]] const std::array<double, BC_SIDE_NUM> &
    release_clearance() const noexcept {
        return release_clearance_;
    }

private:
    [[nodiscard]] std::array<double, BC_SIDE_NUM> wheel_clearance(
        sim::MujocoPlant &plant) const;
    void release(sim::MujocoPlant &plant);

    DropCaseSpec spec_;
    std::string name_;
    DropPhase phase_{DropPhase::disabled_settle};
    DropContactLatch touchdown_latch_;
    bc_operator_command_t command_{};
    double active_start_time_{-1.0};
    double settle_start_time_{-1.0};
    double release_time_{};
    double touchdown_time_{};
    std::array<double, BC_SIDE_NUM> release_clearance_{};
    bool balance_engaged_{};
    bool heading_initialized_{};
    float held_heading_{};
    const char *issue_{"none"};
    int base_qpos_{};
    int base_dof_{};
    int ground_{};
    std::array<int, BC_SIDE_NUM> wheel_{};
};

} // namespace balance::benchmark

#endif
