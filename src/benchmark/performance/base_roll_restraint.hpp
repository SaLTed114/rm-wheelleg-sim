#ifndef BALANCE_BENCHMARK_BASE_ROLL_RESTRAINT_HPP
#define BALANCE_BENCHMARK_BASE_ROLL_RESTRAINT_HPP

#include <mujoco/mujoco.h>

namespace balance::benchmark {

class BaseRollRestraint {
public:
    BaseRollRestraint(const mjModel &model, bool enabled);

    void reset() noexcept;
    void apply(mjData &data, double roll, double roll_rate);

    [[nodiscard]] bool enabled() const noexcept { return enabled_; }
    [[nodiscard]] double torque() const noexcept { return torque_; }

private:
    const mjModel &model_;
    int base_body_{};
    bool enabled_{};
    double torque_{};
};

} // namespace balance::benchmark

#endif
