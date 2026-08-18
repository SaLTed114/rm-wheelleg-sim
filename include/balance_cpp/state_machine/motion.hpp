#ifndef BALANCE_CPP_MOTION_STATE_MACHINE_HPP
#define BALANCE_CPP_MOTION_STATE_MACHINE_HPP

#include "balance_cpp/math.hpp"
#include "balance_cpp/state_machine/condition_hold.hpp"
#include "balance_cpp/types.hpp"

namespace balance::control {

struct MotionConfig {
    float startup_leg_length{0.18F};
    float startup_leg_angle{-0.5F * math::pi};
    float length_tolerance{0.035F};
    float length_velocity_tolerance{0.03F};
    float angle_tolerance{math::radians(8.0F)};
    float angular_velocity_tolerance{0.15F};
    float stable_duration{0.10F};
    float engage_duration{0.05F};
};

class MotionStateMachine {
public:
    explicit MotionStateMachine(MotionConfig config = {});

    void reset();
    void start();
    [[nodiscard]] ControlCommand update(
        const Estimate &estimate, float timestep);
    MotionStatus status() const;
    bool wheel_observation_enabled() const;

private:
    void transition(const Estimate &estimate, float timestep);
    ControlCommand action() const;
    bool legs_stable(const Estimate &estimate) const;

    MotionConfig config_{};
    MotionState state_{MotionState::idle};
    StateVector state_reference_{};
    ConditionHold leg_stable_hold_{};
    ConditionHold engage_hold_{};
};

} // namespace balance::control

#endif
