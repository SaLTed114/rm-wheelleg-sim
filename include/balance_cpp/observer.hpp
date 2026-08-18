#ifndef BALANCE_CPP_OBSERVER_HPP
#define BALANCE_CPP_OBSERVER_HPP

#include "balance_cpp/leg_kinematics.hpp"
#include "balance_cpp/state_machine/condition_hold.hpp"
#include "balance_cpp/types.hpp"
#include "balance_cpp/velocity_estimator.hpp"

namespace balance::control {

struct Vector3 {
    float x{};
    float y{};
    float z{};
};

struct ObserverConfig {
    LegKinematicsConfig leg_kinematics{};
    VelocityEstimatorConfig velocity_estimator{};
    float wheel_radius{0.05806F};
    Vector3 imu_position{-0.10F, 0.0F, -0.03F};
    Vector3 hip_center_position{-0.0193914F, 0.0F, -0.05F};
    float wheel_velocity_startup_delay{0.5F};
};

class Observer {
public:
    explicit Observer(ObserverConfig config = {});

    void reset();
    [[nodiscard]] Estimate update(
        const SensorFrame &sensor,
        const ObservationContext &context,
        float timestep
    );
    const Estimate &estimate() const {
        return estimate_;
    }

private:
    ObserverConfig config_{};
    LegKinematicsSolver leg_kinematics_{};
    VelocityEstimator velocity_estimator_{};
    ConditionHold wheel_startup_hold_{};
    Estimate estimate_{};
    float previous_yaw_{};
    float yaw_{};
    bool initialized_{};
};

} // namespace balance::control

#endif
