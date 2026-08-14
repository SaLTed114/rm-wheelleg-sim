#ifndef BALANCE_OBSERVER_H
#define BALANCE_OBSERVER_H

#include "balance/leg_kinematics.h"
#include "balance/impact_observer.h"
#include "balance/observation_context.h"
#include "balance/state_machine/condition_hold.h"
#include "balance/types.h"
#include "balance/velocity_estimator.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} bc_body_point_t;

typedef struct {
    float wheel_odometry;
    float estimated_axle;
} bc_forward_velocity_output_t;

typedef struct {
    bc_leg_geometry_t leg_geometry;
    bc_impact_observer_config_t impact_observer;
    bc_velocity_estimator_config_t velocity_estimator;
    bc_body_point_t imu_position;
    bc_body_point_t hip_center_position;
    float wheel_radius;
    float wheel_velocity_startup_delay;
} bc_observer_config_t;

typedef struct {
    bc_observer_config_t config;
    bc_leg_kinematics_t leg[BC_SIDE_NUM];
    bc_impact_observer_t impact_observer;
    bc_velocity_estimator_t velocity_estimator;
    bc_forward_velocity_output_t forward_velocity;
    bc_state_vector_t state;
    bc_condition_hold_t wheel_velocity_startup_hold;
    float roll;
    float roll_rate;
    float previous_yaw;
    float yaw;
    uint8_t initialized;
} bc_observer_t;

void bc_observer_init(
    bc_observer_t *observer,
    const bc_observer_config_t *config);
void bc_observer_reset(bc_observer_t *observer);

void bc_observer_update(
    bc_observer_t *observer,
    const bc_sensor_feedback_t *feedback,
    const bc_observation_context_t *context,
    float timestep_seconds);
void bc_observer_reject_wheel_velocity(bc_observer_t *observer);

#ifdef __cplusplus
}
#endif

#endif
