#ifndef BALANCE_OBSERVER_H
#define BALANCE_OBSERVER_H

#include "balance/leg_kinematics.h"
#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bc_leg_geometry_t leg_geometry;
    float wheel_radius;
} bc_observer_config_t;

typedef struct {
    bc_observer_config_t config;
    bc_leg_kinematics_t leg[BC_SIDE_NUM];
    bc_state_vector_t state;
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
    float timestep_seconds);

#ifdef __cplusplus
}
#endif

#endif
