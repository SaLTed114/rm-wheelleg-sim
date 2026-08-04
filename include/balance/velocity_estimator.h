#ifndef BALANCE_VELOCITY_ESTIMATOR_H
#define BALANCE_VELOCITY_ESTIMATOR_H

#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float gravity;
    float initial_velocity_variance;
    float initial_bias_variance;
    float acceleration_variance;
    float bias_walk_variance;
    float wheel_velocity_variance;
    float nis_gate;
} bc_velocity_estimator_config_t;

typedef struct {
    float prior_velocity_x;
    float prior_velocity_y;
    float velocity_x;
    float velocity_y;
    float acceleration_bias_x;
    float acceleration_bias_y;
    float linear_acceleration_x;
    float linear_acceleration_y;
    float wheel_velocity_measurement;
    float innovation;
    float innovation_variance;
    float nis;
    uint8_t measurement_accepted;
} bc_velocity_estimator_output_t;

typedef struct {
    bc_velocity_estimator_config_t config;
    float state[4];
    float covariance[4][4];
    bc_velocity_estimator_output_t output;
    uint8_t measurement_initialized;
} bc_velocity_estimator_t;

void bc_velocity_estimator_init(
    bc_velocity_estimator_t *estimator,
    const bc_velocity_estimator_config_t *config);
void bc_velocity_estimator_reset(bc_velocity_estimator_t *estimator);
void bc_velocity_estimator_skip_update(
    bc_velocity_estimator_t *estimator);
void bc_velocity_estimator_update(
    bc_velocity_estimator_t *estimator,
    float wheel_velocity_measurement);
void bc_velocity_estimator_predict(
    bc_velocity_estimator_t *estimator,
    const bc_imu_feedback_t *imu,
    float timestep_seconds);

#ifdef __cplusplus
}
#endif

#endif
