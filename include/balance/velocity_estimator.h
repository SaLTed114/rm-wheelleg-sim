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
    float recovery_velocity_variance;
    float nis_gate;
    float wheel_rejection_duration;
    float wheel_recovery_duration;
    float reacquisition_stable_duration;
    float reacquisition_max_wheel_speed;
    float reacquisition_max_wheel_acceleration;
    float reacquisition_velocity_rate;
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
    float velocity_variance_x;
    float rejection_elapsed_seconds;
    float recovery_elapsed_seconds;
    float reacquisition_elapsed_seconds;
    uint8_t measurement_accepted;
    uint8_t wheel_velocity_reliable;
    uint8_t reacquisition_active;
} bc_velocity_estimator_output_t;

typedef struct {
    bc_velocity_estimator_config_t config;
    float state[4];
    float covariance[4][4];
    bc_velocity_estimator_output_t output;
    float rejection_elapsed_seconds;
    float recovery_elapsed_seconds;
    float reacquisition_elapsed_seconds;
    float previous_wheel_velocity_measurement;
    uint8_t measurement_initialized;
    uint8_t wheel_velocity_reliable;
    uint8_t previous_wheel_measurement_initialized;
} bc_velocity_estimator_t;

typedef enum {
    BC_WHEEL_UPDATE_NORMAL,
    BC_WHEEL_UPDATE_REACQUIRE
} bc_wheel_update_mode_t;

void bc_velocity_estimator_init(
    bc_velocity_estimator_t *estimator,
    const bc_velocity_estimator_config_t *config);
void bc_velocity_estimator_reset(bc_velocity_estimator_t *estimator);
void bc_velocity_estimator_skip_update(
    bc_velocity_estimator_t *estimator);
void bc_velocity_estimator_reject_wheel(
    bc_velocity_estimator_t *estimator);
void bc_velocity_estimator_update(
    bc_velocity_estimator_t *estimator,
    float wheel_velocity_measurement,
    bc_wheel_update_mode_t mode,
    float timestep_seconds);
void bc_velocity_estimator_predict(
    bc_velocity_estimator_t *estimator,
    const bc_imu_feedback_t *imu,
    float timestep_seconds);

#ifdef __cplusplus
}
#endif

#endif
