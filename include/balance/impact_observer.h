#ifndef BALANCE_IMPACT_OBSERVER_H
#define BALANCE_IMPACT_OBSERVER_H

#include "balance/leg_kinematics.h"
#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    BC_IMPACT_WINDOW_SHORT,
    BC_IMPACT_WINDOW_LONG,
    BC_IMPACT_WINDOW_NUM
};

enum { BC_IMPACT_HISTORY_CAPACITY = 64 };

typedef struct {
    float gravity;
    float window_seconds[BC_IMPACT_WINDOW_NUM];
} bc_impact_observer_config_t;

typedef struct {
    float duration_seconds;
    float forward_delta_velocity;
    float vertical_delta_velocity;
    float pitch_rate_delta;
    float leg_rate_delta[BC_SIDE_NUM];
    float wheel_velocity_delta;
    float wheel_imu_delta_mismatch;
    uint8_t valid;
} bc_impact_window_output_t;

typedef struct {
    float forward_acceleration;
    float vertical_acceleration;
    bc_impact_window_output_t window[BC_IMPACT_WINDOW_NUM];
    uint8_t valid;
} bc_impact_observer_output_t;

typedef struct {
    float duration_seconds;
    float forward_acceleration_start;
    float forward_acceleration_end;
    float vertical_acceleration_start;
    float vertical_acceleration_end;
    float pitch_rate_start;
    float pitch_rate_end;
    float leg_rate_start[BC_SIDE_NUM];
    float leg_rate_end[BC_SIDE_NUM];
    float wheel_velocity_start;
    float wheel_velocity_end;
} bc_impact_history_interval_t;

typedef struct {
    bc_impact_observer_config_t config;
    bc_impact_observer_output_t output;
    bc_impact_history_interval_t history[BC_IMPACT_HISTORY_CAPACITY];
    unsigned int history_start;
    unsigned int history_count;
    float previous_forward_acceleration;
    float previous_vertical_acceleration;
    float previous_pitch_rate;
    float previous_leg_rate[BC_SIDE_NUM];
    float previous_wheel_velocity;
    uint8_t initialized;
} bc_impact_observer_t;

void bc_impact_observer_default_config(
    bc_impact_observer_config_t *config);
void bc_impact_observer_init(
    bc_impact_observer_t *observer,
    const bc_impact_observer_config_t *config);
void bc_impact_observer_reset(bc_impact_observer_t *observer);
void bc_impact_observer_update(
    bc_impact_observer_t *observer,
    const bc_imu_feedback_t *imu,
    const bc_leg_kinematics_t leg[BC_SIDE_NUM],
    float wheel_velocity,
    float timestep_seconds);

#ifdef __cplusplus
}
#endif

#endif
