#ifndef BALANCE_CONTROL_CORE_H
#define BALANCE_CONTROL_CORE_H

#include "balance/control_law/pd.h"
#include "balance/observer.h"
#include "balance/support_force.h"
#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float leg_angle_trim;
    float yaw_acceleration_feedforward_scale;
} bc_lqr_compensation_t;

typedef struct {
    bc_observer_config_t observer;
    bc_pd_controller_t length_controller;
    bc_pd_controller_t angle_controller;
    bc_pd_controller_t roll_controller;
    float roll_force_sign[BC_SIDE_NUM];
    bc_lqr_compensation_t lqr_compensation;
    bc_support_force_config_t support_force_estimator;
    float support_force;
    float wheel_torque_limit;
    float joint_torque_limit;
} bc_control_config_t;

typedef struct {
    bc_control_config_t config;
    bc_observer_t observer;
    bc_actuation_t actuation_request;
    bc_support_force_t support_force[BC_SIDE_NUM];
    float roll_force_request;
    uint32_t tick_count;
} bc_control_core_t;

void bc_control_default_config(bc_control_config_t *config);
void bc_control_core_init(
    bc_control_core_t *core,
    const bc_control_config_t *config);
void bc_control_core_reset(bc_control_core_t *core);

void bc_control_core_update(
    bc_control_core_t *core,
    const bc_sensor_feedback_t *feedback,
    float timestep_seconds,
    uint8_t wheel_velocity_update_enabled);
void bc_control_core_reject_wheel_velocity(bc_control_core_t *core);
void bc_control_core_calculate(
    bc_control_core_t *core,
    const bc_control_command_t *command);
float bc_control_core_roll_force(const bc_control_core_t *core);
void bc_control_core_execute(
    bc_control_core_t *core,
    uint8_t output_enabled,
    bc_actuation_t *actuation);

#ifdef __cplusplus
}
#endif

#endif
