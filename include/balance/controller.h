#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include "balance/control_core.h"
#include "balance/state_machine/system.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bc_control_config_t control;
    bc_motion_config_t motion;
    float velocity_estimator_update_delay;
} bc_controller_config_t;

typedef struct {
    bc_control_core_t control_core;
    bc_system_t system;
    bc_operator_command_t operator_command;
    bc_gimbal_feedback_t gimbal_feedback;
    bc_actuation_t last_actuation;
    bc_condition_hold_t velocity_estimator_hold;
    float velocity_estimator_update_delay;
    float specific_force_norm;
    float timestep_seconds;
} bc_controller_t;

void bc_controller_default_config(bc_controller_config_t *config);
void bc_controller_init(
    bc_controller_t *controller,
    const bc_controller_config_t *config);
void bc_controller_reset(bc_controller_t *controller);

void bc_controller_update(
    bc_controller_t *controller,
    const bc_sensor_feedback_t *feedback,
    float timestep_seconds);
void bc_controller_set_command(
    bc_controller_t *controller,
    const bc_operator_command_t *command);
void bc_controller_calculate(bc_controller_t *controller);
void bc_controller_execute(
    bc_controller_t *controller,
    bc_actuation_t *actuation);

#ifdef __cplusplus
}
#endif

#endif
