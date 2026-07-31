#ifndef BALANCE_CONTROL_CORE_H
#define BALANCE_CONTROL_CORE_H

#include "balance/observer.h"
#include "balance/pd_controller.h"
#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bc_observer_config_t observer;
    bc_pd_controller_t length_controller;
    bc_pd_controller_t angle_controller;
    float support_force;
    float wheel_torque_limit;
    float joint_torque_limit;
} bc_control_config_t;

typedef struct {
    bc_control_config_t config;
    bc_observer_t observer;
    bc_operator_command_t command;
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
    float timestep_seconds);
void bc_control_core_set_command(
    bc_control_core_t *core,
    const bc_operator_command_t *command);
void bc_control_core_execute(
    bc_control_core_t *core,
    bc_actuation_t *actuation);

#ifdef __cplusplus
}
#endif

#endif
