#ifndef BALANCE_CONTROL_CORE_H
#define BALANCE_CONTROL_CORE_H

#include <stdint.h>

#include "balance/pd_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BC_L,
    BC_R,
    BC_SIDE_NUM
} bc_side_t;

typedef enum {
    BC_FRONT,
    BC_REAR,
    BC_JOINT_NUM
} bc_joint_t;

typedef enum {
    BC_LEG_LENGTH,
    BC_LEG_ANGLE,
    BC_LEG_COORD_NUM
} bc_leg_coordinate_t;

typedef struct {
    float angle;
    float angular_velocity;
} bc_joint_feedback_t;

typedef struct {
    bc_joint_feedback_t joint[BC_JOINT_NUM];
} bc_leg_feedback_t;

typedef struct {
    float angle;
    float angular_velocity;
} bc_wheel_feedback_t;

typedef struct {
    bc_leg_feedback_t leg[BC_SIDE_NUM];
    bc_wheel_feedback_t wheel[BC_SIDE_NUM];
} bc_observation_t;

typedef struct {
    float length;
    float angle_body;
} bc_leg_target_t;

typedef struct {
    uint8_t enabled;
    bc_leg_target_t leg[BC_SIDE_NUM];
} bc_operator_command_t;

typedef struct {
    float joint_torque[BC_JOINT_NUM];
} bc_leg_request_t;

typedef struct {
    bc_leg_request_t leg[BC_SIDE_NUM];
    float wheel_torque[BC_SIDE_NUM];
} bc_actuation_t;

typedef struct {
    float hip_link_length;
    float wheel_link_length;
} bc_leg_geometry_t;

typedef struct {
    float length;
    float angle_body;
    float length_velocity;
    float angular_velocity;
    float jacobian[BC_LEG_COORD_NUM][BC_JOINT_NUM];
} bc_leg_kinematics_t;

typedef struct {
    bc_leg_geometry_t leg_geometry;
    bc_pd_controller_t length_controller;
    bc_pd_controller_t angle_controller;
    float joint_torque_limit;
} bc_control_config_t;

typedef struct {
    bc_control_config_t config;
    bc_leg_kinematics_t leg[BC_SIDE_NUM];
    uint32_t tick_count;
} bc_control_core_t;

void bc_control_default_config(bc_control_config_t *config);
void bc_control_core_init(
    bc_control_core_t *core,
    const bc_control_config_t *config);
void bc_control_core_reset(bc_control_core_t *core);

void bc_control_core_step(
    bc_control_core_t *core,
    const bc_observation_t *observation,
    const bc_operator_command_t *command,
    bc_actuation_t *actuation);

#ifdef __cplusplus
}
#endif

#endif
