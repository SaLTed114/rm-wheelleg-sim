#ifndef BALANCE_TYPES_H
#define BALANCE_TYPES_H

#include <stdint.h>

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

typedef enum {
    BC_STATE_S,
    BC_STATE_DS,
    BC_STATE_PSI,
    BC_STATE_DPSI,
    BC_STATE_THETA_L,
    BC_STATE_DTHETA_L,
    BC_STATE_THETA_R,
    BC_STATE_DTHETA_R,
    BC_STATE_THETA_B,
    BC_STATE_DTHETA_B,
    BC_STATE_NUM
} bc_state_index_t;

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
    float yaw;
    float pitch;
    float yaw_rate;
    float pitch_rate;
} bc_imu_feedback_t;

typedef struct {
    bc_leg_feedback_t leg[BC_SIDE_NUM];
    bc_wheel_feedback_t wheel[BC_SIDE_NUM];
    bc_imu_feedback_t imu;
} bc_sensor_feedback_t;

typedef struct {
    float value[BC_STATE_NUM];
} bc_state_vector_t;

typedef struct {
    float length;
    float angle_body;
} bc_leg_target_t;

typedef enum {
    BC_LEG_LENGTH_DISABLED,
    BC_LEG_LENGTH_POSITION,
    BC_LEG_LENGTH_POSITION_SUPPORT
} bc_leg_length_strategy_t;

typedef enum {
    BC_LEG_ANGLE_DISABLED,
    BC_LEG_ANGLE_POSITION,
    BC_LEG_ANGLE_LQR
} bc_leg_angle_strategy_t;

typedef enum {
    BC_WHEEL_DISABLED,
    BC_WHEEL_LQR
} bc_wheel_strategy_t;

typedef struct {
    uint8_t system_enabled;
    uint8_t balance_restart;
    float forward_velocity;
    float yaw_rate;
} bc_operator_command_t;

typedef struct {
    bc_leg_length_strategy_t length_strategy;
    bc_leg_angle_strategy_t angle_strategy;
    bc_leg_target_t target;
} bc_leg_control_command_t;

typedef struct {
    bc_leg_control_command_t leg[BC_SIDE_NUM];
    bc_wheel_strategy_t wheel_strategy;
    bc_state_vector_t state_reference;
} bc_control_command_t;

typedef struct {
    float joint_torque[BC_JOINT_NUM];
} bc_leg_request_t;

typedef struct {
    bc_leg_request_t leg[BC_SIDE_NUM];
    float wheel_torque[BC_SIDE_NUM];
} bc_actuation_t;

#ifdef __cplusplus
}
#endif

#endif
