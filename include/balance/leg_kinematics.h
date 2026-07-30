#ifndef BALANCE_LEG_KINEMATICS_H
#define BALANCE_LEG_KINEMATICS_H

#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

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

void bc_leg_kinematics_calculate(
    const bc_leg_geometry_t *geometry,
    const bc_leg_feedback_t *feedback,
    bc_leg_kinematics_t *kinematics);

#ifdef __cplusplus
}
#endif

#endif
