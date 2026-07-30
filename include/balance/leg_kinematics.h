#ifndef BALANCE_LEG_KINEMATICS_H
#define BALANCE_LEG_KINEMATICS_H

#include "balance/control_core.h"

#ifdef __cplusplus
extern "C" {
#endif

void bc_leg_kinematics_calculate(
    const bc_leg_geometry_t *geometry,
    const bc_leg_feedback_t *feedback,
    bc_leg_kinematics_t *kinematics);

#ifdef __cplusplus
}
#endif

#endif
