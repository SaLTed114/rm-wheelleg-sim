#ifndef BALANCE_STATE_MACHINE_INPUT_H
#define BALANCE_STATE_MACHINE_INPUT_H

#include "balance/impact_observer.h"
#include "balance/leg_kinematics.h"
#include "balance/support_force.h"
#include "balance/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const bc_operator_command_t *operator_command;
    const bc_gimbal_feedback_t *gimbal_feedback;
    const bc_state_vector_t *state;
    const bc_leg_kinematics_t *leg;
    const bc_support_force_output_t *support_force;
    const bc_impact_observer_output_t *impact_observer;
    float nominal_axial_force[BC_SIDE_NUM];
    float length_position_kp;
    float length_position_kd;
    float specific_force_norm;
    float wheel_odometry_velocity;
    uint8_t wheel_velocity_reliable;
    float timestep_seconds;
} bc_state_machine_input_t;

#ifdef __cplusplus
}
#endif

#endif
