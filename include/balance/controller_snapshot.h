#ifndef BALANCE_CONTROLLER_SNAPSHOT_H
#define BALANCE_CONTROLLER_SNAPSHOT_H

#include "balance/controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bc_system_state_t system;
    bc_motion_state_t motion;
    bc_forward_state_t forward;
    bc_support_phase_state_t support;
    bc_chassis_alignment_t alignment;
} bc_state_machine_snapshot_t;

typedef struct {
    bc_state_machine_snapshot_t state_machine;
    bc_state_vector_t state;
    bc_state_vector_t state_reference;
    float yaw_acceleration_reference;
    float roll;
    float roll_rate;
    float roll_force_request;
    float specific_force_norm;
    bc_gimbal_feedback_t gimbal;
    float mapped_forward_velocity;
    float heading_error;
    bc_forward_velocity_output_t forward_velocity;
    bc_impact_observer_output_t impact_observer;
    bc_velocity_estimator_output_t velocity_estimator;
    bc_leg_kinematics_t leg[BC_SIDE_NUM];
    bc_support_force_output_t support_force[BC_SIDE_NUM];
    bc_support_phase_request_t support_request;
    bc_actuation_t actuation_request;
    bc_actuation_t actuation;
    uint32_t tick_count;
} bc_controller_snapshot_t;

void bc_controller_capture_snapshot(
    const bc_controller_t *controller,
    bc_controller_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
