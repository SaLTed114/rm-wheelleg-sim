#ifndef BALANCE_STATE_MACHINE_STEP_TASK_H
#define BALANCE_STATE_MACHINE_STEP_TASK_H

#include "balance/state_machine/condition_hold.h"
#include "balance/state_machine/input.h"
#include "balance/state_machine/support_phase.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BC_STEP_TASK_INACTIVE,
    BC_STEP_TASK_PREPARE,
    BC_STEP_TASK_IMPACT_PASSIVE,
    BC_STEP_TASK_TRANSFER,
    BC_STEP_TASK_TRANSFER_HOLD,
    BC_STEP_TASK_RECOVER,
    BC_STEP_TASK_RECOVER_LOCK,
    BC_STEP_TASK_COMPLETE,
    BC_STEP_TASK_RECOVERY_FAILED
} bc_step_task_state_t;

typedef enum {
    BC_STEP_CONTROL_NORMAL,
    BC_STEP_CONTROL_PASSIVE,
    BC_STEP_CONTROL_TRANSFER,
    BC_STEP_CONTROL_RECOVER
} bc_step_control_mode_t;

typedef struct {
    bc_step_control_mode_t control_mode;
    uint8_t active;
    uint8_t force_front_alignment;
    uint8_t suppress_forward;
    uint8_t recovery_entered;
    uint8_t recovery_reference_capture;
    uint8_t suppress_position_heading_feedback;
    float working_leg_length;
    float leg_length[BC_SIDE_NUM];
    float leg_angle_body[BC_SIDE_NUM];
} bc_step_task_request_t;

typedef struct {
    float prepare_leg_length;
    float leg_length_tolerance;
    float alignment_tolerance;
    float minimum_forward_velocity;
    float leg_rate_delta_threshold;
    float wheel_imu_mismatch_threshold;
    float impact_confirm_duration;
    float impact_passive_duration;
    float transfer_first_time;
    float transfer_second_time;
    float transfer_third_time;
    float transfer_end_time;
    float transfer_hold_duration;
    float transfer_first_length;
    float transfer_second_length;
    float transfer_third_length;
    float transfer_final_length;
    float transfer_first_angle_body;
    float transfer_second_angle_body;
    float transfer_third_angle_body;
    float transfer_final_angle_body;
    float recovery_pitch_tolerance;
    float recovery_pitch_rate_tolerance;
    float recovery_roll_tolerance;
    float recovery_roll_rate_tolerance;
    float recovery_velocity_tolerance;
    float recovery_yaw_rate_tolerance;
    float recovery_leg_length_tolerance;
    float recovery_leg_speed_tolerance;
    float recovery_leg_angle_tolerance;
    float recovery_leg_angular_velocity_tolerance;
    float recovery_stable_duration;
    float recovery_timeout;
} bc_step_task_config_t;

typedef struct {
    bc_step_task_config_t config;
    bc_step_task_state_t state;
    bc_condition_hold_t impact_hold;
    bc_condition_hold_t recovery_hold;
    bc_step_task_request_t request;
    float state_elapsed_seconds;
    float recovery_elapsed_seconds;
    float transfer_start_length[BC_SIDE_NUM];
    float transfer_start_angle_body[BC_SIDE_NUM];
    float transfer_length_reference[BC_SIDE_NUM];
    float transfer_angle_reference[BC_SIDE_NUM];
    uint8_t impact_armed;
    uint8_t command_rearm_required;
    uint8_t recovery_timed_out;
    uint8_t recovery_reference_captured;
} bc_step_task_t;

void bc_step_task_default_config(bc_step_task_config_t *config);
void bc_step_task_init(
    bc_step_task_t *task,
    const bc_step_task_config_t *config);
void bc_step_task_reset(bc_step_task_t *task);

void bc_step_task_update(
    bc_step_task_t *task,
    const bc_state_machine_input_t *input,
    bc_support_phase_state_t support_state,
    float heading_error);

const char *bc_step_task_state_name(bc_step_task_state_t state);

#ifdef __cplusplus
}
#endif

#endif
