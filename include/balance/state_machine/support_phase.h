#ifndef BALANCE_STATE_MACHINE_SUPPORT_PHASE_H
#define BALANCE_STATE_MACHINE_SUPPORT_PHASE_H

#include "balance/state_machine/condition_hold.h"
#include "balance/state_machine/input.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BC_SUPPORT_GROUND,
    BC_SUPPORT_AIRBORNE,
    BC_SUPPORT_LANDING_RETRACT,
    BC_SUPPORT_GROUND_RECOVER
} bc_support_phase_state_t;

typedef struct {
    float leg_speed_threshold;
    float leg_length_tolerance;
    float stable_duration;
    float unloaded_force_threshold;
    float landing_force_threshold;
    float landing_confirm_duration;
    float airborne_leg_length;
    float landing_stiffness;
    float landing_damping;
    float landing_minimum_force;
    float landing_maximum_force;
    float landing_force_rate_limit;
    float landing_retraction_speed;
    float recovery_reference_speed;
} bc_support_phase_config_t;

typedef struct {
    uint8_t override_length;
    bc_leg_length_strategy_t length_strategy;
    float target;
    uint8_t contact_latched;
    float captured_length;
    float equilibrium_length;
    float requested_force;
    float applied_force;
    uint8_t force_rate_limited;
} bc_support_leg_request_t;

typedef struct {
    uint8_t disable_wheels;
    uint16_t disabled_state_feedback;
    bc_support_leg_request_t leg[BC_SIDE_NUM];
} bc_support_phase_request_t;

typedef struct {
    bc_support_phase_config_t config;
    bc_support_phase_state_t state;
    bc_condition_hold_t transition_hold;
    bc_condition_hold_t landing_hold;
    bc_support_phase_request_t request;
} bc_support_phase_t;

void bc_support_phase_default_config(bc_support_phase_config_t *config);
void bc_support_phase_init(
    bc_support_phase_t *phase,
    const bc_support_phase_config_t *config);
void bc_support_phase_reset(bc_support_phase_t *phase);
void bc_support_phase_update(
    bc_support_phase_t *phase,
    const bc_state_machine_input_t *input,
    float working_leg_length);

const char *bc_support_phase_state_name(bc_support_phase_state_t state);

#ifdef __cplusplus
}
#endif

#endif
