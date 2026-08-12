#ifndef BALANCE_SUPPORT_FORCE_H
#define BALANCE_SUPPORT_FORCE_H

#include "balance/leg_kinematics.h"
#include "balance/state_machine/condition_hold.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BC_CONTACT_GROUND,
    BC_CONTACT_AIR,
} bc_contact_state_t;

typedef struct {
    float filter_alpha;
    float ground_threshold;
    float air_threshold;
    float ground_confirm_duration;
    float air_confirm_duration;
    float wheel_mass;
    float gravity;
} bc_support_force_config_t;

typedef struct {
    float axial_force;
    float leg_torque;
    float vertical_force;
    float filtered_vertical_force;
    bc_contact_state_t state;
    uint8_t valid;
} bc_support_force_output_t;

typedef struct {
    bc_support_force_config_t config;
    bc_support_force_output_t output;
    bc_condition_hold_t ground_hold;
    bc_condition_hold_t air_hold;
} bc_support_force_t;

void bc_support_force_default_config(bc_support_force_config_t *config);
void bc_support_force_init(
    bc_support_force_t *estimator,
    const bc_support_force_config_t *config);
void bc_support_force_reset(bc_support_force_t *estimator);
void bc_support_force_update(
    bc_support_force_t *estimator,
    const bc_leg_kinematics_t *leg,
    const bc_leg_feedback_t *feedback,
    float leg_angle_ground,
    float timestep_seconds);
const char *bc_contact_state_name(bc_contact_state_t state);

#ifdef __cplusplus
}
#endif

#endif
