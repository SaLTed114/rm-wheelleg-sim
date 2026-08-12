#include "balance/support_force.h"

#include <math.h>

void bc_support_force_default_config(bc_support_force_config_t *config) {
    *config = (bc_support_force_config_t){
        .filter_alpha = 0.2F,
        .ground_threshold = 15.0F,
        .air_threshold = 10.0F,
        .ground_confirm_duration = 0.015F,
        .air_confirm_duration = 0.020F,
        .wheel_mass = 0.71F,
        .gravity = 9.81F,
    };
}

void bc_support_force_init(
    bc_support_force_t *estimator,
    const bc_support_force_config_t *config
) {
    estimator->config = *config;
    bc_support_force_reset(estimator);
}

void bc_support_force_reset(bc_support_force_t *estimator) {
    estimator->output = (bc_support_force_output_t){
        .state = BC_CONTACT_GROUND,
    };
    bc_condition_hold_reset(&estimator->ground_hold);
    bc_condition_hold_reset(&estimator->air_hold);
}

void bc_support_force_update(
    bc_support_force_t *estimator,
    const bc_leg_kinematics_t *leg,
    const bc_leg_feedback_t *feedback,
    const float leg_angle_ground,
    const float timestep_seconds
) {
    const float a = leg->jacobian[BC_LEG_LENGTH][BC_FRONT];
    const float b = leg->jacobian[BC_LEG_LENGTH][BC_REAR];
    const float c = leg->jacobian[BC_LEG_ANGLE][BC_FRONT];
    const float d = leg->jacobian[BC_LEG_ANGLE][BC_REAR];
    const float determinant = a * d - b * c;
    const float front_torque = feedback->joint[BC_FRONT].torque;
    const float rear_torque = feedback->joint[BC_REAR].torque;
    if (!isfinite(determinant) || fabsf(determinant) < 1.0e-6F ||
        !isfinite(front_torque) || !isfinite(rear_torque) ||
        !isfinite(leg_angle_ground) || !isfinite(leg->length) ||
        leg->length <= 1.0e-6F) {
        estimator->output.valid = 0U;
        bc_condition_hold_reset(&estimator->ground_hold);
        bc_condition_hold_reset(&estimator->air_hold);
        return;
    }

    const float axial_force =
        (d * front_torque - c * rear_torque) / determinant;
    const float leg_torque =
        (-b * front_torque + a * rear_torque) / determinant;
    const float vertical_force =
        axial_force * cosf(leg_angle_ground) -
        (leg_torque / leg->length) * sinf(leg_angle_ground) +
        estimator->config.wheel_mass * estimator->config.gravity;
    if (!isfinite(axial_force) || !isfinite(leg_torque) ||
        !isfinite(vertical_force)) {
        estimator->output.valid = 0U;
        bc_condition_hold_reset(&estimator->ground_hold);
        bc_condition_hold_reset(&estimator->air_hold);
        return;
    }

    estimator->output.axial_force = axial_force;
    estimator->output.leg_torque = leg_torque;
    estimator->output.vertical_force = vertical_force;
    if (!estimator->output.valid) {
        estimator->output.filtered_vertical_force = vertical_force;
    } else {
        estimator->output.filtered_vertical_force +=
            estimator->config.filter_alpha *
            (vertical_force - estimator->output.filtered_vertical_force);
    }
    estimator->output.valid = 1U;

    const uint8_t ground = bc_condition_hold_update(
        &estimator->ground_hold,
        estimator->output.filtered_vertical_force >
            estimator->config.ground_threshold,
        estimator->config.ground_confirm_duration,
        timestep_seconds);
    const uint8_t air = bc_condition_hold_update(
        &estimator->air_hold,
        estimator->output.filtered_vertical_force <
            estimator->config.air_threshold,
        estimator->config.air_confirm_duration,
        timestep_seconds);
    if (estimator->output.state == BC_CONTACT_GROUND && air) {
        estimator->output.state = BC_CONTACT_AIR;
        bc_condition_hold_reset(&estimator->ground_hold);
        bc_condition_hold_reset(&estimator->air_hold);
    } else if (estimator->output.state == BC_CONTACT_AIR && ground) {
        estimator->output.state = BC_CONTACT_GROUND;
        bc_condition_hold_reset(&estimator->ground_hold);
        bc_condition_hold_reset(&estimator->air_hold);
    }
}

const char *bc_contact_state_name(const bc_contact_state_t state) {
    switch (state) {
    case BC_CONTACT_GROUND: return "ground";
    case BC_CONTACT_AIR: return "air";
    }
    return "unknown";
}
