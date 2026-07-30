#include "balance/leg_kinematics.h"

#include <math.h>

void bc_leg_kinematics_calculate(
    const bc_leg_geometry_t *geometry,
    const bc_leg_feedback_t *feedback,
    bc_leg_kinematics_t *kinematics
) {
    const float phi_front = feedback->joint[BC_FRONT].angle;
    const float phi_rear  = feedback->joint[BC_REAR].angle;

    const float dot_phi_front = feedback->joint[BC_FRONT].angular_velocity;
    const float dot_phi_rear  = feedback->joint[BC_REAR].angular_velocity;

    const float l1    = geometry->hip_link_length;
    const float l2    = geometry->wheel_link_length;
    const float l1_sq = l1 * l1;

    const float delta     = 0.5F * (phi_front - phi_rear);
    const float sin_delta = sinf(delta);
    const float cos_delta = cosf(delta);

    float radicand = l2 * l2 - l1_sq * sin_delta * sin_delta;
    if (radicand < 0.0F) radicand = 0.0F;

    const float sqrt_term = sqrtf(radicand);
    const float denom = sqrt_term > 1.0e-6F ? sqrt_term : 1.0e-6F;
    const float dlen_dphi_front = 0.5F * (-l1 * sin_delta - l1_sq * sin_delta * cos_delta / denom);

    kinematics->length     = l1 * cos_delta + sqrt_term;
    kinematics->angle_body = 0.5F * (phi_front + phi_rear);

    kinematics->length_velocity  = (dot_phi_front - dot_phi_rear) * dlen_dphi_front;
    kinematics->angular_velocity = (dot_phi_front + dot_phi_rear) * 0.5F;

    kinematics->jacobian[BC_LEG_LENGTH][BC_FRONT] = +dlen_dphi_front;
    kinematics->jacobian[BC_LEG_LENGTH][BC_REAR ] = -dlen_dphi_front;
    kinematics->jacobian[BC_LEG_ANGLE ][BC_FRONT] = 0.5F;
    kinematics->jacobian[BC_LEG_ANGLE ][BC_REAR ] = 0.5F;
}
