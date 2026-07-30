#include "balance/leg_kinematics.h"

#include <math.h>
#include <stdio.h>

static int expect_near(
    const char *name, float actual, float expected, float tolerance
) {
    const float error = fabsf(actual - expected);
    if (error <= tolerance) return 0;

    fprintf(
        stderr, "%s: expected %.9f, got %.9f (error %.9f)\n",
        name, expected, actual, error);
    return 1;
}

static bc_leg_kinematics_t calculate(
    const bc_leg_geometry_t *geometry, float phi_front, float phi_rear,
    float dot_phi_front, float dot_phi_rear
) {
    const bc_leg_feedback_t feedback = {
        .joint = {
            [BC_FRONT] = {
                .angle = phi_front,
                .angular_velocity = dot_phi_front,
            },
            [BC_REAR] = {
                .angle = phi_rear,
                .angular_velocity = dot_phi_rear,
            },
        },
    };
    bc_leg_kinematics_t kinematics;
    bc_leg_kinematics_calculate(geometry, &feedback, &kinematics);
    return kinematics;
}

int main() {
    const bc_leg_geometry_t geometry = {
        .hip_link_length   = 0.215F,
        .wheel_link_length = 0.254F,
    };
    int failures = 0;

    const bc_leg_kinematics_t symmetric = calculate(
        &geometry, -1.2F, -1.2F, 0.0F, 0.0F);
    failures += expect_near(
        "symmetric length", symmetric.length, 0.469F, 1.0e-6F);
    failures += expect_near(
        "symmetric angle", symmetric.angle_body, -1.2F, 1.0e-6F);
    failures += expect_near(
        "symmetric front length Jacobian",
        symmetric.jacobian[BC_LEG_LENGTH][BC_FRONT], 0.0F, 1.0e-6F);
    failures += expect_near(
        "symmetric rear length Jacobian",
        symmetric.jacobian[BC_LEG_LENGTH][BC_REAR], 0.0F, 1.0e-6F);

    const float poses[][BC_JOINT_NUM] = {
        {-2.80F, -0.20F},
        {-2.55F, -0.60F},
        {-2.30F, -0.90F},
    };
    const float epsilon = 1.0e-3F;
    for (int pose = 0; pose < 3; ++pose) {
        const float phi_front = poses[pose][BC_FRONT];
        const float phi_rear  = poses[pose][BC_REAR];
        const bc_leg_kinematics_t current = calculate(
            &geometry, phi_front, phi_rear, 0.7F, -0.3F);

        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            float plus_front  = phi_front;
            float plus_rear   = phi_rear;
            float minus_front = phi_front;
            float minus_rear  = phi_rear;

            if (joint == BC_FRONT) {
                plus_front += epsilon;
                minus_front -= epsilon;
            } else {
                plus_rear += epsilon;
                minus_rear -= epsilon;
            }

            const bc_leg_kinematics_t plus = calculate(
                &geometry, plus_front, plus_rear, 0.0F, 0.0F);
            const bc_leg_kinematics_t minus = calculate(
                &geometry, minus_front, minus_rear, 0.0F, 0.0F);

            const float numerical_length =
                (plus.length - minus.length) / (2.0F * epsilon);
            const float numerical_angle =
                (plus.angle_body - minus.angle_body) /
                (2.0F * epsilon);

            failures += expect_near(
                "length Jacobian",
                current.jacobian[BC_LEG_LENGTH][joint],
                numerical_length, 3.0e-5F);
            failures += expect_near(
                "angle Jacobian",
                current.jacobian[BC_LEG_ANGLE][joint],
                numerical_angle, 5.0e-5F);
        }

        const float expected_length_velocity =
            current.jacobian[BC_LEG_LENGTH][BC_FRONT] * 0.7F +
            current.jacobian[BC_LEG_LENGTH][BC_REAR] * -0.3F;
        const float expected_angular_velocity =
            current.jacobian[BC_LEG_ANGLE][BC_FRONT] * 0.7F +
            current.jacobian[BC_LEG_ANGLE][BC_REAR] * -0.3F;
        failures += expect_near(
            "length velocity", current.length_velocity,
            expected_length_velocity, 1.0e-6F);
        failures += expect_near(
            "angular velocity", current.angular_velocity,
            expected_angular_velocity, 1.0e-6F);
    }

    return failures == 0 ? 0 : 1;
}
