#include "balance/support_force.h"

#include <math.h>
#include <stdio.h>

static int near(const float actual, const float expected) {
    return fabsf(actual - expected) < 1.0e-4F;
}

int main(void) {
    bc_support_force_config_t config;
    bc_support_force_default_config(&config);
    if (config.filter_alpha != 0.2F ||
        config.ground_confirm_duration != 0.015F ||
        config.air_confirm_duration != 0.020F) {
        fputs("default support force timing is incorrect\n", stderr);
        return 1;
    }
    config.filter_alpha = 1.0F;
    config.ground_confirm_duration = 0.002F;
    config.air_confirm_duration = 0.002F;
    config.wheel_mass = 0.0F;

    bc_support_force_t estimator;
    bc_support_force_init(&estimator, &config);
    bc_leg_kinematics_t leg = {
        .length = 0.2F,
        .jacobian = {{0.2F, -0.2F}, {0.5F, 0.5F}},
    };
    bc_leg_feedback_t feedback = {0};

    const float expected_axial = 20.0F;
    const float expected_leg_torque = 3.0F;
    feedback.joint[BC_FRONT].torque =
        0.2F * expected_axial + 0.5F * expected_leg_torque;
    feedback.joint[BC_REAR].torque =
        -0.2F * expected_axial + 0.5F * expected_leg_torque;
    bc_support_force_update(
        &estimator, &leg, &feedback, 0.0F, 0.001F);
    if (!estimator.output.valid ||
        !near(estimator.output.axial_force, expected_axial) ||
        !near(estimator.output.leg_torque, expected_leg_torque) ||
        !near(estimator.output.vertical_force, expected_axial) ||
        estimator.output.state != BC_CONTACT_GROUND) {
        fputs("support force inverse mapping is incorrect\n", stderr);
        return 1;
    }

    feedback.joint[BC_FRONT].torque = 0.0F;
    feedback.joint[BC_REAR].torque = 0.0F;
    bc_support_force_update(
        &estimator, &leg, &feedback, 0.0F, 0.001F);
    bc_support_force_update(
        &estimator, &leg, &feedback, 0.0F, 0.001F);
    if (estimator.output.state != BC_CONTACT_AIR ||
        bc_contact_state_name(estimator.output.state)[0] != 'a') {
        fputs("support force did not enter air state\n", stderr);
        return 1;
    }

    feedback.joint[BC_FRONT].torque =
        0.2F * expected_axial + 0.5F * expected_leg_torque;
    feedback.joint[BC_REAR].torque =
        -0.2F * expected_axial + 0.5F * expected_leg_torque;
    bc_support_force_update(
        &estimator, &leg, &feedback, 0.0F, 0.001F);
    bc_support_force_update(
        &estimator, &leg, &feedback, 0.0F, 0.001F);
    if (estimator.output.state != BC_CONTACT_GROUND) {
        fputs("support force did not return to ground state\n", stderr);
        return 1;
    }

    const bc_support_force_output_t previous = estimator.output;
    leg.jacobian[BC_LEG_LENGTH][BC_FRONT] = 0.0F;
    leg.jacobian[BC_LEG_LENGTH][BC_REAR] = 0.0F;
    bc_support_force_update(
        &estimator, &leg, &feedback, 0.0F, 0.001F);
    if (estimator.output.valid ||
        estimator.output.vertical_force != previous.vertical_force ||
        estimator.output.state != previous.state) {
        fputs("invalid support force input changed the estimate\n", stderr);
        return 1;
    }
    return 0;
}
