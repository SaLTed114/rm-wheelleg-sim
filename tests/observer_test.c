#include "balance/observer.h"
#include "balance/math_utils.h"

#include <math.h>
#include <stdio.h>

static int expect_near(const char *name, float actual, float expected) {
    if (fabsf(actual - expected) <= 1.0e-5F) return 0;

    fprintf(stderr, "%s: expected %.7f, got %.7f\n", name, expected, actual);
    return 1;
}

static bc_observer_config_t observer_config(void) {
    return (bc_observer_config_t){
        .leg_geometry = {
            .hip_link_length   = 0.215F,
            .wheel_link_length = 0.254F,
        },
        .velocity_estimator = {
            .gravity                   = 9.81F,
            .initial_velocity_variance = 0.0004F,
            .initial_bias_variance     = 0.000001F,
            .acceleration_variance     = 0.02F,
            .bias_walk_variance        = 0.00000001F,
            .wheel_velocity_variance   = 0.0004F,
            .recovery_velocity_variance = 0.0008F,
            .nis_gate                  = 9.0F,
            .wheel_rejection_duration  = 0.02F,
            .wheel_recovery_duration   = 0.02F,
        },
        .wheel_radius = 0.06F,
    };
}

static int test_wheel_measurement_transform(void) {
    bc_observer_config_t config = observer_config();
    bc_observer_t observer;
    bc_sensor_feedback_t feedback = {0};

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        feedback.leg[side].joint[BC_FRONT].angle = -0.5F * BC_PI_F;
        feedback.leg[side].joint[BC_REAR].angle = -0.5F * BC_PI_F;
        feedback.wheel[side].angular_velocity = 2.0F;
    }

    feedback.imu.pitch_rate = 1.0F;
    bc_observer_init(&observer, &config);
    bc_observer_update(&observer, &feedback, 0.001F, 1U);
    if (expect_near(
            "pitch-rate wheel measurement",
            observer.velocity_estimator.output.wheel_velocity_measurement,
            0.589F)) {
        return 1;
    }

    feedback.imu.pitch_rate = 0.0F;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        feedback.leg[side].joint[BC_FRONT].angular_velocity = 0.4F;
        feedback.leg[side].joint[BC_REAR].angular_velocity = 0.4F;
    }
    bc_observer_reset(&observer);
    bc_observer_update(&observer, &feedback, 0.001F, 1U);
    if (expect_near(
            "leg-motion wheel measurement",
            observer.velocity_estimator.output.wheel_velocity_measurement,
            0.3076F)) {
        return 1;
    }

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        feedback.leg[side].joint[BC_FRONT].angular_velocity = 0.0F;
        feedback.leg[side].joint[BC_REAR].angular_velocity = 0.0F;
    }
    feedback.imu.yaw_rate = 0.5F;
    config.hip_center_position.y = 0.2F;
    bc_observer_init(&observer, &config);
    bc_observer_update(&observer, &feedback, 0.001F, 1U);
    if (expect_near(
            "yaw-rate wheel measurement",
            observer.velocity_estimator.output.wheel_velocity_measurement,
            0.22F)) {
        return 1;
    }

    return 0;
}

static int test_estimated_axle_state(void) {
    const bc_observer_config_t config = observer_config();
    bc_observer_t observer;
    bc_sensor_feedback_t feedback = {0};

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        feedback.leg[side].joint[BC_FRONT].angle = -0.5F * BC_PI_F;
        feedback.leg[side].joint[BC_REAR].angle = -0.5F * BC_PI_F;
    }
    bc_observer_init(&observer, &config);
    bc_observer_update(&observer, &feedback, 0.01F, 1U);

    observer.velocity_estimator.state[0] = 0.3F;
    bc_observer_update(&observer, &feedback, 0.01F, 1U);
    if (expect_near(
            "wheel odometry velocity",
            observer.forward_velocity.wheel_odometry, 0.0F) ||
        expect_near(
            "estimated axle velocity",
            observer.forward_velocity.estimated_axle, 0.3F) ||
        expect_near("estimator ds", observer.state.value[BC_STATE_DS], 0.3F) ||
        expect_near("estimator s", observer.state.value[BC_STATE_S], 0.003F)) {
        return 1;
    }

    return 0;
}

int main() {
    const bc_observer_config_t config = observer_config();
    bc_observer_t observer;
    bc_sensor_feedback_t feedback = {0};

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        feedback.leg[side].joint[BC_FRONT].angle = -0.5F * BC_PI_F;
        feedback.leg[side].joint[BC_REAR].angle = -0.5F * BC_PI_F;
    }
    feedback.wheel[BC_L].angle = 10.0F;
    feedback.wheel[BC_R].angle = -4.0F;
    feedback.imu.yaw = 2.8F;

    bc_observer_init(&observer, &config);
    bc_observer_update(&observer, &feedback, 0.01F, 1U);
    if (expect_near("initial s", observer.state.value[BC_STATE_S], 0.0F) ||
        expect_near("initial psi", observer.state.value[BC_STATE_PSI], 0.0F) ||
        expect_near("initial left leg", observer.state.value[BC_STATE_THETA_L], 0.0F) ||
        expect_near("initial right leg", observer.state.value[BC_STATE_THETA_R], 0.0F)) {
        return 1;
    }

    feedback.wheel[BC_L].angle += 2.0F;
    feedback.wheel[BC_R].angle += 4.0F;
    feedback.wheel[BC_L].angular_velocity = 3.0F;
    feedback.wheel[BC_R].angular_velocity = 5.0F;
    feedback.imu.yaw = -3.0F;
    feedback.imu.yaw_rate = 0.7F;
    feedback.imu.roll = -0.4F;
    feedback.imu.roll_rate = 0.6F;
    feedback.imu.pitch = 0.2F;
    feedback.imu.pitch_rate = -0.3F;

    feedback.leg[BC_L].joint[BC_FRONT].angle += 0.1F;
    feedback.leg[BC_L].joint[BC_REAR].angle += 0.1F;
    feedback.leg[BC_L].joint[BC_FRONT].angular_velocity = 0.4F;
    feedback.leg[BC_L].joint[BC_REAR].angular_velocity = 0.4F;
    feedback.leg[BC_R].joint[BC_FRONT].angle -= 0.25F;
    feedback.leg[BC_R].joint[BC_REAR].angle -= 0.25F;
    feedback.leg[BC_R].joint[BC_FRONT].angular_velocity = -0.6F;
    feedback.leg[BC_R].joint[BC_REAR].angular_velocity = -0.6F;

    bc_observer_update(&observer, &feedback, 0.01F, 1U);
    if (expect_near(
            "wheel odometry",
            observer.forward_velocity.wheel_odometry, 0.24F) ||
        expect_near(
            "s", observer.state.value[BC_STATE_S],
            0.01F * observer.forward_velocity.estimated_axle) ||
        expect_near(
            "ds", observer.state.value[BC_STATE_DS],
            observer.forward_velocity.estimated_axle) ||
        expect_near("psi", observer.state.value[BC_STATE_PSI], 0.4831853F) ||
        expect_near("dpsi", observer.state.value[BC_STATE_DPSI], 0.7F) ||
        expect_near("roll", observer.roll, -0.4F) ||
        expect_near("roll rate", observer.roll_rate, 0.6F) ||
        expect_near("theta left", observer.state.value[BC_STATE_THETA_L], 0.3F) ||
        expect_near("dtheta left", observer.state.value[BC_STATE_DTHETA_L], 0.1F) ||
        expect_near("theta right", observer.state.value[BC_STATE_THETA_R], -0.05F) ||
        expect_near("dtheta right", observer.state.value[BC_STATE_DTHETA_R], -0.9F) ||
        expect_near("theta body", observer.state.value[BC_STATE_THETA_B], 0.2F) ||
        expect_near("dtheta body", observer.state.value[BC_STATE_DTHETA_B], -0.3F)) {
        return 1;
    }

    feedback.wheel[BC_L].angle += 1.0F;
    feedback.wheel[BC_R].angle -= 1.0F;
    feedback.wheel[BC_L].angular_velocity = 2.0F;
    feedback.wheel[BC_R].angular_velocity = -2.0F;
    const float previous_position = observer.state.value[BC_STATE_S];
    bc_observer_update(&observer, &feedback, 0.01F, 1U);
    if (expect_near(
            "opposite wheel odometry",
            observer.forward_velocity.wheel_odometry, 0.0F) ||
        expect_near(
            "opposite wheel s", observer.state.value[BC_STATE_S],
            previous_position +
                0.01F * observer.forward_velocity.estimated_axle) ||
        expect_near(
            "opposite wheel ds", observer.state.value[BC_STATE_DS],
            observer.forward_velocity.estimated_axle)) {
        return 1;
    }

    bc_observer_reset(&observer);
    if (expect_near("reset roll", observer.roll, 0.0F) ||
        expect_near("reset roll rate", observer.roll_rate, 0.0F) ||
        expect_near("reset s", observer.state.value[BC_STATE_S], 0.0F)) {
        return 1;
    }
    bc_observer_update(&observer, &feedback, 0.01F, 1U);
    if (expect_near(
            "first integrated s", observer.state.value[BC_STATE_S],
            0.01F * observer.forward_velocity.estimated_axle) ||
        expect_near("reset psi", observer.state.value[BC_STATE_PSI], 0.0F)) {
        return 1;
    }

    if (test_wheel_measurement_transform()) return 1;
    return test_estimated_axle_state();
}
