#include "balance/observer.h"

#include "balance/leg_kinematics.h"
#include "balance/math_utils.h"

#include <math.h>
#include <string.h>

static float axle_velocity_offset_x(
    const bc_observer_t *observer,
    const bc_sensor_feedback_t *feedback
) {
    float axle_z = observer->config.hip_center_position.z;
    float relative_velocity_x = 0.0F;

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const bc_leg_kinematics_t *leg = &observer->leg[side];
        const float sin_angle = sinf(leg->angle_body);
        const float cos_angle = cosf(leg->angle_body);
        axle_z += 0.5F * leg->length * sin_angle;
        relative_velocity_x += 0.5F * (
            -leg->length_velocity * cos_angle +
            leg->length * leg->angular_velocity * sin_angle);
    }

    const float imu_to_axle_y =
        observer->config.hip_center_position.y -
        observer->config.imu_position.y;
    const float imu_to_axle_z =
        axle_z - observer->config.imu_position.z;
    const float point_velocity_x =
        feedback->imu.pitch_rate * imu_to_axle_z -
        feedback->imu.yaw_rate * imu_to_axle_y +
        relative_velocity_x;
    return point_velocity_x;
}

void bc_observer_init(
    bc_observer_t *observer,
    const bc_observer_config_t *config
) {
    observer->config = *config;
    bc_velocity_estimator_init(
        &observer->velocity_estimator,
        &config->velocity_estimator);
    bc_observer_reset(observer);
}

void bc_observer_reset(bc_observer_t *observer) {
    memset(observer->leg, 0, sizeof(observer->leg));
    memset(
        &observer->forward_velocity, 0,
        sizeof(observer->forward_velocity));
    memset(&observer->state, 0, sizeof(observer->state));
    bc_velocity_estimator_reset(&observer->velocity_estimator);
    observer->roll = 0.0F;
    observer->roll_rate = 0.0F;
    observer->previous_yaw = 0.0F;
    observer->yaw = 0.0F;
    observer->initialized = 0U;
}

void bc_observer_update(
    bc_observer_t *observer,
    const bc_sensor_feedback_t *feedback,
    const float timestep_seconds,
    const uint8_t wheel_velocity_update_enabled
) {
    if (!observer->initialized) {
        observer->previous_yaw = feedback->imu.yaw;
        observer->initialized = 1U;
    }

    observer->yaw += bc_wrap_anglef(
        feedback->imu.yaw - observer->previous_yaw);
    observer->previous_yaw = feedback->imu.yaw;
    observer->roll = bc_wrap_anglef(feedback->imu.roll);
    observer->roll_rate = feedback->imu.roll_rate;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        bc_leg_kinematics_calculate(
            &observer->config.leg_geometry, &feedback->leg[side],
            &observer->leg[side]);
    }

    const float common_wheel_rate = 0.5F * (
        feedback->wheel[BC_L].angular_velocity +
        feedback->wheel[BC_R].angular_velocity);
    const float velocity_offset = axle_velocity_offset_x(observer, feedback);
    observer->forward_velocity.wheel_odometry =
        observer->config.wheel_radius * common_wheel_rate;
    if (wheel_velocity_update_enabled) {
        bc_velocity_estimator_update(
            &observer->velocity_estimator,
            observer->forward_velocity.wheel_odometry - velocity_offset,
            timestep_seconds);
    } else {
        bc_velocity_estimator_skip_update(
            &observer->velocity_estimator);
    }
    bc_velocity_estimator_predict(
        &observer->velocity_estimator,
        &feedback->imu, timestep_seconds);
    observer->forward_velocity.estimated_axle =
        observer->velocity_estimator.output.velocity_x + velocity_offset;

    float *state = observer->state.value;
    state[BC_STATE_DS]       = observer->forward_velocity.estimated_axle;
    if (timestep_seconds > 0.0F) {
        state[BC_STATE_S] += state[BC_STATE_DS] * timestep_seconds;
    }
    state[BC_STATE_PSI]      = observer->yaw;
    state[BC_STATE_DPSI]     = feedback->imu.yaw_rate;
    state[BC_STATE_THETA_B]  = bc_wrap_anglef(feedback->imu.pitch);
    state[BC_STATE_DTHETA_B] = feedback->imu.pitch_rate;

    const int angle_index[BC_SIDE_NUM] = {
        BC_STATE_THETA_L, BC_STATE_THETA_R,
    };
    const int velocity_index[BC_SIDE_NUM] = {
        BC_STATE_DTHETA_L, BC_STATE_DTHETA_R,
    };
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        state[angle_index[side]] = bc_wrap_anglef(
            observer->leg[side].angle_body + 0.5F * BC_PI_F +
            feedback->imu.pitch);
        state[velocity_index[side]] =
            observer->leg[side].angular_velocity +
            feedback->imu.pitch_rate;
    }
}
