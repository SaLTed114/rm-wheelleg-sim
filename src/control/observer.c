#include "balance/observer.h"

#include "balance/leg_kinematics.h"
#include "balance/math_utils.h"

#include <string.h>

void bc_observer_init(
    bc_observer_t *observer,
    const bc_observer_config_t *config
) {
    observer->config = *config;
    bc_observer_reset(observer);
}

void bc_observer_reset(bc_observer_t *observer) {
    memset(observer->leg, 0, sizeof(observer->leg));
    memset(&observer->state, 0, sizeof(observer->state));
    observer->previous_yaw = 0.0F;
    observer->yaw = 0.0F;
    observer->initialized = 0U;
}

void bc_observer_update(
    bc_observer_t *observer,
    const bc_sensor_feedback_t *feedback,
    const float timestep_seconds
) {
    if (!observer->initialized) {
        observer->previous_yaw = feedback->imu.yaw;
        observer->initialized = 1U;
    }

    observer->yaw += bc_wrap_anglef(
        feedback->imu.yaw - observer->previous_yaw);
    observer->previous_yaw = feedback->imu.yaw;

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        bc_leg_kinematics_calculate(
            &observer->config.leg_geometry, &feedback->leg[side],
            &observer->leg[side]);
    }

    const float wheel_velocity = 0.5F * (
        feedback->wheel[BC_L].angular_velocity +
        feedback->wheel[BC_R].angular_velocity);

    float *state = observer->state.value;
    state[BC_STATE_DS]       = observer->config.wheel_radius * wheel_velocity;
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
