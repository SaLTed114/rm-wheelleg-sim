#include "balance/velocity_estimator.h"

#include <math.h>
#include <string.h>

typedef enum {
    VELOCITY_X,
    VELOCITY_Y,
    BIAS_X,
    BIAS_Y,
    ESTIMATOR_STATE_NUM
} estimator_state_index_t;

static void symmetrize_covariance(float covariance[4][4]) {
    for (int row = 0; row < ESTIMATOR_STATE_NUM; ++row) {
        for (int column = row + 1; column < ESTIMATOR_STATE_NUM; ++column) {
            const float average = 0.5F * (
                covariance[row][column] + covariance[column][row]);
            covariance[row][column] = average;
            covariance[column][row] = average;
        }
    }
}

static void capture_estimate(bc_velocity_estimator_t *estimator) {
    estimator->output.velocity_x = estimator->state[VELOCITY_X];
    estimator->output.velocity_y = estimator->state[VELOCITY_Y];
    estimator->output.acceleration_bias_x = estimator->state[BIAS_X];
    estimator->output.acceleration_bias_y = estimator->state[BIAS_Y];
}

static void reset_covariance(bc_velocity_estimator_t *estimator) {
    memset(estimator->covariance, 0, sizeof(estimator->covariance));
    estimator->covariance[VELOCITY_X][VELOCITY_X] =
        estimator->config.initial_velocity_variance;
    estimator->covariance[VELOCITY_Y][VELOCITY_Y] =
        estimator->config.initial_velocity_variance;
    estimator->covariance[BIAS_X][BIAS_X] =
        estimator->config.initial_bias_variance;
    estimator->covariance[BIAS_Y][BIAS_Y] =
        estimator->config.initial_bias_variance;
}

void bc_velocity_estimator_init(
    bc_velocity_estimator_t *estimator,
    const bc_velocity_estimator_config_t *config
) {
    estimator->config = *config;
    bc_velocity_estimator_reset(estimator);
}

void bc_velocity_estimator_reset(bc_velocity_estimator_t *estimator) {
    memset(estimator->state, 0, sizeof(estimator->state));
    memset(&estimator->output, 0, sizeof(estimator->output));
    reset_covariance(estimator);
    estimator->measurement_initialized = 0U;
}

void bc_velocity_estimator_skip_update(
    bc_velocity_estimator_t *estimator
) {
    estimator->output.prior_velocity_x = estimator->state[VELOCITY_X];
    estimator->output.prior_velocity_y = estimator->state[VELOCITY_Y];
    estimator->output.wheel_velocity_measurement = 0.0F;
    estimator->output.innovation = 0.0F;
    estimator->output.innovation_variance = 0.0F;
    estimator->output.nis = 0.0F;
    estimator->output.measurement_accepted = 0U;
    estimator->measurement_initialized = 0U;
    capture_estimate(estimator);
}

void bc_velocity_estimator_update(
    bc_velocity_estimator_t *estimator,
    const float wheel_velocity_measurement
) {
    if (!estimator->measurement_initialized) {
        reset_covariance(estimator);
        estimator->measurement_initialized = 1U;
    }

    bc_velocity_estimator_output_t *output = &estimator->output;
    output->prior_velocity_x = estimator->state[VELOCITY_X];
    output->prior_velocity_y = estimator->state[VELOCITY_Y];
    output->wheel_velocity_measurement = wheel_velocity_measurement;
    output->measurement_accepted = 0U;

    const float measurement_variance =
        estimator->config.wheel_velocity_variance;
    const float innovation_variance =
        estimator->covariance[VELOCITY_X][VELOCITY_X] +
        measurement_variance;
    if (!isfinite(wheel_velocity_measurement) ||
        !isfinite(measurement_variance) || measurement_variance <= 0.0F ||
        !isfinite(innovation_variance) || innovation_variance <= 0.0F ||
        !isfinite(estimator->config.nis_gate) ||
        estimator->config.nis_gate < 0.0F) {
        output->innovation = 0.0F;
        output->innovation_variance = 0.0F;
        output->nis = 0.0F;
        capture_estimate(estimator);
        return;
    }

    const float innovation =
        wheel_velocity_measurement - estimator->state[VELOCITY_X];
    const float nis = innovation * innovation / innovation_variance;
    output->innovation = innovation;
    output->innovation_variance = innovation_variance;
    output->nis = nis;
    if (!isfinite(nis) || nis > estimator->config.nis_gate) {
        capture_estimate(estimator);
        return;
    }

    float gain[ESTIMATOR_STATE_NUM];
    for (int row = 0; row < ESTIMATOR_STATE_NUM; ++row) {
        gain[row] =
            estimator->covariance[row][VELOCITY_X] /
            innovation_variance;
    }
    // A forward wheel measurement does not constrain lateral slip reliably.
    gain[VELOCITY_Y] = 0.0F;
    gain[BIAS_Y] = 0.0F;
    for (int row = 0; row < ESTIMATOR_STATE_NUM; ++row) {
        estimator->state[row] += gain[row] * innovation;
    }

    float left[ESTIMATOR_STATE_NUM][ESTIMATOR_STATE_NUM] = {0};
    for (int row = 0; row < ESTIMATOR_STATE_NUM; ++row) {
        for (int column = 0; column < ESTIMATOR_STATE_NUM; ++column) {
            left[row][column] = row == column ? 1.0F : 0.0F;
        }
        left[row][VELOCITY_X] -= gain[row];
    }

    float product[ESTIMATOR_STATE_NUM][ESTIMATOR_STATE_NUM] = {0};
    float covariance[ESTIMATOR_STATE_NUM][ESTIMATOR_STATE_NUM] = {0};
    for (int row = 0; row < ESTIMATOR_STATE_NUM; ++row) {
        for (int column = 0; column < ESTIMATOR_STATE_NUM; ++column) {
            for (int inner = 0; inner < ESTIMATOR_STATE_NUM; ++inner) {
                product[row][column] +=
                    left[row][inner] * estimator->covariance[inner][column];
            }
        }
    }
    for (int row = 0; row < ESTIMATOR_STATE_NUM; ++row) {
        for (int column = 0; column < ESTIMATOR_STATE_NUM; ++column) {
            for (int inner = 0; inner < ESTIMATOR_STATE_NUM; ++inner) {
                covariance[row][column] +=
                    product[row][inner] * left[column][inner];
            }
            covariance[row][column] +=
                gain[row] * measurement_variance * gain[column];
        }
    }
    memcpy(estimator->covariance, covariance, sizeof(covariance));
    symmetrize_covariance(estimator->covariance);

    output->measurement_accepted = 1U;
    capture_estimate(estimator);
}

void bc_velocity_estimator_predict(
    bc_velocity_estimator_t *estimator,
    const bc_imu_feedback_t *imu,
    const float timestep_seconds
) {
    if (!isfinite(timestep_seconds) || timestep_seconds <= 0.0F) return;
    if (!isfinite(estimator->config.gravity) ||
        !isfinite(estimator->config.acceleration_variance) ||
        estimator->config.acceleration_variance < 0.0F ||
        !isfinite(estimator->config.bias_walk_variance) ||
        estimator->config.bias_walk_variance < 0.0F ||
        !isfinite(imu->roll) || !isfinite(imu->pitch) ||
        !isfinite(imu->yaw_rate) ||
        !isfinite(imu->specific_force_x) ||
        !isfinite(imu->specific_force_y)) return;

    const float sin_roll  = sinf(imu->roll);
    const float sin_pitch = sinf(imu->pitch);
    const float cos_pitch = cosf(imu->pitch);
    const float acceleration_x =
        imu->specific_force_x +
        estimator->config.gravity * sin_pitch -
        estimator->state[BIAS_X];
    const float acceleration_y =
        imu->specific_force_y -
        estimator->config.gravity * sin_roll * cos_pitch -
        estimator->state[BIAS_Y];

    estimator->output.linear_acceleration_x = acceleration_x;
    estimator->output.linear_acceleration_y = acceleration_y;

    const float yaw_rate = imu->yaw_rate;
    const float k1_x = acceleration_x + yaw_rate * estimator->state[VELOCITY_Y];
    const float k1_y = acceleration_y - yaw_rate * estimator->state[VELOCITY_X];
    const float middle_velocity_x =
        estimator->state[VELOCITY_X] +
        0.5F * timestep_seconds * k1_x;
    const float middle_velocity_y =
        estimator->state[VELOCITY_Y] +
        0.5F * timestep_seconds * k1_y;
    const float k2_x = acceleration_x + yaw_rate * middle_velocity_y;
    const float k2_y = acceleration_y - yaw_rate * middle_velocity_x;

    estimator->state[VELOCITY_X] += timestep_seconds * k2_x;
    estimator->state[VELOCITY_Y] += timestep_seconds * k2_y;

    float dynamics[ESTIMATOR_STATE_NUM][ESTIMATOR_STATE_NUM] = {0};
    dynamics[VELOCITY_X][VELOCITY_Y] = yaw_rate;
    dynamics[VELOCITY_X][BIAS_X] = -1.0F;
    dynamics[VELOCITY_Y][VELOCITY_X] = -yaw_rate;
    dynamics[VELOCITY_Y][BIAS_Y] = -1.0F;

    float transition[ESTIMATOR_STATE_NUM][ESTIMATOR_STATE_NUM] = {0};
    for (int row = 0; row < ESTIMATOR_STATE_NUM; ++row) {
        for (int column = 0; column < ESTIMATOR_STATE_NUM; ++column) {
            transition[row][column] = row == column ? 1.0F : 0.0F;
            transition[row][column] +=
                timestep_seconds * dynamics[row][column];
            for (int inner = 0; inner < ESTIMATOR_STATE_NUM; ++inner) {
                transition[row][column] +=
                    0.5F * timestep_seconds * timestep_seconds *
                    dynamics[row][inner] * dynamics[inner][column];
            }
        }
    }

    float product[ESTIMATOR_STATE_NUM][ESTIMATOR_STATE_NUM] = {0};
    float covariance[ESTIMATOR_STATE_NUM][ESTIMATOR_STATE_NUM] = {0};
    for (int row = 0; row < ESTIMATOR_STATE_NUM; ++row) {
        for (int column = 0; column < ESTIMATOR_STATE_NUM; ++column) {
            for (int inner = 0; inner < ESTIMATOR_STATE_NUM; ++inner) {
                product[row][column] +=
                    transition[row][inner] *
                    estimator->covariance[inner][column];
            }
        }
    }
    for (int row = 0; row < ESTIMATOR_STATE_NUM; ++row) {
        for (int column = 0; column < ESTIMATOR_STATE_NUM; ++column) {
            for (int inner = 0; inner < ESTIMATOR_STATE_NUM; ++inner) {
                covariance[row][column] +=
                    product[row][inner] * transition[column][inner];
            }
        }
    }

    const float timestep_squared = timestep_seconds * timestep_seconds;
    covariance[VELOCITY_X][VELOCITY_X] +=
        estimator->config.acceleration_variance * timestep_squared;
    covariance[VELOCITY_Y][VELOCITY_Y] +=
        estimator->config.acceleration_variance * timestep_squared;
    covariance[BIAS_X][BIAS_X] +=
        estimator->config.bias_walk_variance * timestep_seconds;
    covariance[BIAS_Y][BIAS_Y] +=
        estimator->config.bias_walk_variance * timestep_seconds;
    memcpy(estimator->covariance, covariance, sizeof(covariance));
    symmetrize_covariance(estimator->covariance);
}
