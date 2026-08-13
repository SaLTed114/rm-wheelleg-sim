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
    estimator->output.velocity_variance_x =
        estimator->covariance[VELOCITY_X][VELOCITY_X];
    estimator->output.rejection_elapsed_seconds =
        estimator->rejection_elapsed_seconds;
    estimator->output.recovery_elapsed_seconds =
        estimator->recovery_elapsed_seconds;
    estimator->output.reacquisition_elapsed_seconds =
        estimator->reacquisition_elapsed_seconds;
    estimator->output.wheel_velocity_reliable =
        estimator->wheel_velocity_reliable;
    estimator->output.reacquisition_active =
        estimator->reacquisition_elapsed_seconds > 0.0F &&
        estimator->reacquisition_elapsed_seconds >=
            estimator->config.reacquisition_stable_duration;
}

static void reset_reacquisition(bc_velocity_estimator_t *estimator) {
    estimator->reacquisition_elapsed_seconds = 0.0F;
    estimator->previous_wheel_measurement_initialized = 0U;
}

static void apply_reacquisition(
    bc_velocity_estimator_t *estimator,
    const float wheel_velocity_measurement,
    const uint8_t enabled,
    const float timestep_seconds
) {
    if (!enabled || estimator->wheel_velocity_reliable ||
        !isfinite(wheel_velocity_measurement) ||
        fabsf(wheel_velocity_measurement) >
            estimator->config.reacquisition_max_wheel_speed ||
        !isfinite(timestep_seconds) || timestep_seconds <= 0.0F) {
        reset_reacquisition(estimator);
        return;
    }

    uint8_t stable = 0U;
    if (estimator->previous_wheel_measurement_initialized) {
        const float acceleration = fabsf(
            wheel_velocity_measurement -
            estimator->previous_wheel_velocity_measurement) /
            timestep_seconds;
        stable = isfinite(acceleration) &&
            acceleration <=
                estimator->config.reacquisition_max_wheel_acceleration;
    }
    estimator->previous_wheel_velocity_measurement =
        wheel_velocity_measurement;
    estimator->previous_wheel_measurement_initialized = 1U;
    if (!stable) {
        estimator->reacquisition_elapsed_seconds = 0.0F;
        return;
    }

    estimator->reacquisition_elapsed_seconds += timestep_seconds;
    if (estimator->reacquisition_elapsed_seconds <
        estimator->config.reacquisition_stable_duration) return;

    const float maximum_step =
        estimator->config.reacquisition_velocity_rate * timestep_seconds;
    if (!isfinite(maximum_step) || maximum_step <= 0.0F) return;
    const float error =
        wheel_velocity_measurement - estimator->state[VELOCITY_X];
    estimator->state[VELOCITY_X] += fmaxf(
        -maximum_step, fminf(error, maximum_step));
}

static void inflate_recovery_covariance(
    bc_velocity_estimator_t *estimator
) {
    const float variance =
        estimator->config.recovery_velocity_variance;
    if (!isfinite(variance) || variance < 0.0F) return;
    if (estimator->covariance[VELOCITY_X][VELOCITY_X] < variance) {
        estimator->covariance[VELOCITY_X][VELOCITY_X] = variance;
    }
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

static uint8_t calculate_innovation(
    bc_velocity_estimator_t *estimator,
    const float wheel_velocity_measurement
) {
    bc_velocity_estimator_output_t *output = &estimator->output;
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
        return 0U;
    }

    const float innovation =
        wheel_velocity_measurement - estimator->state[VELOCITY_X];
    const float nis = innovation * innovation / innovation_variance;
    output->innovation = innovation;
    output->innovation_variance = innovation_variance;
    output->nis = nis;

    return isfinite(nis) && nis <= estimator->config.nis_gate;
}

static uint8_t update_wheel_reliability(
    bc_velocity_estimator_t *estimator,
    const uint8_t innovation_is_inlier,
    const float timestep_seconds
) {
    if (!isfinite(timestep_seconds) || timestep_seconds <= 0.0F) return 0U;

    if (innovation_is_inlier) {
        estimator->rejection_elapsed_seconds = 0.0F;
        if (estimator->wheel_velocity_reliable) {
            estimator->recovery_elapsed_seconds = 0.0F;
            return 1U;
        }

        const float required = estimator->config.wheel_recovery_duration;
        if (!isfinite(required) || required < 0.0F) return 0U;
        estimator->recovery_elapsed_seconds += timestep_seconds;
        if (estimator->recovery_elapsed_seconds < required) return 0U;

        estimator->recovery_elapsed_seconds = 0.0F;
        estimator->wheel_velocity_reliable = 1U;
        return 1U;
    }

    estimator->recovery_elapsed_seconds = 0.0F;
    if (!estimator->wheel_velocity_reliable) return 0U;

    const float required = estimator->config.wheel_rejection_duration;
    if (!isfinite(required) || required < 0.0F) {
        estimator->wheel_velocity_reliable = 0U;
        return 0U;
    }
    estimator->rejection_elapsed_seconds += timestep_seconds;
    if (estimator->rejection_elapsed_seconds >= required) {
        estimator->rejection_elapsed_seconds = 0.0F;
        estimator->wheel_velocity_reliable = 0U;
    }
    return 0U;
}

static void apply_measurement_update(
    bc_velocity_estimator_t *estimator
) {
    const float innovation = estimator->output.innovation;
    const float innovation_variance =
        estimator->output.innovation_variance;
    const float measurement_variance =
        estimator->config.wheel_velocity_variance;

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
    estimator->rejection_elapsed_seconds = 0.0F;
    estimator->recovery_elapsed_seconds = 0.0F;
    reset_reacquisition(estimator);
    estimator->measurement_initialized = 0U;
    estimator->wheel_velocity_reliable = 0U;
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
    estimator->rejection_elapsed_seconds = 0.0F;
    estimator->recovery_elapsed_seconds = 0.0F;
    reset_reacquisition(estimator);
    estimator->measurement_initialized = 0U;
    estimator->wheel_velocity_reliable = 0U;
    capture_estimate(estimator);
}

void bc_velocity_estimator_reject_wheel(
    bc_velocity_estimator_t *estimator
) {
    estimator->output.prior_velocity_x = estimator->state[VELOCITY_X];
    estimator->output.prior_velocity_y = estimator->state[VELOCITY_Y];
    estimator->output.wheel_velocity_measurement = 0.0F;
    estimator->output.innovation = 0.0F;
    estimator->output.innovation_variance = 0.0F;
    estimator->output.nis = 0.0F;
    estimator->output.measurement_accepted = 0U;
    estimator->rejection_elapsed_seconds = 0.0F;
    estimator->recovery_elapsed_seconds = 0.0F;
    reset_reacquisition(estimator);
    estimator->measurement_initialized = 1U;
    if (estimator->wheel_velocity_reliable) {
        inflate_recovery_covariance(estimator);
    }
    estimator->wheel_velocity_reliable = 0U;
    capture_estimate(estimator);
}

void bc_velocity_estimator_update(
    bc_velocity_estimator_t *estimator,
    const float wheel_velocity_measurement,
    const bc_wheel_update_mode_t mode,
    const float timestep_seconds
) {
    if (!estimator->measurement_initialized) {
        reset_covariance(estimator);
        estimator->rejection_elapsed_seconds = 0.0F;
        estimator->recovery_elapsed_seconds = 0.0F;
        estimator->measurement_initialized = 1U;
        estimator->wheel_velocity_reliable = 1U;
    }

    bc_velocity_estimator_output_t *output = &estimator->output;
    output->prior_velocity_x = estimator->state[VELOCITY_X];
    output->prior_velocity_y = estimator->state[VELOCITY_Y];
    output->wheel_velocity_measurement = wheel_velocity_measurement;
    output->measurement_accepted = 0U;
    const uint8_t innovation_is_inlier = calculate_innovation(
        estimator, wheel_velocity_measurement);
    const uint8_t was_reliable = estimator->wheel_velocity_reliable;
    const uint8_t measurement_is_usable = update_wheel_reliability(
        estimator, innovation_is_inlier, timestep_seconds);
    if (was_reliable && !estimator->wheel_velocity_reliable) {
        inflate_recovery_covariance(estimator);
    }
    if (!measurement_is_usable) {
        apply_reacquisition(
            estimator, wheel_velocity_measurement,
            mode == BC_WHEEL_UPDATE_REACQUIRE, timestep_seconds);
        calculate_innovation(estimator, wheel_velocity_measurement);
        capture_estimate(estimator);
        return;
    }

    reset_reacquisition(estimator);
    apply_measurement_update(estimator);
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
