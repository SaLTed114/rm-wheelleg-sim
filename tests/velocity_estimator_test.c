#include "balance/velocity_estimator.h"

#include <math.h>
#include <stdio.h>

static int expect_near(
    const char *name, const float actual,
    const float expected, const float tolerance
) {
    if (fabsf(actual - expected) <= tolerance) return 0;

    fprintf(
        stderr, "%s: expected %.7f, got %.7f\n",
        name, expected, actual);
    return 1;
}

static bc_velocity_estimator_config_t test_config(void) {
    return (bc_velocity_estimator_config_t){
        .gravity = 9.81F,
        .initial_velocity_variance = 1.0F,
        .initial_bias_variance = 1.0F,
        .acceleration_variance = 2.0F,
        .bias_walk_variance = 0.02F,
        .wheel_velocity_variance = 0.0004F,
        .recovery_velocity_variance = 0.0008F,
        .nis_gate = 9.0F,
        .wheel_rejection_duration = 0.003F,
        .wheel_recovery_duration = 0.002F,
    };
}

static int test_prediction(void) {
    const bc_velocity_estimator_config_t config = test_config();
    bc_velocity_estimator_t estimator;
    bc_imu_feedback_t imu = {0};
    bc_velocity_estimator_init(&estimator, &config);

    imu.roll = 0.3F;
    imu.pitch = -0.2F;
    imu.specific_force_x = -config.gravity * sinf(imu.pitch);
    imu.specific_force_y = config.gravity * sinf(imu.roll) * cosf(imu.pitch);
    for (int step = 0; step < 1000; ++step) {
        bc_velocity_estimator_predict(&estimator, &imu, 0.001F);
    }
    if (expect_near("stationary vx", estimator.state[0], 0.0F, 1.0e-5F) ||
        expect_near("stationary vy", estimator.state[1], 0.0F, 1.0e-5F)) {
        return 1;
    }

    bc_velocity_estimator_reset(&estimator);
    imu = (bc_imu_feedback_t){0};
    imu.specific_force_x = 2.0F;
    bc_velocity_estimator_predict(&estimator, &imu, 0.1F);
    if (expect_near("forward vx", estimator.state[0], 0.2F, 1.0e-6F) ||
        expect_near("forward vy", estimator.state[1], 0.0F, 1.0e-6F)) {
        return 1;
    }

    bc_velocity_estimator_reset(&estimator);
    estimator.state[2] = 0.5F;
    imu.specific_force_x = 0.5F;
    bc_velocity_estimator_predict(&estimator, &imu, 0.2F);
    if (expect_near("bias corrected vx", estimator.state[0], 0.0F, 1.0e-6F)) {
        return 1;
    }

    return 0;
}

static int test_yaw_prediction(void) {
    const bc_velocity_estimator_config_t config = test_config();
    bc_velocity_estimator_t estimator;
    bc_imu_feedback_t imu = {0};
    bc_velocity_estimator_init(&estimator, &config);

    const float radius = 0.1F;
    const float yaw_acceleration = 2.0F;
    const float timestep = 0.001F;
    for (int step = 0; step < 1000; ++step) {
        const float time = ((float)step + 0.5F) * timestep;
        const float yaw_rate = yaw_acceleration * time;
        imu.yaw_rate = yaw_rate;
        imu.specific_force_x = yaw_rate * yaw_rate * radius;
        imu.specific_force_y = -yaw_acceleration * radius;
        bc_velocity_estimator_predict(&estimator, &imu, timestep);
    }
    if (expect_near("yawing vx", estimator.state[0], 0.0F, 5.0e-4F) ||
        expect_near(
            "yawing vy", estimator.state[1],
            -yaw_acceleration * radius, 5.0e-4F)) {
        return 1;
    }

    return 0;
}

static int test_update(void) {
    bc_velocity_estimator_config_t config = test_config();
    config.initial_velocity_variance = 0.01F;
    bc_velocity_estimator_t estimator;
    bc_velocity_estimator_init(&estimator, &config);

    bc_velocity_estimator_update(&estimator, 0.02F, 0.001F);
    if (!estimator.output.measurement_accepted ||
        estimator.output.velocity_x <= 0.0F ||
        estimator.output.velocity_x >= 0.02F ||
        estimator.output.velocity_y != 0.0F ||
        estimator.output.nis >= config.nis_gate) {
        fputs("valid wheel update was not accepted\n", stderr);
        return 1;
    }

    estimator.state[1] = 0.4F;
    estimator.state[3] = -0.3F;
    estimator.covariance[0][1] = 0.0001F;
    estimator.covariance[1][0] = 0.0001F;
    estimator.covariance[0][3] = 0.00005F;
    estimator.covariance[3][0] = 0.00005F;
    bc_velocity_estimator_update(&estimator, 0.02F, 0.001F);
    if (!estimator.output.measurement_accepted ||
        expect_near("lateral velocity", estimator.state[1], 0.4F, 0.0F) ||
        expect_near("lateral bias", estimator.state[3], -0.3F, 0.0F)) {
        fputs("forward update modified a lateral state\n", stderr);
        return 1;
    }

    const float velocity_before_rejection = estimator.state[0];
    bc_velocity_estimator_update(&estimator, 0.2F, 0.001F);
    if (estimator.output.measurement_accepted ||
        estimator.output.nis <= config.nis_gate ||
        expect_near(
            "rejected velocity", estimator.state[0],
            velocity_before_rejection, 0.0F)) {
        fputs("wheel innovation was not rejected cleanly\n", stderr);
        return 1;
    }

    for (int repeat = 0; repeat < 10; ++repeat) {
        bc_velocity_estimator_update(&estimator, 0.2F, 0.001F);
    }
    if (expect_near(
            "repeated rejection", estimator.state[0],
            velocity_before_rejection, 0.0F)) {
        return 1;
    }

    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            if (expect_near(
                    "covariance symmetry",
                    estimator.covariance[row][column],
                    estimator.covariance[column][row], 1.0e-6F)) {
                return 1;
            }
        }
    }

    return 0;
}

static int test_bias_convergence(void) {
    bc_velocity_estimator_config_t config = test_config();
    config.nis_gate = 100.0F;
    bc_velocity_estimator_t estimator;
    bc_imu_feedback_t imu = {0};
    bc_velocity_estimator_init(&estimator, &config);

    imu.specific_force_x = 0.4F;
    for (int step = 0; step < 5000; ++step) {
        bc_velocity_estimator_update(&estimator, 0.0F, 0.001F);
        bc_velocity_estimator_predict(&estimator, &imu, 0.001F);
    }
    bc_velocity_estimator_update(&estimator, 0.0F, 0.001F);
    if (expect_near(
            "estimated bias", estimator.output.acceleration_bias_x,
            0.4F, 0.02F) ||
        expect_near(
            "bias-corrected velocity", estimator.output.velocity_x,
            0.0F, 0.01F)) {
        return 1;
    }

    return 0;
}

static int test_measurement_health(void) {
    const bc_velocity_estimator_config_t config = test_config();
    bc_velocity_estimator_t estimator;
    bc_velocity_estimator_init(&estimator, &config);

    bc_velocity_estimator_update(&estimator, 0.0F, 0.001F);
    if (!estimator.output.measurement_accepted ||
        !estimator.output.wheel_velocity_reliable) {
        fputs("initial wheel measurement was not accepted\n", stderr);
        return 1;
    }

    for (int sample = 0; sample < 2; ++sample) {
        bc_velocity_estimator_update(&estimator, 1.0F, 0.001F);
    }
    if (estimator.output.measurement_accepted ||
        !estimator.output.wheel_velocity_reliable ||
        estimator.output.rejection_elapsed_seconds <= 0.0F) {
        fputs("transient rejection changed wheel reliability\n", stderr);
        return 1;
    }

    estimator.state[2] = 0.25F;
    const float velocity_before_recovery = estimator.state[0];
    const float bias_before_recovery = estimator.state[2];
    float covariance_before_recovery[4][4];
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            covariance_before_recovery[row][column] =
                estimator.covariance[row][column];
        }
    }
    bc_velocity_estimator_update(&estimator, 1.0F, 0.001F);
    if (estimator.output.measurement_accepted ||
        estimator.output.wheel_velocity_reliable ||
        expect_near(
            "rejection preserved velocity", estimator.state[0],
            velocity_before_recovery, 0.0F) ||
        expect_near(
            "rejection preserved bias", estimator.state[2],
            bias_before_recovery, 0.0F) ||
        estimator.covariance[0][0] <
            config.recovery_velocity_variance ||
        estimator.output.velocity_variance_x <
            config.recovery_velocity_variance) {
        fputs("sustained rejection did not invalidate wheel velocity\n", stderr);
        return 1;
    }
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            if (row == 0 && column == 0) continue;
            if (expect_near(
                    "recovery preserved covariance",
                    estimator.covariance[row][column],
                    covariance_before_recovery[row][column], 0.0F)) {
                return 1;
            }
        }
    }

    bc_velocity_estimator_update(
        &estimator, estimator.state[0] + 0.07F, 0.001F);
    if (estimator.output.measurement_accepted ||
        estimator.output.wheel_velocity_reliable ||
        estimator.output.recovery_elapsed_seconds <= 0.0F) {
        fputs("wheel velocity recovered without a sufficient hold\n", stderr);
        return 1;
    }

    bc_velocity_estimator_update(
        &estimator, estimator.state[0] + 0.07F, 0.001F);
    if (!estimator.output.measurement_accepted ||
        !estimator.output.wheel_velocity_reliable) {
        fputs("wheel velocity did not recover after valid samples\n", stderr);
        return 1;
    }

    for (int sample = 0; sample < 3; ++sample) {
        bc_velocity_estimator_update(
            &estimator, estimator.state[0] + 0.17F, 0.001F);
    }
    if (estimator.output.wheel_velocity_reliable) {
        fputs("gross wheel outlier did not invalidate reliability\n", stderr);
        return 1;
    }
    for (int sample = 0; sample < 10; ++sample) {
        bc_velocity_estimator_update(
            &estimator, estimator.state[0] + 0.17F, 0.001F);
    }
    if (estimator.output.measurement_accepted ||
        estimator.output.wheel_velocity_reliable ||
        estimator.output.nis <= config.nis_gate) {
        fputs("gross wheel outlier bypassed recovery gate\n", stderr);
        return 1;
    }

    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            if (!isfinite(estimator.covariance[row][column]) ||
                expect_near(
                    "recovery covariance symmetry",
                    estimator.covariance[row][column],
                    estimator.covariance[column][row], 1.0e-6F)) {
                return 1;
            }
        }
    }

    bc_velocity_estimator_skip_update(&estimator);
    if (estimator.output.wheel_velocity_reliable) {
        fputs("disabled wheel update remained reliable\n", stderr);
        return 1;
    }

    return 0;
}

static int test_reset_and_invalid_timestep(void) {
    const bc_velocity_estimator_config_t config = test_config();
    bc_velocity_estimator_t estimator;
    bc_imu_feedback_t imu = {0};
    bc_velocity_estimator_init(&estimator, &config);

    estimator.state[0] = 1.0F;
    estimator.state[1] = -2.0F;
    bc_velocity_estimator_skip_update(&estimator);
    if (expect_near(
            "skipped update vx", estimator.output.velocity_x,
            1.0F, 0.0F) ||
        expect_near(
            "skipped update vy", estimator.output.velocity_y,
            -2.0F, 0.0F) ||
        estimator.output.measurement_accepted) {
        return 1;
    }

    bc_velocity_estimator_predict(&estimator, &imu, 0.0F);
    bc_velocity_estimator_predict(&estimator, &imu, NAN);
    if (expect_near("invalid dt vx", estimator.state[0], 1.0F, 0.0F) ||
        expect_near("invalid dt vy", estimator.state[1], -2.0F, 0.0F)) {
        return 1;
    }

    bc_velocity_estimator_reset(&estimator);
    for (int index = 0; index < 4; ++index) {
        if (estimator.state[index] != 0.0F) {
            fputs("reset did not clear estimator state\n", stderr);
            return 1;
        }
    }

    return 0;
}

int main() {
    if (test_prediction()) return 1;
    if (test_yaw_prediction()) return 1;
    if (test_update()) return 1;
    if (test_bias_convergence()) return 1;
    if (test_measurement_health()) return 1;
    if (test_reset_and_invalid_timestep()) return 1;
    return 0;
}
