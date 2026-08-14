#include "balance/impact_observer.h"
#include "balance/math_utils.h"

#include <math.h>
#include <stdio.h>

static int near(const float actual, const float expected) {
    return fabsf(actual - expected) <= 1.0e-5F;
}

static bc_impact_observer_config_t config(void) {
    bc_impact_observer_config_t result;
    bc_impact_observer_default_config(&result);
    return result;
}

static int test_gravity_transform(void) {
    const bc_impact_observer_config_t observer_config = config();
    bc_impact_observer_t observer;
    bc_leg_kinematics_t leg[BC_SIDE_NUM] = {0};
    bc_imu_feedback_t imu = {
        .specific_force_z = 9.81F,
    };
    bc_impact_observer_init(&observer, &observer_config);
    bc_impact_observer_update(&observer, &imu, leg, 0.0F, 0.001F);
    if (!observer.output.valid ||
        !near(observer.output.forward_acceleration, 0.0F) ||
        !near(observer.output.vertical_acceleration, 0.0F) ||
        observer.output.window[BC_IMPACT_WINDOW_SHORT].valid) {
        fputs("level gravity transform is incorrect\n", stderr);
        return 1;
    }

    bc_impact_observer_reset(&observer);
    imu.pitch = 0.3F;
    imu.roll = -0.2F;
    imu.specific_force_x = -9.81F * sinf(imu.pitch);
    imu.specific_force_y =
        9.81F * sinf(imu.roll) * cosf(imu.pitch);
    imu.specific_force_z =
        9.81F * cosf(imu.roll) * cosf(imu.pitch);
    bc_impact_observer_update(&observer, &imu, leg, 0.0F, 0.001F);
    if (!near(observer.output.forward_acceleration, 0.0F) ||
        !near(observer.output.vertical_acceleration, 0.0F)) {
        fputs("tilted gravity transform is incorrect\n", stderr);
        return 1;
    }
    return 0;
}

static int test_rolling_windows(void) {
    const bc_impact_observer_config_t observer_config = config();
    bc_impact_observer_t observer;
    bc_leg_kinematics_t leg[BC_SIDE_NUM] = {0};
    bc_imu_feedback_t imu = {
        .specific_force_x = 2.0F,
        .specific_force_z = 9.81F,
    };
    bc_impact_observer_init(&observer, &observer_config);
    bc_impact_observer_update(&observer, &imu, leg, 0.0F, 0.001F);

    for (int sample = 1; sample <= 10; ++sample) {
        imu.pitch_rate = 0.1F * (float)sample;
        leg[BC_L].angular_velocity = 0.1F * (float)sample;
        leg[BC_R].angular_velocity = -0.2F * (float)sample;
        bc_impact_observer_update(
            &observer, &imu, leg, 0.3F * (float)sample, 0.001F);
        if (sample == 5 &&
            (!observer.output.window[BC_IMPACT_WINDOW_SHORT].valid ||
             observer.output.window[BC_IMPACT_WINDOW_LONG].valid)) {
            fputs("impact window validity timing is incorrect\n", stderr);
            return 1;
        }
    }

    const bc_impact_window_output_t *short_window =
        &observer.output.window[BC_IMPACT_WINDOW_SHORT];
    const bc_impact_window_output_t *long_window =
        &observer.output.window[BC_IMPACT_WINDOW_LONG];
    if (!short_window->valid || !long_window->valid ||
        !near(short_window->forward_delta_velocity, 0.010F) ||
        !near(long_window->forward_delta_velocity, 0.020F) ||
        !near(long_window->vertical_delta_velocity, 0.0F) ||
        !near(short_window->pitch_rate_delta, 0.5F) ||
        !near(long_window->pitch_rate_delta, 1.0F) ||
        !near(short_window->leg_rate_delta[BC_L], 1.0F) ||
        !near(short_window->leg_rate_delta[BC_R], -0.5F) ||
        !near(long_window->leg_rate_delta[BC_L], 2.0F) ||
        !near(long_window->leg_rate_delta[BC_R], -1.0F) ||
        !near(short_window->wheel_velocity_delta, 1.5F) ||
        !near(long_window->wheel_velocity_delta, 3.0F) ||
        !near(long_window->wheel_imu_delta_mismatch, 2.98F)) {
        fputs("impact rolling window output is incorrect\n", stderr);
        return 1;
    }
    return 0;
}

static int test_invalid_input_resets_history(void) {
    const bc_impact_observer_config_t observer_config = config();
    bc_impact_observer_t observer;
    bc_leg_kinematics_t leg[BC_SIDE_NUM] = {0};
    bc_imu_feedback_t imu = {
        .specific_force_z = 9.81F,
    };
    bc_impact_observer_init(&observer, &observer_config);
    bc_impact_observer_update(&observer, &imu, leg, 0.0F, 0.001F);
    imu.specific_force_x = NAN;
    bc_impact_observer_update(&observer, &imu, leg, 0.0F, 0.001F);
    if (observer.output.valid || observer.history_count != 0U ||
        observer.initialized) {
        fputs("invalid impact input did not reset history\n", stderr);
        return 1;
    }
    return 0;
}

static int test_partial_interval_interpolation(void) {
    const bc_impact_observer_config_t observer_config = config();
    bc_impact_observer_t observer;
    bc_leg_kinematics_t leg[BC_SIDE_NUM] = {0};
    bc_imu_feedback_t imu = {
        .specific_force_z = 9.81F,
    };
    bc_impact_observer_init(&observer, &observer_config);
    bc_impact_observer_update(&observer, &imu, leg, 0.0F, 0.003F);
    imu.specific_force_x = 2.0F;
    bc_impact_observer_update(&observer, &imu, leg, 3.0F, 0.003F);
    imu.specific_force_x = 4.0F;
    bc_impact_observer_update(&observer, &imu, leg, 6.0F, 0.003F);

    const bc_impact_window_output_t *window =
        &observer.output.window[BC_IMPACT_WINDOW_SHORT];
    if (!window->valid ||
        !near(window->forward_delta_velocity, 0.011666667F) ||
        !near(window->wheel_velocity_delta, 5.0F)) {
        fputs("partial impact interval was not interpolated\n", stderr);
        return 1;
    }
    return 0;
}

int main(void) {
    return test_gravity_transform() ||
        test_rolling_windows() ||
        test_partial_interval_interpolation() ||
        test_invalid_input_resets_history();
}
