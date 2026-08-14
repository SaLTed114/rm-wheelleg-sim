#include "balance/impact_observer.h"

#include <math.h>
#include <string.h>

static uint8_t finite_input(
    const bc_imu_feedback_t *imu,
    const bc_leg_kinematics_t leg[BC_SIDE_NUM],
    const float wheel_velocity,
    const float timestep_seconds
) {
    if (!isfinite(timestep_seconds) || timestep_seconds <= 0.0F ||
        !isfinite(wheel_velocity) || !isfinite(imu->roll) ||
        !isfinite(imu->pitch) || !isfinite(imu->pitch_rate) ||
        !isfinite(imu->specific_force_x) ||
        !isfinite(imu->specific_force_y) ||
        !isfinite(imu->specific_force_z)) {
        return 0U;
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (!isfinite(leg[side].angular_velocity)) return 0U;
    }
    return 1U;
}

static void body_to_heading_frame(
    const bc_impact_observer_t *observer,
    const bc_imu_feedback_t *imu,
    float *forward_acceleration,
    float *vertical_acceleration
) {
    const float sin_roll = sinf(imu->roll);
    const float cos_roll = cosf(imu->roll);
    const float sin_pitch = sinf(imu->pitch);
    const float cos_pitch = cosf(imu->pitch);
    const float force_x = imu->specific_force_x;
    const float force_y = imu->specific_force_y;
    const float force_z = imu->specific_force_z;

    *forward_acceleration =
        cos_pitch * force_x +
        sin_pitch * sin_roll * force_y +
        sin_pitch * cos_roll * force_z;
    *vertical_acceleration =
        -sin_pitch * force_x +
        cos_pitch * sin_roll * force_y +
        cos_pitch * cos_roll * force_z - observer->config.gravity;
}

static unsigned int history_index(
    const bc_impact_observer_t *observer,
    const unsigned int offset
) {
    return (observer->history_start + offset) %
        BC_IMPACT_HISTORY_CAPACITY;
}

static void append_interval(
    bc_impact_observer_t *observer,
    const bc_impact_history_interval_t *interval
) {
    if (observer->history_count == BC_IMPACT_HISTORY_CAPACITY) {
        observer->history_start =
            (observer->history_start + 1U) % BC_IMPACT_HISTORY_CAPACITY;
        --observer->history_count;
    }
    const unsigned int index = history_index(
        observer, observer->history_count);
    observer->history[index] = *interval;
    ++observer->history_count;
}

static bc_impact_window_output_t calculate_window(
    const bc_impact_observer_t *observer,
    const float window_seconds
) {
    bc_impact_window_output_t output = {0};
    if (!isfinite(window_seconds) || window_seconds <= 0.0F) return output;

    float remaining = window_seconds;
    for (unsigned int count = 0U;
         count < observer->history_count && remaining > 1.0e-7F;
         ++count) {
        const unsigned int offset = observer->history_count - 1U - count;
        const bc_impact_history_interval_t *interval =
            &observer->history[history_index(observer, offset)];
        if (!isfinite(interval->duration_seconds) ||
            interval->duration_seconds <= 0.0F) {
            return (bc_impact_window_output_t){0};
        }

        const float used = fminf(remaining, interval->duration_seconds);
        const float fraction = used / interval->duration_seconds;
        const float boundary_pitch_rate = interval->pitch_rate_end +
            fraction * (interval->pitch_rate_start -
                interval->pitch_rate_end);
        const float boundary_wheel_velocity =
            interval->wheel_velocity_end + fraction *
            (interval->wheel_velocity_start -
                interval->wheel_velocity_end);

        output.forward_delta_velocity += used * 0.5F * (
            interval->forward_acceleration_end +
            interval->forward_acceleration_end + fraction *
                (interval->forward_acceleration_start -
                    interval->forward_acceleration_end));
        output.vertical_delta_velocity += used * 0.5F * (
            interval->vertical_acceleration_end +
            interval->vertical_acceleration_end + fraction *
                (interval->vertical_acceleration_start -
                    interval->vertical_acceleration_end));
        output.pitch_rate_delta =
            observer->previous_pitch_rate - boundary_pitch_rate;
        output.wheel_velocity_delta =
            observer->previous_wheel_velocity - boundary_wheel_velocity;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            const float boundary_leg_rate = interval->leg_rate_end[side] +
                fraction * (interval->leg_rate_start[side] -
                    interval->leg_rate_end[side]);
            output.leg_rate_delta[side] =
                observer->previous_leg_rate[side] - boundary_leg_rate;
        }
        remaining -= used;
    }

    if (remaining > 1.0e-6F) return (bc_impact_window_output_t){0};
    output.duration_seconds = window_seconds;
    output.wheel_imu_delta_mismatch =
        output.wheel_velocity_delta - output.forward_delta_velocity;
    output.valid = 1U;
    return output;
}

void bc_impact_observer_default_config(
    bc_impact_observer_config_t *config
) {
    *config = (bc_impact_observer_config_t){
        .gravity = 9.81F,
        .window_seconds = {0.005F, 0.010F},
    };
}

void bc_impact_observer_init(
    bc_impact_observer_t *observer,
    const bc_impact_observer_config_t *config
) {
    observer->config = *config;
    bc_impact_observer_reset(observer);
}

void bc_impact_observer_reset(bc_impact_observer_t *observer) {
    memset(&observer->output, 0, sizeof(observer->output));
    memset(observer->history, 0, sizeof(observer->history));
    observer->history_start = 0U;
    observer->history_count = 0U;
    observer->previous_forward_acceleration = 0.0F;
    observer->previous_vertical_acceleration = 0.0F;
    observer->previous_pitch_rate = 0.0F;
    memset(
        observer->previous_leg_rate, 0,
        sizeof(observer->previous_leg_rate));
    observer->previous_wheel_velocity = 0.0F;
    observer->initialized = 0U;
}

void bc_impact_observer_update(
    bc_impact_observer_t *observer,
    const bc_imu_feedback_t *imu,
    const bc_leg_kinematics_t leg[BC_SIDE_NUM],
    const float wheel_velocity,
    const float timestep_seconds
) {
    if (!finite_input(imu, leg, wheel_velocity, timestep_seconds)) {
        bc_impact_observer_reset(observer);
        return;
    }

    float forward_acceleration = 0.0F;
    float vertical_acceleration = 0.0F;
    body_to_heading_frame(
        observer, imu, &forward_acceleration, &vertical_acceleration);
    if (!isfinite(forward_acceleration) ||
        !isfinite(vertical_acceleration)) {
        bc_impact_observer_reset(observer);
        return;
    }

    float leg_rate[BC_SIDE_NUM];
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        leg_rate[side] = leg[side].angular_velocity + imu->pitch_rate;
    }

    if (observer->initialized) {
        bc_impact_history_interval_t interval = {
            .duration_seconds = timestep_seconds,
            .forward_acceleration_start =
                observer->previous_forward_acceleration,
            .forward_acceleration_end = forward_acceleration,
            .vertical_acceleration_start =
                observer->previous_vertical_acceleration,
            .vertical_acceleration_end = vertical_acceleration,
            .pitch_rate_start = observer->previous_pitch_rate,
            .pitch_rate_end = imu->pitch_rate,
            .wheel_velocity_start = observer->previous_wheel_velocity,
            .wheel_velocity_end = wheel_velocity,
        };
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            interval.leg_rate_start[side] =
                observer->previous_leg_rate[side];
            interval.leg_rate_end[side] = leg_rate[side];
        }
        append_interval(observer, &interval);
    }

    observer->previous_forward_acceleration = forward_acceleration;
    observer->previous_vertical_acceleration = vertical_acceleration;
    observer->previous_pitch_rate = imu->pitch_rate;
    memcpy(observer->previous_leg_rate, leg_rate, sizeof(leg_rate));
    observer->previous_wheel_velocity = wheel_velocity;
    observer->initialized = 1U;

    observer->output.forward_acceleration = forward_acceleration;
    observer->output.vertical_acceleration = vertical_acceleration;
    observer->output.valid = 1U;
    for (int window = 0; window < BC_IMPACT_WINDOW_NUM; ++window) {
        observer->output.window[window] = calculate_window(
            observer, observer->config.window_seconds[window]);
    }
}
