#include "balance/state_machine/support_phase.h"

#include <math.h>
#include <stddef.h>

#include "balance/math_utils.h"

static uint8_t bc_support_all_contact(
    const bc_state_machine_input_t *input,
    const bc_contact_state_t state
) {
    if (input->support_force == NULL) return 0U;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (!input->support_force[side].valid ||
            input->support_force[side].state != state) return 0U;
    }
    return 1U;
}

static uint8_t bc_support_any_contact(
    const bc_state_machine_input_t *input,
    const bc_contact_state_t state
) {
    if (input->support_force == NULL) return 0U;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (input->support_force[side].valid &&
            input->support_force[side].state == state) return 1U;
    }
    return 0U;
}

static uint8_t bc_support_legs_slow(
    const bc_support_phase_t *phase,
    const bc_state_machine_input_t *input
) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (fabsf(input->leg[side].length_velocity) >=
            phase->config.leg_speed_threshold) return 0U;
    }
    return 1U;
}

static uint8_t bc_support_legs_at_working_length(
    const bc_support_phase_t *phase,
    const bc_state_machine_input_t *input,
    const float working_leg_length
) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (fabsf(input->leg[side].length - working_leg_length) >
            phase->config.leg_length_tolerance) return 0U;
    }
    return 1U;
}

static uint8_t bc_support_retraction_complete(
    const bc_support_phase_t *phase,
    const float working_leg_length
) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        const bc_support_leg_request_t *request =
            &phase->request.leg[side];
        if (!request->contact_latched ||
            fabsf(request->equilibrium_length - working_leg_length) >
                1.0e-6F) return 0U;
    }
    return 1U;
}

static uint8_t bc_support_all_contact_latched(
    const bc_support_phase_t *phase
) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (!phase->request.leg[side].contact_latched) return 0U;
    }
    return 1U;
}

static uint8_t bc_support_all_force_below(
    const bc_state_machine_input_t *input,
    const float threshold
) {
    if (input->support_force == NULL) return 0U;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (!input->support_force[side].valid ||
            !isfinite(input->support_force[side].filtered_vertical_force) ||
            input->support_force[side].filtered_vertical_force >= threshold) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t bc_support_any_force_above(
    const bc_state_machine_input_t *input,
    const float threshold
) {
    if (input->support_force == NULL) return 0U;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (input->support_force[side].valid &&
            isfinite(input->support_force[side].filtered_vertical_force) &&
            input->support_force[side].filtered_vertical_force > threshold) {
            return 1U;
        }
    }
    return 0U;
}

static uint8_t bc_support_side_contact(
    const bc_support_phase_t *phase,
    const bc_state_machine_input_t *input,
    const int side
) {
    if (input->support_force == NULL ||
        !input->support_force[side].valid) return 0U;
    return (phase->side_airborne_diagnosed[side] &&
            input->support_force[side].state == BC_CONTACT_GROUND) ||
        ((phase->airborne_unloaded ||
          (phase->fast_air_mixed_contact &&
           !phase->airborne_diagnosed)) &&
         isfinite(input->support_force[side].filtered_vertical_force) &&
         input->support_force[side].filtered_vertical_force >
            phase->config.landing_force_threshold);
}

static void bc_support_reset_request(bc_support_phase_t *phase) {
    phase->request = (bc_support_phase_request_t){0};
}

static void bc_support_prepare_airborne_request(
    bc_support_phase_t *phase
) {
    phase->request.disable_wheels = 1U;
    phase->request.disabled_state_feedback =
        BC_STATE_FEEDBACK_MASK(BC_STATE_S) |
        BC_STATE_FEEDBACK_MASK(BC_STATE_DS) |
        BC_STATE_FEEDBACK_MASK(BC_STATE_PSI) |
        BC_STATE_FEEDBACK_MASK(BC_STATE_DPSI) |
        BC_STATE_FEEDBACK_MASK(BC_STATE_THETA_B) |
        BC_STATE_FEEDBACK_MASK(BC_STATE_DTHETA_B);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        phase->request.leg[side].override_length = 1U;
        phase->request.leg[side].length_strategy =
            BC_LEG_LENGTH_POSITION;
        phase->request.leg[side].target =
            phase->config.airborne_leg_length;
    }
}

static void bc_support_update_landing_request(
    bc_support_phase_t *phase,
    const bc_state_machine_input_t *input,
    const float working_leg_length
) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        bc_support_leg_request_t *request = &phase->request.leg[side];
        request->override_length = 1U;
        if (!request->contact_latched &&
            bc_support_side_contact(phase, input, side)) {
            request->contact_latched = 1U;
            request->captured_length = input->leg[side].length;
            request->equilibrium_length = input->leg[side].length;
            const bc_support_force_output_t *support =
                &input->support_force[side];
            request->applied_force =
                isfinite(support->axial_force) &&
                support->axial_force >=
                    phase->config.landing_minimum_force &&
                support->axial_force <=
                    phase->config.landing_maximum_force ?
                support->axial_force : input->nominal_axial_force[side];
        }
        if (!request->contact_latched) {
            request->length_strategy = BC_LEG_LENGTH_POSITION;
            request->target = phase->config.airborne_leg_length;
            continue;
        }

        request->equilibrium_length = fmaxf(
            request->equilibrium_length -
                phase->config.landing_retraction_speed *
                    input->timestep_seconds,
            working_leg_length);
        request->requested_force =
            input->nominal_axial_force[side] +
            phase->config.landing_stiffness *
                (request->equilibrium_length - input->leg[side].length) -
            phase->config.landing_damping *
                input->leg[side].length_velocity;
        const float limited_force = bc_clampf(
            request->requested_force,
            phase->config.landing_minimum_force,
            phase->config.landing_maximum_force);
        const float maximum_step = fmaxf(
            0.0F, phase->config.landing_force_rate_limit *
                input->timestep_seconds);
        const float force_delta = bc_clampf(
            limited_force - request->applied_force,
            -maximum_step, maximum_step);
        request->force_rate_limited =
            fabsf(limited_force - request->applied_force) >
                maximum_step + 1.0e-6F;
        request->applied_force += force_delta;
        request->length_strategy = BC_LEG_LENGTH_AXIAL_FORCE;
        request->target = request->applied_force;
    }
}

static void bc_support_prepare_recovery_request(
    bc_support_phase_t *phase,
    const bc_state_machine_input_t *input,
    const float working_leg_length,
    const uint8_t entering
) {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        bc_support_leg_request_t *request = &phase->request.leg[side];
        request->override_length = 1U;
        request->length_strategy = BC_LEG_LENGTH_POSITION_SUPPORT;
        if (entering) {
            request->target = input->leg[side].length;
            if (input->length_position_kp > 0.0F) {
                request->target +=
                    (request->applied_force -
                     input->nominal_axial_force[side] +
                     input->length_position_kd *
                        input->leg[side].length_velocity) /
                    input->length_position_kp;
            }
        } else {
            const float maximum_step =
                phase->config.recovery_reference_speed *
                input->timestep_seconds;
            request->target += bc_clampf(
                working_leg_length - request->target,
                -maximum_step, maximum_step);
        }
    }
}

static void bc_support_set_state(
    bc_support_phase_t *phase,
    const bc_support_phase_state_t state
) {
    phase->state = state;
    bc_condition_hold_reset(&phase->transition_hold);
    bc_condition_hold_reset(&phase->fast_air_hold);
    bc_condition_hold_reset(&phase->landing_hold);
}

void bc_support_phase_default_config(bc_support_phase_config_t *config) {
    *config = (bc_support_phase_config_t){
        .leg_speed_threshold = 0.1F,
        .leg_length_tolerance = 0.02F,
        .stable_duration = 0.05F,
        .fast_air_force_threshold = 50.0F,
        .fast_air_specific_force_threshold = 5.0F,
        .fast_air_confirm_duration = 0.005F,
        .unloaded_force_threshold = 10.0F,
        .landing_force_threshold = 15.0F,
        .landing_confirm_duration = 0.003F,
        .airborne_leg_length = 0.38F,
        .landing_stiffness = 800.0F,
        .landing_damping = 80.0F,
        .landing_minimum_force = 0.0F,
        .landing_maximum_force = 240.0F,
        .landing_force_rate_limit = 3000.0F,
        .landing_retraction_speed = 0.8F,
        .recovery_reference_speed = 0.1F,
    };
}

void bc_support_phase_init(
    bc_support_phase_t *phase,
    const bc_support_phase_config_t *config
) {
    phase->config = *config;
    bc_support_phase_reset(phase);
}

void bc_support_phase_reset(bc_support_phase_t *phase) {
    phase->state = BC_SUPPORT_GROUND;
    bc_condition_hold_reset(&phase->transition_hold);
    bc_condition_hold_reset(&phase->fast_air_hold);
    bc_condition_hold_reset(&phase->landing_hold);
    phase->airborne_unloaded = 0U;
    phase->airborne_diagnosed = 0U;
    phase->fast_air_mixed_contact = 0U;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        phase->side_airborne_diagnosed[side] = 0U;
    }
    bc_support_reset_request(phase);
}

void bc_support_phase_update(
    bc_support_phase_t *phase,
    const bc_state_machine_input_t *input,
    const float working_leg_length
) {
    const bc_support_phase_state_t previous_state = phase->state;
    const uint8_t all_air = bc_support_all_contact(input, BC_CONTACT_AIR);
    const uint8_t any_ground =
        bc_support_any_contact(input, BC_CONTACT_GROUND);
    const uint8_t all_ground =
        bc_support_all_contact(input, BC_CONTACT_GROUND);
    const uint8_t fast_air =
        bc_support_all_force_below(
            input, phase->config.fast_air_force_threshold) &&
        isfinite(input->specific_force_norm) &&
        input->specific_force_norm <
            phase->config.fast_air_specific_force_threshold;

    switch (phase->state) {
    case BC_SUPPORT_GROUND: {
        const uint8_t fast_air_confirmed = bc_condition_hold_update(
                &phase->fast_air_hold, fast_air,
                phase->config.fast_air_confirm_duration,
                input->timestep_seconds);
        if (all_air || fast_air_confirmed) {
            bc_support_set_state(phase, BC_SUPPORT_AIRBORNE);
            phase->airborne_unloaded = all_air ||
                bc_support_all_force_below(
                    input, phase->config.unloaded_force_threshold);
            phase->airborne_diagnosed = all_air;
            phase->fast_air_mixed_contact =
                !all_air && fast_air_confirmed &&
                bc_support_any_contact(input, BC_CONTACT_AIR);
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                phase->side_airborne_diagnosed[side] =
                    input->support_force != NULL &&
                    input->support_force[side].valid &&
                    input->support_force[side].state == BC_CONTACT_AIR;
            }
        }
        break;
    }

    case BC_SUPPORT_AIRBORNE: {
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            phase->side_airborne_diagnosed[side] =
                phase->side_airborne_diagnosed[side] ||
                (input->support_force != NULL &&
                 input->support_force[side].valid &&
                 input->support_force[side].state == BC_CONTACT_AIR);
        }
        phase->airborne_diagnosed =
            phase->airborne_diagnosed || all_air;
        phase->airborne_unloaded = phase->airborne_unloaded || all_air ||
            bc_support_all_force_below(
                input, phase->config.unloaded_force_threshold);
        const uint8_t touchdown_after_fast_air =
            phase->fast_air_mixed_contact && !fast_air &&
            bc_support_any_force_above(
                input, phase->config.landing_force_threshold);
        const uint8_t touchdown_after_contact_air =
            phase->airborne_unloaded &&
            ((phase->airborne_diagnosed && any_ground) ||
             bc_support_any_force_above(
                input, phase->config.landing_force_threshold));
        if (bc_condition_hold_update(
                &phase->landing_hold,
                touchdown_after_contact_air || touchdown_after_fast_air,
                phase->config.landing_confirm_duration,
                input->timestep_seconds)) {
            bc_support_set_state(phase, BC_SUPPORT_LANDING_RETRACT);
        }
        break;
    }

    case BC_SUPPORT_LANDING_RETRACT:
        if (all_air && bc_support_all_force_below(
                input, phase->config.unloaded_force_threshold)) {
            bc_support_set_state(phase, BC_SUPPORT_AIRBORNE);
            phase->airborne_unloaded = 1U;
            phase->airborne_diagnosed = 1U;
            phase->fast_air_mixed_contact = 0U;
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                phase->side_airborne_diagnosed[side] = 1U;
            }
        } else if (bc_condition_hold_update(
                &phase->transition_hold,
                bc_support_all_contact_latched(phase) &&
                    bc_support_retraction_complete(
                    phase, working_leg_length) &&
                    bc_support_legs_slow(phase, input),
                phase->config.stable_duration,
                input->timestep_seconds)) {
            bc_support_set_state(phase, BC_SUPPORT_GROUND_RECOVER);
        }
        break;

    case BC_SUPPORT_GROUND_RECOVER:
        if (all_air && bc_support_all_force_below(
                input, phase->config.unloaded_force_threshold)) {
            bc_support_set_state(phase, BC_SUPPORT_AIRBORNE);
            phase->airborne_unloaded = 1U;
            phase->airborne_diagnosed = 1U;
            phase->fast_air_mixed_contact = 0U;
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                phase->side_airborne_diagnosed[side] = 1U;
            }
        } else if (bc_condition_hold_update(
                &phase->transition_hold,
                all_ground && bc_support_legs_slow(phase, input) &&
                    bc_support_legs_at_working_length(
                        phase, input, working_leg_length),
                phase->config.stable_duration,
                input->timestep_seconds)) {
            bc_support_set_state(phase, BC_SUPPORT_GROUND);
        }
        break;
    }

    const uint8_t entering = phase->state != previous_state;
    if (entering && phase->state == BC_SUPPORT_AIRBORNE) {
        bc_support_reset_request(phase);
    }
    switch (phase->state) {
    case BC_SUPPORT_GROUND:
        bc_support_reset_request(phase);
        break;
    case BC_SUPPORT_AIRBORNE:
        bc_support_prepare_airborne_request(phase);
        break;
    case BC_SUPPORT_LANDING_RETRACT:
        if (entering) {
            bc_support_reset_request(phase);
        }
        bc_support_update_landing_request(
            phase, input, working_leg_length);
        break;
    case BC_SUPPORT_GROUND_RECOVER:
        bc_support_prepare_recovery_request(
            phase, input, working_leg_length, entering);
        break;
    }
}

const char *bc_support_phase_state_name(
    const bc_support_phase_state_t state
) {
    switch (state) {
    case BC_SUPPORT_GROUND: return "ground";
    case BC_SUPPORT_AIRBORNE: return "airborne";
    case BC_SUPPORT_LANDING_RETRACT: return "landing retract";
    case BC_SUPPORT_GROUND_RECOVER: return "ground recover";
    }
    return "unknown";
}
