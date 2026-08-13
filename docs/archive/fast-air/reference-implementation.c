/*
 * Archived fast-air excerpts. This file is documentation and is not built.
 * See DESIGN.md for the experiment results and the reason it was removed.
 */

typedef struct {
    float fast_air_force_threshold;          /* 50 N per leg */
    float fast_air_specific_force_threshold; /* 5 m/s^2 */
    float fast_air_confirm_duration;         /* 5 ms */
} archived_fast_air_config_t;

typedef struct {
    bc_condition_hold_t fast_air_hold;
    uint8_t airborne_unloaded;
    uint8_t airborne_diagnosed;
    uint8_t fast_air_mixed_contact;
    uint8_t side_airborne_diagnosed[BC_SIDE_NUM];
} archived_fast_air_state_t;

/* Original GROUND-side candidate. */
static uint8_t archived_fast_air_candidate(
    const bc_state_machine_input_t *input,
    const archived_fast_air_config_t *config
) {
    return bc_support_all_force_below(
            input, config->fast_air_force_threshold) &&
        isfinite(input->specific_force_norm) &&
        input->specific_force_norm <
            config->fast_air_specific_force_threshold;
}

/*
 * Original transition semantics. The problematic part was not the cue itself,
 * but granting it the same authority as a complete two-leg AIR diagnosis.
 */
static void archived_ground_transition(
    bc_support_phase_t *phase,
    archived_fast_air_state_t *state,
    const archived_fast_air_config_t *config,
    const bc_state_machine_input_t *input
) {
    const uint8_t all_air =
        bc_support_all_contact(input, BC_CONTACT_AIR);
    const uint8_t fast_air =
        archived_fast_air_candidate(input, config);
    const uint8_t fast_air_confirmed = bc_condition_hold_update(
        &state->fast_air_hold, fast_air,
        config->fast_air_confirm_duration,
        input->timestep_seconds);

    if (all_air || fast_air_confirmed) {
        bc_support_set_state(phase, BC_SUPPORT_AIRBORNE);
        state->airborne_unloaded = all_air ||
            bc_support_all_force_below(
                input, phase->config.unloaded_force_threshold);
        state->airborne_diagnosed = all_air;
        state->fast_air_mixed_contact =
            !all_air && fast_air_confirmed &&
            bc_support_any_contact(input, BC_CONTACT_AIR);
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            state->side_airborne_diagnosed[side] =
                input->support_force != NULL &&
                input->support_force[side].valid &&
                input->support_force[side].state == BC_CONTACT_AIR;
        }
    }
}

/* Original recovery accommodation added after the first ramp collision. */
static uint8_t archived_fast_air_touchdown(
    const archived_fast_air_state_t *state,
    const bc_support_phase_config_t *phase_config,
    const bc_state_machine_input_t *input,
    const uint8_t fast_air
) {
    return state->fast_air_mixed_contact && !fast_air &&
        bc_support_any_force_above(
            input, phase_config->landing_force_threshold);
}
