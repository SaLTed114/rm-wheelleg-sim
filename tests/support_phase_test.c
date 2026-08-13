#include "balance/state_machine/support_phase.h"

#include <stdio.h>
#include <string.h>

static void update_for(
    bc_support_phase_t *phase,
    bc_state_machine_input_t *input,
    const float seconds
) {
    const int steps = (int)(seconds / input->timestep_seconds + 0.5F);
    for (int step = 0; step < steps; ++step) {
        bc_support_phase_update(phase, input, 0.18F);
    }
}

static void set_nominal_control_input(
    bc_state_machine_input_t *input
) {
    input->nominal_axial_force[BC_L] = 80.0F;
    input->nominal_axial_force[BC_R] = 72.0F;
    input->length_position_kp = 1600.0F;
    input->length_position_kd = 75.0F;
}

int main(void) {
    bc_support_phase_config_t config;
    bc_support_phase_default_config(&config);
    bc_support_phase_t phase;
    bc_support_phase_init(&phase, &config);
    bc_leg_kinematics_t leg[BC_SIDE_NUM] = {0};
    bc_support_force_output_t support[BC_SIDE_NUM] = {0};
    bc_state_machine_input_t input = {
        .leg = leg,
        .support_force = support,
        .specific_force_norm = 9.81F,
        .timestep_seconds = 0.01F,
    };
    set_nominal_control_input(&input);

    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        support[side].valid = 1U;
        support[side].state = BC_CONTACT_AIR;
        leg[side].length = 0.30F;
    }
    bc_support_phase_update(&phase, &input, 0.18F);
    if (phase.state != BC_SUPPORT_AIRBORNE) {
        fputs("both air diagnoses did not enter airborne\n", stderr);
        return 1;
    }
    if (!phase.request.disable_wheels ||
        phase.request.leg[BC_L].length_strategy !=
            BC_LEG_LENGTH_POSITION ||
        phase.request.leg[BC_L].target !=
            config.airborne_leg_length) {
        fputs("airborne request is incorrect\n", stderr);
        return 1;
    }

    bc_support_phase_reset(&phase);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        support[side].state = BC_CONTACT_GROUND;
        support[side].filtered_vertical_force = 40.0F;
    }
    update_for(
        &phase, &input,
        config.fast_air_confirm_duration + input.timestep_seconds);
    if (phase.state != BC_SUPPORT_GROUND) {
        fputs("low support without free fall triggered fast air\n", stderr);
        return 1;
    }
    input.specific_force_norm = 0.0F;
    update_for(
        &phase, &input,
        config.fast_air_confirm_duration + input.timestep_seconds);
    if (phase.state != BC_SUPPORT_AIRBORNE) {
        fputs("free-fall unloading did not trigger fast air\n", stderr);
        return 1;
    }
    bc_support_phase_update(&phase, &input, 0.18F);
    if (phase.state != BC_SUPPORT_AIRBORNE) {
        fputs("fast air treated stale ground state as touchdown\n", stderr);
        return 1;
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        support[side].filtered_vertical_force = 5.0F;
    }
    bc_support_phase_update(&phase, &input, 0.18F);
    support[BC_L].filtered_vertical_force = 20.0F;
    bc_support_phase_update(&phase, &input, 0.18F);
    if (phase.state != BC_SUPPORT_LANDING_RETRACT) {
        fputs("force recovery after unloading did not trigger landing\n", stderr);
        return 1;
    }
    if (!phase.request.leg[BC_L].contact_latched ||
        phase.request.leg[BC_L].length_strategy !=
            BC_LEG_LENGTH_AXIAL_FORCE ||
        phase.request.disable_wheels) {
        fputs("landing request did not restore balance control\n", stderr);
        return 1;
    }

    bc_support_phase_reset(&phase);
    input.specific_force_norm = 3.0F;
    support[BC_L].state = BC_CONTACT_AIR;
    support[BC_L].filtered_vertical_force = -17.0F;
    support[BC_R].state = BC_CONTACT_GROUND;
    support[BC_R].filtered_vertical_force = 39.0F;
    update_for(
        &phase, &input,
        config.fast_air_confirm_duration + input.timestep_seconds);
    if (phase.state != BC_SUPPORT_AIRBORNE ||
        !phase.fast_air_mixed_contact || phase.airborne_unloaded) {
        fputs("mixed-contact fast air was not diagnosed correctly\n", stderr);
        return 1;
    }
    input.specific_force_norm = 30.0F;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        support[side].state = BC_CONTACT_GROUND;
        support[side].filtered_vertical_force = 100.0F;
    }
    update_for(
        &phase, &input,
        config.landing_confirm_duration + input.timestep_seconds);
    if (phase.state != BC_SUPPORT_LANDING_RETRACT) {
        fputs("impact after mixed-contact fast air did not trigger landing\n",
              stderr);
        return 1;
    }
    if (!phase.request.leg[BC_L].contact_latched ||
        !phase.request.leg[BC_R].contact_latched) {
        fputs("fast-air landing did not latch both loaded legs\n", stderr);
        return 1;
    }

    bc_support_phase_reset(&phase);
    input.specific_force_norm = 9.81F;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        support[side].state = BC_CONTACT_AIR;
        support[side].filtered_vertical_force = 0.0F;
    }
    bc_support_phase_update(&phase, &input, 0.18F);

    support[BC_L].state = BC_CONTACT_GROUND;
    bc_support_phase_update(&phase, &input, 0.18F);
    if (phase.state != BC_SUPPORT_LANDING_RETRACT) {
        fputs("first ground diagnosis did not enter landing\n", stderr);
        return 1;
    }

    support[BC_R].state = BC_CONTACT_GROUND;
    leg[BC_L].length = 0.30F;
    leg[BC_R].length = 0.30F;
    leg[BC_L].length_velocity = 0.2F;
    update_for(&phase, &input, 0.10F);
    if (phase.state != BC_SUPPORT_LANDING_RETRACT) {
        fputs("moving leg exited landing early\n", stderr);
        return 1;
    }
    leg[BC_L].length_velocity = 0.0F;
    update_for(&phase, &input, 0.10F);
    if (phase.state != BC_SUPPORT_LANDING_RETRACT) {
        fputs("landing exited before retraction completed\n", stderr);
        return 1;
    }
    update_for(
        &phase, &input,
        0.20F + config.stable_duration + input.timestep_seconds);
    if (phase.state != BC_SUPPORT_GROUND_RECOVER) {
        fputs("settled landing did not enter recovery\n", stderr);
        return 1;
    }
    if (phase.request.leg[BC_L].length_strategy !=
            BC_LEG_LENGTH_POSITION_SUPPORT ||
        phase.request.leg[BC_L].target == 0.18F) {
        fputs("recovery did not start from a bumpless reference\n", stderr);
        return 1;
    }

    update_for(&phase, &input, 0.10F);
    if (phase.state != BC_SUPPORT_GROUND_RECOVER) {
        fputs("extended legs exited recovery early\n", stderr);
        return 1;
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        leg[side].length = 0.17F;
    }
    update_for(
        &phase, &input,
        config.stable_duration + input.timestep_seconds);
    if (phase.state != BC_SUPPORT_GROUND ||
        strcmp(bc_support_phase_state_name(phase.state), "ground") != 0) {
        fputs("working-length legs did not return to ground\n", stderr);
        return 1;
    }

    support[BC_L].state = BC_CONTACT_AIR;
    bc_support_phase_update(&phase, &input, 0.18F);
    if (phase.state != BC_SUPPORT_GROUND) {
        fputs("single air diagnosis left ground\n", stderr);
        return 1;
    }
    support[BC_R].state = BC_CONTACT_AIR;
    bc_support_phase_update(&phase, &input, 0.18F);
    if (phase.state != BC_SUPPORT_AIRBORNE) {
        fputs("second air diagnosis did not leave ground\n", stderr);
        return 1;
    }
    return 0;
}
