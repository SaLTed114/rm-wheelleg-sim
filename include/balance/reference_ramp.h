#ifndef BALANCE_REFERENCE_RAMP_H
#define BALANCE_REFERENCE_RAMP_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float value_limit;
    float rate_limit;
} bc_reference_ramp_config_t;

typedef struct {
    float value;
} bc_reference_ramp_t;

void bc_reference_ramp_reset(bc_reference_ramp_t *ramp);
float bc_reference_ramp_update(
    bc_reference_ramp_t *ramp,
    const bc_reference_ramp_config_t *config,
    float target,
    float timestep_seconds);

#ifdef __cplusplus
}
#endif

#endif
