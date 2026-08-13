#ifndef BALANCE_OBSERVATION_CONTEXT_H
#define BALANCE_OBSERVATION_CONTEXT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BC_WHEEL_OBSERVATION_DISABLED,
    BC_WHEEL_OBSERVATION_GROUND,
    BC_WHEEL_OBSERVATION_CONTACT_TRANSIENT,
    BC_WHEEL_OBSERVATION_AIRBORNE
} bc_wheel_observation_mode_t;

typedef struct {
    bc_wheel_observation_mode_t wheel_velocity;
} bc_observation_context_t;

#ifdef __cplusplus
}
#endif

#endif
