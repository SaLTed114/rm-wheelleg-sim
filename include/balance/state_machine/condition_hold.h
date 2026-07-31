#ifndef BALANCE_STATE_MACHINE_CONDITION_HOLD_H
#define BALANCE_STATE_MACHINE_CONDITION_HOLD_H

#include <stdint.h>

typedef struct {
    float elapsed_seconds;
} bc_condition_hold_t;

static inline void bc_condition_hold_reset(
    bc_condition_hold_t *hold
) {
    hold->elapsed_seconds = 0.0F;
}

static inline uint8_t bc_condition_hold_update(
    bc_condition_hold_t *hold,
    const uint8_t condition,
    const float required_seconds,
    const float timestep_seconds
) {
    if (!condition) {
        bc_condition_hold_reset(hold);
        return 0U;
    }

    hold->elapsed_seconds += timestep_seconds;
    return hold->elapsed_seconds >= required_seconds;
}

#endif
