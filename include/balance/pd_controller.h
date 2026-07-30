#ifndef BALANCE_PD_CONTROLLER_H
#define BALANCE_PD_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float kd;
    float output_limit;
} bc_pd_controller_t;

float bc_pd_calculate(
    const bc_pd_controller_t *controller,
    float position_error, float velocity_error);

#ifdef __cplusplus
}
#endif

#endif
