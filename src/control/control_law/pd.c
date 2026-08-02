#include "balance/control_law/pd.h"

float bc_pd_calculate(
    const bc_pd_controller_t *controller,
    float position_error, float velocity_error
) {
    const float output = controller->kp * position_error + controller->kd * velocity_error;

    if (output > +controller->output_limit) return +controller->output_limit;
    if (output < -controller->output_limit) return -controller->output_limit;
    return output;
}
