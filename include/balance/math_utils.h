#ifndef BALANCE_MATH_UTILS_H
#define BALANCE_MATH_UTILS_H

#include <math.h>

#define BC_PI   3.14159265358979323846
#define BC_PI_F 3.14159265358979323846F

static inline float bc_clampf(float value, float lower, float upper) {
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static inline float bc_wrap_anglef(float angle) {
    return remainderf(angle, 2.0F * BC_PI_F);
}

static inline double bc_wrap_angle(double angle) {
    return remainder(angle, 2.0 * BC_PI);
}

#endif
