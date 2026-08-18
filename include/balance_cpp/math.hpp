#ifndef BALANCE_CPP_MATH_HPP
#define BALANCE_CPP_MATH_HPP

#include <cmath>

namespace balance::control::math {

inline constexpr float pi = 3.14159265358979323846F;

inline constexpr float radians(const float degrees) noexcept {
    return degrees * pi / 180.0F;
}

inline float wrap_angle(const float angle) noexcept {
    return std::remainder(angle, 2.0F * pi);
}

} // namespace balance::control::math

#endif
