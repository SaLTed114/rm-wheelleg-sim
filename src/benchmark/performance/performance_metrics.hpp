#ifndef BALANCE_BENCHMARK_PERFORMANCE_METRICS_HPP
#define BALANCE_BENCHMARK_PERFORMANCE_METRICS_HPP

#include <algorithm>
#include <cmath>
#include <limits>

namespace balance::benchmark {

inline double normalized_response_progress(
    const double actual, const double initial, const double target
) noexcept {
    const double span = target - initial;
    return std::abs(span) < 1.0e-12 ? 0.0 : (actual - initial) / span;
}

inline void capture_response_crossing(
    const double progress, const double level,
    const double elapsed, double &crossing
) noexcept {
    if (!std::isfinite(crossing) && progress >= level) crossing = elapsed;
}

inline double response_overshoot(
    const double maximum_progress,
    const double initial, const double target
) noexcept {
    return std::max(0.0, maximum_progress - 1.0) *
        std::abs(target - initial);
}

inline double t10_t90_acceleration(
    const double t10, const double t90,
    const double initial, const double target
) noexcept {
    return std::isfinite(t10) && std::isfinite(t90) && t90 > t10 ?
        0.8 * std::abs(target - initial) / (t90 - t10) :
        std::numeric_limits<double>::quiet_NaN();
}

} // namespace balance::benchmark

#endif
