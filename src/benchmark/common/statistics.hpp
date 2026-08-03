#ifndef BALANCE_BENCHMARK_STATISTICS_HPP
#define BALANCE_BENCHMARK_STATISTICS_HPP

#include <cmath>
#include <cstddef>

namespace balance::benchmark {

class SampleStatistics {
public:
    void add(const double value) {
        sum_ += value;
        squared_sum_ += value * value;
        ++count_;
    }

    [[nodiscard]] std::size_t count() const { return count_; }
    [[nodiscard]] double mean() const {
        return count_ == 0U ? 0.0 :
            sum_ / static_cast<double>(count_);
    }
    [[nodiscard]] double rms() const {
        return count_ == 0U ? 0.0 :
            std::sqrt(squared_sum_ / static_cast<double>(count_));
    }

private:
    double sum_{};
    double squared_sum_{};
    std::size_t count_{};
};

class LinearTrend {
public:
    void add(const double time, const double value) {
        sum_time_ += time;
        sum_value_ += value;
        sum_time_squared_ += time * time;
        sum_time_value_ += time * value;
        ++count_;
    }

    [[nodiscard]] double slope() const {
        const double denominator =
            static_cast<double>(count_) * sum_time_squared_ -
            sum_time_ * sum_time_;
        if (count_ < 2U || std::abs(denominator) < 1.0e-12) return 0.0;
        return (
            static_cast<double>(count_) * sum_time_value_ -
            sum_time_ * sum_value_) / denominator;
    }

private:
    double sum_time_{};
    double sum_value_{};
    double sum_time_squared_{};
    double sum_time_value_{};
    std::size_t count_{};
};

} // namespace balance::benchmark

#endif
