#ifndef BALANCE_BENCHMARK_DROP_POLICY_HPP
#define BALANCE_BENCHMARK_DROP_POLICY_HPP

#include <string_view>

#include "balance/types.h"

namespace balance::benchmark {

enum class DropAirPolicy {
    length_only,
    leg_lqr,
};

class DropContactLatch {
public:
    [[nodiscard]] bool update(bool wheel_contact) noexcept {
        latched_ = latched_ || wheel_contact;
        return latched_;
    }
    [[nodiscard]] bool latched() const noexcept { return latched_; }

private:
    bool latched_{};
};

[[nodiscard]] const char *drop_air_policy_name(DropAirPolicy policy) noexcept;
[[nodiscard]] DropAirPolicy parse_drop_air_policy(std::string_view name);

void apply_drop_air_policy(
    DropAirPolicy policy,
    bc_control_command_t &command) noexcept;

} // namespace balance::benchmark

#endif
