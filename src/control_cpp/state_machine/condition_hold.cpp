#include "balance_cpp/state_machine/condition_hold.hpp"

namespace balance::control {

bool ConditionHold::update(
    const bool condition,
    const float required,
    const float timestep
) {
    if (!condition) {
        reset();
        return false;
    }
    elapsed_ += timestep;
    return elapsed_ >= required;
}

} // namespace balance::control
