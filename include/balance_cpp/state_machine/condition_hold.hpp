#ifndef BALANCE_CPP_CONDITION_HOLD_HPP
#define BALANCE_CPP_CONDITION_HOLD_HPP

namespace balance::control {

class ConditionHold {
public:
    [[nodiscard]] bool update(
        bool condition, float required, float timestep
    );
    void reset() { elapsed_ = 0.0F; }
    float elapsed() const { return elapsed_; }

private:
    float elapsed_{};
};

} // namespace balance::control

#endif
