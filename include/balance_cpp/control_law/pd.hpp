#ifndef BALANCE_CPP_PD_CONTROLLER_HPP
#define BALANCE_CPP_PD_CONTROLLER_HPP

namespace balance::control {

struct PdConfig {
    float kp{};
    float kd{};
    float output_limit{};
};

class PdController {
public:
    explicit PdController(PdConfig config = {});

    [[nodiscard]] float calculate(
        float position_error, float velocity_error) const;

private:
    PdConfig config_{};
};

} // namespace balance::control

#endif
