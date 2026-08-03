#include "base_roll_restraint.hpp"

#include <algorithm>
#include <stdexcept>

namespace balance::benchmark {
namespace {

constexpr double kRollStiffness = 200.0;
constexpr double kRollDamping = 20.0;
constexpr double kTorqueLimit = 20.0;

} // namespace

BaseRollRestraint::BaseRollRestraint(
    const mjModel &model, const bool enabled
) : model_(model), enabled_(enabled) {
    base_body_ = mj_name2id(&model_, mjOBJ_BODY, "base_link");
    if (base_body_ < 0) {
        throw std::runtime_error("MuJoCo model is missing base_link");
    }
}

void BaseRollRestraint::reset() noexcept {
    torque_ = 0.0;
}

void BaseRollRestraint::apply(
    mjData &data, const double roll, const double roll_rate
) {
    torque_ = 0.0;
    if (!enabled_) return;

    mju_zero(data.qfrc_applied, model_.nv);
    torque_ = std::clamp(
        -kRollStiffness * roll - kRollDamping * roll_rate,
        -kTorqueLimit, kTorqueLimit);

    const mjtNum *rotation = data.xmat + 9 * base_body_;
    const mjtNum torque[3] = {
        torque_ * rotation[0],
        torque_ * rotation[3],
        torque_ * rotation[6],
    };
    const mjtNum force[3] = {};
    mj_applyFT(
        &model_, &data, force, torque,
        data.xipos + 3 * base_body_, base_body_, data.qfrc_applied);
}

} // namespace balance::benchmark
