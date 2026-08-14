#include "step_dock.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "balance/math_utils.h"
#include "common/common_diagnostics.hpp"

namespace balance::benchmark {
namespace {

constexpr double kDisabledSettleSeconds = 2.0;
constexpr double kScenarioTimeoutSeconds = 15.0;
constexpr double kReadyHoldSeconds = 0.25;
constexpr double kSpeedWindowSeconds = 0.25;
constexpr double kMinimumReadyClearance = 0.20;
constexpr double kMinimumApproachVelocity = 1.0;
constexpr double kMaximumSpeedWindowRange = 0.10;
constexpr double kLegLengthTolerance = 0.012;
constexpr double kLegSpeedTolerance = 0.05;
constexpr double kPitchTolerance = 5.0 * BC_PI / 180.0;
constexpr double kPitchRateTolerance = 0.50;
constexpr double kRollTolerance = 3.0 * BC_PI / 180.0;
constexpr double kRollRateTolerance = 0.15;
constexpr double kYawTolerance = 5.0 * BC_PI / 180.0;
constexpr double kYawRateTolerance = 0.10;
constexpr double kFacePositionTolerance = 0.025;
constexpr double kTopPositionTolerance = 0.010;
constexpr double kZeroTolerance = 1.0e-8;
constexpr double kHorizontalDetectionThreshold = 5.0;
constexpr double kHorizontalDetectionHoldSeconds = 0.003;
constexpr double kTransferHoldWindowStartSeconds = 0.05;
constexpr double kTransferHoldWindowEndSeconds = 0.10;
constexpr double kLowBasePlatformSlidingFriction = 0.001;
constexpr char kBasePlatformContactPair[] =
    "base_keyboard_platform_contact";

int require_id(const mjModel &model, const mjtObj type, const char *name) {
    const int id = mj_name2id(&model, type, name);
    if (id < 0) {
        throw std::runtime_error(
            "missing MuJoCo object '" + std::string(name) + "'");
    }
    return id;
}

void set_initial_heading(
    const mjModel &model,
    mjData &data,
    const double heading_radians
) {
    const int body = require_id(model, mjOBJ_BODY, "base_link");
    const int joint = model.body_jntadr[body];
    if (model.body_jntnum[body] != 1 || joint < 0 ||
        model.jnt_type[joint] != mjJNT_FREE) {
        throw std::runtime_error(
            "step dock expects base_link to have one free joint");
    }
    const int quaternion = model.jnt_qposadr[joint] + 3;
    data.qpos[quaternion] = std::cos(0.5 * heading_radians);
    data.qpos[quaternion + 1] = 0.0;
    data.qpos[quaternion + 2] = 0.0;
    data.qpos[quaternion + 3] = std::sin(0.5 * heading_radians);
    mj_forward(&model, &data);
}

bool is_descendant(const mjModel &model, int body, const int ancestor) {
    while (body > 0) {
        if (body == ancestor) return true;
        body = model.body_parentid[body];
    }
    return false;
}

double maximum_actuation(const bc_controller_snapshot_t &snapshot) {
    double maximum = 0.0;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        maximum = std::max(maximum, std::abs(static_cast<double>(
            snapshot.actuation.wheel_torque[side])));
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            maximum = std::max(maximum, std::abs(static_cast<double>(
                snapshot.actuation.leg[side].joint_torque[joint])));
        }
    }
    return maximum;
}

struct HorizontalLegForce {
    std::array<double, BC_SIDE_NUM> tangential{};
    std::array<double, BC_SIDE_NUM> tangential_horizontal{};
    std::array<double, BC_SIDE_NUM> total_horizontal{};
};

HorizontalLegForce horizontal_leg_force(
    const bc_controller_snapshot_t &snapshot
) {
    const int angle_state[BC_SIDE_NUM] = {
        BC_STATE_THETA_L, BC_STATE_THETA_R,
    };
    HorizontalLegForce force{};
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (snapshot.leg[side].length <= 1.0e-9) continue;
        force.tangential[side] =
            snapshot.support_force[side].leg_torque /
            snapshot.leg[side].length;
        const double angle = snapshot.state.value[angle_state[side]];
        force.tangential_horizontal[side] =
            force.tangential[side] * std::cos(angle);
        force.total_horizontal[side] =
            snapshot.support_force[side].axial_force * std::sin(angle) +
            force.tangential_horizontal[side];
    }
    return force;
}

} // namespace

std::string step_dock_case_name(const StepDockSpec &spec) {
    if (spec.production_task) return "step_dock_complete";
    const auto delay_milliseconds = static_cast<long>(
        std::lround(1000.0 * spec.cut_delay_seconds));
    std::string name = delay_milliseconds == 0L ?
        "step_dock_passive" :
        "step_dock_delay_" + std::to_string(delay_milliseconds) + "ms";
    const auto heading_degrees = static_cast<long>(
        std::lround(spec.approach_heading_radians * 180.0 / BC_PI));
    if (heading_degrees != 0L) {
        name += heading_degrees > 0L ? "_yaw_pos" : "_yaw_neg";
        name += std::to_string(std::abs(heading_degrees)) + "deg";
    }
    if (std::isfinite(spec.target_collision_travel)) {
        const auto travel_millimeters = static_cast<long>(std::lround(
            1000.0 * spec.target_collision_travel));
        name += "_travel_" + std::to_string(travel_millimeters) + "mm";
    } else if (std::isfinite(spec.platform_gap_at_acceleration)) {
        const auto gap_millimeters = static_cast<long>(std::lround(
            1000.0 * spec.platform_gap_at_acceleration));
        name += "_gap_" + std::to_string(gap_millimeters) + "mm";
    }
    return name;
}

const StepDockSpec &step_dock_complete_case() {
    static const StepDockSpec complete = [] {
        StepDockSpec configured{};
        configured.production_task = true;
        configured.require_speed_stable = false;
        configured.base_platform_sliding_friction =
            kLowBasePlatformSlidingFriction;
        return configured;
    }();
    return complete;
}

const std::array<StepDockSpec, 1> &step_dock_cases() {
    static const std::array<StepDockSpec, 1> cases = {{
        step_dock_complete_case(),
    }};
    return cases;
}

const StepDockSpec *find_step_dock_case(const std::string_view name) noexcept {
    const auto &cases = step_dock_cases();
    const auto found = std::find_if(
        cases.begin(), cases.end(), [name](const StepDockSpec &spec) {
            return step_dock_case_name(spec) == name;
        });
    return found == cases.end() ? nullptr : &*found;
}

StepDockScenario::StepDockScenario(
    const StepDockSpec &spec,
    const mjModel &model
) : spec_(spec), name_(step_dock_case_name(spec)), model_(&model) {
    if (!std::isfinite(spec_.cut_delay_seconds) ||
        spec_.cut_delay_seconds < 0.0) {
        throw std::invalid_argument(
            "step dock cut delay must be finite and non-negative");
    }
    if (!std::isfinite(spec_.base_platform_sliding_friction) ||
        spec_.base_platform_sliding_friction < 0.0) {
        throw std::invalid_argument(
            "step dock base-platform friction must be finite and non-negative");
    }
    if (std::isfinite(spec_.platform_gap_at_acceleration) &&
        spec_.platform_gap_at_acceleration <= 0.0) {
        throw std::invalid_argument(
            "step dock platform gap must be positive when specified");
    }
    if (std::isfinite(spec_.target_collision_travel) &&
        spec_.target_collision_travel <= 0.0) {
        throw std::invalid_argument(
            "step dock target collision travel must be positive");
    }
    platform_ = require_id(
        model, mjOBJ_GEOM, "keyboard_platform_200mm");
    base_body_ = require_id(model, mjOBJ_BODY, "base_link");
    if (model.body_geomnum[base_body_] != 1) {
        throw std::runtime_error(
            "step dock expects one base_link collision geom");
    }
    base_geom_ = model.body_geomadr[base_body_];
    if (model.geom_type[base_geom_] != mjGEOM_MESH) {
        throw std::runtime_error("base_link collision geom is not a mesh");
    }
    leg_root_front_ = {{
        require_id(model, mjOBJ_BODY, "Right_front_link"),
        require_id(model, mjOBJ_BODY, "Left_front_link"),
    }};
    leg_root_rear_ = {{
        require_id(model, mjOBJ_BODY, "Right_rear_link"),
        require_id(model, mjOBJ_BODY, "Left_rear_link"),
    }};
    wheel_axis_ = {{
        require_id(model, mjOBJ_SITE, "Right_wheel_axis_site"),
        require_id(model, mjOBJ_SITE, "Left_wheel_axis_site"),
    }};
    wheel_collision_ = {{
        require_id(model, mjOBJ_GEOM, "Right_wheel_collision"),
        require_id(model, mjOBJ_GEOM, "Left_wheel_collision"),
    }};
    for (const int geom : wheel_collision_) {
        wheel_radius_ = std::max(
            wheel_radius_, model.geom_aabb[6 * geom + 3]);
    }

    for (int axis = 0; axis < 3; ++axis) {
        base_bounds_center_[axis] = model.geom_aabb[6 * base_geom_ + axis];
        base_bounds_half_[axis] =
            model.geom_aabb[6 * base_geom_ + 3 + axis];
    }
}

void StepDockScenario::reset(sim::MujocoPlant &plant) {
    layout_ = plant.configure_step_dock_benchmark();
    if (std::abs(layout_.height - spec_.platform_height) > 1.0e-9) {
        throw std::runtime_error("step dock platform height mismatch");
    }
    phase_ = StepDockPhase::disabled_settle;
    command_ = {};
    speed_window_.clear();
    reset_time_ = plant.data().time;
    ready_hold_start_ = -1.0;
    passive_start_ = std::numeric_limits<double>::quiet_NaN();
    control_cut_time_ = std::numeric_limits<double>::quiet_NaN();
    collision_time_ = std::numeric_limits<double>::quiet_NaN();
    collision_velocity_ = std::numeric_limits<double>::quiet_NaN();
    collision_clearance_ = std::numeric_limits<double>::quiet_NaN();
    collision_world_heading_ = std::numeric_limits<double>::quiet_NaN();
    acceleration_start_x_ = std::numeric_limits<double>::quiet_NaN();
    collision_x_ = std::numeric_limits<double>::quiet_NaN();
    trigger_contact_force_ = 0.0;
    transfer_start_length_ = {};
    transfer_start_angle_ = {};
    transfer_length_reference_ = {};
    transfer_angle_reference_ = {};
    first_contact_pair_ = "none";
    balance_engaged_ = false;
    start_ready_ = false;
    speed_stable_ = false;
    contact_detected_ = false;
    body_contact_before_trigger_ = false;
    observation_complete_ = false;
    issue_ = "none";
}

const char *StepDockScenario::phase_name() const noexcept {
    switch (phase_) {
    case StepDockPhase::disabled_settle: return "step_disabled_settle";
    case StepDockPhase::standing: return "step_standing";
    case StepDockPhase::accelerating: return "step_accelerating";
    case StepDockPhase::approach: return "step_approach";
    case StepDockPhase::impact_delay: return "step_impact_delay";
    case StepDockPhase::passive: return "step_passive";
    case StepDockPhase::transfer: return "step_transfer";
    case StepDockPhase::transfer_hold: return "step_transfer_hold";
    case StepDockPhase::rebalance: return "step_rebalance";
    case StepDockPhase::complete: return "step_complete";
    case StepDockPhase::failed: return "step_failed";
    }
    return "step_unknown";
}

int StepDockScenario::leg_side_for_body(const int body) const noexcept {
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        if (is_descendant(*model_, body, leg_root_front_[side]) ||
            is_descendant(*model_, body, leg_root_rear_[side])) {
            return side;
        }
    }
    return -1;
}

std::string StepDockScenario::contact_pair(
    const mjContact &contact
) const {
    std::string result;
    for (int pair = 0; pair < 2; ++pair) {
        if (!result.empty()) result += '+';
        const int geom = contact.geom[pair];
        const char *name = mj_id2name(model_, mjOBJ_GEOM, geom);
        if (name != nullptr) {
            result += name;
            continue;
        }
        const int body = model_->geom_bodyid[geom];
        const char *body_name = mj_id2name(model_, mjOBJ_BODY, body);
        result += body_name != nullptr ? body_name : "unnamed";
    }
    return result;
}

StepDockContact StepDockScenario::observe_contacts(
    const mjData &data
) const {
    StepDockContact result{};
    for (int index = 0; index < data.ncon; ++index) {
        const mjContact &contact = data.contact[index];
        int robot_geom = -1;
        if (contact.geom[0] == platform_) robot_geom = contact.geom[1];
        if (contact.geom[1] == platform_) robot_geom = contact.geom[0];
        if (robot_geom < 0) continue;
        const int body = model_->geom_bodyid[robot_geom];
        if (!is_descendant(*model_, body, base_body_)) continue;

        mjtNum force[6]{};
        mj_contactForce(model_, &data, index, force);
        const double normal_force = std::max(0.0, force[0]);
        result.total_normal_force += normal_force;
        if (normal_force >= result.strongest_normal_force) {
            result.strongest_normal_force = normal_force;
            result.strongest_x = contact.pos[0];
            result.strongest_z = contact.pos[2];
            result.strongest_pair = contact_pair(contact);
        }

        const bool vertical_face =
            std::abs(contact.pos[0] - layout_.edge_x) <=
                kFacePositionTolerance &&
            contact.pos[2] < layout_.top_z - kTopPositionTolerance;
        const bool top_face =
            contact.pos[2] >= layout_.top_z - kTopPositionTolerance;
        if (body == base_body_) {
            result.base_face = result.base_face || vertical_face;
            result.base_top = result.base_top || top_face;
            continue;
        }
        const int side = leg_side_for_body(body);
        if (side >= 0 && top_face &&
            robot_geom == wheel_collision_[side]) {
            result.side_wheel_top[side] = true;
        }
        if (side >= 0 && vertical_face) {
            result.leg_face = true;
            result.side_face[side] = true;
            if (robot_geom == wheel_collision_[side]) {
                result.wheel_face = true;
            } else {
                result.other_leg_face = true;
            }
        }
    }
    return result;
}

double StepDockScenario::base_clearance(const mjData &data) const noexcept {
    const mjtNum *position = data.xpos + 3 * base_body_;
    const mjtNum *rotation = data.xmat + 9 * base_body_;
    double lowest = std::numeric_limits<double>::infinity();
    for (int sx : {-1, 1}) {
        for (int sy : {-1, 1}) {
            for (int sz : {-1, 1}) {
                const std::array<double, 3> local{{
                    base_bounds_center_[0] + sx * base_bounds_half_[0],
                    base_bounds_center_[1] + sy * base_bounds_half_[1],
                    base_bounds_center_[2] + sz * base_bounds_half_[2],
                }};
                const double world_z = position[2] +
                    rotation[6] * local[0] +
                    rotation[7] * local[1] +
                    rotation[8] * local[2];
                lowest = std::min(lowest, world_z);
            }
        }
    }
    return lowest - layout_.ground_z;
}

double StepDockScenario::base_world_heading(
    const mjData &data
) const noexcept {
    const mjtNum *rotation = data.xmat + 9 * base_body_;
    return std::atan2(rotation[3], rotation[0]);
}

double StepDockScenario::wheel_axis_x(
    const mjData &data,
    const int side
) const {
    return data.site_xpos[3 * wheel_axis_[side]];
}

double StepDockScenario::wheel_axis_z(
    const mjData &data,
    const int side
) const {
    return data.site_xpos[3 * wheel_axis_[side] + 2];
}

void StepDockScenario::update_speed_window(
    const double time,
    const double velocity
) {
    if (speed_stable_) return;
    speed_window_.emplace_back(time, velocity);
    while (!speed_window_.empty() &&
           speed_window_.front().first < time - kSpeedWindowSeconds) {
        speed_window_.pop_front();
    }
    if (speed_window_.empty() ||
        time - speed_window_.front().first < kSpeedWindowSeconds) {
        return;
    }
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    for (const auto &[sample_time, sample_velocity] : speed_window_) {
        static_cast<void>(sample_time);
        minimum = std::min(minimum, sample_velocity);
        maximum = std::max(maximum, sample_velocity);
    }
    speed_stable_ = minimum >= kMinimumApproachVelocity &&
        maximum - minimum <= kMaximumSpeedWindowRange;
    if (speed_stable_) phase_ = StepDockPhase::approach;
}

void StepDockScenario::record_collision(
    const double time,
    const double base_x,
    const double velocity,
    const double clearance,
    const double world_heading,
    const StepDockContact &contact,
    const bool body_first
) noexcept {
    phase_ = StepDockPhase::impact_delay;
    collision_time_ = time;
    collision_x_ = base_x;
    collision_velocity_ = velocity;
    collision_clearance_ = clearance;
    collision_world_heading_ = world_heading;
    trigger_contact_force_ = contact.strongest_normal_force;
    first_contact_pair_ = contact.strongest_pair;
    body_contact_before_trigger_ = body_first;
    contact_detected_ = !body_first;
    if (body_first) {
        issue_ = "body_contact_before_trigger";
    } else if (spec_.require_speed_stable && !speed_stable_) {
        issue_ = "entry_speed_not_stable";
    }
}

void StepDockScenario::enter_passive(
    const double time,
    const bc_controller_snapshot_t &snapshot
) noexcept {
    phase_ = StepDockPhase::passive;
    passive_start_ = time;
    control_cut_time_ = time;
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        transfer_start_length_[side] = snapshot.leg[side].length;
        transfer_start_angle_[side] = snapshot.leg[side].angle_body;
        transfer_length_reference_[side] = transfer_start_length_[side];
        transfer_angle_reference_[side] = transfer_start_angle_[side];
    }
}

double StepDockScenario::passive_elapsed(const double time) const noexcept {
    return std::isfinite(passive_start_) ? time - passive_start_ : 0.0;
}

double StepDockScenario::collision_elapsed(const double time) const noexcept {
    return std::isfinite(collision_time_) ? time - collision_time_ : 0.0;
}

void StepDockScenario::fail(const char *issue) noexcept {
    issue_ = issue;
    phase_ = StepDockPhase::failed;
}

void StepDockScenario::step(
    sim::MujocoPlant &plant,
    sim::SimulationRunner &runner,
    const SimulationSampler &sampler
) {
    if (finished()) return;
    const double time = plant.data().time;
    if (time - reset_time_ > kScenarioTimeoutSeconds) {
        fail("step_dock_timeout");
        return;
    }

    const SimulationSample sample = sampler.read(
        plant.data(), runner.snapshot());
    const StepDockContact contact = observe_contacts(plant.data());
    const double clearance = base_clearance(plant.data());
    const double world_heading = base_world_heading(plant.data());
    const double velocity = sample.base.forward_velocity;
    command_.system_enabled = static_cast<uint8_t>(
        phase_ != StepDockPhase::disabled_settle);
    command_.balance_restart = static_cast<uint8_t>(
        command_.system_enabled &&
        runner.snapshot().state_machine.system == BC_SYSTEM_OFF);
    command_.forward_velocity = 0.0F;
    command_.task = spec_.production_task &&
            phase_ != StepDockPhase::disabled_settle ?
        BC_OPERATOR_TASK_STEP_DOCK : BC_OPERATOR_TASK_NORMAL;

    if (phase_ == StepDockPhase::disabled_settle) {
        runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
        if (time - reset_time_ >= kDisabledSettleSeconds) {
            phase_ = StepDockPhase::standing;
        }
        return;
    }

    if (phase_ == StepDockPhase::standing) {
        const auto &snapshot = runner.snapshot();
        const bool active =
            snapshot.state_machine.motion == BC_MOTION_ACTIVE;
        balance_engaged_ = balance_engaged_ || active;
        if (spec_.production_task && active) {
            start_ready_ = true;
            acceleration_start_x_ = sample.base.x;
            phase_ = StepDockPhase::accelerating;
            command_.forward_velocity =
                static_cast<float>(spec_.target_velocity);
            runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
            return;
        }
        bool ready = active && clearance >= kMinimumReadyClearance &&
            std::abs(snapshot.state.value[BC_STATE_THETA_B]) <=
                kPitchTolerance &&
            std::abs(snapshot.state.value[BC_STATE_DTHETA_B]) <=
                kPitchRateTolerance &&
            std::abs(snapshot.roll) <= kRollTolerance &&
            std::abs(snapshot.roll_rate) <= kRollRateTolerance &&
            std::abs(snapshot.state.value[BC_STATE_PSI]) <= kYawTolerance &&
            std::abs(snapshot.state.value[BC_STATE_DPSI]) <=
                kYawRateTolerance;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            ready = ready &&
                std::abs(snapshot.leg[side].length - spec_.leg_length) <=
                    kLegLengthTolerance &&
                std::abs(snapshot.leg[side].length_velocity) <=
                    kLegSpeedTolerance;
        }
        if (ready) {
            if (ready_hold_start_ < 0.0) ready_hold_start_ = time;
            if (time - ready_hold_start_ >= kReadyHoldSeconds) {
                start_ready_ = true;
                acceleration_start_x_ = sample.base.x;
                if (std::isfinite(spec_.platform_gap_at_acceleration)) {
                    const double wheel_front = std::max(
                        wheel_axis_x(plant.data(), BC_L),
                        wheel_axis_x(plant.data(), BC_R)) + wheel_radius_;
                    layout_ = plant.configure_step_dock_benchmark(
                        wheel_front + spec_.platform_gap_at_acceleration);
                }
                phase_ = StepDockPhase::accelerating;
                speed_window_.clear();
            }
        } else {
            ready_hold_start_ = -1.0;
        }
        runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
        return;
    }

    if (spec_.production_task && std::isfinite(passive_start_)) {
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            transfer_length_reference_[side] =
                runner.snapshot().step_request.leg_length[side];
            transfer_angle_reference_[side] =
                runner.snapshot().step_request.leg_angle_body[side];
        }
        switch (runner.snapshot().state_machine.step_task) {
        case BC_STEP_TASK_IMPACT_PASSIVE:
            phase_ = StepDockPhase::passive;
            break;
        case BC_STEP_TASK_TRANSFER:
            phase_ = StepDockPhase::transfer;
            break;
        case BC_STEP_TASK_TRANSFER_HOLD:
            phase_ = StepDockPhase::transfer_hold;
            break;
        case BC_STEP_TASK_RECOVER:
        case BC_STEP_TASK_RECOVER_LOCK:
            phase_ = StepDockPhase::rebalance;
            break;
        case BC_STEP_TASK_COMPLETE:
            phase_ = StepDockPhase::complete;
            observation_complete_ = true;
            return;
        case BC_STEP_TASK_RECOVERY_FAILED:
            fail("step_recovery_timeout");
            return;
        case BC_STEP_TASK_INACTIVE:
            if (runner.snapshot().step_command_rearm_required) {
                phase_ = StepDockPhase::complete;
                observation_complete_ = true;
                return;
            }
            break;
        case BC_STEP_TASK_PREPARE:
            break;
        }
        runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
        return;
    }

    if (phase_ == StepDockPhase::impact_delay) {
        command_.forward_velocity = static_cast<float>(spec_.target_velocity);
        if (runner.snapshot().state_machine.step_task ==
            BC_STEP_TASK_IMPACT_PASSIVE) {
            enter_passive(time, runner.snapshot());
        }
        runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
        return;
    }

    command_.forward_velocity = static_cast<float>(spec_.target_velocity);
    update_speed_window(time, velocity);
    if (runner.snapshot().state_machine.step_task ==
            BC_STEP_TASK_IMPACT_PASSIVE &&
        !std::isfinite(collision_time_)) {
        fail("impact_detected_before_contact");
        return;
    }
    if (contact.base_face && !contact.leg_face) {
        record_collision(
            time, sample.base.x, velocity, clearance, world_heading,
            contact, true);
        runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
        return;
    }
    if (contact.leg_face) {
        record_collision(
            time, sample.base.x, velocity, clearance, world_heading,
            contact, false);
        runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
        return;
    }
    if (sample.base.x > layout_.platform_end_x) {
        fail("platform_contact_not_detected");
        return;
    }
    runner.step_with_gimbal_heading(command_, 0.0F, 0.0F);
}

StepDockBenchmark::StepDockBenchmark(
    const std::filesystem::path &model_path,
    const std::filesystem::path &output_directory
) : output_directory_(output_directory),
    plant_(model_path, 0.001),
    adapter_(plant_.model()),
    sampler_(plant_.model()),
    summary_(output_directory_ / "summary.csv", {
        "case", "target_velocity", "leg_length", "platform_height",
        "cut_delay_command", "production_task",
        "base_platform_sliding_friction",
        "forward_acceleration_rate",
        "initial_heading_deg", "approach_heading_deg",
        "platform_gap_at_acceleration", "target_collision_travel",
        "require_speed_stable",
        "measurement_complete", "finite", "balance_engaged",
        "start_ready", "speed_stable", "contact_detected",
        "body_contact_before_trigger", "control_cut", "issue",
        "first_contact_pair", "collision_time", "collision_velocity",
        "collision_clearance", "collision_world_heading_deg",
        "acceleration_start_x", "collision_travel",
        "minimum_approach_clearance",
        "control_cut_delay", "trigger_contact_force",
        "strongest_contact_force",
        "platform_normal_impulse", "maximum_post_cut_actuation",
        "maximum_post_impact_joint_request",
        "maximum_recovery_wheel_request", "post_impact_joint_saturated",
        "recovery_reference_captured",
        "approach_horizontal_force_l", "approach_horizontal_force_r",
        "approach_horizontal_range_l", "approach_horizontal_range_r",
        "delay_peak_horizontal_residual",
        "horizontal_force_detection_latency", "maximum_delay_pitch_deg",
        "maximum_delay_pitch_rate", "base_contact_before_cut",
        "cut_base_velocity", "cut_pitch_deg", "cut_leg_length_l",
        "cut_leg_length_r",
        "retained_on_platform", "passively_supported", "final_settled",
        "final_base_top_contact_ratio", "final_wheel_top_contact_ratio",
        "hold_window_minimum_wheel_edge_margin",
        "hold_window_maximum_base_advance",
        "hold_window_maximum_abs_pitch_deg",
        "hold_window_both_wheel_top_contact_ratio",
        "final_base_x", "final_base_z", "final_pitch_deg",
        "final_roll_deg", "final_leg_length_l", "final_leg_length_r",
        "final_leg_angle_l_deg", "final_leg_angle_r_deg",
        "final_wheel_x_l", "final_wheel_x_r", "final_wheel_z_l",
        "final_wheel_z_r", "final_max_forward_speed",
        "final_max_vertical_speed", "final_max_pitch_rate",
        "final_max_roll_rate",
    }) {}

StepDockResult StepDockBenchmark::run(const StepDockSpec &spec) {
    plant_.set_contact_pair_sliding_friction(
        kBasePlatformContactPair,
        spec.base_platform_sliding_friction);
    plant_.reset();
    set_initial_heading(
        plant_.model(), plant_.data(), spec.initial_heading_radians);
    bc_controller_config_t config{};
    bc_controller_default_config(&config);
    if (!spec.production_task) {
        config.motion.leg_length = static_cast<float>(spec.leg_length);
    }
    config.motion.forward_reference.velocity_ramp.rate_limit =
        static_cast<float>(spec.forward_acceleration_rate);
    sim::SimulationRunner runner(plant_, adapter_, config);
    StepDockScenario scenario(spec, plant_.model());
    scenario.reset(plant_);

    StepDockResult result{};
    result.spec = spec;
    result.name = scenario.name();
    CsvWriter trace(output_directory_ / result.name / "trace.csv", {
        "case", "phase", "time", "cut_delay_command", "production_task",
        "base_platform_sliding_friction",
        "forward_acceleration_rate", "initial_heading_deg",
        "approach_heading_deg", "base_world_heading_deg",
        "platform_gap_at_acceleration", "target_collision_travel",
        "require_speed_stable", "step_task", "step_impact_armed",
        "step_impact_confirm_elapsed", "step_state_elapsed",
        "step_recovery_elapsed",
        "step_recovery_stable_elapsed", "step_rearm_required",
        "step_recovery_timed_out", "step_control_mode",
        "step_recovery_reference_capture",
        "step_position_heading_suppressed",
        "collision_elapsed", "control_cut", "command_velocity", "base_x",
        "base_z", "base_velocity", "base_vertical_velocity",
        "base_clearance", "pitch", "pitch_rate", "roll", "roll_rate",
        "yaw", "yaw_rate", "ref_s", "ref_ds", "ref_psi", "ref_dpsi",
        "leg_length_l", "leg_length_r",
        "leg_rate_l", "leg_rate_r", "leg_angle_l", "leg_angle_r",
        "transfer_length_ref_l", "transfer_length_ref_r",
        "transfer_angle_ref_l", "transfer_angle_ref_r",
        "wheel_x_l", "wheel_x_r", "wheel_z_l", "wheel_z_r",
        "platform_leg_face", "platform_wheel_face",
        "platform_other_leg_face", "platform_face_l", "platform_face_r",
        "platform_wheel_top_l", "platform_wheel_top_r",
        "platform_base_face", "platform_base_top", "contact_pair",
        "contact_normal_force", "contact_x", "contact_z",
        "axial_force_l", "axial_force_r", "leg_torque_l",
        "leg_torque_r", "support_vertical_raw_l",
        "support_vertical_raw_r", "support_vertical_filtered_l",
        "support_vertical_filtered_r", "tangential_force_l",
        "tangential_force_r", "tangential_horizontal_l",
        "tangential_horizontal_r", "horizontal_force_l",
        "horizontal_force_r",
        "wheel_request_l", "wheel_request_r", "wheel_applied_l",
        "wheel_applied_r", "joint_request_l_front",
        "joint_request_l_rear", "joint_request_r_front",
        "joint_request_r_rear", "joint_applied_l_front",
        "joint_applied_l_rear", "joint_applied_r_front",
        "joint_applied_r_rear", "joint_feedback_l_front",
        "joint_feedback_l_rear", "joint_feedback_r_front",
        "joint_feedback_r_rear", "joint_angle_l_front",
        "joint_angle_l_rear", "joint_angle_r_front",
        "joint_angle_r_rear", "joint_rate_l_front", "joint_rate_l_rear",
        "joint_rate_r_front", "joint_rate_r_rear", "support_phase",
        "impact_forward_acceleration", "impact_vertical_acceleration",
        "impact_5ms_valid", "impact_5ms_forward_dv",
        "impact_5ms_vertical_dv", "impact_5ms_pitch_rate_delta",
        "impact_5ms_leg_rate_delta_l", "impact_5ms_leg_rate_delta_r",
        "impact_5ms_wheel_velocity_delta",
        "impact_5ms_wheel_imu_mismatch",
        "impact_10ms_valid", "impact_10ms_forward_dv",
        "impact_10ms_vertical_dv", "impact_10ms_pitch_rate_delta",
        "impact_10ms_leg_rate_delta_l", "impact_10ms_leg_rate_delta_r",
        "impact_10ms_wheel_velocity_delta",
        "impact_10ms_wheel_imu_mismatch",
    });

    std::size_t final_samples = 0U;
    std::size_t final_top_contact_samples = 0U;
    std::size_t final_wheel_top_contact_samples = 0U;
    std::size_t hold_window_samples = 0U;
    std::size_t hold_window_both_wheel_top_samples = 0U;
    double transfer_hold_start_time =
        std::numeric_limits<double>::quiet_NaN();
    double transfer_hold_start_base_x =
        std::numeric_limits<double>::quiet_NaN();
    double hold_window_minimum_wheel_edge_margin =
        std::numeric_limits<double>::infinity();
    double hold_window_maximum_base_advance =
        -std::numeric_limits<double>::infinity();
    double horizontal_detection_hold_start =
        std::numeric_limits<double>::quiet_NaN();
    std::array<double, BC_SIDE_NUM> approach_horizontal_sum{};
    std::array<double, BC_SIDE_NUM> approach_horizontal_min{{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
    }};
    std::array<double, BC_SIDE_NUM> approach_horizontal_max{{
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
    }};
    std::size_t approach_horizontal_samples = 0U;
    bool cut_pose_captured = false;
    bool recovery_reference_checked = false;
    while (!scenario.finished()) {
        scenario.step(plant_, runner, sampler_);
        const SimulationSample sample = sampler_.read(
            plant_.data(), runner.snapshot());
        const StepDockContact contact = scenario.observe_contacts(
            plant_.data());
        write_trace(trace, scenario, sample, contact, runner.feedback());

        result.finite = result.finite &&
            controller_snapshot_is_finite(sample.controller) &&
            std::isfinite(scenario.base_clearance(plant_.data()));
        result.balance_engaged = result.balance_engaged ||
            scenario.balance_engaged();
        result.start_ready = result.start_ready || scenario.start_ready();
        result.speed_stable = result.speed_stable || scenario.speed_stable();
        result.contact_detected = result.contact_detected ||
            scenario.contact_detected();
        result.body_contact_before_trigger =
            result.body_contact_before_trigger ||
            scenario.body_contact_before_trigger();
        const HorizontalLegForce horizontal =
            horizontal_leg_force(sample.controller);
        if (scenario.phase() == StepDockPhase::transfer_hold) {
            if (!std::isfinite(transfer_hold_start_time)) {
                transfer_hold_start_time = sample.time;
                transfer_hold_start_base_x = sample.base.x;
            }
            const double hold_elapsed =
                sample.time - transfer_hold_start_time;
            if (hold_elapsed + 1.0e-12 >=
                    kTransferHoldWindowStartSeconds &&
                hold_elapsed <= kTransferHoldWindowEndSeconds + 1.0e-12) {
                ++hold_window_samples;
                if (contact.side_wheel_top[BC_L] &&
                    contact.side_wheel_top[BC_R]) {
                    ++hold_window_both_wheel_top_samples;
                }
                const double minimum_wheel_x = std::min(
                    scenario.wheel_axis_x(plant_.data(), BC_L),
                    scenario.wheel_axis_x(plant_.data(), BC_R));
                hold_window_minimum_wheel_edge_margin = std::min(
                    hold_window_minimum_wheel_edge_margin,
                    minimum_wheel_x - scenario.layout().edge_x);
                hold_window_maximum_base_advance = std::max(
                    hold_window_maximum_base_advance,
                    sample.base.x - transfer_hold_start_base_x);
                result.hold_window_maximum_abs_pitch = std::max(
                    result.hold_window_maximum_abs_pitch,
                    std::abs(static_cast<double>(sample.controller.
                        state.value[BC_STATE_THETA_B])));
            }
        }
        if (scenario.phase() == StepDockPhase::approach) {
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                approach_horizontal_sum[side] +=
                    horizontal.total_horizontal[side];
                approach_horizontal_min[side] = std::min(
                    approach_horizontal_min[side],
                    horizontal.total_horizontal[side]);
                approach_horizontal_max[side] = std::max(
                    approach_horizontal_max[side],
                    horizontal.total_horizontal[side]);
            }
            ++approach_horizontal_samples;
        }
        if (scenario.phase() == StepDockPhase::impact_delay) {
            double maximum_residual = 0.0;
            if (approach_horizontal_samples > 0U) {
                for (int side = 0; side < BC_SIDE_NUM; ++side) {
                    const double baseline =
                        approach_horizontal_sum[side] /
                        static_cast<double>(approach_horizontal_samples);
                    maximum_residual = std::max(
                        maximum_residual,
                        std::abs(horizontal.total_horizontal[side] -
                            baseline));
                }
            }
            result.delay_peak_horizontal_residual = std::max(
                result.delay_peak_horizontal_residual,
                maximum_residual);
            result.maximum_delay_pitch = std::max(
                result.maximum_delay_pitch,
                std::abs(static_cast<double>(sample.controller.
                    state.value[BC_STATE_THETA_B])));
            result.maximum_delay_pitch_rate = std::max(
                result.maximum_delay_pitch_rate,
                std::abs(static_cast<double>(sample.controller.
                    state.value[BC_STATE_DTHETA_B])));
            result.base_contact_before_cut =
                result.base_contact_before_cut || contact.base_face;
            if (maximum_residual >=
                kHorizontalDetectionThreshold) {
                if (!std::isfinite(horizontal_detection_hold_start)) {
                    horizontal_detection_hold_start = sample.time;
                }
                if (!std::isfinite(
                        result.horizontal_force_detection_latency) &&
                    sample.time - horizontal_detection_hold_start + 1.0e-12 >=
                        kHorizontalDetectionHoldSeconds) {
                    result.horizontal_force_detection_latency =
                        sample.time - scenario.collision_time();
                }
            } else {
                horizontal_detection_hold_start =
                    std::numeric_limits<double>::quiet_NaN();
            }
        }
        if (scenario.start_ready() &&
            scenario.phase() != StepDockPhase::impact_delay &&
            scenario.phase() != StepDockPhase::passive &&
            scenario.phase() != StepDockPhase::transfer &&
            scenario.phase() != StepDockPhase::transfer_hold &&
            scenario.phase() != StepDockPhase::rebalance &&
            scenario.phase() != StepDockPhase::complete) {
            result.minimum_approach_clearance = std::min(
                result.minimum_approach_clearance,
                scenario.base_clearance(plant_.data()));
        }
        if (scenario.phase() == StepDockPhase::passive ||
            scenario.phase() == StepDockPhase::transfer ||
            scenario.phase() == StepDockPhase::transfer_hold ||
            scenario.phase() == StepDockPhase::rebalance) {
            result.control_cut = true;
            if (!cut_pose_captured) {
                result.cut_base_velocity = sample.base.forward_velocity;
                result.cut_pitch =
                    sample.controller.state.value[BC_STATE_THETA_B];
                for (int side = 0; side < BC_SIDE_NUM; ++side) {
                    result.cut_leg_length[side] =
                        sample.controller.leg[side].length;
                }
                cut_pose_captured = true;
            }
            result.maximum_post_cut_actuation = std::max(
                result.maximum_post_cut_actuation,
                maximum_actuation(sample.controller));
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                const bool recovery_control =
                    sample.controller.state_machine.step_task ==
                        BC_STEP_TASK_RECOVER ||
                    sample.controller.state_machine.step_task ==
                        BC_STEP_TASK_RECOVER_LOCK;
                result.maximum_recovery_wheel_request = recovery_control ?
                        std::max(
                            result.maximum_recovery_wheel_request,
                            std::abs(static_cast<double>(sample.controller.
                                actuation_request.wheel_torque[side]))) :
                        result.maximum_recovery_wheel_request;
                for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                    const double request = std::abs(static_cast<double>(
                        sample.controller.actuation_request.leg[side].
                            joint_torque[joint]));
                    result.maximum_post_impact_joint_request = std::max(
                        result.maximum_post_impact_joint_request, request);
                    result.post_impact_joint_saturated =
                        result.post_impact_joint_saturated ||
                        request > config.control.joint_torque_limit +
                            kZeroTolerance;
                }
            }
            if (!recovery_reference_checked &&
                sample.controller.state_machine.step_task ==
                    BC_STEP_TASK_RECOVER_LOCK) {
                recovery_reference_checked = true;
                const auto &state = sample.controller.state;
                const auto &reference = sample.controller.state_reference;
                result.recovery_reference_captured =
                    std::abs(reference.value[BC_STATE_S] -
                        state.value[BC_STATE_S]) <= 0.005 &&
                    std::abs(reference.value[BC_STATE_DS]) <= 1.0e-7 &&
                    std::abs(bc_wrap_angle(
                        reference.value[BC_STATE_PSI] -
                        state.value[BC_STATE_PSI])) <= 0.001 &&
                    std::abs(reference.value[BC_STATE_DPSI]) <= 1.0e-7 &&
                    sample.controller.state_machine.forward ==
                        BC_FORWARD_HOLD &&
                    std::abs(sample.controller.
                        yaw_acceleration_reference) <= 1.0e-7;
            }
            result.strongest_contact_force = std::max(
                result.strongest_contact_force,
                contact.strongest_normal_force);
            result.platform_normal_impulse +=
                contact.total_normal_force * plant_.timestep();
            const bool final_window =
                sample.controller.state_machine.step_task ==
                    BC_STEP_TASK_RECOVER_LOCK &&
                sample.controller.step_recovery_stable_elapsed > 0.0F;
            if (final_window) {
                ++final_samples;
                if (contact.base_top) ++final_top_contact_samples;
                if (contact.side_wheel_top[BC_L] &&
                    contact.side_wheel_top[BC_R]) {
                    ++final_wheel_top_contact_samples;
                }
                result.final_maximum_forward_speed = std::max(
                    result.final_maximum_forward_speed,
                    std::abs(sample.base.forward_velocity));
                result.final_maximum_vertical_speed = std::max(
                    result.final_maximum_vertical_speed,
                    std::abs(sample.base.vertical_velocity));
                result.final_maximum_pitch_rate = std::max(
                    result.final_maximum_pitch_rate,
                    std::abs(static_cast<double>(sample.controller.
                        state.value[BC_STATE_DTHETA_B])));
                result.final_maximum_roll_rate = std::max(
                    result.final_maximum_roll_rate,
                    std::abs(static_cast<double>(sample.controller.roll_rate)));
            }
        } else if (scenario.phase() == StepDockPhase::impact_delay) {
            result.strongest_contact_force = std::max(
                result.strongest_contact_force,
                contact.strongest_normal_force);
            result.platform_normal_impulse +=
                contact.total_normal_force * plant_.timestep();
        }
        result.final_base_x = sample.base.x;
        result.final_base_z = sample.base.z;
        result.final_pitch = sample.controller.state.value[BC_STATE_THETA_B];
        result.final_roll = sample.controller.roll;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            result.final_leg_length[side] =
                sample.controller.leg[side].length;
            result.final_leg_angle[side] =
                sample.controller.leg[side].angle_body;
            result.final_wheel_x[side] =
                scenario.wheel_axis_x(plant_.data(), side);
            result.final_wheel_z[side] =
                scenario.wheel_axis_z(plant_.data(), side);
        }
        if (!result.finite) break;
    }
    trace.flush();

    result.measurement_complete = scenario.observation_complete() ||
        scenario.phase() == StepDockPhase::complete;
    result.issue = result.finite ? scenario.issue() : "non_finite_telemetry";
    result.first_contact_pair = scenario.first_contact_pair();
    result.collision_time = scenario.collision_time();
    result.collision_velocity = scenario.collision_velocity();
    result.collision_clearance = scenario.collision_clearance();
    result.collision_world_heading = scenario.collision_world_heading();
    result.acceleration_start_x = scenario.acceleration_start_x();
    result.collision_travel = scenario.collision_travel();
    result.trigger_contact_force = scenario.trigger_contact_force();
    result.control_cut_delay = result.control_cut ?
        scenario.control_cut_time() - scenario.collision_time() :
        std::numeric_limits<double>::quiet_NaN();
    if (approach_horizontal_samples > 0U) {
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            result.approach_horizontal_force[side] =
                approach_horizontal_sum[side] /
                static_cast<double>(approach_horizontal_samples);
            result.approach_horizontal_range[side] =
                approach_horizontal_max[side] - approach_horizontal_min[side];
        }
    }
    result.final_base_top_contact_ratio = final_samples > 0U ?
        static_cast<double>(final_top_contact_samples) /
            static_cast<double>(final_samples) : 0.0;
    result.final_wheel_top_contact_ratio = final_samples > 0U ?
        static_cast<double>(final_wheel_top_contact_samples) /
            static_cast<double>(final_samples) : 0.0;
    if (hold_window_samples > 0U) {
        result.hold_window_minimum_wheel_edge_margin =
            hold_window_minimum_wheel_edge_margin;
        result.hold_window_maximum_base_advance =
            hold_window_maximum_base_advance;
        result.hold_window_both_wheel_top_contact_ratio =
            static_cast<double>(hold_window_both_wheel_top_samples) /
            static_cast<double>(hold_window_samples);
    }
    result.retained_on_platform = final_samples > 0U &&
        (result.final_base_top_contact_ratio >= 0.90 ||
         (spec.production_task &&
          result.final_wheel_top_contact_ratio >= 0.90)) &&
        result.final_base_x >= scenario.layout().edge_x;
    result.final_settled = result.retained_on_platform &&
        std::abs(result.final_pitch) <= kPitchTolerance &&
        std::abs(result.final_roll) <= kRollTolerance &&
        result.final_maximum_forward_speed <= 0.10 &&
        result.final_maximum_vertical_speed <= 0.10 &&
        result.final_maximum_pitch_rate <= 0.20 &&
        result.final_maximum_roll_rate <= 0.20;
    result.passively_supported = false;
    if (!std::isfinite(result.minimum_approach_clearance)) {
        result.minimum_approach_clearance =
            std::numeric_limits<double>::quiet_NaN();
    }
    write_summary(result);
    return result;
}

void StepDockBenchmark::write_summary(const StepDockResult &result) {
    summary_.begin_row();
    summary_.value(result.name)
        .value(result.spec.target_velocity)
        .value(result.spec.leg_length)
        .value(result.spec.platform_height)
        .value(result.spec.cut_delay_seconds)
        .value(result.spec.production_task)
        .value(result.spec.base_platform_sliding_friction)
        .value(result.spec.forward_acceleration_rate)
        .value(result.spec.initial_heading_radians * 180.0 / BC_PI)
        .value(result.spec.approach_heading_radians * 180.0 / BC_PI)
        .value(result.spec.platform_gap_at_acceleration)
        .value(result.spec.target_collision_travel)
        .value(result.spec.require_speed_stable)
        .value(result.measurement_complete)
        .value(result.finite)
        .value(result.balance_engaged)
        .value(result.start_ready)
        .value(result.speed_stable)
        .value(result.contact_detected)
        .value(result.body_contact_before_trigger)
        .value(result.control_cut)
        .value(result.issue)
        .value(result.first_contact_pair)
        .value(result.collision_time)
        .value(result.collision_velocity)
        .value(result.collision_clearance)
        .value(result.collision_world_heading * 180.0 / BC_PI)
        .value(result.acceleration_start_x)
        .value(result.collision_travel)
        .value(result.minimum_approach_clearance)
        .value(result.control_cut_delay)
        .value(result.trigger_contact_force)
        .value(result.strongest_contact_force)
        .value(result.platform_normal_impulse)
        .value(result.maximum_post_cut_actuation)
        .value(result.maximum_post_impact_joint_request)
        .value(result.maximum_recovery_wheel_request)
        .value(result.post_impact_joint_saturated)
        .value(result.recovery_reference_captured)
        .value(result.approach_horizontal_force[BC_L])
        .value(result.approach_horizontal_force[BC_R])
        .value(result.approach_horizontal_range[BC_L])
        .value(result.approach_horizontal_range[BC_R])
        .value(result.delay_peak_horizontal_residual)
        .value(result.horizontal_force_detection_latency)
        .value(result.maximum_delay_pitch * 180.0 / BC_PI)
        .value(result.maximum_delay_pitch_rate)
        .value(result.base_contact_before_cut)
        .value(result.cut_base_velocity)
        .value(result.cut_pitch * 180.0 / BC_PI)
        .value(result.cut_leg_length[BC_L])
        .value(result.cut_leg_length[BC_R])
        .value(result.retained_on_platform)
        .value(result.passively_supported)
        .value(result.final_settled)
        .value(result.final_base_top_contact_ratio)
        .value(result.final_wheel_top_contact_ratio)
        .value(result.hold_window_minimum_wheel_edge_margin)
        .value(result.hold_window_maximum_base_advance)
        .value(result.hold_window_maximum_abs_pitch * 180.0 / BC_PI)
        .value(result.hold_window_both_wheel_top_contact_ratio)
        .value(result.final_base_x)
        .value(result.final_base_z)
        .value(result.final_pitch * 180.0 / BC_PI)
        .value(result.final_roll * 180.0 / BC_PI)
        .value(result.final_leg_length[BC_L])
        .value(result.final_leg_length[BC_R])
        .value(result.final_leg_angle[BC_L] * 180.0 / BC_PI)
        .value(result.final_leg_angle[BC_R] * 180.0 / BC_PI)
        .value(result.final_wheel_x[BC_L])
        .value(result.final_wheel_x[BC_R])
        .value(result.final_wheel_z[BC_L])
        .value(result.final_wheel_z[BC_R])
        .value(result.final_maximum_forward_speed)
        .value(result.final_maximum_vertical_speed)
        .value(result.final_maximum_pitch_rate)
        .value(result.final_maximum_roll_rate);
    summary_.end_row();
    summary_.flush();
}

void StepDockBenchmark::write_trace(
    CsvWriter &trace,
    const StepDockScenario &scenario,
    const SimulationSample &sample,
    const StepDockContact &contact,
    const bc_sensor_feedback_t &feedback
) const {
    const auto &snapshot = sample.controller;
    const HorizontalLegForce horizontal = horizontal_leg_force(snapshot);

    trace.begin_row();
    trace.value(scenario.name())
        .value(scenario.phase_name())
        .value(sample.time)
        .value(scenario.spec().cut_delay_seconds)
        .value(scenario.spec().production_task)
        .value(scenario.spec().base_platform_sliding_friction)
        .value(scenario.spec().forward_acceleration_rate)
        .value(scenario.spec().initial_heading_radians * 180.0 / BC_PI)
        .value(scenario.spec().approach_heading_radians * 180.0 / BC_PI)
        .value(scenario.base_world_heading(plant_.data()) * 180.0 / BC_PI)
        .value(scenario.spec().platform_gap_at_acceleration)
        .value(scenario.spec().target_collision_travel)
        .value(scenario.spec().require_speed_stable)
        .value(bc_step_task_state_name(snapshot.state_machine.step_task))
        .value(static_cast<int>(snapshot.step_impact_armed))
        .value(snapshot.step_impact_confirm_elapsed)
        .value(snapshot.step_state_elapsed)
        .value(snapshot.step_recovery_elapsed)
        .value(snapshot.step_recovery_stable_elapsed)
        .value(static_cast<int>(snapshot.step_command_rearm_required))
        .value(static_cast<int>(snapshot.step_recovery_timed_out))
        .value(static_cast<int>(snapshot.step_request.control_mode))
        .value(static_cast<int>(
            snapshot.step_request.recovery_reference_capture))
        .value(static_cast<int>(snapshot.step_request.
            suppress_position_heading_feedback))
        .value(scenario.collision_elapsed(sample.time))
        .value(scenario.control_cut())
        .value(scenario.commanded_velocity())
        .value(sample.base.x)
        .value(sample.base.z)
        .value(sample.base.forward_velocity)
        .value(sample.base.vertical_velocity)
        .value(scenario.base_clearance(plant_.data()))
        .value(snapshot.state.value[BC_STATE_THETA_B])
        .value(snapshot.state.value[BC_STATE_DTHETA_B])
        .value(snapshot.roll)
        .value(snapshot.roll_rate)
        .value(snapshot.state.value[BC_STATE_PSI])
        .value(snapshot.state.value[BC_STATE_DPSI])
        .value(snapshot.state_reference.value[BC_STATE_S])
        .value(snapshot.state_reference.value[BC_STATE_DS])
        .value(snapshot.state_reference.value[BC_STATE_PSI])
        .value(snapshot.state_reference.value[BC_STATE_DPSI])
        .value(snapshot.leg[BC_L].length)
        .value(snapshot.leg[BC_R].length)
        .value(snapshot.leg[BC_L].length_velocity)
        .value(snapshot.leg[BC_R].length_velocity)
        .value(snapshot.leg[BC_L].angle_body)
        .value(snapshot.leg[BC_R].angle_body)
        .value(scenario.transfer_length_reference()[BC_L])
        .value(scenario.transfer_length_reference()[BC_R])
        .value(scenario.transfer_angle_reference()[BC_L])
        .value(scenario.transfer_angle_reference()[BC_R])
        .value(scenario.wheel_axis_x(plant_.data(), BC_L))
        .value(scenario.wheel_axis_x(plant_.data(), BC_R))
        .value(scenario.wheel_axis_z(plant_.data(), BC_L))
        .value(scenario.wheel_axis_z(plant_.data(), BC_R))
        .value(contact.leg_face)
        .value(contact.wheel_face)
        .value(contact.other_leg_face)
        .value(contact.side_face[BC_L])
        .value(contact.side_face[BC_R])
        .value(contact.side_wheel_top[BC_L])
        .value(contact.side_wheel_top[BC_R])
        .value(contact.base_face)
        .value(contact.base_top)
        .value(contact.strongest_pair)
        .value(contact.strongest_normal_force)
        .value(contact.strongest_x)
        .value(contact.strongest_z)
        .value(snapshot.support_force[BC_L].axial_force)
        .value(snapshot.support_force[BC_R].axial_force)
        .value(snapshot.support_force[BC_L].leg_torque)
        .value(snapshot.support_force[BC_R].leg_torque)
        .value(snapshot.support_force[BC_L].vertical_force)
        .value(snapshot.support_force[BC_R].vertical_force)
        .value(snapshot.support_force[BC_L].filtered_vertical_force)
        .value(snapshot.support_force[BC_R].filtered_vertical_force)
        .value(horizontal.tangential[BC_L])
        .value(horizontal.tangential[BC_R])
        .value(horizontal.tangential_horizontal[BC_L])
        .value(horizontal.tangential_horizontal[BC_R])
        .value(horizontal.total_horizontal[BC_L])
        .value(horizontal.total_horizontal[BC_R])
        .value(snapshot.actuation_request.wheel_torque[BC_L])
        .value(snapshot.actuation_request.wheel_torque[BC_R])
        .value(snapshot.actuation.wheel_torque[BC_L])
        .value(snapshot.actuation.wheel_torque[BC_R])
        .value(snapshot.actuation_request.leg[BC_L].joint_torque[BC_FRONT])
        .value(snapshot.actuation_request.leg[BC_L].joint_torque[BC_REAR])
        .value(snapshot.actuation_request.leg[BC_R].joint_torque[BC_FRONT])
        .value(snapshot.actuation_request.leg[BC_R].joint_torque[BC_REAR])
        .value(snapshot.actuation.leg[BC_L].joint_torque[BC_FRONT])
        .value(snapshot.actuation.leg[BC_L].joint_torque[BC_REAR])
        .value(snapshot.actuation.leg[BC_R].joint_torque[BC_FRONT])
        .value(snapshot.actuation.leg[BC_R].joint_torque[BC_REAR]);
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            trace.value(feedback.leg[side].joint[joint].torque);
        }
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            trace.value(feedback.leg[side].joint[joint].angle);
        }
    }
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
            trace.value(feedback.leg[side].joint[joint].angular_velocity);
        }
    }
    trace.value(bc_support_phase_state_name(
        snapshot.state_machine.support));
    const bc_impact_observer_output_t &impact = snapshot.impact_observer;
    trace.value(impact.forward_acceleration)
        .value(impact.vertical_acceleration);
    for (int window = 0; window < BC_IMPACT_WINDOW_NUM; ++window) {
        const bc_impact_window_output_t &output = impact.window[window];
        trace.value(static_cast<int>(output.valid))
            .value(output.forward_delta_velocity)
            .value(output.vertical_delta_velocity)
            .value(output.pitch_rate_delta)
            .value(output.leg_rate_delta[BC_L])
            .value(output.leg_rate_delta[BC_R])
            .value(output.wheel_velocity_delta)
            .value(output.wheel_imu_delta_mismatch);
    }
    trace.end_row();
}

} // namespace balance::benchmark
