#include "mujoco_plant.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace balance::sim {

void MujocoPlant::ModelDeleter::operator()(mjModel *model) const noexcept {
    mj_deleteModel(model);
}

void MujocoPlant::DataDeleter::operator()(mjData *data) const noexcept {
    mj_deleteData(data);
}

MujocoPlant::MujocoPlant(
    const std::filesystem::path &model_path, const double timestep_seconds
) {
    if (timestep_seconds <= 0.0) {
        throw std::invalid_argument("MuJoCo timestep must be positive");
    }

    std::array<char, 1024> error{};
    model_.reset(mj_loadXML(
        model_path.string().c_str(), nullptr, error.data(),
        static_cast<int>(error.size())));
    if (!model_) {
        throw std::runtime_error(
            "failed to load MuJoCo model '" + model_path.string() +
            "': " + error.data());
    }

    model_->opt.timestep = timestep_seconds;
    data_.reset(mj_makeData(model_.get()));
    if (!data_) {
        throw std::runtime_error("failed to allocate MuJoCo data");
    }

    reset();
}

void MujocoPlant::reset() {
    mj_resetData(model_.get(), data_.get());
    mj_forward(model_.get(), data_.get());
}

void MujocoPlant::step() {
    mj_step(model_.get(), data_.get());
}

void MujocoPlant::set_equality_active(
    const char *name, const bool active
) {
    const int equality = mj_name2id(model_.get(), mjOBJ_EQUALITY, name);
    if (equality < 0) {
        throw std::runtime_error(
            "MuJoCo model is missing equality '" +
            std::string(name) + "'");
    }
    data_->eq_active[equality] = active ? 1 : 0;
}

void MujocoPlant::place_mocap_surface(
    const char *name, const double x, const double y, const double z,
    const bool visible
) {
    place_mocap_surface(name, x, y, z, 0.0, visible);
}

void MujocoPlant::place_mocap_surface(
    const char *name, const double x, const double y, const double z,
    const double pitch_radians, const bool visible
) {
    const int geom = mj_name2id(model_.get(), mjOBJ_GEOM, name);
    if (geom < 0) {
        throw std::runtime_error(
            "MuJoCo model is missing geom '" +
            std::string(name) + "'");
    }
    const int body = model_->geom_bodyid[geom];
    const int mocap = model_->body_mocapid[body];
    if (mocap < 0) {
        throw std::runtime_error(
            "MuJoCo surface '" + std::string(name) +
            "' is not attached to a mocap body");
    }
    data_->mocap_pos[3 * mocap] = x;
    data_->mocap_pos[3 * mocap + 1] = y;
    data_->mocap_pos[3 * mocap + 2] = z;
    data_->mocap_quat[4 * mocap] = std::cos(0.5 * pitch_radians);
    data_->mocap_quat[4 * mocap + 1] = 0.0;
    data_->mocap_quat[4 * mocap + 2] = std::sin(0.5 * pitch_radians);
    data_->mocap_quat[4 * mocap + 3] = 0.0;
    model_->geom_rgba[4 * geom + 3] = visible ? 1.0F : 0.0F;
    mj_forward(model_.get(), data_.get());
}

void MujocoPlant::configure_keyboard_course() {
    constexpr double kGroundHeight = -0.43;
    constexpr double kRampStartX = 2.0;
    constexpr double kPi = 3.14159265358979323846;

    const auto place_wedge = [this](
        const char *ramp_name,
        const double lane_y,
        const double height,
        const double angle_degrees
    ) {
        const double angle = angle_degrees * kPi / 180.0;
        const double run = height / std::tan(angle);
        place_mocap_surface(
            ramp_name, kRampStartX + 0.5 * run, lane_y,
            kGroundHeight + 0.5 * height, true);
        return kRampStartX + run;
    };

    const double platform_start = place_wedge(
        "keyboard_ramp_15deg", -1.5, 0.2, 15.0);
    place_mocap_surface(
        "keyboard_platform_200mm", platform_start + 1.0,
        -1.5, kGroundHeight + 0.1, true);
    place_wedge("keyboard_ramp_17deg", 1.0, 0.35, 17.0);
}

void MujocoPlant::configure_ramp_climb_benchmark() {
    constexpr double kGroundHeight = -0.43;
    constexpr double kRampStartX = 1.5;
    constexpr double kHeight = 0.35;
    constexpr double kAngle = 17.0 * 3.14159265358979323846 / 180.0;
    const double run = kHeight / std::tan(kAngle);
    place_mocap_surface(
        "benchmark_ramp_17deg",
        kRampStartX + 0.5 * run, 0.0,
        kGroundHeight + 0.5 * kHeight, true);
}

} // namespace balance::sim
