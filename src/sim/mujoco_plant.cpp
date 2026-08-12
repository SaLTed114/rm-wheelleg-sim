#include "mujoco_plant.hpp"

#include <array>
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
    model_->geom_rgba[4 * geom + 3] = visible ? 1.0F : 0.0F;
    mj_forward(model_.get(), data_.get());
}

} // namespace balance::sim
