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

} // namespace balance::sim
