#ifndef BALANCE_SIM_MUJOCO_PLANT_HPP
#define BALANCE_SIM_MUJOCO_PLANT_HPP

#include <filesystem>
#include <memory>

#include <mujoco/mujoco.h>

namespace balance::sim {

class MujocoPlant {
public:
    MujocoPlant(
        const std::filesystem::path &model_path, double timestep_seconds);

    MujocoPlant(const MujocoPlant &) = delete;
    MujocoPlant &operator=(const MujocoPlant &) = delete;

    void reset();
    void step();

    [[nodiscard]] const mjModel &model() const noexcept { return *model_; }
    [[nodiscard]] mjData &data() noexcept { return *data_; }
    [[nodiscard]] const mjData &data() const noexcept { return *data_; }
    [[nodiscard]] double timestep() const noexcept {
        return model_->opt.timestep;
    }

private:
    struct ModelDeleter {
        void operator()(mjModel *model) const noexcept;
    };

    struct DataDeleter {
        void operator()(mjData *data) const noexcept;
    };

    std::unique_ptr<mjModel, ModelDeleter> model_;
    std::unique_ptr<mjData, DataDeleter> data_;
};

} // namespace balance::sim

#endif
