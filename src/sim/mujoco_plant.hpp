#ifndef BALANCE_SIM_MUJOCO_PLANT_HPP
#define BALANCE_SIM_MUJOCO_PLANT_HPP

#include <filesystem>
#include <memory>

#include <mujoco/mujoco.h>

namespace balance::sim {

struct RampCourseLayout {
    double ascent_start_x{};
    double ascent_end_x{};
    double platform_end_x{};
    double descent_end_x{};
    double height{};
};

struct RampJumpLayout {
    double start_x{};
    double lip_x{};
    double height{};
    double angle_radians{};
};

struct StepDockLayout {
    double edge_x{};
    double platform_end_x{};
    double ground_z{};
    double top_z{};
    double height{};
};

class MujocoPlant {
public:
    MujocoPlant(
        const std::filesystem::path &model_path, double timestep_seconds);

    MujocoPlant(const MujocoPlant &) = delete;
    MujocoPlant &operator=(const MujocoPlant &) = delete;

    void reset();
    void step();
    void set_equality_active(const char *name, bool active);
    void place_mocap_surface(
        const char *name, double x, double y, double z, bool visible);
    void place_mocap_surface(
        const char *name, double x, double y, double z,
        double pitch_radians, bool visible);
    void configure_keyboard_course();
    [[nodiscard]] StepDockLayout configure_step_dock_benchmark(
        double edge_x = 4.0);
    void configure_ramp_climb_benchmark();
    [[nodiscard]] RampJumpLayout configure_ramp_jump_benchmark();
    [[nodiscard]] RampCourseLayout configure_ramp_course_benchmark(
        bool beveled_transition = false);

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
