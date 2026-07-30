#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "balance/math_utils.h"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "mujoco_viewer.hpp"
#include "simulation_runner.hpp"

namespace {

constexpr double kPhaseDuration = 4.0;
constexpr double kSettleDuration = 3.0;
constexpr int kPhaseCount = 5;

struct DemoTarget {
    bc_operator_command_t command;
    std::array<double, 3> support_position;
    std::array<double, 4> support_quaternion;
    const char *phase;
};

DemoTarget make_demo_target(double simulation_time) {
    constexpr float kLengthCenter = 0.34F;
    constexpr float kAngleCenter = -0.5F * BC_PI_F;
    constexpr float kAngleAmplitude = 15.0F * BC_PI_F / 180.0F;
    constexpr double kTravelAmplitude = 0.08;
    constexpr double kPitchAmplitude = 10.0 * BC_PI / 180.0;
    constexpr double kYawAmplitude = 15.0 * BC_PI / 180.0;

    const double cycle_time = std::fmod(
        simulation_time,
        kSettleDuration + kPhaseCount * kPhaseDuration);
    const bool settling = cycle_time < kSettleDuration;
    const double excitation_time = settling
        ? 0.0 : cycle_time - kSettleDuration;
    const int phase = static_cast<int>(
        excitation_time / kPhaseDuration);
    const double phase_time = excitation_time - phase * kPhaseDuration;
    const double wave = std::sin(
        2.0 * BC_PI * phase_time / kPhaseDuration);

    DemoTarget target{};
    target.command.enabled = 1U;
    target.support_quaternion = {1.0, 0.0, 0.0, 0.0};
    for (int side = 0; side < BC_SIDE_NUM; ++side) {
        target.command.leg[side].length = kLengthCenter;
        target.command.leg[side].angle_body = kAngleCenter;
    }

    if (settling) {
        target.phase = "settling on ground";
        return target;
    }

    const double pitch_axis[] = {0.0, 1.0, 0.0};
    const double yaw_axis[] = {0.0, 0.0, 1.0};
    switch (phase) {
    case 0:
        target.phase = "forward odometry";
        target.support_position[0] = kTravelAmplitude * wave;
        break;
    case 1:
        target.phase = "positive pitch";
        mju_axisAngle2Quat(
            target.support_quaternion.data(), pitch_axis,
            kPitchAmplitude * wave);
        break;
    case 2:
        target.phase = "positive yaw";
        mju_axisAngle2Quat(
            target.support_quaternion.data(), yaw_axis,
            kYawAmplitude * wave);
        break;
    case 3:
        target.phase = "physical left leg";
        target.command.leg[BC_L].angle_body +=
            kAngleAmplitude * static_cast<float>(wave);
        break;
    default:
        target.phase = "physical right leg";
        target.command.leg[BC_R].angle_body +=
            kAngleAmplitude * static_cast<float>(wave);
        break;
    }
    return target;
}

int require_mocap_id(const mjModel &model, const char *body_name) {
    const int body = mj_name2id(&model, mjOBJ_BODY, body_name);
    if (body < 0 || model.body_mocapid[body] < 0) {
        throw std::runtime_error(
            "missing mocap support body '" + std::string(body_name) + "'");
    }
    return model.body_mocapid[body];
}

void apply_demo_target(
    mjData &data, const int mocap, const DemoTarget &target
) {
    std::copy(
        target.support_position.begin(), target.support_position.end(),
        data.mocap_pos + 3 * mocap);
    std::copy(
        target.support_quaternion.begin(), target.support_quaternion.end(),
        data.mocap_quat + 4 * mocap);
}

void print_state(const char *phase, const bc_state_vector_t &state) {
    std::cout << phase << ':';
    for (int index = 0; index < BC_STATE_NUM; ++index) {
        std::cout << ' ' << state.value[index];
    }
    std::cout << '\n';
}

} // namespace

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: rm_balance_sim <model.xml>\n";
        return EXIT_FAILURE;
    }

    try {
        constexpr double kTimestepSeconds = 0.001;
        constexpr double kMaxFrameTimeSeconds = 0.1;

        balance::sim::MujocoPlant plant(
            std::filesystem::path(argv[1]), kTimestepSeconds);
        balance::sim::MujocoAdapter adapter(plant.model());
        balance::sim::SimulationRunner runner(plant, adapter);
        balance::sim::MujocoViewer viewer(plant.model());
        const int support_mocap = require_mocap_id(
            plant.model(), "base_support");
        runner.reset();
        auto target = make_demo_target(plant.data().time);
        apply_demo_target(plant.data(), support_mocap, target);
        runner.set_command(target.command);

        using Clock = std::chrono::steady_clock;
        auto previous_time = Clock::now();
        double accumulated_time = 0.0;
        double next_state_print = 0.0;

        while (!viewer.should_close()) {
            const auto current_time = Clock::now();
            const std::chrono::duration<double> frame_time =
                current_time - previous_time;
            previous_time = current_time;

            if (viewer.consume_reset_request()) {
                runner.reset();
                target = make_demo_target(plant.data().time);
                apply_demo_target(plant.data(), support_mocap, target);
                runner.set_command(target.command);
                accumulated_time = 0.0;
                next_state_print = 0.0;
            }

            if (viewer.paused()) {
                accumulated_time = 0.0;
            } else {
                accumulated_time += std::clamp(
                    frame_time.count(), 0.0, kMaxFrameTimeSeconds);
                while (accumulated_time >= plant.timestep()) {
                    target = make_demo_target(plant.data().time);
                    apply_demo_target(plant.data(), support_mocap, target);
                    runner.set_command(target.command);
                    runner.step();
                    accumulated_time -= plant.timestep();
                }
            }

            if (plant.data().time >= next_state_print) {
                print_state(target.phase, runner.state());
                next_state_print += 0.5;
            }

            viewer.render(plant.data());
            viewer.poll_events();
        }
    } catch (const std::exception &error) {
        std::cerr << "rm_balance_sim: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
