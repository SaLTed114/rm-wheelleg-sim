#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "balance_cpp/math.hpp"
#include "cpp/simulation_runner.hpp"

namespace {

int require_id(const mjModel &model, const mjtObj type, const char *name) {
    const int id = mj_name2id(&model, type, name);
    if (id < 0) {
        throw std::runtime_error("missing MuJoCo object '" +
                                 std::string(name) + "'");
    }
    return id;
}

bool any_non_wheel_contact(
    const mjData &data,
    const int ground,
    const std::array<int, 2> &wheel
) {
    for (int index = 0; index < data.ncon; ++index) {
        const auto &contact = data.contact[index];
        const bool has_ground =
            contact.geom[0] == ground || contact.geom[1] == ground;
        bool wheel_pair = false;
        for (const int wheel_geom : wheel) {
            wheel_pair = wheel_pair ||
                (contact.geom[0] == ground &&
                 contact.geom[1] == wheel_geom) ||
                (contact.geom[1] == ground &&
                 contact.geom[0] == wheel_geom);
        }
        if (!has_ground || !wheel_pair) return true;
    }
    return false;
}

bool both_wheels_contact(
    const mjData &data,
    const int ground,
    const std::array<int, 2> &wheel
) {
    std::array<bool, 2> contact{};
    for (int index = 0; index < data.ncon; ++index) {
        const auto &pair = data.contact[index];
        for (std::size_t side = 0; side < wheel.size(); ++side) {
            contact[side] = contact[side] ||
                (pair.geom[0] == ground && pair.geom[1] == wheel[side]) ||
                (pair.geom[1] == ground && pair.geom[0] == wheel[side]);
        }
    }
    return contact[0] && contact[1];
}

} // namespace

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: mujoco_cpp_startup_test <model.xml>\n";
        return EXIT_FAILURE;
    }
    try {
        balance::sim::MujocoPlant plant(
            std::filesystem::path(argv[1]), 0.001);
        balance::sim::MujocoAdapter adapter(plant.model());
        balance::sim::cpp::SimulationRunner runner(plant, adapter);
        const int ground = require_id(plant.model(), mjOBJ_GEOM, "ground");
        const std::array<int, 2> wheel{{
            require_id(
                plant.model(), mjOBJ_GEOM, "Right_wheel_collision"),
            require_id(
                plant.model(), mjOBJ_GEOM, "Left_wheel_collision"),
        }};

        runner.reset();
        balance::control::OperatorCommand command{};
        while (plant.data().time < 8.0 &&
               runner.snapshot().motion_state !=
                   balance::control::MotionState::active) {
            command.system_enabled = plant.data().time >= 2.0;
            command.balance_restart = command.system_enabled &&
                runner.snapshot().motion_state ==
                    balance::control::MotionState::idle;
            runner.step(command);
        }
        if (runner.snapshot().motion_state !=
            balance::control::MotionState::active) {
            std::cerr << "C++ controller did not reach active hold\n";
            return EXIT_FAILURE;
        }

        const double end_time = plant.data().time + 8.0;
        const double evaluation_start = end_time - 3.0;
        int samples = 0;
        int wheel_contact_samples = 0;
        int other_contact_samples = 0;
        double maximum_pitch = 0.0;
        double maximum_pitch_rate = 0.0;
        double maximum_leg_difference = 0.0;
        bool finite = true;
        command.balance_restart = false;
        while (plant.data().time < end_time) {
            runner.step(command);
            if (plant.data().time < evaluation_start) continue;
            ++samples;
            const auto &snapshot = runner.snapshot();
            maximum_pitch = std::max(maximum_pitch, std::abs(
                static_cast<double>(snapshot.state[
                    balance::control::StateIndex::pitch])));
            maximum_pitch_rate = std::max(maximum_pitch_rate, std::abs(
                static_cast<double>(snapshot.state[
                    balance::control::StateIndex::pitch_rate])));
            maximum_leg_difference = std::max(
                maximum_leg_difference,
                std::abs(static_cast<double>(
                    snapshot.leg[0].length - snapshot.leg[1].length)));
            if (both_wheels_contact(plant.data(), ground, wheel)) {
                ++wheel_contact_samples;
            }
            if (any_non_wheel_contact(plant.data(), ground, wheel)) {
                ++other_contact_samples;
            }
            for (const float value : snapshot.state.value) {
                finite = finite && std::isfinite(value);
            }
            for (const auto &leg : snapshot.actuation.leg) {
                for (const float torque : leg.joint_torque) {
                    finite = finite && std::isfinite(torque);
                }
            }
        }
        const double contact_ratio =
            static_cast<double>(wheel_contact_samples) / samples;
        std::cout << "C++ startup: pitch="
                  << maximum_pitch * 180.0 /
                         static_cast<double>(balance::control::math::pi)
                  << " deg, pitch_rate=" << maximum_pitch_rate
                  << ", contact=" << contact_ratio
                  << ", leg_delta_mm=" << 1000.0 * maximum_leg_difference
                  << ", other_contact=" << other_contact_samples << '\n';
        const bool standing = finite &&
            maximum_pitch < static_cast<double>(
                balance::control::math::radians(5.0F)) &&
            maximum_pitch_rate < 0.2 &&
            maximum_leg_difference <= 0.002 &&
            contact_ratio >= 0.99 && other_contact_samples == 0;
        return standing ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
