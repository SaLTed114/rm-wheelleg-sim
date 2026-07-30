#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>

#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: mujoco_smoke_test <model.xml>\n";
        return EXIT_FAILURE;
    }

    try {
        constexpr double kTimestepSeconds = 0.001;
        constexpr double kDurationSeconds = 0.010;
        balance::sim::MujocoPlant plant(
            std::filesystem::path(argv[1]), kTimestepSeconds);

        const bool unexpected_model_dimensions =
            plant.model().nq != 14 ||
            plant.model().nv != 14 ||
            plant.model().nu != 6;
        if (unexpected_model_dimensions) {
            std::cerr << "unexpected model dimensions: nq=" << plant.model().nq
                      << " nv=" << plant.model().nv
                      << " nu=" << plant.model().nu << '\n';
            return EXIT_FAILURE;
        }

        balance::sim::MujocoAdapter adapter(plant.model());
        bc_actuation_t mapped_actuation{};
        mapped_actuation.leg[BC_L].joint_torque[BC_FRONT] = 1.0F;
        mapped_actuation.leg[BC_L].joint_torque[BC_REAR] = 2.0F;
        mapped_actuation.wheel_torque[BC_L] = 3.0F;
        mapped_actuation.leg[BC_R].joint_torque[BC_FRONT] = 4.0F;
        mapped_actuation.leg[BC_R].joint_torque[BC_REAR] = 5.0F;
        mapped_actuation.wheel_torque[BC_R] = 6.0F;
        adapter.write(plant.data(), mapped_actuation);

        const char *actuator_names[] = {
            "Left_front_joint_actuator",
            "Left_rear_joint_actuator",
            "Left_Wheel_joint_actuator",
            "Right_front_joint_actuator",
            "Right_rear_joint_actuator",
            "Right_Wheel_joint_actuator",
        };
        for (int index = 0; index < 6; ++index) {
            const int actuator = mj_name2id(
                &plant.model(), mjOBJ_ACTUATOR, actuator_names[index]);
            if (plant.data().ctrl[actuator] != index + 1.0) {
                std::cerr << "incorrect actuator mapping at "
                          << actuator_names[index] << '\n';
                return EXIT_FAILURE;
            }
        }

        balance::sim::SimulationRunner runner(plant, adapter);
        runner.reset();
        const auto stats = runner.run_for(kDurationSeconds);

        if (stats.physics_steps != 10 || stats.control_ticks != 10) {
            std::cerr << "unexpected step counts\n";
            return EXIT_FAILURE;
        }
        if (std::abs(stats.final_time - kDurationSeconds) > 1.0e-12) {
            std::cerr << "unexpected final time: " << stats.final_time << '\n';
            return EXIT_FAILURE;
        }
        for (int actuator = 0; actuator < plant.model().nu; ++actuator) {
            if (plant.data().ctrl[actuator] != 0.0) {
                std::cerr << "actuator " << actuator << " was not zero\n";
                return EXIT_FAILURE;
            }
        }
    } catch (const std::exception &error) {
        std::cerr << "mujoco_smoke_test: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
