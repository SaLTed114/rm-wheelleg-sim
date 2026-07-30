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
            plant.model().nq != 21 ||
            plant.model().nv != 20 ||
            plant.model().nu != 6;
        if (unexpected_model_dimensions) {
            std::cerr << "unexpected model dimensions: nq=" << plant.model().nq
                      << " nv=" << plant.model().nv
                      << " nu=" << plant.model().nu << '\n';
            return EXIT_FAILURE;
        }

        balance::sim::MujocoAdapter adapter(plant.model());
        bc_sensor_feedback_t feedback{};
        adapter.read(plant.data(), feedback);
        const double expected_joint_angles[BC_SIDE_NUM][BC_JOINT_NUM] = {
            {-2.932150759729568, -0.067812378106530},
            {-3.030735772282508, -0.032996075602418},
        };
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                const double error = std::abs(
                    feedback.leg[side].joint[joint].angle -
                    expected_joint_angles[side][joint]);
                if (error > 1.0e-6) {
                    std::cerr << "incorrect joint feedback mapping at "
                              << side << ", " << joint << '\n';
                    return EXIT_FAILURE;
                }
            }
        }

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
        const double expected_controls[] = {
            4.0, -5.0, 6.0, -1.0, 2.0, -3.0,
        };
        for (int index = 0; index < 6; ++index) {
            const int actuator = mj_name2id(
                &plant.model(), mjOBJ_ACTUATOR, actuator_names[index]);
            if (plant.data().ctrl[actuator] != expected_controls[index]) {
                std::cerr << "incorrect actuator mapping at "
                          << actuator_names[index] << '\n';
                return EXIT_FAILURE;
            }
        }

        const char *joint_names[] = {
            "Left_front_joint",
            "Left_rear_joint",
            "Right_front_joint",
            "Right_rear_joint",
        };
        const double raw_velocities[] = {0.5, -0.75, 0.25, -0.4};
        for (int index = 0; index < 4; ++index) {
            const int joint = mj_name2id(
                &plant.model(), mjOBJ_JOINT, joint_names[index]);
            const int dof = plant.model().jnt_dofadr[joint];
            plant.data().qvel[dof] = raw_velocities[index];
        }
        adapter.read(plant.data(), feedback);

        const int left_wheel = mj_name2id(
            &plant.model(), mjOBJ_JOINT, "Left_Wheel_joint");
        const int right_wheel = mj_name2id(
            &plant.model(), mjOBJ_JOINT, "Right_Wheel_joint");
        plant.data().qvel[plant.model().jnt_dofadr[left_wheel]] = 2.0;
        plant.data().qvel[plant.model().jnt_dofadr[right_wheel]] = -2.0;
        adapter.read(plant.data(), feedback);
        if (feedback.wheel[BC_L].angular_velocity != 2.0F ||
            feedback.wheel[BC_R].angular_velocity != 2.0F) {
            std::cerr << "forward wheel velocity mapping is incorrect\n";
            return EXIT_FAILURE;
        }

        double raw_power = 0.0;
        for (int index = 0; index < 4; ++index) {
            const int actuator = mj_name2id(
                &plant.model(), mjOBJ_ACTUATOR, actuator_names[
                    index < 2 ? index : index + 1]);
            raw_power += plant.data().ctrl[actuator] *
                raw_velocities[index];
        }
        double control_power = 0.0;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                control_power +=
                    mapped_actuation.leg[side].joint_torque[joint] *
                    feedback.leg[side].joint[joint].angular_velocity;
            }
        }
        if (std::abs(raw_power - control_power) > 1.0e-6) {
            std::cerr << "joint mapping does not preserve power\n";
            return EXIT_FAILURE;
        }

        const int base_joint = mj_name2id(
            &plant.model(), mjOBJ_JOINT, "base_free_joint");
        const int base_qpos = plant.model().jnt_qposadr[base_joint];
        const int base_dof = plant.model().jnt_dofadr[base_joint];
        const double pitch_axis[] = {0.0, 1.0, 0.0};
        mju_axisAngle2Quat(
            plant.data().qpos + base_qpos + 3, pitch_axis, 0.2);
        plant.data().qvel[base_dof + 4] = 0.7;
        mj_forward(&plant.model(), &plant.data());
        adapter.read(plant.data(), feedback);
        if (std::abs(feedback.imu.pitch - 0.2F) > 1.0e-6F ||
            std::abs(feedback.imu.pitch_rate - 0.7F) > 1.0e-6F) {
            std::cerr << "positive pitch mapping is incorrect\n";
            return EXIT_FAILURE;
        }

        plant.reset();
        const double yaw_axis[] = {0.0, 0.0, 1.0};
        mju_axisAngle2Quat(
            plant.data().qpos + base_qpos + 3, yaw_axis, 0.3);
        plant.data().qvel[base_dof + 5] = 0.8;
        mj_forward(&plant.model(), &plant.data());
        adapter.read(plant.data(), feedback);
        if (std::abs(feedback.imu.yaw - 0.3F) > 1.0e-6F ||
            std::abs(feedback.imu.yaw_rate - 0.8F) > 1.0e-6F) {
            std::cerr << "positive yaw mapping is incorrect\n";
            return EXIT_FAILURE;
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
