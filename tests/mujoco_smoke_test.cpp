#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>

#include "balance/math_utils.h"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "performance/performance_scenario.hpp"
#include "simulation_runner.hpp"

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: mujoco_smoke_test <model.xml>\n";
        return EXIT_FAILURE;
    }

    try {
        const auto &yaw_cases = balance::benchmark::yaw_acceleration_cases();
        const auto *yaw_case = balance::benchmark::find_performance_case(
            "yaw_pos_2pi_a7p5");
        const auto *yaw_speed_case =
            balance::benchmark::find_performance_case("yaw_pos_4pi");
        if (yaw_cases.size() != 14U || yaw_case == nullptr ||
            yaw_case->axis != balance::benchmark::PerformanceAxis::yaw ||
            std::abs(yaw_case->target - 2.0 * BC_PI) > 1.0e-12 ||
            std::abs(yaw_case->command_rate - 7.5) > 1.0e-12 ||
            yaw_speed_case == nullptr ||
            std::abs(yaw_speed_case->target - 4.0 * BC_PI) > 1.0e-12 ||
            std::abs(yaw_speed_case->command_rate - 5.0) > 1.0e-12) {
            std::cerr << "yaw performance cases are incorrect\n";
            return EXIT_FAILURE;
        }

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

        const int base_body = mj_name2id(
            &plant.model(), mjOBJ_BODY, "base_link");
        const int support_weld = mj_name2id(
            &plant.model(), mjOBJ_EQUALITY, "base_support_weld");
        const int imu_site = mj_name2id(
            &plant.model(), mjOBJ_SITE, "imu_site");
        if (imu_site < 0) {
            std::cerr << "IMU site is missing\n";
            return EXIT_FAILURE;
        }
        const double expected_base_com[] = {
            -0.019917, -0.00040396, 0.021412,
        };
        const double expected_base_inertia[] = {
            2.8640678, 2.8736324, 3.0472,
        };
        bool invalid_base_properties =
            std::abs(plant.model().body_mass[base_body] - 11.0) > 1.0e-9 ||
            plant.model().eq_active0[support_weld] != 0;
        for (int axis = 0; axis < 3; ++axis) {
            invalid_base_properties = invalid_base_properties ||
                std::abs(
                    plant.model().body_ipos[3 * base_body + axis] -
                    expected_base_com[axis]) > 1.0e-9 ||
                std::abs(
                    plant.model().body_inertia[3 * base_body + axis] -
                    expected_base_inertia[axis]) > 1.0e-7;
        }
        const double expected_imu_position[] = {-0.10, 0.0, -0.03};
        for (int axis = 0; axis < 3; ++axis) {
            invalid_base_properties = invalid_base_properties ||
                std::abs(
                    plant.model().site_pos[3 * imu_site + axis] -
                    expected_imu_position[axis]) > 1.0e-9;
        }
        if (invalid_base_properties) {
            std::cerr << "base physical properties or support state are incorrect\n";
            return EXIT_FAILURE;
        }

        const int ground = mj_name2id(
            &plant.model(), mjOBJ_GEOM, "ground");
        for (int geom = 0; geom < plant.model().ngeom; ++geom) {
            if (geom == ground) continue;

            const bool collides_with_ground =
                plant.model().geom_conaffinity[geom] &
                plant.model().geom_contype[ground];
            const bool has_robot_collision_type =
                plant.model().geom_contype[geom] != 0;
            if (!collides_with_ground || has_robot_collision_type) {
                std::cerr << "incorrect ground collision filter at geom "
                          << geom << '\n';
                return EXIT_FAILURE;
            }
        }

        balance::sim::MujocoAdapter adapter(plant.model());
        const char *wheel_actuator_names[] = {
            "Left_Wheel_joint_actuator",
            "Right_Wheel_joint_actuator",
        };
        for (const char *name : wheel_actuator_names) {
            const int actuator = mj_name2id(
                &plant.model(), mjOBJ_ACTUATOR, name);
            if (actuator < 0 ||
                !plant.model().actuator_ctrllimited[actuator] ||
                plant.model().actuator_ctrlrange[2 * actuator] != -6.32 ||
                plant.model().actuator_ctrlrange[2 * actuator + 1] != 6.32) {
                std::cerr << name << " has incorrect torque limits\n";
                return EXIT_FAILURE;
            }
        }
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

        const int acceleration_sensor = mj_name2id(
            &plant.model(), mjOBJ_SENSOR, "imu_acceleration_sensor");
        if (acceleration_sensor < 0 ||
            plant.model().sensor_dim[acceleration_sensor] != 3) {
            std::cerr << "IMU acceleration sensor is missing\n";
            return EXIT_FAILURE;
        }
        const int acceleration_address =
            plant.model().sensor_adr[acceleration_sensor];
        const double expected_specific_force[] = {1.25, -2.5, 9.5};
        for (int axis = 0; axis < 3; ++axis) {
            plant.data().sensordata[acceleration_address + axis] =
                expected_specific_force[axis];
        }
        adapter.read(plant.data(), feedback);
        if (std::abs(
                feedback.imu.specific_force_x -
                expected_specific_force[0]) > 1.0e-6 ||
            std::abs(
                feedback.imu.specific_force_y -
                expected_specific_force[1]) > 1.0e-6 ||
            std::abs(
                feedback.imu.specific_force_z -
                expected_specific_force[2]) > 1.0e-6) {
            std::cerr << "IMU specific-force mapping is incorrect\n";
            return EXIT_FAILURE;
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
        plant.reset();
        const double roll_axis[] = {1.0, 0.0, 0.0};
        mju_axisAngle2Quat(
            plant.data().qpos + base_qpos + 3, roll_axis, 0.25);
        plant.data().qvel[base_dof + 3] = 0.6;
        mj_forward(&plant.model(), &plant.data());
        adapter.read(plant.data(), feedback);
        if (std::abs(feedback.imu.roll - 0.25F) > 1.0e-6F ||
            std::abs(feedback.imu.roll_rate - 0.6F) > 1.0e-6F) {
            std::cerr << "positive roll mapping is incorrect\n";
            return EXIT_FAILURE;
        }

        plant.reset();
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
        if (runner.snapshot().state_machine.system != BC_SYSTEM_OFF ||
            runner.snapshot().state_machine.motion != BC_MOTION_IDLE ||
            runner.snapshot().tick_count != 0U) {
            std::cerr << "runner reset snapshot is incorrect\n";
            return EXIT_FAILURE;
        }
        const auto stats = runner.run_for(kDurationSeconds);

        if (stats.physics_steps != 10 || stats.control_ticks != 10 ||
            runner.snapshot().tick_count != 10U) {
            std::cerr << "unexpected step counts\n";
            return EXIT_FAILURE;
        }
        if (std::abs(stats.final_time - kDurationSeconds) > 1.0e-12) {
            std::cerr << "unexpected final time: " << stats.final_time << '\n';
            return EXIT_FAILURE;
        }
        if (plant.data().qvel[base_dof + 2] >= -0.05) {
            std::cerr << "free chassis did not accelerate under gravity\n";
            return EXIT_FAILURE;
        }
        for (int actuator = 0; actuator < plant.model().nu; ++actuator) {
            if (plant.data().ctrl[actuator] != 0.0) {
                std::cerr << "actuator " << actuator << " was not zero\n";
                return EXIT_FAILURE;
            }
        }

        runner.reset();
        if (runner.snapshot().tick_count != 0U ||
            runner.snapshot().state_machine.system != BC_SYSTEM_OFF ||
            plant.data().time != 0.0) {
            std::cerr << "runner did not refresh snapshot on reset\n";
            return EXIT_FAILURE;
        }
    } catch (const std::exception &error) {
        std::cerr << "mujoco_smoke_test: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
