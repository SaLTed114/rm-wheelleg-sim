#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>

#include "balance/math_utils.h"
#include "generated/mujoco_leg_calibration.hpp"
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
        const auto &cases =
            balance::benchmark::formal_performance_cases();
        const auto *heading_case =
            balance::benchmark::find_performance_case(
                "heading_pos_1p5pi");
        if (cases.size() != 4U || heading_case == nullptr ||
            heading_case->kind !=
                balance::benchmark::PerformanceCaseKind::heading_response ||
            std::abs(heading_case->yaw_target - 1.5 * BC_PI) > 1.0e-12 ||
            std::abs(heading_case->yaw_rate - 10.0) > 1.0e-12 ||
            balance::benchmark::find_performance_case(
                "yaw_pos_4pi") != nullptr) {
            std::cerr << "heading performance cases are incorrect\n";
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
            -0.019917, -0.00040396, -0.037,
        };
        const double expected_base_inertia[] = {
            0.3663415964, 0.367565, 0.413477,
        };
        bool invalid_base_properties =
            std::abs(plant.model().body_mass[base_body] - 17.65) > 1.0e-9 ||
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
        for (int element = 0; element < 4; ++element) {
            const double expected = element == 0 ? 1.0 : 0.0;
            invalid_base_properties = invalid_base_properties ||
                std::abs(
                    plant.model().body_iquat[4 * base_body + element] -
                    expected) > 1.0e-12;
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

        const char *leg_body_names[2][6] = {
            {
                "Left_front_link", "Left_front_child1_link",
                "Left_front_child2_link", "Left_front_child3_link",
                "Left_rear_link", "Left_rear_child1_link",
            },
            {
                "Right_front_link", "Right_front_child1_link",
                "Right_front_child2_link", "Right_front_child3_link",
                "Right_rear_link", "Right_rear_child1_link",
            },
        };
        for (const auto &side : leg_body_names) {
            double mass = 0.0;
            for (const char *name : side) {
                const int body = mj_name2id(
                    &plant.model(), mjOBJ_BODY, name);
                mass += plant.model().body_mass[body];
            }
            if (std::abs(mass - 1.19) > 1.0e-9) {
                std::cerr << "incorrect aggregate leg mass: " << mass << '\n';
                return EXIT_FAILURE;
            }
        }
        const char *wheel_body_names[] = {
            "Left_Wheel_link", "Right_Wheel_link",
        };
        for (const char *name : wheel_body_names) {
            const int body = mj_name2id(
                &plant.model(), mjOBJ_BODY, name);
            if (std::abs(plant.model().body_mass[body] - 0.71) > 1.0e-9 ||
                std::abs(
                    plant.model().body_inertia[3 * body] -
                    0.001194190264) > 1.0e-10) {
                std::cerr << name << " has incorrect mass or inertia\n";
                return EXIT_FAILURE;
            }
        }
        double total_mass = 0.0;
        for (int body = 1; body < plant.model().nbody; ++body) {
            total_mass += plant.model().body_mass[body];
        }
        if (std::abs(total_mass - 21.45) > 1.0e-9) {
            std::cerr << "incorrect total robot mass: " << total_mass << '\n';
            return EXIT_FAILURE;
        }

        const int ground = mj_name2id(
            &plant.model(), mjOBJ_GEOM, "ground");
        const int platform_200 = mj_name2id(
            &plant.model(), mjOBJ_GEOM, "drop_platform_200mm");
        const int platform_400 = mj_name2id(
            &plant.model(), mjOBJ_GEOM, "drop_platform_400mm");
        const int benchmark_ramp = mj_name2id(
            &plant.model(), mjOBJ_GEOM, "benchmark_ramp_17deg");
        const char *keyboard_surface_names[] = {
            "keyboard_platform_200mm",
            "keyboard_ramp_15deg",
            "keyboard_ramp_17deg",
        };
        const char *ramp_course_surface_names[] = {
            "ramp_course_up",
            "ramp_course_platform",
            "ramp_course_down",
            "ramp_course_beveled_up",
            "ramp_course_bevel",
            "ramp_course_beveled_platform",
            "ramp_course_beveled_down",
        };
        std::array<int, 3> keyboard_surfaces{};
        std::array<int, 7> ramp_course_surfaces{};
        bool invalid_hidden_surface = platform_200 < 0 || platform_400 < 0 ||
            benchmark_ramp < 0;
        for (std::size_t index = 0; index < keyboard_surfaces.size(); ++index) {
            keyboard_surfaces[index] = mj_name2id(
                &plant.model(), mjOBJ_GEOM, keyboard_surface_names[index]);
            invalid_hidden_surface = invalid_hidden_surface ||
                keyboard_surfaces[index] < 0;
        }
        for (std::size_t index = 0;
             index < ramp_course_surfaces.size(); ++index) {
            ramp_course_surfaces[index] = mj_name2id(
                &plant.model(), mjOBJ_GEOM,
                ramp_course_surface_names[index]);
            invalid_hidden_surface = invalid_hidden_surface ||
                ramp_course_surfaces[index] < 0;
        }
        const int hidden_surfaces[] = {
            platform_200,
            platform_400,
            benchmark_ramp,
            keyboard_surfaces[0],
            keyboard_surfaces[1],
            keyboard_surfaces[2],
            ramp_course_surfaces[0],
            ramp_course_surfaces[1],
            ramp_course_surfaces[2],
            ramp_course_surfaces[3],
            ramp_course_surfaces[4],
            ramp_course_surfaces[5],
            ramp_course_surfaces[6],
        };
        for (const int geom : hidden_surfaces) {
            invalid_hidden_surface = invalid_hidden_surface || geom < 0 ||
                plant.model().geom_contype[geom] != 1 ||
                plant.data().geom_xpos[3 * geom + 2] > -1.0 ||
                plant.model().geom_rgba[4 * geom + 3] != 0.0F;
        }
        if (invalid_hidden_surface) {
            std::cerr << "optional terrain is missing or active by default\n";
            return EXIT_FAILURE;
        }
        for (int geom = 0; geom < plant.model().ngeom; ++geom) {
            const bool optional_surface = std::find(
                std::begin(hidden_surfaces), std::end(hidden_surfaces),
                geom) != std::end(hidden_surfaces);
            if (geom == ground || optional_surface) continue;

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

        plant.configure_keyboard_course();
        constexpr double kPi = 3.14159265358979323846;
        struct ExpectedSurface {
            int geom;
            std::array<double, 3> position;
            std::array<double, 3> size;
        };
        const ExpectedSurface expected_surface[] = {
            {keyboard_surfaces[0], {3.746410162, -1.5, -0.33},
             {1.0, 1.0, 0.1}},
        };
        for (const auto &surface : expected_surface) {
            const int geom = surface.geom;
            for (int axis = 0; axis < 3; ++axis) {
                if (std::abs(
                        plant.data().geom_xpos[3 * geom + axis] -
                        surface.position[axis]) > 1.0e-8) {
                    std::cerr << "keyboard terrain position is incorrect\n";
                    return EXIT_FAILURE;
                }
                if (std::abs(
                        plant.model().geom_size[3 * geom + axis] -
                        surface.size[axis]) > 1.0e-8) {
                    std::cerr << "keyboard terrain size is incorrect\n";
                    return EXIT_FAILURE;
                }
            }
            if (plant.model().geom_rgba[4 * geom + 3] != 1.0F) {
                std::cerr << "keyboard terrain was not revealed\n";
                return EXIT_FAILURE;
            }
        }
        const double ramp_angles[] = {
            15.0 * kPi / 180.0, 17.0 * kPi / 180.0,
        };
        const double ramp_heights[] = {0.2, 0.35};
        const double ramp_widths[] = {2.0, 0.86};
        const double ramp_lanes[] = {-1.5, 1.0};
        const int ramp_geoms[] = {
            keyboard_surfaces[1], keyboard_surfaces[2],
        };
        for (int index = 0; index < 2; ++index) {
            const int mesh = plant.model().geom_dataid[ramp_geoms[index]];
            const double run = ramp_heights[index] /
                std::tan(ramp_angles[index]);
            double minimum_x = std::numeric_limits<double>::infinity();
            double maximum_x = -std::numeric_limits<double>::infinity();
            double minimum_y = std::numeric_limits<double>::infinity();
            double maximum_y = -std::numeric_limits<double>::infinity();
            double minimum_z = std::numeric_limits<double>::infinity();
            double maximum_z = -std::numeric_limits<double>::infinity();
            if (mesh >= 0) {
                const int first_vertex = plant.model().mesh_vertadr[mesh];
                for (int vertex = 0;
                     vertex < plant.model().mesh_vertnum[mesh]; ++vertex) {
                    const int offset = 3 * (first_vertex + vertex);
                    double world[3]{};
                    for (int row = 0; row < 3; ++row) {
                        world[row] = plant.data().geom_xpos[
                            3 * ramp_geoms[index] + row];
                        for (int column = 0; column < 3; ++column) {
                            world[row] += plant.data().geom_xmat[
                                9 * ramp_geoms[index] + 3 * row + column] *
                                plant.model().mesh_vert[offset + column];
                        }
                    }
                    minimum_x = std::min(minimum_x, world[0]);
                    maximum_x = std::max(maximum_x, world[0]);
                    minimum_y = std::min(minimum_y, world[1]);
                    maximum_y = std::max(maximum_y, world[1]);
                    minimum_z = std::min(minimum_z, world[2]);
                    maximum_z = std::max(maximum_z, world[2]);
                }
            }
            if (plant.model().geom_type[ramp_geoms[index]] != mjGEOM_MESH ||
                mesh < 0 || plant.model().mesh_vertnum[mesh] != 6 ||
                std::abs(minimum_x - 2.0) > 1.0e-6 ||
                std::abs(0.5 * (minimum_y + maximum_y) -
                    ramp_lanes[index]) > 1.0e-6 ||
                std::abs(maximum_y - minimum_y - ramp_widths[index]) >
                    1.0e-6 ||
                std::abs(maximum_z - minimum_z - ramp_heights[index]) >
                    1.0e-6 ||
                std::abs(maximum_x - minimum_x - run) > 1.0e-6 ||
                std::abs(minimum_z + 0.43) > 1.0e-6 ||
                std::abs(maximum_z - (-0.43 + ramp_heights[index])) >
                    1.0e-6 ||
                std::abs((maximum_z - minimum_z) /
                    (maximum_x - minimum_x) -
                    std::tan(ramp_angles[index])) > 1.0e-6) {
                std::cerr << "keyboard triangular ramp is incorrect: index="
                          << index << " x=[" << minimum_x << ',' << maximum_x
                          << "] y=[" << minimum_y << ',' << maximum_y
                          << "] z=[" << minimum_z << ',' << maximum_z
                          << "] center=("
                          << plant.data().geom_xpos[3 * ramp_geoms[index]]
                          << ','
                          << plant.data().geom_xpos[
                              3 * ramp_geoms[index] + 1]
                          << ','
                          << plant.data().geom_xpos[
                              3 * ramp_geoms[index] + 2]
                          << ")\n";
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
        const char *mapped_actuator_names[BC_SIDE_NUM][BC_JOINT_NUM] = {
            {"Right_front_joint_actuator", "Right_rear_joint_actuator"},
            {"Left_front_joint_actuator", "Left_rear_joint_actuator"},
        };
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                const int actuator = mj_name2id(
                    &plant.model(), mjOBJ_ACTUATOR,
                    mapped_actuator_names[side][joint]);
                plant.data().actuator_force[actuator] =
                    1.0 + 2.0 * side + joint;
            }
        }
        adapter.read(plant.data(), feedback);
        const char *mapped_joint_names[BC_SIDE_NUM][BC_JOINT_NUM] = {
            {"Right_front_joint", "Right_rear_joint"},
            {"Left_front_joint", "Left_rear_joint"},
        };
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            for (int joint = 0; joint < BC_JOINT_NUM; ++joint) {
                const int model_joint = mj_name2id(
                    &plant.model(), mjOBJ_JOINT,
                    mapped_joint_names[side][joint]);
                const int qpos = plant.model().jnt_qposadr[model_joint];
                const double expected =
                    balance::sim::calibration::kJointScales[side][joint] *
                        plant.data().qpos[qpos] +
                    balance::sim::calibration::kJointOffsets[side][joint];
                const double error = std::abs(
                    feedback.leg[side].joint[joint].angle -
                    expected);
                const double expected_torque =
                    balance::sim::calibration::kJointScales[side][joint] *
                    (1.0 + 2.0 * side + joint);
                if (error > 1.0e-6 ||
                    std::abs(feedback.leg[side].joint[joint].torque -
                             expected_torque) > 1.0e-6) {
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
