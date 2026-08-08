#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

#include "balance/math_utils.h"
#include "input/virtual_gimbal.hpp"
#include "mujoco_adapter.hpp"
#include "mujoco_plant.hpp"
#include "simulation_runner.hpp"

namespace {

int require_id(
    const mjModel &model, const mjtObj type, const char *name
) {
    const int id = mj_name2id(&model, type, name);
    if (id < 0) {
        throw std::runtime_error(
            "missing MuJoCo object '" + std::string(name) + "'");
    }
    return id;
}

struct ContactState {
    std::array<bool, BC_SIDE_NUM> wheel{};
    bool other{};
};

ContactState read_contacts(
    const mjData &data, const int ground,
    const std::array<int, BC_SIDE_NUM> &wheel
) {
    ContactState state{};
    for (int index = 0; index < data.ncon; ++index) {
        const mjContact &contact = data.contact[index];
        const bool has_ground =
            contact.geom[0] == ground || contact.geom[1] == ground;
        if (!has_ground) {
            state.other = true;
            continue;
        }

        bool wheel_contact = false;
        for (int side = 0; side < BC_SIDE_NUM; ++side) {
            const bool pair =
                (contact.geom[0] == ground && contact.geom[1] == wheel[side]) ||
                (contact.geom[1] == ground && contact.geom[0] == wheel[side]);
            state.wheel[side] = state.wheel[side] || pair;
            wheel_contact = wheel_contact || pair;
        }
        state.other = state.other || !wheel_contact;
    }
    return state;
}

struct MotionMetrics {
    double forward_displacement{};
    double yaw_change{};
    double average_forward_velocity{};
    double average_yaw_rate{};
    double wheel_contact_ratio{};
    double maximum_pitch{};
    int other_contact_steps{};
};

struct MotionCommandSource {
    balance::sim::VirtualGimbal gimbal{};
    bool active_initialized{};
};

void step_controller(
    balance::sim::SimulationRunner &runner,
    balance::sim::MujocoPlant &plant,
    MotionCommandSource &source,
    const float forward_velocity,
    const float gimbal_yaw_rate
) {
    bc_operator_command_t command{};
    command.system_enabled = static_cast<uint8_t>(
        plant.data().time >= 2.0);
    command.balance_restart =
        command.system_enabled &&
        runner.snapshot().state_machine.system == BC_SYSTEM_OFF;
    if (runner.snapshot().state_machine.motion == BC_MOTION_ACTIVE) {
        if (!source.active_initialized) {
            source.gimbal.reset(
                runner.snapshot().state.value[BC_STATE_PSI]);
            source.active_initialized = true;
        }
        source.gimbal.update(
            gimbal_yaw_rate, static_cast<float>(plant.timestep()));
        command.forward_velocity = forward_velocity;
    } else {
        source.active_initialized = false;
    }
    const bc_gimbal_feedback_t gimbal_feedback =
        source.active_initialized ?
            source.gimbal.feedback(
                runner.snapshot().state.value[BC_STATE_PSI],
                runner.snapshot().state.value[BC_STATE_DPSI]) :
            bc_gimbal_feedback_t{};
    runner.step(command, gimbal_feedback);
}

MotionMetrics run_motion_phase(
    balance::sim::SimulationRunner &runner,
    balance::sim::MujocoPlant &plant,
    MotionCommandSource &source,
    const int base_qpos,
    const int ground,
    const std::array<int, BC_SIDE_NUM> &wheel,
    const float forward_velocity,
    const float yaw_rate,
    const double duration
) {
    const double start_time = plant.data().time;
    const double evaluation_start = start_time + duration - 1.0;
    const double initial_x = plant.data().qpos[base_qpos];
    const double initial_y = plant.data().qpos[base_qpos + 1];
    const double initial_yaw = runner.snapshot().state.value[BC_STATE_PSI];
    int steps = 0;
    int both_wheels_steps = 0;
    double forward_velocity_sum = 0.0;
    double yaw_rate_sum = 0.0;
    MotionMetrics metrics{};

    while (plant.data().time < start_time + duration) {
        step_controller(
            runner, plant, source, forward_velocity, yaw_rate);
        const auto contact = read_contacts(plant.data(), ground, wheel);
        if (contact.wheel[BC_L] && contact.wheel[BC_R]) {
            ++both_wheels_steps;
        }
        if (contact.other) ++metrics.other_contact_steps;
        metrics.maximum_pitch = std::max(
            metrics.maximum_pitch,
            std::abs(static_cast<double>(
                runner.snapshot().state.value[BC_STATE_THETA_B])));

        ++steps;
        if (plant.data().time >= evaluation_start) {
            forward_velocity_sum +=
                runner.snapshot().state.value[BC_STATE_DS];
            yaw_rate_sum += runner.snapshot().state.value[BC_STATE_DPSI];
        }
    }

    const int evaluation_steps = static_cast<int>(std::round(
        1.0 / plant.timestep()));
    const double delta_x = plant.data().qpos[base_qpos] - initial_x;
    const double delta_y = plant.data().qpos[base_qpos + 1] - initial_y;
    metrics.forward_displacement =
        delta_x * std::cos(initial_yaw) + delta_y * std::sin(initial_yaw);
    metrics.yaw_change =
        runner.snapshot().state.value[BC_STATE_PSI] - initial_yaw;
    metrics.average_forward_velocity =
        forward_velocity_sum / evaluation_steps;
    metrics.average_yaw_rate = yaw_rate_sum / evaluation_steps;
    metrics.wheel_contact_ratio =
        static_cast<double>(both_wheels_steps) / steps;
    return metrics;
}

void print_motion_metrics(
    const char *name, const MotionMetrics &metrics
) {
    std::cout << name
              << ": ds=" << metrics.average_forward_velocity
              << " m/s, delta=" << metrics.forward_displacement
              << " m, dpsi=" << metrics.average_yaw_rate
              << " rad/s, delta psi=" << metrics.yaw_change
              << " rad, contact=" << metrics.wheel_contact_ratio
              << ", max pitch="
              << metrics.maximum_pitch * 180.0 / BC_PI << " deg\n";
}

} // namespace

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: mujoco_static_stand_test <model.xml>\n";
        return EXIT_FAILURE;
    }

    try {
        balance::sim::MujocoPlant plant(
            std::filesystem::path(argv[1]), 0.001);
        balance::sim::MujocoAdapter adapter(plant.model());
        balance::sim::SimulationRunner runner(plant, adapter);
        MotionCommandSource command_source{};
        const int weld = require_id(
            plant.model(), mjOBJ_EQUALITY, "base_support_weld");
        const int ground = require_id(
            plant.model(), mjOBJ_GEOM, "ground");
        const std::array<int, BC_SIDE_NUM> wheel{{
            require_id(
                plant.model(), mjOBJ_GEOM, "Right_wheel_collision"),
            require_id(
                plant.model(), mjOBJ_GEOM, "Left_wheel_collision"),
        }};
        const std::array<int, BC_SIDE_NUM> wheel_axis{{
            require_id(
                plant.model(), mjOBJ_SITE, "Right_wheel_axis_site"),
            require_id(
                plant.model(), mjOBJ_SITE, "Left_wheel_axis_site"),
        }};
        const int base_joint = require_id(
            plant.model(), mjOBJ_JOINT, "base_free_joint");
        const int base_qpos = plant.model().jnt_qposadr[base_joint];

        runner.reset();
        if (plant.data().eq_active[weld]) {
            std::cerr << "reset did not disable chassis support\n";
            return EXIT_FAILURE;
        }

        while (plant.data().time < 8.0 &&
               runner.snapshot().state_machine.motion !=
                   BC_MOTION_ACTIVE) {
            step_controller(runner, plant, command_source, 0.0F, 0.0F);
        }
        if (runner.snapshot().state_machine.motion !=
                BC_MOTION_ACTIVE ||
            plant.data().eq_active[weld]) {
            std::cerr << "controller did not balance after posture settled: "
                      << bc_motion_state_name(
                             runner.snapshot().state_machine.motion)
                      << ", leg length="
                      << runner.snapshot().leg[BC_L].length << '/'
                      << runner.snapshot().leg[BC_R].length << ", leg angle="
                      << runner.snapshot().leg[BC_L].angle_body << '/'
                      << runner.snapshot().leg[BC_R].angle_body
                      << ", leg velocity="
                      << runner.snapshot().leg[BC_L].length_velocity << '/'
                      << runner.snapshot().leg[BC_R].length_velocity
                      << ", angular velocity="
                      << runner.snapshot().leg[BC_L].angular_velocity << '/'
                      << runner.snapshot().leg[BC_R].angular_velocity << '\n';
            return EXIT_FAILURE;
        }

        const bc_state_vector_t enable_state = runner.snapshot().state;
        const std::array<bc_leg_kinematics_t, BC_SIDE_NUM> enable_leg{{
            runner.snapshot().leg[BC_L],
            runner.snapshot().leg[BC_R],
        }};
        const ContactState enable_contact = read_contacts(
            plant.data(), ground, wheel);

        constexpr double kBalanceDuration = 8.0;
        constexpr double kEvaluationDuration = 3.0;
        const double balance_start_time = plant.data().time;
        const double end_time = balance_start_time + kBalanceDuration;
        int evaluation_steps = 0;
        int both_wheels_steps = 0;
        int other_contact_steps = 0;
        double maximum_final_pitch = 0.0;
        double maximum_final_pitch_rate = 0.0;
        bool finite = true;

        while (plant.data().time < end_time) {
            step_controller(runner, plant, command_source, 0.0F, 0.0F);
            if (plant.data().time < end_time - kEvaluationDuration) continue;

            ++evaluation_steps;
            const auto contact = read_contacts(plant.data(), ground, wheel);
            if (contact.wheel[BC_L] && contact.wheel[BC_R]) {
                ++both_wheels_steps;
            }
            if (contact.other) ++other_contact_steps;

            const auto &state = runner.snapshot().state;
            maximum_final_pitch = std::max(
                maximum_final_pitch,
                std::abs(static_cast<double>(
                    state.value[BC_STATE_THETA_B])));
            maximum_final_pitch_rate = std::max(
                maximum_final_pitch_rate,
                std::abs(static_cast<double>(
                    state.value[BC_STATE_DTHETA_B])));
            for (float value : state.value) {
                finite = finite && std::isfinite(value);
            }
            for (int side = 0; side < BC_SIDE_NUM; ++side) {
                finite = finite &&
                    runner.snapshot().leg[side].length >= 0.13F &&
                    runner.snapshot().leg[side].length <= 0.20F;
                finite = finite && std::isfinite(
                    runner.snapshot().actuation.wheel_torque[side]);
                for (float torque :
                     runner.snapshot().actuation.leg[side].joint_torque) {
                    finite = finite && std::isfinite(torque);
                }
            }
        }

        const double wheel_contact_ratio =
            static_cast<double>(both_wheels_steps) / evaluation_steps;
        std::cout << "static stand: release=" << balance_start_time
                  << " s, max final pitch="
                  << maximum_final_pitch * 180.0 / BC_PI
                  << " deg, max final pitch rate="
                  << maximum_final_pitch_rate
                  << " rad/s, wheel contact=" << wheel_contact_ratio
                  << ", other contact steps=" << other_contact_steps << '\n';

        if (other_contact_steps != 0) {
            std::cout << "final geometry: base z="
                      << plant.data().qpos[base_qpos + 2]
                      << ", wheel axis z="
                      << plant.data().site_xpos[3 * wheel_axis[BC_L] + 2]
                      << '/' << plant.data().site_xpos[3 * wheel_axis[BC_R] + 2]
                      << ", leg length="
                      << runner.snapshot().leg[BC_L].length << '/'
                      << runner.snapshot().leg[BC_R].length << '\n';
            std::cout << "final contacts:";
            for (int index = 0; index < plant.data().ncon; ++index) {
                const mjContact &contact = plant.data().contact[index];
                for (int pair = 0; pair < 2; ++pair) {
                    const char *name = mj_id2name(
                        &plant.model(), mjOBJ_GEOM, contact.geom[pair]);
                    const int body =
                        plant.model().geom_bodyid[contact.geom[pair]];
                    const char *body_name = mj_id2name(
                        &plant.model(), mjOBJ_BODY, body);
                    std::cout << ' ' << (name ? name : body_name);
                }
            }
            std::cout << '\n';
        }

        const bool standing = finite &&
            maximum_final_pitch < 5.0 * BC_PI / 180.0 &&
            maximum_final_pitch_rate < 0.2 &&
            wheel_contact_ratio >= 0.99 &&
            other_contact_steps == 0;
        if (!standing) {
            std::cerr << "balance enable: state=";
            for (const float value : enable_state.value) {
                std::cerr << ' ' << value;
            }
            std::cerr << ", leg length="
                      << enable_leg[BC_L].length << '/'
                      << enable_leg[BC_R].length
                      << ", leg angle="
                      << enable_leg[BC_L].angle_body << '/'
                      << enable_leg[BC_R].angle_body
                      << ", wheel contact="
                      << enable_contact.wheel[BC_L] << '/'
                      << enable_contact.wheel[BC_R]
                      << ", other contact=" << enable_contact.other << '\n';
            std::cerr << "final actuation: wheel="
                      << runner.snapshot().actuation.wheel_torque[BC_L] << '/'
                      << runner.snapshot().actuation.wheel_torque[BC_R]
                      << ", joint="
                      << runner.snapshot().actuation.leg[BC_L]
                             .joint_torque[BC_FRONT] << '/'
                      << runner.snapshot().actuation.leg[BC_L]
                             .joint_torque[BC_REAR] << ' '
                      << runner.snapshot().actuation.leg[BC_R]
                             .joint_torque[BC_FRONT] << '/'
                      << runner.snapshot().actuation.leg[BC_R]
                             .joint_torque[BC_REAR] << '\n';
            std::cerr << "controller did not maintain static standing\n";
            return EXIT_FAILURE;
        }

        const auto forward = run_motion_phase(
            runner, plant, command_source, base_qpos, ground, wheel,
            0.25F, 0.0F, 3.0);
        run_motion_phase(
            runner, plant, command_source, base_qpos, ground, wheel,
            0.0F, 0.0F, 1.5);
        const auto reverse = run_motion_phase(
            runner, plant, command_source, base_qpos, ground, wheel,
            -0.25F, 0.0F, 3.0);
        run_motion_phase(
            runner, plant, command_source, base_qpos, ground, wheel,
            0.0F, 0.0F, 1.5);
        const auto yaw_left = run_motion_phase(
            runner, plant, command_source, base_qpos, ground, wheel,
            0.0F, 1.57F, 3.0);
        run_motion_phase(
            runner, plant, command_source, base_qpos, ground, wheel,
            0.0F, 0.0F, 1.5);
        const auto yaw_right = run_motion_phase(
            runner, plant, command_source, base_qpos, ground, wheel,
            0.0F, -1.57F, 3.0);

        print_motion_metrics("forward", forward);
        print_motion_metrics("reverse", reverse);
        print_motion_metrics("yaw left", yaw_left);
        print_motion_metrics("yaw right", yaw_right);

        const auto motion_is_safe = [](const MotionMetrics &metrics) {
            return metrics.wheel_contact_ratio >= 0.99 &&
                metrics.other_contact_steps == 0 &&
                metrics.maximum_pitch < 10.0 * BC_PI / 180.0;
        };
        const bool motion_valid =
            motion_is_safe(forward) &&
            forward.average_forward_velocity > 0.10 &&
            forward.forward_displacement > 0.20 &&
            motion_is_safe(reverse) &&
            reverse.average_forward_velocity < -0.10 &&
            reverse.forward_displacement < -0.20 &&
            motion_is_safe(yaw_left) &&
            yaw_left.average_yaw_rate > 1.0 &&
            yaw_left.yaw_change > 2.0 &&
            motion_is_safe(yaw_right) &&
            yaw_right.average_yaw_rate < -1.0 &&
            yaw_right.yaw_change < -2.0;
        if (!motion_valid) {
            std::cerr << "velocity commands did not track safely\n";
            return EXIT_FAILURE;
        }

        runner.reset();
        if (plant.data().eq_active[weld] ||
            runner.snapshot().state_machine.system != BC_SYSTEM_OFF ||
            runner.snapshot().state_machine.motion != BC_MOTION_IDLE) {
            std::cerr << "reset did not restore disabled settling\n";
            return EXIT_FAILURE;
        }
    } catch (const std::exception &error) {
        std::cerr << "mujoco_static_stand_test: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
