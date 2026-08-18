#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <type_traits>
#include <utility>

#include "balance/controller_snapshot.h"
#include "balance_cpp/controller.hpp"
#include "balance_cpp/math.hpp"

namespace {

using balance::control::Controller;
using balance::control::ControllerOutput;
using balance::control::MotionState;
using balance::control::OperatorCommand;
using balance::control::SensorFrame;
using balance::control::SystemState;

float solve_delta(const float target_length) {
    float low = 0.0F;
    float high = 0.5F * balance::control::math::pi;
    for (int iteration = 0; iteration < 80; ++iteration) {
        const float delta = 0.5F * (low + high);
        const float length = 0.215F * std::cos(delta) + std::sqrt(
            0.254F * 0.254F - 0.215F * 0.215F *
                std::sin(delta) * std::sin(delta));
        if (length > target_length) {
            low = delta;
        } else {
            high = delta;
        }
    }
    return 0.5F * (low + high);
}

struct Harness {
    bc_controller_t legacy{};
    bc_controller_snapshot_t legacy_snapshot{};
    Controller modern{};
    bc_sensor_feedback_t legacy_sensor{};
    SensorFrame modern_sensor{};
    bc_operator_command_t legacy_command{};
    OperatorCommand modern_command{};
    bc_actuation_t legacy_actuation{};
    ControllerOutput modern_output{};

    Harness() {
        bc_controller_config_t config{};
        bc_controller_default_config(&config);
        bc_controller_init(&legacy, &config);
        const float delta = solve_delta(0.18F);
        for (std::size_t side = 0; side < balance::control::side_count;
             ++side) {
            legacy_sensor.leg[side].joint[BC_FRONT].angle =
                -0.5F * balance::control::math::pi + delta;
            legacy_sensor.leg[side].joint[BC_REAR].angle =
                -0.5F * balance::control::math::pi - delta;
            modern_sensor.leg[side].joint[0].angle =
                legacy_sensor.leg[side].joint[BC_FRONT].angle;
            modern_sensor.leg[side].joint[1].angle =
                legacy_sensor.leg[side].joint[BC_REAR].angle;
        }
        legacy_sensor.imu.specific_force_z = 9.81F;
        modern_sensor.imu.specific_force_z = 9.81F;
    }

    void step() {
        bc_controller_update(&legacy, &legacy_sensor, 0.001F);
        bc_controller_set_command(&legacy, &legacy_command);
        bc_controller_calculate(&legacy);
        bc_controller_execute(&legacy, &legacy_actuation);
        bc_controller_capture_snapshot(&legacy, &legacy_snapshot);
        modern_output = modern.tick(
            modern_sensor, modern_command, 0.001F);
    }
};

bool near(const float left, const float right, const float tolerance) {
    return std::abs(left - right) <= tolerance;
}

bool compare_common(const Harness &harness, const float tolerance) {
    for (std::size_t index = 0; index < balance::control::state_count;
         ++index) {
        if (!near(
                harness.legacy_snapshot.state.value[index],
                harness.modern_output.snapshot.state.value[index],
                tolerance)) {
            std::cerr << "state mismatch at " << index << '\n';
            return false;
        }
    }
    for (std::size_t side = 0; side < balance::control::side_count; ++side) {
        const auto &legacy_leg = harness.legacy_snapshot.leg[side];
        const auto &modern_leg = harness.modern_output.snapshot.leg[side];
        if (!near(legacy_leg.length, modern_leg.length, tolerance) ||
            !near(legacy_leg.angle_body, modern_leg.angle_body, tolerance) ||
            !near(
                legacy_leg.length_velocity,
                modern_leg.length_velocity, tolerance) ||
            !near(
                legacy_leg.angular_velocity,
                modern_leg.angular_velocity, tolerance)) {
            std::cerr << "leg kinematics mismatch at " << side << '\n';
            return false;
        }
        if (!near(
                harness.legacy_actuation.wheel_torque[side],
                harness.modern_output.applied.wheel_torque[side],
                4.0e-5F)) {
            std::cerr << "wheel actuation mismatch at " << side
                      << ": legacy="
                      << harness.legacy_actuation.wheel_torque[side]
                      << ", cpp="
                      << harness.modern_output.applied.wheel_torque[side]
                      << '\n';
            for (std::size_t index = 0;
                 index < balance::control::state_count; ++index) {
                std::cerr << "  ref[" << index << "] legacy="
                          << harness.legacy_snapshot.state_reference
                                 .value[index]
                          << " cpp="
                          << harness.modern_output.snapshot.state_reference
                                 .value[index]
                          << '\n';
            }
            return false;
        }
        for (std::size_t joint = 0; joint < balance::control::joint_count;
             ++joint) {
            if (!near(
                    harness.legacy_actuation.leg[side].joint_torque[joint],
                    harness.modern_output.applied.leg[side]
                        .joint_torque[joint],
                    4.0e-5F)) {
                std::cerr << "joint actuation mismatch at "
                          << side << '/' << joint << '\n';
                return false;
            }
        }
    }
    return true;
}

} // namespace

int main() {
    static_assert(std::is_nothrow_destructible_v<Controller>);
    static_assert(noexcept(std::declval<Controller &>().tick(
        std::declval<const SensorFrame &>(),
        std::declval<const OperatorCommand &>(), 0.001F)));

    Harness harness;
    harness.step();
    if (harness.modern_output.snapshot.system_state != SystemState::off ||
        harness.modern_output.snapshot.motion_state != MotionState::idle ||
        !compare_common(harness, 2.0e-6F)) {
        std::cerr << "disabled startup parity failed\n";
        return EXIT_FAILURE;
    }

    harness.legacy_command.system_enabled = 1U;
    harness.modern_command.system_enabled = true;
    harness.step();
    if (harness.modern_output.snapshot.system_state != SystemState::on ||
        harness.modern_output.snapshot.motion_state != MotionState::idle ||
        harness.legacy_snapshot.state_machine.motion != BC_MOTION_IDLE ||
        !compare_common(harness, 2.0e-6F)) {
        std::cerr << "idle parity failed\n";
        return EXIT_FAILURE;
    }

    harness.legacy_command.balance_restart = 1U;
    harness.modern_command.balance_restart = true;
    harness.step();
    harness.legacy_command.balance_restart = 0U;
    harness.modern_command.balance_restart = false;
    if (harness.modern_output.snapshot.motion_state !=
            MotionState::leg_positioning ||
        harness.legacy_snapshot.state_machine.motion !=
            BC_MOTION_LEG_POSITIONING ||
        !compare_common(harness, 2.0e-6F)) {
        std::cerr << "leg positioning parity failed\n";
        return EXIT_FAILURE;
    }

    bool saw_engaging = false;
    bool saw_active = false;
    for (int tick = 0; tick < 200; ++tick) {
        harness.step();
        const auto modern_state = harness.modern_output.snapshot.motion_state;
        if (modern_state == MotionState::balance_engaging) {
            saw_engaging = true;
            if (harness.legacy_snapshot.state_machine.motion !=
                BC_MOTION_BALANCE_ENGAGING) {
                std::cerr << "engaging changed on a different tick\n";
                return EXIT_FAILURE;
            }
        }
        if (modern_state == MotionState::active) {
            saw_active = true;
            if (harness.legacy_snapshot.state_machine.motion !=
                BC_MOTION_ACTIVE) {
                std::cerr << "active changed on a different tick\n";
                return EXIT_FAILURE;
            }
        }
        if (modern_state != MotionState::active &&
            !compare_common(harness, 3.0e-5F)) {
            std::cerr << "startup parity failed at tick " << tick << '\n';
            return EXIT_FAILURE;
        }
        if (saw_active) break;
    }
    if (!saw_engaging || !saw_active) {
        std::cerr << "startup did not reach active hold\n";
        return EXIT_FAILURE;
    }
    for (const float torque : harness.modern_output.applied.wheel_torque) {
        if (!std::isfinite(torque)) {
            std::cerr << "active hold produced non-finite wheel torque\n";
            return EXIT_FAILURE;
        }
    }

    harness.modern_command.balance_restart = true;
    harness.legacy_command.balance_restart = 1U;
    harness.step();
    harness.modern_command.balance_restart = false;
    harness.legacy_command.balance_restart = 0U;
    if (harness.modern_output.snapshot.motion_state !=
            MotionState::leg_positioning ||
        harness.legacy_snapshot.state_machine.motion !=
            BC_MOTION_LEG_POSITIONING ||
        !compare_common(harness, 3.0e-5F)) {
        std::cerr << "active restart parity failed\n";
        return EXIT_FAILURE;
    }

    harness.modern_command.system_enabled = false;
    harness.legacy_command.system_enabled = 0U;
    harness.step();
    if (harness.modern_output.snapshot.system_state != SystemState::off ||
        harness.modern_output.snapshot.motion_state != MotionState::idle) {
        std::cerr << "disable did not leave active hold\n";
        return EXIT_FAILURE;
    }
    for (const auto &leg : harness.modern_output.applied.leg) {
        for (const float torque : leg.joint_torque) {
            if (torque != 0.0F) return EXIT_FAILURE;
        }
    }
    for (const float torque : harness.modern_output.applied.wheel_torque) {
        if (torque != 0.0F) return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
