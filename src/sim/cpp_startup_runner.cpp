#include "cpp_startup_runner.hpp"

namespace balance::sim {

CppStartupRunner::CppStartupRunner(
    MujocoPlant &plant,
    const MujocoAdapter &adapter
)
    : CppStartupRunner(plant, adapter, control::ControllerConfig{}) {}

CppStartupRunner::CppStartupRunner(
    MujocoPlant &plant,
    const MujocoAdapter &adapter,
    const control::ControllerConfig &config
)
    : plant_(plant), adapter_(adapter), controller_(config) {}

void CppStartupRunner::reset() {
    plant_.reset();
    controller_.reset();
    output_ = {};
}

void CppStartupRunner::step(const control::OperatorCommand &command) {
    bc_sensor_feedback_t feedback{};
    adapter_.read(plant_.data(), feedback);
    output_ = controller_.tick(
        convert_sensor(feedback), command,
        static_cast<float>(plant_.timestep()));
    const bc_actuation_t actuation = convert_actuation(output_.applied);
    adapter_.write(plant_.data(), actuation);
    plant_.step();
}

control::SensorFrame CppStartupRunner::convert_sensor(
    const bc_sensor_feedback_t &feedback
) {
    control::SensorFrame result{};
    for (std::size_t side = 0; side < control::side_count; ++side) {
        for (std::size_t joint = 0; joint < control::joint_count; ++joint) {
            result.leg[side].joint[joint].angle =
                feedback.leg[side].joint[joint].angle;
            result.leg[side].joint[joint].angular_velocity =
                feedback.leg[side].joint[joint].angular_velocity;
            result.leg[side].joint[joint].torque =
                feedback.leg[side].joint[joint].torque;
        }
        result.wheel[side].angle = feedback.wheel[side].angle;
        result.wheel[side].angular_velocity =
            feedback.wheel[side].angular_velocity;
    }
    result.imu.roll = feedback.imu.roll;
    result.imu.pitch = feedback.imu.pitch;
    result.imu.yaw = feedback.imu.yaw;
    result.imu.roll_rate = feedback.imu.roll_rate;
    result.imu.pitch_rate = feedback.imu.pitch_rate;
    result.imu.yaw_rate = feedback.imu.yaw_rate;
    result.imu.specific_force_x = feedback.imu.specific_force_x;
    result.imu.specific_force_y = feedback.imu.specific_force_y;
    result.imu.specific_force_z = feedback.imu.specific_force_z;
    return result;
}

bc_actuation_t CppStartupRunner::convert_actuation(
    const control::Actuation &actuation
) {
    bc_actuation_t result{};
    for (std::size_t side = 0; side < control::side_count; ++side) {
        result.wheel_torque[side] = actuation.wheel_torque[side];
        for (std::size_t joint = 0; joint < control::joint_count; ++joint) {
            result.leg[side].joint_torque[joint] =
                actuation.leg[side].joint_torque[joint];
        }
    }
    return result;
}

} // namespace balance::sim
