#ifndef BALANCE_SIM_MUJOCO_ADAPTER_HPP
#define BALANCE_SIM_MUJOCO_ADAPTER_HPP

#include <array>

#include <mujoco/mujoco.h>

#include "balance/types.h"

namespace balance::sim {

class MujocoAdapter {
public:
    explicit MujocoAdapter(const mjModel &model);

    void read(const mjData &data, bc_sensor_feedback_t &feedback) const;
    void write(mjData &data, const bc_actuation_t &actuation) const;

private:
    struct ChannelAddress {
        int qpos;
        int dof;
        int actuator;
        double scale;
        double offset;
    };

    static ChannelAddress resolve_channel(
        const mjModel &model,
        const char *joint_name,
        const char *actuator_name,
        double scale,
        double offset);
    static int resolve_sensor(
        const mjModel &model, const char *name, int dimension);

    std::array<std::array<ChannelAddress, BC_JOINT_NUM>, BC_SIDE_NUM>
        joint_addresses_{};
    std::array<ChannelAddress, BC_SIDE_NUM> wheel_addresses_{};
    int imu_attitude_address_{};
    int imu_gyro_address_{};
    int actuator_count_{};
};

} // namespace balance::sim

#endif
