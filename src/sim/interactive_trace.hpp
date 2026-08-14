#ifndef BALANCE_SIM_INTERACTIVE_TRACE_HPP
#define BALANCE_SIM_INTERACTIVE_TRACE_HPP

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string_view>

#include "common/csv_writer.hpp"
#include "common/simulation_sample.hpp"
#include "input/interactive_scenario.hpp"

namespace balance::sim {

class InteractiveTraceWriter {
public:
    explicit InteractiveTraceWriter(const std::filesystem::path &path);

    void write(
        std::uint32_t reset_index,
        std::string_view phase,
        const KeyboardDriveInput &keyboard,
        const InteractiveScenarioFrame &frame,
        const benchmark::SimulationSample &sample,
        const benchmark::ImuMotionState &velocity_truth,
        const bc_sensor_feedback_t &feedback);
    void flush();

private:
    benchmark::CsvWriter csv_;
    std::size_t row_count_{};
    bc_system_state_t previous_system_{BC_SYSTEM_OFF};
    bc_motion_state_t previous_motion_{BC_MOTION_IDLE};
    bc_forward_state_t previous_forward_{BC_FORWARD_IDLE};
    bc_step_task_state_t previous_step_task_{BC_STEP_TASK_INACTIVE};
    bc_support_phase_state_t previous_support_{BC_SUPPORT_GROUND};
    bool state_initialized_{};
};

} // namespace balance::sim

#endif
