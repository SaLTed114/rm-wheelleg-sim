cmake_minimum_required(VERSION 3.22)

project(rm_balance_sim LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(MuJoCo REQUIRED)

option(BALANCE_BUILD_GUI "Build the Dear ImGui diagnostic simulator" ON)
option(BALANCE_BUILD_CPP_VIEWER
  "Build the UI-free C++ startup viewer" ${BALANCE_BUILD_GUI})
if(BALANCE_BUILD_GUI OR BALANCE_BUILD_CPP_VIEWER)
  find_package(glfw3 3.3 CONFIG QUIET)
  if(NOT TARGET glfw)
    include(FetchContent)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(glfw
      GIT_REPOSITORY https://github.com/glfw/glfw.git
      GIT_TAG 7b6aead9fb88b3623e3b3725ebb42670cbe4c579)
    FetchContent_MakeAvailable(glfw)
  endif()
endif()

function(balance_enable_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

function(balance_configure_mujoco_runtime target)
  if(WIN32 AND MUJOCO_RUNTIME)
    add_custom_command(TARGET ${target} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MUJOCO_RUNTIME}"
        "$<TARGET_FILE_DIR:${target}>"
      VERBATIM)
  elseif(MUJOCO_RUNTIME_DIR)
    set_property(TARGET ${target} PROPERTY BUILD_RPATH "${MUJOCO_RUNTIME_DIR}")
  endif()
endfunction()

add_library(balance_control_core STATIC
  src/control/control_law/lqr.c
  src/control/control_law/pd.c
  src/control/control_core.c
  src/control/controller.c
  src/control/controller_snapshot.c
  src/control/impact_observer.c
  src/control/reference/forward.c
  src/control/leg_kinematics.c
  src/control/observer.c
  src/control/support_force.c
  src/control/velocity_estimator.c
  src/control/reference/ramp.c
  src/control/reference/yaw.c
  src/control/state_machine/forward_mode.c
  src/control/state_machine/motion.c
  src/control/state_machine/step_task.c
  src/control/state_machine/support_phase.c
  src/control/state_machine/system.c)
set(BALANCE_LQR_SCHEDULE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/tools/lqr/generated"
  CACHE PATH "Directory containing current_model_schedule.h")
target_include_directories(balance_control_core
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
  PRIVATE "${BALANCE_LQR_SCHEDULE_DIR}")
if(NOT MSVC)
  target_link_libraries(balance_control_core PUBLIC m)
endif()
balance_enable_warnings(balance_control_core)

add_library(balance_control_cpp STATIC
  src/control_cpp/control_core.cpp
  src/control_cpp/control_law/lqr.cpp
  src/control_cpp/control_law/pd.cpp
  src/control_cpp/controller.cpp
  src/control_cpp/observer.cpp
  src/control_cpp/output_gate.cpp
  src/control_cpp/snapshot_assembler.cpp
  src/control_cpp/state_machine/condition_hold.cpp
  src/control_cpp/state_machine/motion.cpp
  src/control_cpp/state_machine/system.cpp
  src/control_cpp/velocity_estimator.cpp)
target_compile_features(balance_control_cpp PUBLIC cxx_std_20)
target_include_directories(balance_control_cpp
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
  PRIVATE "${BALANCE_LQR_SCHEDULE_DIR}")
balance_enable_warnings(balance_control_cpp)

add_library(balance_mujoco STATIC
  src/sim/mujoco_adapter.cpp
  src/sim/mujoco_plant.cpp)
target_include_directories(balance_mujoco
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sim")
target_link_libraries(balance_mujoco PUBLIC mujoco::mujoco)
balance_enable_warnings(balance_mujoco)

add_library(balance_simulation STATIC
  src/sim/input/interactive_scenario.cpp
  src/sim/input/virtual_gimbal.cpp
  src/sim/simulation_runner.cpp)
target_include_directories(balance_simulation
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sim")
target_link_libraries(balance_simulation
  PUBLIC balance_control_core balance_mujoco)
balance_enable_warnings(balance_simulation)

add_library(balance_cpp_simulation STATIC
  src/sim/cpp/simulation_runner.cpp)
target_include_directories(balance_cpp_simulation
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sim")
target_link_libraries(balance_cpp_simulation
  PUBLIC balance_control_cpp balance_mujoco)
balance_enable_warnings(balance_cpp_simulation)

if(BALANCE_BUILD_GUI OR BALANCE_BUILD_CPP_VIEWER)
  add_library(balance_mujoco_viewer STATIC
    src/sim/mujoco_viewer.cpp)
  target_include_directories(balance_mujoco_viewer
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/sim")
  target_link_libraries(balance_mujoco_viewer
    PUBLIC mujoco::mujoco glfw)
  balance_enable_warnings(balance_mujoco_viewer)
endif()

if(BALANCE_BUILD_CPP_VIEWER)
  add_executable(rm_balance_cpp_viewer
    src/sim/cpp/viewer_main.cpp)
  target_link_libraries(rm_balance_cpp_viewer
    PRIVATE balance_cpp_simulation balance_mujoco_viewer)
  balance_enable_warnings(rm_balance_cpp_viewer)
  balance_configure_mujoco_runtime(rm_balance_cpp_viewer)
endif()

add_library(balance_benchmark_support STATIC
  src/benchmark/common/common_diagnostics.cpp
  src/benchmark/common/csv_writer.cpp
  src/benchmark/common/simulation_sample.cpp
  src/benchmark/drop/drop_benchmark.cpp
  src/benchmark/drop/drop_policy.cpp
  src/benchmark/drop/drop_scenario.cpp
  src/benchmark/drop/jump_impulse.cpp
  src/benchmark/drop/platform_drop.cpp
  src/benchmark/drop/ramp_climb.cpp
  src/benchmark/drop/ramp_course.cpp
  src/benchmark/drop/ramp_jump.cpp
  src/benchmark/performance/base_roll_restraint.cpp
  src/benchmark/performance/forward_velocity_diagnostic.cpp
  src/benchmark/performance/performance_benchmark.cpp
  src/benchmark/performance/performance_scenario.cpp
  src/benchmark/step/step_dock.cpp
  src/benchmark/trim/trim_scanner.cpp)
target_include_directories(balance_benchmark_support
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/benchmark")
target_link_libraries(balance_benchmark_support PUBLIC balance_simulation)
balance_enable_warnings(balance_benchmark_support)

add_executable(rm_balance_performance
  src/benchmark/performance/main.cpp)
target_link_libraries(rm_balance_performance PRIVATE balance_benchmark_support)
balance_enable_warnings(rm_balance_performance)
balance_configure_mujoco_runtime(rm_balance_performance)

add_executable(rm_balance_drop
  src/benchmark/drop/main.cpp)
target_link_libraries(rm_balance_drop PRIVATE balance_benchmark_support)
balance_enable_warnings(rm_balance_drop)
balance_configure_mujoco_runtime(rm_balance_drop)

add_executable(rm_balance_ramp_course
  src/benchmark/drop/ramp_course_main.cpp)
target_link_libraries(
  rm_balance_ramp_course PRIVATE balance_benchmark_support)
balance_enable_warnings(rm_balance_ramp_course)
balance_configure_mujoco_runtime(rm_balance_ramp_course)

add_executable(rm_balance_ramp_jump
  src/benchmark/drop/ramp_jump_main.cpp)
target_link_libraries(
  rm_balance_ramp_jump PRIVATE balance_benchmark_support)
balance_enable_warnings(rm_balance_ramp_jump)
balance_configure_mujoco_runtime(rm_balance_ramp_jump)

add_executable(rm_balance_jump_impulse
  src/benchmark/drop/jump_impulse_main.cpp)
target_link_libraries(
  rm_balance_jump_impulse PRIVATE balance_benchmark_support)
balance_enable_warnings(rm_balance_jump_impulse)
balance_configure_mujoco_runtime(rm_balance_jump_impulse)

add_executable(rm_balance_step_dock
  src/benchmark/step/step_dock_main.cpp)
target_link_libraries(
  rm_balance_step_dock PRIVATE balance_benchmark_support)
balance_enable_warnings(rm_balance_step_dock)
balance_configure_mujoco_runtime(rm_balance_step_dock)

add_executable(rm_balance_trim_scan
  src/benchmark/trim/main.cpp)
target_link_libraries(rm_balance_trim_scan PRIVATE balance_benchmark_support)
balance_enable_warnings(rm_balance_trim_scan)
balance_configure_mujoco_runtime(rm_balance_trim_scan)

if(BALANCE_BUILD_GUI)
  set(IMGUI_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui"
    CACHE PATH "Dear ImGui source directory")
  if(NOT EXISTS "${IMGUI_ROOT}/imgui.cpp" OR
      NOT EXISTS "${IMGUI_ROOT}/backends/imgui_impl_glfw.cpp" OR
      NOT EXISTS "${IMGUI_ROOT}/backends/imgui_impl_opengl3.cpp")
    message(FATAL_ERROR
      "Dear ImGui sources were not found under IMGUI_ROOT=${IMGUI_ROOT}. "
      "Provide Dear ImGui v1.92.9b with -DIMGUI_ROOT=<path> or configure "
      "with -DBALANCE_BUILD_GUI=OFF.")
  endif()
  set(BALANCE_IMGUI_DIR "${IMGUI_ROOT}")
  set(BALANCE_IMGUI_SOURCES
    "${BALANCE_IMGUI_DIR}/imgui.cpp"
    "${BALANCE_IMGUI_DIR}/imgui_draw.cpp"
    "${BALANCE_IMGUI_DIR}/imgui_tables.cpp"
    "${BALANCE_IMGUI_DIR}/imgui_widgets.cpp"
    "${BALANCE_IMGUI_DIR}/backends/imgui_impl_glfw.cpp"
    "${BALANCE_IMGUI_DIR}/backends/imgui_impl_opengl3.cpp")

  add_executable(rm_balance_sim
    ${BALANCE_IMGUI_SOURCES}
    src/sim/gui/interactive_trace.cpp
    src/sim/gui/main.cpp
    src/sim/gui/simulation_app.cpp
    src/sim/gui/simulation_ui.cpp
    src/sim/input/interactive_keyboard.cpp)
  target_include_directories(rm_balance_sim SYSTEM PRIVATE
    "${BALANCE_IMGUI_DIR}"
    "${BALANCE_IMGUI_DIR}/backends")
  target_link_libraries(rm_balance_sim
    PRIVATE balance_benchmark_support balance_mujoco_viewer glfw)
  balance_enable_warnings(rm_balance_sim)
  balance_configure_mujoco_runtime(rm_balance_sim)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(control_core_test tests/control_core_test.c)
  target_link_libraries(control_core_test PRIVATE balance_control_core)
  balance_enable_warnings(control_core_test)
  add_test(NAME control_core_zero_output COMMAND control_core_test)

  add_executable(cpp_controller_test tests/cpp_controller_test.cpp)
  target_link_libraries(cpp_controller_test
    PRIVATE balance_control_cpp balance_control_core)
  balance_enable_warnings(cpp_controller_test)
  add_test(NAME cpp_controller_startup COMMAND cpp_controller_test)

  add_executable(cpp_module_test tests/cpp_module_test.cpp)
  target_link_libraries(cpp_module_test PRIVATE balance_control_cpp)
  balance_enable_warnings(cpp_module_test)
  add_test(NAME cpp_control_modules COMMAND cpp_module_test)

  add_executable(mujoco_cpp_startup_test
    tests/mujoco_cpp_startup_test.cpp)
  target_link_libraries(mujoco_cpp_startup_test
    PRIVATE balance_cpp_simulation)
  balance_enable_warnings(mujoco_cpp_startup_test)
  balance_configure_mujoco_runtime(mujoco_cpp_startup_test)
  add_test(NAME mujoco_cpp_startup
    COMMAND mujoco_cpp_startup_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml")

  add_executable(state_machine_test tests/state_machine_test.c)
  target_link_libraries(state_machine_test PRIVATE balance_control_core)
  balance_enable_warnings(state_machine_test)
  add_test(NAME layered_state_machine COMMAND state_machine_test)

  add_executable(forward_mode_test tests/forward_mode_test.c)
  target_link_libraries(forward_mode_test PRIVATE balance_control_core)
  balance_enable_warnings(forward_mode_test)
  add_test(NAME forward_mode COMMAND forward_mode_test)

  add_executable(controller_test tests/controller_test.c)
  target_link_libraries(controller_test PRIVATE balance_control_core)
  balance_enable_warnings(controller_test)
  add_test(NAME controller_facade COMMAND controller_test)

  add_executable(pd_controller_test tests/pd_controller_test.c)
  target_link_libraries(pd_controller_test PRIVATE balance_control_core)
  balance_enable_warnings(pd_controller_test)
  add_test(NAME pd_controller COMMAND pd_controller_test)

  add_executable(reference_ramp_test tests/reference_ramp_test.c)
  target_link_libraries(reference_ramp_test PRIVATE balance_control_core)
  balance_enable_warnings(reference_ramp_test)
  add_test(NAME reference_ramp COMMAND reference_ramp_test)

  add_executable(forward_reference_test tests/forward_reference_test.c)
  target_link_libraries(forward_reference_test PRIVATE balance_control_core)
  balance_enable_warnings(forward_reference_test)
  add_test(NAME forward_reference COMMAND forward_reference_test)

  add_executable(yaw_reference_test tests/yaw_reference_test.c)
  target_link_libraries(yaw_reference_test PRIVATE balance_control_core)
  balance_enable_warnings(yaw_reference_test)
  add_test(NAME yaw_reference COMMAND yaw_reference_test)

  add_executable(virtual_gimbal_test tests/virtual_gimbal_test.cpp)
  target_link_libraries(virtual_gimbal_test PRIVATE balance_simulation)
  balance_enable_warnings(virtual_gimbal_test)
  add_test(NAME virtual_gimbal COMMAND virtual_gimbal_test)

  add_executable(interactive_scenario_test
    tests/interactive_scenario_test.cpp)
  target_link_libraries(interactive_scenario_test PRIVATE balance_simulation)
  balance_enable_warnings(interactive_scenario_test)
  add_test(NAME interactive_scenario COMMAND interactive_scenario_test)

  add_executable(lqr_controller_test tests/lqr_controller_test.c)
  target_link_libraries(lqr_controller_test PRIVATE balance_control_core)
  target_include_directories(lqr_controller_test
    PRIVATE "${BALANCE_LQR_SCHEDULE_DIR}")
  if(BALANCE_LQR_SCHEDULE_DIR STREQUAL
      "${CMAKE_CURRENT_SOURCE_DIR}/tools/lqr/generated")
    target_compile_definitions(lqr_controller_test
      PRIVATE BALANCE_DEFAULT_LQR_SCHEDULE=1)
  endif()
  balance_enable_warnings(lqr_controller_test)
  add_test(NAME lqr_controller COMMAND lqr_controller_test)

  add_executable(leg_kinematics_test tests/leg_kinematics_test.c)
  target_link_libraries(leg_kinematics_test PRIVATE balance_control_core)
  balance_enable_warnings(leg_kinematics_test)
  add_test(NAME leg_kinematics COMMAND leg_kinematics_test)

  add_executable(impact_observer_test tests/impact_observer_test.c)
  target_link_libraries(impact_observer_test PRIVATE balance_control_core)
  balance_enable_warnings(impact_observer_test)
  add_test(NAME impact_observer COMMAND impact_observer_test)

  add_executable(support_force_test tests/support_force_test.c)
  target_link_libraries(support_force_test PRIVATE balance_control_core)
  balance_enable_warnings(support_force_test)
  add_test(NAME support_force COMMAND support_force_test)

  add_executable(support_phase_test tests/support_phase_test.c)
  target_link_libraries(support_phase_test PRIVATE balance_control_core)
  balance_enable_warnings(support_phase_test)
  add_test(NAME support_phase COMMAND support_phase_test)

  add_executable(step_task_test tests/step_task_test.c)
  target_link_libraries(step_task_test PRIVATE balance_control_core)
  balance_enable_warnings(step_task_test)
  add_test(NAME step_task COMMAND step_task_test)

  add_executable(observer_test tests/observer_test.c)
  target_link_libraries(observer_test PRIVATE balance_control_core)
  balance_enable_warnings(observer_test)
  add_test(NAME observer_state_assembly COMMAND observer_test)

  add_executable(velocity_estimator_test
    tests/velocity_estimator_test.c)
  target_link_libraries(
    velocity_estimator_test PRIVATE balance_control_core)
  balance_enable_warnings(velocity_estimator_test)
  add_test(NAME velocity_estimator COMMAND velocity_estimator_test)

  add_executable(performance_scenario_test
    tests/performance_scenario_test.cpp)
  target_link_libraries(
    performance_scenario_test PRIVATE balance_benchmark_support)
  balance_enable_warnings(performance_scenario_test)
  balance_configure_mujoco_runtime(performance_scenario_test)
  add_test(NAME performance_scenario COMMAND performance_scenario_test)

  add_executable(drop_policy_test tests/drop_policy_test.cpp)
  target_link_libraries(drop_policy_test PRIVATE balance_benchmark_support)
  balance_enable_warnings(drop_policy_test)
  add_test(NAME drop_policy COMMAND drop_policy_test)

  add_executable(drop_scenario_test tests/drop_scenario_test.cpp)
  target_link_libraries(drop_scenario_test PRIVATE balance_benchmark_support)
  balance_enable_warnings(drop_scenario_test)
  balance_configure_mujoco_runtime(drop_scenario_test)
  add_test(NAME drop_scenario
    COMMAND drop_scenario_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml")

  add_executable(mujoco_drop_test tests/mujoco_drop_test.cpp)
  target_link_libraries(mujoco_drop_test PRIVATE balance_benchmark_support)
  balance_enable_warnings(mujoco_drop_test)
  balance_configure_mujoco_runtime(mujoco_drop_test)
  add_test(NAME mujoco_drop
    COMMAND mujoco_drop_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml"
      "${CMAKE_CURRENT_BINARY_DIR}/test-results/drop")

  add_executable(mujoco_platform_drop_test
    tests/mujoco_platform_drop_test.cpp)
  target_link_libraries(
    mujoco_platform_drop_test PRIVATE balance_benchmark_support)
  balance_enable_warnings(mujoco_platform_drop_test)
  balance_configure_mujoco_runtime(mujoco_platform_drop_test)
  add_test(NAME mujoco_platform_drop
    COMMAND mujoco_platform_drop_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml"
      "${CMAKE_CURRENT_BINARY_DIR}/test-results/platform-drop")

  add_executable(mujoco_active_landing_test
    tests/mujoco_active_landing_test.cpp)
  target_link_libraries(
    mujoco_active_landing_test PRIVATE balance_benchmark_support)
  balance_enable_warnings(mujoco_active_landing_test)
  balance_configure_mujoco_runtime(mujoco_active_landing_test)
  add_test(NAME mujoco_active_landing
    COMMAND mujoco_active_landing_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml"
      "${CMAKE_CURRENT_BINARY_DIR}/test-results/active-landing")

  add_executable(mujoco_ramp_climb_test
    tests/mujoco_ramp_climb_test.cpp)
  target_link_libraries(
    mujoco_ramp_climb_test PRIVATE balance_benchmark_support)
  balance_enable_warnings(mujoco_ramp_climb_test)
  balance_configure_mujoco_runtime(mujoco_ramp_climb_test)
  add_test(NAME mujoco_ramp_climb_recovery
    COMMAND mujoco_ramp_climb_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml"
      "${CMAKE_CURRENT_BINARY_DIR}/test-results/ramp-climb")

  add_executable(mujoco_ramp_course_test
    tests/mujoco_ramp_course_test.cpp)
  target_link_libraries(
    mujoco_ramp_course_test PRIVATE balance_benchmark_support)
  balance_enable_warnings(mujoco_ramp_course_test)
  balance_configure_mujoco_runtime(mujoco_ramp_course_test)
  add_test(NAME mujoco_ramp_course_matrix
    COMMAND mujoco_ramp_course_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml"
      "${CMAKE_CURRENT_BINARY_DIR}/test-results/ramp-course")

  add_executable(mujoco_ramp_jump_test
    tests/mujoco_ramp_jump_test.cpp)
  target_link_libraries(
    mujoco_ramp_jump_test PRIVATE balance_benchmark_support)
  balance_enable_warnings(mujoco_ramp_jump_test)
  balance_configure_mujoco_runtime(mujoco_ramp_jump_test)
  add_test(NAME mujoco_ramp_jump_sweep
    COMMAND mujoco_ramp_jump_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml"
      "${CMAKE_CURRENT_BINARY_DIR}/test-results/ramp-jump")

  add_executable(mujoco_jump_impulse_test
    tests/mujoco_jump_impulse_test.cpp)
  target_link_libraries(
    mujoco_jump_impulse_test PRIVATE balance_benchmark_support)
  balance_enable_warnings(mujoco_jump_impulse_test)
  balance_configure_mujoco_runtime(mujoco_jump_impulse_test)
  add_test(NAME mujoco_jump_impulse_sweep
    COMMAND mujoco_jump_impulse_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml"
      "${CMAKE_CURRENT_BINARY_DIR}/test-results/jump-impulse")

  add_executable(mujoco_step_dock_test
    tests/mujoco_step_dock_test.cpp)
  target_link_libraries(
    mujoco_step_dock_test PRIVATE balance_benchmark_support)
  balance_enable_warnings(mujoco_step_dock_test)
  balance_configure_mujoco_runtime(mujoco_step_dock_test)
  add_test(NAME mujoco_step_dock_complete
    COMMAND mujoco_step_dock_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml"
      "${CMAKE_CURRENT_BINARY_DIR}/test-results/step-dock")

  add_executable(mujoco_impact_observer_matrix_test
    tests/mujoco_impact_observer_matrix_test.cpp)
  target_link_libraries(
    mujoco_impact_observer_matrix_test PRIVATE balance_benchmark_support)
  balance_enable_warnings(mujoco_impact_observer_matrix_test)
  balance_configure_mujoco_runtime(mujoco_impact_observer_matrix_test)
  add_test(NAME mujoco_impact_observer_negative_matrix
    COMMAND mujoco_impact_observer_matrix_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml"
      "${CMAKE_CURRENT_BINARY_DIR}/test-results/impact-observer-negative")

  add_executable(mujoco_smoke_test tests/mujoco_smoke_test.cpp)
  target_link_libraries(mujoco_smoke_test PRIVATE balance_benchmark_support)
  balance_enable_warnings(mujoco_smoke_test)
  balance_configure_mujoco_runtime(mujoco_smoke_test)
  add_test(NAME mujoco_model_smoke
    COMMAND mujoco_smoke_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml")

  add_executable(mujoco_leg_test tests/mujoco_leg_test.cpp)
  target_link_libraries(mujoco_leg_test PRIVATE balance_simulation)
  balance_enable_warnings(mujoco_leg_test)
  balance_configure_mujoco_runtime(mujoco_leg_test)
  add_test(NAME mujoco_leg_kinematics_and_control
    COMMAND mujoco_leg_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml")

  add_executable(mujoco_static_stand_test
    tests/mujoco_static_stand_test.cpp)
  target_link_libraries(mujoco_static_stand_test PRIVATE balance_simulation)
  balance_enable_warnings(mujoco_static_stand_test)
  balance_configure_mujoco_runtime(mujoco_static_stand_test)
  add_test(NAME mujoco_static_stand
    COMMAND mujoco_static_stand_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml")
endif()
