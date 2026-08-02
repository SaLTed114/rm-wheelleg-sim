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

option(BALANCE_BUILD_GUI "Build the interactive MuJoCo viewer" ON)
if(BALANCE_BUILD_GUI)
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
  src/control/leg_kinematics.c
  src/control/observer.c
  src/control/state_machine/motion.c
  src/control/state_machine/system.c)
target_include_directories(balance_control_core
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tools/lqr/generated")
if(NOT MSVC)
  target_link_libraries(balance_control_core PUBLIC m)
endif()
balance_enable_warnings(balance_control_core)

add_library(balance_simulation STATIC
  src/sim/mujoco_adapter.cpp
  src/sim/mujoco_plant.cpp
  src/sim/simulation_runner.cpp)
target_include_directories(balance_simulation
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sim")
target_link_libraries(balance_simulation
  PUBLIC balance_control_core mujoco::mujoco)
balance_enable_warnings(balance_simulation)

if(BALANCE_BUILD_GUI)
  add_executable(rm_balance_sim
    src/sim/main.cpp
    src/sim/mujoco_viewer.cpp)
  target_link_libraries(rm_balance_sim PRIVATE balance_simulation glfw)
  balance_enable_warnings(rm_balance_sim)
  balance_configure_mujoco_runtime(rm_balance_sim)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(control_core_test tests/control_core_test.c)
  target_link_libraries(control_core_test PRIVATE balance_control_core)
  balance_enable_warnings(control_core_test)
  add_test(NAME control_core_zero_output COMMAND control_core_test)

  add_executable(state_machine_test tests/state_machine_test.c)
  target_link_libraries(state_machine_test PRIVATE balance_control_core)
  balance_enable_warnings(state_machine_test)
  add_test(NAME layered_state_machine COMMAND state_machine_test)

  add_executable(controller_test tests/controller_test.c)
  target_link_libraries(controller_test PRIVATE balance_control_core)
  balance_enable_warnings(controller_test)
  add_test(NAME controller_facade COMMAND controller_test)

  add_executable(pd_controller_test tests/pd_controller_test.c)
  target_link_libraries(pd_controller_test PRIVATE balance_control_core)
  balance_enable_warnings(pd_controller_test)
  add_test(NAME pd_controller COMMAND pd_controller_test)

  add_executable(lqr_controller_test tests/lqr_controller_test.c)
  target_link_libraries(lqr_controller_test PRIVATE balance_control_core)
  balance_enable_warnings(lqr_controller_test)
  add_test(NAME lqr_controller COMMAND lqr_controller_test)

  add_executable(leg_kinematics_test tests/leg_kinematics_test.c)
  target_link_libraries(leg_kinematics_test PRIVATE balance_control_core)
  balance_enable_warnings(leg_kinematics_test)
  add_test(NAME leg_kinematics COMMAND leg_kinematics_test)

  add_executable(observer_test tests/observer_test.c)
  target_link_libraries(observer_test PRIVATE balance_control_core)
  balance_enable_warnings(observer_test)
  add_test(NAME observer_state_assembly COMMAND observer_test)

  add_executable(mujoco_smoke_test tests/mujoco_smoke_test.cpp)
  target_link_libraries(mujoco_smoke_test PRIVATE balance_simulation)
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
